import re
from .condition import Condition, ThunkCondition
from .utils.string import format_string

always = Condition.wrap(True)
msg = ThunkCondition(lambda item, scope: (item.msg, scope))
ts = ThunkCondition(lambda item, scope: (item.ts, scope))
topic = ThunkCondition(lambda item, scope: (item.topic, scope))
msgtype = ThunkCondition(lambda item, scope: (item.msgtype, scope))


def get_start_time(item, scope):
    value = item.ts
    if "start_time" in scope:
        value = scope["start_time"]
    return value, scope


condition_start_time = ThunkCondition(get_start_time)


@Condition.wrap_args
def and_(*conditions):
    assert len(conditions) > 0, "and_ must have at least 1 condition"

    def new_thunk(item, scope):
        value = True
        for condition in conditions:
            value, scope = condition.evaluate_condition_at(item, scope)
            if not value:
                break
        return value, scope

    return ThunkCondition(new_thunk)


@Condition.wrap_args
def or_(*conditions):
    assert len(conditions) > 0, "or_ must have at least 1 condition"

    def new_thunk(item, scope):
        value = False
        for condition in conditions:
            value, scope = condition.evaluate_condition_at(item, scope)
            if value:
                break
        return value, scope

    return ThunkCondition(new_thunk)


@Condition.wrap_args
def not_(condition):
    return Condition.map(condition, lambda x: not x)


@Condition.wrap_args
def is_none(condition):
    def new_thunk(item, scope):
        value, scope = condition.evaluate_condition_at(item, scope)
        return value is None, scope

    return ThunkCondition(new_thunk)


@Condition.wrap_args
def concat(*pieces):
    def new_thunk(item, scope):
        str_pieces = []
        for p in pieces:
            value, scope = p.evaluate_condition_at(item, scope)
            str_pieces.append(str(value))
        return "".join(str_pieces), scope

    return ThunkCondition(new_thunk)


@Condition.wrap_args
def get_value(key):
    return Condition.flatmap(
        key,
        lambda k: ThunkCondition(
            lambda item, scope: (scope[k], scope) if k in scope else (None, scope)
        ),
    )


@Condition.wrap_args
def set_value(key, value):
    return Condition.apply(
        lambda scope, actual_key, actual_value: (
            True,
            dict(scope, **{actual_key: actual_value}),
        ),
        key,
        value,
    )


@Condition.wrap_args
def has(parent, child):
    return Condition.apply(
        lambda scope, p, c: (
            c in p,
            dict(scope, **{"cos/contains": c if c in p else None})),
        parent,
        child,
    )


def regex(value, pattern):
    return Condition.flatmap(
        Condition.map(Condition.wrap(value), lambda v: re.search(pattern, v)),
        lambda match_result: ThunkCondition(
            lambda item, scope: (
                match_result,
                dict(scope, **{"cos/regex": match_result}))
        ),
    )


def func_apply(func, *args):
    return Condition.apply(
        lambda scope, f, *a: (f(*a), scope),
        Condition.wrap(func),
        *[Condition.wrap(arg) for arg in args]
    )

def template_str(template, *args):
    def new_thunk(item, scope):
        values = []
        for arg in args:
            value, _ = Condition.wrap(arg).evaluate_condition_at(item, scope)
            values.append(value)
        return format_string(template.format(*values)), scope
    return ThunkCondition(new_thunk)


__all__ = [
    "always",
    "msg",
    "ts",
    "topic",
    "msgtype",
    "condition_start_time",
    "and_",
    "or_",
    "not_",
    "concat",
    "get_value",
    "set_value",
    "has",
    "regex",
    "is_none",
    "func_apply",
    "template_str"
]
