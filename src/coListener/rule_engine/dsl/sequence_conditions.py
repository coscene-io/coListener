from .base_conditions import and_
from .condition import Condition


def sustained(context_condition, variable_condition, duration):
    """
    This condition triggers when the variable condition continues to be true for
    the given duration.

    The context condition is used to limit the scope of the variable condition.
    For example, we might say "if topic X has value Y for 10 seconds", which
    translates to context being topic == X, and variable being value == Y
    """
    return and_(
        context_condition,
        RisingEdgeCondition(SustainedCondition(variable_condition, duration)),
    )


def sequential(*condition_factories, **kwargs):
    return SequenceMatchCondition(list(condition_factories), kwargs.pop('duration', None))


def timeout(*condition_factories, **kwargs):
    return SequenceMatchCondition(list(condition_factories), kwargs.pop('duration', None), True)


def any_order(*conditions):
    return RisingEdgeCondition(AnyOrderCondition(list(conditions)))


def repeated(condition_factory, times, duration):
    assert times > 1, "In repeated condition, times must be more than 1"
    return and_(
        condition_factory(),
        RisingEdgeCondition(RepeatedCondition(condition_factory(), times, duration)),
    )


def debounce(condition_factory, duration):
    return and_(condition_factory(), RisingEdgeCondition(condition_factory(), duration))


def throttle(condition, duration):
    return ThrottleCondition(condition, duration)


class RisingEdgeCondition(Condition):
    """
    A condition that detects when child condition changes from false to true.

    Once child condition becomes true, this condition will not fire unless the
    child condition first becomes false, then back to true.

    If max_gap is given, and two invocations of this condition have timestamps
    further apart than the gap, the state is reset. It behaves as if a false is
    inserted in the gap.
    """

    def __init__(self, condition, max_gap=None):
        assert isinstance(condition, Condition)
        self.__condition = condition
        self.__active = False
        self.__last_activation = None
        self.__max_gap = max_gap

    def evaluate_condition_at(self, item, scope):
        value, new_scope = self.__condition.evaluate_condition_at(item, scope)
        if not value:
            self.__active = False
            return False, new_scope

        if self.__active and (
            self.__max_gap is None or item.ts - self.__last_activation < self.__max_gap
        ):
            self.__last_activation = item.ts
            return False, new_scope

        self.__active = True
        self.__last_activation = item.ts
        return True, new_scope


class SustainedCondition(Condition):
    """
    This condition triggers when the child condition is true for the given
    duration.
    The state is reset once the child condition becomes false.
    """

    def __init__(self, condition, duration=-1):
        super(SustainedCondition, self).__init__()

        assert isinstance(condition, Condition)
        self.__condition = condition
        self.__duration = duration
        self.__start = None

    def evaluate_condition_at(self, item, scope):
        value, new_scope = self.__condition.evaluate_condition_at(item, scope)
        if not value:
            self.__start = None
            return False, new_scope

        if self.__start is None:
            self.__start = item.ts

        if item.ts - self.__start > self.__duration:
            updated_scope = dict(new_scope, start_time=self.__start)
            return True, updated_scope

        return False, new_scope


class SequenceMatchCondition(Condition):
    """
    This condition triggers when the child conditions are true in the given
    order, and within the given duration if set.

    If trigger_on_timeout is set, the condition will only trigger when the
    sequence is not matched within the given duration.

    Note that the sequence input is a list of condition factories, not conditions.

    Also note that we do not support overlapping sequences.
    """

    def __init__(self, factory_sequence, duration=None, trigger_on_timeout=False):
        super(SequenceMatchCondition, self).__init__()

        assert len(factory_sequence) > 1, "Sequence must be longer than 1"
        for factory in factory_sequence:
            assert isinstance(factory(), Condition)

        self.__factory_seq = factory_sequence
        self.__duration = duration
        self.__trigger_on_timeout = trigger_on_timeout
        self.__current_scope = None
        self.__start_time = None
        self._reset()

    def _reset(self):
        self.__start_time = None
        self.__current_index = 0
        self.__current_scope = None
        self.__seq = [factory() for factory in self.__factory_seq]

    def evaluate_condition_at(self, item, scope):
        if (
            self.__duration is not None
            and self.__start_time is not None
            and item.ts - self.__start_time > self.__duration
        ):
            ret = (bool(self.__trigger_on_timeout),
                   dict(self.__current_scope, start_time=self.__start_time))
            self._reset()
            return ret

        matched, new_scope = self.__seq[self.__current_index].evaluate_condition_at(
            item, self.__current_scope or scope
        )
        if matched:
            self.__current_index += 1
            if self.__current_index == len(self.__seq):
                ret = (not bool(self.__trigger_on_timeout),
                       dict(self.__current_scope, start_time=self.__start_time))
                self._reset()
                return ret

            self.__current_scope = new_scope
            if self.__start_time is None:
                self.__start_time = item.ts

        return False, scope


class RepeatedCondition(Condition):
    """
    This condition triggers when the child condition is true for the given
    number of times within the given duration.
    """

    def __init__(self, condition, times, duration):
        assert isinstance(condition, Condition)
        self.__condition = condition
        self.__times = times
        self.__duration = duration
        self.__trigger_times = []

    def evaluate_condition_at(self, item, scope):
        value, new_scope = self.__condition.evaluate_condition_at(item, scope)

        if not value:
            return False, scope

        self.__trigger_times.append(item.ts)
        self.__trigger_times = [
            t for t in self.__trigger_times if item.ts - t <= self.__duration
        ]

        if len(self.__trigger_times) >= self.__times:
            updated_scope = dict(new_scope, start_time=self.__trigger_times[0])
            return True, updated_scope

        return False, scope


class ThrottleCondition(Condition):
    """
    This condition triggers when the child condition is true, but only if the
    last trigger is more than the given duration ago.
    """

    def __init__(self, condition, duration):
        assert isinstance(condition, Condition)
        self.__condition = condition
        self.__duration = duration
        self.__last_trigger = -duration

    def evaluate_condition_at(self, item, scope):
        if item.ts - self.__last_trigger < self.__duration:
            return False, scope

        value, scope = self.__condition.evaluate_condition_at(item, scope)
        if value:
            self.__last_trigger = item.ts
            return value, scope

        return False, scope


class AnyOrderCondition(Condition):
    """
    This condition triggers when all the child conditions are true, but the
    order of the conditions does not matter.
    The condition will remain true once triggered.
    """

    def __init__(self, conditions):
        self.__unsatisfied = conditions
        self.__curr_scope = None

    def evaluate_condition_at(self, item, scope):
        if self.__curr_scope is None:
            self.__curr_scope = scope

        new_conditions = []
        for c in self.__unsatisfied:
            value, new_scope = c.evaluate_condition_at(item, self.__curr_scope)
            if value:
                self.__curr_scope = new_scope
            else:
                new_conditions.append(c)

        if not new_conditions:
            return True, self.__curr_scope

        self.__unsatisfied = new_conditions
        return False, self.__curr_scope


__all__ = [
    "any_order",
    "debounce",
    "repeated",
    "sequential",
    "sustained",
    "throttle",
    "timeout",
]
