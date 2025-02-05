import inspect
from ..base_actions import upload_factory, create_moment_factory

class UnknownFunctionKeywordArgException(Exception):
    def __init__(self, name):
        if not isinstance(name, str):
            raise TypeError("name must be a string")
        self.name = name

    def __repr__(self):
        return "UnknownFunctionKeywordArgException(name={})".format(self.name)

    def __eq__(self, other):
        if isinstance(other, UnknownFunctionKeywordArgException):
            return self.name == other.name
        return False


class ActionValidator:
    def __init__(self, action_impls):
        self.__impls = action_impls

    def upload(self, *args, **kwargs):
        def validate_arg_types(args):
            def check_is_list_of_string(name):
                value = args.get(name, [])
                if not isinstance(value, list) or any(
                    not isinstance(i, str) for i in value
                ):
                    # TODO: Make this an actual error instead of a generic error
                    raise Exception("{} must be list of strings".format(name))

            check_is_list_of_string("labels")
            check_is_list_of_string("extra_files")

            if not isinstance(args.get("before", 0), int):
                # TODO: Make this an actual error instead of a generic error
                raise Exception("before must be an int")

            if not isinstance(args.get("after", 0), int):
                # TODO: Make this an actual error instead of a generic error
                raise Exception("after must be an int")

        return self._validate_factory_func(
            upload_factory(self.__impls["upload"]),
            args,
            kwargs,
            validate_arg_types,
        )

    def create_moment(self, *args, **kwargs):
        def validate_arg_types(args):
            assign_to = args.get("assign_to", "")
            if not isinstance(assign_to, str):
                # TODO: Make this an actual error instead of a generic error
                raise Exception("assign_to must be string")

        return self._validate_factory_func(
            create_moment_factory(self.__impls["create_moment"]),
            args,
            kwargs,
            validate_arg_types,
        )

    def _validate_factory_func(self, factory, args, kwargs, param_type_check):
        if not callable(factory):
            raise Exception("factory must be callable")

        sig = inspect.getargspec(factory)
        self._validate_signature(sig, args, kwargs)
        param_type_check(dict(zip(sig.args, args)))

        return factory(*args, **kwargs)

    def _validate_signature(self, sig, args, kwargs):
        if len(args) > len(sig.args):
            raise Exception("too many arguments")

        for key in kwargs:
            if key not in sig.args:
                raise UnknownFunctionKeywordArgException(key)

        for i, name in enumerate(sig.args):
            if i < len(args):
                if name in kwargs:
                    raise Exception("multiple values given for `{}`".format(name))
            else:
                defaults_index = i - (len(sig.args) - len(sig.defaults or []))
                if defaults_index < 0 and name not in kwargs:
                    raise Exception("value not specified for parameter `{}`".format(name))
