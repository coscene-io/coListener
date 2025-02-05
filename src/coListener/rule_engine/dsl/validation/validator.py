import inspect
from .validation_result import ValidationResult, ValidationErrorType
from .validate_expression import validate_expression
from .actions import ActionValidator, UnknownFunctionKeywordArgException
from .. import base_conditions, log_conditions, sequence_conditions
from ..condition import Condition
from ..action import Action
from ....logger.logger import logger

base_dsl_values = dict(
    inspect.getmembers(base_conditions)
    + inspect.getmembers(log_conditions)
    + inspect.getmembers(sequence_conditions)
)


def validate_condition(cond_str):
    return _do_validate(
        cond_str, base_dsl_values, Condition, ValidationErrorType.NOT_CONDITION
    )


def validate_action(action_str, action_impls):
    action_validator = ActionValidator(action_impls)

    action_dsl_values = {
        "upload": action_validator.upload,
        "create_moment": action_validator.create_moment,
    }

    action_dsl_values.update(base_dsl_values)
    return _do_validate(
        action_str,
        action_dsl_values,
        Action,
        ValidationErrorType.NOT_ACTION,
    )

def _do_validate(expr_str, injected_values, expected_class, class_expectation_error):
    if not expr_str.strip():
        logger.debug("ValidationErrorType.EMPTY")
        return ValidationResult(False, ValidationErrorType.EMPTY)

    try:
        logger.debug(u"validate_expression: {}".format(expr_str))
        res = validate_expression(expr_str, injected_values)
        if not res.success:
            logger.debug("success: {}, error_type: {}, details:{}".format(res.success, res.error_type, res.details))
            return res
    except UnknownFunctionKeywordArgException as e:
        return ValidationResult(False, ValidationErrorType.UNDEFINED, {"name": e.name})
    except Exception as e:
        return ValidationResult(False, ValidationErrorType.UNKNOWN, {"message": str(e)})

    if not isinstance(res.entity, expected_class):
        return ValidationResult(
            False, class_expectation_error, {"actual": type(res.entity).__name__}
        )

    return res
