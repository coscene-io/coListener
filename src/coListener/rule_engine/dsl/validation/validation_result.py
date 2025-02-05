from ..utils.enum import Enum

ValidationErrorType = Enum(
    "ValidationErrorType",
    ["SYNTAX", "EMPTY", "NOT_CONDITION", "NOT_ACTION", "UNDEFINED", "UNKNOWN"],
)

class ValidationResult:
    def __init__(self, success, error_type=None, details=None, entity=None):
        self.success = success
        # If not success, fill in error type and details
        self.error_type = error_type
        self.details = details if details is not None else {}

        # If success, fill in validated entity
        self.entity = entity

    def __post_init__(self):
        if self.success:
            assert self.entity is not None
        else:
            assert self.error_type
