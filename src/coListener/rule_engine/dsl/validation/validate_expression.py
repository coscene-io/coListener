import ast
# from ..ast import ast

from .validation_result import ValidationResult, ValidationErrorType
from .normalizer import normalize_expression_tree


def validate_expression(expr_str, injected_values):
    try:
        parsed = ast.parse(expr_str, filename="<string>", mode="eval")
    except SyntaxError as e:
        return ValidationResult(False, ValidationErrorType.SYNTAX)

    for node in ast.walk(parsed):
        if isinstance(node, ast.Name):
            name = node.id
            if name == 'True' or name == 'False':
                continue
            if name not in injected_values:
                return ValidationResult(
                    False, ValidationErrorType.UNDEFINED, {"name": name}
                )

    tree = normalize_expression_tree(parsed)
    code = compile(tree, "", mode="eval")

    return ValidationResult(True, entity=eval(code, injected_values))
