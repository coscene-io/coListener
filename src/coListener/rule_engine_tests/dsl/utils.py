import ast
from ...rule_engine.dsl.validation.normalizer import normalize_expression_tree
from ...rule_engine.dsl.validation.validator import base_dsl_values


def str_to_condition(expr_str, additional_injected_values=None):
    parsed = ast.parse(expr_str, mode="eval")
    code = compile(normalize_expression_tree(parsed), "", mode="eval")

    eval_globals = base_dsl_values.copy()
    if additional_injected_values:
        eval_globals.update(additional_injected_values)

    return eval(code, eval_globals)