import ast
import astor
import unittest

from ...rule_engine.dsl.validation.normalizer import normalize_expression_tree


class NormalizerTest(unittest.TestCase):
    def test_transform_boolean_ops(self):
        self._assert_equivalent("a and b and c", "and_(a, b, c)")

        self._assert_equivalent("a or b or c", "or_(a, b, c)")

        self._assert_equivalent("a or b and c", "or_(a, and_(b, c))")

        self._assert_equivalent("not a", "not_(a)")

        self._assert_equivalent(
            "not not a or b and c", "or_(not_(not_(a)), and_(b, c))"
        )

        self._assert_equivalent(
            "a in b", "(lambda arg0, arg1: and_(has(arg1, arg0)))(a, b)"
        )

        self._assert_equivalent(
            "a > 1 in b",
            "(lambda arg0, arg1, arg2: and_(arg0 > arg1, has(arg2, arg1)))(a, 1, b)",
        )

        self._assert_equivalent(
            "c or a > 1 in b and d",
            "or_(c, and_((lambda arg0, arg1, arg2: and_(arg0 > arg1, has(arg2, arg1)))(a, 1, b), d))",
        )

    def test_transform_f_strings(self):
        self._assert_equivalent(
            "f'hello {aa} world {bb} !!'",
            "func_apply(lambda arg0, arg1: f'hello {arg0} world {arg1} !!', aa, bb)",
        )

        self._assert_equivalent(
            "f'hello {1 + 1} world {s.upper()} !!'",
            "func_apply(lambda arg0, arg1: f'hello {arg0} world {arg1} !!', 1 + 1, s.upper())",
        )

        self._assert_equivalent(
            "f'hello {aa!s} world {bb!r} !!'",
            "func_apply(lambda arg0, arg1: f'hello {arg0!s} world {arg1!r} !!', aa, bb)",
        )

        self._assert_equivalent(
            "f'hello {aa:100} world {bb:5.2f} !!'",
            "func_apply(lambda arg0, arg1: f'hello {arg0:100} world {arg1:5.2f} !!', aa, bb)",
        )

        with self.assertRaises(Exception):
            normalize_expression_tree(ast.parse("f'hello {aa:{bb}}'"))

    def _assert_equivalent(self, original, normalized):
        left = normalize_expression_tree(ast.parse(original, mode="eval"))
        right = ast.parse(normalized, mode="eval")

        right_code = astor.to_source(right)
        left_code = astor.to_source(left)
        self.assertEqual(right_code, left_code)
