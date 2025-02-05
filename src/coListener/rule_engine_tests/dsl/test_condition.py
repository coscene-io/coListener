import unittest

from ...rule_engine.dsl.condition import Condition


class ConditionTest(unittest.TestCase):
    def test_arithmetics(self):
        self.assertEqual(-4, int(self._get_eval_result(-Condition.wrap(4))))
        self.assertEqual(4, int(self._get_eval_result(+Condition.wrap(4))))

        self.assertEqual(10, int(self._get_eval_result(Condition.wrap(4) + 6)))
        self.assertEqual(10, int(self._get_eval_result(6 + Condition.wrap(4))))

        self.assertEqual(-2, int(self._get_eval_result(Condition.wrap(4) - 6)))
        self.assertEqual(2, int(self._get_eval_result(6 - Condition.wrap(4))))

        self.assertEqual(24, int(self._get_eval_result(Condition.wrap(4) * 6)))
        self.assertEqual(24, int(self._get_eval_result(6 * Condition.wrap(4))))

        self.assertEqual(2, int(self._get_eval_result(Condition.wrap(12) / 6)))
        self.assertEqual(2, int(self._get_eval_result(12 / Condition.wrap(6))))

    def _get_eval_result(self, cond):
        return cond.evaluate_condition_at(None, {})[0]
