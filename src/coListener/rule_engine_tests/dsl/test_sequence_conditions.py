import unittest
from collections import namedtuple

from ...rule_engine.dsl.action import Action
from ...rule_engine.engine import Engine, Rule, DiagnosisItem
from .utils import str_to_condition

MockMessage = namedtuple("MockMessage", "int_value str_value")

simple_sequence = [
    DiagnosisItem("t1", MockMessage(1, "hello"), 0, "MockMessage"),
    DiagnosisItem("t1", MockMessage(2, "hello"), 0, "MockMessage"),
    DiagnosisItem("t2", MockMessage(1, "hello"), 1, "MockMessage"),
    DiagnosisItem("t2", MockMessage(2, "hello"), 1, "MockMessage"),
    DiagnosisItem("t1", MockMessage(3, "hello"), 2, "MockMessage"),
    DiagnosisItem("t1", MockMessage(3, "hello"), 3, "MockMessage"),
    DiagnosisItem("t2", MockMessage(4, "hello"), 3, "MockMessage"),
    DiagnosisItem("t2", MockMessage(4, "hello"), 3, "MockMessage"),
    DiagnosisItem("t2", MockMessage(5, "world"), 4, "MockMessage"),
    DiagnosisItem("t2", MockMessage(5, "world"), 4, "MockMessage"),
    DiagnosisItem("t2", MockMessage(4, "hello"), 5, "MockMessage"),
    DiagnosisItem("t2", MockMessage(4, "hello"), 6, "MockMessage"),
    DiagnosisItem("t2", MockMessage(4, "hello"), 7, "MockMessage"),
    DiagnosisItem("t2", MockMessage(4, "hello"), 7, "MockMessage"),
    DiagnosisItem("t2", MockMessage(4, "hello"), 9, "MockMessage"),
    DiagnosisItem("t3", MockMessage(4, "single"), 9, "MockMessage"),
]


class CollectAction(Action):
    def __init__(self):
        self.collector = []

    def run(self, item, scope):
        self.collector.append((item, scope))


def get_start_times(res):
    return [i[1]["start_time"] for i in res]


def get_trigger_times(res):
    return [i[0].ts for i in res]


class SequenceConditionTest(unittest.TestCase):
    def test_sustained_sequence(self):
        result = self.__run_test('sustained(always, msg.str_value == "hello", 2)')
        self.assertEqual(get_start_times(result), [0, 5])

        result = self.__run_test('sustained(always, msg.str_value == "hello", 6)')
        self.assertEqual(get_start_times(result), [])

        result = self.__run_test(
            'sustained(topic == "t1", msg.str_value == "hello", 2)'
        )
        self.assertEqual(get_start_times(result), [0])

        result = self.__run_test(
            'sustained(topic == "t1", msg.str_value == "hello", 6)'
        )
        self.assertEqual(get_start_times(result), [])

    def test_sequence_pattern(self):
        result = self.__run_test(
            """sequential(
                topic == "t1" and msg.int_value == 1,
                topic == "t2" and msg.int_value == 4 and set_value("somekey", msg.int_value),
                topic == "t2" and msg.int_value == get_value("somekey"),
                duration=4,
            )"""
        )
        self.assertEqual(get_start_times(result), [0])

        result = self.__run_test(
            """sequential(
                topic == "t1" and msg.int_value == 1,
                topic == "t2" and msg.int_value == 4 and set_value("somekey", msg.int_value),
                topic == "t2" and msg.int_value == get_value("somekey"),
                duration=2,
            )"""
        )
        self.assertEqual(get_start_times(result), [])

    def test_sequence_timeout(self):
        result = self.__run_test(
            'timeout(msg.str_value == "hello", msg.str_value == "world", duration=3)'
        )
        self.assertEqual(get_start_times(result), [0, 5])

    def test_repeated(self):
        result = self.__run_test("repeated(always, 2, 5)")
        self.assertEqual(get_start_times(result), [0])

        result = self.__run_test('repeated(topic == "t2", 2, 5)')
        self.assertEqual(get_start_times(result), [1])

        result = self.__run_test('repeated(topic == "t2", 5, 5)')
        self.assertEqual(get_start_times(result), [1])

        result = self.__run_test('repeated(topic == "t2", 2, 0.5)')
        self.assertEqual(get_start_times(result), [1, 3, 4, 7])

    def test_debounce(self):
        result = self.__run_test('debounce(msg.str_value == "hello", 3)')
        self.assertEqual(get_trigger_times(result), [0])

        result = self.__run_test('debounce(msg.str_value == "hello", 1.5)')
        self.assertEqual(get_trigger_times(result), [0, 5, 9])

        result = self.__run_test('debounce(msg.str_value == "single", 3)')
        self.assertEqual(get_trigger_times(result), [9])

    def test_throttle(self):
        result = self.__run_test('throttle(msg.str_value == "hello", 3)')
        self.assertEqual(get_trigger_times(result), [0, 3, 6, 9])

    def test_any_order(self):
        result = self.__run_test(
            'any_order(topic == "t1", topic == "t2", topic == "t3")'
        )
        self.assertEqual(get_trigger_times(result), [9])

        result = self.__run_test(
            'any_order(topic == "t3", topic == "t2", topic == "t1")'
        )
        self.assertEqual(get_trigger_times(result), [9])

    def test_any_order_in_sequence(self):
        result = self.__run_test(
            """sequential(
                msg.str_value == "hello",
                any_order(msg.int_value == 1, msg.int_value == 5),
                duration=2,
            )"""
        )
        self.assertEqual(get_trigger_times(result), [])

    @staticmethod
    def __run_test(expr_str):
        action = CollectAction()
        engine = Engine([Rule([str_to_condition(expr_str)], [action], {})])
        for item in simple_sequence:
            engine.consume_next(item)
        return action.collector
