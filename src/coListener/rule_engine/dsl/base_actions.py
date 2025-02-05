from .condition import Condition
from .base_conditions import ts, condition_start_time, concat
from .action import Action


def noop_upload(
        # trigger_ts: int
        trigger_ts,
        # before: int
        before,
        # after: int
        after,
        # title: str
        title,
        # description: str
        description,
        # labels: List[str]
        labels,
        # extra_files: List[str]
        extra_files,
        # white_list: List[str],
        white_list,
):
    pass


def noop_create_moment(
        # title: str
        title,
        # description: str
        description,
        # timestamp: int
        timestamp,
        # start_time: int
        start_time,
        # create_task: bool
        create_task,
        # sync_task: bool
        sync_task,
        # assign_to: Optional[str]
        assign_to,
        # custom_fields: Optional[str]
        custom_fields,
):
    pass


noop = {"upload": noop_upload, "create_moment": noop_create_moment}


class ForwardingAction(Action):
    def __init__(self, thunk, args):
        self.__thunk = thunk
        self.__args = args

    def run(self, item, scope):
        actual_args = {}
        for name, value in self.__args.items():
            if isinstance(value, Condition):
                new_value, _ = value.evaluate_condition_at(item, scope)
                actual_args[name] = new_value
            else:
                actual_args[name] = value
        self.__thunk(**actual_args)


def upload_factory(impl):
    def res(
        title=concat("Device auto upload @ ", ts),
        description="",
        labels=[],
        extra_files=[],
        white_list=[],
        before=10,
        after=0,
    ):
        args = {
            "before": before,
            "after": after,
            "title": Condition.map(Condition.wrap(title), str),
            "description": Condition.map(Condition.wrap(description), str),
            "labels": labels,
            "extra_files": extra_files,
            "white_list": white_list,
            "trigger_ts": ts,
        }

        return ForwardingAction(impl, args)

    return res


def create_moment_factory(impl):
    def res(
        title,
        description="",
        timestamp=ts,
        start_time=condition_start_time,
        create_task=False,
        sync_task=False,
        assign_to=None,
        custom_fields="",
    ):
        args = {
            "title": Condition.map(Condition.wrap(title), str),
            "description": Condition.map(Condition.wrap(description), str),
            "timestamp": Condition.map(Condition.wrap(timestamp), float),
            "start_time": Condition.map(Condition.wrap(start_time), float),
            "create_task": Condition.wrap(create_task),
            "sync_task": Condition.wrap(sync_task),
            "assign_to": assign_to,
            "custom_fields": Condition.map(Condition.wrap(custom_fields), str),
        }
        return ForwardingAction(impl, args)

    return res
