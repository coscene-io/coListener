import logging

_log = logging.getLogger(__name__)


class DiagnosisItem(object):
    def __init__(self, topic, msg, ts, msgtype):
        self.topic = topic
        self.msg = msg
        self.ts = ts
        self.msgtype = msgtype

    def __repr__(self):
        return (
            "DiagnosisItem(topic={!r}, msg={!r}, ts={!r}, msgtype={!r})"
            .format(self.topic, self.msg, self.ts, self.msgtype)
        )

    def __eq__(self, other):
        if isinstance(other, DiagnosisItem):
            return (self.topic == other.topic and
                    self.msg == other.msg and
                    self.ts == other.ts and
                    self.msgtype == other.msgtype)
        return False

    def __setattr__(self, name, value):
        if hasattr(self, name):
            raise AttributeError("Cannot modify attribute '{}' (frozen)".format(name))
        super(DiagnosisItem, self).__setattr__(name, value)

    def __getitem__(self, key):
        if key == "topic":
            return self.topic
        elif key == "msg":
            return self.msg
        elif key == "ts":
            return self.ts
        elif key == "msgtype":
            return self.msgtype
        else:
            raise KeyError("Key '{}' not found".format(key))


class Rule(object):
    def __init__(self, conditions, actions, initial_scope={}, upload_limit=None, spec=None, project_name=""):
        self.conditions = conditions
        self.actions = actions
        self.initial_scope = initial_scope
        self.upload_limit = upload_limit if upload_limit is not None else {}
        self.spec = spec if spec is not None else {}
        self.project_name = project_name

    def __repr__(self):
        return (
            "Rule(conditions={!r}, actions={!r}, initial_scope={!r}, upload_limit={!r}, spec={!r}, project_name={!r})"
            .format(self.conditions, self.actions, self.initial_scope, self.upload_limit, self.spec, self.project_name)
        )

    def __eq__(self, other):
        if isinstance(other, Rule):
            return (self.conditions == other.conditions and
                    self.actions == other.actions and
                    self.initial_scope == other.initial_scope and
                    self.upload_limit == other.upload_limit and
                    self.spec == other.spec and
                    self.project_name == other.project_name)
        return False



class Engine:
    def __init__(self, rules, should_trigger_action=None, trigger_cb=None):
        self.__rules = rules
        self.__should_trigger_action = should_trigger_action
        self.__trigger_cb = trigger_cb

    def consume_next(self, item):
        for rule in self.__rules:
            triggered_condition_indices = []
            triggered_scope = None

            for i, cond in enumerate(rule.conditions):
                res, scope = cond.evaluate_condition_at(item, rule.initial_scope)
                _log.debug("evaluate condition, result: {}, scope: {}".format(res, scope))
                if res:
                    triggered_condition_indices.append(i)
                if not triggered_scope:
                    triggered_scope = scope

            if not triggered_condition_indices:
                continue

            # For testing, rule.spec is not specified
            if not rule.spec:
                hit = {}
            else:
                hit = dict(rule.spec)
                if "when" in hit:
                    hit["when"] = [hit["when"][i] for i in triggered_condition_indices]

            action_triggered = False
            if not self.__should_trigger_action or self.__should_trigger_action(
                rule.project_name, rule.spec, hit
            ):
                action_triggered = True
                for action in rule.actions:
                    action.run(item, triggered_scope)

            if self.__trigger_cb:
                self.__trigger_cb(
                    rule.project_name, rule.spec, hit, action_triggered, item
                )
