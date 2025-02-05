#!/usr/bin/env python

import json
import os
import subprocess
import threading
import time
from functools import partial
import urllib

from ..rule_engine.dsl.validation.config_validator import validate_config_wrapped
from ..rule_engine.engine import Engine, DiagnosisItem

from ..logger.logger import logger as _log

coScout_path = os.path.join(os.getenv('HOME'), '.local', 'bin') + "/cos"
cos_cache_path = os.path.join(os.getenv('HOME'), '.cache', 'coListener', 'rules')
base_url = "http://localhost:22524/"

def build_engine_from_rules(
        rules, should_trigger, trigger_cb, trigger_obj, upload_fn=None, moment_fn=None
):
    _log.info("check rules...")
    rule_list = []
    for project_name, project_rules in rules.items():

        for rules in project_rules["rules"]:
            if not rules["enabled"]:
                continue
            vali_result, vali_rules = validate_config_wrapped(
                rules,
                {
                    "upload": lambda rule, _: partial(upload_fn, project_name=project_name, rule=rule, trigger_obj=trigger_obj),
                    "create_moment": lambda rule, each_scope: partial(moment_fn, trigger_obj=trigger_obj, rule_id=rule["id"], code=each_scope.get("code", "")),
                },
                project_name,
            )
            if not vali_result["success"]:
                _log.error("Failed to build rule! {}".format(vali_result))
                return None
            rule_list.extend(vali_rules)

    _log.info("check rules succeed, build rule engine!")
    return Engine(rule_list, should_trigger, partial(trigger_cb, trigger_obj=trigger_obj))


class RuleExecutor:
    def __init__(
            self,
            upload_fn,
            moment_fn,
            should_trigger,
            trigger_cb,
            trigger_obj,
            api_client = None,
            rules_cache_file= None,
    ):
        self._upload_fn = upload_fn
        self._moment_fn = moment_fn
        self._should_trigger = should_trigger
        self._trigger_cb = trigger_cb
        self._trigger_obj = trigger_obj
        self._rules_cache_file = rules_cache_file

        self._engine = None
        self._rules = ""
        self._exit_thread_event = threading.Event()
        self._lock = threading.Lock()

        self._api_url = ""
        self._token = ""
        self._device_name = ""

        self._rules_dict = {}

        if not os.path.exists(cos_cache_path):
            os.makedirs(cos_cache_path)

        self._build_engine_from_cache()
        # self._build_engine_from_coScout_command()

        self._update_rules_thread = threading.Thread(
            target=self.update_rules,
            args=[api_client],
            kwargs={},
            name="update_rules_thread",
        )
        self._update_rules_thread.daemon = True
        self._update_rules_thread.start()
        self.min_before_triggered = -1  # one day =.=!
        self.min_after_triggered = -1

    def consume_msg(self, topic, msg, ts, msgtype):
        with self._lock:
            if self._engine is not None:
                self._engine.consume_next(DiagnosisItem(topic, msg, ts, msgtype))

    def check_rules_version(self, rules):
        if len(rules) == 0:
            _log.info("remote server return no rules, skip this update.")
            return False

        if len(rules) != len(self._rules_dict):
            _log.info(
                "inconsistency in the amount of rules between remote and local, update rules"
            )
            for project, rules in rules.items():
                self._rules_dict[project] = rules["version"]
            return True

        for project, version in rules.items():
            if project not in self._rules_dict or version != self._rules_dict[project]:
                self._rules_dict[project] = version
                # _log.info(
                #     f"project {project} version {version}, "
                #     f"which in cache is {self.rules_dict[project]}"
                # )
                return True

        _log.info("check finished, no rules need to update")
        return False

    def update_rules(self, api_client):
        if self._exit_thread_event.wait(180.0):
            return
        while True:
            start_time = time.time()
            try:
                if self._rules_cache_file == "":
                    _log.info("fetch rules version from remote server...")
                    rule_version = api_client.get_value().fetch_rules_version()
                    if self.check_rules_version(rule_version):
                        _log.info("fetch new rules from remote server...")
                        rules_response, is_success = api_client.get_value().fetch_rules(
                            rule_version
                        )
                        if is_success:
                            _log.info("build new rule engine by rules: " + str(rules_response))
                            self._write_rules_to_cache(rules_response)
                            rules = []
                            for rule in json.loads(rules_response):
                                rules.extend(rule["rules"])
                            with self._lock:
                                self._engine = build_engine_from_rules(
                                    rules,
                                    self._should_trigger,
                                    self._trigger_cb,
                                    self._trigger_obj,
                                    self._upload_fn,
                                    self._moment_fn,
                                )
                        else:
                            _log.warn("get rules failed, node will try again later, "
                                + "error info: " + rules_response
                            )
                else:
                    _log.info("fetch rules version from coScout cache...")
                    self._build_engine_from_cache()
                    # self._build_engine_from_coScout_command()

            except Exception as e:
                _log.error("An error occurred when get rules from remote server! " + str(e))

            waiting_time = 180.0 - time.time() + start_time
            _log.info("update_rules ended, wait {} secs to refetch.".format(waiting_time))
            if self._exit_thread_event.wait(waiting_time):
                break

        _log.info("update_rules_thread ended")

    def exit_thread(self):
        _log.info("exit update_rules_thread...")
        self._exit_thread_event.set()

    def _write_rules_to_cache(self, rules):
        with open(cos_cache_path + "/rules.cache", "w") as f:
            f.write(rules)

    def _read_rules_from_cos(self):
        _log.info("get rules from cos...")
        if self._rules_cache_file is None or self._rules_cache_file == "":
            response = urllib.urlopen(base_url + "/rules")

            rules_str = json.dumps(json.loads(response.read())["data"], ensure_ascii=False)
            _log.info("cos response value: " + rules_str)
            return rules_str

        else:
            cache_file = self._rules_cache_file
            _log.info("read rules from config file: {}".format(cache_file))

            if not os.path.exists(cache_file):
                return ""

            with open(cache_file, "r") as f:
                return f.read()

    def _build_engine_from_cache(self):
        cache_rules = self._read_rules_from_cos()
        if cache_rules != "":
            _log.info("build engine from cos response: ")

            rules_dict = json.loads(cache_rules, encoding="utf-8")
            if self.check_rules_version(rules_dict):
                with self._lock:
                    self._engine = build_engine_from_rules(
                        rules_dict,
                        self._should_trigger,
                        self._trigger_cb,
                        self._trigger_obj,
                        self._upload_fn,
                        self._moment_fn,
                    )
        else:
            _log.info("local cache rules is empty, skip build engine from local.")

        # self.trigger_rules_test()


    def trigger_rules_test(self):
        msg_dict = {
            'stamp': {
                'secs': "1732682909",
                'nsecs': "442989909",
            },
            'code': "115100",
            'msg': "stm32 uart init fail!",
        }
        msg_obj = json.loads(json.dumps(msg_dict))
        _log.info("Received message from /error_report: {}".format(msg_obj))
        self.consume_msg(
            "/error_report", msg_obj, time.time(), ""
        )

