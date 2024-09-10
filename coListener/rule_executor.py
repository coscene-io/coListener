# Copyright 2024 coScene
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import json
import os
import threading
import time
from functools import partial
from typing import List, Tuple
from ruleengine.dsl.validation.config_validator import validate_config
from ruleengine.engine import Engine, DiagnosisItem

from coListener.shared_client_api import SharedClientAPI
from coListener.logger import logger as _log

cos_cache_path = os.path.expanduser("~/.cache") + "/cos/coListener"


def build_engine_from_rules(
        rules: List, should_trigger, trigger_cb, trigger_obj, upload_fn=None, moment_fn=None
):
    _log.info("check rules...")
    rule_list = []
    for project_rule in rules:
        project_name = project_rule["name"]

        if project_name.endswith("/diagnosisRule"):
            project_name = project_name[: -len("/diagnosisRule")]

        for rule in project_rule["rules"]:
            if not rule["enabled"]:
                continue
            vali_result, vali_rules = validate_config(
                rule,
                {
                    "upload": partial(
                        upload_fn, project_name=project_name, trigger_obj=trigger_obj
                    ),
                    "create_moment": partial(moment_fn, trigger_obj=trigger_obj),
                },
                project_name,
            )
            if not vali_result["success"]:
                _log.error(
                    f"Failed to build rule for {project_name} "
                    f"{json.dumps(rule_list, indent=2, ensure_ascii=False)}"
                    f"due to {json.dumps(vali_result, indent=2, ensure_ascii=False)}, skipping"
                )
                continue
            rule_list.extend(vali_rules)

    _log.info("check rules succeed, build rule engine!")
    return Engine(rule_list, should_trigger, trigger_cb)


class RuleExecutor:
    def __init__(
            self,
            upload_fn: partial,
            moment_fn: partial,
            should_trigger,
            trigger_cb,
            trigger_obj,
            api_client: SharedClientAPI
    ):
        self.__upload_fn = upload_fn
        self.__moment_fn = moment_fn
        self.__should_trigger = should_trigger
        self.__trigger_cb = trigger_cb
        self.__engine = None
        self.__rules = ""
        self.__exit_thread_event = threading.Event()

        self.__api_url = ""
        self.__token = ""
        self.__device_name = ""
        self.__trigger_obj = trigger_obj

        self.rules_dict = {}
        self._build_engine_from_cache()

        self.__lock = threading.Lock()
        self.__update_rules_thread = threading.Thread(
            target=self.update_rules,
            args=[api_client],
            kwargs={},
            daemon=True,
            name="update_rules_thread",
        )
        self.__update_rules_thread.start()
        self.min_before_triggered = -1  # one day =.=!
        self.min_after_triggered = -1

    def consume_msg(self, topic, msg, ts, msgtype):
        self.__lock.acquire()
        if self.__engine is not None:
            self.__engine.consume_next(DiagnosisItem(topic, msg, ts, msgtype))
        self.__lock.release()

    def check_rules_version_old(self, rules_str: str) -> Tuple[list, bool]:
        rules_json = json.loads(rules_str)

        rules = []
        need_update = False

        if len(rules_json) != len(self.rules_dict):
            need_update = True
            self.rules_dict.clear()

        for rule in rules_json:
            rules.extend(rule["rules"])

            version = rule["version"]
            project_name = rule["project_name"]
            if project_name not in self.rules_dict:
                self.rules_dict[project_name] = version
                need_update = True
            else:
                if version != self.rules_dict[project_name]:
                    self.rules_dict[project_name] = version
                    need_update = True

        return rules, need_update

    def check_rules_version(self, rules: dict) -> bool:
        for project, version in rules.items():
            if project not in self.rules_dict or version != self.rules_dict[project]:
                self.rules_dict[project] = version
                return True
            else:
                return False

    def update_rules(self, api_client: SharedClientAPI):
        while True:
            start_time = time.time()
            try:
                _log.info("fetch rules version from remote server...")
                rule_version = api_client.get_value().fetch_rules_version()
                if self.check_rules_version(rule_version):
                    _log.info("rules version has been changed, "
                              "fetch new rules from remote server...")
                    rules_response, is_success = api_client.get_value().fetch_rules(rule_version)
                    if is_success:
                        _log.info(f"build new rule engine by rules: {rules_response}")
                        self._write_rules_to_cache(rules_response)
                        rules = []
                        for rule in json.loads(rules_response):
                            rules.extend(rule["rules"])
                        self.__lock.acquire()
                        self.__engine = build_engine_from_rules(
                            rules,
                            self.__should_trigger,
                            self.__trigger_cb,
                            self.__trigger_obj,
                            self.__upload_fn,
                            self.__moment_fn,
                        )
                        self.__lock.release()
                    else:
                        _log.warn(f"get rules failed, node will try again later, "
                                  f"error info: {rules_response}")
                else:
                    _log.info("No changes to the rules.")
            except Exception as e:
                _log.error(f"An error occurred when get rules from remote server! {e}")

            if self.__exit_thread_event.wait(60.0 - time.time() + start_time):
                break

        _log.info("update_rules_thread ended")

    def exit_thread(self):
        _log.info("exit update_rules_thread...")
        self.__exit_thread_event.set()

    def _write_rules_to_cache(self, rules: str) -> None:
        with open(cos_cache_path + "/rules.cache", "w") as f:
            f.write(rules)

    def _read_rules_from_cache(self) -> str:
        if not os.path.exists(cos_cache_path + "/rules.cache"):
            return ""

        with open(cos_cache_path + "/rules.cache", "r") as f:
            return f.read()

    def _build_engine_from_cache(self) -> None:
        cache_rules = self._read_rules_from_cache()
        if cache_rules != "":
            _log.info(f'build engine from local cache: {cache_rules}')
            rules = []
            for rule in json.loads(cache_rules):
                rules.extend(rule["rules"])
                self.rules_dict[rule["project_name"]] = rule["version"]
            self.__engine = build_engine_from_rules(
                rules,
                self.__should_trigger,
                self.__trigger_cb,
                self.__trigger_obj,
                self.__upload_fn,
                self.__moment_fn,
            )
        else:
            _log.info('local cache rules is empty, skip build engine from local.')
