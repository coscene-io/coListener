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
import requests
import yaml
from typing import Tuple, Dict
from requests.auth import HTTPBasicAuth
from coListener.logger import logger as _log


class RestApiClient:
    def __init__(self):
        self.__api_url = ""
        self.__token = ""
        self.__device_name = ""

        config_path = os.path.expanduser("~/.config/cos/config.yaml")
        if os.path.exists(config_path):
            with open(config_path, "r") as config_file:
                config = yaml.safe_load(config_file)
                self.__api_url = config["api"]["server_url"]

        client_state_path = os.path.expanduser(
            "~/.local/state/cos/api_client.state.json"
        )
        if os.path.exists(client_state_path):
            with open(client_state_path, "r") as client_cfg:
                client_state = json.load(client_cfg)
                self.__token = client_state["api_key"]
                self.__device_name = client_state["device"]["name"]

        self.__basic_auth = HTTPBasicAuth("apikey", self.__token)
        self.__request_headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }

    def _get_response(self, url, params):
        response = requests.get(
            url=url,
            params=params,
            headers=self.__request_headers,
            auth=self.__basic_auth,
        )
        if response.status_code != 200:
            _log.error(
                f"get response from {url} failed, "
                f"response code: {response.status_code}, message: {response.text}"
            )
            return {}
        return response.json()

    def _post_response(self, url, payload):
        response = requests.post(
            url=url,
            json=payload,
            headers=self.__request_headers,
            auth=self.__basic_auth,
        )
        if response.status_code != 200:
            _log.error(
                f"get response from {url} failed, "
                f"response code: {response.status_code}, message: {response.text}"
            )
            return {}
        return response.json()

    def fetch_rules_version(self) -> Dict[str, int]:
        rules_version = {}
        try:
            url = (
                f"{self.__api_url}/dataplatform/v1alpha1/{self.__device_name}/projects"
            )
            response = self._get_response(url, {"pageSize": 1000})
            projects = response.get("deviceProjects", [])

            for project in projects:
                project_name = str(project["name"])

                version_url = (f"{self.__api_url}/dataplatform/v1alpha2/"
                               f"{project_name}/diagnosisRule/metadata")
                ver = self._get_response(version_url, {}).get("currentVersion", -1)
                rules_version[project_name] = ver
            return rules_version
        except Exception:
            return {}

    def fetch_rules(self, projects: Dict) -> Tuple[str, bool]:
        try:
            device_rules = []
            for project, ver in projects.items():
                rule_url = (
                    f"{self.__api_url}/dataplatform/v1alpha2/{project}/diagnosisRules"
                )
                rules = self._get_response(rule_url, {}).get("diagnosisRules", [])

                device_rules.append(
                    {
                        "project_name": project,
                        "version": ver,
                        "rules": rules,
                    }
                )
            return json.dumps(device_rules), True
        except Exception as e:
            return str(e), False

    def rules_trigger_count(self, diagnosis_rule, hit, device=None) -> Tuple[any, bool]:
        try:
            url = f"{self.__api_url}/dataplatform/v1alpha2/{diagnosis_rule}:countDiagnosisRuleHits"

            payload = {
                "diagnosis_rule": diagnosis_rule,
                "hit": hit,
                "device": self.__device_name if device is None else device,
            }
            return self._post_response(url, payload).get("count", 0), True
        except Exception as e:
            return str(e), False

    def rules_triggered(
        self, diagnosis_rule, hit, upload, device=None
    ) -> Tuple[str, bool]:
        try:
            url = f"{self.__api_url}/dataplatform/v1alpha2/{diagnosis_rule.get('name', '')}:hit"
            payload = {
                "diagnosis_rule": diagnosis_rule,
                "hit": hit,
                "device": self.__device_name if device is None else device,
                "upload": upload,
            }
            return self._post_response(url, payload), True
        except Exception as e:
            return str(e), False
