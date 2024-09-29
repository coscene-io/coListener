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
import re

import appdirs
import datetime
import json
import os
import pytz
import uuid

from collections import namedtuple

from rclpy.callback_groups import ReentrantCallbackGroup

from clean_msgs.srv import CommonService
from functools import partial
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import String
from typing import Optional, List, Dict
from diagnostic_msgs.msg import KeyValue

from coListener.rule_executor import RuleExecutor
from coListener.logger import logger as _log
from coListener.rest import RestApiClient
from coListener.shared_client_api import SharedClientAPI

RuleDataItem = namedtuple("RuleDataItem", "topic msg ts msgtype")
cos_agent_path = os.path.expanduser("~/.local") + "/bin/cos"


class TaskInfo:
    def __init__(
        self,
        title: str,
        description: str,
        assignee: str,
        record_name: str,
        sync_task: bool,
    ):
        self.title = title
        self.description = description
        self.assignee = assignee
        self.record_name = record_name
        self.sync_task = sync_task

    def to_dict(self):
        return {
            "title": self.title,
            "description": self.description,
            "assignee": self.assignee,
            "record_name": self.record_name,
            "sync_task": self.sync_task,
        }


class MomentInfo:
    def __init__(
        self,
        title: str,
        description: str,
        timestamp: int,
        duration: int,
        task: TaskInfo,
    ):
        self.title = title
        self.description = description
        self.timestamp = timestamp
        self.duration = duration
        self.task = task

    def to_dict(self):
        return {
            "title": self.title,
            "description": self.description,
            "timestamp": self.timestamp,
            "duration": self.duration,
            "task": self.task.to_dict(),
        }


class UploadFileInfo:
    def __init__(
        self,
        uploaded: bool = False,
        skipped: bool = False,
        event_code: str = "" or None,
        project_name: str = "",
        timestamp: int = 0,
        labels: List[str] = None,
        record: Dict = None,
        moments: List[MomentInfo] = None,
        task: Dict = None,
        files: List = None,
        file_infos: List = None,
    ):
        if labels is None:
            labels = []
        if record is None:
            record = {}
        if moments is None:
            moments = []
        if task is None:
            task = {}
        if files is None:
            files = []
        if file_infos is None:
            file_infos = []

        self.uploaded = uploaded
        self.skipped = skipped
        self.event_code = event_code
        self.project_name = project_name
        self.timestamp = timestamp
        self.labels = labels
        self.record = record
        self.moments = moments
        self.task = task
        self.files = files
        self.file_infos = file_infos

    def to_dict(self):
        return {
            "uploaded": self.uploaded,
            "skipped": self.skipped,
            "event_code": self.event_code,
            "project_name": self.project_name,
            "timestamp": self.timestamp,
            "labels": self.labels,
            "record": self.record,
            "moments": [moment.to_dict() for moment in self.moments],
            "task": self.task,
            "files": self.files,
            "file_infos": self.file_infos,
        }


def collect_files(dir_or_file: str) -> List:
    files = []
    if os.path.isfile(dir_or_file):
        files.append(
            {
                "filepath": dir_or_file,
                "filename": os.path.basename(dir_or_file),
                "size": None,
                "sha256": None,
            }
        )
    else:
        for sub_file in Path(dir_or_file).glob("**/*"):
            if sub_file.is_file():
                filename = str(sub_file.relative_to(Path(dir_or_file).parent))
                files.append(
                    {
                        "filepath": str(sub_file),
                        "filename": filename,
                        "size": None,
                        "sha256": None,
                    }
                )
    return files


class CoListener(Node):
    def __init__(self):
        super().__init__("coListener_node")

        self.trigger_uuid = {"uuid": ""}
        self.merge_cache_dir = appdirs.user_cache_dir() + "/cos/coListener/merge_cache/"
        self.upload_info_cache_dir = appdirs.user_state_dir() + "/cos/records/"

        self.declare_parameter("bag_storage_path", "/home/cos/files/bags")
        self.bag_storage_path = (
            self.get_parameter("bag_storage_path").get_parameter_value().string_value
        )

        self.declare_parameter("log_storage_path", "/home/cos/files/logs")
        self.log_storage_path = (
            self.get_parameter("log_storage_path").get_parameter_value().string_value
        )

        self.declare_parameter("error_code_topic", "/error_code")
        self.error_code_topic = (
            self.get_parameter("error_code_topic").get_parameter_value().string_value
        )

        self.declare_parameter("waiting_data_minutes", 10)
        self.waiting_minutes = (
            self.get_parameter("waiting_data_minutes")
            .get_parameter_value()
            .integer_value
        )

        self.declare_parameter("use_service", False)
        self.use_service = (
            self.get_parameter("use_service").get_parameter_value().bool_value
        )

        package_share_directory = os.path.dirname(__file__)
        version_file_path = os.path.join(package_share_directory, "version")
        with open(version_file_path) as f:
            self.version = f.read()

        _log.info(
            f"[node params] bag_storage_path: {self.bag_storage_path}, "
            f"error_code_topic: {self.error_code_topic}, "
            f"waiting_data_minutes: {self.waiting_minutes},"
            f"use_service: {self.use_service},"
            f"cos_agent_path: {cos_agent_path},"
            f"version: {self.version}"
        )

        if not os.path.exists(self.bag_storage_path):
            os.makedirs(self.bag_storage_path)

        if not os.path.exists(self.merge_cache_dir):
            os.makedirs(self.merge_cache_dir)

        if not os.path.exists(self.upload_info_cache_dir):
            os.makedirs(self.upload_info_cache_dir)

        self.callback_group = ReentrantCallbackGroup()

        _log.info(f"subscribe topic: {self.error_code_topic}")
        self.subscription = self.create_subscription(
            String,
            self.error_code_topic,
            self.error_code_callback,
            10,
            callback_group=self.callback_group,
        )

        if self.use_service:
            self.service_client = self.create_client(
                CommonService, "record_service", callback_group=self.callback_group
            )
            while not self.service_client.wait_for_service(timeout_sec=1.0):
                _log.info("Service not available, waiting again...")
            self.timer = self.create_timer(30, self._collect_upload_files)
        else:
            self.timer = self.create_timer(60, self._check_and_upload_files)

        self.client_api = SharedClientAPI(RestApiClient())
        self.update_config_timer = self.create_timer(3600, self._reload_config)

        self.rule_engine = RuleExecutor(
            partial(self.upload_file, storage_dir=""),
            partial(self.create_moment),
            partial(self.should_trigger),
            partial(self.actions_run_ended),
            self.trigger_uuid,
            self.client_api,
        )

    def should_trigger(self, project_name: str, rule_spec: dict, hit):
        self.trigger_uuid["uuid"] = str(uuid.uuid4())
        _log.info(
            f"{self.trigger_uuid['uuid']} RULES TRIGGERED !!! {rule_spec['when']}"
        )

        if not hit.get("uploadLimit", ""):
            return True

        proj_rule_spec = {
            "name": f"{project_name}/diagnosisRule",
            "rules": [
                {
                    "rules": [rule_spec],
                }
            ],
        }
        upload_limit = hit["uploadLimit"]
        if upload_limit.get("device", ""):
            device_count, is_success = self.client_api.get_value().rules_trigger_count(
                proj_rule_spec["name"], hit
            )
            if is_success:
                _log.info(
                    f"rules has been triggered {device_count} times in this device."
                )
                if device_count >= upload_limit["device"]["times"]:
                    _log.info(
                        f"device triggered count: {device_count},"
                        f" reached upload limit: {upload_limit['device']['times']}, skipping"
                    )
                    return False
            else:
                _log.warn(
                    f"get rules device-trigger count failed, because: {device_count}."
                )

        if upload_limit.get("global", ""):
            global_count, is_success = self.client_api.get_value().rules_trigger_count(
                proj_rule_spec["name"], hit, ""
            )
            if is_success:
                _log.info(
                    f"rules has been triggered {global_count} times in global scope."
                )
                if global_count >= upload_limit["global"]["times"]:
                    _log.info(
                        f"global triggered count: {global_count},"
                        f" reached upload limit: {upload_limit['global']['times']}, skipping"
                    )
                    return False
            else:
                _log.warn(
                    f"get rules global-trigger count failed, because: {device_count}."
                )

        return True

    def actions_run_ended(self, project_name, rule_spec, hit, action_triggered, item):
        _log.info(f"{self.trigger_uuid['uuid']} all actions completed.")
        project_rule_spec = {
            "name": f"{project_name}/diagnosisRule",
            "rules": [
                {
                    "rules": [rule_spec],
                }
            ],
        }
        self.client_api.get_value().rules_triggered(
            project_rule_spec, hit, action_triggered
        )

        if self.use_service:
            self.send_request(
                "record_bag",
                self.trigger_uuid["uuid"],
                [
                    KeyValue(key="uuid", value=self.trigger_uuid["uuid"]),
                    KeyValue(key="rules", value=f"{rule_spec['when']}"),
                ],
            )
        else:
            cache_file = f"{self.merge_cache_dir}{self.trigger_uuid['uuid']}"
            if os.path.exists(cache_file):
                with open(cache_file, "r+") as file:
                    json_data = json.load(file)
                    json_data["completed"] = True
                    file.seek(0)
                    file.truncate()
                    json.dump(json_data, file, indent=4)

    def create_moment(
        self,
        title: str,
        description: str,
        timestamp: float,
        create_task: bool,
        sync_task: bool,
        trigger_obj,
        start_time: float,
        assign_to: Optional[str],
        custom_fields: str,
    ):
        _log.info(f"{trigger_obj['uuid']} action - create moment: {title}")
        cache_file = f"{self.merge_cache_dir}{trigger_obj['uuid']}"
        json_data = {}
        if os.path.exists(cache_file):
            with open(cache_file, "r") as file:
                json_data = json.load(file)

        json_data["moments"] = [
            {
                "title": title,
                "description": description,
                "timestamp": timestamp,
                "start_time": start_time if start_time != 0 else timestamp,
                "duration": 1 if start_time == 0 else timestamp - start_time,
                "create_task": create_task,
                "sync_task": sync_task,
                "assign_to": assign_to,
            }
        ]
        json_data["completed"] = False

        with open(cache_file, "w", encoding="utf8") as fp:
            json.dump(json_data, fp, indent=4)

    def upload_file(
        self,
        before: int,
        title: str,
        description: str,
        labels: List,
        extra_files: List,
        storage_dir: Path,
        project_name: str,
        trigger_ts: float,
        trigger_obj,
        after=0,
    ):
        trigger_time = datetime.datetime.fromtimestamp(trigger_ts)
        _log.info(
            f"{trigger_obj['uuid']} trigger time: {trigger_ts} ( {trigger_time} ) ."
        )
        _log.info(
            f"{trigger_obj['uuid']} action - upload file: coListener will collect bags "
            f"which time range is {before} minutes ago "
            f"and {after} minutes after"
        )
        cache_file = f"{self.merge_cache_dir}{trigger_obj['uuid']}"
        json_data = {}
        if os.path.exists(cache_file):
            with open(cache_file, "r") as file:
                json_data = json.load(file)

        json_data["before"] = before
        json_data["after"] = after
        json_data["title"] = title
        json_data["description"] = description
        json_data["labels"] = labels
        json_data["extra_files"] = extra_files
        json_data["storage_dir"] = storage_dir
        json_data["project_name"] = project_name
        json_data["completed"] = False
        json_data["trigger_time"] = trigger_ts

        with open(cache_file, "w", encoding="utf8") as fp:
            json.dump(json_data, fp, indent=4)

    def send_request(self, srv_type, srv_data, kvs):
        service_request = CommonService.Request()
        service_request.type = srv_type
        service_request.data = srv_data
        service_request.kvs = kvs
        _log.info("sending service request")
        future = self.service_client.call_async(service_request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        cache_file = ""
        files = []
        try:
            response = future.result()
            _log.info(f"service response: {response}")
            cache_file = f"{self.merge_cache_dir}{response.data}"
            for kv in response.kvs:
                if kv.key == "destination_bag":
                    files.append(kv.value)
        except Exception as e:
            _log.error(f"call service failed: {e}")

        with open(cache_file, "r") as file:
            json_data = json.load(file)

        json_data["files"] = files
        json_data["completed"] = True

        with open(cache_file, "w", encoding="utf8") as fp:
            json.dump(json_data, fp, indent=4)

    def error_code_callback(self, msg):
        try:
            json_str = msg.data
            _log.info(f"received message: {json_str}")

            json_obj = json.loads(json_str)
            self.rule_engine.consume_msg(
                self.error_code_topic, json_obj, datetime.datetime.now().timestamp(), ""
            )
        except Exception as e:
            _log.info(f"catch exception when processing topic message: {e}")

    def create_upload_info_by_json(
        self, cache_data: dict, append_files: List or None = None
    ) -> UploadFileInfo:
        info = UploadFileInfo()
        info.uploaded = False
        info.skipped = False
        info.event_code = None
        info.project_name = cache_data["project_name"]
        info.timestamp = int(cache_data["trigger_time"] * 1000)
        info.labels = cache_data["labels"]
        info.record = {
            "title": cache_data["title"],
            "description": cache_data["description"]
            + f"\n\ncoListener version: {self.version}",
        }
        if append_files is not None:
            info.file_infos.extend(append_files)

        if "files" in cache_data:
            for upload_file in cache_data["files"]:
                info.file_infos.extend(collect_files(upload_file))

        if "extra_files" in cache_data:
            for extra_file in cache_data["extra_files"]:
                info.file_infos.extend(collect_files(extra_file))

        if "moments" in cache_data:
            for moment in cache_data["moments"]:
                info.moments.append(
                    MomentInfo(
                        title=moment["title"],
                        description=moment["description"],
                        timestamp=int(moment["start_time"] * 1000),
                        duration=int(moment["duration"] * 1000),
                        task=TaskInfo(
                            title="",
                            description="",
                            assignee=(
                                moment["assign_to"]
                                if "assign_to" in moment
                                and moment["assign_to"] is not None
                                else ""
                            ),
                            record_name="",
                            sync_task=(
                                moment["sync_task"]
                                if "sync_task" in moment
                                and moment["sync_task"] is not None
                                else False
                            ),
                        ),
                    )
                )
        return info

    def _write_upload_info_to_cos_state(
        self, trigger_time: datetime, info: UploadFileInfo
    ):
        milliseconds = trigger_time.microsecond // 1000
        state_dir = (
            self.upload_info_cache_dir
            + trigger_time.astimezone(pytz.utc).strftime("%Y-%m-%d-%H-%M-%S_")
            + str(milliseconds)
            + "/.cos/"
        )

        _log.info(f"write json to {state_dir + 'state.json'}")
        if not os.path.exists(state_dir):
            os.makedirs(state_dir)

        with open(state_dir + "state.json", "w", encoding="utf8") as fp:
            json.dump(info.to_dict(), fp, indent=4)

    def _collect_bag_files(self, trigger_time, cache_data):
        bag_interval_minutes = 2
        bags_to_upload = []
        bag_files = []
        files = os.listdir(self.bag_storage_path)
        for bag_file in files:
            bag_full_path = os.path.join(self.bag_storage_path, bag_file)
            if os.path.isfile(bag_full_path) and bag_file.endswith(".tar.gz"):
                bag_files.append(os.path.abspath(bag_full_path))
        if len(bag_files) == 0:
            _log.info(f"can't find any bag files in {self.bag_storage_path}")
            return False, bags_to_upload
        bag_files.sort()

        end_bag_datetime = datetime.datetime.strptime(
            # bag_yyyy_mm_dd_hhmmss.tar.gz
            os.path.basename(bag_files[-1])[4:-7],
            "%Y_%m_%d_%H%M%S",
        )
        needed_time = trigger_time + datetime.timedelta(minutes=cache_data["after"])
        limit_time = trigger_time + datetime.timedelta(minutes=self.waiting_minutes)
        current_time = datetime.datetime.now()

        if end_bag_datetime < needed_time and limit_time > current_time:
            _log.info(
                f"The time condition is not met, {needed_time} was needed, "
                + f"but latest bag time is {end_bag_datetime}, skip."
            )
            return False, bags_to_upload

        if limit_time < current_time:
            cache_data["description"] += (
                f"[ {self.waiting_minutes} minutes after the "
                f"error code was triggered,"
                f" data collection was incomplete, maybe you should "
                f"check the recording node.]"
            )

        minutes_before = trigger_time - datetime.timedelta(
            minutes=cache_data["before"] + bag_interval_minutes
        )
        minutes_after = trigger_time + datetime.timedelta(minutes=cache_data["after"])

        for bag_file in bag_files:
            bag_datetime = datetime.datetime.strptime(
                os.path.basename(bag_file)[4:-7], "%Y_%m_%d_%H%M%S"
            )
            if minutes_before < bag_datetime < minutes_after:
                bags_to_upload.append(
                    {
                        "filepath": bag_file,
                        "filename": os.path.relpath(bag_file, self.bag_storage_path),
                        "size": None,
                        "sha256": None,
                    }
                )
        return True, bags_to_upload

    def _collect_log_files(self):
        log_files = []
        log_directory = os.path.join(
            self.log_storage_path, datetime.datetime.now().strftime("%Y%m%d")
        )

        for node_name in os.listdir(log_directory):
            node_log_dir = os.path.join(log_directory, node_name)
            if node_name == "edge_gateway":
                for node_log in os.listdir(node_log_dir):
                    if not node_log.endswith(".log.gz"):
                        log_files.append(os.path.join(node_log_dir, node_log))
            elif node_name == "edge_ui":
                for node_log in os.listdir(node_log_dir):
                    log_files.append(os.path.join(node_log_dir, node_log))
            elif node_name == "navigation":
                # 2024-09-23-09-54-49-199047-RK3588-3810
                max_datetime = datetime.datetime.min
                dir_to_update = ""
                for node_log in os.listdir(node_log_dir):
                    match = re.search(
                        r"(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})", node_log
                    )
                    if match:
                        file_date_time = datetime.datetime.strptime(
                            match.group(1), "%Y-%m-%d-%H-%M-%S"
                        )
                        if file_date_time > max_datetime:
                            dir_to_update = os.path.join(node_log_dir, node_log)
                log_files.append(os.path.join(dir_to_update, "launch.log"))
            else:
                max_datetime = datetime.datetime.min
                file_to_update = ""
                for node_log in os.listdir(node_log_dir):
                    if node_log.endswith(".INFO") or node_log.endswith(".WARNING"):
                        log_files.append(os.path.join(node_log_dir, node_log))
                    match = re.search(rf"{node_name}(\d{{8}}-\d{{6}})", node_log)
                    if match:
                        file_date_time = datetime.datetime.strptime(
                            match.group(1), "%Y%m%d-%H%M%S"
                        )
                        if file_date_time > max_datetime:
                            file_to_update = os.path.join(node_log_dir, node_log)
                if file_to_update != "":
                    log_files.append(file_to_update)

        logs_to_upload = []
        for log_file in log_files:
            logs_to_upload.append(
                {
                    "filepath": log_file,
                    "filename": os.path.relpath(log_file, self.log_storage_path),
                    "size": None,
                    "sha256": None,
                }
            )

        return logs_to_upload

    def _check_and_upload_files(self):
        _log.info("check and upload files...")
        for cache_file in os.listdir(self.merge_cache_dir):
            _log.info(f"find cache file: {cache_file}")
            full_path = os.path.join(self.merge_cache_dir, cache_file)
            if not os.path.isfile(full_path):
                continue

            cache_data = {}
            with open(full_path, "r") as file:
                cache_data = json.load(file)

            if not (cache_data["completed"]):
                _log.info(
                    f"cache file [{cache_file}] not ready, wait for next check..."
                )
                continue

            trigger_time = datetime.datetime.fromtimestamp(cache_data["trigger_time"])

            is_ok, files_to_upload = self._collect_bag_files(trigger_time, cache_data)
            if not is_ok:
                _log.info("the files that need to be uploaded are not yet ready.")
                continue

            logs_to_upload = self._collect_log_files()
            files_to_upload.extend(logs_to_upload)

            info = self.create_upload_info_by_json(cache_data, files_to_upload)

            self._write_upload_info_to_cos_state(trigger_time, info)
            _log.info(f"remove {full_path}")
            os.remove(full_path)

    def _collect_upload_files(self):
        def check_files_exist(files: List[str]) -> bool:
            for f in files:
                if not os.path.exists(f):
                    return False
            return True

        for cache_file in os.listdir(self.merge_cache_dir):
            full_path = os.path.join(self.merge_cache_dir, cache_file)
            if os.path.isfile(full_path):
                cache_data = {}
                with open(full_path, "r") as file:
                    cache_data = json.load(file)
                    if not (
                        cache_data["completed"]
                        and "files" in cache_data
                        and check_files_exist(cache_data["files"])
                    ):
                        _log.info("upload files not ready, wait for next check...")
                        continue

                info = self.create_upload_info_by_json(cache_data)

                trigger_time = datetime.datetime.fromtimestamp(
                    cache_data["trigger_time"]
                )
                self._write_upload_info_to_cos_state(trigger_time, info)

                _log.info(f"remove {full_path}")
                os.remove(full_path)

    def _reload_config(self):
        _log.info("reload config")
        self.client_api.set_value(RestApiClient())
