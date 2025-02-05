#!/usr/bin/env python
import shutil
import subprocess

import datetime
import json
import os.path
import rospy
import time
import uuid
import urllib2

from .logger.logger import logger as _log
from .rule_executor.rule_executor import RuleExecutor
from functools import partial
from keenon_ui_msgs.msg import error_report
from keenon_event_tracking.msg import event_info_array

coScout_path = os.path.join(os.getenv('HOME'), '.local', 'bin') + "/cos"
base_url = "http://localhost:22524/"

class CoListener:
    def __init__(self):
        self.bag_storage = "/home/peanut/keenon/bag"
        self.log_storage = "/home/peanut/keenon/log"
        self.schedule_storage = self.log_storage + "/schedule"
        self.state_storage = os.path.join(os.getenv('HOME'), '.local', 'state', 'cos', 'default', "state")
        self.merge_cache_dir = os.path.join(os.getenv('HOME'), '.cache', 'coListener', 'caches')

        if not os.path.exists(self.state_storage):
            os.makedirs(self.state_storage)

        if not os.path.exists(self.merge_cache_dir):
            os.makedirs(self.merge_cache_dir)

        self.trigger_uuid = {"uuid": ""}
        # self.client_api = SharedClientAPI(RestApiClient())

        self.rule_engine = RuleExecutor(
            partial(self.upload_file, storage_dir=""),
            partial(self.create_moment),
            partial(self.should_trigger),
            partial(self.actions_run_ended),
            self.trigger_uuid,
            # self.client_api
        )

        self.error_subscriber = rospy.Subscriber(
            "/error_report",
            error_report,
            self.error_report_callback
        )
        _log.info("CoListener subscribed to /error_report")

        self.event_subscriber = rospy.Subscriber(
            "/event_tracking_report",
            event_info_array,
            self.event_tracking_callback
        )
        _log.info("CoListener subscribed to /event_tracking_report")

    def error_report_callback(self, msg):
        msg_dict = {
            'stamp': {
                'secs': str(msg.stamp.secs),
                'nsecs': str(msg.stamp.nsecs),
            },
            'code': str(msg.code),
            'msg': msg.msg,
        }
        msg_obj = json.loads(json.dumps(msg_dict))
        _log.info("/error_report: {}".format(msg_obj))

        self.rule_engine.consume_msg(
            "/error_report", msg_obj, msg.stamp.secs + float(msg.stamp.nsecs) / 1000000000.0, ""
        )

    def event_tracking_callback(self, msg):
        msg_dict = {
            "events_size": msg.events_size,
            "code": [],
            "time": [],
            "data": [],
            "remark":[],
        }
        for event_info in msg.event_array:
            msg_dict["code"].append(event_info.code)
            msg_dict["time"].append(event_info.time)
            msg_dict["data"].append(event_info.data)
            msg_dict["remark"].append(event_info.remark)

        msg_obj = json.loads(json.dumps(msg_dict))
        _log.info("/event_tracking_report: {}".format(msg_obj))

        self.rule_engine.consume_msg(
            "/event_tracking_report", msg_obj, time.time(), ""
        )

    def should_trigger(self, project_name, rule_spec, hit):
        self.trigger_uuid["uuid"] = str(uuid.uuid4())
        _log.info("should_trigger:  trigger_uuid: {}, hit: {}".format(self.trigger_uuid["uuid"], hit))
        hit_json = json.dumps(hit)
        if "uploadLimit" not in hit:
            return True

        upload_limit = hit["uploadLimit"]
        try:
            if upload_limit.get("device", ""):

                data = {"project": project_name + "/diagnosisRule", "hit": hit}
                headers = {'Content-Type': 'application/json'}
                _log.info("data: {}".format(data))
                json_data = json.dumps(data)
                _log.info("json_data: {}".format(json_data))
                request = urllib2.Request(base_url + "/rules/count", data=json_data, headers=headers)
                response = urllib2.urlopen(request)

                device_trigger_count = json.loads(response.read())
                _log.info("get trigger-count from coScout: {}".format(device_trigger_count))
                if device_trigger_count["data"]["count"] >= upload_limit["device"]["times"]:
                    _log.info(
                        "device triggered count: {}, reached upload limit: {}, skipping"\
                            .format(device_trigger_count, upload_limit['device']['times'])
                    )
                    return False

            if upload_limit.get("global", ""):
                data = {"project": project_name + "/diagnosisRule", "hit": hit, "device": ""}
                headers = {'Content-Type': 'application/json'}
                json_data = json.dumps(data, ensure_ascii=False).encode('utf-8')
                request = urllib2.Request(base_url + "/rules/count", data=json_data, headers=headers)
                response = urllib2.urlopen(request)

                global_trigger_count = json.loads(response.read())
                if global_trigger_count["data"]["count"] >= upload_limit["global"]["times"]:
                    _log.info(
                        "global triggered count: {}, reached upload limit: {}, skipping"
                        .format(global_trigger_count, upload_limit['global']['times'])
                    )
                    return False
        except Exception as e:
            _log.info("get trigger count from coScout failed: {}".format(e))
            return False
        return True

    def actions_run_ended(self, project_name, rule_spec, hit, action_triggered, item, trigger_obj):
        _log.info("actions_run_ended:  trigger_uuid: {}".format(self.trigger_uuid["uuid"]))

        src = os.path.join(self.merge_cache_dir, trigger_obj["uuid"] + ".json")
        if os.path.exists(src):
            dst = os.path.join(self.state_storage, trigger_obj["uuid"] + ".json")
            _log.info("move file from [{}] to [{}]".format(src, dst))
            shutil.move(src, dst)

        project_rule_spec = {
            "name": "{}/diagnosisRule".format(project_name),
            "rules": [
                {
                    "rules": [rule_spec],
                }
            ],
        }
        payload = {
            "rule": project_rule_spec,
            "hit": hit,
            "triggered": action_triggered,
        }
        response = urllib2.urlopen(base_url + "/rules/trigger", json.dumps(payload).encode('utf-8'))
        _log.info("trigger rules response: {}".format(response.read()))

    def create_moment(
            self,
            title,
            description,
            timestamp,
            create_task,
            sync_task,
            trigger_obj,
            start_time,
            assign_to,
            custom_fields,
            rule_id,
            code,
    ):
        _log.debug("\ncreate_moment: \n\ttitle: {} \n\tdescription: {} \n\ttimestamp: {} \n\tcreate_task: {} \n\t"
                   "sync_task: {} \n\ttrigger_obj: {} \n\tstart_time:{} \n\tassign_to:{} \n\tcustom_fields: {}"
                   .format(title, description, timestamp, create_task, sync_task,
                           trigger_obj, start_time, assign_to, custom_fields))
        moment_data = {
            "moments": [
            {
                "title": title,
                "description": description,
                "timestamp": timestamp,
                "start_time": start_time if start_time != 0 else timestamp,
                "duration": 1 if start_time == 0 else timestamp - start_time,
                "create_task": create_task,
                "sync_task": sync_task,
                "assign_to": assign_to,
                "rule_id": rule_id,
                "code": code,
            }
        ]}

        cache_file = os.path.join(self.merge_cache_dir, trigger_obj["uuid"] + ".json")
        if os.path.exists(cache_file):
            with open(cache_file, "r") as f:
                json_data = json.load(f)
                moment_data.update(json_data)

        with open(cache_file, "w") as f:
            json.dump(moment_data, f)

    def upload_file(
            self,
            before,
            after,
            title,
            description,
            labels,
            extra_files,
            storage_dir,
            project_name,
            trigger_ts,
            rule,
            white_list,
            trigger_obj,
    ):
        _log.debug("\nupload_file:  \n\tbefore: {} \n\ttitle: {} \n\tdescription: {} \n\tlabels: {} \n\t"
                   "extra_files: {} \n\tstorage_dir: {} \n\tproject_name: {} \n\t"
                   "trigger_ts: {} \n\ttrigger_obj: {} \n\tafter: {}"
                  .format(before, title, description, labels, extra_files,
                          storage_dir, project_name, trigger_ts, trigger_obj, after))

        start = datetime.datetime.fromtimestamp(trigger_ts - before * 60)
        end = datetime.datetime.fromtimestamp(trigger_ts + after * 60)
        start_timestamp = int(time.mktime(start.timetuple()))
        end_timestamp = int(time.mktime(end.timetuple()))

        upload_data = {
            "flag": False,
            "projectName": project_name,
            "record": {},
            "diagnosis_task": {},
            "cut": {
                "extraFiles": extra_files,
                "start": start_timestamp,
                "end": end_timestamp,
                "whiteList": white_list,
            },
        }
        if title:
            upload_data["record"]["title"] = title
        if description:
            upload_data["record"]["description"] = description
        if labels:
            upload_data["record"]["labels"] = labels
        if rule:
            upload_data["record"]["rules"] = [{"id": rule.get("id", "")}]
            upload_data["diagnosis_task"]["rule_id"] = rule.get("id", "")
            upload_data["diagnosis_task"]["rule_name"] = rule.get("name", "")
        upload_data["diagnosis_task"]["trigger_time"] = int(trigger_ts)
        upload_data["diagnosis_task"]["start_time"] = start_timestamp
        upload_data["diagnosis_task"]["end_time"] = end_timestamp

        cache_file = os.path.join(self.merge_cache_dir, trigger_obj["uuid"] + ".json")

        if os.path.exists(cache_file):
            with open(cache_file, "r") as f:
                json_data = json.load(f)
                upload_data.update(json_data)

        with open(cache_file, "w") as f:
            json.dump(upload_data, f)

