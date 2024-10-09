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


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bag_storage_path = "/home/cos/files/bags"
    log_storage_path = "/home/cos/files/logs"
    error_code_topic = "/error_code"
    use_service = False
    waiting_data_minutes = 60

    return LaunchDescription(
        [
            Node(
                package="coListener",
                executable="coListener",
                name="coListener",
                output="screen",
                parameters=[
                    {
                        "bag_storage_path": bag_storage_path,
                        "log_storage_path": log_storage_path,
                        "error_code_topic": error_code_topic,
                        "use_service": use_service,
                        "waiting_data_minutes": waiting_data_minutes,
                    }
                ],
            )
        ]
    )
