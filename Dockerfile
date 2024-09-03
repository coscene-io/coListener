FROM ros:foxy

ENV ROS_DISTRO=foxy

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN apt update && apt install wget -y && apt install ros-foxy-nav2-msgs -y && apt install unzip -y
RUN wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.34/mcap-linux-amd64 -O /usr/bin/mcap && chmod +x /usr/bin/mcap
RUN apt install pip -y
RUN pip install cos_ruleengine==0.1.30.dev0 appdirs requests


RUN mkdir -p /cos_ws/src/coListener
RUN mkdir -p /root/.config/cos
RUN mkdir -p /home/tyzc/AgiBot/scrubber/config/

COPY . /cos_ws/src/cosListener/.
COPY depends/cos-agent /usr/local/bin/cos-agent
COPY depends/config.yaml /root/.config/cos/config.yaml

COPY depends/robot.yaml /home/tyzc/AgiBot/scrubber/config/robot.yaml
COPY depends/robot.sn /home/tyzc/AgiBot/scrubber/config/robot.sn

WORKDIR /cos_ws
