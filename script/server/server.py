#!/usr/bin/env python3

import os
import rospy
from rospy_message_converter import message_converter

from flask import Flask, send_from_directory
from pr2_msgs.msg import BatteryServer2

batteryInfo: BatteryServer2 = None

app = Flask(__name__)


@app.route("/")
def base():
    return send_from_directory('../dist', 'index.html')


@app.route("/battery-charge")
def charge():
    global batteryInfo
    return message_converter.convert_ros_message_to_dictionary(batteryInfo)


@app.route("/<path:path>")
def home(path):
    return send_from_directory('../dist', path)


def on_battery_msg(msg: BatteryServer2):
    global batteryInfo
    batteryInfo = msg


def main():
    rospy.init_node("pr2_pi_server")

    sub = rospy.Subscriber("/battery/server2", BatteryServer2, on_battery_msg)

    os.system("pkill midori")
    os.system("midori -e Fullscreen http://localhost:5750/ &")
    app.run(host="localhost", port=5750)
    os.system("pkill midori")


if __name__ == '__main__':
    main()
