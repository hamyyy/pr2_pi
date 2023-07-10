#!/usr/bin/env python3

import PySimpleGUI as sg
import rospy

from pr2_msgs.msg import BatteryServer

charge = [0] * 4

sg.theme('DarkAmber')
layout = [  [sg.Text('Some text on Row 1')],
            [sg.Text('Enter something on Row 2'), sg.InputText()],
            [sg.Button('Ok'), sg.Button('Cancel')] ]

window = sg.Window('Window Title', layout, no_titlebar=True)

def on_battery_msg(msg: BatteryServer):
  charge[msg.id - 1] = msg.averageCharge
  rospy.loginfo(f"{charge} : {msg.id}")
  layout[0] = [sg.Text(f'Some text on Row 1 ({charge[0]})')]

rospy.init_node("pi_battery_display_node")
sub = rospy.Subscriber("/battery/server", BatteryServer, on_battery_msg)

while not rospy.is_shutdown():
    event, values = window.read(timeout=10)
    if event == sg.WIN_CLOSED or event == 'Cancel':
        break

window.close()
