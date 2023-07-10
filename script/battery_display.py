#!/usr/bin/env python3

import PySimpleGUI as sg
import rospy

from pr2_msgs.msg import BatteryServer2

charge = [0] * 4

sg.theme('DarkAmber')
layout = [  [sg.Text('Some text on Row 1')],
            [sg.Text('Enter something on Row 2'), sg.InputText()],
            [sg.Button('Ok'), sg.Button('Cancel')] ]

window = sg.Window('Window Title', layout, no_titlebar=True)

def on_battery_msg(msg: BatteryServer2):
  charge[msg.id - 1] = msg.average_charge
  rospy.loginfo(f"{charge} : {msg.id}")
  layout[0] = [sg.Text(f'Some text on Row 1 ({charge[0]})')]
  for bat in msg.battery:
    bat.battery_register = []
    bat.battery_update_flag = []
    bat.battery_register_update = []

  rospy.loginfo(msg)

rospy.init_node("pi_battery_display_node")
sub = rospy.Subscriber("/battery/server2", BatteryServer2, on_battery_msg)

while not rospy.is_shutdown():
    event, values = window.read(timeout=10)
    if event == sg.WIN_CLOSED or event == 'Cancel':
        break

window.close()
