import type rosnodejs from 'rosnodejs'
import { writable } from 'svelte/store'

export const batteryInfo = writable<any[]>([{}, {}, {}, {}])

const ros: typeof rosnodejs = (window as any).api.ros

export function setup_ros(): void {
  ros.initNode('/pr2_web_dashboard')
  const nh = ros.nodeHandle

  nh.subscribe('/battery/server2', 'pr2_msgs/BatteryServer2', (msg: any) => {
    batteryInfo.update(info => {
      info[msg.id] = msg;
      return info
    })
  })
}

export function shutdown_ros(): void {
  ros.shutdown()
}
