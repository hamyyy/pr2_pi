import type rosnodejs from 'rosnodejs'
import type * as Shell from 'child_process'
import { writable } from 'svelte/store'

let shutdownCalledInternally = false;

export const batteryInfo = writable<any[]>([{}, {}, {}, {}])
export const rosInfo = writable({
  running: false,
  teleop: false,
})

const ros: typeof rosnodejs = (window as any).api.ros
const shell: typeof Shell = (window as any).api.shell
const ipcRenderer: Electron.IpcRenderer = (window as any).api.ipcRenderer

export function setup_ros(): void {
  ros.initNode('/pr2_web_dashboard')
  const nh = ros.nodeHandle

  nh.subscribe('/battery/server2', 'pr2_msgs/BatteryServer2', (msg: any) => {
    batteryInfo.update(info => {
      info[msg.id] = msg;
      return info
    })
  })

  shell.exec('rosnode list', (err, stdout, _stderr) => {
    if (err) {
      console.error(err)
      return
    }

    const nodes = stdout.split("\n");

    let teleop = false
    for (let nodeName of nodes) {
      if (nodeName === "/pr2_teleop_general_joystick") {
        teleop = true
      }
    }

    rosInfo.set({
      running: true,
      teleop: teleop
    })
  });

  const id = setInterval(() => {
    if (nh.isShutdown() && !shutdownCalledInternally) {
      clearInterval(id);
      ipcRenderer.send('shutdown')
    }
  }, 1000)

}

export function shutdown_ros(): void {
  shutdownCalledInternally = true
  ros.shutdown()
}