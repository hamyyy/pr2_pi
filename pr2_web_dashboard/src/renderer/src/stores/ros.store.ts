import type rosnodejs from 'rosnodejs'
import type * as Shell from 'child_process'
import { writable } from 'svelte/store'

let shutdownCalledInternally = false;

export enum LogLevel {
  DEBUG = 1,
  INFO = 2,
  WARN = 4,
  ERROR = 8,
  FATAL = 16
}

export const batteryInfo = writable<any[]>([{}, {}, {}, {}])
export const rosInfo = writable({
  running: false,
  teleop: false,
})

const ros: typeof rosnodejs = (window as any).api.ros
const shell: typeof Shell = (window as any).api.shell
const ipcRenderer: Electron.IpcRenderer = (window as any).api.ipcRenderer

export async function setupROS(): Promise<void> {
  await ros.initNode('/pr2_web_dashboard')
  const nh = ros.nodeHandle

  nh.subscribe('/battery/server2', 'pr2_msgs/BatteryServer2', (msg: any) => {
    batteryInfo.update(info => {
      info[msg.id] = msg;
      return info
    })
  })

  nh.subscribe('/rosout', 'rosgraph_msgs/Log', (msg: any) => {
    switch (msg.level) {
      case LogLevel.DEBUG:
        console.debug(`[${LogLevel[msg.level]}] [${msg.name}] ${msg.msg}`);
        break;
      case LogLevel.INFO:
        console.info(`[${LogLevel[msg.level]}] [${msg.name}] ${msg.msg}`);
        break;
      case LogLevel.WARN:
        console.warn(`[${LogLevel[msg.level]}] [${msg.name}] ${msg.msg}`);
        break;
      case LogLevel.ERROR:
        console.error(`[${LogLevel[msg.level]}] [${msg.name}] ${msg.msg}`);
        break;
      case LogLevel.FATAL:
        console.error(`[${LogLevel[msg.level]}] [${msg.name}] ${msg.msg}`);
        break;
    }
  })

  shell.exec('rosnode list', (err, stdout, stderr) => {
    if (err || stdout.startsWith('ERROR') || stderr.startsWith('ERROR')) {
      console.error(err)
      console.error(stdout)
      console.error(stderr)
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
    } else if (nh.isShutdown() && shutdownCalledInternally) {
      clearInterval(id);
      setTimeout(() => {
        shutdownCalledInternally = false
      }, 2000)
    }
  }, 1000)

}

export function shutdownROS(): void {
  shutdownCalledInternally = true
  ros.shutdown()
}