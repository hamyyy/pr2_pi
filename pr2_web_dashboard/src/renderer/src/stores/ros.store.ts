import type rosnodejs from 'rosnodejs'
import type * as Shell from 'child_process'
import { writable } from 'svelte/store'

let shutdownCalledInternally = false;

export enum LogLevel { // values from "bunyan" package
  TRACE = 10,
  DEBUG = 20,
  INFO = 30,
  WARN = 40,
  ERROR = 50,
  FATAL = 60,
}

export const batteryInfo = writable<any[]>([{}, {}, {}, {}])
export const rosInfo = writable({
  running: false,
  teleop: false,
})
export const logs = writable<{
  message: string,
  level: LogLevel
}[]>([])

const ros: typeof rosnodejs = (window as any).api.ros
const shell: typeof Shell = (window as any).api.shell
const ipcRenderer: Electron.IpcRenderer = (window as any).api.ipcRenderer

export async function setupROS(): Promise<void> {
  await ros.initNode('/pr2_web_dashboard', {
    logging: {
      level: LogLevel.INFO
    }
  })
  const nh = ros.nodeHandle

  await new Promise<void>((res) => setTimeout(res, 1000))

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

    logs.update(logs => {
      logs.push({
        message: msg.msg,
        level: msg.level
      })

      if (logs.length > 100) {
        logs.shift()
      }
      return logs
    });
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