import { writable } from "svelte/store"
import type * as SystemInformation from "systeminformation"
const sys: typeof SystemInformation = (window as any).api.sys

interface SystemStore {
    cpuTemperature: number
    cpuLoad: number
}

export const systemInfo = writable<SystemStore>(null)

async function getData() {
    const [temp, load] = await Promise.all([sys.cpuTemperature(), sys.currentLoad()])
    systemInfo.set({
        cpuTemperature: temp.main,
        cpuLoad: load.currentLoad
    })
}


let intervalID: NodeJS.Timeout = null
export function startStstemInfoUpdate() {
    getData()
    clearInterval(intervalID)
    intervalID = setInterval(getData, 3000)
}

export function stopSystemInfoUpdate() {
    clearInterval(intervalID)
}