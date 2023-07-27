import type rosnodejs from "rosnodejs"
import { writable } from "svelte/store";

export const batteryInfo = writable<any>({});

const ros: typeof rosnodejs = (window as any).api.ros;

export function setup_ros() {
    ros.initNode('/pr2_web_dashboard')
    const nh = ros.nodeHandle;
    
    nh.subscribe("/battery/server", "pr2_msgs/BatteryServer", (msg: any) => {        
        batteryInfo.set(msg);
        console.log(msg);
    });
}

export function shutdown_ros() {
    ros.shutdown();
}