import { ElectronAPI } from '@electron-toolkit/preload'
import rosnodejs from 'rosnodejs';
import * as shell from "child_process";
import * as sys from "systeminformation";

declare global {
  interface Window {
    electron: ElectronAPI
    api: {
      ros: typeof rosnodejs,
      shell: typeof shell
      sys: typeof sys
    }
  }
}
