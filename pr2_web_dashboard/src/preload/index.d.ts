import { ElectronAPI } from '@electron-toolkit/preload'
import rosnodejs from 'rosnodejs';

declare global {
  interface Window {
    electron: ElectronAPI
    api: {
      ros: typeof rosnodejs
    }
  }
}
