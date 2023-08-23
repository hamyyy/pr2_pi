<script lang="ts">
  import { setupROS, shutdownROS } from './stores/ros.store'
  import { onDestroy, onMount } from 'svelte'
  import { themeChange } from 'theme-change'

  import PageManager from './pages/PageManager.svelte'
  import BatteryMonitor from './pages/BatteryMonitor.svelte'
  import RosManager from './pages/RosManager.svelte'
  import AppSettings from './pages/AppSettings.svelte'
  import { startStstemInfoUpdate, stopSystemInfoUpdate } from './stores/sys.store'
  import DemoPage from './pages/DemoPage.svelte'
  import { scanForDemos } from './stores/ros-demos.store'
  import LogsPage from './pages/LogsPage.svelte'

  onMount(async () => {
    themeChange(false)
    setupROS()
    startStstemInfoUpdate()
    scanForDemos()
  })

  onDestroy(() => {
    shutdownROS()
    stopSystemInfoUpdate()
  })

  let pages = [
    {
      title: 'ROS Manager',
      component: RosManager
    },
    {
      title: 'Demo Page',
      component: DemoPage
    },
    {
      title: 'Battery Monitor',
      component: BatteryMonitor
    },
    {
      title: 'ROS Logs',
      component: LogsPage
    },
    {
      title: 'App Settings',
      component: AppSettings
    }
  ]
</script>

<main class="flex flex-row justify-around items-center h-screen">
  <PageManager {pages} />
</main>
