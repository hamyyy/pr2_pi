<script lang="ts">
  import { setupROS, shutdownROS } from './stores/ros.store'
  import { onDestroy, onMount } from 'svelte'
  import { themeChange } from 'theme-change'

  import PageManager from './pages/PageManager.svelte'
  import BatteryMonitor from './pages/BatteryMonitor.svelte'
  import RosManager from './pages/RosManager.svelte'
  import AppSettings from './pages/AppSettings.svelte'
  import { startStstemInfoUpdate, stopSystemInfoUpdate } from './stores/sys.store'

  onMount(() => {
    themeChange(false)
    setupROS()
    startStstemInfoUpdate()
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
      title: 'Battery Monitor',
      component: BatteryMonitor
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
