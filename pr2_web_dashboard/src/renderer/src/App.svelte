<script lang="ts">
  import { setup_ros, shutdown_ros } from './stores/ros.store'
  import { onDestroy, onMount } from 'svelte'

  import PageManager from './pages/PageManager.svelte'
  import BatteryMonitor from './pages/BatteryMonitor.svelte'
  import RosManager from './pages/RosManager.svelte'

  onMount(() => {
    setup_ros()
  })

  onDestroy(() => {
    shutdown_ros()
  })

  let pages = [
    {
      title: 'ROS Manager',
      component: RosManager
    },
    {
      title: 'Battery Monitor',
      component: BatteryMonitor
    }
  ]
</script>

<main>
  <PageManager {pages} />
</main>

<style lang="scss">
  main {
    display: flex;
    flex-direction: row;
    justify-content: space-around;
    align-items: center;
    height: 100vh;

    overflow: hidden; // Remove this if you want scrolling content
  }
</style>
