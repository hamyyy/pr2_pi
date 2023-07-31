<script lang="ts">
  import { setup_ros, shutdown_ros } from './stores/ros.store'
  import { onDestroy, onMount } from 'svelte'
  import ArrowButton from './components/ArrowButton.svelte'

  import PageManager from './pages/PageManager.svelte'
  import BatteryMonitor from './pages/BatteryMonitor.svelte'
  import RosManager from './pages/RosManager.svelte'

  onMount(() => {
    setup_ros()
  })

  onDestroy(() => {
    shutdown_ros()
  })

  let page = 0
  let pages = [RosManager, BatteryMonitor]
</script>

<main>
  <ArrowButton direction="left" on:click={() => page--} disabled={page == 0} />
  <PageManager {page} {pages} />
  <ArrowButton direction="right" on:click={() => page++} disabled={page == pages.length - 1} />
</main>

<style>
  main {
    display: flex;
    flex-direction: row;
    justify-content: space-around;
    align-items: center;
    height: 100vh;
  }
</style>
