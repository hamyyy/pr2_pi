<script lang="ts">
  import { runDemo, type ROSDemo, stopDemo } from '../stores/ros-demos.store'

  export let demo: ROSDemo

  console.log(demo);
  

  let color = 'btn-error'

  $: {
    switch (demo.state) {
      case 'running':
        color = 'btn-success'
        break

      case 'starting':
      case 'stopping':
        color = 'btn-warning'
        break

      case 'stopped':
      default:
        color = 'btn-error'
        break
    }
  }

  function onClick() {
    if (demo.state === 'stopped') {
      runDemo(demo)
    } else {
      stopDemo(demo)
    }
  }
</script>

<button class="w-full btn btn-lg {color} flex flex-row gap-4" on:click={onClick}>
  <h3>
    {demo.name}
  </h3>

  {#if demo.state === 'starting' || demo.state === 'stopping'}
    <span class="loading loading-spinner loading-lg" />
  {/if}
</button>
