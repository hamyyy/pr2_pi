<script lang="ts">
  import RosButton from '../components/ROSButton.svelte'
  import { demos, scanForDemos } from '../stores/ros-demos.store'
</script>

<main class="flex flex-col justify-start items-start h-full w-full p-4">
  <button class="btn btn-block btn-outline" on:click={() => scanForDemos()}>Refresh</button>

  <br />

  <div class="flex flex-row w-full justify-around items-center">
    <h2>C1 Demos</h2>
    <h2>C2 Demos</h2>
  </div>

  <br />

  <div class="flex flex-row w-full gap-4">
    <div class="flex-1 flex flex-col justify-center items-center gap-2">
      {#each $demos as demo}
        {#if demo.machine === 'c1'}
          <RosButton bind:demo />
        {/if}
      {/each}
    </div>

    <div class="flex-1 flex flex-col justify-center items-center gap-2">
      {#each $demos as demo}
        {#if demo.machine === 'c2'}
          <RosButton bind:demo />
        {/if}
      {/each}
    </div>
  </div>
</main>
