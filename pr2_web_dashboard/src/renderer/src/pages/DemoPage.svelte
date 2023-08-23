<script lang="ts">
  import RosButton from '../components/ROSButton.svelte'
  import { demos, scanForDemos } from '../stores/ros-demos.store'

  let howToUseModal = false
  function toggleHowToUse() {
    howToUseModal = !howToUseModal
  }
</script>

<main class="flex flex-col justify-start items-start h-full w-full p-4">
  <div class="w-full flex flex-row justify-center items-center gap-2">
    <button class="btn flex-[0.80] btn-outline" on:click={() => scanForDemos()}>Refresh</button>
    <button class="btn flex-[0.20] btn-info" on:click={() => toggleHowToUse()}>How To Use</button>
  </div>

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

{#if howToUseModal}
  <main
    class="fixed top-0 left-0 w-screen h-screen bg-base-100 bg-opacity-70 flex justify-center items-center p-20"
    on:click|self={() => toggleHowToUse()}
    on:keydown|self={() => toggleHowToUse()}
  >
    <div class="relative bg-base-100 shadow-lg shadow-black rounded-lg p-4">
      <div class="flex justify-between">
        <h3 class="">How to Add a Demo (Script)</h3>
        <button
          class="absolute top-2 right-2 btn btn-sm btn-circle btn-outline"
          on:click={() => toggleHowToUse()}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            class="h-6 w-6"
            fill="none"
            viewBox="0 0 24 24"
            stroke="currentColor"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              stroke-width="2"
              d="M6 18L18 6M6 6l12 12"
            />
          </svg>
        </button>
      </div>

      <hr />
      <p class="text-lg">
        Simply place a <code class="bg-base-200 text-base-content px-2 py-1 rounded">.launch</code>
        file to
        <code class="bg-base-200 text-base-content px-2 py-1 rounded">~/pr2pi_shared/</code>
        on either C1 or C2 (you may need to ssh into C1 or C2)
      </p>

      <br />

      <p class="text-lg">Refresh if needed</p>
      <p class="text-lg font-bold">Do not delete other files!</p>

      <br />

      <p class="text-lg">
        A <span class="text-error">red</span> button means the demo is not running
      </p>
      <p class="text-lg">
        A <span class="text-success">green</span> button means the demo is running
      </p>
    </div>
  </main>
{/if}
