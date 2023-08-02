<script lang="ts">
  import { expoOut } from 'svelte/easing'
  import ArrowButton from '../components/ArrowButton.svelte'

  export let page: number = 0
  export let pages

  let prevPage = page

  $: page = Math.max(0, Math.min(page, pages.length - 1))

  function swipe(_node: HTMLElement, { page, prevPage, direction }: any) {
    return {
      duration: 300,
      easing: expoOut,
      css: (_t, u) => {
        if (direction === 'out') {
          return `
          position: absolute;
          height: 100%;
          width: 100%;
          transform: translateX(${(page - prevPage) * -u * 100}vw);`
        }
        return `transform: translateX(${(page - prevPage) * u * 100}vw);`
      }
    }
  }
</script>

<main class="overflow-hidden h-screen flex-1 flex flex-col">
  <div class="flex flex-row justify-between items-center w-full border-b border-b-gray-500">
    <ArrowButton direction="left" on:click={() => (prevPage = page--)} disabled={page == 0} />
    <h1>{pages[page].title}</h1>
    <ArrowButton
      direction="right"
      on:click={() => (prevPage = page++)}
      disabled={page == pages.length - 1}
    />
  </div>

  {#if pages.length > 0}
    <div class="flex flex-row justify-center relative flex-grow w-full overflow-hidden overflow-y-auto">
      {#key page}
        <div
          class="flex w-full"
          in:swipe={{ page, prevPage, direction: 'in' }}
          out:swipe={{ page, prevPage, direction: 'out' }}
        >
          <svelte:component this={pages[page].component} />
        </div>
      {/key}
    </div>
  {:else}
    <div style="flex: 1; display: flex; justify-content: center; align-items: center;">
      <h1>No pages found</h1>
    </div>
  {/if}
</main>
