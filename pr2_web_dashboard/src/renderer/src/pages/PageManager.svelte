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

<main>
  <div class="header">
    <ArrowButton direction="left" on:click={() => (prevPage = page--)} disabled={page == 0} />
    <h1>{pages[page].title}</h1>
    <ArrowButton
      direction="right"
      on:click={() => (prevPage = page++)}
      disabled={page == pages.length - 1}
    />
  </div>

  {#if pages.length > 0}
    <div class="content-container">
      {#key page}
        <div
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

<style lang="scss">
  main {
    flex: 1;

    height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
  }

  .header {
    display: flex;
    flex-direction: row;
    justify-content: space-between;
    align-items: center;

    width: 100%;

    border-bottom: 1px solid var(--text-color);
  }

  .content-container {
    position: relative;
    flex: 1;
    display: flex;
    flex-direction: row;
  }
</style>
