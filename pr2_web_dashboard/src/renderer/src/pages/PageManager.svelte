<script lang="ts">
  export let page: number = 0
  export let pages

  $: page = Math.max(0, Math.min(page, pages.length - 1))
</script>

<main>
  {#if pages.length > 0}
    <svelte:component this={pages[page]} />
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
  }
</style>
