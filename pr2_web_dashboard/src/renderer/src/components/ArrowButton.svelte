<script lang="ts">
  export let direction: 'left' | 'right'
  export let disabled = false
</script>

<button class="btn btn-ghost h-full flex items-center" on:click {disabled}>
  {#if direction === 'right'}
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" width="3rem">
      <path
        fill="none"
        stroke="currentColor"
        stroke-linecap="round"
        stroke-linejoin="round"
        stroke-width="48"
        d="M268 112l144 144-144 144M392 256H100"
      />
    </svg>
  {:else}
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" width="3rem">
      <path
        fill="none"
        stroke="currentColor"
        stroke-linecap="round"
        stroke-linejoin="round"
        stroke-width="48"
        d="M244 400L100 256l144-144M120 256h292"
      />
    </svg>
  {/if}
  <!-- <img class={disabled ? 'disabled' : ''} src={arrowSrc} alt="" /> -->
</button>


<style>
  button:disabled {
    background-color: transparent;
  }
</style>