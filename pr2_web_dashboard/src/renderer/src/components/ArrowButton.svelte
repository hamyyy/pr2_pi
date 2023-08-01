<script lang="ts">
  import LeftArrow from '/arrow-back.svg'
  import RightArrow from '/arrow-forward.svg'

  export let direction: 'left' | 'right'
  export let disabled = false
  let arrowSrc = direction === 'left' ? LeftArrow : RightArrow
</script>

<main>
  <button class="arrow-button" on:click {disabled}>
    <img class={disabled ? 'disabled' : ''} src={arrowSrc} alt="" />
  </button>
</main>

<style lang="scss">
  button {
    background-color: transparent;
    border-color: transparent;
    outline: transparent;
    // height: 40vh;
  }

  img {
    width: 3rem;
    pointer-events: none;

    &.disabled {
      filter: opacity(0.5);
    }
  }

  @media (prefers-color-scheme: dark) {
    img {
      filter: invert(1);
    }
  }
</style>
