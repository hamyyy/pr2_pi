<script lang="ts">
  import { onMount, onDestroy } from 'svelte'
  import { LogLevel, logs } from '../stores/ros.store'

  let containerElmt: HTMLDivElement

  let unsub: () => void
  onMount(() => {
    unsub = logs.subscribe(() => {
      if (containerElmt) {
        setTimeout(() => {
          containerElmt.scrollTop = containerElmt.scrollHeight
        })
      }
    })
  })

  onDestroy(() => {
    unsub()
  })
</script>

<main class="w-full h-full p-4">
  <div
    bind:this={containerElmt}
    class="w-full h-fit max-h-full overflow-auto py-4 rounded-lg bg-base-200 flex flex-col"
  >
    {#each $logs as line, i}
      <div
        class="px-4 {line.level === LogLevel.ERROR || line.level === LogLevel.FATAL
          ? 'text-error'
          : line.level === LogLevel.DEBUG
          ? 'text-info'
          : line.level === LogLevel.WARN
          ? 'text-warning'
          : ''} {i === $logs.length - 1 ? 'bg-base-300' : ''}"
      >
        {line.message}
      </div>
    {/each}
  </div>
</main>
