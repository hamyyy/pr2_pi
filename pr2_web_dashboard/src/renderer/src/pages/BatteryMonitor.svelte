<script lang="ts">
  import BatteryIndicator from '../components/BatteryIndicator.svelte'
  import { batteryInfo } from '../stores/ros.store'

  const positions = ['Top Right', 'Bottom Right', 'Top Left', 'Bottom Left']

  $: batteryBays = $batteryInfo
</script>

<main class="flex flex-col justify-center items-center h-full w-full py-8 px-8">
  <div class="flex flex-row justify-center items-center gap-16 w-full">
    {#each batteryBays as batteryBay, i}
      <div class="flex flex-col justify-center items-center gap-1 flex-1">
        <h5 class="mb-4 whitespace-nowrap">{positions[i]}</h5>
        {#each batteryBay.battery ?? [false, false, false, false] as battery}
          {#if battery === false}
            <BatteryIndicator waiting />
          {:else}
            <div class="battery-container">
              <BatteryIndicator {battery} />
            </div>
          {/if}
        {/each}
        <h5 class="whitespace-nowrap">Average: {batteryBay?.average_charge ?? '???'}%</h5>
      </div>
    {/each}
  </div>
</main>
