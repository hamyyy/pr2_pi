<script lang="ts">
  import BatteryIndicator from '../components/BatteryIndicator.svelte'
  import { batteryInfo } from '../stores/ros.store'

  $: batteryBays = $batteryInfo
</script>

<main>
  <div class="battery-server-container">
    {#each batteryBays as batteryBay}
      <div class="battery-server">
        {#each batteryBay.battery ?? [] as battery}
          <div class="battery-container">
            <BatteryIndicator {battery} {batteryBay} />
          </div>
        {/each}
      </div>
    {/each}
  </div>
</main>

<style lang="scss">
  main {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    height: 100%;
  }

  .battery-server {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    gap: 1rem;
  }

  .battery-server-container {
    display: flex;
    flex-direction: row;
    justify-content: center;
    align-items: center;
    gap: 3rem;
  }

  .battery-container {
    display: flex;
    flex-direction: row;
    justify-content: center;
    align-items: center;
    gap: 1rem;
  }
</style>
