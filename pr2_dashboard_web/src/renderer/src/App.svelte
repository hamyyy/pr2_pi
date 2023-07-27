<script lang="ts">
  import BatteryCharging from '/battery-charging.svg'
  import BatteryFull from '/battery-full.svg'
  import BatteryDead from '/battery-dead.svg'
  import BatteryHalf from '/battery-half.svg'
  import { onDestroy, onMount } from 'svelte'
  import { setup_ros, shutdown_ros, batteryInfo } from './stores/ros.store'

  $: batteryCharge = $batteryInfo.averageCharge ?? -1
  $: batteries = $batteryInfo.battery ?? []

  onMount(() => {
    setup_ros()
  })

  onDestroy(() => {
    shutdown_ros()
  })
</script>

<main>
  <div>Battery Monitor</div>
  {#each batteries as batt}
    <div class="battery-container">
      <div class="battery-percentage">
        {batteryCharge}%
      </div>

      <div>
        <img
          class="battery"
          src={batt.charging
            ? BatteryCharging
            : batteryCharge > 95
            ? BatteryFull
            : batteryCharge > 10
            ? BatteryHalf
            : BatteryDead}
          alt=""
        />
      </div>
      <!-- <img class="battery" src={BatteryFull} alt="" />
    <img class="battery" src={BatteryDead} alt="" />
    <img class="battery" src={BatteryHalf} alt="" /> -->
    </div>
  {/each}
</main>

<style lang="scss">
  main {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    height: 100vh;
  }

  .battery-container {
    display: flex;
    flex-direction: row;
    justify-content: center;
    align-items: center;
    gap: 1rem;
  }

  .battery {
    width: 3rem;
    margin: 0 auto;
  }

  .battery-percentage {
    font-size: 2.5rem;
    text-align: center;
  }

  @media (prefers-color-scheme: dark) {
    .battery {
      filter: invert(1);
    }
  }
</style>
