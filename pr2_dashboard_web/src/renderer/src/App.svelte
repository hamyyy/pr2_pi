<script lang="ts">
  import BatteryCharging from '/battery-charging.svg'
  import BatteryFull from '/battery-full.svg'
  import BatteryDead from '/battery-dead.svg'
  import BatteryHalf from '/battery-half.svg'
  import { onDestroy, onMount } from 'svelte'
  import { setup_ros, shutdown_ros, batteryInfo } from './stores/ros.store'

  $: batteries = $batteryInfo

  onMount(() => {
    setup_ros()
  })

  onDestroy(() => {
    shutdown_ros()
  })
</script>

<main>
  <h3>Battery Monitor</h3>
  <div class="battery-server-container">
    {#each batteries as battery}
      <div class="battery-server">
        {#each battery.battery ?? [] as cell}
          <div class="battery-container">
            <div class="battery-percentage">
              {battery.average_charge ?? -1}%
            </div>

            <div>
              <img
                class="battery"
                src={cell.charging
                  ? BatteryCharging
                  : battery.average_charge > 95
                  ? BatteryFull
                  : battery.average_charge > 10
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
    height: 100vh;
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
