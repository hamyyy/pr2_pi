<script lang="ts">
  import BatteryCharging from "/battery-charging.svg";
  import BatteryFull from "/battery-full.svg";
  import BatteryDead from "/battery-dead.svg";
  import BatteryHalf from "/battery-half.svg";
  import { onDestroy, onMount } from "svelte";

  let batteryCharge = -1;
  let batteries = [];

  function updateCharge() {
    fetch("/battery-charge")
      .then((response) => response.json())
      .then((data) => {
        batteryCharge = data.average_charge ?? -1;
        batteries = data.battery ?? [];
        // console.log(data);
      })
      .catch((error) => {
        console.log(error);
      });
  }

  let intervalID = 0;
  onMount(() => {
    intervalID = setInterval(() => {
      updateCharge();
    }, 1000);
  });

  onDestroy(() => {
    clearInterval(intervalID);
  });
</script>

<main>
  {#each batteries as batt}
    <div class="battery-container">
      <div class="battery-percentage">
        {batteryCharge}%
      </div>

      <div>
        <img class="battery" src={batt.charging ? BatteryCharging : batteryCharge > 95 ? BatteryFull : batt.discharging ? BatteryHalf : BatteryDead} alt="" />
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
