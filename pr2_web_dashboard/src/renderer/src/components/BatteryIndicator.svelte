<script lang="ts">
  import BatteryDead from '/battery-dead.svg'
  import BatteryCharge from '/battery-charge.svg'

  export let battery = null
  export let batteryBay = null

  $: charging = battery.charging ?? false
  $: charge = batteryBay.average_charge ?? -1

  $: alert = charge <= 10 && !charging

  $: divWidth = map(charge, 0, 100, 0, 26)

  function map(val, in_min, in_max, out_min, out_max) {
    return ((val - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min
  }
</script>

<main class={alert ? 'alert' : ''}>
  <div class="battery-percentage">{charge}%</div>
  <div class="icon-container">
    <img class="battery" src={BatteryDead} alt="" />
    <div class="battery-bar" style="width: {divWidth}px;" />
    {#if charging}
      <img class="battery-charge-outline" src={BatteryCharge} alt="" />
      <img class="battery-charge" src={BatteryCharge} alt="" />
    {/if}
  </div>
</main>

<style lang="scss">
  main {
    display: flex;
    flex-direction: row;
    justify-content: center;
    align-items: center;
    gap: 1rem;
  }

  .icon-container {
    position: relative;
    width: 2.5rem;
    margin: 0 auto;

    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;

    * {
      position: absolute;
      left: 0;
    }
  }

  .battery-bar {
    background-color: black;
    height: 12px;
    border-radius: 2px;
    margin-left: 5px;
  }

  .battery-percentage {
    font-size: 1.8rem;
    text-align: center;
  }

  .battery-charge {
    scale: 0.8;
    filter: invert(86%) sepia(49%) saturate(6435%) hue-rotate(353deg) brightness(105%) contrast(91%);
  }
  
  .battery-charge-outline {
    scale: 0.8;
    -webkit-filter: drop-shadow(1px 1px 0 var(--background-color))
      drop-shadow(-1px 1px 0 var(--background-color))
      drop-shadow(1px -1px 0 var(--background-color))
      drop-shadow(-1px -1px 0 var(--background-color));

    filter: drop-shadow(1px 1px 0 var(--background-color))
      drop-shadow(-1px 1px 0 var(--background-color))
      drop-shadow(1px -1px 0 var(--background-color))
      drop-shadow(-1px -1px 0 var(--background-color));
  }

  @media (prefers-color-scheme: dark) {
    .battery {
      filter: invert(1);
    }
    .battery-bar {
      filter: invert(1);
    }
  }

  main.alert {
    .battery-bar {
      filter: none;
      background-color: red;
    }
  }
</style>
