<script lang="ts">
  import { onMount, onDestroy } from 'svelte'

  export let battery = null
  export let waiting = false
  export let debug = false

  $: charging = battery?.charging ?? false
  let checked = true
  let lostConnection = false
  let intervalID = null

  $: voltage = battery?.battery_register[0x9] / 1000
  $: current = battery?.battery_register[0xa] / 1000
  $: power = voltage * current
  $: charge = battery?.battery_register[0x0d] ?? -1
  $: rem_cap = battery?.battery_register[0x0f] / 1000
  $: tte_min = battery?.battery_register[0x12]
  $: ttf_min = battery?.battery_register[0x13]

  $: {
    battery
    checked = true

    if (debug) {
      console.log('Voltage: ' + voltage)
      console.log('Current: ' + current)
      console.log('Power: ' + power)
      console.log('Remaining Standby Capacity: ' + charge)
      console.log('Remaining Capacity: ' + rem_cap)
      console.log('Time to Empty (min): ' + tte_min)
      console.log('Time to Full (min): ' + ttf_min)
      console.log('')
    }
  }

  $: alert = charge <= 10 && !charging

  $: width = map(Math.max(Math.min(charge, 100), 0), 0, 100, 0, 336)

  function map(val, in_min, in_max, out_min, out_max) {
    return ((val - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min
  }

  function checkBattery() {
    clearInterval(intervalID)
    intervalID = setInterval(() => {
      if (checked) {
        checked = false
      } else {
        lostConnection = true
      }
    }, 5000)
  }

  onMount(() => {
    checkBattery()
  })

  onDestroy(() => {
    clearInterval(intervalID)
  })
</script>

<main class="flex flex-row justify-center items-center gap-4">
  {#if lostConnection || waiting || !battery.present}
    <div class="relative w-10 h-8 my-auto flex flex-col justify-center items-center">
      <svg class="absolute" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512">
        <rect
          x="31"
          y="144"
          width="400"
          height="224"
          rx="45.7"
          ry="45.7"
          fill="none"
          stroke={!alert ? 'currentColor' : 'hsl(var(--er))'}
          stroke-linecap="square"
          stroke-miterlimit="10"
          stroke-width="32"
        />
        <path
          fill="none"
          stroke={!alert ? 'currentColor' : 'hsl(var(--er))'}
          stroke-linecap="round"
          stroke-miterlimit="10"
          stroke-width="32"
          d="M479 218.67v74.66"
        />
      </svg>
    </div>
  {:else}
    <div class="text-3xl">{charge}%</div>
    <div class="relative w-10 h-8 my-auto flex flex-col justify-center items-center">
      <svg class="absolute" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512">
        <rect
          x="31"
          y="144"
          width="400"
          height="224"
          rx="45.7"
          ry="45.7"
          fill="none"
          stroke={!alert ? 'currentColor' : 'hsl(var(--er))'}
          stroke-linecap="square"
          stroke-miterlimit="10"
          stroke-width="32"
        />
        <path
          fill="none"
          stroke={!alert ? 'currentColor' : 'hsl(var(--er))'}
          stroke-linecap="round"
          stroke-miterlimit="10"
          stroke-width="32"
          d="M479 218.67v74.66"
        />
        <rect
          x="63"
          y="176"
          {width}
          height="160"
          rx="45.7"
          ry="45.7"
          fill={!alert ? 'currentColor' : 'hsl(var(--er))'}
          stroke="none"
          stroke-linecap="square"
          stroke-miterlimit="10"
          stroke-width="32"
        />
      </svg>
      {#if charging}
        <svg class="absolute" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512">
          <path
            stroke="none"
            fill="hsl(var(--wa))"
            d="M276.07 280.89l27.07-35.49a5.2 5.2 0 00.77-1.91 5 5 0 00.08-.66 5 5 0 00-.08-1.29 5.11 5.11 0 00-.68-1.75 4.76 4.76 0 00-.78-.95 3.48 3.48 0 00-.48-.38 4 4 0 00-1.11-.55 4.28 4.28 0 00-1.31-.2h-61.62l12.12-43.21 3.23-11.5 6.21-22.16.51-1.84 7.79-27.76a3.51 3.51 0 00.05-.55v-.16c0-.05 0-.26-.05-.38s0-.09 0-.14a2.2 2.2 0 00-.17-.45 3.77 3.77 0 00-.26-.39l-.09-.1a2.73 2.73 0 00-.25-.23l-.1-.08a3.14 3.14 0 00-.39-.24 2 2 0 00-.41-.14H265.53a2.3 2.3 0 00-.45 0 1.9 1.9 0 00-.42.15l-.13.07-.3.21-.11.1a2.4 2.4 0 00-.36.41l-18 23.63-13.14 17.22-9.85 12.83-63.71 83.55a5.72 5.72 0 00-.44.8 4.78 4.78 0 00-.35 1.09 4.7 4.7 0 00-.08 1.29 4.86 4.86 0 002 3.71 4.74 4.74 0 00.54.31 4.31 4.31 0 001.89.43h61.62L194.42 380.6a3.64 3.64 0 000 .56v.15a2.32 2.32 0 00.06.38.58.58 0 000 .14 2.2 2.2 0 00.17.45 3.62 3.62 0 00.26.38l.09.1.25.24a.39.39 0 01.1.08 2.22 2.22 0 00.39.23 2.83 2.83 0 00.41.14h.13a1.86 1.86 0 00.33 0h.13a2.32 2.32 0 00.45-.06 2.05 2.05 0 00.41-.16l.13-.07.3-.21.11-.09a2.4 2.4 0 00.36-.41L221.82 352l17.53-23z"
          />
        </svg>
      {/if}
    </div>
  {/if}
</main>
