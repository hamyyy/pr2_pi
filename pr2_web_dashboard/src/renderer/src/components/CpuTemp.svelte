<script lang="ts">
  import { systemInfo } from '../stores/sys.store'

  $: temp = $systemInfo?.cpuTemperature ?? '???'
</script>

<h5>
  {temp} °C
</h5>
