<script lang="ts">
  import { systemInfo } from '../stores/sys.store'

  $: load = Math.floor($systemInfo?.cpuLoad) || '???'
</script>

<h5>
  {load}%
</h5>
