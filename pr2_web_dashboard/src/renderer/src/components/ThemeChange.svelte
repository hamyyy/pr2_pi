<script>
  import { onMount } from 'svelte'
  import { themeChange } from 'theme-change'

  let currentTheme = localStorage.getItem('theme')

  onMount(() => {
    if (localStorage.getItem('theme') === null) {
      currentTheme = 'light'
      document.documentElement.setAttribute('data-theme', currentTheme)
    } else document.documentElement.setAttribute('data-theme', localStorage.getItem('theme'))

    themeChange(false)
  })
</script>

<main>
  <label class="label cursor-pointer gap-2">
    <span class="label-text">Dark Mode</span>
    <input
      type="checkbox"
      class="toggle toggle-success my-auto"
      data-toggle-theme="light,dark"
      data-act-class="ACTIVECLASS"
      checked={currentTheme === 'dark'}
    />
  </label>
</main>
