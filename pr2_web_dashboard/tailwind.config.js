/** @type {import('tailwindcss').Config} */

const daisyui = require("daisyui")

module.exports = {
  content: [
    "./src/index.html",
    "./src/**/*.{js,ts,svelte}",
  ],
  plugins: [daisyui],
}

