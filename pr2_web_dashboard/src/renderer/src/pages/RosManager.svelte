<script lang="ts">
  import { rosInfo, shutdownROS } from '../stores/ros.store'
  const shell = window.api.shell

  function rosmaster() {
    if ($rosInfo.running) {
      shutdownROS()
      shell.exec('rosnode kill -a && pkill ros')
      rosInfo.set({
        running: false,
        teleop: false
      })
      shell.exec(`ssh pr2@c1 "pkill ros"`)
    } else {
      shell.exec(`ssh pr2@c1 -t "source ~/.profile && roslaunch ~/pr2_bringup.launch"`)

      setTimeout(() => {
        location.reload()
      }, 2000)
    }
  }

  function teleop() {
    if ($rosInfo.teleop) {
      shell.exec('rosnode kill /pr2_teleop_general_joystick')

      rosInfo.update((info) => {
        info.teleop = false
        return info
      })
    } else {
      shell.exec(
        `ssh pr2@c1  "source ~/.profile && roslaunch pr2_teleop_general pr2_teleop_general_joystick.launch"`
      )

      rosInfo.update((info) => {
        info.teleop = true
        return info
      })
    }
  }
</script>

<main>
  <h3>
    ROS Master
    <span class={$rosInfo.running ? 'text-success' : 'text-error'}>
      {$rosInfo.running ? 'Running' : 'Stopped'}
    </span>
  </h3>

  <div class="button-container flex-1">
    <button
      class="btn btn-lg {$rosInfo.running ? 'btn-error' : 'btn-success'}"
      on:click={rosmaster}
    >
      <h3>
        {$rosInfo.running ? 'Stop' : 'Start'} PR2
      </h3>
    </button>

    <button
      class="btn btn-lg {$rosInfo.teleop ? 'btn-error' : 'btn-success'} "
      on:click={teleop}
      disabled={!$rosInfo.running}
    >
      <h3>
        {$rosInfo.teleop ? 'Stop' : 'Start'} Teleop
      </h3>
    </button>

    <button class="btn btn-lg btn-outline" on:click={() => location.reload()}>
      <h3>Refresh App</h3>
    </button>
  </div>
</main>

<style lang="scss">
  main {
    flex: 1;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    align-items: center;

    padding-block: 2rem;
  }

  .button-container {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    gap: 1rem;

    button {
      font-size: 2rem;
      width: 100%;
    }
  }
</style>
