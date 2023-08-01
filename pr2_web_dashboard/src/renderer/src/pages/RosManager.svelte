<script lang="ts">
  import { rosInfo, shutdown_ros } from '../stores/ros.store'
  const shell = window.api.shell

  function rosmaster() {
    if ($rosInfo.running) {
      shutdown_ros()
      shell.exec('rosnode kill -a && pkill ros')
      rosInfo.set({
        running: false,
        teleop: false
      })
      shell.exec(`ssh pr2@10.68.0.1 "pkill ros"`)
    } else {
      shell.exec(
        `ssh pr2@10.68.0.1 "source /opt/ros/noetic/setup.bash && source /home/pr2/pr2_ws/devel/setup.bash && export ROS_ENV_LOADER=/opt/ros/noetic/env.sh && roslaunch ~/pr2_bringup.launch"`
      )
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
        `ssh pr2@10.68.0.1 "source /opt/ros/noetic/setup.bash && source /home/pr2/pr2_ws/devel/setup.bash && export ROS_ENV_LOADER=/opt/ros/noetic/env.sh && roslaunch pr2_teleop_general pr2_teleop_general_joystick.launch"`
      )

      rosInfo.update((info) => {
        info.teleop = true
        return info
      })
    }
  }
</script>

<main>
  <h2>
    ROS Master <span
      style="color: {$rosInfo.running ? 'var(--success-color)' : 'var(--danger-color)'}"
      >{$rosInfo.running ? 'Running' : 'Stopped'}</span
    >
  </h2>

  <div style="flex: 1" />

  <div class="button-container">
    <button on:click={() => location.reload()}>Refresh</button>
    <button class={$rosInfo.running ? 'danger' : 'success'} on:click={rosmaster}
      >{$rosInfo.running ? 'Stop' : 'Start'} ROS Master</button
    >
    <button
      class={$rosInfo.teleop ? 'warning' : 'success'}
      on:click={teleop}
      disabled={!$rosInfo.running}>{$rosInfo.teleop ? 'Stop' : 'Start'} Teleop</button
    >
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
