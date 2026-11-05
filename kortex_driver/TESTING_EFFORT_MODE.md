# Testing effort (joint torque) mode on a real Gen3

Branch `effort-torque-mode` of `katallen405/ros2_kortex`. This adds a
joint-torque mode to `KortexMultiInterfaceHardware`. You switch between it and
the normal `joint_trajectory_controller` (MoveIt) with `controller_manager`.
**None of the hardware-interface torque code has run on an arm yet.** This
checklist is the first time. Go in order, and stop at the first surprise.

## What changed (so you know what you're testing)

- **Entering torque mode.** Activating `effort_controller` runs the sequence
  that `gen3_torque_control` (a Python driver, AABL-Lab) validated on our arm:
  1. `ClearFaults`.
  2. Low-level servoing.
  3. 10 position-hold frames.
  4. `SetControlMode(TORQUE)` on actuators 1–7.
  5. 100 ms of zero-torque frames.

  This blocks the controller manager for about 0.7 s. Expect one overrun
  warning per switch.
- **Every torque frame:**
  - Commanded torque is clamped to `torque_limit_nm` (39 N·m on joints 1–4,
    9 N·m on joints 5–7).
  - Position mirrors the measured position, so the actuators'
    following-error guard still works.
- **Leaving torque mode.** Any of these puts the actuators back in POSITION
  mode with a single-level hold:
  - `effort_controller` stops getting commands for 0.2 s (it then sends NaN);
  - an actuator faults;
  - you deactivate the controller.

  After a watchdog or fault stop, the controller ignores further commands
  until it is restarted.
- **No gravity compensation in torque mode.** Gen3 torque mode doesn't
  compensate gravity. Whatever publishes `/effort_controller/commands` must
  include it (springcontroller does, with `add_gravity_compensation:=true`).
  **A zero command lets the arm fall.**

## Setup

```bash
export ROS_LOCALHOST_ONLY=1          # E-STOP kills the link; see gen3_torque_control README
colcon build --packages-up-to kortex_bringup kortex_effort_controller
source install/setup.bash
# offline sanity check first (no arm):
python3 src/ros2_kortex/kortex_bringup/scripts/check_effort_mode_mock.py   # MOCK CHECK PASS
```

Launch on the arm:

```bash
ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10 \
  gripper:=robotiq_2f_85 effort_controller:=effort_controller launch_rviz:=false
ros2 control list_controllers      # effort_controller: inactive, joint_trajectory_controller: active
```

Switch in and out:

```bash
# into torque mode (commands must already be streaming -- see step 2)
ros2 control switch_controllers --activate effort_controller --deactivate joint_trajectory_controller
# back to position control / MoveIt
ros2 control switch_controllers --activate joint_trajectory_controller --deactivate effort_controller
```

## Checklist

Keep one hand on the E-STOP for every step. Run each torque step from the
"candle" pose (arm straight up), where gravity torques are smallest.

1. **Regression, no torque.** With `effort_controller` left inactive, run a
   MoveIt plan as you normally would. Expected: it behaves exactly as on
   upstream `main`.

2. **Gravity only.** Start springcontroller's Gen3 launch with
   `driver:=ros2_kortex` (no springs, gravity compensation on). It publishes
   on `/effort_controller/commands` continuously. Then switch to
   `effort_controller`.
   - Expected: the arm floats and can be pushed around. It should not sag,
     jump or buzz at the switch.
   - Record the switch duration and any `controller_manager` overrun
     warnings.

3. **Torque sign.**
   - With gravity-only running, add +1 N·m on joint 7 only, by adding a
     joint spring or editing the published torque.
   - Check that joint 7 turns in the positive direction of its URDF axis.
   - Watch `ros2 topic echo /joint_states --field effort`. Joint 7's effort
     should read about +1 N·m more than before.
   - If it reads about −1 instead, set `effort_state_sign` to `-1.0` in
     `kortex_description/arms/gen3/7dof/urdf/kortex.ros2_control.xacro`
     (macro default) and rebuild. gen3_torque_control negates measured torque
     for this reason; the default here deliberately leaves upstream behaviour
     unchanged.
   - If the *command* direction is wrong, stop and report it. Don't patch
     around it.

4. **Watchdog.** While floating, kill the torque publisher (Ctrl-C
   springcontroller).
   - Expected: within about 0.2 s the arm goes to a position hold, and the
     log shows "No torque command for more than 0.200s" (controller) and
     "Effort control stopped: effort command is NaN" (hardware).
   - Restart the publisher. The arm must *not* resume torque control until
     you deactivate and re-activate `effort_controller`.

5. **Fault / E-STOP.** While floating, press the E-STOP.
   - Expected: the hardware leaves torque mode.
   - After recovery and a relaunch, the actuators come up in POSITION mode
     (`on_init` resets them), and MoveIt works.

6. **Repeated switching.** Switch effort ↔ joint_trajectory_controller 20
   times, running a short MoveIt plan after each switch back. Expected: no
   faults, no jumps.

7. **Report back:**
   - switch latency;
   - overrun warnings;
   - the sign result from step 3;
   - anything that differed from "Expected".

## Known limitations

- `torque_limit_nm` and `effort_state_sign` are xacro macro arguments with
  defaults. They are not yet plumbed through `gen3.xacro` / the launch
  files, so change the defaults in `kortex.ros2_control.xacro` for now.
- Torque mode is all seven joints or nothing. It is exclusive with joint
  position/velocity and twist control.
- `picknik_twist_controller` / `picknik_reset_fault_controller` must be
  installed for the stock launch's twist and fault controllers. That's
  unrelated to this change, but their spawners fail without them.
