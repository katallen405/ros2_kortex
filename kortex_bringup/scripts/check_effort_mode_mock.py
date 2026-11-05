#!/usr/bin/env python3
"""Offline plumbing check for the Gen3 effort path on mock hardware (no arm):
launch gen3.launch.py with use_fake_hardware, switch JTC -> effort_controller,
stream torques, check they reach the effort interfaces (mock mirrors command
to state), stop streaming, check the watchdog fires and later commands no
longer get through, switch back.

  colcon build --packages-up-to kortex_bringup kortex_effort_controller
  source install/setup.bash
  python3 src/ros2_kortex/kortex_bringup/scripts/check_effort_mode_mock.py   # "MOCK CHECK PASS"

This exercises the controller, config and switching only -- mock hardware has
no TORQUE mode, so the hardware interface's effort code is not run (that's
what TESTING_EFFORT_MODE.md on the real arm is for)."""
import math
import os
import signal
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

launch = subprocess.Popen(
    ["ros2", "launch", "kortex_bringup", "gen3.launch.py", "robot_ip:=yyy.yyy.yyy.yyy",
     "use_fake_hardware:=true", "launch_rviz:=false", "effort_controller:=effort_controller"],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(("PASS " if ok else "FAIL ") + name + (f" -- {detail}" if detail else ""), flush=True)


try:
    rclpy.init()
    node = rclpy.create_node("mock_effort_check")
    latest = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: latest.update(js=m), 10)
    pub = node.create_publisher(Float64MultiArray, "/effort_controller/commands", 10)
    lister = node.create_client(ListControllers, "/controller_manager/list_controllers")
    switcher = node.create_client(SwitchController, "/controller_manager/switch_controller")

    def spin(sec, publish=None):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            if publish is not None:
                pub.publish(Float64MultiArray(data=publish))
            rclpy.spin_once(node, timeout_sec=0.01)

    def states():
        fut = lister.call_async(ListControllers.Request())
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        return {c.name: c.state for c in fut.result().controller}

    def switch(activate, deactivate):
        req = SwitchController.Request(activate_controllers=activate,
                                       deactivate_controllers=deactivate, strictness=2)
        fut = switcher.call_async(req)
        while not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result().ok

    assert lister.wait_for_service(timeout_sec=30.0), "controller_manager never came up"
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline and not (
            states().get("effort_controller") == "inactive"
            and states().get("joint_trajectory_controller") == "active"):
        spin(0.5)
    st = states()
    check("effort_controller loaded inactive", st.get("effort_controller") == "inactive", str(st))
    check("joint_trajectory_controller active", st.get("joint_trajectory_controller") == "active")

    torques = [0.5, -1.0, 1.5, -2.0, 0.25, -0.5, 0.75]
    spin(0.3, publish=torques)                     # stream before switching, as the adapter requires
    ok = switch(["effort_controller"], ["joint_trajectory_controller"])
    st = states()
    check("switch JTC -> effort", ok and st.get("effort_controller") == "active"
          and st.get("joint_trajectory_controller") == "inactive", str(st))

    spin(0.5, publish=torques)
    eff = list(latest["js"].effort) if "js" in latest else []
    names = list(latest["js"].name) if "js" in latest else []
    by_name = dict(zip(names, eff))
    got = [by_name.get(f"joint_{i + 1}") for i in range(7)]
    check("torques reach effort interfaces",
          all(g is not None and abs(g - t) < 1e-9 for g, t in zip(got, torques)), str(got))

    # Stop publishing: the watchdog commands NaN. mock_components'
    # GenericSystem doesn't mirror NaN commands to state, so the evidence is
    # the controller's own log line (checked from the launch output below)
    # and that later commands no longer get through.
    silence_started = time.monotonic()
    spin(0.5)
    resumed = [t + 3.0 for t in torques]
    spin(0.3, publish=resumed)                     # stream resumes with new values...
    by_name = dict(zip(latest["js"].name, latest["js"].effort))
    got = [by_name.get(f"joint_{i + 1}") for i in range(7)]
    check("stays stopped after a timeout until restarted (new values never applied)",
          all(abs(g - r) > 1e-6 for g, r in zip(got, resumed)), str(got))

    ok = switch(["joint_trajectory_controller"], ["effort_controller"])
    st = states()
    check("switch effort -> JTC", ok and st.get("joint_trajectory_controller") == "active", str(st))

    node.destroy_node()
    rclpy.shutdown()
finally:
    os.killpg(launch.pid, signal.SIGINT)     # the whole launch: ros2_control_node too
    try:
        out, _ = launch.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(launch.pid, signal.SIGKILL)
        out, _ = launch.communicate()
    errors = [ln for ln in out.splitlines() if "ERROR" in ln or "FATAL" in ln]
    print(f"launch log: {len(errors)} ERROR/FATAL lines")
    for ln in errors[:15]:
        print("  ", ln)
    check("watchdog fired (controller log: commanding NaN)",
          any("No torque command for more than 0.200s" in ln for ln in out.splitlines()))
print("MOCK CHECK", "PASS" if results and all(ok for _, ok in results) else "FAIL")
