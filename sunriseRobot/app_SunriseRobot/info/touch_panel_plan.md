# Touch panel plan

Onboard touchscreen to monitor the robot and switch things on/off without a PC, joystick or phone.
Status: plan only, screen not bought yet.

## Decisions

- **Screen**: 4.3" 800x480 HDMI + USB capacitive touch (the Raspberry Pi kind). Touch is plain USB HID, no driver.
- **Runs on the Jetson Nano**, not the RDK X3. Reasons: standard NVIDIA/Linux HDMI driver (the RDK X3 may not
  accept 800x480), and separate hardware, so the panel keeps working even when the whole RDK X3 hangs and can
  report "RDK X3 unreachable". The Jetson is switched on whenever the panel is needed.
- **Separate process**, own systemd service with `Restart=always`. Never a thread inside another program.
- **pygame**, drawing full screen. Goal: no desktop needed (see *Display without a desktop*).
- **No driving** of wheels or arm from the panel. Only status, settings, toggles and a STOP button.
- **The panel never touches robot hardware.** Every action goes through a program on the RDK X3 that owns it.
- Still a network client: the RDK X3 address is a config value, so the panel could later run on the RDK X3
  itself (pointing at `127.0.0.1`) without code changes.

## Architecture

```
 Jetson Nano                                RDK X3  (192.168.10.11, wired link only)
┌───────────────────┐   status 2 Hz / commands   ┌───────────────────────────────┐
│ panel (pygame)    │ ◀────────────────────────▶ │ main app: panel endpoint      │ :65436
│ systemd service   │                            │ (inside main_thread.py)       │
│                   │   system stats / recovery  ├───────────────────────────────┤
│                   │ ◀────────────────────────▶ │ supervisor (own systemd svc)  │ :65437
└───────────────────┘                            └───────────────────────────────┘
```

Two connections, so each side stays simple:

1. **Panel endpoint (in the main app, port 65436)**. Pushes a status message about twice a second and
   accepts commands. The status message is also the heartbeat: no message for ~3 s means "main app down".
   Commands go through a dedicated **whitelist** of functions (reusing the `FunctionCaller` dispatch idea),
   not through the voice function list.
2. **Supervisor (separate small service, port 65437)**. Knows nothing about the robot logic, so it survives
   when the main app crashes. It reports system stats and runs recovery actions.

Both bind **only to `192.168.10.11`** (the wired link, like the audio bridge), never to wlan0, the hotspot or
tailscale: the supervisor can reboot the robot.

Message framing: `robot_link.protocol` (already shared by both repos). The status message layout and the
command names go into `robot_link` too, so both sides agree on them in one place.

## What the panel shows

**Status page**
- link state of both connections (main app / supervisor / RDK X3 unreachable)
- mode, sub-mode, target, active controller, speed coefficient
- battery voltage
- RDK X3 and Jetson: CPU, RAM, temperature, disk
- network: IPs, hotspot on/off, Tailscale
- ROS2 on/off, lidar listener on/off, Jetson command link connected, voice session active

**Controls page**: next/previous mode, sub-mode, target, vision model; speed +/-; headlight on/off and
brightness; internal lights; fan; toggles for lidar, hotspot, ROS2.

**Threads page**: one row per main-app thread, green / yellow / red, with time since last activity and last
error message.

**System page**: last log lines of the main app; restart main app; restart ROS2; reboot RDK X3; shut down
everything cleanly (RDK X3 through the supervisor, then the Jetson itself).

**STOP**: always visible on every page. Sends a stop to the main app.

Touch targets about 10 mm minimum, so roughly 6-8 buttons per page, with tabs.

## Parts to build

Each step is useful on its own, in this order.

### 1. Main app under systemd (RDK X3) - implemented, not yet tested on the robot
- `robot_control.service` and `robot_fan.service` in `sunriseRobot/systemd/`, installed by
  `script_launchers/install_services.sh`. ROS2 processes are children of the main app (`ros2/ros2_processes.py`),
  no more gnome-terminal windows.
- Read logs with `journalctl -u robot_control`; the supervisor reads them the same way.

### 2. Thread health tracking (RDK X3, main app)
- Small registry: each thread loop records "last ran at" and "last exception". Exceptions caught at the
  top of each thread loop are recorded before being printed.
- Thresholds per thread (a camera loop beats often, a button listener rarely), so yellow/red is meaningful.

### 3. Panel endpoint (RDK X3, main app)
- New server in `ethernet_connection/`, config in `configs/panel_server.yaml` (host, port, status rate).
- Status message built from RobotHead / RobotBody state plus the thread registry.
- Command whitelist mapping names to RobotHead methods (`next_mode`, `toggle_lidar_listener`,
  `toggle_headlight`, ...).

### 4. Supervisor (RDK X3, separate service)
- Own systemd unit, own config file, very small, no robot imports beyond `robot_link`.
- Reports: CPU, RAM, temperature, disk, IPs, whether the main app unit is active.
- Actions: restart main app unit, restart ROS2, reboot, shutdown, return last N log lines.

### 5. Panel app (Jetson, `voice_robot_interaction` repo)
- pygame, full screen, config file with RDK X3 address, ports, page layout options.
- Reconnects forever on its own; every widget shows "unknown" rather than stale values when a link is down.
- Own systemd unit, started at boot.

### 6. Hardware
- Buy the screen, test HDMI and touch on the Jetson first.
- Power: check that the Jetson's USB can supply the screen (depends on how the Jetson is powered), otherwise
  feed it from the robot's 5 V rail.

## Open questions / checks

- **Motor board on crash**: if the main app dies while the robot moves, does the motor board keep the last
  speed? If yes, the supervisor could send a stop on the serial port, but **only after confirming the main
  app process is not running** (two programs on one serial port otherwise). Test before deciding.
- **Display without a desktop on the Jetson**: SDL's KMS/DRM backend on the Jetson's NVIDIA driver is not
  guaranteed to work. Fallback: a minimal X server started just for the panel (`xinit`, no desktop
  environment). Either way it does not depend on the RDK X3 no-GUI work.
- **Python version on the Jetson**: JetPack 4 ships Python 3.6. Check which pygame version installs there.
- Should the panel also show Jetson-side state (voice interaction, arm camera)? Cheap to add since it runs
  there.
