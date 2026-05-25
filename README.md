# dual_arm_teleop_ws

Dual-arm teleoperation in RViz: a single operator's hands (if they have two hands)
drive two 6-DOF UR5e arms through MoveIt Servo, with noise filtering and gripper-open/close control.

The operator gets 2D end-effector translation + 1D wrist yaw per arm (one camera =
no depth, so motion is in a vertical plane in front of each arm). Hand tracking
runs on Windows because WSL2 cannot pass through a USB webcam and tracking output is
streamed to the ROS2 workspace over TCP.

## Demo video

demo_video.mp4

## Architecture

The system consists of two machines connected over TCP. On the Windows side, 
camera_bridge.py (running in a Python 3.10 venv) captures the webcam, 
runs MediaPipe Hands, and shows an OpenCV preview with landmark overlays 
and the arm-slot each hand is routed to. It serializes per-frame hand state 
— wrist position, rotation, and pinch distance for each hand — as newline-delimited 
JSON and streams it over a TCP socket at ~30 Hz.

On the WSL2 / Linux (ROS 2 Humble) side, teleop_node.py runs against the system rclpy (no venv). 
Each tick it reads the latest hand state from the socket, applies a One-Euro filter per channel, 
maps the filtered coordinates into each arm's workspace, and runs a closed-loop P-controller that 
turns the resulting pose error into a velocity command. It publishes a TwistStamped at 50 Hz (as 
per the requiremenrts) on each arm's delta_twist_cmds topic, plus a std_msgs/Float64 gripper command. 
Two MoveIt Servo instances (left_servo_node, right_servo_node) convert those twists into joint commands 
via a JointGroupPositionController, updating joint_states and driving the arms in RViz. demo.launch.py 
brings up MoveIt and RViz and servo.launch.py starts the two servo nodes.

Hands are routed to arms by **image position** in the (mirrored) preview, not by
MediaPipe's `Left`/`Right` label — that label is unreliable across webcams.
The operator's right hand sits on the right of the selfie-view frame, gets routed
to the right arm. Each arm runs a closed-loop P-controller in the node:
`twist = K_p * (target_pose − current_pose_via_TF)`, clipped to safe limits, then
published on `delta_twist_cmds` which Servo then turns into joint commands.

`pose_target_cmds` is not subscribed by `servo_node_main` in MoveIt2 Humble
(verified via `ros2 node info`), which is why the P-controller lives in the node
rather than just publishing pose targets.

## Tested environment

| Component        | Version                              |
| ---------------- | ------------------------------------ |
| WSL2 distro      | Ubuntu 22.04                         |
| ROS              | ROS 2 Humble Hawksbill               |
| MoveIt           | MoveIt 2 Humble                      |
| Host OS (bridge) | Windows 11                           |
| Python (bridge)  | 3.10 (matching MediaPipe wheel)      |
| Logitech C270 HD | Front-facing laptop webcam (non-mirrored output) |

## Setup — fresh machine

### 1. WSL2 / Linux side

Install ROS 2 Humble (desktop) following
<https://docs.ros.org/en/humble/Installation.html>, then:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-ur-description \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-joint-state-broadcaster \
  python3-colcon-common-extensions

# Clone and build
git clone <this-repo-url> ~/dual_arm_teleop_ws
cd ~/dual_arm_teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The `teleop_node.py` runs against the **system** `rclpy`, not a venv. Pytest
dependencies (only needed to run tests): `python3 -m pip install pytest`.

### 2. Windows side — MediaPipe bridge

```powershell
# Copy or clone the windows_bridge/ directory to a Windows machine, e.g.:
# (in my case)  C:\Users\<you>\dev\dual_arm_bridge\ 
cd C:\Users\<you>\dev\dual_arm_bridge
py -3.10 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

`requirements.txt`: `mediapipe==0.10.14`, `opencv-python`. The 0.10.14 pin matters
as newer MediaPipe versions drop or restructure the legacy `mp.solutions.hands` API.

### 3. Networking — finding the bridge IP

The teleop node connects to the Windows bridge over TCP. From WSL the Windows
host is reachable on the **vEthernet (WSL) adapter IP**, *not* the nameserver in
`/etc/resolv.conf`:

```bash
ip route show | grep default
# e.g.   default via 172.22.128.1 dev eth0 proto kernel ...
```

That third field is the host IP. Update `BRIDGE_HOST` at the top of
`scripts/teleop_node.py` if it differs from the default `172.22.128.1`.

## Run

Start everything in this order. Each WSL terminal must first source:
`source install/setup.bash`.

```powershell
# Windows PowerShell — start the bridge first and the preview window will  come up
cd C:\Users\<you>\dev\dual_arm_bridge
.venv\Scripts\python.exe camera_bridge.py
```

Confirm the preview window opens and shows hand landmarks with `LEFT` / `RIGHT`
labels under each detected wrist.

```bash
# WSL terminal A — MoveIt + RViz, puts both arms at the home pose
ros2 launch dual_arm_moveit_config demo.launch.py

# WSL terminal B — Servo nodes
ros2 launch dual_arm_moveit_config servo.launch.py

# WSL terminal C — switch controllers and start the servo loops
./bringup_servo.sh

# WSL terminal D — teleop node, connects to the bridge over TCP
python3 scripts/teleop_node.py
```

`teleop_node.py` will log `connected to bridge`. The bridge preview's status bar
flips to `STREAMING`. Move your hands; arms move.

`ESC` quits the bridge cleanly; `Ctrl-C` exits the teleop node.

### Verifying the gripper

```bash
ros2 topic echo /left_gripper_cmd
ros2 topic echo /right_gripper_cmd
```

Pinch thumb to index → values climb toward 1.0. Spread → fall toward 0.0.
Currently there are no gripper meshes in the URDF, so this is the only
observable signal — see Known limitations below.

### Running the tests

```bash
cd ~/dual_arm_teleop_ws/scripts
python3 -m pytest tests/
```

The tests cover the mapper (workspace mapping, pinch normalization, L/R
crosstalk) and the One-Euro filter (steady-state, ramp tracking, edge cases).

## Tuning

Most knobs are grouped at the top of `scripts/teleop_node.py`:

| Constant            | Default | Effect                                              |
| ------------------- | ------- | --------------------------------------------------- |
| `REACH_FORWARD`     | 0.40    | Forward depth (m) of the planar workspace from base |
| `KP_LIN` / `KP_ANG` | 4.0     | Proportional gain on pose error                     |
| `MAX_LIN`           | 0.50    | Linear twist magnitude clip (m/s)                   |
| `MAX_ANG`           | 1.00    | Angular twist magnitude clip (rad/s)                |
| `FILTER_MIN_CUTOFF` | 1.0     | One-Euro baseline cutoff — lower = smoother at rest |
| `FILTER_BETA`       | 0.05    | One-Euro speed coefficient — higher = less lag      |

Workspace bounds live in `scripts/teleop_mapper.py`:

| Constant       | Default     | Effect                                  |
| -------------- | ----------- | --------------------------------------- |
| `XMIN`, `XMAX` | -0.5, 0.5   | Horizontal workspace (m, base-link y)   |
| `YMIN`, `YMAX` | 0.15, 0.70  | Vertical workspace (m, base-link z)     |

If the bridge preview shows your hands mirrored differently than expected
(e.g., your webcam already streams in selfie-view), comment out the
`cv2.flip(frame, 1)` line in `windows_bridge/camera_bridge.py`.

## Project structure

```
dual_arm_teleop_ws/
├── bringup_servo.sh                # spawn position controllers, switch + start servo
├── windows_bridge/                 # Windows-side MediaPipe + TCP server
│   ├── camera_bridge.py
│   ├── probe_camera.py             # standalone debug preview
│   └── requirements.txt
├── scripts/                        # WSL-side teleop node + helpers
│   ├── teleop_node.py              # main ROS node (P-controller, TF, twist + gripper pub)
│   ├── teleop_mapper.py            # HandsState -> TeleopCommand in workspace coords
│   ├── mapper_contract.py          # dataclasses (HandObservation, ArmTarget, ArmMode...)
│   ├── one_euro.py                 # One-Euro low-pass filter
│   └── tests/                      # pytest unit tests
└── src/
    ├── dual_arm_description/       # URDF (xacro) for two UR5e on a shared base plate
    └── dual_arm_moveit_config/     # SRDF, MoveIt + Servo configs, launches
```

## Known limitations

- **No gripper visualization.** The URDF places two UR5e arms on a base plate
  but does not attach gripper meshes (Robotiq 2F-85 was the intended choice but
  was not integrated in time). The gripper command is still published as
  `std_msgs/Float64` on `/<side>_gripper_cmd` in the range `[0, 1]`
  (`0 = open`, `1 = closed`) and is observable via `ros2 topic echo`. A real
  Robotiq controller subscribing to this topic would work; the visualization
  gap is purely cosmetic.
- **Monocular hand tracking.** A single 2D webcam cannot reliably measure palm
  twist (the palm just gets thinner in the image with no signed direction). The
  "1D rotation" axis is therefore the angle of the wrist→middle-finger vector
  in the image plane, not a palm-twist gesture. Rotate the whole hand around
  the wrist to drive EE yaw.
- **Workspace is a vertical plane.** With no depth, the EE moves in a y–z plane
  at a fixed forward distance (`REACH_FORWARD`) from each arm's base.
- **Wrist-singularity trade-off.** The home pose sits near a UR wrist-2
  singularity. To keep translation range usable, only yaw (z-axis) is
  closed-loop in orientation; the EE roll/pitch drifts to whatever IK gives it.
  This keeps the workspace large at the cost of holding tool-z exactly down.
- **Bridge IP can change** across Windows reboots — see Networking above.
- **Webcam mirror behavior varies.** The bridge applies `cv2.flip(frame, 1)`
  to enter MediaPipe's expected selfie view. Some webcams already stream
  mirrored; remove the flip if your preview shows you "backwards".
- **No auto-recovery from arm drift.** If the arms have been moved out of the
  configured home pose (e.g., by a previous teleop session or an RViz goal),
  reset by restarting `demo.launch.py` before the next teleop run.

## Attributions and references

- **[Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)**
  (`ur_description` apt package) — UR5e URDF/meshes, joint limits, kinematics.
  Used unmodified; included as a runtime dependency.
- **[MoveIt 2](https://moveit.ros.org/)** — used for `moveit_servo`
  (`servo_node_main`) and the standard MoveIt config layout. The two servo
  instances run side-by-side as separate nodes (one per arm group), each with
  their own planning context.
- **[MediaPipe Hands](https://developers.google.com/mediapipe/solutions/vision/hand_landmarker)**
  (Google) — hand landmark detection on the Windows bridge. Pinned to 0.10.14
  for API stability.
- **One-Euro filter** — Géry Casiez, Nicolas Roussel, Daniel Vogel,
  *"1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive
  Systems"*, CHI 2012. Implemented in `scripts/one_euro.py`, with equations following the paper directly. 
- **OpenCV** — used for the camera capture and the preview window on the
  Windows bridge.
- **[Claude Code](https://claude.com/product/claude-code)** — used as a
  pair-programming assistant during development, particularly for debugging
  workspace setup as well as optimizing tests.
- **Claude** - used to cover personal domain knowledge gaps, and for planning and researching 
  before making decisions regarding the task.
