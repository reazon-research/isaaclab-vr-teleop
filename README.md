# IsaacLab VR Teleoperation via TCP

This project enables teleoperation of robot manipulation tasks in **Isaac Lab** simulation by streaming VR hand joint pose data from a Meta Quest 3 headset via TCP.

---

## Overview

The system consists of:

- **Unity VR Application** (running on Meta Quest 3)   @WeirongShao0430 for developing the VR application.
  - Captures VR hand joint poses and finger distances.  
  - Sends JSON-formatted pose data over TCP to a PC.

- **Isaac Lab Teleoperation Interface** (running in Isaac Lab)  
  - Listens for incoming TCP data from the VR app.  
  - Parses joint pose data (position + quaternion rotation) for specific joints.  
  - Converts VR poses to robot end-effector delta poses for teleoperation.  
  - Applies gripper commands based on finger distance (open/close).

---

## Components

### 1. Unity VR Sender

- Sends data of selected hand joints (e.g., Right Hand Wrist, Bone ID 45) in ROS-compatible coordinate system.  
- Streams at ~20 Hz using TCP to the PC’s IP and port.

### 2. Isaac Lab TCP Receiver (`tcp_teleop.py`)

- Starts a TCP server listening on specified IP and port.  
- Continuously receives JSON data and extracts joint pose and finger distance.  
- Calculates delta position and rotation relative to the last frame for smooth control.  
- Maps delta pose and gripper command(if finger_distance < 0.03: gripper_closed = True, else: gripper_closed = False) into robot action tensors used by Isaac Lab environments.

### 3. The main script (`teleop_se3_agent_copy.py`)

- The main script to launch teleoperation with a chosen device(e.g. keyboard, in ouse case: VR via TCP using customized code)
- The task expect: 6-DoF delta {dx, dy, dz, drx, dry, drz} + gripper velocity The last value is: -1.0 → close the gripper +1.0 → open the gripper

## Setup

### On PC (Isaac Lab)

1. Place `tcp_teleop.py` and `teleop_se3_agent_copy.py` under `scripts/environments/teleoperation/`.  
2. Run Isaac Lab with:
    ```bash
    ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent_copy.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --num_envs 1 --teleop_device tcp
    ```

3. The teleop script listens for TCP connections on IP and port configured in `tcp_teleop.py` (default `0.0.0.0:5006`).

### On Meta Quest 3 (Unity)

- Run the Unity app that streams hand joint pose data to the PC’s IP and port (e.g., `192.168.xxx.xxx:5006`).  
- Make sure Quest and PC are on the same local network.

---

## How it works

- VR app sends joint pose and finger distance JSON messages over TCP.  
- Isaac Lab TCP teleop server receives the messages, extracts pose of the right hand wrist (joint 45), calculates the movement delta relative to last pose.  
- The movement delta is scaled and smoothed, converted to an action tensor.  
- Gripper open/close is controlled based on finger distance threshold.  
- These actions are fed into the Isaac Lab robot environment to control robot arm and gripper in real time.

---

## Notes

- Position and rotation deltas are scaled to match the robot workspace and provide smooth control.  
- Gripper threshold can be adjusted in the teleop receiver code.  
- The system currently maps only one joint (Right Hand Wrist) for simplicity but can be extended.

---

## Future Work

- Use rotation data of multiple joints for improved teleoperation fidelity.  
- Add support for dual-hand teleoperation.  
- Implement adaptive scaling or calibration between VR and robot workspace.
