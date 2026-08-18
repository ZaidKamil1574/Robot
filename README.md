# da Vinci PSM Simulator (Unity + URDF + ROS)

A Unity simulation of Intuitive Surgical's da Vinci **Patient-Side Manipulator (PSM)**, built from the real open-source kinematic model published by Johns Hopkins' [dVRK (da Vinci Research Kit)](https://github.com/jhu-dvrk/dvrk_model) project with the actual link geometry, joint limits, and parallelogram RCM linkage from a research-grade da Vinci description.

The robot can be driven two ways: jogging individual joints directly, or commanding the tool tip in Cartesian space via a custom inverse-kinematics solver — the same paradigm real PSM teleoperation uses.

## Why this exists

Built as a hands on prototype to explore da Vinci-style surgical robot kinematics in Unity

## Controls

Press **Tab** to switch modes.

| Mode | Keys | Behavior |
|---|---|---|
| Cartesian IK (default) | W/A/S/D/Q/E | Move the tool tip in world X/Y/Z; IK solves the arm |
| | Arrows, Z/X | Wrist yaw/pitch, roll |
| Joint Space | W/S, A/D, Q/E | insertion, yaw, pitch |
| | Arrows, Z/X | wrist_yaw, wrist_pitch, roll |
| Both modes | C/V | Close/open gripper |

<img width="1061" height="630" alt="image" src="https://github.com/user-attachments/assets/f4f697a5-466f-438a-bb15-869b878870b5" />

## Kinematics

- **Joint chain**: `yaw → pitch → insertion → roll → wrist_pitch → wrist_yaw → tool_tip`, matching the real PSM's 6-DOF structure (plus a 7th gripper DOF).
- **Forward kinematics** (`DaVinciKinematics.ForwardKinematics`): explicit `Matrix4x4`/`Quaternion` homogeneous-transform composition through each joint's *actual* anchor frames (`ArticulationBody.anchorPosition/Rotation`), not hand-derived from raw URDF text — reuses the axis conventions Unity's importer already resolved correctly.
- **Inverse kinematics** (`DaVinciKinematics.SolveIK`): a numerical Jacobian-transpose solver. Each step estimates a 6xN task-space Jacobian by finite differences against a *hypothetical* joint vector (no physics side effects), then gradient-descends joint values toward a Cartesian target — enforcing joint limits on every iteration, not just the final result. Runs every fixed-timestep against a continuously-moving target (resolved-rate control), which is what real teleoperated robots do rather than solving IK once per command.
- **`<mimic>` joint reproduction**: the URDF's parallelogram linkage (`pitch_2`..`pitch_6`) and two-jaw gripper (`jaw_1`/`jaw_2`) are defined via `<mimic joint="..." multiplier="...">`. Unity's URDF Importer parses this tag but never actually drives it at runtime — this project reproduces the coupling manually by copying the master joint's drive target every fixed step.





## Stack

- Unity 6 (6000.4), Universal Render Pipeline
- [Unity URDF Importer](https://github.com/Unity-Technologies/URDF-Importer) (Apache 2.0)
- [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) (Apache 2.0) — installed for future ROS-side integration
- [jhu-dvrk/dvrk_model](https://github.com/jhu-dvrk/dvrk_model) (MIT License, © da Vinci Research Kit) — robot description and meshes, under `Assets/DaVinciPSM/dvrk_model/`

## Running it

1. Open the project in Unity 6000.4+.
2. **Tools → da Vinci → Import PSM1 Demo Scene** (rebuilds the robot, fixes materials, pins the RCM in place).
3. Press Play.
