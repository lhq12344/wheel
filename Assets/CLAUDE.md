# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Unity-based robotics simulation project for a differential drive robot with a 6-DOF articulated arm. The project combines:
- A differential drive mobile base with twin-track locomotion
- A 6-joint robotic arm (Zu5_LDASM) imported from URDF
- Unity ArticulationBody physics for realistic joint dynamics

## Architecture

### Core Components

**DiffDriveTwinController.cs** - Main differential drive controller
- Implements two control modes: TargetPoint (go-to-goal) and TargetYaw (orientation-only)
- Uses kinematic model: converts chassis velocity (v, ω) to left/right wheel velocities
- Features trapezoidal velocity profiling with configurable limits (vMax, aMax, wMax, alphaMax)
- Auto-detects robot geometry (track width, wheel radius) from wheel transforms
- Handles mouse click navigation and keyboard yaw control
- Applies motion via Rigidbody.MovePosition/MoveRotation for physics integration

**OneJointTrapezoidController.cs** - Single joint trajectory controller
- Implements trapezoidal velocity profile for ArticulationBody joints
- Uses position + velocity feedforward control via xDrive
- Configurable velocity/acceleration limits and PD gains (stiffness, damping)

**Link.cs** (ArticulationArmBindToCar) - Arm-to-chassis binding
- Binds the articulated arm root (Link_00) to the mobile base using TeleportRoot
- Synchronizes arm position with car mount point in FixedUpdate
- Critical for maintaining rigid connection between mobile base and arm

### Robot Model (Zu5_LDASM)

- URDF-based 6-DOF arm imported from SolidWorks
- Joint configuration in `config/joint_names_Zu 5.SLDASM.yaml`
- Meshes stored in `Zu5_LDASM/urdf/meshes/` and root `meshes/`
- Two URDF variants: original export and Unity-fixed version

## Key Design Patterns

**Differential Drive Kinematics**
```
vLeft = v - ω * (trackWidth / 2)
vRight = v + ω * (trackWidth / 2)
wheelAngularVelocity = linearVelocity / wheelRadius
```

**Trapezoidal Velocity Profile**
- Accelerate at aMax until reaching vMax or deceleration distance
- Deceleration distance: d_brake = v² / (2a)
- Ensures smooth stops without overshoot

**ArticulationBody Integration**
- Use xDrive with stiffness/damping for joint control
- TeleportRoot for kinematic constraints (arm binding)
- Physics.SyncTransforms() after teleportation to prevent visual lag

## Development Notes

**Control Tuning**
- Differential drive gains: kDist (linear), kYaw (angular)
- Joint controller: adjust stiffness/damping/forceLimit in xDrive
- Velocity limits prevent instability; acceleration limits ensure smooth motion

**Geometry Detection**
- Auto-detection uses wheel transforms and lateralAxis/wheelRollAxis settings
- Falls back to Collider radius or Mesh bounds for wheel radius
- Verify trackWidth_b and wheelRadius_r in Inspector after Start()

**Physics Considerations**
- Use FixedUpdate for all physics-based control
- Rigidbody on mobile base, ArticulationBody for arm joints
- Disable collisions between arm and chassis (recommended via layers)

**Scene Structure**
- Main scene: `Scenes/SampleScene.unity`
- Robot prefab likely contains: CarRoot with Rigidbody, ArmMount transform, and arm hierarchy with ArticulationBodies
- Wheel transforms: wheelLF, wheelLR, wheelRF, wheelRR (visual only, no WheelColliders)

## File Organization

```
/
├── DiffDriveTwinController.cs    # Mobile base controller
├── OneJointTrapezoidController.cs # Joint trajectory controller
├── Link.cs                        # Arm-chassis binding
├── Scenes/                        # Unity scenes
│   └── SampleScene.unity
└── Zu5_LDASM/                     # Robot arm package
    ├── urdf/                      # URDF files and meshes
    ├── config/                    # Joint configuration
    └── meshes/                    # 3D models
```

## Common Modifications

**Adding new control modes**: Extend ControlMode enum in DiffDriveTwinController and add corresponding target computation method

**Multi-joint coordination**: Create controller that manages multiple OneJointTrapezoidController instances or implements coordinated trajectory planning

**Path following**: Extend TargetPoint mode to track waypoint sequences instead of single target

**Arm inverse kinematics**: Implement IK solver that computes joint angles for desired end-effector pose, then use OneJointTrapezoidController for execution
