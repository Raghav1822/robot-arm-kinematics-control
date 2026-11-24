# 3-Link Robot Arm: Inverse Kinematics & Motion Control

[![Live demo](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-58d7e8)](https://raghav1822.github.io/robot-arm-kinematics-control/)
[![Tests](https://img.shields.io/badge/Tests-node:test-ff9f43)](./tests/kinematics.test.mjs)

An interactive 3-link planar robot-arm project covering the complete engineering workflow: forward kinematics, inverse kinematics, Cartesian trajectory generation, joint-level PD control, and in-browser visualization.

## Live demo

Open the deployed app: **https://raghav1822.github.io/robot-arm-kinematics-control/**

Move the sliders or click the workspace to set a target, then select **Plan & Run**. The controller solves inverse kinematics and follows a smooth quintic trajectory.

## Technical approach

- **Forward kinematics:** iterative joint transforms compute every link frame and the tool position.
- **Inverse kinematics:** damped least-squares pseudoinverse solves the 3R planar manipulator redundantly and numerically.
- **Trajectory planning:** a quintic polynomial provides continuous position, bounded velocity, and zero velocity at endpoints.
- **Motion control:** independent proportional-derivative joint control tracks planned angles.
- **Workspace visualization:** canvas-based 220 mm link model with planned path and target position.

## Run locally

```bash
npm test
npm run serve
```

Then open `http://localhost:8080`.

## Project structure

```text
index.html                 Interactive interface
styles.css                 Responsive dashboard styling
kinematics.js              Kinematics and trajectory functions
app.js                     Simulation, controller, and canvas rendering
tests/kinematics.test.mjs  Unit tests for core math
```

## Validation

Run `npm test`. The test suite verifies the straight-arm forward model, a representative inverse-kinematics solution, quintic endpoint conditions, and exact endpoint interpolation.

## Future engineering extensions

- Dynamic torque control with manipulator mass matrix
- Collision-aware workspace planning
- ROS 2 integration and URDF model export
- Experimental encoder telemetry with hardware-in-the-loop testing

## License

MIT
