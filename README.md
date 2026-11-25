# 3-Link Robot Arm: Inverse Kinematics & Motion Control

An interactive MATLAB simulation of a 3-link planar robot arm. It demonstrates forward and inverse kinematics, Cartesian trajectory generation, joint-level PD control, and live visualization.

## Run the simulation

Open MATLAB in this repository directory and run:

```matlab
robot_arm_kinematics_control
```

Use the target and controller sliders, then select **Plan & Run**. The simulation solves inverse kinematics and follows a quintic Cartesian trajectory. Select **Stop** to interrupt motion.

## Technical approach

- **Forward kinematics:** cumulative planar joint transforms calculate link endpoints and the tool position.
- **Inverse kinematics:** damped least-squares Jacobian iterations solve the redundant 3R arm while limiting joint angles.
- **Trajectory planning:** a quintic blend interpolates the Cartesian path with zero endpoint velocity and acceleration.
- **Motion control:** independent proportional-derivative controllers track the planned joint angles.
- **Visualization:** MATLAB graphics display the 220 mm links, workspace, target, planned path, and live tip error.

All angles are in radians internally; Cartesian coordinates and link lengths are in millimeters.

## Project structure

```text
robot_arm_kinematics_control.m   Interactive simulation and controls
forward_kinematics.m             Planar forward-kinematics model
robot_jacobian.m                 Cartesian Jacobian
inverse_kinematics.m             Damped least-squares inverse kinematics
quintic_blend.m                  Quintic trajectory profile
interpolate_point.m              Cartesian trajectory interpolation
wrap_angle.m                     Angle normalization helper
tests/testRobotArmKinematics.m   MATLAB unit tests
```

## Tests

Run the MATLAB unit tests from the repository directory:

```matlab
runtests('tests')
```

The tests cover the straight-arm forward model, Jacobian derivatives, inverse-kinematics convergence, and trajectory endpoint behavior.

## Future engineering extensions

- Dynamic torque control with manipulator mass matrix
- Collision-aware workspace planning
- ROS 2 integration and URDF model export
- Experimental encoder telemetry with hardware-in-the-loop testing

## License

MIT
