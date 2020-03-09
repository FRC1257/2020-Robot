# FRC 1257's 2020 Robot Code

[![Build Status](https://travis-ci.org/FRC1257/2020-Robot.svg?branch=master)](https://travis-ci.org/FRC1257/2020-Robot)

Code for FRC Team 1257 Parallel Universe's 2020 Robot.

## General Info

Our code is written in Java and the 2020 WPILib command-based framework.

## Subsystems

- Drivetrain
- Indexer
- Intake
- Shooter
- Elevator

## Trajectory Visualizer
To plan our auto paths, we use the builtin WPILib `TrajectoryGenerator`, but we wrote our own program called [`TrajectoryVisualizer`](src/main/java/frc/robot/commands/auto/trajectory/visualization/TrajectoryVisualizer.java) that helps us visualize these on top of a field.

## Vendor Dependencies

- REV Robotics
- Kauai Labs navX-MXP
- REV Color Sensor v3
