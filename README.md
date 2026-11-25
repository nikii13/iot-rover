# Autonomous IoT Rover

A Raspberry Pi robotics project combining embedded software, sensor integration and feedback control.

## Features
- Wheel encoder acquisition through a C++ service and UNIX socket.
- Python motor control and per-wheel PID speed regulation.
- IMU and colour sensor integration, calibration and line-following experiments.
- Hardware-independent tests and a kinematic simulator.

## Repository structure
- Root C++ files and systemd service: encoder acquisition.
- `rover_project/`: control software, calibration, tests and supporting tools.
- `lab/`: experimental code.

## Getting started
See the [technical documentation](rover_project/README.md) for dependencies, build instructions, simulation and hardware checks. The alternative encoder service must not run simultaneously with the existing service.

The black-line-following implementation is documented as tested in simulation, not validated on the physical rover. This is an academic robotics project.
