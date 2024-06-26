
# Self-Driving Vehicle Control

This repository contains the final project for the "Introduction to Self-Driving Cars" course, part of the Self-Driving Cars Specialization. The project involves implementing a controller in the CARLA simulator to navigate a vehicle through preset waypoints on a race track, using both longitudinal and lateral control.

<div align="center">
  <img src="controller_output/video_output_gif.gif" alt="Carla Simulation output GIF">
</div>

## Project Overview

In this project, I had to implement a controller for the CARLA simulator. The goal is to control the vehicle to follow a race track by navigating through preset waypoints, reaching these waypoints at certain desired speeds.

## Installation Instructions

1. **Install the CARLA Simulator**: Follow the CARLA Installation guide.

2. **Download and Setup Project Files**:
   Download the "Self-Driving_Vehicle_Control" folder and place it into the subfolder "PythonClient" inside the "CarlaSimulator" (root) folder. This will create a subfolder "Self-Driving_Vehicle_Control" under "PythonClient" which contains the assignment files.

   ```plaintext
   CarlaSimulator
   ├── PythonClient
       ├── Self-Driving_Vehicle_Control
   ```

## Controller Implementation

The `controller2d.py` file contains a controller object where it was implemented in `update_controls` method. The controller receives information about the vehicle's current state and desired waypoints, and outputs the necessary throttle, steer, and brake commands to follow the waypoints.

- **Longitudinal Control**: Implemented using a PID controller to maintain the desired speed by adjusting the throttle and brake.
- **Lateral Control**: Implemented using the Stanley controller to minimize the cross-track error by adjusting the steering angle.

The controller was able to achieve 100% of the waypoints with the desired velocity, ensuring precise navigation and speed control throughout the race track.

## Running the Simulation

1. **Set Environment Variable**:
   if you dont face the following problem, skip this step.
   Before running CARLA, use the following command to avoid the issue of CARLA closing after around 50 seconds:

   ```sh
   set OPENSSL_ia32cap=:~0x20000000
   ```

2. **Start CARLA Simulator**:
   In one terminal, start the CARLA simulator at a 30Hz fixed time-step with the following command:

   ```sh
   CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30
   ```

3. **Run the Controller**:
   In another terminal, change the directory to go into the "Self-Driving_Vehicle_Control" folder, under the "PythonClient" folder. Execute the following command while CARLA is open:

   ```sh
   python module_7.py
   ```

4. **Viewing Results**:
   The simulator will begin to run if the `module_7` client connects to the server properly. It will open two feedback windows (unless live plotting is disabled). One window shows the trajectory, and the other shows the controls feedback.

   After the simulation completes, a text file containing the trajectory generated by your controller is saved. This file is called `trajectory.txt` and is located inside the "controller_output" folder under the "Self-Driving_Vehicle_Control" folder. The plots for speed, throttle, brake, steering, and the executed 2D trajectory are also saved into this folder.

## Evaluating the Controller Implementation

To evaluate your controller implementation, run the grading script with the following command from the "Self-Driving_Vehicle_Control" directory:

```sh
python grade_c1m7.py racetrack_waypoints.txt /controller_output/trajectory.txt
```

The grading script will plot your trajectory along with the velocity at each waypoint. In this implementation, the controller was able to achieve 100% of the waypoints with the desired velocity.

## Results
The video for the final result is also available in the `controller_output` folder.
The following images show the results of the vehicle's trajectory, forward speed, steering output, and throttle output:

### Trajectory
<div align="center">
  <img src="controller_output/trajectory.png" alt="Trajectory">
</div>

### Forward Speed, Steering Output, and Throttle Output

<div align="center">
  <img src="controller_output/forward_speed.png" alt="Forward Speed" width="30%">
  <img src="controller_output/steer_output.png" alt="Steering Output" width="30%">
  <img src="controller_output/throttle_output.png" alt="Throttle Output" width="30%">
</div>

## Conclusion

This project demonstrates the implementation of a PID controller for longitudinal control and a Stanley controller for lateral control in the CARLA simulator, allowing a vehicle to navigate a race track by following preset waypoints at desired speeds.
