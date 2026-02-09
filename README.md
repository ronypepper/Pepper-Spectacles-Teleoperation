# Humanoid Robot Teleoperation with Augmented Reality Feedback Using Snap Spectacles

![Teleoperation](Pepper Teleoperation Demo.png)

This project allows teleoperation of a SoftBank Robotics Pepper robot (real and in simulation) using Snap Spectacles '24 AR glasses.

For details refer to TODO.

A video of the system in action from the user's augmented perspective is available at [title](link2).

## Installation using Conda:
```
conda env create -f environment-teleoperator.yml
conda env create -f environment-logprocessor.yml
```

The Spectacles project is located in *Spectacles Teleoperation Lens* and requires Lens Studio 5.15.

*PepperTeleoperator.py* is the main script and has several configurations at the top. If using simulation, you will need to accept terms in the terminal for downloading the Pepper robot's URDF files (will be installed at ~/.qibullet).

## Usage Guide

### Connection Setup
If you have a **wired** connection to your Spectacles:
- Set SERVER_USE_LOCALHOST in *PepperTeleoperator.py* to True
- Set use_localhost in *Spectacles Teleoperation Lens/Assets/TeleopWebsocketClient.ts* to true
- Run adb devices, and note XXXXXXXXX_app serial corresponding to your Spectacles
- Run adb -s XXXXXXXXX_app reverse tcp:PORT tcp:PORT, to forward your port connection. PORT must match the ports configured in *PepperTeleoperator.py* and *TeleopWebsocketClient.ts*.

If you have a **wireless** connection to your Spectacles:
- Set SERVER_USE_LOCALHOST to False and configure SERVER_IP in *PepperTeleoperator.py*
- Set use_localhost to false and configure websocket_ip in *Spectacles Teleoperation Lens/Assets/TeleopWebsocketClient.ts*
- Ensure the ports configured in *PepperTeleoperator.py* and *TeleopWebsocketClient.ts* match.

### Start Teleoperation
Run
```
conda activate pepper_spectacles_teleoperation
python3 PepperTeleoperator.py --qi-url tcp://PEPPER_IP_ADDRESS:9559
```
Then, start preview of the Spectacles Teleoperation Lens in Lens Studio. Once you do so, you need to stand upright with arm stretched out for initialization, until you see green, translucent hands overlaid over your own hands. Teleoperation should now be active. The red, translucent hands correspond to the robot's current hand poses, relative to the robot and scaled according to the REMOTE_OPERATOR_SCALE configuration in *PepperTeleoperator.py*.

### Graphs and Metrics Generation
If LOG_POSES_TO_FILE in *PepperTeleoperator.py* is set to True, log files are recorded to *teleop_logs/*. To create graphs and metrics from them, configure log_file_name at the top of *TeleopLogProcessor.py* to match your log in *teleop_logs/* and run:
```
conda activate plotting_env
python3 TeleopLogProcessor.py
```
Metrics will be printed to terminal and graphs will be saved to *teleop_graphs/*.