# fairino-python-sdk 

Introduction
---------------
This is a fork of the Python language SDK library specially designed for fairino collaborative robots.

This fork is targetted at Python > 3.12 for linux platforms. Enhanced with improved installation process, extra examples, tests, and documentation.

Documentation
----------------
Please see [Python SDK](https://fairino-doc-en.readthedocs.io/latest/SDKManual/python_intro.html)

> **⚠️ Important:** If you experience connection issues or reconnection problems, please refer to the [Connection Issues Troubleshooting](#connection-issues-and-ros-node-crashes) section for essential network configuration steps.


Installation
----------------

## Prerequisites
- Python 3.8 or higher (tested with Python 3.12.3)
- Linux x86_64 platform (Windows support available in `windows/` folder)
- Virtual environment recommended

## Quick Install

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd fairino-python-sdk
   ```

2. **create and activate the virtual environment** 
   ```bash
   python3 -m venv env
   source env/bin/activate
   ```

3. **Install required dependencies**:
   ```bash
   pip install cython setuptools
   ```

4. **Install the Fairino SDK**:
   ```bash
   cd linux/fairino
   python setup.py install
   ```

## Verification

Test the installation by importing the SDK:

```python
from fairino import Robot

# Initialize robot connection (replace with your robot's IP)
robot = Robot.RPC('192.168.58.2')

# Test basic functionality
print("SDK Version:", robot.GetSDKVersion())
```

## Usage Example

```python
from fairino import Robot

# Connect to robot
robot = Robot.RPC("192.168.58.2")  # Replace with your robot's IP

# Basic robot operations
robot.Mode(0)           # Set to automatic mode
robot.RobotEnable(1)    # Enable robot
robot.ResetAllError()   # Clear any errors

# Using robot.GetActualJointPosDegree and GetActualTCPPose fails with CTypes error
# Use direct XML-RPC access instead
joint_result = robot.robot.GetActualJointPosDegree(1)  # 1 = non-blocking
tcp_result = robot.robot.GetActualTCPPose(1)           # 1 = non-blocking

# Close connection
robot.CloseRPC()
```

## Troubleshooting

**Import Error**: If you encounter `ModuleNotFoundError: No module named 'fairino'`, ensure you've:
1. Activated the virtual environment
2. Installed the package from the correct directory (`linux/fairino/`)
3. All dependencies are installed

**CTypes Error**: If you get `'_ctypes.CField' object is not subscriptable` when calling robot methods, use direct XML-RPC access instead:
```python
# Instead of: ret, pose = robot.GetActualTCPPose(0)  # This may fail
# Use: tcp_result = robot.robot.GetActualTCPPose(1)  # Direct XML-RPC access

```
### Connection Issues and ROS Node Crashes

If you have connection issues and the ROS node crashes due to reconnection issues, follow these troubleshooting steps:

Reference: [https://www.youtube.com/watch?v=TmgsO-ZCiwU](https://www.youtube.com/watch?v=TmgsO-ZCiwU)

Execute
```shell
./network_config.sh
```
to automatically apply the commands below, or run them manually.

#### Identify interface and driver

Find the wired interface name and basic link state, then confirm the driver/firmware as a baseline.

- Run:  
```shell
ip link show and nmcli device status
```
to identify the Ethernet device (for example, enp3s0).
    
- Show driver/firmware: 
```shell
sudo ethtool -i enp3s0
```
to confirm the NIC driver and note it for any driver‑specific quirks.

#### Force 100 Mb/s Full Duplex (no autoneg)

Lock speed/duplex to eliminate autonegotiation flaps or mismatches, then verify it took effect.

- Set: 
```shell
sudo ethtool -s enp3s0 speed 100 duplex full autoneg off 
```
(requires NIC and switch support).
    
- Verify: 
```shell
ethtool enp3s0
```
should report "Speed: 100Mb/s" and "Duplex: Full" with autoneg off.
    
Note: If the link drops after forcing, restore autoneg and try another cable/switch port as the video suggests.

#### Disable Energy‑Efficient Ethernet (EEE)

EEE can introduce micro‑sleeps that look like brief disconnects; disable it to stabilize the link.

```shell
sudo ethtool --set-eee enp3s0 eee off # Disable
sudo ethtool --show-eee enp3s0 # Verify
```

Some NICs don't support toggling EEE; if "Operation not supported" appears, that's a NIC limitation rather than a failure.




**Python Version**: This fork has been updated for Python 3.12 compatibility. If using older Python versions, you may need to use the original SDK or adapt the installation process.

## Development

See `linux/example/` for comprehensive usage examples and `CHANGES.md` for modification history.

## Testing

A comprehensive pytest test suite is available in `linux/tests/`:

```bash
# Run all tests
pytest

# Run with verbose output
pytest -v
```


Tests validate robot functionality, movement accuracy, and OpenTeach framework compatibility. See `linux/tests/README.md` for detailed documentation.


## Timing analysis 

Timing analysis results (from the [test_movement_timing.py](linux/tests/test_movement_timing.py) test) comparing the delays for the real robot vs fairino SimMachine. 
It is quite disturbing that the real robot is about x800 slower than the simulator for the MoveL command. 

## Configure JODELL Gripper 

The JODELL EPG40-050 gripper has a predefined configuration in the fairino system. However is still required so steps to integrate and activate.
Fairino have their page for installing the gripper but it is difficult to follow.
https://fairino-doc-en.readthedocs.io/latest/CobotsManual/robot_peripherals.html#gripper-peripheral-configuration
The following steps will set up the gripper as required.
1. On the fairino web interface navigate to "Initial -> Peripheral -> End-Tool -> Adapted device"
2. Press "Clear"
3. The configure: Equipment type "Gripper equipment", 
                  Gripper Manu. "JODELL" 
                  Gripper type "RG"
                  Software version "J1.0"
                  Mount location "Port 2 at the end"
4. Press "Configure"
5. Press "Reset"
6. Select Activate the configured gripper "2"
7. press "Active"
This completes the installation.
A test script is located at `linux/fm_examples/gripper_example.py`.


