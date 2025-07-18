# fairino-python-sdk 

Introduction
---------------
This is a fork of the Python language SDK library specially designed for fairino collaborative robots.

This fork is targetted at Python > 3.12 for linux platforms. Enhanced with improved installation procesed, extra examples, and documentation.

Documentation
----------------
Please see [Python SDK](https://fair-documentation.readthedocs.io/en/latest/SDKManual/python_intro.html)。


Installation
----------------

### Prerequisites
- Python 3.8 or higher (tested with Python 3.12.3)
- Linux x86_64 platform (Windows support available in `windows/` folder)
- Virtual environment recommended

### Quick Install

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

### Verification

Test the installation by importing the SDK:

```python
from fairino import Robot

# Initialize robot connection (replace with your robot's IP)
robot = Robot.RPC('192.168.58.2')

# Test basic functionality
print("SDK Version:", robot.GetSDKVersion())
```

### Usage Example

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

### Troubleshooting

**Import Error**: If you encounter `ModuleNotFoundError: No module named 'fairino'`, ensure you've:
1. Activated the virtual environment
2. Installed the package from the correct directory (`linux/fairino/`)
3. All dependencies are installed

**CTypes Error**: If you get `'_ctypes.CField' object is not subscriptable` when calling robot methods, use direct XML-RPC access instead:
```python
# Instead of: ret, pose = robot.GetActualTCPPose(0)  # This may fail
# Use: tcp_result = robot.robot.GetActualTCPPose(1)  # Direct XML-RPC access
```

**Python Version**: This fork has been updated for Python 3.12 compatibility. If using older Python versions, you may need to use the original SDK or adapt the installation process.

### Development

See `linux/example/` for comprehensive usage examples and `CHANGES.md` for modification history.

### Testing

A comprehensive pytest test suite is available in `linux/tests/`:

```bash
# Run all tests
pytest

# Run with verbose output
pytest -v
```


Tests validate robot functionality, movement accuracy, and OpenTeach framework compatibility. See `linux/tests/README.md` for detailed documentation.
