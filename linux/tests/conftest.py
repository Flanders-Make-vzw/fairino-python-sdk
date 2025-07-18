"""
Pytest configuration and fixtures for Fairino robot tests
"""
import pytest
import time
from fairino import Robot


@pytest.fixture(scope="session")
def robot_ip():
    """Default robot IP address"""
    return "192.168.58.2"


@pytest.fixture(scope="function")
def robot_connection(robot_ip):
    """
    Create a robot connection for each test.
    This fixture handles connection setup and cleanup.
    """
    robot = Robot.RPC(robot_ip)
    
    # Test connection
    ret, version = robot.GetSDKVersion()
    if ret != 0:
        pytest.skip(f"Cannot connect to robot at {robot_ip}")
    
    # Initialize robot
    try:
        robot.RobotEnable(1)
        robot.Mode(1)  # Manual mode
        time.sleep(1)  # Brief initialization wait
    except Exception as e:
        pytest.skip(f"Robot initialization failed: {e}")
    
    yield robot
    
    # Cleanup
    try:
        robot.CloseRPC()
    except:
        pass


@pytest.fixture(scope="function")
def current_tcp_pose(robot_connection):
    """
    Get the current TCP pose using direct XML-RPC GetActualTCPPose.
    This fixture provides the starting position for movement tests.
    """
    robot = robot_connection
    
    try:
        # Use direct XML-RPC access like in monitor_tcp_position.py
        tcp_result = robot.robot.GetActualTCPPose(1)  # 1 = non-blocking
        
        if not isinstance(tcp_result, list) or len(tcp_result) < 7:
            pytest.skip(f"Unexpected TCP result format: {tcp_result}")
        
        if tcp_result[0] != 0:
            pytest.skip(f"GetActualTCPPose failed with error code: {tcp_result[0]}")
        
        return list(tcp_result[1:7])  # Return [x, y, z, rx, ry, rz]
    
    except Exception as e:
        pytest.skip(f"Failed to get current TCP pose: {e}")


@pytest.fixture(scope="function")
def current_joint_positions(robot_connection):
    """
    Get the current joint positions in degrees using direct XML-RPC.
    This fixture provides the starting joint positions for tests.
    """
    robot = robot_connection
    
    try:
        # Use direct XML-RPC access for consistency
        joint_result = robot.robot.GetActualJointPosDegree(1)  # 1 = non-blocking
        
        if not isinstance(joint_result, list) or len(joint_result) < 7:
            pytest.skip(f"Unexpected joint result format: {joint_result}")
        
        if joint_result[0] != 0:
            pytest.skip(f"GetActualJointPosDegree failed with error code: {joint_result[0]}")
        
        return list(joint_result[1:7])  # Return 6 joint positions
    
    except Exception as e:
        pytest.skip(f"Failed to get current joint positions: {e}")


class RobotTestHelpers:
    """Helper class with common robot testing utilities"""
    
    @staticmethod
    def get_tcp_pose(robot):
        """Get TCP pose using direct XML-RPC GetActualTCPPose"""
        try:
            # Use direct XML-RPC access like in monitor_tcp_position.py
            tcp_result = robot.robot.GetActualTCPPose(1)  # 1 = non-blocking
            
            if isinstance(tcp_result, list) and len(tcp_result) >= 7 and tcp_result[0] == 0:
                return list(tcp_result[1:7])  # Return [x, y, z, rx, ry, rz]
        except Exception as e:
            print(f"Warning: GetActualTCPPose failed: {e}")
        
        return None
    
    @staticmethod
    def get_joint_positions(robot):
        """Get joint positions in degrees using direct XML-RPC"""
        try:
            # Use direct XML-RPC access for consistency
            joint_result = robot.robot.GetActualJointPosDegree(1)  # 1 = non-blocking
            
            if isinstance(joint_result, list) and len(joint_result) >= 7 and joint_result[0] == 0:
                return list(joint_result[1:7])  # Return 6 joint positions
        except Exception as e:
            print(f"Warning: GetActualJointPosDegree failed: {e}")
        
        return None
    
    @staticmethod
    def wait_for_movement(seconds=3):
        """Wait for robot movement to complete"""
        time.sleep(seconds)
    
    @staticmethod
    def calculate_position_error(pose1, pose2):
        """Calculate position error between two poses (in mm)"""
        try:
            import numpy as np
            pos_error = np.array(pose1[:3]) - np.array(pose2[:3])
            return float(np.linalg.norm(pos_error))
        except ImportError:
            # Fallback without numpy
            pos_error = [(pose1[i] - pose2[i])**2 for i in range(3)]
            return (sum(pos_error) ** 0.5)
    
    @staticmethod
    def calculate_orientation_error(pose1, pose2):
        """Calculate orientation error between two poses (in degrees)"""
        try:
            import numpy as np
            ori_error = np.array(pose1[3:6]) - np.array(pose2[3:6])
            return float(np.linalg.norm(ori_error))
        except ImportError:
            # Fallback without numpy
            ori_error = [(pose1[i] - pose2[i])**2 for i in range(3, 6)]
            return (sum(ori_error) ** 0.5)
    
    @staticmethod
    def is_motion_complete(robot):
        """Check if robot motion is complete"""
        try:
            ret, is_complete = robot.IsMotionDone()
            return ret == 0 and is_complete
        except Exception:
            return False


@pytest.fixture(scope="function")
def robot_helpers():
    """Provide robot test helper utilities"""
    return RobotTestHelpers()


# Test markers
def pytest_configure(config):
    """Configure custom pytest markers"""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers", "cartesian: marks tests that involve cartesian movements"
    )
    config.addinivalue_line(
        "markers", "joint: marks tests that involve joint movements"
    )
    config.addinivalue_line(
        "markers", "monitoring: marks tests that involve continuous monitoring"
    )
    config.addinivalue_line(
        "markers", "realtime: marks tests that involve real-time control"
    )
    config.addinivalue_line(
        "markers", "integration: marks tests that require full robot integration"
    )