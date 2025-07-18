"""
Moves the Fairino robot to a neutral position.
"""

from fairino import Robot
import time
import math

target_joints = [50, -80, 110, -130, -20, 0]  # Neutral position in degrees

def move_to_neutral_position():
    """
    Move the Fairino robot to a neutral position.
    
    Args:
        robot: Fairino robot object
    """
    
    robot_ip = "192.168.58.2"
    robot = Robot.RPC(robot_ip)
    time.sleep(1)  # Allow time for connection
    
    try:

        # Move to neutral position
        ret = robot.MoveJ(target_joints, tool=0, user=0, vel=50)
        
        if ret == 0:
            print("✅ Moved to neutral position successfully")
        else:
            print(f"❌ Failed to move to neutral position: {ret}")
            
    except Exception as e:
        print(f"❌ Exception while moving to neutral position: {e}") 
        
if __name__ == "__main__":

    move_to_neutral_position()