from fairino import Robot
import time

# Connect to robot
robot = Robot.RPC("192.168.58.2")  # Replace with your robot's IP

# Basic robot operations
robot.Mode(0)           # Set to automatic mode
robot.RobotEnable(1)    # Enable robot
robot.ResetAllError()   # Clear any errors




# MoveGripper parameters:
#   index: jaw number --> installed as 2
#   pos: percentage of position, range [0~100] --> 0=open, 100=closed
#   vel: percentage of speed, range [0~100]
#   force: percentage of torque, range [0~100]
#   maxtime: maximum wait time, range [0~30000] ms --> Set high enough otherwise timeout error may occur
#   block: 0-blocking, 1-non-blocking (weird) --> Does not seem to to anything
#   type: type of jaws, 0-parallel jaws; 1-rotary jaws
#   rotNum: number of rotations
#   rotVel: percentage of rotational velocity [0~100]
#   rotTorque: percentage of rotational torque [0~100]


# Timing analysis
# Average time and std for gripper operation in blocking mode: 1.8365 seconds, 0.4304 seconds
# Average time and std for gripper operation in non-blocking mode: 1.8414 seconds, 0.4355 seconds



def move_gripper(pose: int, blocking: bool = False) -> int:
    return robot.MoveGripper(index=2, pos=pose, vel=50, force=10, maxtime=30000, block=0 if blocking else 1, type=0, rotNum=0, rotVel=0, rotTorque=0)

def open_gripper(blocking: bool = False) -> int:
    return move_gripper(0, blocking)

def close_gripper(blocking: bool = False) -> int:
    return move_gripper(100, blocking)


def timing_analysis():
    time_sec_list = []

    n_samples=10

    # First close the gripper to start from a known state
    close_gripper()
    time.sleep(3)

    # Blocking mode
    # open and close the gripper n_samples times, record time taken
    for _ in range(n_samples):
        t1 = time.time()
        open_gripper(blocking=True)
        t2 = time.time()
        time_sec_list.append(t2 - t1)

        time.sleep(3)

        t1 = time.time()
        close_gripper(blocking=True)
        t2 = time.time()
        time_sec_list.append(t2 - t1)

        time.sleep(3)

    avg_time = sum(time_sec_list) / len(time_sec_list)
    std_time = (sum((x - avg_time) ** 2 for x in time_sec_list) / len(time_sec_list)) ** 0.5
    print(f"Average time and std for gripper operation in blocking mode: {avg_time:.4f} seconds, {std_time:.4f} seconds")
    time_sec_list.clear()

    # Non-blocking mode

    close_gripper()
    time.sleep(3)

    for _ in range(n_samples):
        t1 = time.time()
        open_gripper(blocking=False)
        t2 = time.time()
        time_sec_list.append(t2 - t1)

        time.sleep(3)

        t1 = time.time()
        close_gripper(blocking=False)
        t2 = time.time()
        time_sec_list.append(t2 - t1)

        time.sleep(3)

    avg_time = sum(time_sec_list) / len(time_sec_list)
    std_time = (sum((x - avg_time) ** 2 for x in time_sec_list) / len(time_sec_list)) ** 0.5
    print(f"Average time and std for gripper operation in non-blocking mode: {avg_time:.4f} seconds, {std_time:.4f} seconds")
    time_sec_list.clear()

    # Close connection
    robot.CloseRPC()


if __name__ == "__main__":
    timing_analysis()