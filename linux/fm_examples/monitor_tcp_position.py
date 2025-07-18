from fairino import Robot
import time
import numpy as np

def monitor_tcp_position():
    """
    Monitor TCP position in cartesian space using GetActualTCPPose
    This method provides real-time monitoring of the actual TCP position and orientation
    """
    print("🔧 TCP Position Monitor")
    print("Real-time monitoring of TCP position in cartesian space")
    print("=" * 50)
    
    try:
        # Create Robot object connection
        robot_ip = "192.168.58.2"
        robot = Robot.RPC(robot_ip)
        
        # Test basic connection
        ret, version = robot.GetSDKVersion()
        if ret == 0:
            print(f"✅ Robot object connected successfully")
            print(f"📦 SDK Version: {version[0]}")
            print(f"🤖 Robot Version: {version[1]}")
        else:
            print(f"❌ Robot object connection failed: {ret}")
            return
            
        # Initialize robot state
        print("\n⚙️  Initializing robot state...")
        
        # Enable robot
        try:
            ret = robot.RobotEnable(1)
            if ret == 0:
                print("✅ Robot enabled successfully")
            else:
                print(f"⚠️  Robot enable returned: {ret} (may be normal)")
        except Exception as e:
            print(f"⚠️  Robot enable exception: {e}")
            
        # Set to manual mode
        try:
            ret = robot.Mode(1)  # 1 = manual mode
            if ret == 0:
                print("✅ Manual mode set successfully")
            else:
                print(f"⚠️  Manual mode returned: {ret}")
        except Exception as e:
            print(f"⚠️  Manual mode exception: {e}")
            
        # Wait for initialization
        print("⏳ Waiting for robot state initialization...")
        time.sleep(2)
        
        # Test direct XML-RPC GetActualTCPPose method
        print("\n🎯 Testing direct XML-RPC GetActualTCPPose access...")
        
        if not hasattr(robot, 'robot'):
            print("❌ Cannot access underlying XML-RPC client")
            return
            
        # Test single call first
        try:
            tcp_result = robot.robot.GetActualTCPPose(1)  # 1 = non-blocking
            
            if not isinstance(tcp_result, list) or len(tcp_result) < 7:
                print(f"❌ Unexpected TCP result format: {tcp_result}")
                return
                
            tcp_error_code = tcp_result[0]
            if tcp_error_code != 0:
                print(f"❌ TCP position error code: {tcp_error_code}")
                return
                
            tcp_pose = tcp_result[1:7]
            print(f"✅ TCP position successfully retrieved: {tcp_pose}")
            
        except Exception as e:
            print(f"❌ Direct XML-RPC GetActualTCPPose test failed: {e}")
            return
            
        # Start continuous monitoring
        print("\n📊 Starting continuous TCP position monitoring...")
        print("Press Ctrl+C to stop")
        print("-" * 90)
        print(f"{'Time':<10} {'X (mm)':<10} {'Y (mm)':<10} {'Z (mm)':<10} {'RX (deg)':<10} {'RY (deg)':<10} {'RZ (deg)':<10}")
        print("-" * 90)
        
        iteration = 0
        last_position = None
        total_distance = 0.0
        
        while True:
            try:
                iteration += 1
                current_time = time.strftime("%H:%M:%S")
                
                # Get TCP position using direct XML-RPC GetActualTCPPose
                tcp_result = robot.robot.GetActualTCPPose(1)  # 1 = non-blocking
                
                if isinstance(tcp_result, list) and len(tcp_result) >= 7 and tcp_result[0] == 0:
                    tcp_pose = tcp_result[1:7]
                    
                    # Calculate distance moved since last reading
                    if last_position is not None:
                        position_diff = np.array(tcp_pose[:3]) - np.array(last_position[:3])
                        distance = np.linalg.norm(position_diff)
                        total_distance += distance
                    
                    last_position = tcp_pose
                    
                    # Display current position
                    print(f"{current_time:<10} {tcp_pose[0]:<10.3f} {tcp_pose[1]:<10.3f} {tcp_pose[2]:<10.3f} {tcp_pose[3]:<10.3f} {tcp_pose[4]:<10.3f} {tcp_pose[5]:<10.3f}")
                    
                    # Show progress every 20 iterations
                    if iteration % 20 == 0:
                        print(f"📈 Iteration {iteration} - Total distance moved: {total_distance:.3f}mm")
                        
                else:
                    print(f"{current_time:<10} TCP Error: {tcp_result}")
                
                time.sleep(0.5)  # 2 Hz update rate
                
            except KeyboardInterrupt:
                print("\n\n🛑 Monitoring stopped by user")
                break
            except Exception as e:
                print(f"❌ Monitoring error: {e}")
                time.sleep(1)
                
        print(f"\n📊 Final Statistics:")
        print(f"   Total iterations: {iteration}")
        print(f"   Total distance moved: {total_distance:.3f}mm")
        print(f"   Average distance per iteration: {total_distance/max(iteration-1, 1):.3f}mm")
        
        print("\n🔌 Closing robot connection...")
        robot.CloseRPC()
        print("✅ Connection closed successfully")
        
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()

def monitor_tcp_position_with_velocity():
    """
    Enhanced TCP position monitor that also calculates velocity
    """
    print("🔧 TCP Position Monitor with Velocity")
    print("Real-time monitoring of TCP position and velocity")
    print("=" * 50)
    
    try:
        # Create Robot object connection
        robot_ip = "192.168.58.2"
        robot = Robot.RPC(robot_ip)
        
        # Test basic connection
        ret, version = robot.GetSDKVersion()
        if ret == 0:
            print(f"✅ Robot connected successfully")
            print(f"📦 SDK Version: {version[0]}")
        else:
            print(f"❌ Robot connection failed: {ret}")
            return
            
        # Initialize robot
        print("\n⚙️  Initializing robot...")
        robot.RobotEnable(1)
        robot.Mode(1)
        time.sleep(2)
        
        # Start monitoring with velocity calculation
        print("\n📊 Starting TCP position monitoring with velocity...")
        print("Press Ctrl+C to stop")
        print("-" * 110)
        print(f"{'Time':<10} {'X':<8} {'Y':<8} {'Z':<8} {'RX':<8} {'RY':<8} {'RZ':<8} {'Vel(mm/s)':<12} {'AngVel(°/s)':<12}")
        print("-" * 110)
        
        last_pose = None
        last_time = None
        
        while True:
            try:
                current_time = time.time()
                time_str = time.strftime("%H:%M:%S")
                
                # Get TCP position using direct XML-RPC GetActualTCPPose
                tcp_result = robot.robot.GetActualTCPPose(1)  # 1 = non-blocking
                
                if isinstance(tcp_result, list) and len(tcp_result) >= 7 and tcp_result[0] == 0:
                    tcp_pose = tcp_result[1:7]
                    
                    # Calculate velocity if we have previous data
                    linear_vel = 0.0
                    angular_vel = 0.0
                    
                    if last_pose is not None and last_time is not None:
                        dt = current_time - last_time
                        if dt > 0:
                            # Linear velocity
                            pos_diff = np.array(tcp_pose[:3]) - np.array(last_pose[:3])
                            linear_vel = np.linalg.norm(pos_diff) / dt
                            
                            # Angular velocity
                            ori_diff = np.array(tcp_pose[3:]) - np.array(last_pose[3:])
                            angular_vel = np.linalg.norm(ori_diff) / dt
                    
                    # Display data
                    print(f"{time_str:<10} {tcp_pose[0]:<8.2f} {tcp_pose[1]:<8.2f} {tcp_pose[2]:<8.2f} {tcp_pose[3]:<8.2f} {tcp_pose[4]:<8.2f} {tcp_pose[5]:<8.2f} {linear_vel:<12.3f} {angular_vel:<12.3f}")
                    
                    last_pose = tcp_pose
                    last_time = current_time
                    
                else:
                    print(f"{time_str:<10} TCP Error: {tcp_result}")
                    
                time.sleep(0.5)
                
            except KeyboardInterrupt:
                print("\n\n🛑 Monitoring stopped by user")
                break
            except Exception as e:
                print(f"❌ Monitoring error: {e}")
                time.sleep(1)
                
        print("\n🔌 Closing robot connection...")
        robot.CloseRPC()
        print("✅ Connection closed successfully")
        
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--velocity":
        monitor_tcp_position_with_velocity()
    else:
        monitor_tcp_position()