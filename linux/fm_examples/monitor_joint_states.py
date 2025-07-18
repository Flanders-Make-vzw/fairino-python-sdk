from fairino import Robot
import time

def monitor_joint_states():
    """
    Monitor joint states using Method 2: Robot object with direct XML-RPC access
    This method uses the Robot object but accesses the underlying XML-RPC client directly
    """
    print("🔧 Joint State Monitor - Method 2")
    print("Using Robot object with direct XML-RPC access")
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
        
        # Test direct XML-RPC access through robot object
        print("\n🎯 Testing direct XML-RPC access...")
        
        if not hasattr(robot, 'robot'):
            print("❌ Cannot access underlying XML-RPC client")
            return
            
        # Test single call first
        try:
            result = robot.robot.GetActualJointPosDegree(1)  # 1 = non-blocking
            print(f"📊 Direct XML-RPC test result: {result}")
            
            if not isinstance(result, list) or len(result) < 7:
                print(f"❌ Unexpected result format: {result}")
                return
                
            error_code = result[0]
            if error_code != 0:
                print(f"❌ Error code: {error_code}")
                return
                
            joint_positions = result[1:7]
            print(f"✅ Joint positions successfully retrieved: {joint_positions}")
            
        except Exception as e:
            print(f"❌ Direct XML-RPC test failed: {e}")
            return
            
        # Start continuous monitoring
        print("\n📊 Starting continuous joint state monitoring...")
        print("Press Ctrl+C to stop")
        print("-" * 70)
        print(f"{'Time':<10} {'J1 (deg)':<10} {'J2 (deg)':<10} {'J3 (deg)':<10} {'J4 (deg)':<10} {'J5 (deg)':<10} {'J6 (deg)':<10}")
        print("-" * 70)
        
        iteration = 0
        while True:
            try:
                iteration += 1
                current_time = time.strftime("%H:%M:%S")
                
                # Get joint positions using direct XML-RPC access
                result = robot.robot.GetActualJointPosDegree(1)
                
                if isinstance(result, list) and len(result) >= 7 and result[0] == 0:
                    joint_pos = result[1:7]
                    print(f"{current_time:<10} {joint_pos[0]:<10.3f} {joint_pos[1]:<10.3f} {joint_pos[2]:<10.3f} {joint_pos[3]:<10.3f} {joint_pos[4]:<10.3f} {joint_pos[5]:<10.3f}")
                else:
                    print(f"{current_time:<10} Error: {result}")
                
                # Show progress every 20 iterations
                if iteration % 20 == 0:
                    print(f"📈 Iteration {iteration} - System running normally")
                    
                time.sleep(0.5)  # 2 Hz update rate
                
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
    monitor_joint_states()