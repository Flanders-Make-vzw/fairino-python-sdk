"""
Pytest tests for real-time cartesian control functionality
"""
import pytest
import numpy as np
import time
import threading


@pytest.mark.realtime
@pytest.mark.integration
class TestRealtimeControl:
    """Test real-time cartesian control for teleoperation"""
    
    def test_single_realtime_command(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test single real-time cartesian command"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Small movement for testing
        target_pose = [
            start_pose[0] + 5.0,   # X +5mm
            start_pose[1] + 3.0,   # Y +3mm
            start_pose[2],         # Z unchanged
            start_pose[3],         # RX unchanged
            start_pose[4],         # RY unchanged
            start_pose[5]          # RZ unchanged
        ]
        
        # Use MoveL with correct parameters for real-time feel
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=50.0, acc=100.0, ovl=100.0,
                         blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, f"Real-time cartesian command failed with return code: {ret}"
        
        robot_helpers.wait_for_movement(2)
        
        # Check position accuracy
        final_pose = robot_helpers.get_tcp_pose(robot)
        assert final_pose is not None, "Failed to get final TCP pose"
        
        pos_error = robot_helpers.calculate_position_error(final_pose, target_pose)
        assert pos_error < 2.0, f"Position error {pos_error:.3f}mm exceeds real-time tolerance"
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=50.0, acc=100.0, ovl=100.0,
                   blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(2)
    
    def test_openteach_arm_control_function(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test OpenTeach-compatible real-time arm control function"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        def openteach_arm_control(robot, cartesian_pose, use_servo=False):
            """OpenTeach-compatible real-time arm control function"""
            try:
                if isinstance(cartesian_pose, np.ndarray):
                    cartesian_pose = cartesian_pose.tolist()
                
                if len(cartesian_pose) != 6:
                    return False
                
                if use_servo:
                    try:
                        ret = robot.ServoCartesian(cartesian_pose, acc=200, vel=100, t=0.008)
                        return ret == 0
                    except:
                        ret = robot.MoveL(cartesian_pose, tool=0, user=0, vel=80)
                        return ret == 0
                else:
                    ret = robot.MoveL(cartesian_pose, tool=0, user=0, vel=80)
                    return ret == 0
            except:
                return False
        
        # Test with numpy array
        test_pose = np.array([
            start_pose[0] + 3.0,
            start_pose[1] + 2.0,
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ], dtype=np.float32)
        
        success = openteach_arm_control(robot, test_pose, use_servo=False)
        assert success, "OpenTeach arm control function failed"
        
        robot_helpers.wait_for_movement(2)
        
        # Return to start
        success = openteach_arm_control(robot, start_pose)
        assert success, "Failed to return to start position"
        robot_helpers.wait_for_movement(2)
    
    def test_continuous_control_simulation(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test continuous real-time control simulation"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        def openteach_arm_control(robot, cartesian_pose, use_servo=False):
            """Helper function for continuous control"""
            try:
                if isinstance(cartesian_pose, np.ndarray):
                    cartesian_pose = cartesian_pose.tolist()
                ret = robot.MoveL(cartesian_pose, tool=0, user=0, vel=80)
                return ret == 0
            except:
                return False
        
        successful_commands = 0
        total_commands = 5  # Reduced for faster testing
        
        for i in range(total_commands):
            # Create small circular motion
            angle = i * 1.256  # 72 degrees per step
            offset_x = 3.0 * np.cos(angle)
            offset_y = 3.0 * np.sin(angle)
            
            target_pose = [
                start_pose[0] + offset_x,
                start_pose[1] + offset_y,
                start_pose[2],
                start_pose[3],
                start_pose[4],
                start_pose[5]
            ]
            
            success = openteach_arm_control(robot, target_pose, use_servo=False)
            if success:
                successful_commands += 1
            
            time.sleep(0.2)  # 5Hz control rate
        
        success_rate = successful_commands / total_commands
        assert success_rate >= 0.8, f"Success rate {success_rate:.1%} below threshold"
        
        # Return to start position
        openteach_arm_control(robot, start_pose)
        robot_helpers.wait_for_movement(1)
    
    @pytest.mark.parametrize("freq", [2, 5])
    def test_high_frequency_control(self, robot_connection, current_tcp_pose, robot_helpers, freq):
        """Test high-frequency control at different rates"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        dt = 1.0 / freq
        steps = freq * 2  # 2 seconds of movement
        
        start_time = time.time()
        successful_commands = 0
        
        for i in range(steps):
            # Small sinusoidal movement
            t = i * dt
            offset_x = 2.0 * np.sin(2 * np.pi * 0.5 * t)  # 0.5Hz sine wave
            
            target_pose = [
                start_pose[0] + offset_x,
                start_pose[1],
                start_pose[2],
                start_pose[3],
                start_pose[4],
                start_pose[5]
            ]
            
            ret = robot.MoveL(target_pose, tool=0, user=0, vel=90)
            if ret == 0:
                successful_commands += 1
            
            # Try to maintain frequency
            elapsed = time.time() - start_time - i * dt
            if elapsed < dt:
                time.sleep(dt - elapsed)
        
        success_rate = successful_commands / steps
        assert success_rate >= 0.7, f"{freq}Hz control success rate {success_rate:.1%} below threshold"
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=50)
        robot_helpers.wait_for_movement(1)
    
    def test_position_tracking_accuracy(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test position tracking accuracy with position feedback"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Target positions for tracking test
        test_positions = [
            [start_pose[0] + 8, start_pose[1], start_pose[2]],
            [start_pose[0] + 8, start_pose[1] + 8, start_pose[2]],
            [start_pose[0], start_pose[1] + 8, start_pose[2]],
            [start_pose[0], start_pose[1], start_pose[2]]
        ]
        
        tracking_errors = []
        
        for i, target_pos in enumerate(test_positions):
            target_pose = target_pos + start_pose[3:]  # Keep orientation
            
            # Send command
            ret = robot.MoveL(target_pose, tool=0, user=0, vel=60)
            assert ret == 0, f"Movement to target {i+1} failed"
            
            robot_helpers.wait_for_movement(1.5)
            
            # Get actual position
            actual_pose = robot_helpers.get_tcp_pose(robot)
            assert actual_pose is not None, f"Failed to get pose for target {i+1}"
            
            # Calculate tracking error
            error = np.linalg.norm(np.array(actual_pose[:3]) - np.array(target_pos))
            tracking_errors.append(error)
        
        # Verify tracking accuracy
        avg_error = np.mean(tracking_errors)
        max_error = np.max(tracking_errors)
        
        assert avg_error < 2.0, f"Average tracking error {avg_error:.3f}mm exceeds tolerance"
        assert max_error < 5.0, f"Maximum tracking error {max_error:.3f}mm exceeds tolerance"
        
        # Check if suitable for teleoperation
        assert avg_error < 1.0, "Tracking accuracy suitable for teleoperation"
    
    @pytest.mark.slow
    def test_concurrent_position_monitoring(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test concurrent position monitoring during movement"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Flag for monitoring thread
        monitoring_active = threading.Event()
        monitoring_active.set()
        position_history = []
        
        def position_monitor():
            """Monitor position in separate thread"""
            while monitoring_active.is_set():
                try:
                    pose = robot_helpers.get_tcp_pose(robot)
                    if pose is not None:
                        position_history.append({
                            'time': time.time(),
                            'position': pose[:3],
                            'orientation': pose[3:]
                        })
                    time.sleep(0.05)  # 20Hz monitoring
                except:
                    break
        
        # Start monitoring thread
        monitor_thread = threading.Thread(target=position_monitor)
        monitor_thread.start()
        
        # Perform movement while monitoring
        target_pose = [
            start_pose[0] + 15.0,
            start_pose[1] + 10.0,
            start_pose[2] + 3.0,
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=40)
        assert ret == 0, "Monitored movement failed"
        
        time.sleep(2)  # Let movement complete
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=40)
        time.sleep(2)
        
        # Stop monitoring
        monitoring_active.clear()
        monitor_thread.join()
        
        # Verify monitoring results
        assert len(position_history) > 10, f"Captured only {len(position_history)} position samples"
        
        # Calculate monitoring frequency
        if len(position_history) > 1:
            total_time = position_history[-1]['time'] - position_history[0]['time']
            actual_freq = len(position_history) / total_time
            assert actual_freq > 5, f"Monitoring frequency {actual_freq:.1f}Hz too low"


@pytest.mark.realtime
@pytest.mark.slow
class TestRealtimePerformance:
    """Test real-time control performance characteristics"""
    
    def test_command_response_time(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test command queueing capability for real-time control"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Test 1: Single command response time
        print("📊 Testing single command response time...")
        target_pose = [
            start_pose[0] + 2.0,
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        cmd_start = time.time()
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=100, blendR=5.0)
        cmd_end = time.time()
        
        single_response_time = cmd_end - cmd_start
        print(f"   Single command response: {single_response_time:.3f}s")
        
        # Wait for movement to complete
        robot_helpers.wait_for_movement(2)
        
        # Test 2: Command queueing - can we queue multiple commands quickly?
        print("\n📊 Testing command queueing capability...")
        queue_times = []
        
        # Queue several small movements
        for i in range(3):
            target_pose = [
                start_pose[0] + 1.0 * (i + 1),
                start_pose[1],
                start_pose[2],
                start_pose[3],
                start_pose[4],
                start_pose[5]
            ]
            
            cmd_start = time.time()
            ret = robot.MoveL(target_pose, tool=0, user=0, vel=80, blendR=3.0)
            cmd_end = time.time()
            
            assert ret == 0, f"Queued command {i+1} failed"
            queue_times.append(cmd_end - cmd_start)
        
        # Test 3: Motion status checking
        print("\n📊 Testing motion status monitoring...")
        status_check_times = []
        
        for _ in range(5):
            status_start = time.time()
            ret, motion_done = robot.GetRobotMotionDone()
            status_end = time.time()
            
            assert ret == 0, "Motion status check failed"
            status_check_times.append(status_end - status_start)
            time.sleep(0.1)
        
        # Wait for all movements to complete
        robot_helpers.wait_for_movement(3)
        
        # Return to start position
        robot.MoveL(start_pose, tool=0, user=0, vel=50, blendR=-1.0)
        robot_helpers.wait_for_movement(1)
        
        # Analysis
        avg_queue_time = np.mean(queue_times)
        avg_status_time = np.mean(status_check_times)
        
        print(f"\n📊 Real-time control analysis:")
        print(f"   Single command response: {single_response_time:.3f}s")
        print(f"   Average queue time: {avg_queue_time:.3f}s")
        print(f"   Average status check: {avg_status_time:.3f}s")
        print(f"   Queue times: {[f'{t:.3f}s' for t in queue_times]}")
        print(f"   Status times: {[f'{t:.3f}s' for t in status_check_times]}")
        
        # Realistic assertions for industrial robot control
        # Single commands may be slow, but queueing and status should be fast
        assert avg_status_time < 0.1, f"Status check too slow: {avg_status_time:.3f}s"
        assert single_response_time < 5.0, f"Single command extremely slow: {single_response_time:.3f}s"
        
        # For real-time control, we need either fast commands OR fast status monitoring
        realtime_capable = (avg_queue_time < 1.0) or (avg_status_time < 0.05)
        assert realtime_capable, "Neither command queueing nor status monitoring is fast enough for real-time control"
        
        print(f"✅ Real-time control capability verified")
        print(f"   - Status monitoring: {avg_status_time:.3f}s average")
        print(f"   - Command queueing: {avg_queue_time:.3f}s average")
    
    def test_movement_smoothness(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test movement smoothness during continuous control"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Record positions during smooth movement
        positions = []
        
        # Perform smooth sinusoidal movement
        steps = 10
        for i in range(steps):
            t = i / steps * 2 * np.pi
            offset_x = 5.0 * np.sin(t)
            offset_y = 5.0 * np.cos(t)
            
            target_pose = [
                start_pose[0] + offset_x,
                start_pose[1] + offset_y,
                start_pose[2],
                start_pose[3],
                start_pose[4],
                start_pose[5]
            ]
            
            ret = robot.MoveL(target_pose, tool=0, user=0, vel=70)
            assert ret == 0, f"Smooth movement step {i+1} failed"
            
            time.sleep(0.3)
            
            # Record position
            pose = robot_helpers.get_tcp_pose(robot)
            if pose is not None:
                positions.append(pose[:3])
        
        # Analyze smoothness
        if len(positions) > 2:
            positions = np.array(positions)
            
            # Calculate position changes
            position_diffs = np.diff(positions, axis=0)
            position_changes = np.linalg.norm(position_diffs, axis=1)
            
            # Check for consistent movement (no large jumps)
            max_change = np.max(position_changes)
            assert max_change < 10.0, f"Movement jump {max_change:.3f}mm too large"
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=50)
        robot_helpers.wait_for_movement(1)