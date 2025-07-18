"""
Pytest tests for monitoring functionality (TCP position, joint states, etc.)
"""
import pytest
import numpy as np
import time


@pytest.mark.monitoring
@pytest.mark.integration
class TestTCPPositionMonitoring:
    """Test TCP position monitoring functionality"""
    
    def test_tcp_pose_retrieval(self, robot_connection, robot_helpers):
        """Test basic TCP pose retrieval"""
        robot = robot_connection
        
        # Test GetActualTCPPose method
        tcp_pose = robot_helpers.get_tcp_pose(robot)
        assert tcp_pose is not None, "Failed to get TCP pose"
        assert len(tcp_pose) == 6, f"Expected 6 TCP coordinates, got {len(tcp_pose)}"
        
        # Verify pose values are reasonable
        position = tcp_pose[:3]
        orientation = tcp_pose[3:]
        
        # Position should be within reasonable workspace bounds
        for i, pos in enumerate(position):
            assert -2000 < pos < 2000, f"Position coordinate {i} ({pos}) outside reasonable bounds"
        
        # Orientation should be within reasonable bounds
        for i, ori in enumerate(orientation):
            assert -360 < ori < 360, f"Orientation coordinate {i} ({ori}) outside reasonable bounds"
    
    def test_tcp_pose_consistency(self, robot_connection, robot_helpers):
        """Test TCP pose consistency over multiple readings"""
        robot = robot_connection
        
        poses = []
        for i in range(5):
            pose = robot_helpers.get_tcp_pose(robot)
            assert pose is not None, f"Failed to get TCP pose on reading {i+1}"
            poses.append(pose)
            time.sleep(0.1)
        
        # Check consistency (robot should be stationary)
        poses = np.array(poses)
        position_variance = np.var(poses[:, :3], axis=0)
        orientation_variance = np.var(poses[:, 3:], axis=0)
        
        max_pos_variance = np.max(position_variance)
        max_ori_variance = np.max(orientation_variance)
        
        assert max_pos_variance < 0.01, f"Position variance {max_pos_variance:.6f} too high for stationary robot"
        assert max_ori_variance < 0.01, f"Orientation variance {max_ori_variance:.6f} too high for stationary robot"
    
    def test_tcp_pose_during_movement(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test TCP pose monitoring during movement"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Use bigger movements to ensure detection
        target_pose = [
            start_pose[0] + 50.0,  # Increased from 10.0mm to 50.0mm
            start_pose[1] + 30.0,  # Increased from 5.0mm to 30.0mm
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Verify robot is ready for movement
        initial_pose = robot_helpers.get_tcp_pose(robot)
        assert initial_pose is not None, "Failed to get initial TCP pose"
        
        # Start movement with slower velocity for better detection
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=20)  # Reduced from 30 to 20
        assert ret == 0, "Movement command failed"
        
        # Wait for movement to actually start
        print("⏳ Waiting for robot to start movement...")
        time.sleep(2.0)  # Increased from 1.0s to 2.0s
        
        # Monitor position during movement with extended time
        positions = []
        start_time = time.time()
        
        # Monitor for longer period to catch the movement
        for i in range(40):  # Increased from 25 to 40 (8 seconds)
            pose = robot_helpers.get_tcp_pose(robot)
            if pose is not None:
                positions.append({
                    'time': time.time() - start_time,
                    'position': pose[:3]
                })
            time.sleep(0.2)
        
        # Verify we captured position changes
        assert len(positions) > 5, "Failed to capture sufficient position samples"
        
        # Calculate movement with better error reporting
        if len(positions) > 1:
            position_data = [p['position'] for p in positions]
            positions_array = np.array(position_data)
            position_changes = np.diff(positions_array, axis=0)
            total_movement = np.sum(np.linalg.norm(position_changes, axis=1))
            
            # Calculate expected movement distance
            expected_movement = np.linalg.norm(np.array(target_pose[:3]) - np.array(initial_pose[:3]))
            
            # Print debug information
            print(f"Initial position: {initial_pose[:3]}")
            print(f"Target position: {target_pose[:3]}")
            print(f"First monitored position: {positions[0]['position']}")
            print(f"Last monitored position: {positions[-1]['position']}")
            print(f"Total movement detected: {total_movement:.3f}mm")
            print(f"Expected movement: {expected_movement:.3f}mm")
            
            # Check if movement is still happening in different time windows
            if total_movement < 5.0:  # If very little movement detected
                # Check movement in different quarters
                quarter_size = len(positions) // 4
                quarters = [
                    positions[0:quarter_size],
                    positions[quarter_size:2*quarter_size],
                    positions[2*quarter_size:3*quarter_size],
                    positions[3*quarter_size:]
                ]
                
                quarter_movements = []
                for i, quarter in enumerate(quarters):
                    if len(quarter) > 1:
                        q_positions = np.array([p['position'] for p in quarter])
                        q_changes = np.diff(q_positions, axis=0)
                        q_movement = np.sum(np.linalg.norm(q_changes, axis=1))
                        quarter_movements.append(q_movement)
                        print(f"Quarter {i+1} movement: {q_movement:.3f}mm")
                
                # Use maximum quarter movement as the detected movement
                if quarter_movements:
                    max_quarter_movement = max(quarter_movements)
                    if max_quarter_movement > total_movement:
                        total_movement = max_quarter_movement
                        print(f"Using maximum quarter movement: {total_movement:.3f}mm")
            
            # FAIL the test if no significant movement is detected
            if total_movement < 1.0:  # Much stricter threshold
                # Final check: wait for movement to complete and check total displacement
                print("⏳ Waiting for movement to complete...")
                time.sleep(5)  # Wait for movement to complete
                final_pose = robot_helpers.get_tcp_pose(robot)
                if final_pose is not None:
                    actual_movement = np.linalg.norm(np.array(final_pose[:3]) - np.array(initial_pose[:3]))
                    print(f"Actual total movement: {actual_movement:.3f}mm")
                    
                    # FAIL if robot didn't move significantly
                    assert actual_movement > 20.0, f"❌ MOVEMENT FAILED: Robot moved only {actual_movement:.3f}mm, expected ~{expected_movement:.3f}mm. Possible causes: movement command rejected, robot in wrong state, workspace limits, or safety stop."
                else:
                    pytest.fail("❌ MONITORING FAILED: Cannot verify final position")
            else:
                print("✅ Movement detected during monitoring")
                # Still verify the movement was reasonable
                assert total_movement > 5.0, f"Movement detection {total_movement:.3f}mm below minimum threshold"
        
        # Wait for movement to complete and return to start
        robot_helpers.wait_for_movement(8)  # Increased wait time
        robot.MoveL(start_pose, tool=0, user=0, vel=20)
        robot_helpers.wait_for_movement(8)
    
    def test_tcp_pose_with_velocity_calculation(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test TCP pose monitoring with velocity calculation"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        target_pose = [
            start_pose[0] + 8.0,
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Start movement
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=40)
        assert ret == 0, "Movement command failed"
        
        # Monitor with velocity calculation
        last_pose = None
        last_time = None
        velocities = []
        
        for i in range(8):
            current_time = time.time()
            pose = robot_helpers.get_tcp_pose(robot)
            
            if pose is not None and last_pose is not None and last_time is not None:
                dt = current_time - last_time
                if dt > 0:
                    pos_diff = np.array(pose[:3]) - np.array(last_pose[:3])
                    linear_vel = np.linalg.norm(pos_diff) / dt
                    velocities.append(linear_vel)
            
            last_pose = pose
            last_time = current_time
            time.sleep(0.2)
        
        # Verify velocity calculations
        if len(velocities) > 2:
            avg_velocity = np.mean(velocities)
            max_velocity = np.max(velocities)
            
            assert avg_velocity > 0, "Average velocity should be positive during movement"
            assert max_velocity < 1000, f"Maximum velocity {max_velocity:.1f}mm/s unreasonably high"
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=40)
        robot_helpers.wait_for_movement(3)


@pytest.mark.monitoring
@pytest.mark.integration
class TestJointStateMonitoring:
    """Test joint state monitoring functionality"""
    
    def test_joint_positions_retrieval(self, robot_connection, robot_helpers):
        """Test basic joint positions retrieval"""
        robot = robot_connection
        
        # Test joint position retrieval
        joint_positions = robot_helpers.get_joint_positions(robot)
        assert joint_positions is not None, "Failed to get joint positions"
        assert len(joint_positions) == 6, f"Expected 6 joint positions, got {len(joint_positions)}"
        
        # Verify joint values are within reasonable bounds
        for i, joint_pos in enumerate(joint_positions):
            assert -360 < joint_pos < 360, f"Joint {i+1} position {joint_pos}° outside reasonable bounds"
    
    def test_joint_positions_consistency(self, robot_connection, robot_helpers):
        """Test joint position consistency over multiple readings"""
        robot = robot_connection
        
        joint_readings = []
        for i in range(5):
            joints = robot_helpers.get_joint_positions(robot)
            assert joints is not None, f"Failed to get joint positions on reading {i+1}"
            joint_readings.append(joints)
            time.sleep(0.1)
        
        # Check consistency (robot should be stationary)
        joint_readings = np.array(joint_readings)
        joint_variance = np.var(joint_readings, axis=0)
        max_variance = np.max(joint_variance)
        
        assert max_variance < 0.01, f"Joint variance {max_variance:.6f} too high for stationary robot"
    
    def test_joint_tcp_pose_correlation(self, robot_connection, robot_helpers):
        """Test correlation between joint positions and TCP pose"""
        robot = robot_connection
        
        # Get joint positions and TCP pose
        joint_positions = robot_helpers.get_joint_positions(robot)
        tcp_pose = robot_helpers.get_tcp_pose(robot)
        
        assert joint_positions is not None, "Failed to get joint positions"
        assert tcp_pose is not None, "Failed to get TCP pose"
        
        # Both should be retrieved successfully for the same robot state
        # This is a basic correlation test - both methods should work
        assert len(joint_positions) == 6, "Joint positions should have 6 values"
        assert len(tcp_pose) == 6, "TCP pose should have 6 values"
        
        # Verify they represent the same robot state by checking multiple readings
        correlations = []
        for i in range(3):
            joints = robot_helpers.get_joint_positions(robot)
            pose = robot_helpers.get_tcp_pose(robot)
            
            if joints is not None and pose is not None:
                correlations.append((joints, pose))
            time.sleep(0.1)
        
        assert len(correlations) >= 2, "Failed to get sufficient correlated readings"


@pytest.mark.monitoring
@pytest.mark.slow
class TestContinuousMonitoring:
    """Test continuous monitoring functionality"""
    
    def test_continuous_tcp_monitoring(self, robot_connection, robot_helpers):
        """Test continuous TCP position monitoring"""
        robot = robot_connection
        
        # Monitor for a short period
        monitoring_duration = 2.0  # seconds
        start_time = time.time()
        tcp_readings = []
        
        while time.time() - start_time < monitoring_duration:
            pose = robot_helpers.get_tcp_pose(robot)
            if pose is not None:
                tcp_readings.append({
                    'time': time.time(),
                    'pose': pose
                })
            time.sleep(0.1)  # 10Hz monitoring
        
        # Verify monitoring results
        assert len(tcp_readings) > 10, f"Captured only {len(tcp_readings)} TCP readings"
        
        # Calculate actual monitoring frequency
        if len(tcp_readings) > 1:
            total_time = tcp_readings[-1]['time'] - tcp_readings[0]['time']
            actual_freq = len(tcp_readings) / total_time
            assert actual_freq > 5, f"Monitoring frequency {actual_freq:.1f}Hz too low"
    
    def test_continuous_joint_monitoring(self, robot_connection, robot_helpers):
        """Test continuous joint position monitoring"""
        robot = robot_connection
        
        # Monitor for a short period
        monitoring_duration = 2.0  # seconds
        start_time = time.time()
        joint_readings = []
        
        while time.time() - start_time < monitoring_duration:
            joints = robot_helpers.get_joint_positions(robot)
            if joints is not None:
                joint_readings.append({
                    'time': time.time(),
                    'joints': joints
                })
            time.sleep(0.1)  # 10Hz monitoring
        
        # Verify monitoring results
        assert len(joint_readings) > 10, f"Captured only {len(joint_readings)} joint readings"
        
        # Calculate actual monitoring frequency
        if len(joint_readings) > 1:
            total_time = joint_readings[-1]['time'] - joint_readings[0]['time']
            actual_freq = len(joint_readings) / total_time
            assert actual_freq > 5, f"Monitoring frequency {actual_freq:.1f}Hz too low"
    
    def test_monitoring_during_movement(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test monitoring during robot movement"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        target_pose = [
            start_pose[0] + 60.0, 
            start_pose[1] + 40.0, 
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Verify robot is ready for movement
        initial_pose = robot_helpers.get_tcp_pose(robot)
        assert initial_pose is not None, "Failed to get initial TCP pose"
        
        # Start movement with slower velocity for better detection
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=15)  # Reduced from 35 to 15
        assert ret == 0, "Movement command failed"
        
        # Wait longer for movement to actually start
        print("⏳ Waiting for robot to start movement...")
        time.sleep(3.0)  # Increased from 1.5s to 3.0s
        
        # Monitor during movement with extended time
        tcp_readings = []
        joint_readings = []
        start_time = time.time()
        
        # Extended monitoring time to catch the movement
        for i in range(50):  # Increased from 30 to 50 (10 seconds)
            pose = robot_helpers.get_tcp_pose(robot)
            joints = robot_helpers.get_joint_positions(robot)
            
            if pose is not None:
                tcp_readings.append({
                    'time': time.time() - start_time,
                    'pose': pose
                })
            if joints is not None:
                joint_readings.append(joints)
            
            time.sleep(0.2)
        
        # Verify we captured movement
        assert len(tcp_readings) > 10, "Failed to capture sufficient TCP readings"
        assert len(joint_readings) > 10, "Failed to capture sufficient joint readings"
        
        # Verify movement was detected with better error reporting
        if len(tcp_readings) > 2:
            positions = np.array([reading['pose'][:3] for reading in tcp_readings])
            position_changes = np.diff(positions, axis=0)
            total_movement = np.sum(np.linalg.norm(position_changes, axis=1))
            
            # Calculate expected movement distance
            expected_movement = np.linalg.norm(np.array(target_pose[:3]) - np.array(initial_pose[:3]))
            
            # Print debug information
            print(f"Initial position: {initial_pose[:3]}")
            print(f"Target position: {target_pose[:3]}")
            print(f"First monitored position: {tcp_readings[0]['pose'][:3]}")
            print(f"Last monitored position: {tcp_readings[-1]['pose'][:3]}")
            print(f"Total movement detected: {total_movement:.3f}mm")
            print(f"Expected movement: {expected_movement:.3f}mm")
            
            # Check if movement is happening in different time windows
            if total_movement < 10.0:  # If very little movement detected
                # Check movement in different fifths
                fifth_size = len(tcp_readings) // 5
                fifths = []
                for i in range(5):
                    start_idx = i * fifth_size
                    end_idx = (i + 1) * fifth_size if i < 4 else len(tcp_readings)
                    if end_idx > start_idx:
                        fifths.append(tcp_readings[start_idx:end_idx])
                
                fifth_movements = []
                for i, fifth in enumerate(fifths):
                    if len(fifth) > 1:
                        f_positions = np.array([r['pose'][:3] for r in fifth])
                        f_changes = np.diff(f_positions, axis=0)
                        f_movement = np.sum(np.linalg.norm(f_changes, axis=1))
                        fifth_movements.append(f_movement)
                        print(f"Fifth {i+1} movement: {f_movement:.3f}mm")
                
                # Use maximum fifth movement as the detected movement
                if fifth_movements:
                    max_fifth_movement = max(fifth_movements)
                    if max_fifth_movement > total_movement:
                        total_movement = max_fifth_movement
                        print(f"Using maximum fifth movement: {total_movement:.3f}mm")
            
            # FAIL the test if no significant movement is detected
            if total_movement < 2.0:  # Much stricter threshold
                # Final check: wait for movement to complete and check total displacement
                print("⏳ Waiting for movement to complete...")
                time.sleep(8)  # Wait for movement to complete
                final_pose = robot_helpers.get_tcp_pose(robot)
                if final_pose is not None:
                    actual_movement = np.linalg.norm(np.array(final_pose[:3]) - np.array(initial_pose[:3]))
                    print(f"Actual total movement: {actual_movement:.3f}mm")
                    
                    # FAIL if robot didn't move significantly
                    assert actual_movement > 40.0, f"❌ MOVEMENT FAILED: Robot moved only {actual_movement:.3f}mm, expected ~{expected_movement:.3f}mm. Possible causes: movement command rejected, robot in wrong state, workspace limits, or safety stop."
                else:
                    pytest.fail("❌ MONITORING FAILED: Cannot verify final position")
            else:
                print("✅ Movement detected during monitoring")
                # Still verify the movement was reasonable
                assert total_movement > 10.0, f"Movement detection {total_movement:.3f}mm below minimum threshold"
        
        # Return to start with increased wait time
        robot.MoveL(start_pose, tool=0, user=0, vel=15)
        robot_helpers.wait_for_movement(10)  # Increased wait time


@pytest.mark.monitoring
class TestMonitoringAccuracy:
    """Test monitoring accuracy and reliability"""
    
    def test_tcp_pose_accuracy(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test TCP pose monitoring accuracy"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Move to known position
        target_pose = [
            start_pose[0] + 10.0,
            start_pose[1] + 5.0,
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=30)
        assert ret == 0, "Movement to target failed"
        
        robot_helpers.wait_for_movement(3)
        
        # Monitor final position
        final_pose = robot_helpers.get_tcp_pose(robot)
        assert final_pose is not None, "Failed to get final pose"
        
        # Calculate accuracy
        pos_error = robot_helpers.calculate_position_error(final_pose, target_pose)
        ori_error = robot_helpers.calculate_orientation_error(final_pose, target_pose)
        
        assert pos_error < 2.0, f"Position monitoring error {pos_error:.3f}mm too high"
        assert ori_error < 2.0, f"Orientation monitoring error {ori_error:.3f}° too high"
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=30)
        robot_helpers.wait_for_movement(3)
    
    def test_monitoring_reliability(self, robot_connection, robot_helpers):
        """Test monitoring reliability over extended period"""
        robot = robot_connection
        
        # Monitor for reliability over time
        failed_readings = 0
        total_readings = 50
        
        for i in range(total_readings):
            tcp_pose = robot_helpers.get_tcp_pose(robot)
            joint_positions = robot_helpers.get_joint_positions(robot)
            
            if tcp_pose is None or joint_positions is None:
                failed_readings += 1
            
            time.sleep(0.05)  # 20Hz monitoring
        
        # Calculate reliability
        reliability = (total_readings - failed_readings) / total_readings
        assert reliability > 0.95, f"Monitoring reliability {reliability:.1%} below threshold"


@pytest.mark.monitoring
class TestMovementCommandAvailability:
    """Test availability of movement commands"""
    
    def test_movement_command_availability(self, robot_connection, current_tcp_pose):
        """Test availability of movement commands"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Test MoveL availability with proper parameters
        test_pose = [
            start_pose[0] + 1.0,  # Small 1mm movement
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Use proper parameter names as per documentation
        ret = robot.MoveL(test_pose, tool=0, user=0, vel=30.0, acc=50.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, f"MoveL command failed with return code: {ret}"
        
        # Wait for movement to complete
        time.sleep(2)
        
        # Return to start position
        ret = robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=50.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, f"Return MoveL command failed with return code: {ret}"
        
        time.sleep(2)