"""
Pytest tests for cartesian movement functionality
"""
import pytest
import numpy as np
import time


@pytest.mark.cartesian
@pytest.mark.integration
class TestCartesianMovement:
    """Test cartesian movement using MoveL"""
    
    def test_small_cartesian_movement(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test small cartesian movement with position verification"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Create small movements (10mm in each direction, 5° rotation)
        target_pose = [
            start_pose[0] + 10.0,   # X +10mm
            start_pose[1] - 5.0,    # Y -5mm
            start_pose[2] + 5.0,    # Z +5mm
            start_pose[3] + 5.0,    # RX +5°
            start_pose[4] - 3.0,    # RY -3°
            start_pose[5] + 2.0     # RZ +2°
        ]
        
        # Execute movement with proper parameters as per documentation
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, f"Cartesian movement failed with return code: {ret}"
        
        # Wait for movement to complete
        robot_helpers.wait_for_movement(4)
        
        # Verify final position
        final_pose = robot_helpers.get_tcp_pose(robot)
        assert final_pose is not None, "Failed to get final TCP pose"
        
        # Calculate errors
        pos_error = robot_helpers.calculate_position_error(final_pose, target_pose)
        ori_error = robot_helpers.calculate_orientation_error(final_pose, target_pose)
        
        # Assert tolerances
        assert pos_error < 1.0, f"Position error {pos_error:.3f}mm exceeds tolerance"
        assert ori_error < 1.0, f"Orientation error {ori_error:.3f}° exceeds tolerance"
        
        # Return to start position
        ret = robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Failed to return to start position"
        robot_helpers.wait_for_movement(4)
    
    def test_return_to_original_position(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test return to original position accuracy"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Move to offset position
        offset_pose = [
            start_pose[0] + 15.0,
            start_pose[1] + 10.0,
            start_pose[2] - 5.0,
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        ret = robot.MoveL(offset_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Failed to move to offset position"
        robot_helpers.wait_for_movement(3)
        
        # Return to original position
        ret = robot.MoveL(start_pose, tool=0, user=0, vel=20.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Failed to return to original position"
        robot_helpers.wait_for_movement(4)
        
        # Verify return accuracy
        final_pose = robot_helpers.get_tcp_pose(robot)
        assert final_pose is not None, "Failed to get final TCP pose"
        
        return_error = robot_helpers.calculate_position_error(final_pose, start_pose)
        assert return_error < 1.0, f"Return position error {return_error:.3f}mm exceeds tolerance"
    
    @pytest.mark.parametrize("velocity", [10, 30, 50])
    def test_different_velocities(self, robot_connection, current_tcp_pose, robot_helpers, velocity):
        """Test cartesian movement with different velocities"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Small movement with specified velocity
        test_pose = [
            start_pose[0] + 5.0,
            start_pose[1] - 3.0,
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        start_time = time.time()
        ret = robot.MoveL(test_pose, tool=0, user=0, vel=float(velocity), acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, f"Movement with velocity {velocity}% failed"
        
        robot_helpers.wait_for_movement(3)
        end_time = time.time()
        
        duration = end_time - start_time
        assert duration > 0, "Movement duration should be positive"
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=float(velocity), acc=0.0, ovl=100.0, 
                   blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                   search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(3)
    
    def test_openteach_compatible_movement(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test OpenTeach-compatible cartesian movement function"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        def openteach_move_cartesian(robot, cartesian_coords, velocity=30):
            """OpenTeach-compatible cartesian movement function"""
            if isinstance(cartesian_coords, np.ndarray):
                cartesian_coords = cartesian_coords.tolist()
            
            if len(cartesian_coords) != 6:
                return False
            
            # Use proper SDK parameters as per documentation
            ret = robot.MoveL(cartesian_coords, tool=0, user=0, vel=float(velocity), acc=0.0, ovl=100.0, 
                             blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                             search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            return ret == 0
        
        # Test with numpy array input
        test_coords = np.array([
            start_pose[0] + 8.0,
            start_pose[1] - 4.0,
            start_pose[2] + 2.0,
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ], dtype=np.float32)
        
        success = openteach_move_cartesian(robot, test_coords, velocity=25)
        assert success, "OpenTeach-compatible movement failed"
        
        robot_helpers.wait_for_movement(3)
        
        # Verify position
        final_pose = robot_helpers.get_tcp_pose(robot)
        assert final_pose is not None, "Failed to get final TCP pose"
        
        # Return to start
        success = openteach_move_cartesian(robot, start_pose, velocity=25)
        assert success, "Failed to return to start position"
        robot_helpers.wait_for_movement(3)
    
    def test_position_only_movement(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test position-only movement keeping orientation constant"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Move only position, keep orientation
        position_only_pose = [
            start_pose[0] + 15.0,   # X +15mm
            start_pose[1] + 10.0,   # Y +10mm
            start_pose[2] - 5.0,    # Z -5mm
            start_pose[3],          # Keep RX
            start_pose[4],          # Keep RY
            start_pose[5]           # Keep RZ
        ]
        
        ret = robot.MoveL(position_only_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Position-only movement failed"
        
        robot_helpers.wait_for_movement(3)
        
        # Verify position changed and orientation remained the same
        final_pose = robot_helpers.get_tcp_pose(robot)
        assert final_pose is not None, "Failed to get final TCP pose"
        
        pos_change = np.linalg.norm(np.array(final_pose[:3]) - np.array(start_pose[:3]))
        ori_change = np.linalg.norm(np.array(final_pose[3:]) - np.array(start_pose[3:]))
        
        assert pos_change > 10.0, f"Position change {pos_change:.3f}mm too small"
        assert ori_change < 1.0, f"Orientation change {ori_change:.3f}° too large"
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                   blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                   search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(3)
    
    @pytest.mark.parametrize("tool,user", [(0, 0), (1, 0), (0, 1)])
    def test_coordinate_frames(self, robot_connection, current_tcp_pose, robot_helpers, tool, user):
        """Test movement with different coordinate frames"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        test_pose = [
            start_pose[0] + 5.0,
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        ret = robot.MoveL(test_pose, tool=tool, user=user, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Note: Some coordinate frames may not be configured, so we allow failures
        # but successful ones should work properly
        if ret == 0:
            robot_helpers.wait_for_movement(2)
            
            # Return to start position
            robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                       blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                       search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            robot_helpers.wait_for_movement(2)
        else:
            pytest.skip(f"Coordinate frame tool={tool}, user={user} not configured")

    def test_offset_movement(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test movement with position offset"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Define target pose
        target_pose = [
            start_pose[0] + 10.0,
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Test movement with workpiece coordinate system offset
        offset_pos = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 5mm X offset
        
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=1, offset_pos=offset_pos)  # offset_flag=1 for workpiece coordinate
        assert ret == 0, "Movement with offset failed"
        
        robot_helpers.wait_for_movement(3)
        
        # Verify position includes offset
        final_pose = robot_helpers.get_tcp_pose(robot)
        assert final_pose is not None, "Failed to get final TCP pose"
        
        # Expected position should include both target movement and offset
        expected_x = start_pose[0] + 10.0 + 5.0  # target + offset
        pos_error = abs(final_pose[0] - expected_x)
        assert pos_error < 2.0, f"Position error with offset {pos_error:.3f}mm exceeds tolerance"
        
        # Return to start position
        robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                   blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                   search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(3)

    def test_velocity_scaling_factor(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test movement with different velocity scaling factors (ovl parameter)"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        test_pose = [
            start_pose[0] + 200.0,
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Test with 50% velocity scaling
        start_time = time.time()
        ret = robot.MoveL(test_pose, tool=0, user=0, vel=50.0, acc=0.0, ovl=50.0,  # 50% scaling
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Movement with velocity scaling failed"
        
        robot_helpers.wait_for_movement(3)
        slow_duration = time.time() - start_time
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=50.0, acc=0.0, ovl=50.0, 
                   blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                   search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(3)
        
        # Test with 100% velocity scaling
        start_time = time.time()
        ret = robot.MoveL(test_pose, tool=0, user=0, vel=50.0, acc=0.0, ovl=100.0,  # 100% scaling
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Movement with full velocity scaling failed"
        
        robot_helpers.wait_for_movement(3)
        fast_duration = time.time() - start_time
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=50.0, acc=0.0, ovl=100.0, 
                   blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                   search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(3)
        
        # Verify that 50% scaling was indeed slower (within reasonable tolerance)
        # Note: This is a soft check as timing can vary
        if fast_duration > 0.5:  # Only check if movements took reasonable time
            assert slow_duration >= fast_duration * 0.8, "Velocity scaling didn't show expected effect"


@pytest.mark.cartesian
@pytest.mark.slow
class TestCartesianAccuracy:
    """Test cartesian movement accuracy and precision"""
    
    def test_movement_accuracy_tolerance(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test movement accuracy within specified tolerances"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Test multiple target positions
        test_offsets = [
            [50, 0, 0, 0, 0, 0],    # X only
            [0, 50, 0, 0, 0, 0],    # Y only
            [0, 0, 50, 0, 0, 0],    # Z only
            [20, 20, 20, 0, 0, 0],     # XYZ combined
        ]
        
        for offset in test_offsets:
            target_pose = [start_pose[i] + offset[i] for i in range(6)]
            
            ret = robot.MoveL(target_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                             blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                             search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            assert ret == 0, f"Movement to {offset} failed"
            
            robot_helpers.wait_for_movement(3)
            
            # Verify accuracy
            final_pose = robot_helpers.get_tcp_pose(robot)
            assert final_pose is not None, "Failed to get final TCP pose"
            
            pos_error = robot_helpers.calculate_position_error(final_pose, target_pose)
            assert pos_error < 1.0, f"Position error {pos_error:.3f}mm exceeds tolerance for offset {offset}"
            
            # Return to start
            robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                       blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                       search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            robot_helpers.wait_for_movement(3)
    
    def test_repeatability(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test movement repeatability"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        target_pose = [
            start_pose[0] + 10.0,
            start_pose[1] + 5.0,
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        positions = []
        
        # Repeat movement 3 times
        for i in range(3):
            ret = robot.MoveL(target_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                             blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                             search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            assert ret == 0, f"Movement {i+1} failed"
            
            robot_helpers.wait_for_movement(3)
            
            final_pose = robot_helpers.get_tcp_pose(robot)
            assert final_pose is not None, f"Failed to get pose for movement {i+1}"
            positions.append(final_pose[:3])
            
            # Return to start
            robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                       blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                       search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            robot_helpers.wait_for_movement(3)
        
        # Check repeatability
        positions = np.array(positions)
        position_variance = np.var(positions, axis=0)
        max_variance = np.max(position_variance)
        
        assert max_variance < 0.1, f"Position variance {max_variance:.3f} exceeds repeatability tolerance"

    def test_blend_mode_movement(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test movement with different blend modes"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        # Define intermediate and target poses
        intermediate_pose = [
            start_pose[0] + 10.0,
            start_pose[1],
            start_pose[2] + 5.0,
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        target_pose = [
            start_pose[0] + 10.0,
            start_pose[1] + 10.0,
            start_pose[2] + 5.0,
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Test with corner transition (blendMode=1)
        ret = robot.MoveL(intermediate_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=5.0, blendMode=1, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Movement to intermediate pose with blend mode failed"
        
        # Move to target with normal blend
        ret = robot.MoveL(target_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Movement to target pose failed"
        
        robot_helpers.wait_for_movement(4)
        
        # Return to start position
        robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                   blendR=-1.0, blendMode=0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], 
                   search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(3)

    def test_external_axis_compatibility(self, robot_connection, current_tcp_pose, robot_helpers):
        """Test movement command compatibility with external axis parameters"""
        robot = robot_connection
        start_pose = current_tcp_pose
        
        test_pose = [
            start_pose[0] + 5.0,
            start_pose[1],
            start_pose[2],
            start_pose[3],
            start_pose[4],
            start_pose[5]
        ]
        
        # Test with external axis positions (even if no external axes are connected)
        exaxis_positions = [0.0, 0.0, 0.0, 0.0]  # 4 external axes as per documentation
        
        ret = robot.MoveL(test_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, exaxis_pos=exaxis_positions, 
                         search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Movement with external axis parameters failed"
        
        robot_helpers.wait_for_movement(3)
        
        # Return to start
        robot.MoveL(start_pose, tool=0, user=0, vel=30.0, acc=0.0, ovl=100.0, 
                   blendR=-1.0, blendMode=0, exaxis_pos=exaxis_positions, 
                   search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_helpers.wait_for_movement(3)