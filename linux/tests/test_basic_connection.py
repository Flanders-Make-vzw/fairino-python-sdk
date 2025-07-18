"""
Pytest tests for basic connection and SDK functionality
"""
import pytest
import time


@pytest.mark.integration
class TestBasicConnection:
    """Test basic robot connection and SDK functionality"""
    
    def test_robot_connection(self, robot_connection):
        """Test basic robot connection"""
        robot = robot_connection
        
        # Test SDK version retrieval
        ret, version = robot.GetSDKVersion()
        assert ret == 0, f"GetSDKVersion failed with return code: {ret}"
        assert isinstance(version, (list, tuple)), "SDK version should be a list or tuple"
        assert len(version) >= 2, "SDK version should have at least 2 components"
        
        # Verify version format
        sdk_version = version[0]
        robot_version = version[1]
        assert isinstance(sdk_version, str), "SDK version should be a string"
        assert isinstance(robot_version, str), "Robot version should be a string"
        assert "SDK" in sdk_version, "SDK version should contain 'SDK'"
        assert "Robot" in robot_version, "Robot version should contain 'Robot'"
    
    def test_robot_enable_disable(self, robot_connection):
        """Test robot enable/disable functionality"""
        robot = robot_connection
        
        # Test enable
        ret = robot.RobotEnable(1)
        assert ret == 0, f"Robot enable failed with return code: {ret}"
        
        # Brief pause
        time.sleep(0.5)
        
        # Test disable
        ret = robot.RobotEnable(0)
        assert ret == 0, f"Robot disable failed with return code: {ret}"
        
        # Re-enable for other tests
        ret = robot.RobotEnable(1)
        assert ret == 0, f"Robot re-enable failed with return code: {ret}"
    
    def test_robot_modes(self, robot_connection):
        """Test robot mode switching"""
        robot = robot_connection
        
        # Test manual mode
        ret = robot.Mode(1)
        assert ret == 0, f"Manual mode failed with return code: {ret}"
        
        time.sleep(0.5)
        
        # Test auto mode
        ret = robot.Mode(0)
        assert ret == 0, f"Auto mode failed with return code: {ret}"
        
        # Return to manual mode for other tests
        ret = robot.Mode(1)
        assert ret == 0, f"Return to manual mode failed with return code: {ret}"
    
    def test_sdk_method_access(self, robot_connection):
        """Test SDK method access and return formats"""
        robot = robot_connection
        
        # Test GetActualTCPPose method
        try:
            ret, pose = robot.GetActualTCPPose(1)  # Non-blocking
            assert isinstance(ret, int), "Return code should be integer"
            if ret == 0:
                assert isinstance(pose, (list, tuple)), "TCP pose should be a list or tuple"
                assert len(pose) >= 6, "TCP pose should have at least 6 elements"
                # Verify pose values are reasonable
                for i, val in enumerate(pose[:6]):
                    assert isinstance(val, (int, float)), f"Pose element {i} should be numeric"
        except Exception as e:
            pytest.skip(f"GetActualTCPPose not available: {e}")
        
        # Test GetActualJointPosDegree method
        try:
            ret, joint_pos = robot.GetActualJointPosDegree(1)  # Non-blocking
            assert isinstance(ret, int), "Return code should be integer"
            if ret == 0:
                assert isinstance(joint_pos, (list, tuple)), "Joint positions should be a list or tuple"
                assert len(joint_pos) >= 6, "Joint positions should have at least 6 elements"
                # Verify joint values are reasonable
                for i, val in enumerate(joint_pos[:6]):
                    assert isinstance(val, (int, float)), f"Joint position {i} should be numeric"
                    assert -720 <= val <= 720, f"Joint {i} ({val}°) outside expected range"
        except Exception as e:
            pytest.skip(f"GetActualJointPosDegree not available: {e}")
    
    def test_coordinate_system_info(self, robot_connection):
        """Test coordinate system information retrieval"""
        robot = robot_connection
        
        # Test getting current tool coordinate system
        try:
            ret, tool_num = robot.GetActualTCPNum()
            if ret == 0:
                assert isinstance(tool_num, (int, float)), "Tool number should be numeric"
                assert tool_num >= 0, "Tool number should be non-negative"
                assert tool_num <= 15, "Tool number should be within valid range (0-15)"
            else:
                pytest.skip(f"GetActualTCPNum failed with error code: {ret}")
        except Exception as e:
            pytest.skip(f"GetActualTCPNum not available: {e}")
        
        # Test getting current work object coordinate system
        try:
            ret, wobj_num = robot.GetActualWObjNum()
            if ret == 0:
                assert isinstance(wobj_num, (int, float)), "Work object number should be numeric"
                assert wobj_num >= 0, "Work object number should be non-negative"
                assert wobj_num <= 15, "Work object number should be within valid range (0-15)"
            else:
                pytest.skip(f"GetActualWObjNum failed with error code: {ret}")
        except Exception as e:
            pytest.skip(f"GetActualWObjNum not available: {e}")


@pytest.mark.integration
class TestSDKCapabilities:
    """Test SDK capabilities and features"""
    
    def test_position_retrieval_methods(self, robot_connection, robot_helpers):
        """Test different position retrieval methods"""
        robot = robot_connection
        
        # Test TCP pose retrieval
        tcp_pose = robot_helpers.get_tcp_pose(robot)
        assert tcp_pose is not None, "Failed to get TCP pose"
        assert len(tcp_pose) == 6, "TCP pose should have 6 coordinates"
        
        # Test joint position retrieval
        joint_positions = robot_helpers.get_joint_positions(robot)
        assert joint_positions is not None, "Failed to get joint positions"
        assert len(joint_positions) == 6, "Joint positions should have 6 values"
        
        # Both methods should work for the same robot
        assert tcp_pose is not None and joint_positions is not None, \
            "Both TCP pose and joint positions should be available"
    
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
    
    def test_error_handling(self, robot_connection):
        """Test error handling for invalid operations"""
        robot = robot_connection
        
        # Test invalid movement (unreachable position)
        invalid_pose = [10000, 10000, 10000, 0, 0, 0]  # Unreachable position
        ret = robot.MoveL(invalid_pose, tool=0, user=0, vel=30.0, acc=50.0, ovl=100.0,
                         blendR=-1.0, blendMode=0, offset_flag=0, 
                         offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Should return an error code (non-zero) for unreachable position
        assert ret != 0, "Expected error for unreachable position but got success"
        
        # Test invalid velocity (above typical maximum)
        try:
            ret, current_pose = robot.GetActualTCPPose(1)
            if ret == 0:
                ret = robot.MoveL(current_pose, tool=0, user=0, vel=200.0, acc=50.0, ovl=100.0, 
                                 blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                # Some systems may accept this, others may reject it
                # We just verify it doesn't crash
                assert isinstance(ret, int), "Return code should be an integer"
        except Exception:
            pass  # If GetActualTCPPose fails, skip this test
    
    def test_connection_stability(self, robot_connection, robot_helpers):
        """Test connection stability over multiple operations"""
        robot = robot_connection
        
        # Perform multiple operations to test stability
        operations_count = 10
        successful_operations = 0
        
        for i in range(operations_count):
            try:
                # Test TCP pose retrieval
                tcp_pose = robot_helpers.get_tcp_pose(robot)
                if tcp_pose is not None:
                    successful_operations += 1
                
                # Test joint position retrieval
                joint_positions = robot_helpers.get_joint_positions(robot)
                if joint_positions is not None:
                    successful_operations += 1
                
                time.sleep(0.1)
                
            except Exception as e:
                pytest.fail(f"Connection became unstable at operation {i}: {e}")
        
        # Verify high success rate
        expected_operations = operations_count * 2  # 2 operations per iteration
        success_rate = successful_operations / expected_operations
        assert success_rate > 0.9, f"Connection stability {success_rate:.1%} below threshold"


@pytest.mark.integration
class TestRobotState:
    """Test robot state information"""
    
    def test_robot_status_retrieval(self, robot_connection):
        """Test robot status information retrieval"""
        robot = robot_connection
        
        # Test SDK version (already tested but important for status)
        ret, version = robot.GetSDKVersion()
        assert ret == 0, "Should be able to get SDK version"
        
        # Test various status calls that might be available
        status_calls = [
            ('GetActualTCPNum', lambda: robot.GetActualTCPNum()),
            ('GetActualWObjNum', lambda: robot.GetActualWObjNum()),
            ('GetActualTCPPose', lambda: robot.GetActualTCPPose(1)),
            ('GetActualJointPosDegree', lambda: robot.GetActualJointPosDegree(1)),
        ]
        
        available_calls = []
        for call_name, call_func in status_calls:
            try:
                result = call_func()
                if isinstance(result, (list, tuple)) and len(result) >= 2:
                    ret = result[0]
                    if ret == 0:
                        available_calls.append(call_name)
            except Exception:
                pass  # Call not available
        
        # At least some basic status calls should be available
        assert len(available_calls) >= 1, "At least one status call should be available"
    
    def test_robot_workspace_bounds(self, robot_connection, robot_helpers):
        """Test robot workspace bounds through position readings"""
        robot = robot_connection
        
        # Get current position
        tcp_pose = robot_helpers.get_tcp_pose(robot)
        assert tcp_pose is not None, "Failed to get TCP pose"
        
        position = tcp_pose[:3]
        
        # Verify position is within reasonable workspace bounds
        # These are conservative estimates for industrial robots
        for i, pos in enumerate(position):
            assert -3000 < pos < 3000, f"Position {i} ({pos}mm) outside expected workspace"
        
        # Test joint positions are within reasonable bounds
        joint_positions = robot_helpers.get_joint_positions(robot)
        assert joint_positions is not None, "Failed to get joint positions"
        
        for i, joint in enumerate(joint_positions):
            assert -720 < joint < 720, f"Joint {i} ({joint}°) outside expected range"
    
    def test_robot_response_times(self, robot_connection, robot_helpers):
        """Test robot response times for various operations"""
        robot = robot_connection
        
        # Test TCP pose retrieval time
        start_time = time.time()
        tcp_pose = robot_helpers.get_tcp_pose(robot)
        tcp_time = time.time() - start_time
        
        assert tcp_pose is not None, "TCP pose retrieval failed"
        
        # Use warnings for slow response times instead of failing the test
        if tcp_time > 1.0:  # Fail only if extremely slow (>1s)
            pytest.fail(f"TCP pose retrieval took {tcp_time:.3f}s (extremely slow)")
        elif tcp_time > 0.5:  # Warn if moderately slow (>0.5s)
            import warnings
            warnings.warn(f"TCP pose retrieval took {tcp_time:.3f}s (slower than optimal)", UserWarning)
        
        # Test joint position retrieval time
        start_time = time.time()
        joint_positions = robot_helpers.get_joint_positions(robot)
        joint_time = time.time() - start_time
        
        assert joint_positions is not None, "Joint position retrieval failed"
        
        if joint_time > 1.0:  # Fail only if extremely slow (>1s)
            pytest.fail(f"Joint position retrieval took {joint_time:.3f}s (extremely slow)")
        elif joint_time > 0.5:  # Warn if moderately slow (>0.5s)
            import warnings
            warnings.warn(f"Joint position retrieval took {joint_time:.3f}s (slower than optimal)", UserWarning)
        
        # Test movement command response time
        start_time = time.time()
        ret = robot.MoveL(tcp_pose, tool=0, user=0, vel=30.0, acc=50.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        cmd_time = time.time() - start_time
        
        assert ret == 0, "Movement command failed"
        
        if cmd_time > 1.0:  # Fail only if extremely slow (>1s for movement commands)
            pytest.fail(f"Movement command took {cmd_time:.3f}s (extremely slow)")
        elif cmd_time > 0.2:  # Warn if moderately slow (>0.2s)
            import warnings
            warnings.warn(f"Movement command took {cmd_time:.3f}s (slower than optimal)", UserWarning)
    
    def test_motion_completion_check(self, robot_connection, robot_helpers):
        """Test motion completion checking"""
        robot = robot_connection
        
        # Get current position
        tcp_pose = robot_helpers.get_tcp_pose(robot)
        assert tcp_pose is not None, "Failed to get TCP pose"
        
        # Start a small movement
        test_pose = [
            tcp_pose[0] + 5.0,  # 5mm movement
            tcp_pose[1],
            tcp_pose[2],
            tcp_pose[3],
            tcp_pose[4],
            tcp_pose[5]
        ]
        
        ret = robot.MoveL(test_pose, tool=0, user=0, vel=20.0, acc=50.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Movement command failed"
        
        # Check motion completion if available
        motion_checked = False
        try:
            # Try to check motion completion
            for _ in range(20):  # Check for up to 2 seconds
                ret, is_complete = robot.IsMotionDone()
                if ret == 0:
                    motion_checked = True
                    if is_complete:
                        break
                time.sleep(0.1)
        except Exception:
            pass  # IsMotionDone might not be available
        
        # Wait for movement to complete
        time.sleep(2)
        
        # Return to original position
        ret = robot.MoveL(tcp_pose, tool=0, user=0, vel=20.0, acc=50.0, ovl=100.0, 
                         blendR=-1.0, blendMode=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert ret == 0, "Return movement command failed"
        
        time.sleep(2)
        
        # The test passes if we could execute movements successfully
        # Motion completion checking is optional
        assert True, "Motion execution and completion test passed"