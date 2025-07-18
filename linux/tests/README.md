# Fairino Robot Test Suite

This directory contains comprehensive pytest tests for the Fairino robot Python SDK. The tests are designed to validate robot functionality, ensure reproducibility, and support integration with frameworks like OpenTeach.

## Test Structure

### Test Files
- `test_basic_connection.py` - Basic connection and SDK functionality tests
- `test_cartesian_movement.py` - Cartesian movement and positioning tests
- `test_realtime_control.py` - Real-time control and teleoperation tests
- `test_monitoring.py` - TCP position and joint state monitoring tests

### Configuration
- `conftest.py` - Pytest fixtures and configuration
- `pytest.ini` - Test discovery and execution settings

## Test Categories (Markers)

Tests are organized using pytest markers:

- `@pytest.mark.integration` - Full robot integration tests
- `@pytest.mark.cartesian` - Cartesian movement tests
- `@pytest.mark.realtime` - Real-time control tests
- `@pytest.mark.monitoring` - Monitoring and state tests
- `@pytest.mark.slow` - Long-running tests

## Prerequisites

1. **Robot Connection**: Tests require a Fairino robot connected at IP `192.168.58.2`
2. **Robot State**: Robot should be in a safe position and operational
3. **Python Environment**: Install required dependencies:
   ```bash
   pip install pytest numpy
   ```

## Running Tests

### Basic Test Execution
```bash
# Run all tests
pytest

# Run with verbose output
pytest -v

# Run specific test file
pytest test_basic_connection.py

# Run specific test
pytest test_basic_connection.py::TestBasicConnection::test_robot_connection
```

### Selective Test Execution
```bash
# Run only fast tests (exclude slow tests)
pytest -m "not slow"

# Run only cartesian movement tests
pytest -m cartesian

# Run only monitoring tests
pytest -m monitoring

# Run integration tests
pytest -m integration
```

### Test Results and Reporting
```bash
# Generate detailed test report
pytest --tb=short -v

# Run tests with coverage (if pytest-cov installed)
pytest --cov=fairino

# Generate HTML report
pytest --html=report.html --self-contained-html
```

## Test Fixtures

The test suite provides several fixtures for common robot operations:

- `robot_connection` - Establishes robot connection with cleanup
- `current_tcp_pose` - Gets current TCP position using GetActualTCPPose
- `current_joint_positions` - Gets current joint positions
- `robot_helpers` - Helper utilities for common operations

## Key Test Features

### Position Method Validation
Tests validate the fix for the GetActualTCPPose vs GetForwardKin discrepancy:
- Ensures scripts use the same position method as webApp
- Validates accuracy of position retrieval
- Tests compatibility with OpenTeach integration

### Movement Accuracy
Tests verify movement accuracy and precision:
- Position tracking within tolerance
- Repeatability testing
- Different velocity settings

### Real-time Control
Tests validate real-time control capabilities:
- Command response times
- High-frequency control
- Concurrent monitoring

### Monitoring Capabilities
Tests verify monitoring functionality:
- TCP position monitoring
- Joint state monitoring
- Continuous monitoring reliability

## Troubleshooting

### Common Issues

1. **Robot Connection Failed**
   - Check robot IP address (default: 192.168.58.2)
   - Verify robot is powered on and network accessible
   - Ensure robot is in operational state

2. **Movement Tests Fail**
   - Check robot is enabled and in manual mode
   - Verify robot is in safe position
   - Check workspace limits

3. **Position Method Errors**
   - Verify robot firmware supports GetActualTCPPose
   - Check XML-RPC access is available
   - Ensure proper coordinate system configuration

### Test Skipping
Tests automatically skip if:
- Robot connection fails
- Required functionality not available
- Robot not in proper state

## Integration with OpenTeach

The test suite validates compatibility with OpenTeach framework:
- NumPy array handling
- Position retrieval methods
- Movement commands
- Real-time control capabilities

## Safety Considerations

- Tests use small movements (typically < 20mm)
- Automatic return to start position
- Error handling and recovery
- Safe velocity limits

## Contributing

When adding new tests:
1. Use appropriate markers (@pytest.mark.*)
2. Include proper fixtures
3. Add cleanup/return to start position
4. Use helper functions for common operations
5. Include appropriate assertions and error messages