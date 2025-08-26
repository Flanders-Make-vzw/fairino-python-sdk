"""
Pytest tests for robot movement timing and execution delays.
Analyzes delays in movement commands to identify potential inverse kinematics bottlenecks.
"""

import pytest
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
from datetime import datetime
from fairino import Robot


def create_timing_output_dir(test_name="timing_test"):
    """Create output directory for timing test results with descriptive name"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"{test_name}_{timestamp}"
    os.makedirs(output_dir, exist_ok=True)
    return output_dir


def save_timing_data(data, filename, output_dir):
    """Save timing data to CSV file"""
    df = pd.DataFrame(data)
    filepath = os.path.join(output_dir, f"{filename}.csv")
    df.to_csv(filepath, index=False)
    return filepath



@pytest.mark.timing
@pytest.mark.integration
class TestTimingSummary:
    """Timing analysis"""
    
    def test_create_timing_summary_report(self, robot_connection, neutral_joints, robot_helpers):
        """Create a timing summary report with comparing the MoveL, MoveJ and status check timingss"""
        output_dir = create_timing_output_dir("timing_summary")
        
        print("\n🔍 Creating timing summary report...")
        print(f"   Results will be saved to: {output_dir}")
        
        # Collect timing data from different movement types
        timing_summary = {
            'test_type': [],
            'avg_time': [],
            'max_time': [],
            'min_time': [],
            'std_dev': [],
            'sample_count': []
        }
        
        # Quick timing tests for summary
        robot = robot_connection
        
        # Test 1: Joint movements (20 samples with randomized patterns up to 45°)
        print("   Testing joint movements for summary...")
        joint_times = []
        import random
        random.seed(42)  # For reproducible randomization
        
        for i in range(20):
            # Generate randomized movements with different patterns
            target = neutral_joints.copy()
            
            # Random movement patterns
            pattern = i % 5
            if pattern == 0:  # Single joint large movement
                joint_idx = random.randint(0, 5)
                angle = random.uniform(-45, 45)
                target[joint_idx] += angle
                
            elif pattern == 1:  # Two joints moderate movement
                joint_indices = random.sample(range(6), 2)
                for idx in joint_indices:
                    angle = random.uniform(-30, 30)
                    target[idx] += angle
                    
            elif pattern == 2:  # Three joints small movement
                joint_indices = random.sample(range(6), 3)
                for idx in joint_indices:
                    angle = random.uniform(-20, 20)
                    target[idx] += angle
                    
            elif pattern == 3:  # All joints small movement
                for j in range(6):
                    angle = random.uniform(-15, 15)
                    target[j] += angle
                    
            else:  # Mixed pattern - some large, some small
                for j in range(6):
                    if random.random() < 0.5:  # 50% chance to move this joint
                        if random.random() < 0.3:  # 30% chance for large movement
                            angle = random.uniform(-40, 40)
                        else:  # 70% chance for small movement
                            angle = random.uniform(-10, 10)
                        target[j] += angle
            
            start_time = time.time()
            ret = robot.MoveJ(target, tool=0, user=0, vel=15, blendT=0)
            elapsed = time.time() - start_time
            if ret == 0:
                joint_times.append(elapsed)
            robot_helpers.wait_for_movement(2)
            robot.MoveJ(neutral_joints, tool=0, user=0, vel=15, blendT=0)
            robot_helpers.wait_for_movement(2)
        
        # Test 2: Cartesian movements (20 samples with varying distances)
        print("   Testing cartesian movements for summary...")
        cartesian_times = []
        
        # Get current TCP pose instead of using hardcoded values
        current_pose = robot_helpers.get_tcp_pose(robot)
        if current_pose is None:
            print("   Warning: Could not get current TCP pose, skipping cartesian tests")
            cartesian_times = []
        else:
            base_cartesian = current_pose
            
            for i in range(20):
                # Generate randomized cartesian movements with different patterns
                target = base_cartesian.copy()
                
                # Random movement patterns for cartesian space
                pattern = i % 5
                if pattern == 0:  # Single axis large movement
                    axis_idx = random.randint(0, 2)  # Position axes (X, Y, Z)
                    movement = random.uniform(-30, 30)
                    target[axis_idx] += movement
                    
                elif pattern == 1:  # Two position axes combined
                    axes_indices = random.sample(range(3), 2)  # Two of X, Y, Z
                    for idx in axes_indices:
                        movement = random.uniform(-20, 20)
                        target[idx] += movement
                        
                elif pattern == 2:  # Three position axes combined
                    for idx in range(3):  # X, Y, Z
                        movement = random.uniform(-15, 15)
                        target[idx] += movement
                        
                elif pattern == 3:  # Position + orientation combined
                    # Small position changes
                    for idx in range(3):
                        movement = random.uniform(-10, 10)
                        target[idx] += movement
                    # Small orientation changes
                    for idx in range(3, 6):
                        if random.random() < 0.7:  # 70% chance to change this orientation
                            angle = random.uniform(-15, 15)
                            target[idx] += angle
                            
                else:  # Complex mixed pattern
                    # Position movements
                    for idx in range(3):
                        if random.random() < 0.8:  # 80% chance to move this axis
                            if random.random() < 0.4:  # 40% chance for large movement
                                movement = random.uniform(-25, 25)
                            else:  # 60% chance for small movement
                                movement = random.uniform(-8, 8)
                            target[idx] += movement
                    
                    # Orientation movements
                    for idx in range(3, 6):
                        if random.random() < 0.5:  # 50% chance to change orientation
                            if random.random() < 0.3:  # 30% chance for larger rotation
                                angle = random.uniform(-20, 20)
                            else:  # 70% chance for small rotation
                                angle = random.uniform(-5, 5)
                            target[idx] += angle
                
                start_time = time.time()
                ret = robot.MoveL(target, tool=0, user=0, vel=15, blendR=0)
                elapsed = time.time() - start_time
                if ret == 0:
                    cartesian_times.append(elapsed)
                robot_helpers.wait_for_movement(2)
                robot.MoveL(base_cartesian, tool=0, user=0, vel=15, blendR=0)
                robot_helpers.wait_for_movement(2)
        
        # Test 3: Status check timing
        print("   Testing motion status checks for summary...")
        status_times = []
        for i in range(50):  # Keep 50 samples for status checks
            start_time = time.time()
            try:
                ret, is_done = robot.GetRobotMotionDone()
                elapsed = time.time() - start_time
                status_times.append(elapsed)
            except:
                status_times.append(0.001)
            time.sleep(0.05)
        
        # Compile summary data
        test_data = [
            ("Joint Movements (MoveJ)", joint_times),
            ("Cartesian Movements (MoveL)", cartesian_times),
            ("Motion Status Checks", status_times)
        ]
        
        for test_name, times in test_data:
            if times:
                timing_summary['test_type'].append(test_name)
                timing_summary['avg_time'].append(np.mean(times))
                timing_summary['max_time'].append(np.max(times))
                timing_summary['min_time'].append(np.min(times))
                timing_summary['std_dev'].append(np.std(times))
                timing_summary['sample_count'].append(len(times))
        
        # Save summary to CSV
        summary_csv = save_timing_data([timing_summary], "timing_summary_report", output_dir)
        print(f"   📄 Summary report saved to: {summary_csv}")
        
        # Create summary graphs
        plt.figure(figsize=(16, 12))
        
        # Plot 1: Average timing comparison
        plt.subplot(2, 3, 1)
        test_names = timing_summary['test_type']
        avg_times = timing_summary['avg_time']
        bars = plt.bar(test_names, avg_times, alpha=0.7, color=['#1f77b4', '#ff7f0e', '#2ca02c'])
        plt.title('Average Timing Comparison', fontweight='bold')
        plt.ylabel('Average Time (seconds)')
        plt.xticks(rotation=45, ha='right')
        for bar, time_val in zip(bars, avg_times):
            plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.0001,
                    f'{time_val:.4f}s', ha='center', va='bottom', fontsize=8)
        
        # Plot 2: Max vs Min comparison
        plt.subplot(2, 3, 2)
        x_pos = np.arange(len(test_names))
        plt.bar(x_pos - 0.2, timing_summary['max_time'], 0.4, label='Max Time', alpha=0.7)
        plt.bar(x_pos + 0.2, timing_summary['min_time'], 0.4, label='Min Time', alpha=0.7)
        plt.title('Max vs Min Times')
        plt.ylabel('Time (seconds)')
        plt.xticks(x_pos, test_names, rotation=45, ha='right')
        plt.legend()
        
        # Plot 3: Standard deviation
        plt.subplot(2, 3, 3)
        plt.bar(test_names, timing_summary['std_dev'], alpha=0.7, color='red')
        plt.title('Timing Variability (Std Dev)')
        plt.ylabel('Standard Deviation (seconds)')
        plt.xticks(rotation=45, ha='right')
        
        # Plot 4: Sample counts
        plt.subplot(2, 3, 4)
        plt.bar(test_names, timing_summary['sample_count'], alpha=0.7, color='green')
        plt.title('Sample Counts')
        plt.ylabel('Number of Samples')
        plt.xticks(rotation=45, ha='right')
        
        # Plot 5: Detailed timing distribution for each test
        plt.subplot(2, 3, 5)
        all_times = []
        labels = []
        for test_name, times in test_data:
            if times:
                all_times.extend(times)
                labels.extend([test_name] * len(times))
        
        # Create box plot
        unique_tests = list(set(labels))
        box_data = []
        for test in unique_tests:
            test_times = [all_times[i] for i, label in enumerate(labels) if label == test]
            box_data.append(test_times)
        
        plt.boxplot(box_data, labels=unique_tests)
        plt.title('Timing Distribution Box Plot')
        plt.ylabel('Time (seconds)')
        plt.xticks(rotation=45, ha='right')
        
        # Plot 6: Performance summary
        plt.subplot(2, 3, 6)
        plt.text(0.1, 0.9, "🎯 Performance Summary", fontsize=14, fontweight='bold', transform=plt.gca().transAxes)
        summary_text = ""
        for i, test_name in enumerate(test_names):
            avg_time = avg_times[i]
            max_time = timing_summary['max_time'][i]
            summary_text += f"\n{test_name}:\n"
            summary_text += f"  Avg: {avg_time:.4f}s\n"
            summary_text += f"  Max: {max_time:.4f}s\n"
            
            # Performance rating
            if "Status" in test_name and avg_time < 0.01:
                summary_text += "  ✅ Excellent\n"
            elif "Movement" in test_name and avg_time < 0.1:
                summary_text += "  ✅ Good\n"
            elif avg_time < 0.5:
                summary_text += "  ⚠️  Acceptable\n"
            else:
                summary_text += "  ❌ Needs Improvement\n"
        
        plt.text(0.1, 0.8, summary_text, fontsize=10, transform=plt.gca().transAxes, verticalalignment='top')
        plt.axis('off')
        
        plt.tight_layout()
        summary_graph_path = os.path.join(output_dir, "timing_summary.png")
        plt.savefig(summary_graph_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"   📊 Comprehensive summary graph saved to: {summary_graph_path}")
        
        # Print final summary
        print(f"\n🎯 Final Timing Performance Summary:")
        for i, test_name in enumerate(test_names):
            print(f"   {test_name}:")
            print(f"     Average: {avg_times[i]:.4f}s")
            print(f"     Range: {timing_summary['min_time'][i]:.4f}s - {timing_summary['max_time'][i]:.4f}s")
            print(f"     Std Dev: {timing_summary['std_dev'][i]:.4f}s")
            print(f"     Samples: {timing_summary['sample_count'][i]}")
        
        print(f"\n📁 All results saved in directory: {output_dir}")
        
        # Return to neutral
        robot.MoveJ(neutral_joints, tool=0, user=0, vel=15)
        robot_helpers.wait_for_movement(3)
