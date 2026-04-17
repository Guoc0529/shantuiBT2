# Quick Start Testing Guide

## Prerequisites

Make sure ROS master is running:
```bash
roscore
```

## Step 1: Start the Behavior Tree Node

```bash
# Terminal 1: Launch the behavior tree node
roslaunch autonomous_loader_bt autonomous_loader_bt.launch
```

Wait for the node to fully start (you should see messages like "Starting autonomous loader robot behavior tree node").

## Step 2: Verify Node is Running

```bash
# Check if node is running
rosnode list | grep autonomous_loader_bt_node

# Check if service is available
rosservice list | grep liaodou

# Check topics
rostopic list | grep -E "(workstate|scheduler|work_device|scoop|navigation)"
```

## Step 3: Run Tests

```bash
# Terminal 2: Run comprehensive test
rosrun autonomous_loader_bt test_behavior_tree_comprehensive.py

# Or run specific test
rosrun autonomous_loader_bt test_behavior_tree_comprehensive.py --test 6
```

## Troubleshooting

### If service `/liaodou` is not found:

1. **Check if node is running:**
   ```bash
   rosnode list
   ```

2. **Check node status:**
   ```bash
   rosnode info /autonomous_loader_bt_node
   ```

3. **Check for errors in node output:**
   - Look at the terminal where you launched the node
   - Check for any error messages or crashes

4. **Restart the node:**
   ```bash
   # Kill existing node
   rosnode kill /autonomous_loader_bt_node
   
   # Restart
   roslaunch autonomous_loader_bt autonomous_loader_bt.launch
   ```

### If tests still fail:

The test script will now automatically fall back to using topic messages instead of service calls if the service is not available.

## Quick Test (No Service Required)

If you just want to test basic functionality without the service:

```bash
# Terminal 1: Start node
roslaunch autonomous_loader_bt autonomous_loader_bt.launch

# Terminal 2: Send task via topic (simple test)
rostopic pub /scheduler/start_task autonomous_loader_msgs/TaskCommand "task_id: 1
bin_id: 1
hopper_id: 1
task_type: 'scoop'" --once
```

