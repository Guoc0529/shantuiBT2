# Behavior Tree Testing Guide

## Test Scripts

### 1. Comprehensive Test Script (`test_behavior_tree_comprehensive.py`)

This script performs comprehensive testing of all behavior tree branches and functionality.

#### Features:
- **Test 1**: Initial state check
- **Test 2**: End task branch (highest priority)
- **Test 3**: Pause task branch (medium priority)
- **Test 4**: Work branch via service call
- **Test 5**: Work branch via topic message
- **Test 6**: Complete workflow (one full cycle)
- **Test 7**: Idle branch (no tasks)

#### Usage:

```bash
# Run all tests
rosrun autonomous_loader_bt test_behavior_tree_comprehensive.py

# Run specific test (1-7)
rosrun autonomous_loader_bt test_behavior_tree_comprehensive.py --test 1

# Run all tests (explicit)
rosrun autonomous_loader_bt test_behavior_tree_comprehensive.py --all
```

#### What it tests:

1. **Initial State**: Verifies system starts in IDLE state
2. **End Task Branch**: Tests that end task command triggers navigation to parking
3. **Pause Task Branch**: Tests that pause task command triggers navigation pause
4. **Work Branch (Service)**: Tests task delivery via `/liaodou` service
5. **Work Branch (Topic)**: Tests task delivery via `/scheduler/start_task` topic
6. **Complete Workflow**: Tests full task cycle (raise arm → navigate → scoop → dump)
7. **Idle Branch**: Tests system behavior when no tasks are present

#### Expected Output:

The script will:
- Automatically simulate all module responses (arm raised, bucket lowered, scoop completed, navigation, etc.)
- Monitor workstate changes
- Verify parameter settings (canggoal, workstate)
- Print a summary of test results

### 2. Full Workflow Test Script (`test_full_workflow.py`)

Original comprehensive workflow test with interactive mode.

#### Usage:

```bash
# Automatic mode
rosrun autonomous_loader_bt test_full_workflow.py

# Interactive mode
rosrun autonomous_loader_bt test_full_workflow.py --interactive
```

## Running Tests

### Step 1: Start the Behavior Tree Node

```bash
# Terminal 1: Start ROS master (if not already running)
roscore

# Terminal 2: Launch the behavior tree node
roslaunch autonomous_loader_bt autonomous_loader_bt.launch
```

### Step 2: Run Test Script

```bash
# Terminal 3: Run comprehensive test
rosrun autonomous_loader_bt test_behavior_tree_comprehensive.py
```

## Test Coverage

### ✅ What is Tested:

1. **Priority System**:
   - End task command (highest priority)
   - Pause task command (medium priority)
   - Normal work tasks (lowest priority)

2. **Task Delivery**:
   - Service call (`/liaodou`)
   - Topic message (`/scheduler/start_task`)

3. **Complete Workflow**:
   - Raise arm command
   - Navigation to bin center
   - Navigation to scoop point
   - Lower bucket command
   - Scoop command
   - Navigation to hopper
   - Dump process

4. **State Management**:
   - Workstate transitions (0=IDLE, 1=RUNNING, 2=PAUSED, 3=ENDING, 4=ENDED)
   - Parameter updates (canggoal, workstate)

5. **Module Simulation**:
   - All module responses are automatically simulated
   - Navigation distance simulation
   - Timing delays for realistic behavior

## Troubleshooting

### If tests fail:

1. **Check ROS topics**: Ensure all topics are properly connected
   ```bash
   rostopic list
   rostopic echo /workstate
   ```

2. **Check parameters**: Verify parameters are set correctly
   ```bash
   rosparam list
   rosparam get /autonomous_loader_bt_node/workstate
   ```

3. **Check service**: Ensure service is available
   ```bash
   rosservice list | grep liaodou
   rosservice info /liaodou
   ```

4. **Check logs**: Look for error messages in the behavior tree node output

## Expected Test Results

When all tests pass, you should see:

```
============================================================
TEST SUMMARY
============================================================
✓ Initial workstate ✓ PASS
✓ No commands received initially ✓ PASS
✓ End task triggers navigation to parking ✓ PASS
...
✓ ALL TESTS PASSED!
============================================================
```

