# hebi_msgs

Custom ROS 2 message, service, and action definitions for HEBI Robotics platforms.

## Overview

This package provides ROS 2 interface definitions for HEBI-specific communications, including messages for robotic arm control, mobile base control (including the Tready platform), and Mobile IO device integration.

**Version**: 1.0.0
**License**: Apache License 2.0
**Maintainers**: Chris Bollinger, Hariharan Ravichandran

## Package Contents

### Messages (7)

#### Arm Control

- **[SE3Jog.msg](msg/SE3Jog.msg)** - SE3 space jogging commands
  - Linear displacement: `dx`, `dy`, `dz` (meters)
  - Angular displacement: `droll`, `dpitch`, `dyaw` (radians)
  - Duration: Command execution time
  - Includes header for timestamping

- **[SE3Trajectory.msg](msg/SE3Trajectory.msg)** - SE3 trajectory command
  - Array of `SE3TrajectoryPoint` waypoints
  - Includes header for timestamping

- **[SE3TrajectoryPoint.msg](msg/SE3TrajectoryPoint.msg)** - Individual SE3 trajectory waypoint
  - Position: `x`, `y`, `z` (meters)
  - Orientation: `roll`, `pitch`, `yaw` (radians)
  - Gripper state: 0.0 (open) to 1.0 (closed)
  - Time from start

#### Tready/Mobile Base Control

- **[TreadedBaseState.msg](msg/TreadedBaseState.msg)** - State information for treaded base platforms
  - State codes:
    - `0`: STARTUP
    - `1`: HOMING
    - `2`: ALIGNING
    - `3`: TELEOP
    - `4`: EMERGENCY_STOP
    - `5`: EXIT
  - Status flags: `flipper_trajectory_active`, `base_trajectory_active`, `stable_mode`, `mstop_pressed`
  - String message field

- **[TreadyFlipperVelocityCommand.msg](msg/TreadyFlipperVelocityCommand.msg)** - Individual flipper velocity control
  - `front_left`, `front_right`, `back_left`, `back_right` (m/s)

- **[TreadyTorqueModeCommand.msg](msg/TreadyTorqueModeCommand.msg)** - Torque mode parameters for balance control
  - `torque_max`: Maximum torque (up to 25 N-m)
  - `torque_angle`: Angle between 0 and π/2
  - `roll_adjust`, `pitch_adjust`: Adjustment values (0.0 to 1.0)

#### Mobile IO

- **[MobileInput.msg](msg/MobileInput.msg)** - Mobile IO device input
  - `button_states`: Current state of all buttons
  - `axis_states`: Current state of all axes
  - `button_diffs`: Button state changes

### Services (3)

- **[SetPluginEnabled.srv](srv/SetPluginEnabled.srv)** - Enable/disable runtime plugins
  - Request: `plugin_name` (string), `enabled` (bool)
  - Response: `success` (bool), `message` (string)

- **[SetLayoutFile.srv](srv/SetLayoutFile.srv)** - Set UI layout from file
  - Request: `layout_file_name` (string), `layout_package_name` (string)
  - Response: `success` (bool)

- **[SetLayoutJSON.srv](srv/SetLayoutJSON.srv)** - Set UI layout from JSON string
  - Request: `layout_json` (string)
  - Response: `success` (bool)

### Actions (3)

- **[ArmJointMotion.action](action/ArmJointMotion.action)** - Joint space arm motion
  - Goal: Joint trajectory waypoints, optional timing and LED color
  - Result: Success status
  - Feedback: Percent complete (0.0 to 1.0)

- **[ArmSE3Motion.action](action/ArmSE3Motion.action)** - SE3 space arm motion
  - Goal: SE3 trajectory waypoints, optional timing and LED color
  - Result: Success status
  - Feedback: Percent complete (0.0 to 1.0)

- **[BaseMotion.action](action/BaseMotion.action)** - Mobile base motion
  - Goal: Relative position (`x`, `y`, `theta`), optional LED color
  - Result: Success status
  - Feedback: Percent complete (0.0 to 1.0)

## Usage

### Building

This package is automatically built as part of the HEBI ROS 2 workspace:

```bash
cd ~/hebi_ws
colcon build --packages-select hebi_msgs
source install/setup.bash
```

### Importing in C++

```cpp
#include "hebi_msgs/msg/se3_jog.hpp"
#include "hebi_msgs/srv/set_plugin_enabled.hpp"
#include "hebi_msgs/action/arm_joint_motion.hpp"

// Example usage
auto jog_msg = hebi_msgs::msg::SE3Jog();
jog_msg.dx = 0.1;
jog_msg.dy = 0.0;
jog_msg.dz = 0.05;
jog_msg.duration = 1.0;
```

### Importing in Python

```python
from hebi_msgs.msg import SE3Jog, TreadedBaseState
from hebi_msgs.srv import SetPluginEnabled
from hebi_msgs.action import ArmJointMotion

# Example usage
jog_msg = SE3Jog()
jog_msg.dx = 0.1
jog_msg.dy = 0.0
jog_msg.dz = 0.05
jog_msg.duration = 1.0
```

## Dependencies

- `geometry_msgs` - Standard geometry messages
- `trajectory_msgs` - Standard trajectory messages
- `action_msgs` - Standard action interfaces
- `rosidl_default_generators` - Message generation (build-time)
- `rosidl_default_runtime` - Message runtime (execution-time)

## Related Packages

- [hebi_ros2_examples](../hebi_ros2_examples/) - Example nodes using these messages
- [hebi_description](../hebi_description/) - Robot descriptions
- [hebi_bringup](../hebi_bringup/) - Launch configurations
- [hebi_hardware](../hebi_hardware/) - ROS 2 Control hardware interface

## Resources

- **HEBI Documentation**: https://docs.hebi.us
- **Support**: support@hebirobotics.com
- **Forums**: https://forum.hebi.us

## Contributing

When adding new message definitions:

1. Create the `.msg`, `.srv`, or `.action` file in the appropriate directory
2. Add the filename to [CMakeLists.txt](CMakeLists.txt) in the corresponding `rosidl_generate_interfaces()` section
3. Rebuild the package: `colcon build --packages-select hebi_msgs`
4. Update this README with the new interface documentation

## License

This package is released under the Apache License 2.0. See [LICENSE](LICENSE) file for details.
