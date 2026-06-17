# a2_utils

General-purpose utility nodes and libraries for the A2 robot. Hardware and simulation agnostic.

## Safety FSM

The safety FSM (`ModeFsm` / `SafeVelocityRosInterface`) enforces a strict operating mode state machine and velocity limits before any commands reach the robot.

### Operating modes

| Mode | Value | Description |
|------|-------|-------------|
| `ESTOP` | 0 | Joints disabled — most extreme emergency stop |
| `STAND_DOWN` | 1 | Robot stands down, joints locked |
| `STAND_UP` | 2 | Robot stands up, joints locked |
| `BALANCE_STAND` | 3 | Robot stands up, joints unlocked |
| `VELOCITY_MOVE` | 4 | Robot moves with velocity commands |
| `FREE` | 5 | Stops the robot without disabling anything |

### Allowed transitions

```
ESTOP        → STAND_DOWN
FREE         → STAND_DOWN  (or back to prev mode before FREE was entered)
STAND_DOWN   → STAND_UP
STAND_UP     → BALANCE_STAND
STAND_UP     → STAND_DOWN
BALANCE_STAND→ VELOCITY_MOVE
BALANCE_STAND→ STAND_UP
VELOCITY_MOVE→ BALANCE_STAND

Any mode     → ESTOP   (always allowed)
Any mode     → FREE    (always allowed)
```

The FSM resets `cmd_vel` to zero on every mode transition.

### ROS interface

`SafeVelocityRosInterface` wraps `ModeFsm` with a ROS 2 interface. It is an abstract base class — subclasses implement `onControl()` to drive hardware or simulation.

**Subscribed topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/a2/mode` | `a2_interfaces/msg/OperatingMode` | Mode transition requests |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity commands (stamped for age checking) |

**Control loop**

`onControl()` is called on a fixed timer. It fires on every tick while in `VELOCITY_MOVE`, and on the first tick after any mode transition. It is a no-op otherwise.

```cpp
virtual void onControl(OpMode mode, bool mode_changed, std::array<float, 3> vel) = 0;
// vel = {x, y, yaw}, already clamped to max limits
```

### What can be modified

**Max velocities** — passed to `SafeVelocityRosInterface` (and forwarded to `ModeFsm`) at construction time. Incoming `cmd_vel` values are clamped symmetrically to `[-max, +max]` on each axis before being stored. Commands arriving outside `VELOCITY_MOVE` mode are silently zeroed.

```cpp
SafeVelocityRosInterface(
    float max_vel_x,          // m/s forward/backward
    float max_vel_y,          // m/s lateral
    float max_yaw_rate,       // rad/s
    std::chrono::milliseconds control_period,
    int64_t cmd_vel_max_age_ns
);
```

**Control rate** — set via `control_period`. The default used in the implementation is `20 ms` (50 Hz). Pass a different `std::chrono::milliseconds` value to change the rate.

**cmd_vel staleness window** — `cmd_vel_max_age_ns` is the maximum age (in nanoseconds) of an incoming `TwistStamped` message. Messages older than this threshold are dropped with a warning. The stamp is taken from the message header.

### Thread safety

`ModeFsm` itself is **not** thread-safe — callers must hold a lock. `SafeVelocityRosInterface` provides a `std::mutex` (`state_mutex_`) that guards all FSM access from the mode callback, cmd_vel callback, and the control timer.

### Linking

```cmake
find_package(a2_utils REQUIRED)

# Pure C++ FSM (no ROS dependency)
target_link_libraries(my_target a2_mode_fsm)

# ROS interface (pulls in a2_mode_fsm transitively)
target_link_libraries(my_target a2_safe_velocity_ros_interface)
ament_target_dependencies(my_target rclcpp geometry_msgs a2_interfaces)
```

## Utility nodes

| Executable | Description |
|------------|-------------|
| `registered_scan_pub` | Publishes registered LiDAR scans |
| `joint_states_pub` | Publishes joint states from Unitree HG |
| `imu_pub` | Publishes IMU data from Unitree HG |
| `cmd_vel_sport_relay` | Relays `cmd_vel` to the Unitree sport API |
