# Silvanus Robot Base Teleoperation Guide

## Requirements

- Ubuntu 24.04 laptop with ROS 2 Jazzy.
- Laptop connected to the robot's Wi-Fi.
- Robot computer address: `192.168.1.102`.
- Robot ROS domain: `42`.
- Physical emergency stop accessible.
- Clear operating area around the robot.
- Only one laptop should publish movement commands.

Install keyboard teleoperation on the laptop:

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

## 1. Connect to the Robot

From the laptop:

```bash
ping -c 3 192.168.1.102
ssh agilex@192.168.1.102
```

Enter the robot password when requested.

## 2. Check the Robot Before Starting

On the robot computer through SSH:

```bash
cd /opt/silvanus
```

Confirm no old ROS 1 system is running:

```bash
pgrep -af 'rosmaster|roscore|roslaunch'
```

This command should produce no output.

Run the preflight checks:

```bash
./test_robot.sh preflight
```

Check the CAN interface:

```bash
ip -details -statistics link show can0
```

Expected:

- Interface is `UP`.
- CAN state is `ERROR-ACTIVE`.
- Bitrate is `500000`.
- Interface is not `BUS-OFF`.

Optionally check that CAN messages are arriving:

```bash
timeout 5 candump -n 10 can0
```

## 3. Prepare the Physical Robot

Before enabling the base:

1. Clear the area around the chassis.
2. Keep the remote-control sticks neutral.
3. Keep the emergency stop within reach.
4. Initially engage the chassis emergency stop.
5. Ensure the arm is secured in a safe pose.
6. Remove any object held by the gripper.
7. Ensure nobody is close to the wheels.
8. Stop any other software controlling the robot.

For a standard Scout remote:

- SWB middle: handheld control.
- SWB top: CAN/computer control.

Confirm the switch mapping for the specific robot before moving it.

## 4. Start the Base Driver

Start only the robot description and Scout base driver:

```bash
cd /opt/silvanus

./silvanus-up.sh \
  --no-lidar \
  --no-camera \
  --detach
```

This enables:

- Scout base driver
- `/cmd_vel`
- `/odom`
- `/scout_status`
- Robot transforms

It does not enable:

- LiDAR
- Camera
- Arm
- Gripper
- Xsens

The startup summary may show `gripper=true`, but the gripper is not started while `arm=false`.

## 5. Inspect the Driver Logs

```bash
docker logs --since 2m --tail 150 silvanus
```

For continuous logs:

```bash
docker logs -f silvanus
```

Press `Ctrl-C` to leave the log viewer. This does not stop a detached container.

## 6. Validate the Base

```bash
cd /opt/silvanus

./test_robot.sh live \
  --base \
  --no-lidar \
  --no-camera
```

The following checks must pass:

- `/joint_states`
- `/odom`
- `/scout_status`
- `odom -> base_footprint` transform

Also verify:

- Battery voltage is positive and reasonable.
- Scout error code is `0`.
- Odometry is updating.
- No unexpected movement occurs.

## 7. Configure ROS 2 on the Laptop

Open a new terminal on the laptop, not through SSH:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=192.168.1.102

ros2 daemon stop
ros2 topic list --no-daemon --spin-time 5
```

Expected topics include:

```text
/cmd_vel
/joint_states
/odom
/robot_description
/scout_status
/tf
/tf_static
```

## 8. Verify the Velocity Interface

On the laptop:

```bash
ros2 topic info /cmd_vel --verbose
```

Expected:

```text
Type: geometry_msgs/msg/Twist
Subscription count: 1
```

The physical Scout uses `geometry_msgs/msg/Twist`. Do not use `TwistStamped`, which is used by the Gazebo simulator.

## 9. Start Low-Speed Keyboard Teleoperation

Before starting:

1. Keep the chassis emergency stop within reach.
2. Place the Scout remote in CAN/computer-control mode.
3. Clear the movement area.
4. Ensure only one teleoperation publisher is running.

Start teleoperation from the laptop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=false \
  -p speed:=0.10 \
  -p turn:=0.20 \
  -p repeat_rate:=10.0 \
  -p key_timeout:=0.5
```

After the teleoperation program starts, release the chassis emergency stop according to the approved procedure.

## 10. Keyboard Controls

```text
u    i    o
j    k    l
m    ,    .
```

| Key | Action |
|---|---|
| `i` | Move forward |
| `,` | Move backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `k` | Stop |
| `u` | Forward-left curve |
| `o` | Forward-right curve |
| `m` | Backward-left curve |
| `.` | Backward-right curve |

## 11. Initial Movement Test

Test one movement at a time:

1. Briefly press `i`.
2. Immediately press `k`.
3. Confirm that the robot stops.
4. Briefly press `,`.
5. Immediately press `k`.
6. Briefly press `j`.
7. Immediately press `k`.
8. Briefly press `l`.
9. Immediately press `k`.

During the test, verify:

- The robot moves in the expected direction.
- It stops when `k` is pressed.
- The emergency stop works.
- No arm or gripper movement occurs.
- No error appears in `/scout_status`.
- Odometry direction matches physical movement.

Do not increase the speed until the low-speed test passes.

## 12. Optional Monitoring

In another laptop terminal, configure ROS discovery again:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=192.168.1.102
```

Monitor commands:

```bash
ros2 topic echo /cmd_vel
```

Monitor odometry:

```bash
ros2 topic hz /odom
```

Inspect the current robot status:

```bash
ros2 topic echo /scout_status --once
```

## 13. Safe Shutdown

1. Press `k` in the teleoperation terminal.
2. Confirm that the chassis is completely stationary.
3. Engage the physical chassis emergency stop.
4. Press `Ctrl-C` to stop keyboard teleoperation.
5. SSH into the robot if necessary:

```bash
ssh agilex@192.168.1.102
```

6. Stop the ROS 2 hardware container:

```bash
docker stop --time 30 silvanus
```

7. Verify that it stopped:

```bash
docker ps --filter name=silvanus
```

Stopping Docker, closing SSH, or losing Wi-Fi is not an emergency stop or power isolation.

## Troubleshooting

### Robot topics are not visible on the laptop

Check the connection:

```bash
ping -c 3 192.168.1.102
```

Reset the laptop environment:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=192.168.1.102
ros2 daemon stop
ros2 topic list --no-daemon --spin-time 5
```

### `/cmd_vel` has no subscriber

Confirm the base container is running:

```bash
ssh agilex@192.168.1.102
docker ps --filter name=silvanus
```

Restart the base if necessary:

```bash
cd /opt/silvanus

./silvanus-up.sh \
  --no-lidar \
  --no-camera \
  --detach
```

### Robot does not move

Check:

- The chassis emergency stop is released.
- The remote is in CAN/computer-control mode.
- `/cmd_vel` has one subscriber.
- Base validation passes.
- No other controller is publishing commands.
- Scout error code is zero.

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /scout_status --once
```

### Keyboard package is missing

Install it on the laptop:

```bash
sudo apt update
sudo apt install ros-jazzy-teleop-twist-keyboard
```

The package does not need to be installed inside the Silvanus Docker container.

### Stop Immediately

Use the physical chassis emergency stop. Do not rely on Wi-Fi, SSH, Docker, or keyboard commands for emergency stopping.
