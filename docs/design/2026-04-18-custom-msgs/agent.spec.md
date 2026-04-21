# Custom Message Package — Agent Specification

## Requirements

- `ros2_bridge/pybullet_fleet_msgs/` に `ament_cmake` パッケージを新規作成
- `ExecuteAction.action` を定義
- `pybullet_fleet_ros/package.xml` に `<depend>pybullet_fleet_msgs</depend>` 追加
- Docker の `colcon build` で正しい順序でビルド

## Constraints

- `ament_cmake` のみ (Python コード不要 — メッセージ定義だけ)
- `rosidl_default_generators` でメッセージ生成
- package.xml format version 3

## Approach

最小限の `ament_cmake` パッケージを作成し、Phase 2 で使う1つの action だけ定義する。

## Design

### Package Structure

```
ros2_bridge/pybullet_fleet_msgs/
├── CMakeLists.txt
├── package.xml
└── action/
    └── ExecuteAction.action
```

### ExecuteAction.action

```
# Goal
string robot_name
string action_type          # "move" | "pick" | "drop" | "wait" | "joint"
string action_params_json   # JSON parameters
---
# Result
bool success
string message
---
# Feedback
string status               # "NOT_STARTED" | "IN_PROGRESS" | "COMPLETED" | "FAILED"
float32 progress            # 0.0 ~ 1.0
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(pybullet_fleet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteAction.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pybullet_fleet_msgs</name>
  <version>0.1.0</version>
  <description>ROS 2 message definitions for PyBulletFleet</description>
  <maintainer email="yu.okamoto@rapyuta-robotics.com">Yu Okamoto</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Docker Changes

`Dockerfile.rmf_demos` の `colcon build --packages-select` に `pybullet_fleet_msgs` を追加:

```dockerfile
COPY ros2_bridge/pybullet_fleet_msgs /rmf_demos_ws/src/pybullet_fleet_msgs
COPY ros2_bridge/pybullet_fleet_ros /rmf_demos_ws/src/pybullet_fleet_ros
WORKDIR /rmf_demos_ws
RUN source /opt/ros/jazzy/setup.bash && \
    source /rmf_demos_ws/install/setup.bash && \
    colcon build --symlink-install \
    --packages-select pybullet_fleet_msgs pybullet_fleet_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## File References

- `ros2_bridge/pybullet_fleet_ros/setup.py` — 現在の ament_python パッケージ
- `ros2_bridge/pybullet_fleet_ros/package.xml` — 依存追加先
- `docker/Dockerfile.rmf_demos` — Docker ビルド変更
- ROADMAP.md Item 10 — ExecuteAction.action の使用例

## Success Criteria

- [ ] `colcon build --packages-select pybullet_fleet_msgs` 成功
- [ ] `from pybullet_fleet_msgs.action import ExecuteAction` が Python で import 可
- [ ] `pybullet_fleet_ros` の package.xml に依存追加
- [ ] Docker ビルド成功
- [ ] `ros2 interface show pybullet_fleet_msgs/action/ExecuteAction` が表示
