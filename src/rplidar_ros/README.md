<!-- ─────────────────────────────────────────────────────────────
  🚨 SSF 내재화(vendoring) 기록 — 2026-07-23
  rplidar_ros (Slamtec, ros2 branch) v2.1.4 — RPLIDAR A3 드라이버

  원본 upstream 커밋: b1df5d7695b20a3e595ad6c47e3dd44f983e1227

  왜 여기 있나:
    이 패키지는 원래 **gitlink(서브모듈 포인터, mode 160000)** 로 커밋돼 있었다.
    그런데 `.gitmodules` 가 없어서 **받아올 URL 이 없었다** — 로컬엔 파일이 다 있는데
    다른 머신에서 fresh clone 하면 **빈 폴더**로 온다. 라이다·아두이노 통신 경로가
    통째로 비는 것이라 멀티머신 작업에서 반복 사고가 났다.
    → 서브모듈을 풀고 파일을 저장소에 직접 넣었다. 드라이버 자체는 **바꾸지 않았다**
      (동작 변화 0). 위 해시가 이후 업스트림 패치 대조의 유일한 단서다.

  원칙: 이 디렉터리 내부는 수정하지 않는다.
        부득이 수정하면 그 지점과 이유를 이 블록 아래에 적을 것.
────────────────────────────────────────────────────────────── -->

# SLAMTEC LIDAR ROS2 Package

ROS2 node for SLAMTEC LIDAR

Visit following Website for more details about SLAMTEC LIDAR:

SLAMTEC LIDAR roswiki: <http://wiki.ros.org/rplidar>

SLAMTEC LIDAR HomePage: <http://www.slamtec.com/en/Lidar>

SLAMTEC LIDAR SDK: <https://github.com/Slamtec/rplidar_sdk>

SLAMTEC LIDAR Tutorial: <https://github.com/robopeak/rplidar_ros/wiki>

## Supported SLAMTEC LIDAR

| Lidar Model |
| ---------------------- |
|RPLIDAR A1              |
|RPLIDAR A2              |
|RPLIDAR A3              |
|RPLIDAR S1              |
|RPLIDAR S2              |
|RPLIDAR S2E             |
|RPLIDAR S3              |
|RPLIDAR T1              |
|RPLIDAR C1              |

## How to install ROS2

[rolling](https://docs.ros.org/en/rolling/Installation.html),
[humble](https://docs.ros.org/en/humble/Installation.html),
[galactic](https://docs.ros.org/en/galactic/Installation.html),
[foxy](https://docs.ros.org/en/foxy/Installation.html)

## How to configuring your ROS 2 environment

[Configuring your ROS 2 environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)

## How to Create a ROS2 workspace

[ROS2 Tutorials Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)

1. example, choose the directory name ros2_ws, for "development workspace" :

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

## Compile & Install rplidar_ros package

1. Clone rplidar_ros package from github

   Ensure you're still in the ros2_ws/src directory before you clone:

   ```bash
   git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
   ```

2. Build rpidar_ros package

   From the root of your workspace (ros2_ws), you can now build rplidar_ros package using the command:

   ```bash
   cd ~/ros2_ws/
   source /opt/ros/<rosdistro>/setup.bash
   colcon build --symlink-install
   ```

   if you find output like "colcon:command not found",you need separate [install colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon) build tools.

3. Package environment setup

    ```bash
    source ./install/setup.bash
    ```

    Note: Add permanent workspace environment variables.
    It's convenientif the ROS2 environment variables are automatically added to your bash session every time a new shell is launched:

    ```bash
    $echo "source <your_own_ros2_ws>/install/setup.bash" >> ~/.bashrc
    $source ~/.bashrc
    ```

4. Create udev rules for rplidar

   rplidar_ros running requires the read and write permissions of the serial device.
   You can manually modify it with the following command:

   ```bash
   sudo chmod 777 /dev/ttyUSB0
   ```

   But a better way is to create a udev rule:

   ```bash
   cd src/rpldiar_ros/
   source scripts/create_udev_rules.sh
   ```

## Run rplidar_ros

### Run rplidar node and view in the rviz

The command for RPLIDAR A1 is :

```bash
ros2 launch rplidar_ros view_rplidar_a1_launch.py
```

The command for RPLIDAR A2M7 is :

```bash
ros2 launch rplidar_ros view_rplidar_a2m7_launch.py
```

The command for RPLIDAR A2M8 is :

```bash
ros2 launch rplidar_ros view_rplidar_a2m8_launch.py
```

The command for RPLIDAR A2M12 is :

```bash
ros2 launch rplidar_ros view_rplidar_a2m12_launch.py
```

The command for RPLIDAR A3 is :

```bash
ros2 launch rplidar_ros view_rplidar_a3_launch.py
```

The command for RPLIDAR S1 is :

```bash
ros2 launch rplidar_ros view_rplidar_s1_launch.py
```

The command for RPLIDAR S1(TCP connection) is :

```bash
ros2 launch rplidar_ros view_rplidar_s1_tcp_launch.py
```

The command for RPLIDAR S2 is :

```bash
ros2 launch rplidar_ros view_rplidar_s2_launch.py
```

The command for RPLIDAR S2E is :

```bash
ros2 launch rplidar_ros view_rplidar_s2e_launch.py
```

The command for RPLIDAR S3 is :

```bash
ros2 launch rplidar_ros view_rplidar_s3_launch.py
```

The command for RPLIDAR T1 is :

```bash
ros2 launch rplidar_ros view_rplidar_t1_launch.py
```

The command for RPLIDAR C1 is :

```bash
ros2 launch rplidar_ros view_rplidar_c1_launch.py
```

Notice: different lidar use different serial_baudrate.

## RPLIDAR frame

RPLIDAR frame must be broadcasted according to picture shown in rplidar-frame.png
