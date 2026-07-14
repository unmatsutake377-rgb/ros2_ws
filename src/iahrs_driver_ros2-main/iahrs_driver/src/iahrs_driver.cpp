#include <chrono>
#include <memory>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "ublox_msgs/msg/nav_pvt.hpp"   // GPS heading

#define SERIAL_PORT "/dev/IMU"
#define SERIAL_SPEED B115200

typedef struct IMU_DATA
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
} IMU_DATA;
IMU_DATA imu_data;

int serial_fd = -1;
std::atomic<bool> stop_requested(false);

double yaw_initial_offset = 0.0;        // 부팅 시 offset
double gps_heading = NAN;               // 최신 GPS heading
double gps_heading_confidence = 30.0;   // GPS acc threshold

void signal_handler(int signal)
{
  stop_requested = true;
  printf("Signal received: %d. Preparing to shutdown...\n", signal);
}

class IAHRS : public rclcpp::Node
{
public:
  IAHRS() : Node("iahrs_driver")
  {
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    yaw_pub = this->create_publisher<std_msgs::msg::Float64>("/imu/yaw", 10);

    // GPS NAV-PVT 구독
    gps_sub = this->create_subscription<ublox_msgs::msg::NavPVT>(
        "/ublox/navpvt", 10,
        std::bind(&IAHRS::gps_callback, this, std::placeholders::_1));
  }

  // GPS callback → heading 저장
  void gps_callback(const ublox_msgs::msg::NavPVT::SharedPtr msg)
  {
    double acc_deg = msg->head_acc / 100000.0;   // rad 이었음 → deg 변환
    if (acc_deg < gps_heading_confidence)
    {
      gps_heading = msg->heading / 100000.0;     // deg
      gps_heading = fmod(gps_heading + 360.0, 360.0);
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr gps_sub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub;

  int serial_open()
  {
    printf("Try to open serial: %s\n", SERIAL_PORT);
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd < 0)
    {
      perror("open");
      return -1;
    }

    struct termios tio{};
    tcgetattr(serial_fd, &tio);
    cfmakeraw(&tio);
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_iflag &= ~(IXON | IXOFF);
    cfsetspeed(&tio, SERIAL_SPEED);
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN] = 0;
    tcsetattr(serial_fd, TCSAFLUSH, &tio);
    return 0;
  }

  static unsigned long GetTickCount()
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
  }

  int SendRecv(const char* cmd, double* data, int len)
  {
    char tmp[256];
    read(serial_fd, tmp, 256);

    write(serial_fd, cmd, strlen(cmd));

    char buf[1024];
    int recv_len = 0;
    unsigned long start = GetTickCount();

    while (recv_len < 1024)
    {
      int n = read(serial_fd, buf + recv_len, 1024 - recv_len);
      if (n > 0)
      {
        recv_len += n;
        if (buf[recv_len - 1] == '\r' || buf[recv_len - 1] == '\n') break;
      }
      else usleep(1000);

      if (GetTickCount() - start > 30) break;
    }

    if (recv_len <= 0) return 0;
    buf[recv_len] = '\0';

    char* p = strchr(buf, '=');
    if (!p) return 0;
    p++;

    for (int i = 0; i < len; i++)
    {
      data[i] = strtod(p, &p);
      if (*p != ',') break;
      p++;
    }

    return len;
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IAHRS>();
  if (node->serial_open() < 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open IMU serial port!");
    return -1;
  }

  rclcpp::WallRate loop_rate(100);

  bool initial_set = false;

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    double data[10];
    int n = node->SendRecv("e\n", data, 10);
    if (n >= 3)
    {
      imu_data.roll  = data[0];
      imu_data.pitch = data[1];

      //
      // ① IMU 기본 yaw(C-CW 증가) → 시계방향 증가(CW)로 변환
      //
      double imu_yaw_ccw = fmod(data[2] + 360.0, 360.0);    // 원본
      double imu_yaw_cw  = fmod(-imu_yaw_ccw + 360.0, 360.0); // 방향 뒤집기

      //
      // ② 부팅 offset 적용
      //
      if (!initial_set)
      {
        yaw_initial_offset = imu_yaw_cw;
        initial_set = true;
      }

      double yaw_corrected = imu_yaw_cw - yaw_initial_offset;
      yaw_corrected = fmod(yaw_corrected + 360.0, 360.0);

      //
      // ③ GPS heading이 유효하면 GPS heading으로 override
      //
      if (!std::isnan(gps_heading))
      {
        yaw_corrected = gps_heading;
      }

      //
      // ④ Yaw publish
      //
      std_msgs::msg::Float64 yaw_msg;
      yaw_msg.data = yaw_corrected;
      node->yaw_pub->publish(yaw_msg);
    }

    loop_rate.sleep();
  }

  close(serial_fd);
  rclcpp::shutdown();
  return 0;
}
