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
#include <string>

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
    //
    // 🚨 CLAUDE.md 3-5 — 작년엔 이 드라이버가 /imu/yaw 를 직접 냈고, 그 값이 절대방위가 아니었다.
    //    ② 부팅 시점 뱃머리를 0 으로 만들어 '상대각' 이 됐다.
    //       그런데 north_goal_angle 은 '절대방위' 를 계산하고 ship_goal_angle 은 둘을 뺀다.
    //       → 뺄셈이 무의미했다. 배를 정북으로 놓고 켰을 때만 우연히 맞았다.
    //    ③ GPS heading 이 유효하면 IMU 를 통째로 덮어썼다.
    //       NavPVT.heading 은 'Heading of motion'(COG, 대지침로)이지 뱃머리가 아니다.
    //       정지/저속에서 COG 는 노이즈고, 조류·바람에 게걸음하면 뱃머리와 벌어진다.
    //       (게다가 g_speed 를 안 봐서 속도 0 에서도 통과했다)
    //
    //    → 둘 다 파라미터로 빼고 **기본 OFF**. 이 드라이버는 보정 안 한 상대 yaw 만 낸다.
    //      절대방위 합성은 ssf_heading/yaw_mux 가 전담한다.
    //
    //    ⚠️ yaw_topic 기본값이 /imu/yaw_raw 인 것이 중요하다.
    //       /imu/yaw 로 되돌리면 yaw_mux 와 **한 토픽에 발행자 2개**가 되어 두 값이 번갈아 나온다.
    //       에러는 안 난다 — 이 프로젝트가 반복해 당한 침묵 실패 유형이다.
    //
    zero_yaw_on_boot = this->declare_parameter<bool>("zero_yaw_on_boot", false);
    use_gps_heading_override =
        this->declare_parameter<bool>("use_gps_heading_override", false);
    yaw_topic = this->declare_parameter<std::string>("yaw_topic", "/imu/yaw_raw");

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    yaw_pub = this->create_publisher<std_msgs::msg::Float64>(yaw_topic, 10);
    // N4: 지자기 융합 절대방위(imu_yaw_cw)를 별도 토픽으로 관찰용 발행.
    //     기존 yaw 경로(/imu/yaw_raw)는 안 건드린다 → 한 토픽 발행자 2개(침묵실패) 방지.
    mag_heading_pub = this->create_publisher<std_msgs::msg::Float64>("/imu/mag_heading", 10);

    RCLCPP_INFO(this->get_logger(),
                "iahrs_driver: yaw_topic=%s, zero_yaw_on_boot=%s, gps_override=%s",
                yaw_topic.c_str(),
                zero_yaw_on_boot ? "true" : "false",
                use_gps_heading_override ? "true" : "false");
    if (yaw_topic == "/imu/yaw")
    {
      RCLCPP_WARN(this->get_logger(),
                  "yaw_topic 이 /imu/yaw 다. yaw_mux 를 함께 띄우면 발행자가 2개가 된다.");
    }

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
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mag_heading_pub;   // N4 지자기 절대방위(관찰용)

  // CLAUDE.md 3-5 — 기본 OFF. 아래 main 루프 ②③ 에서 읽는다.
  bool zero_yaw_on_boot = false;
  bool use_gps_heading_override = false;
  std::string yaw_topic = "/imu/yaw_raw";

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

      // N4: 지자기 융합 절대방위(offset·override 적용 전 원값)를 관찰용으로 발행.
      //     blackbox 가 /imu/mag_heading 을 GPS COG·yaw_raw 와 사후 대조한다.
      {
        std_msgs::msg::Float64 mag_msg;
        mag_msg.data = imu_yaw_cw;
        node->mag_heading_pub->publish(mag_msg);
      }

      //
      // ② 부팅 offset 적용 — 🚨 기본 OFF (CLAUDE.md 3-5)
      //    켜면 /imu/yaw_raw 가 '부팅 시점 뱃머리 기준 상대각' 이 된다.
      //    절대방위 보정은 yaw_mux 의 mount_offset_deg 가 담당하므로 여기선 끈다.
      //
      if (node->zero_yaw_on_boot)
      {
        if (!initial_set)
        {
          yaw_initial_offset = imu_yaw_cw;
          initial_set = true;
        }
      }
      else
      {
        yaw_initial_offset = 0.0;
      }

      double yaw_corrected = imu_yaw_cw - yaw_initial_offset;
      yaw_corrected = fmod(yaw_corrected + 360.0, 360.0);

      //
      // ③ GPS heading override — 🚨 기본 OFF (CLAUDE.md 3-5)
      //    NavPVT.heading 은 COG(대지침로)지 뱃머리가 아니다. 정지 시 노이즈고,
      //    게걸음하면 뱃머리와 벌어진다. 그 차이를 '추정' 하는 게 옵션 B(yaw_mux, N2)이고
      //    여기서 하던 건 차이를 0 이라고 '가정' 하는 것이었다.
      //
      if (node->use_gps_heading_override && !std::isnan(gps_heading))
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
