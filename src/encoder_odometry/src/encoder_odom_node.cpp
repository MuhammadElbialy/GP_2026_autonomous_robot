// 🔥 أهم تعديل: استخدام ROS clock الصحيح
// بدل now() استخدم this->get_clock()->now()

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <sstream>
#include <string>
#include <cmath>
#include <memory>

class EncoderOdomNode : public rclcpp::Node
{
public:
  EncoderOdomNode()
  : Node("encoder_odometry_node"),
    fd_(-1),
    x_(0.0), y_(0.0), theta_(0.0),
    last_left_counts_(0), last_right_counts_(0),
    first_read_(true)
  {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baud", 115200);
    declare_parameter<double>("wheel_radius", 0.075);
    declare_parameter<double>("wheel_base", 0.60);
    declare_parameter<int>("ticks_per_rev_left",  870);
    declare_parameter<int>("ticks_per_rev_right", 800);

    declare_parameter<std::string>("frame_id", "odom");
    declare_parameter<std::string>("child_frame_id", "base_link");

    port_  = get_parameter("port").as_string();
    baud_  = get_parameter("baud").as_int();
    R_     = get_parameter("wheel_radius").as_double();
    L_     = get_parameter("wheel_base").as_double();
    ticks_per_rev_left_  = get_parameter("ticks_per_rev_left").as_int();
    ticks_per_rev_right_ = get_parameter("ticks_per_rev_right").as_int();
    frame_id_       = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();

    if (!openSerial()) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", port_.c_str());
      throw std::runtime_error("Cannot open serial port");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&EncoderOdomNode::timerCallback, this));

    // 🔥 FIX
    last_time_ = this->get_clock()->now();

    RCLCPP_INFO(get_logger(), "Encoder + IMU running on %s", port_.c_str());
  }

private:

  bool openSerial()
  {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);

    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag |= CS8;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;

    return tcsetattr(fd_, TCSANOW, &tio) == 0;
  }

  bool readLine(std::string &line)
  {
    char buf[256];
    ssize_t n = read(fd_, buf, sizeof(buf));
    if (n <= 0) return false;

    buffer_.append(buf, n);
    auto pos = buffer_.find('\n');
    if (pos == std::string::npos) return false;

    line = buffer_.substr(0, pos);
    buffer_.erase(0, pos + 1);
    return true;
  }

  void timerCallback()
  {
    std::string line;
    while (readLine(line)) {
      processLine(line);
    }
  }

  void processLine(const std::string &line)
  {
    if (line.rfind("ENC", 0) != 0) return;

    std::istringstream iss(line);

    std::string tag;
    long left_counts, right_counts;

    if (!(iss >> tag >> left_counts >> right_counts)) return;

    double gx=0, gy=0, gz=0, ax=0, ay=0, az=0;
    bool has_imu = false;

    size_t imu_pos = line.find("IMU");
    if (imu_pos != std::string::npos)
    {
      std::istringstream imu_stream(line.substr(imu_pos));
      std::string imu_tag;

      if (imu_stream >> imu_tag >> gx >> gy >> gz >> ax >> ay >> az)
      {
        has_imu = true;
      }
    }

    // 🔥 FIX الأساسي
    rclcpp::Time now_time = this->get_clock()->now();

    if (first_read_) {
      last_left_counts_  = left_counts;
      last_right_counts_ = right_counts;
      last_time_ = now_time;
      first_read_ = false;
      return;
    }

    long dL_ticks = left_counts  - last_left_counts_;
    long dR_ticks = right_counts - last_right_counts_;

    last_left_counts_  = left_counts;
    last_right_counts_ = right_counts;

    double dt = (now_time - last_time_).seconds();
    if (dt <= 0.0) return;
    last_time_ = now_time;

    double ticks_L = 4.0 * ticks_per_rev_left_;
    double ticks_R = 4.0 * ticks_per_rev_right_;

    double dL = (dL_ticks / ticks_L) * (2.0 * M_PI * R_);
    double dR = (dR_ticks / ticks_R) * (2.0 * M_PI * R_);

    double d_center = (dR + dL) / 2.0;
    double d_theta  = (dL - dR) / L_;

    double theta_mid = theta_ + d_theta / 2.0;

    x_     += d_center * std::cos(theta_mid);
    y_     += d_center * std::sin(theta_mid);
    theta_ += d_theta;

    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    double v     = d_center / dt;
    double omega = d_theta  / dt;

    publishOdom(now_time, v, omega);

    if (has_imu)
    {
      sensor_msgs::msg::Imu imu;

      imu.header.stamp = now_time;
      imu.header.frame_id = "imu_link";

      imu.angular_velocity.x = gx;
      imu.angular_velocity.y = gy;
      imu.angular_velocity.z = gz;

      imu.linear_acceleration.x = ax;
      imu.linear_acceleration.y = ay;
      imu.linear_acceleration.z = az;

      imu.orientation.w = 1.0;

      imu.angular_velocity_covariance[0] = 0.01;
      imu.angular_velocity_covariance[4] = 0.01;
      imu.angular_velocity_covariance[8] = 0.01;

      imu.linear_acceleration_covariance[0] = 0.1;
      imu.linear_acceleration_covariance[4] = 0.1;
      imu.linear_acceleration_covariance[8] = 0.1;

      imu.orientation_covariance[0] = -1;

      imu_pub_->publish(imu);
    }
  }

  void publishOdom(const rclcpp::Time &stamp, double v, double omega)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.pose.covariance[0]  = 0.05;
    odom.pose.covariance[7]  = 0.05;
    odom.pose.covariance[35] = 0.1;

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = omega;

    odom.twist.covariance[0]  = 0.02;
    odom.twist.covariance[7]  = 0.02;
    odom.twist.covariance[35] = 0.05;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = stamp;
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = child_frame_id_;

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
  }

  int fd_;
  std::string port_;
  int baud_;

  double R_, L_;
  int ticks_per_rev_left_, ticks_per_rev_right_;

  std::string frame_id_, child_frame_id_;
  std::string buffer_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, theta_;
  long last_left_counts_, last_right_counts_;
  bool first_read_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderOdomNode>());
  rclcpp::shutdown();
  return 0;
}