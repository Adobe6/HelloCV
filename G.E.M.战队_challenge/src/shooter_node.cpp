#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "referee_pkg/msg/multi_object.hpp"
#include "referee_pkg/msg/object.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

using std::placeholders::_1;
using referee_pkg::msg::MultiObject;
using referee_pkg::msg::Object;

class ShooterNode : public rclcpp::Node
{
public:
  ShooterNode() : Node("shooter_node")
  {
    // ---------------- 参数 ----------------
    declare_parameter<std::string>("vision_topic", "/vision/target");
    declare_parameter<std::string>("gimbal_cmd_topic", "/gimbal/cmd");

    // 相机内参（要根据裁判系统说明填）
    declare_parameter<double>("camera_fx", 800.0);
    declare_parameter<double>("camera_fy", 800.0);
    declare_parameter<double>("camera_cx", 320.0);
    declare_parameter<double>("camera_cy", 320.0);

    // 装甲板真实尺寸（米）——需要根据规则填
    declare_parameter<double>("armor_width", 0.135);   // 例如 135mm
    declare_parameter<double>("armor_height", 0.055);  // 例如 55mm

    // 简单的“到点就射击”阈值
    declare_parameter<double>("max_yaw_error_deg", 1.0);
    declare_parameter<double>("max_pitch_error_deg", 1.0);

    get_parameter("vision_topic", vision_topic_);
    get_parameter("gimbal_cmd_topic", gimbal_topic_);

    get_parameter("camera_fx", fx_);
    get_parameter("camera_fy", fy_);
    get_parameter("camera_cx", cx_);
    get_parameter("camera_cy", cy_);

    get_parameter("armor_width", armor_w_);
    get_parameter("armor_height", armor_h_);

    get_parameter("max_yaw_error_deg", max_yaw_err_deg_);
    get_parameter("max_pitch_error_deg", max_pitch_err_deg_);

    // 构造相机矩阵
    camera_matrix_ = (cv::Mat1d(3,3) <<
      fx_, 0.0, cx_,
      0.0, fy_, cy_,
      0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);  // 默认无畸变，如有需要改这里

    // ---------------- 订阅 / 发布 ----------------
    sub_vision_ = create_subscription<MultiObject>(
      vision_topic_, 10, std::bind(&ShooterNode::onVision, this, _1));

    pub_gimbal_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
      gimbal_topic_, 10);

    RCLCPP_INFO(get_logger(),
                "shooter_node started. Sub: %s, Pub: %s",
                vision_topic_.c_str(), gimbal_topic_.c_str());
  }

private:
  void onVision(const MultiObject::SharedPtr msg)
  {
    if (msg->num_objects == 0 || msg->objects.empty()) {
      // 没目标，清空状态即可
      return;
    }

    // 这里简单用 objects[0]，也可以按某种策略挑选
    const Object &obj = msg->objects[0];

    if (obj.corners.size() < 4) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Object has less than 4 corners.");
      return;
    }

    // --------- 构造像素点（u, v） -----------
    // 注意：ROS 中 Point 的 x,y 对应像素坐标 (u,v)
    std::vector<cv::Point2f> image_points(4);
    image_points[0] = cv::Point2f(obj.corners[0].x, obj.corners[0].y);
    image_points[1] = cv::Point2f(obj.corners[1].x, obj.corners[1].y);
    image_points[2] = cv::Point2f(obj.corners[2].x, obj.corners[2].y);
    image_points[3] = cv::Point2f(obj.corners[3].x, obj.corners[3].y);

    // --------- 构造装甲板 3D 点 -------------
    // 以装甲板中心为原点，z=0 平面
    double hw = armor_w_ * 0.5;
    double hh = armor_h_ * 0.5;
    std::vector<cv::Point3f> object_points(4);
    // 这里和 image_points 顺序必须一致
    // 假设 corners: 左下, 左上, 右上, 右下
    object_points[0] = cv::Point3f(-hw, -hh, 0.0f);  // 左下
    object_points[1] = cv::Point3f(-hw,  hh, 0.0f);  // 左上
    object_points[2] = cv::Point3f( hw,  hh, 0.0f);  // 右上
    object_points[3] = cv::Point3f( hw, -hh, 0.0f);  // 右下

    // --------- solvePnP 求位姿 --------------
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(object_points, image_points,
                           camera_matrix_, dist_coeffs_,
                           rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "solvePnP failed.");
      return;
    }

    // 相机坐标系下的装甲板中心位置
    double x = tvec.at<double>(0);
    double y = tvec.at<double>(1);
    double z = tvec.at<double>(2);

    // --------- 计算 yaw / pitch --------------
    // 这里假设相机坐标：x 向右，y 向下，z 向前
    double yaw = std::atan2(x, z); // 左右偏角
    double pitch = std::atan2(-y, std::sqrt(x*x + z*z)); // 上下偏角

    double yaw_deg   = yaw   * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;

    // 打印一下方便调试
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "target: yaw=%.2f deg, pitch=%.2f deg, dist=%.2f m",
                         yaw_deg, pitch_deg, std::sqrt(x*x + y*y + z*z));

    // --------- 构造云台控制命令 ---------------
    geometry_msgs::msg::Twist cmd;
    // 这里约定：
    // angular.z 表示 yaw 角度（弧度）
    // angular.y 表示 pitch 角度（弧度）
    cmd.angular.z = yaw;
    cmd.angular.y = pitch;

    // 线速度部分暂时不用
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;

    pub_gimbal_cmd_->publish(cmd);

    // --------- 简单“到点就射击”的示例 ----------
    bool ready_to_fire =
      std::fabs(yaw_deg)   < max_yaw_err_deg_ &&
      std::fabs(pitch_deg) < max_pitch_err_deg_;

    if (ready_to_fire) {
      // TODO: 在这里调用裁判的击发服务 /referee/hit_armor
      // 或者发布一个开火信号到你们自己的发射控制话题。
      // 这里先只打印日志：
      RCLCPP_INFO(get_logger(), "Ready to fire!");
    }
  }

  // -------- 成员变量 --------
  std::string vision_topic_;
  std::string gimbal_topic_;

  double fx_, fy_, cx_, cy_;
  double armor_w_, armor_h_;
  double max_yaw_err_deg_, max_pitch_err_deg_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  rclcpp::Subscription<MultiObject>::SharedPtr sub_vision_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gimbal_cmd_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShooterNode>());
  rclcpp::shutdown();
  return 0;
}
