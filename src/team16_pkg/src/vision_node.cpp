#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <referee_pkg/msg/race_stage.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node"), current_stage_(0) {
        // 1. 声明参数（适配裁判系统相机参数）
        declare_parameters();
        // 2. 获取参数
        get_parameters();
        // 3. 订阅相机图像（裁判系统固定话题）
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&VisionNode::image_callback, this, std::placeholders::_1)
        );
        // 4. 订阅比赛阶段（裁判系统阶段话题）
        stage_sub_ = create_subscription<referee_pkg::msg::RaceStage>(
            "/referee/race_stage", 10,
            std::bind(&VisionNode::stage_callback, this, std::placeholders::_1)
        );
        // 5. 发布识别结果（裁判系统要求话题）
        target_pub_ = create_publisher<referee_pkg::msg::MultiObject>(
            "/vision/target", 10
        );

        RCLCPP_INFO(get_logger(), "STAGE_1 Vision Node initialized");
        RCLCPP_INFO(get_logger(), "Subscribed to: /camera/image_raw");
        RCLCPP_INFO(get_logger(), "Publishing to: /vision/target");
    }

private:
    // 参数
    int red_h1_low_, red_h1_high_, red_h2_low_, red_h2_high_;
    int s_low_, s_high_, v_low_, v_high_;
    double ring_min_r_, ring_max_r_;  // 圆环半径阈值
    double circle_center_dist_thresh_; // 内外圆中心距离阈值
    int current_stage_;

    // ROS组件
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr stage_sub_;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr target_pub_;

    // 声明参数（STAGE_1 专用配置）
    void declare_parameters() {
        // 红色HSV阈值（适配比赛环境）
        this->declare_parameter("red_h1_low", 0);
        this->declare_parameter("red_h1_high", 10);
        this->declare_parameter("red_h2_low", 160);
        this->declare_parameter("red_h2_high", 180);
        this->declare_parameter("s_low", 90);
        this->declare_parameter("s_high", 255);
        this->declare_parameter("v_low", 90);
        this->declare_parameter("v_high", 255);
        // 圆环参数（适配裁判系统圆环尺寸）
        this->declare_parameter("ring_min_r", 30.0);
        this->declare_parameter("ring_max_r", 150.0);
        this->declare_parameter("circle_center_dist_thresh", 20.0); // 内外圆中心允许偏差
    }

    // 获取参数
    void get_parameters() {
        // 红色阈值
        this->get_parameter("red_h1_low", red_h1_low_);
        this->get_parameter("red_h1_high", red_h1_high_);
        this->get_parameter("red_h2_low", red_h2_low_);
        this->get_parameter("red_h2_high", red_h2_high_);
        this->get_parameter("s_low", s_low_);
        this->get_parameter("s_high", s_high_);
        this->get_parameter("v_low", v_low_);
        this->get_parameter("v_high", v_high_);
        // 圆环参数
        this->get_parameter("ring_min_r", ring_min_r_);
        this->get_parameter("ring_max_r", ring_max_r_);
        this->get_parameter("circle_center_dist_thresh", circle_center_dist_thresh_);
    }

    // 比赛阶段回调（仅响应STAGE_1）
    void stage_callback(const referee_pkg::msg::RaceStage::SharedPtr stage_msg) {
        current_stage_ = stage_msg->stage;
        if (current_stage_ == 1) {
            RCLCPP_INFO(get_logger(), "Switched to STAGE_1 (Ring_red detection)");
        } else {
            RCLCPP_WARN(get_logger(), "Current stage is not STAGE_1, skip detection");
        }
    }

    // 图像回调（核心识别逻辑）
    void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
        // 仅在STAGE_1时执行识别
        if (current_stage_ != 1) return;

        try {
            // 1. 图像格式转换（裁判系统固定bgr8格式）
            cv::Mat img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
            cv::Mat hsv, mask;

            // 2. 红色阈值分割（双范围覆盖红色光谱）
            cv::Mat mask1, mask2;
            cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(red_h1_low_, s_low_, v_low_), 
                          cv::Scalar(red_h1_high_, s_high_, v_high_), mask1);
            cv::inRange(hsv, cv::Scalar(red_h2_low_, s_low_, v_low_), 
                          cv::Scalar(red_h2_high_, s_high_, v_high_), mask2);
            mask = mask1 | mask2;

            // 3. 形态学去噪（保留圆环轮廓）
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::erode(mask, mask, kernel);
            cv::dilate(mask, mask, kernel);

            // 4. 霍夫圆检测（识别内外圆）
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT,
                1.5, mask.rows/8, 100, 30, ring_min_r_, ring_max_r_);

            // 5. 筛选有效圆环对（外圆+内圆）
            referee_pkg::msg::MultiObject multi_obj_msg;
            multi_obj_msg.header = img_msg->header; // 沿用图像时间戳（裁判要求）

            if (circles.size() >= 2) {
                // 按半径排序（大的为外圆，小的为内圆）
                std::sort(circles.begin(), circles.end(),
                    [](const cv::Vec3f& a, const cv::Vec3f& b) {
                        return a[2] > b[2];
                    });

                cv::Vec3f outer_circle = circles[0]; // 外圆
                cv::Vec3f inner_circle = circles[1]; // 内圆

                // 验证内外圆中心距离（避免误匹配）
                double center_dist = cv::norm(
                    cv::Point2f(outer_circle[0], outer_circle[1]) - 
                    cv::Point2f(inner_circle[0], inner_circle[1])
                );
                if (center_dist > circle_center_dist_thresh_) {
                    RCLCPP_WARN(get_logger(), "Inner/outer circle center distance too large: %.2f", center_dist);
                    return;
                }

                // 6. 构造裁判要求的Object消息（外圆在前，内圆在后）
                referee_pkg::msg::Object outer_obj = create_circle_object(outer_circle, "Ring_red");
                referee_pkg::msg::Object inner_obj = create_circle_object(inner_circle, "Ring_red");

                // 7. 填充MultiObject消息（裁判要求num_objects=2）
                multi_obj_msg.objects.push_back(outer_obj);
                multi_obj_msg.objects.push_back(inner_obj);
                multi_obj_msg.num_objects = 2;

                // 发布结果
                target_pub_->publish(multi_obj_msg);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "STAGE_1: Published Ring_red (outer r=%.1f, inner r=%.1f)",
                    outer_circle[2], inner_circle[2]);
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "STAGE_1: No valid circle pair found (detected %ld circles)", circles.size());
                // 发布空消息（保持话题频率）
                multi_obj_msg.num_objects = 0;
                target_pub_->publish(multi_obj_msg);
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV bridge error: %s", e.what());
        }
    }

    // 生成圆环Object消息（裁判要求：左点起始，逆时针4点）
    referee_pkg::msg::Object create_circle_object(const cv::Vec3f& circle, const std::string& target_type) {
        referee_pkg::msg::Object obj;
        obj.target_type = target_type; // 固定类型为"Ring_red"
        obj.corners.resize(4); // 必须4个角点

        float x = circle[0]; // 圆心x
        float y = circle[1]; // 圆心y
        float r = circle[2]; // 半径

        // 按裁判要求排序：左点→下点→右点→上点（逆时针）
        geometry_msgs::msg::Point p1, p2, p3, p4;
        // 1. 左点（起始点）
        p1.x = x - r;
        p1.y = y;
        p1.z = 0.0;
        // 2. 下点
        p2.x = x;
        p2.y = y + r;
        p2.z = 0.0;
        // 3. 右点
        p3.x = x + r;
        p3.y = y;
        p3.z = 0.0;
        // 4. 上点
        p4.x = x;
        p4.y = y - r;
        p4.z = 0.0;

        obj.corners[0] = p1;
        obj.corners[1] = p2;
        obj.corners[2] = p3;
        obj.corners[3] = p4;

        return obj;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

