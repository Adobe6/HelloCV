#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <referee_pkg/msg/multi_object.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using referee_pkg::msg::MultiObject;
using referee_pkg::msg::Object;   // 如果你们包里名称不同，这里改一下

struct ArmorCandidate {
  cv::Rect rect;
  double score_area;
  double score_center;
  double score_color;
  double total_score;
};

class VisionNode : public rclcpp::Node {
public:
  VisionNode() : Node("vision_node") {
    // === 参数 ===
    camera_topic_   = declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    enemy_color_    = declare_parameter<std::string>("enemy_color", "red");  // "red" or "blue"
    min_area_       = declare_parameter<double>("min_area", 200.0);
    aspect_min_     = declare_parameter<double>("aspect_min", 0.5);
    aspect_max_     = declare_parameter<double>("aspect_max", 4.0);
    persistence_    = declare_parameter<int>("persistence_frames", 3);
    max_lost_       = declare_parameter<int>("max_lost_frames", 10);
    w_area_         = declare_parameter<double>("w_area", 1.0);
    w_center_       = declare_parameter<double>("w_center", 0.5);
    w_color_        = declare_parameter<double>("w_color", 1.0);

    // HSV 阈值可以后面放到 params.yaml 里去调
    if (enemy_color_ == "red") {
      lower1_ = cv::Scalar(0, 90, 90);
      upper1_ = cv::Scalar(10, 255, 255);
      lower2_ = cv::Scalar(160, 90, 90);
      upper2_ = cv::Scalar(180, 255, 255);
    } else { // blue
      lower1_ = cv::Scalar(90, 80, 80);
      upper1_ = cv::Scalar(130, 255, 255);
      use_two_range_ = false;
    }

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10, std::bind(&VisionNode::onImage, this, _1));
    pub_target_ = create_publisher<MultiObject>("/vision/target", 10);

    RCLCPP_INFO(get_logger(), "VisionNode started. Subscribing to %s, publishing to /vision/target",
                camera_topic_.c_str());
  }

private:
  // 图像回调：Level1+2 主流程
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (img.empty()) return;

    // 1. 颜色阈值 -> 二值图
    cv::Mat hsv; cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, lower1_, upper1_, mask1);
    if (use_two_range_) {
      cv::inRange(hsv, lower2_, upper2_, mask2);
      mask = mask1 | mask2;
    } else {
      mask = mask1;
    }

    // 一点形态学去噪
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // 2. 找轮廓，过滤成候选装甲板
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<ArmorCandidate> cands;
    cv::Point2f img_center(img.cols * 0.5f, img.rows * 0.5f);

    for (auto &c : contours) {
      double area = cv::contourArea(c);
      if (area < min_area_) continue;

      cv::RotatedRect rrect = cv::minAreaRect(c);
      float w = std::max(rrect.size.width, rrect.size.height);
      float h = std::min(rrect.size.width, rrect.size.height);
      if (h <= 1) continue;
      double aspect = static_cast<double>(w) / h;
      if (aspect < aspect_min_ || aspect > aspect_max_) continue;

      cv::Rect rect = rrect.boundingRect();
      rect &= cv::Rect(0, 0, img.cols, img.rows);
      if (rect.empty()) continue;

      ArmorCandidate cand;
      cand.rect = rect;
      cand.score_area = area;
      // 中心越靠近图像中心，分越高
      cv::Point2f cpt(rect.x + rect.width/2.0f, rect.y + rect.height/2.0f);
      double dist = cv::norm(cpt - img_center);
      cand.score_center = -dist; // 距离越小越好，用负号
      cand.score_color = 1.0;    // 目前颜色筛掉了就保留 1，后面可以更精细
      cand.total_score = w_area_ * cand.score_area +
                         w_center_ * cand.score_center +
                         w_color_ * cand.score_color;
      cands.push_back(cand);
    }

    // 3. 多目标排序（Level2）：按 total_score 从高到低排序
    std::sort(cands.begin(), cands.end(),
              [](const ArmorCandidate &a, const ArmorCandidate &b) {
                return a.total_score > b.total_score;
              });

    MultiObject out;
    out.header = msg->header;  // 时间戳沿用相机时间戳（裁判要求）

    // 4. 跟踪 + 抗干扰
    if (cands.empty()) {
      // 没有候选，丢帧计数 +1
      no_target_frames_++;
      if (no_target_frames_ > max_lost_) {
        stable_frames_ = 0;
        has_last_ = false;
      }
      pub_target_->publish(out);
      return;
    }

    // 选择一个“当前目标”：优先追踪上一帧的那个框
    int best_index = 0;
    if (has_last_) {
      double best_dist = 1e9;
      for (size_t i = 0; i < cands.size(); ++i) {
        cv::Point2f cpt(cands[i].rect.x + cands[i].rect.width/2.0f,
                        cands[i].rect.y + cands[i].rect.height/2.0f);
        double d = cv::norm(cpt - last_center_);
        if (d < best_dist) {
          best_dist = d;
          best_index = static_cast<int>(i);
        }
      }
    } else {
      best_index = 0; // 第一次就取总分最高的
    }

    ArmorCandidate &best = cands[best_index];

    // 更新跟踪状态
    cv::Point2f cur_center(best.rect.x + best.rect.width/2.0f,
                           best.rect.y + best.rect.height/2.0f);
    if (has_last_) {
      double move = cv::norm(cur_center - last_center_);
      // 移动不离谱就认为是连续
      if (move < 100.0) {
        stable_frames_++;
      } else {
        stable_frames_ = 1;
      }
    } else {
      stable_frames_ = 1;
      has_last_ = true;
    }
    last_center_ = cur_center;
    no_target_frames_ = 0;

    // 抗干扰：连续出现帧数 < persistence_ 就不输出任何目标
    if (stable_frames_ < persistence_) {
      pub_target_->publish(out); // 空列表
      return;
    }

    // 5. 填 MultiObject 输出（Level1 + 排序信息）
    out.objects.reserve(cands.size());
    for (size_t i = 0; i < cands.size(); ++i) {
      const auto &cand = cands[i];
      Object o;
      // 这里字段名需要你对照 referee_pkg/msg/Object 实际定义改
      o.id = static_cast<int32_t>(i);   // 如果没有 id 字段就删掉这一行
      o.confidence = 1.0;               // 如果有置信度字段
      o.type = enemy_color_;            // 如果有 type/string 字段
      o.center.x = cand.rect.x + cand.rect.width / 2.0;
      o.center.y = cand.rect.y + cand.rect.height / 2.0;
      o.center.z = 0.0;

      // 如果消息里有像素级四个角点，可以这样填：
      // o.corners[0].x = cand.rect.x;
      // o.corners[0].y = cand.rect.y;
      // o.corners[1].x = cand.rect.x + cand.rect.width;
      // o.corners[1].y = cand.rect.y;
      // o.corners[2].x = cand.rect.x + cand.rect.width;
      // o.corners[2].y = cand.rect.y + cand.rect.height;
      // o.corners[3].x = cand.rect.x;
      // o.corners[3].y = cand.rect.y + cand.rect.height;

      // 按排序后的顺序 push_back
      out.objects.push_back(o);
    }

    out.num_objects = static_cast<int32_t>(out.objects.size()); // 如果有这个字段

    pub_target_->publish(out);
  }

  // === 成员变量 ===
  std::string camera_topic_;
  std::string enemy_color_;
  double min_area_, aspect_min_, aspect_max_;
  int persistence_, max_lost_;
  double w_area_, w_center_, w_color_;
  cv::Scalar lower1_, upper1_, lower2_, upper2_;
  bool use_two_range_ = true;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<MultiObject>::SharedPtr pub_target_;

  // 跟踪状态
  bool has_last_ = false;
  cv::Point2f last_center_;
  int stable_frames_ = 0;
  int no_target_frames_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "referee_pkg/msg/object.hpp"
#include "referee_pkg/msg/multi_object.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using referee_pkg::msg::Object;
using referee_pkg::msg::MultiObject;

struct ArmorCandidate
{
  cv::RotatedRect rrect;
  cv::Rect bbox;
  double score_area;
  double score_center;
  double score_aspect;
  double total_score;
};

class VisionNode : public rclcpp::Node
{
public:
  VisionNode() : Node("vision_node")
  {
    // ---------------- 参数 ----------------
    camera_topic_ = declare_parameter<std::string>("camera_topic", "/camera/image_raw");
    enemy_color_  = declare_parameter<std::string>("enemy_color", "red"); // red / blue

    min_area_     = declare_parameter<double>("min_area", 150.0);
    aspect_min_   = declare_parameter<double>("aspect_min", 1.0);
    aspect_max_   = declare_parameter<double>("aspect_max", 5.0);

    w_area_       = declare_parameter<double>("w_area", 1.0);
    w_center_     = declare_parameter<double>("w_center", 0.5);
    w_aspect_     = declare_parameter<double>("w_aspect", 0.3);

    persistence_frames_ = declare_parameter<int>("persistence_frames", 3);
    max_lost_frames_    = declare_parameter<int>("max_lost_frames", 10);
    max_track_dist_     = declare_parameter<double>("max_track_dist", 120.0);

    // 根据敌方颜色设置大致 HSV 阈值（后面可放到 params.yaml 细调）
    if (enemy_color_ == "red") {
      // 红色有两段
      use_two_ranges_ = true;
      lower1_ = cv::Scalar(0,   80, 80);
      upper1_ = cv::Scalar(10, 255,255);
      lower2_ = cv::Scalar(160,80, 80);
      upper2_ = cv::Scalar(180,255,255);
    } else {
      // 蓝色一段
      use_two_ranges_ = false;
      lower1_ = cv::Scalar(90,  80, 80);
      upper1_ = cv::Scalar(130,255,255);
    }

    // ---------------- 订阅 / 发布 ----------------
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10, std::bind(&VisionNode::onImage, this, _1));

    pub_target_ = create_publisher<MultiObject>("/vision/target", 10);

    RCLCPP_INFO(get_logger(),
                "vision_node started. Sub: %s, Pub: /vision/target, enemy_color=%s",
                camera_topic_.c_str(), enemy_color_.c_str());
  }

private:
  // 获取旋转矩形的 4 个角点，按：左下→左上→右上→右下（逆时针）排序
  std::array<cv::Point2f,4> getOrderedCorners(const cv::RotatedRect &rr)
  {
    cv::Point2f pts[4];
    rr.points(pts);

    // 先按 y 从小到大（上→下）排序
    std::sort(pts, pts+4, [](const cv::Point2f &a, const cv::Point2f &b){
      return a.y < b.y;
    });

    cv::Point2f top1 = pts[0];
    cv::Point2f top2 = pts[1];
    cv::Point2f bottom1 = pts[2];
    cv::Point2f bottom2 = pts[3];

    cv::Point2f top_left   = (top1.x < top2.x) ? top1 : top2;
    cv::Point2f top_right  = (top1.x < top2.x) ? top2 : top1;
    cv::Point2f bottom_left  = (bottom1.x < bottom2.x) ? bottom1 : bottom2;
    cv::Point2f bottom_right = (bottom1.x < bottom2.x) ? bottom2 : bottom1;

    // 图像坐标 y 向下，所以“左下”是 bottom_left
    std::array<cv::Point2f,4> out {
      bottom_left,   // 左下
      top_left,      // 左上
      top_right,     // 右上
      bottom_right   // 右下
    };
    return out;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }
    if (img.empty()) return;

    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // 1. 颜色阈值
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, lower1_, upper1_, mask1);
    if (use_two_ranges_) {
      cv::inRange(hsv, lower2_, upper2_, mask2);
      mask = mask1 | mask2;
    } else {
      mask = mask1;
    }

    // 简单去噪
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    // 2. 找轮廓，筛出候选装甲板
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<ArmorCandidate> cands;
    cands.reserve(contours.size());

    cv::Point2f img_center(img.cols*0.5f, img.rows*0.5f);

    for (auto &c : contours) {
      double area = cv::contourArea(c);
      if (area < min_area_) continue;

      cv::RotatedRect rr = cv::minAreaRect(c);
      float w = std::max(rr.size.width, rr.size.height);
      float h = std::min(rr.size.width, rr.size.height);
      if (h <= 1) continue;
      double aspect = w / h;
      if (aspect < aspect_min_ || aspect > aspect_max_) continue;

      cv::Rect bbox = rr.boundingRect();
      bbox &= cv::Rect(0,0,img.cols,img.rows);
      if (bbox.empty()) continue;

      ArmorCandidate cand;
      cand.rrect = rr;
      cand.bbox = bbox;
      cand.score_area = area;

      cv::Point2f cpt = rr.center;
      double dist = cv::norm(cpt - img_center);      // 越居中越好
      cand.score_center = -dist;

      double aspect_mid = 0.5*(aspect_min_+aspect_max_);
      cand.score_aspect = -std::abs(aspect - aspect_mid);

      cand.total_score = w_area_*cand.score_area
                       + w_center_*cand.score_center
                       + w_aspect_*cand.score_aspect;

      cands.push_back(cand);
    }

    // 多目标排序（Level2）
    std::sort(cands.begin(), cands.end(),
              [](const ArmorCandidate &a, const ArmorCandidate &b){
                return a.total_score > b.total_score;
              });

    MultiObject out_msg;
    out_msg.header = msg->header; // 时间戳跟相机一致

    if (cands.empty()) {
      // 没检测到目标，做丢失计数
      lost_frames_++;
      if (lost_frames_ > max_lost_frames_) {
        has_last_target_ = false;
        stable_frames_ = 0;
      }
      out_msg.num_objects = 0;
      pub_target_->publish(out_msg);
      return;
    }

    // 3. 跟踪：优先沿用上一帧目标
    int main_index = 0;
    if (has_last_target_) {
      double best_d = 1e9;
      for (size_t i=0;i<cands.size();++i) {
        double d = cv::norm(cands[i].rrect.center - last_center_);
        if (d < best_d) {
          best_d = d;
          main_index = static_cast<int>(i);
        }
      }

      // 如果移动太离谱，视为新目标
      if (best_d > max_track_dist_) {
        has_last_target_ = false;
        stable_frames_ = 0;
        main_index = 0; // 退回总分最高
      }
    }

    ArmorCandidate &main_cand = cands[main_index];
    cv::Point2f cur_center = main_cand.rrect.center;

    if (!has_last_target_) {
      has_last_target_ = true;
      stable_frames_ = 1;
    } else {
      double move = cv::norm(cur_center - last_center_);
      if (move < max_track_dist_) {
        stable_frames_++;
      } else {
        stable_frames_ = 1;
      }
    }
    last_center_ = cur_center;
    lost_frames_ = 0;

    // 抗干扰：连续帧数不足就认为不稳定，清空输出
    if (stable_frames_ < persistence_frames_) {
      out_msg.num_objects = 0;
      pub_target_->publish(out_msg);
      return;
    }

    // 4. 构造 MultiObject / Object 列表
    out_msg.objects.reserve(cands.size());

    for (auto &cand : cands) {
      Object obj;
      obj.target_type = "armor_1";   // 简单写死一种类型，后面可以根据装甲板ID再细分

      auto corners = getOrderedCorners(cand.rrect);
      obj.corners.resize(4);
      for (int i=0;i<4;++i) {
        geometry_msgs::msg::Point p;
        p.x = corners[i].x;
        p.y = corners[i].y;
        p.z = 0.0;
        obj.corners[i] = p;
      }

      out_msg.objects.push_back(obj);
    }

    out_msg.num_objects = static_cast<uint32_t>(out_msg.objects.size());

    pub_target_->publish(out_msg);
  }

  // -------- 成员变量 --------
  std::string camera_topic_;
  std::string enemy_color_;

  double min_area_;
  double aspect_min_, aspect_max_;
  double w_area_, w_center_, w_aspect_;

  int    persistence_frames_;
  int    max_lost_frames_;
  double max_track_dist_;

  bool   use_two_ranges_{false};
  cv::Scalar lower1_, upper1_, lower2_, upper2_;

  // 跟踪状态
  bool has_last_target_{false};
  cv::Point2f last_center_;
  int stable_frames_{0};
  int lost_frames_{0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<MultiObject>::SharedPtr pub_target_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
