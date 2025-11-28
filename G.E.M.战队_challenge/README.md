使用说明:
本项目基于 ROS 2 构建，包含两个核心节点：
    vision_node：负责从相机采集图像，进行装甲板识别与跟踪，发布目标信息
    shooter_node：接收目标信息，计算云台控制角度，实现自动瞄准与射击判断
系统可用于机器人射击类比赛中对敌方装甲板的自动识别与瞄准控制。


依赖项：<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>referee_pkg</depend>
<depend>cv_bridge</depend>
<depend>opencv2</depend>

算法原理和命令操作：该视觉识别系统采用传统机器视觉算法框架，核心基于颜色特征分割 + 几何形状筛选 + 目标跟踪的流程实现装甲板识别，具体原理拆解如下：
一、颜色空间转换与阈值分割（目标初筛）

    RGB 转 HSV：

相机采集的原始图像为 RGB 格式，但 RGB 对光照变化敏感（如强光下红色易泛白）。因此先将图像转换为 HSV（色相 Hue、饱和度 Saturation、明度 Value）空间 ——HSV 能分离 “颜色” 与 “亮度”，更适合颜色筛选。
颜色阈值过滤：

    根据敌方装甲板颜色（红 / 蓝），设定对应的 HSV 阈值范围（如红色取 H=0-10 和 160-180 区间，蓝色取 H=90-130 区间），生成二值化掩码：符合阈值的像素保留为白色（疑似目标），其余为黑色（背景），实现目标与背景的初步分离。

二、形态学操作（去噪与轮廓优化）
对二值化掩码执行 ** 腐蚀（Erode）+ 膨胀（Dilate）** 操作：

    腐蚀：缩小白色区域，消除小的噪声点（如光斑、杂色像素），避免误判；
    膨胀：扩大白色区域，修复腐蚀导致的目标轮廓缺口，还原装甲板的完整形状。
    通过这两步过滤背景干扰，强化目标轮廓的连续性。

三、轮廓检测与几何特征筛选（目标确认）

    轮廓提取：

使用cv::findContours算法从处理后的图像中提取所有白色区域的轮廓，每个轮廓对应一个疑似目标。
几何特征过滤：

    对每个轮廓计算关键几何特征，筛选出符合装甲板特征的目标：
        面积筛选：过滤面积过小（如小于 200 像素）的噪声轮廓；
        宽高比筛选：装甲板的宽高比通常在 0.5~3.0 之间，排除过扁或过窄的轮廓；
        形状近似：通过cv::minAreaRect拟合轮廓的最小外接矩形，判断是否接近装甲板的矩形形状。

四、目标跟踪与多帧验证（稳定性提升）

    多帧匹配：

对连续帧中的目标轮廓进行位置匹配（如计算两帧间轮廓的中心距离），实现目标跟踪，避免单帧误检。
稳定性验证：

    设置persistence_frames（连续出现帧数）和max_lost_frames（最大丢失帧数），只有连续多帧出现且短暂遮挡后仍能匹配的目标，才判定为有效装甲板，提升识别的鲁棒性。

五、角点提取与坐标输出（为瞄准提供数据）
对最终确认的装甲板轮廓，提取其四个角点并按固定顺序排序（如左下→左上→右上→右下），结合图像坐标系输出角点坐标，为后续射击节点的 PnP 位姿解算提供关键数据。


cd Vision_Arena_2025/
source install/setup.bash 
ros2 launch referee_pkg referee_pkg_launch.xml     TeamName:="TEAM16"     StageSelect:=1     ModeSelect:=1  球

cd Vision_Arena_2025/
source install/setup.bash 
ros2 launch referee_pkg referee_pkg_launch.xml     TeamName:="TEAM16"     StageSelect:=2     ModeSelect:=1  矩形

cd Vision_Arena_2025/
source install/setup.bash 
ros2 launch referee_pkg referee_pkg_launch.xml     TeamName:="TEAM16"     StageSelect:=3     ModeSelect:=1  装甲板


