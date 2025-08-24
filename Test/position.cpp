#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class PoseSourceNode : public rclcpp::Node
{
public:
    PoseSourceNode() : Node("pose_source")
    {
        // 파라미터: 출력 frame_id, 입력 토픽 이름
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        in_pose_topic_ = declare_parameter<std::string>("in_pose_topic", "/mavros/local_position/pose");
        in_twist_topic_ = declare_parameter<std::string>("in_twist_topic", "/mavros/local_position/velocity_local");

        // QoS 설정(신뢰성 위주)
        auto qos_in = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto qos_out = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // 구독: MAVROS 원본
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            in_pose_topic_, qos_in,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                last_pose_time_ = now();
                // frame_id 표준화
                msg->header.frame_id = frame_id_;
                pose_pub_->publish(*msg);
                last_pose_ = *msg;
                pose_count_++;
            });

        twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            in_twist_topic_, qos_in,
            [this](geometry_msgs::msg::TwistStamped::SharedPtr msg)
            {
                last_twist_time_ = now();
                msg->header.frame_id = frame_id_;
                twist_pub_->publish(*msg);
                last_twist_ = *msg;
                twist_count_++;
            });

        // 발행: 표준화된 상태
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot/state/pose", qos_out);
        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/robot/state/twist", qos_out);

        // 간단 진단(1초 주기)
        diag_timer_ = create_wall_timer(1s, std::bind(&PoseSourceNode::onDiag, this));
    }

private:
    void onDiag()
    {
        auto nowt = now();
        auto pose_age_ms = (nowt - last_pose_time_).nanoseconds() / 1e6;
        auto twist_age_ms = (nowt - last_twist_time_).nanoseconds() / 1e6;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "[pose_source] pose Hz≈%d/s, age≈%.0fms | twist Hz≈%d/s, age≈%.0fms | frame='%s'",
                             static_cast<int>(pose_count_), pose_age_ms,
                             static_cast<int>(twist_count_), twist_age_ms,
                             frame_id_.c_str());

        pose_count_ = 0;
        twist_count_ = 0;
    }

    // 파라미터
    std::string frame_id_;
    std::string in_pose_topic_;
    std::string in_twist_topic_;

    // Pub/Sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    // 상태/진단
    geometry_msgs::msg::PoseStamped last_pose_;
    geometry_msgs::msg::TwistStamped last_twist_;
    rclcpp::Time last_pose_time_{0, 0, get_clock()->get_clock_type()};
    rclcpp::Time last_twist_time_{0, 0, get_clock()->get_clock_type()};
    int pose_count_{0};
    int twist_count_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSourceNode>());
    rclcpp::shutdown();
    return 0;
}