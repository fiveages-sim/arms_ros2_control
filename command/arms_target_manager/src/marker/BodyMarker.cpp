#include "arms_target_manager/marker/BodyMarker.h"

namespace arms_ros2_control::command
{

BodyMarker::BodyMarker(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<MarkerFactory> marker_factory,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& frame_id,
    const std::string& control_base_frame,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_publisher,
    const std::string& current_pose_topic,
    double publish_rate,
    UpdateCallback update_callback)
    : node_(std::move(node))
    , marker_factory_(std::move(marker_factory))
    , tf_buffer_(std::move(tf_buffer))
    , frame_id_(frame_id)
    , control_base_frame_(control_base_frame)
    , target_stamped_publisher_(std::move(target_stamped_publisher))
    , publish_rate_(publish_rate)
    , last_publish_time_(node_->now())
    , last_subscription_update_time_(node_->now())
    , update_callback_(std::move(update_callback))
{
    pose_.position.x = 0.0;
    pose_.position.y = 0.0;
    pose_.position.z = 0.9;
    pose_.orientation.w = 1.0;
    pose_.orientation.x = 0.0;
    pose_.orientation.y = 0.0;
    pose_.orientation.z = 0.0;

    auto callback = [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        this->updateFromTopic(msg);
    };

    current_pose_subscription_ =
        node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            current_pose_topic, 10, callback);
}

visualization_msgs::msg::InteractiveMarker BodyMarker::createMarker(
    const std::string& name,
    MarkerState mode,
    bool enable_interaction) const
{
    return marker_factory_->createArmMarker(
        name,
        "Body Target",
        pose_,
        "green",   // 建议用绿色区分
        mode,
        enable_interaction);
}

geometry_msgs::msg::Pose BodyMarker::handleFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
    const std::string& source_frame_id) const
{
    return transformPose(feedback->pose, source_frame_id, frame_id_);
}

void BodyMarker::updateFromTopic(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)
{
    if (!pose_msg) {
        return;
    }

    if (current_pose_callback_) {
        current_pose_callback_(pose_msg);
    }

    // 节流到 30Hz 左右
    const double interval = 1.0 / 30.0;
    auto now = node_->now();
    if ((now - last_subscription_update_time_).seconds() < interval) {
        return;
    }
    last_subscription_update_time_ = now;

    // 如果外部状态检查不允许自动更新，则直接返回
    if (state_check_callback_ && !state_check_callback_()) {
        return;
    }

    geometry_msgs::msg::Pose transformed =
        transformPose(pose_msg->pose, pose_msg->header.frame_id, frame_id_);

    pose_ = transformed;

    if (update_callback_) {
        update_callback_(getMarkerName(), pose_);
    }
}

bool BodyMarker::publishTargetPose(bool force)
{
    if (!target_stamped_publisher_) {
        return false;
    }

    const double interval = (publish_rate_ > 1e-6) ? (1.0 / publish_rate_) : 0.0;
    if (!force && interval > 0.0 && !shouldThrottle(interval)) {
        return false;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = control_base_frame_;
    msg.pose = transformPose(pose_, frame_id_, control_base_frame_);

    target_stamped_publisher_->publish(msg);
    return true;
}

geometry_msgs::msg::Pose BodyMarker::transformPose(
    const geometry_msgs::msg::Pose& pose,
    const std::string& source_frame_id,
    const std::string& target_frame_id) const
{
    if (source_frame_id == target_frame_id) {
        return pose;
    }

    try {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = source_frame_id;
        pose_stamped.header.stamp = rclcpp::Time(0);
        pose_stamped.pose = pose;

        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            target_frame_id, source_frame_id, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped result_stamped;
        tf2::doTransform(pose_stamped, result_stamped, transform);
        return result_stamped.pose;
    } catch (const tf2::TransformException&) {
        return pose;
    }
}

bool BodyMarker::shouldThrottle(double interval) const
{
    auto now = node_->now();
    auto dt = (now - last_publish_time_).seconds();
    if (dt >= interval) {
        last_publish_time_ = now;
        return true;
    }
    return false;
}

void BodyMarker::add6DofControls(
    visualization_msgs::msg::InteractiveMarker& int_marker,
    bool enable_interaction)
{
    using visualization_msgs::msg::InteractiveMarkerControl;

    if (!enable_interaction) {
        return;
    }

    auto add_control = [&](double ox, double oy, double oz,
                           uint8_t interaction_mode,
                           const std::string& name) {
        InteractiveMarkerControl control;
        control.orientation.w = 1.0;
        control.orientation.x = ox;
        control.orientation.y = oy;
        control.orientation.z = oz;
        control.name = name;
        control.interaction_mode = interaction_mode;
        int_marker.controls.push_back(control);
    };

    add_control(1, 0, 0, InteractiveMarkerControl::ROTATE_AXIS, "rotate_x");
    add_control(1, 0, 0, InteractiveMarkerControl::MOVE_AXIS,   "move_x");

    add_control(0, 1, 0, InteractiveMarkerControl::ROTATE_AXIS, "rotate_y");
    add_control(0, 1, 0, InteractiveMarkerControl::MOVE_AXIS,   "move_y");

    add_control(0, 0, 1, InteractiveMarkerControl::ROTATE_AXIS, "rotate_z");
    add_control(0, 0, 1, InteractiveMarkerControl::MOVE_AXIS,   "move_z");
}

} // namespace arms_ros2_control::command