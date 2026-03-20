#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

namespace arms_rviz_control_plugin
{
    class WaistControlPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit WaistControlPanel(QWidget* parent = nullptr);
        ~WaistControlPanel() override;

        void onInitialize() override;
        void load(const rviz_common::Config& config) override;
        void save(rviz_common::Config config) const override;

    private Q_SLOTS:
        void onLiftingSliderChanged(int value);
        void onTurningSliderChanged(int value);

        void onUpPressed();
        void onUpReleased();

        void onDownPressed();
        void onDownReleased();

        void onLeftPressed();
        void onLeftReleased();

        void onRightPressed();
        void onRightReleased();

        void onRepeatTimeout();

    private:
        void publishLifting(double value);
        void publishTurning(double value);

        double getLiftingScale() const;
        double getTurningScale() const;
        void updateScaleLabels();

        void updateRepeatTimerState();
        void stopLifting();
        void stopTurning();

    private:
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_lifting_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_turning_publisher_;

        std::unique_ptr<QVBoxLayout> main_layout_;

        std::unique_ptr<QVBoxLayout> lifting_layout_;
        std::unique_ptr<QHBoxLayout> lifting_slider_layout_;
        std::unique_ptr<QLabel> lifting_label_;
        std::unique_ptr<QSlider> lifting_slider_;
        std::unique_ptr<QLabel> lifting_value_label_;

        std::unique_ptr<QVBoxLayout> turning_layout_;
        std::unique_ptr<QHBoxLayout> turning_slider_layout_;
        std::unique_ptr<QLabel> turning_label_;
        std::unique_ptr<QSlider> turning_slider_;
        std::unique_ptr<QLabel> turning_value_label_;

        std::unique_ptr<QHBoxLayout> button_layout_top_;
        std::unique_ptr<QHBoxLayout> button_layout_bottom_;

        std::unique_ptr<QPushButton> up_button_;
        std::unique_ptr<QPushButton> down_button_;
        std::unique_ptr<QPushButton> left_button_;
        std::unique_ptr<QPushButton> right_button_;

        std::unique_ptr<QTimer> repeat_timer_;

        bool up_pressed_;
        bool down_pressed_;
        bool left_pressed_;
        bool right_pressed_;
    };
} // namespace arms_rviz_control_plugin