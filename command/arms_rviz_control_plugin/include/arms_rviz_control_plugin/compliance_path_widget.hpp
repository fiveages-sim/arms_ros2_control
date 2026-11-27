#ifndef ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_PATH_WIDGET_HPP
#define ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_PATH_WIDGET_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStringList>
#include <QTableWidget>
#include <QWidget>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arms_ros2_control_msgs/msg/compliance_path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace arms_rviz_control_plugin
{

/**
 * @brief 折线轨迹编辑器：多个脚点 + 总运行时间 → compliance_path 一次性下发。
 *
 * 与控制器侧 PathPlan 的约定（见 CompliancePath.msg）：
 *   blend_radius = 0（本面板固定发 0）
 *       → 每段各自一条梯形曲线，段末速度归零；角点停到 0 再转向，
 *         速度方向不突变，因此不产生法向加速度，也不存在切角误差。
 *   duration > 0
 *       → 忽略 speed/accel 的绝对值，按 v→v/k、a→a/k² 等比缩放命中该总时长
 *         （含全部 loops，不含入段）。
 *
 * 前提：控制器必须处于 COMPLIANCE 状态；且需要把
 * compliance_task_selection 全设为 0，否则被选为力控的轴不会跟轨迹。
 */
class CompliancePathWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CompliancePathWidget(QWidget* parent = nullptr);

    /// 由宿主面板在 onInitialize() 里调用（RViz 的 ROS 节点）。
    /// default_frame = RViz 当前的 Fixed Frame：启动时拿它作脚点坐标系，
    /// 这样分体（base_footprint）/ 全身（world 或 task 配置的控制系）都不用改。
    void initialize(const rclcpp::Node::SharedPtr& node,
                    const QString& default_frame = QString());

    /// 跟随 RViz 的 Fixed Frame。用户手动改过「坐标系」之后不再覆盖。
    void setDefaultFrame(const QString& frame);

    /// 控制器上报的路径回放状态（来自 compliance_force_status.path_state[2]）。
    /// 只有 arm 与当前选择一致、且 sequence 是新的时才刷新提示文字。
    void onControllerPathState(const QString& arm, uint32_t sequence,
                               const QString& state, const QString& message);

private Q_SLOTS:
    void onAddLine();
    void onRemoveLine();
    void onMoveUp();
    void onMoveDown();
    void onCapturePose();
    void onClearLines();
    void onSend();
    void onStop();

private:
    /// 解析「脚点文本」重建 groups_ / 当前组 / 结果表 / 摘要 / Marker。
    void parseInput();
    /// 按 groups_ 重建「组」下拉框（保持 active_group_ 不变）。
    void rebuildGroupCombo();
    /// 把 groups_[active_group_] 的脚点装进 parsed_ 并刷新表格/摘要/Marker。
    void loadActiveGroup();
    /// 把 parsed_ 投影到只读结果表（含 RPY 提示）。
    void refreshResultTable();
    void refreshSummary();
    void setStatus(const QString& text, const char* color);
    /// 把当前脚点画到 RViz 3D 视图：/compliance_path_markers（MarkerArray）。
    /// 任何增删改都会重发；另有低频心跳，保证后加的 Marker 显示也能拿到。
    void publishMarkers();
    std::string sideName() const;

    QStringList inputLines() const;
    void setInputLines(const QStringList& lines);
    /// 当前光标所在行（0 基）。
    int currentLine() const;
    void setCurrentLine(int line);
    void moveCurrentLine(int delta);
    /// 位姿 → 一行文本（X Y Z R P Y，米 / 度）。
    QString poseToLine(const geometry_msgs::msg::Pose& pose) const;
    /// 当前实测位姿（拿不到时给单位位姿）。
    geometry_msgs::msg::Pose currentPoseOrIdentity();
    /// 输入为空且已拿到当前位姿时，自动填第一行。
    void seedFromCurrentPose();
    /// 5 Hz 轮询当前位姿快照（ROS 回调只存值，不跨线程操作 Qt）。
    void refreshPoseState();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::CompliancePath>::SharedPtr path_pub_left_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::CompliancePath>::SharedPtr path_pub_right_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr single_pub_left_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr single_pub_right_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_left_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_right_;

    QComboBox* arm_combo_{nullptr};
    QLineEdit* frame_edit_{nullptr};
    QPlainTextEdit* input_{nullptr};
    QComboBox* group_combo_{nullptr};
    QTableWidget* table_{nullptr};
    QDoubleSpinBox* duration_spin_{nullptr};
    QSpinBox* loops_spin_{nullptr};
    QPushButton* add_btn_{nullptr};
    QPushButton* remove_btn_{nullptr};
    QPushButton* up_btn_{nullptr};
    QPushButton* down_btn_{nullptr};
    QPushButton* capture_btn_{nullptr};
    QPushButton* clear_btn_{nullptr};
    QPushButton* send_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
    QLabel* summary_label_{nullptr};
    QLabel* status_label_{nullptr};
    QTimer* pose_timer_{nullptr};
    QTimer* parse_timer_{nullptr};

    /**
     * @brief 一条独立路径。
     *
     * 分组规则（见 8.2.2）：**一个矩阵 = 一条路径**。
     * 顶层方括号里还套了一层方括号（`[[…], […]]`）就是一个矩阵组；
     * 没有套层的裸行 / 单层 `[…]` 行归到同一个「松散组」（第 1 组），
     * 这样旧的"每行一个点"粘贴方式仍然只有一组，行为不变。
     */
    struct PathGroup
    {
        /// 一个位姿在输入框里的位置：只记行号和字符区间，删的时候精确挖掉
        /// 那 7 个数，不动方括号/缩进/注释，也不影响同一行里的别的点。
        struct PointSpan
        {
            int line{0};        ///< 1 基行号
            int col_begin{0};   ///< 该行内的起始字符下标（含）
            int col_end{0};     ///< 该行内的结束字符下标（不含）
        };

        QString label;                              ///< 组名：上方紧邻的注释，或「路径 N」
        std::vector<geometry_msgs::msg::Pose> poses;
        std::vector<PointSpan> spans;               ///< 与 poses 一一对应
        QStringList errors;                         ///< 该组的解析错误（有错就不能下发）
    };

    /// 当前选中组的解析结果：结果表 / Marker / 下发都用这一份，保证三处一致。
    std::vector<geometry_msgs::msg::Pose> parsed_;
    /// 本次解析出的全部路径组（下拉框可切换）。
    std::vector<PathGroup> groups_;
    int active_group_{0};
    int bad_lines_{0};
    QString first_error_;

    /// `/left_current_pose` 是控制周期级频率（~500 Hz）。ROS 回调只把值存进
    /// 这里，Qt 线程靠 pose_timer_ 5 Hz 轮询 —— 否则事件队列会被打满。
    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped pose_left_{};
    geometry_msgs::msg::PoseStamped pose_right_{};
    bool have_pose_left_{false};
    bool have_pose_right_{false};
    bool seeded_pose_row_{false};


    /// 自动跟随 Fixed Frame 的痕迹：auto_frame_ 是上一次自动填的值，
    /// 一旦输入框的内容和它不一致，就当成用户手动指定，不再自动覆盖。
    QString auto_frame_;
    bool applying_auto_frame_{false};
    bool frame_user_locked_{false};
    /// 已处理过的控制器事件序号（按当前选中的臂）。
    uint32_t controller_sequence_{0};
};

}  // namespace arms_rviz_control_plugin

#endif  // ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_PATH_WIDGET_HPP
