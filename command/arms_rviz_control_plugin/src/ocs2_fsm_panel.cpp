#include "arms_rviz_control_plugin/ocs2_fsm_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QTimer>
#include <QStandardItemModel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFont>
#include <QFontMetrics>
#include <QGridLayout>
#include <QGuiApplication>
#include <QScreen>
#include <QSizePolicy>
#include <QStyledItemDelegate>
#include <QAbstractItemView>
#include <QPainter>
#include <QStylePainter>
#include <QStyleOptionComboBox>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>

namespace arms_rviz_control_plugin
{
    namespace
    {
        qreal uiScale()
        {
            qreal scale = 1.0;
            const QScreen* screen = QGuiApplication::primaryScreen();
            if (screen)
            {
                scale = std::max(scale, screen->availableGeometry().width() / 1920.0);
                scale = std::max(scale, screen->logicalDotsPerInchX() / 96.0);
            }
            return std::min(scale, 1.35);
        }

        int scaled(int value)
        {
            return std::max(1, static_cast<int>(std::round(value * uiScale())));
        }

        QFrame* makeVerticalDivider(QWidget* parent, int height)
        {
            auto* line = new QFrame(parent);
            line->setFrameShape(QFrame::VLine);
            line->setFrameShadow(QFrame::Sunken);
            line->setFixedSize(scaled(2), height);
            line->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            return line;
        }

        QString buttonStyle(const QString& background)
        {
            return QString("QPushButton { background-color: %1; color: white; font-weight: bold; font-size: %2px; }")
                .arg(background)
                .arg(scaled(13));
        }

        /** 居中绘制下拉列表项，避免默认左对齐。 */
        class CenteredItemDelegate : public QStyledItemDelegate
        {
        public:
            explicit CenteredItemDelegate(QObject* parent = nullptr)
                : QStyledItemDelegate(parent) {}

            void paint(QPainter* painter, const QStyleOptionViewItem& option,
                       const QModelIndex& index) const override
            {
                QStyleOptionViewItem centered(option);
                centered.displayAlignment = Qt::AlignCenter;
                QStyledItemDelegate::paint(painter, centered, index);
            }
        };

        /** 当前文本居中绘制，其余行为与普通 QComboBox 一致（仍可整框点击弹出）。 */
        class CenteredComboBox : public QComboBox
        {
        public:
            using QComboBox::QComboBox;

        protected:
            void paintEvent(QPaintEvent*) override
            {
                QStylePainter painter(this);
                QStyleOptionComboBox opt;
                initStyleOption(&opt);

                // 先让样式画边框和背景，文字清空以避免默认左对齐文字叠加。
                const QString text = opt.currentText;
                opt.currentText.clear();
                painter.drawComplexControl(QStyle::CC_ComboBox, opt);

                if (!text.isEmpty())
                {
                    const QRect edit_rect = style()->subControlRect(
                        QStyle::CC_ComboBox, &opt, QStyle::SC_ComboBoxEditField, this);
                    // 下拉框背景固定为白色，文字颜色显式取黑/灰，避免深色主题下不可见。
                    QPalette text_palette = opt.palette;
                    text_palette.setColor(QPalette::ButtonText,
                                          isEnabled() ? QColor(Qt::black) : QColor(150, 150, 150));
                    painter.drawItemText(edit_rect, Qt::AlignCenter, text_palette,
                                         true, text, QPalette::ButtonText);
                }
            }
        };

        /** 弹出列表项居中显示。 */
        void centerComboPopupItems(QComboBox* combo)
        {
            if (combo && combo->view())
            {
                combo->view()->setItemDelegate(new CenteredItemDelegate(combo));
            }
        }
    }  // namespace

    OCS2FSMPanel::OCS2FSMPanel(QWidget* parent)
        : Panel(parent)
    {
        const int title_font_size = scaled(14);
        const int badge_font_size = scaled(9);
        const int label_font_size = scaled(13);
        const int switch_width = scaled(72);
        const int switch_height = scaled(32);
        const int inner_spacing = scaled(4);
        const int mode_label_height = scaled(18);
        const int combo_height = scaled(28);
        const int hold_button_height = scaled(28);
        const int grid_h_spacing = scaled(12);
        const int grid_v_spacing = scaled(4);
        QFont wbc_label_font;
        wbc_label_font.setPixelSize(label_font_size);
        const QFontMetrics label_fm(wbc_label_font);
        const int pair_label_width = std::max({
            label_fm.horizontalAdvance(QString::fromUtf8("双臂耦合")),
            label_fm.horizontalAdvance(QString::fromUtf8("身体模式")),
            label_fm.horizontalAdvance(QString::fromUtf8("参考关节"))
        }) + scaled(2);
        const int pair_content_width = pair_label_width + inner_spacing + switch_width;
        const int pair_cell_width = pair_content_width + scaled(12);
        const int wbc_row_width = 2 * pair_cell_width + grid_h_spacing;
        const int compact_cell_width = (wbc_row_width - 2 * grid_h_spacing) / 3;
        // 模式下拉框单独收窄：以最长选项“自定义”为准，列宽仍按 compact_cell_width 均分。
        const int combo_width =
            label_fm.horizontalAdvance(QString::fromUtf8("自定义")) + scaled(24);
        const int panel_width = wbc_row_width;
        const int combo_padding_left = scaled(4);
        const int combo_padding_right = combo_padding_left;
        const int combo_dropdown_width = 0;

        // Create main layout
        auto* main_layout = new QVBoxLayout(this);
        setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

        // Title with status indicator - centered
        auto* title_layout = new QHBoxLayout();
        auto* title_label = new QLabel("Current FSM", this);
        title_label->setStyleSheet(QString("QLabel { font-weight: bold; font-size: %1px; }").arg(title_font_size));

        // Small status indicator
        current_state_label_ = std::make_unique<QLabel>("HOLD", this);
        current_state_label_->setStyleSheet(QString(
            "QLabel { padding: %1px %2px; background-color: #e3f2fd; border: 1px solid #2196F3; "
            "font-weight: bold; font-size: %3px; border-radius: %4px; margin: 0px; }")
            .arg(scaled(1))
            .arg(scaled(4))
            .arg(badge_font_size)
            .arg(scaled(2)));
        current_state_label_->setFixedSize(scaled(44), scaled(16));
        current_state_label_->setAlignment(Qt::AlignCenter);

        title_layout->addStretch();
        title_layout->addWidget(title_label);
        title_layout->addWidget(current_state_label_.get());
        title_layout->addStretch();
        main_layout->addLayout(title_layout);

        // State transition buttons group
        button_group_ = std::make_unique<QGroupBox>(this);
        button_group_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        auto* button_layout = new QVBoxLayout(button_group_.get());
        button_layout->setSpacing(2);

        // HOME → HOLD (command = 2)
        home_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        home_to_hold_btn_->setStyleSheet(buttonStyle("#FF9800"));
        button_layout->addWidget(home_to_hold_btn_.get());

        // Switch pose button (command = 4) - 只在HOME状态显示
        switch_pose_btn_ = std::make_unique<QPushButton>("切换姿态 (Home ↔ Rest)", this);
        switch_pose_btn_->setStyleSheet(buttonStyle("#FF5722"));
        button_layout->addWidget(switch_pose_btn_.get());

        // HOLD → OCS2 和 HOLD → MOVEJ 同一行
        auto* hold_to_row = new QHBoxLayout();
        hold_to_row->setSpacing(2);
        hold_to_ocs2_btn_ = std::make_unique<QPushButton>("OCS2", this);
        hold_to_ocs2_btn_->setStyleSheet(buttonStyle("#2196F3"));
        hold_to_row->addWidget(hold_to_ocs2_btn_.get());
        hold_to_movej_btn_ = std::make_unique<QPushButton>("MOVEJ", this);
        hold_to_movej_btn_->setStyleSheet(buttonStyle("#009688"));
        hold_to_row->addWidget(hold_to_movej_btn_.get());
        button_layout->addLayout(hold_to_row);

        // OCS2 → HOLD 和 MOVEJ → HOLD 同一行
        auto* to_hold_row = new QHBoxLayout();
        to_hold_row->setSpacing(2);
        ocs2_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        ocs2_to_hold_btn_->setStyleSheet(buttonStyle("#FF9800"));
        to_hold_row->addWidget(ocs2_to_hold_btn_.get());
        movej_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        movej_to_hold_btn_->setStyleSheet(buttonStyle("#FF9800"));
        to_hold_row->addWidget(movej_to_hold_btn_.get());
        button_layout->addLayout(to_hold_row);

        // HOLD → HOME (command = 1)
        hold_to_home_btn_ = std::make_unique<QPushButton>("HOME", this);
        hold_to_home_btn_->setStyleSheet(buttonStyle("#9C27B0"));
        button_layout->addWidget(hold_to_home_btn_.get());

        main_layout->addWidget(button_group_.get(), 0, Qt::AlignHCenter);

        // ==================== WBC Control Section (OCS2 mode only) ====================
        wbc_container_ = std::make_unique<QWidget>(this);
        wbc_container_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        wbc_layout_ = std::make_unique<QVBoxLayout>(wbc_container_.get());
        wbc_layout_->setContentsMargins(0, scaled(8), 0, 0);
        wbc_layout_->setSpacing(scaled(4));

        // Add separator line
        auto* separator = new QFrame(this);
        separator->setFrameShape(QFrame::HLine);
        separator->setFrameShadow(QFrame::Sunken);
        wbc_layout_->addWidget(separator);

        // Upper button layout (equal-width rows)
        upper_button_layout_ = std::make_unique<QGridLayout>();
        upper_button_layout_->setHorizontalSpacing(0);
        upper_button_layout_->setVerticalSpacing(grid_v_spacing);

        const QString label_style = QString("QLabel { font-size: %1px; padding: 0px; }").arg(label_font_size);

        auto makeDividerCol = [&](int height) {
            auto* wrap = new QWidget(this);
            wrap->setFixedSize(grid_h_spacing, height);
            auto* wrap_layout = new QHBoxLayout(wrap);
            wrap_layout->setContentsMargins(0, 0, 0, 0);
            wrap_layout->addWidget(makeVerticalDivider(this, height), 0, Qt::AlignHCenter);
            return wrap;
        };

        auto makeInlineCell = [&](QLabel* label, SwitchButton* sw, bool label_first) {
            label->setFixedWidth(pair_label_width);
            label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
            label->setStyleSheet(label_style);
            label->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

            auto* layout = new QHBoxLayout();
            layout->setContentsMargins(0, 0, 0, 0);
            layout->setSpacing(inner_spacing);
            if (label_first) {
                layout->addWidget(label);
                layout->addWidget(sw);
            } else {
                layout->addWidget(sw);
                layout->addWidget(label);
            }

            auto* widget = new QWidget(this);
            widget->setFixedSize(pair_content_width, switch_height);
            widget->setLayout(layout);
            return widget;
        };

        const int pair_block_height = 2 * switch_height + grid_v_spacing;
        auto* pair_block = new QWidget(this);
        pair_block->setFixedSize(wbc_row_width, pair_block_height);
        auto* pair_grid = new QGridLayout(pair_block);
        const int pair_padding = (pair_cell_width - pair_content_width) / 2;
        pair_grid->setContentsMargins(pair_padding, 0, pair_padding, 0);
        pair_grid->setHorizontalSpacing(0);
        pair_grid->setVerticalSpacing(grid_v_spacing);
        pair_grid->setColumnMinimumWidth(0, pair_label_width);
        pair_grid->setColumnMinimumWidth(1, inner_spacing);
        pair_grid->setColumnMinimumWidth(2, switch_width);
        pair_grid->setColumnMinimumWidth(3, 0);
        pair_grid->setColumnMinimumWidth(4, grid_h_spacing + 2 * pair_padding);
        pair_grid->setColumnMinimumWidth(5, pair_label_width);
        pair_grid->setColumnMinimumWidth(6, inner_spacing);
        pair_grid->setColumnMinimumWidth(7, switch_width);

        // 第一行：左臂 | 右臂（标签在外侧，开关靠中间）
        left_arm_label_ = std::make_unique<QLabel>("左臂", this);
        left_arm_switch_ = std::make_unique<SwitchButton>(this);
        pair_grid->addWidget(makeInlineCell(left_arm_label_.get(), left_arm_switch_.get(), true),
                             0, 0, 1, 3, Qt::AlignVCenter | Qt::AlignLeft);

        right_arm_label_ = std::make_unique<QLabel>("右臂", this);
        right_arm_switch_ = std::make_unique<SwitchButton>(this);
        pair_grid->addWidget(makeInlineCell(right_arm_label_.get(), right_arm_switch_.get(), false),
                             0, 5, 1, 3, Qt::AlignVCenter | Qt::AlignRight);

        pair_grid->addWidget(makeDividerCol(pair_block_height), 0, 4, 2, 1, Qt::AlignHCenter);

        // 第二行：双臂耦合 | 参考关节（同样镜像摆放）
        bimanual_label_ = std::make_unique<QLabel>("双臂耦合", this);
        bimanual_switch_ = std::make_unique<SwitchButton>(this);
        pair_grid->addWidget(makeInlineCell(bimanual_label_.get(), bimanual_switch_.get(), true),
                             1, 0, 1, 3, Qt::AlignVCenter | Qt::AlignLeft);

        home_pose_label_ = std::make_unique<QLabel>("参考关节", this);
        home_pose_switch_ = std::make_unique<SwitchButton>(this);
        pair_grid->addWidget(makeInlineCell(home_pose_label_.get(), home_pose_switch_.get(), false),
                             1, 5, 1, 3, Qt::AlignVCenter | Qt::AlignRight);

        // 第三、四行：底盘模式 / 身体模式 / 头部模式 + 三个下拉框
        const int mode_block_height = mode_label_height + grid_v_spacing + combo_height;
        auto* mode_block = new QWidget(this);
        mode_block->setFixedSize(wbc_row_width, mode_block_height);
        auto* mode_grid = new QGridLayout(mode_block);
        mode_grid->setContentsMargins(0, 0, 0, 0);
        mode_grid->setHorizontalSpacing(0);
        mode_grid->setVerticalSpacing(grid_v_spacing);
        // 5 列：模式 | 分割线 | 模式 | 分割线 | 模式。
        for (int col : {0, 2, 4}) {
            mode_grid->setColumnMinimumWidth(col, compact_cell_width);
        }
        mode_grid->setColumnMinimumWidth(1, grid_h_spacing);
        mode_grid->setColumnMinimumWidth(3, grid_h_spacing);

        auto setupModeLabel = [&](QLabel* label) {
            label->setStyleSheet(label_style);
            label->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
            label->setFixedHeight(mode_label_height);
        };

        const QString combo_style = QString(R"(
            QComboBox {
                font-size: %1px;
                padding: %2px %3px %2px %4px;
                border: 1px solid #bdbdbd;
                border-radius: %5px;
                background: white;
            }
            QComboBox:hover {
                border: 1px solid #9e9e9e;
            }
            QComboBox:focus {
                border: 1px solid #66bb6a;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: %6px;
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
            }
            QComboBox QAbstractItemView {
                font-size: %1px;
                border: 1px solid #cfcfcf;
                selection-background-color: #e8f5e9;
                selection-color: black;
                outline: none;
            }
        )")
            .arg(label_font_size)
            .arg(scaled(2))
            .arg(combo_padding_right)
            .arg(combo_padding_left)
            .arg(scaled(6))
            .arg(combo_dropdown_width);

        base_label_ = std::make_unique<QLabel>("底盘模式", this);
        setupModeLabel(base_label_.get());
        body_label_ = std::make_unique<QLabel>("身体模式", this);
        setupModeLabel(body_label_.get());
        head_label_ = std::make_unique<QLabel>("头部模式", this);
        setupModeLabel(head_label_.get());

        using State = arms_ros2_control_msgs::msg::WbcCurrentState;

        base_combo_box_ = std::make_unique<CenteredComboBox>(this);
        base_combo_box_->setFixedSize(combo_width, combo_height);
        base_combo_box_->setStyleSheet(combo_style);
        base_combo_box_->addItem("锁定", State::BASE_LOCKED);
        base_combo_box_->addItem("随动", State::BASE_UNLOCKED);
        base_combo_box_->addItem("追踪", State::BASE_TRACKING);

        body_combo_box_ = std::make_unique<CenteredComboBox>(this);
        body_combo_box_->setFixedSize(combo_width, combo_height);
        body_combo_box_->setStyleSheet(combo_style);
        body_combo_box_->addItem("自由", State::BODY_FREE);
        body_combo_box_->addItem("竖直", State::BODY_VERTICAL);
        body_combo_box_->addItem("追踪", State::BODY_TRACKING);
        body_combo_box_->addItem("锁定", State::BODY_LOCKED);
        body_combo_box_->addItem("自定义", State::BODY_CUSTOM_LOCKED);

        head_combo_box_ = std::make_unique<CenteredComboBox>(this);
        head_combo_box_->setFixedSize(combo_width, combo_height);
        head_combo_box_->setStyleSheet(combo_style);
        head_combo_box_->addItem("禁用", State::HEAD_DISABLED);
        head_combo_box_->addItem("追踪", State::HEAD_TRACKING);
        head_combo_box_->addItem("注视", State::HEAD_GAZE);
        head_combo_box_->addItem("朝前", State::HEAD_FORWARD);

        centerComboPopupItems(base_combo_box_.get());
        centerComboPopupItems(body_combo_box_.get());
        centerComboPopupItems(head_combo_box_.get());

        mode_grid->addWidget(base_label_.get(), 0, 0, Qt::AlignHCenter | Qt::AlignVCenter);
        mode_grid->addWidget(body_label_.get(), 0, 2, Qt::AlignHCenter | Qt::AlignVCenter);
        mode_grid->addWidget(head_label_.get(), 0, 4, Qt::AlignHCenter | Qt::AlignVCenter);
        mode_grid->addWidget(base_combo_box_.get(), 1, 0, Qt::AlignHCenter | Qt::AlignVCenter);
        mode_grid->addWidget(body_combo_box_.get(), 1, 2, Qt::AlignHCenter | Qt::AlignVCenter);
        mode_grid->addWidget(head_combo_box_.get(), 1, 4, Qt::AlignHCenter | Qt::AlignVCenter);
        mode_grid->addWidget(makeDividerCol(mode_block_height), 0, 1, 2, 1, Qt::AlignHCenter);
        mode_grid->addWidget(makeDividerCol(mode_block_height), 0, 3, 2, 1, Qt::AlignHCenter);

        auto* ocs2_to_hold_wbc_btn = new QPushButton("HOLD", this);
        ocs2_to_hold_wbc_btn->setStyleSheet(buttonStyle("#FF9800"));
        ocs2_to_hold_wbc_btn->setFixedHeight(hold_button_height);

        upper_button_layout_->addWidget(pair_block, 0, 0);
        upper_button_layout_->addWidget(mode_block, 1, 0);

        wbc_layout_->addLayout(upper_button_layout_.get());
        wbc_layout_->addWidget(ocs2_to_hold_wbc_btn, 0, Qt::AlignHCenter);
        wbc_container_->setFixedSize(panel_width, wbc_container_->sizeHint().height());
        button_group_->setFixedSize(panel_width, wbc_container_->height());
        button_group_->ensurePolished();
        button_layout->activate();
        ocs2_to_hold_wbc_btn->setFixedWidth(hold_to_home_btn_->width());

        main_layout->addWidget(wbc_container_.get(), 0, Qt::AlignHCenter);

        // Reserve the full WBC height before its initially hidden controls are shown.
        const auto margins = main_layout->contentsMargins();
        setMinimumSize(panel_width + margins.left() + margins.right(),
            margins.top() + title_layout->sizeHint().height()
            + main_layout->spacing() + wbc_container_->height() + margins.bottom());

        // Connect signals
        connect(home_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHomeToHold);
        connect(hold_to_ocs2_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToOCS2);
        connect(ocs2_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onOCS2ToHold);
        connect(hold_to_movej_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToMoveJ);
        connect(movej_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onMoveJToHold);
        connect(hold_to_home_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToHome);
        connect(switch_pose_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onSwitchPose);

        // Connect WBC signals
        connect(base_combo_box_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged), this, &OCS2FSMPanel::onBaseModeChanged);
        connect(bimanual_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onBimanualToggled);
        connect(left_arm_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onLeftArmToggled);
        connect(right_arm_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onRightArmToggled);
        connect(home_pose_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onHomeJointReferenceToggled);
        connect(head_combo_box_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged), this, &OCS2FSMPanel::onHeadModeChanged);
        connect(body_combo_box_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged), this, &OCS2FSMPanel::onBodyModeChanged);
        connect(ocs2_to_hold_wbc_btn, &QPushButton::clicked, this, &OCS2FSMPanel::onOCS2ToHold);

        // Initialize button visibility (initial state is HOLD)
        updateButtonVisibility();

        // Initially hide WBC container (only shown in OCS2 mode)
        wbc_container_->setVisible(false);

        // Create reset timer
        reset_timer_ = std::make_unique<QTimer>(this);
        reset_timer_->setSingleShot(true);
        reset_timer_->setInterval(100);
        connect(reset_timer_.get(), &QTimer::timeout, this, &OCS2FSMPanel::onSwitchPoseReleased);
    }

    OCS2FSMPanel::~OCS2FSMPanel() = default;

    void OCS2FSMPanel::onInitialize()
    {
        // Use RViz display context to get the node
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Check if ocs2_wbc_controller is available
        if (!node_->has_parameter("wbc_available")) {
            node_->declare_parameter("wbc_available", false);
        }
        wbc_available_ = node_->get_parameter("wbc_available").as_bool();

        if (!wbc_available_)
        {
            RCLCPP_INFO(node_->get_logger(), "ocs2_wbc_controller not found, WBC controls will be hidden");
            // Hide WBC container if not available
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }

        // 话题全部使用相对名：多机器人部署时靠节点命名空间隔离
        // （/robot_a/fsm_command 与 /robot_b/fsm_command）。
        fsm_command_publisher_ = node_->create_publisher<std_msgs::msg::Int32>(
            "fsm_command", 10);

        // 订阅真实状态（transient_local：控制器只在变化时发，后加入也能拿到）
        fsm_state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "fsm_state", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&OCS2FSMPanel::onFsmStateReceived, this, std::placeholders::_1));

        // Only create WBC related ROS2 interfaces if controller is available
        if (wbc_available_)
        {
            // Create WBC mode command publisher
            mode_command_pub_ = node_->create_publisher<std_msgs::msg::String>(
                "mode_command", 10);

            // Create WBC subscribers
            capability_sub_ = node_->create_subscription<arms_ros2_control_msgs::msg::WbcCapability>(
                "ocs2_wbc_controller/wbc_capabilities",
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(&OCS2FSMPanel::onReceiveCapability, this, std::placeholders::_1));

            state_sub_ = node_->create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
                "ocs2_wbc_controller/current_state",
                rclcpp::QoS(1).reliable().transient_local(),
                std::bind(&OCS2FSMPanel::onReceiveCurrentState, this, std::placeholders::_1));
    }

    RCLCPP_INFO(node_->get_logger(), "OCS2 FSM Panel initialized with fsm_command / fsm_state topics");
    }

    void OCS2FSMPanel::onHomeToHold()
    {
        publishCommand(2);
    }

    void OCS2FSMPanel::onHoldToOCS2()
    {
        publishCommand(3);
    }

    void OCS2FSMPanel::onOCS2ToHold()
    {
        publishCommand(2);
    }

    void OCS2FSMPanel::onHoldToHome()
    {
        publishCommand(1);
    }

    void OCS2FSMPanel::onHoldToMoveJ()
    {
        publishCommand(4);
    }

    void OCS2FSMPanel::onMoveJToHold()
    {
        publishCommand(2);
    }

    void OCS2FSMPanel::onSwitchPose()
    {
        if (!switch_pose_pressed_)
        {
            publishCommand(100);
            switch_pose_pressed_ = true;
            reset_timer_->start();
        }
    }

    void OCS2FSMPanel::onSwitchPoseReleased()
    {
        publishCommand(0);
        switch_pose_pressed_ = false;
    }

    void OCS2FSMPanel::publishCommand(int32_t command)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = command;
        fsm_command_publisher_->publish(msg);
        current_command_ = command;
        RCLCPP_INFO(node_->get_logger(), "Published FSM command: %d", command);
    }

    void OCS2FSMPanel::updateStatusDisplay()
    {
        updateButtonVisibility();
    }

    void OCS2FSMPanel::updateButtonVisibility()
    {
        // Update FSM buttons visibility based on current state
        if (current_state_ == "HOME")
        {
            // Show button group
            button_group_->setVisible(true);

            hold_to_ocs2_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
            home_to_hold_btn_->setVisible(true);
            home_to_hold_btn_->setEnabled(true);
            switch_pose_btn_->setVisible(true);
            switch_pose_btn_->setEnabled(true);

            // Hide WBC controls in HOME mode (if available)
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }
        else if (current_state_ == "HOLD")
        {
            // Show button group
            button_group_->setVisible(true);

            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(true);
            hold_to_home_btn_->setEnabled(true);
            hold_to_ocs2_btn_->setVisible(true);
            hold_to_ocs2_btn_->setEnabled(true);
            hold_to_movej_btn_->setVisible(true);
            hold_to_movej_btn_->setEnabled(true);

            // Hide WBC controls in HOLD mode (if available)
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }
        else if (current_state_ == "OCS2")
        {
            if (wbc_available_)
            {
                // WBC模式：隐藏原按钮组，显示WBC控件
                button_group_->setVisible(false);

                if (wbc_container_)
                {
                    wbc_container_->setVisible(true);
                }
                refreshWbcUi();
            }
            else
            {
                // 非WBC模式：仍显示原按钮组，但只保留 OCS2 -> HOLD 按钮
                button_group_->setVisible(true);

                home_to_hold_btn_->setVisible(false);
                switch_pose_btn_->setVisible(false);
                hold_to_ocs2_btn_->setVisible(false);
                hold_to_movej_btn_->setVisible(false);
                movej_to_hold_btn_->setVisible(false);
                hold_to_home_btn_->setVisible(false);

                ocs2_to_hold_btn_->setVisible(true);
                ocs2_to_hold_btn_->setEnabled(true);

                if (wbc_container_)
                {
                    wbc_container_->setVisible(false);
                }
            }
        }
        else if (current_state_ == "MOVEJ")
        {
            // Show button group
            button_group_->setVisible(true);

            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(true);
            movej_to_hold_btn_->setEnabled(true);

            // Hide WBC controls in MOVEJ mode
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }
        else
        {
            // Show button group but hide all buttons
            button_group_->setVisible(true);

            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);

            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }

        if (layout())
        {
            layout()->invalidate();
            layout()->activate();
        }
    }

    void OCS2FSMPanel::setCurrentState(const std::string& state)
    {
        current_state_ = state;
        current_state_label_->setText(QString::fromStdString(state));
        updateButtonVisibility();
    }

    void OCS2FSMPanel::load(const rviz_common::Config& config)
    {
        Panel::load(config);
    }

    void OCS2FSMPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void OCS2FSMPanel::onFsmStateReceived(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        // 直接采用控制器报的真实状态。这里不再用本地规则表推断：
        // 规则表与控制器实际行为并不一致（例如 HOME + 命令 3 控制器会回 HOLD）。
        static constexpr const char* kNames[] = {"RESET", "HOME", "HOLD", "OCS2", "MOVEJ"};
        const int32_t code = msg->data;
        const char* name = (code >= 0 && code <= 4) ? kNames[code] : "UNKNOWN";

        current_command_ = code;
        // 本回调运行在 rviz 的 executor 线程，标签/按钮只能在 GUI 线程改。
        const std::string state_name(name);
        QMetaObject::invokeMethod(
            this, [this, state_name]() { setCurrentState(state_name); }, Qt::QueuedConnection);
    }

    // ==================== WBC Control Methods ====================

    void OCS2FSMPanel::onReceiveCapability(const arms_ros2_control_msgs::msg::WbcCapability::SharedPtr msg)
    {
        if (!msg) return;

        capability_state_.has_mobile_base = msg->has_mobile_base;
        capability_state_.has_body_relative_constraint = msg->has_body_relative_constraint;
        capability_state_.has_waist_lock = msg->has_waist_lock;
        capability_state_.has_head_coupling = msg->has_head_coupling;
        capability_state_.has_custom_joint_lock = msg->has_custom_joint_lock;
        capability_state_.has_bimanual_coupling = msg->has_bimanual_coupling;
        capability_state_.body_tracking_ee_enabled = msg->body_tracking_ee_enabled;
        capability_state_.head_tracking_ee_enabled = msg->head_tracking_ee_enabled;
        capability_state_.head_midpoint_gaze_enabled = msg->head_midpoint_gaze_enabled;
        capability_state_.has_home_joint_reference = msg->has_home_joint_reference;
        capability_state_.base_tracking_enabled = msg->base_tracking_enabled;

        QMetaObject::invokeMethod(this, [this]() { refreshWbcUi(); }, Qt::QueuedConnection);
    }

    void OCS2FSMPanel::onReceiveCurrentState(const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg)
    {
        if (!msg) return;

        current_wbc_state_.base_state = msg->base_state;
        current_wbc_state_.body_state = msg->body_state;
        current_wbc_state_.head_state = msg->head_state;
        current_wbc_state_.bimanual_state = msg->bimanual_state;
        current_wbc_state_.left_arm_state = msg->left_arm_state;
        current_wbc_state_.right_arm_state = msg->right_arm_state;
        current_wbc_state_.home_joint_reference_enabled = msg->home_joint_reference_enabled;

        QMetaObject::invokeMethod(this, [this]() { refreshWbcUi(); }, Qt::QueuedConnection);
    }


    int OCS2FSMPanel::getCurrentBaseModeIndex() const
    {
        return current_wbc_state_.base_state;
    }

    QString OCS2FSMPanel::getBaseModeCommand(int modeIndex) const
    {
        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        switch (modeIndex)
        {
            case State::BASE_LOCKED:
                return "BASE_LOCK";
            case State::BASE_UNLOCKED:
                return "BASE_UNLOCK";
            case State::BASE_TRACKING:
                return "BASE_TRACK";
            default:
                return "";
        }
    }

    void OCS2FSMPanel::updateBaseComboBox()
    {
        if (!base_combo_box_) return;

        base_combo_box_->blockSignals(true);
        auto* model = qobject_cast<QStandardItemModel*>(base_combo_box_->model());
        const bool available[] = {capability_state_.has_mobile_base,
                                  capability_state_.has_mobile_base,
                                  capability_state_.base_tracking_enabled};
        int enabled_count = 0;
        for (int i = 0; i < base_combo_box_->count() && i < 3; ++i)
        {
            model->item(i)->setEnabled(available[i]);
            if (available[i]) ++enabled_count;
        }
        base_combo_box_->setCurrentIndex(base_combo_box_->findData(getCurrentBaseModeIndex()));
        base_combo_box_->setEnabled(enabled_count > 1);
        base_combo_box_->setToolTip("锁定：底盘不动；随动：底盘由 OCS2 自由决定；跟随：底盘跟踪世界系 marker");
        base_combo_box_->blockSignals(false);
    }


    int OCS2FSMPanel::getCurrentBodyModeIndex() const
    {
        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        switch (current_wbc_state_.body_state)
        {
            case State::BODY_FREE:
            case State::BODY_VERTICAL:
            case State::BODY_TRACKING:
            case State::BODY_LOCKED:
            case State::BODY_CUSTOM_LOCKED:
                return current_wbc_state_.body_state;
            default:
                return State::BODY_LOCKED;
        }
    }

    QString OCS2FSMPanel::getBodyModeCommand(int modeIndex) const
    {
        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        switch (modeIndex)
        {
            case State::BODY_FREE:
                return "BODY_FREE";
            case State::BODY_VERTICAL:
                return "BODY_RELATIVE";
            case State::BODY_TRACKING:
                return "BODY_TRACKING";
            case State::BODY_LOCKED:
                return "BODY_LOCK";
            case State::BODY_CUSTOM_LOCKED:
                return "BODY_CUSTOM_LOCK";
            default:
                return "";
        }
    }

    int OCS2FSMPanel::getCurrentHeadModeIndex() const
    {
        return current_wbc_state_.head_state;
    }

    QString OCS2FSMPanel::getHeadModeCommand(int modeIndex) const
    {
        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        switch (modeIndex)
        {
            case State::HEAD_DISABLED:
                return "HEAD_DISABLE";
            case State::HEAD_TRACKING:
                return "HEAD_TRACKING";
            case State::HEAD_GAZE:
                return "HEAD_GAZE";
            case State::HEAD_FORWARD:
                return "HEAD_FORWARD";
            default:
                return "";
        }
    }

    void OCS2FSMPanel::updateBodyComboBox()
    {
        if (!body_combo_box_) return;

        body_combo_box_->blockSignals(true);
        auto* model = qobject_cast<QStandardItemModel*>(body_combo_box_->model());
        const bool available[] = {true,
                                  capability_state_.has_body_relative_constraint,
                                  capability_state_.body_tracking_ee_enabled,
                                  capability_state_.has_waist_lock,
                                  capability_state_.has_custom_joint_lock};
        int enabled_count = 0;
        for (int i = 0; i < 5; ++i)
        {
            model->item(i)->setEnabled(available[i]);
            if (available[i]) ++enabled_count;
        }
        body_combo_box_->setCurrentIndex(body_combo_box_->findData(getCurrentBodyModeIndex()));

        const bool forward = current_wbc_state_.head_state == arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_FORWARD;
        body_combo_box_->setEnabled(enabled_count > 1 && !forward);
        body_combo_box_->setToolTip(forward ? "朝前模式使用头腰联动及腰部约束；退出后恢复此身体模式" : "");
        body_combo_box_->blockSignals(false);
    }

    void OCS2FSMPanel::updateHeadComboBox()
    {
        head_combo_box_->blockSignals(true);
        auto* model = qobject_cast<QStandardItemModel*>(head_combo_box_->model());
        const bool available[] = {true, capability_state_.head_tracking_ee_enabled,
                                  capability_state_.head_midpoint_gaze_enabled,
                                  capability_state_.has_head_coupling};
        int enabled_count = 0;
        for (int i = 0; i < head_combo_box_->count() && i < 4; ++i)
        {
            model->item(i)->setEnabled(available[i]);
            if (available[i]) ++enabled_count;
        }
        head_combo_box_->setCurrentIndex(head_combo_box_->findData(getCurrentHeadModeIndex()));
        head_combo_box_->setEnabled(enabled_count > 1);
        head_combo_box_->setToolTip("禁用：保持头部关节；追踪：头部 marker 控制；注视：双手末端中点；朝前：头腰联动及腰部约束");
        head_combo_box_->blockSignals(false);
    }



    void OCS2FSMPanel::updateSwitchVisualState(SwitchButton* sw, bool capability_available, bool logical_on)
    {
        if (!sw) return;

        if (!capability_available)
        {
            sw->setVisualState(SwitchButton::VisualState::InvalidOffGray);
            sw->setClickable(false);
            return;
        }

        sw->setClickable(true);

        if (logical_on)
        {
            sw->setVisualState(SwitchButton::VisualState::EnabledOnGreen);
        }
        else
        {
            sw->setVisualState(SwitchButton::VisualState::DisabledOffRed);
        }
    }

    void OCS2FSMPanel::refreshWbcUi()
    {
        if (!wbc_container_ || !wbc_container_->isVisible() || !wbc_available_) return;

        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        const bool left_enabled = current_wbc_state_.left_arm_state == State::ARM_ENABLED;
        const bool right_enabled = current_wbc_state_.right_arm_state == State::ARM_ENABLED;
        const bool bimanual_coupled = current_wbc_state_.bimanual_state == State::BIMANUAL_COUPLED;
        const bool any_arm_disabled = (!left_enabled || !right_enabled);

        // 底盘模式由下拉框显示，锁定/随动/跟随三态直接对应控制器状态。
        updateBaseComboBox();

        // 限制1：任意一侧手臂禁用时，不允许开启双臂耦合，并显示灰色OFF
        if (!capability_state_.has_bimanual_coupling || any_arm_disabled)
        {
            bimanual_switch_->setVisualState(SwitchButton::VisualState::InvalidOffGray);
            bimanual_switch_->setClickable(false);
        }
        else
        {
            updateSwitchVisualState(bimanual_switch_.get(), true, bimanual_coupled);
        }

        // 限制2：双臂耦合启用时，不允许调整左右臂禁用，并显示为ON
        if (bimanual_coupled)
        {
            left_arm_switch_->setVisualState(SwitchButton::VisualState::EnabledOnGreen);
            left_arm_switch_->setClickable(false);
            right_arm_switch_->setVisualState(SwitchButton::VisualState::EnabledOnGreen);
            right_arm_switch_->setClickable(false);
        }
        else
        {
            updateSwitchVisualState(left_arm_switch_.get(), true, left_enabled);
            updateSwitchVisualState(right_arm_switch_.get(), true, right_enabled);
        }

        updateSwitchVisualState(home_pose_switch_.get(),
                                capability_state_.has_home_joint_reference,
                                current_wbc_state_.home_joint_reference_enabled);

        updateBodyComboBox();
        updateHeadComboBox();
    }

    void OCS2FSMPanel::publishModeCommand(const std::string& cmd)
    {
        if (!mode_command_pub_) return;

        std_msgs::msg::String msg;
        msg.data = cmd;
        mode_command_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published mode command: %s", cmd.c_str());
    }

    void OCS2FSMPanel::onBaseModeChanged(int index)
    {
        if (index < 0) return;
        const int mode = base_combo_box_->itemData(index).toInt();
        if (mode == getCurrentBaseModeIndex()) return;
        const QString command = getBaseModeCommand(mode);
        if (command.isEmpty()) return;
        publishModeCommand(command.toStdString());
        // Keep displaying the acknowledged mode until the controller accepts the command.
        updateBaseComboBox();
    }

    void OCS2FSMPanel::onBodyModeChanged(int index)
    {
        if (index < 0) return;
        const int mode = body_combo_box_->itemData(index).toInt();
        if (mode == getCurrentBodyModeIndex()) return;
        const QString command = getBodyModeCommand(mode);
        if (command.isEmpty()) return;
        publishModeCommand(command.toStdString());
        // Keep displaying the acknowledged mode until the controller accepts the command.
        updateBodyComboBox();
    }

    void OCS2FSMPanel::onHeadModeChanged(int index)
    {
        if (index < 0) return;
        const int mode = head_combo_box_->itemData(index).toInt();
        if (mode == getCurrentHeadModeIndex()) return;
        const QString command = getHeadModeCommand(mode);
        if (command.isEmpty()) return;
        publishModeCommand(command.toStdString());
        // Keep displaying the acknowledged mode until the controller accepts the command.
        updateHeadComboBox();
    }



    void OCS2FSMPanel::onLeftArmToggled()
    {
        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        // 双臂耦合启用时，禁止调整左右臂禁用状态
        if (current_wbc_state_.bimanual_state == State::BIMANUAL_COUPLED)
        {
            RCLCPP_INFO(node_->get_logger(), "Left arm toggle blocked: bimanual coupling is enabled");
            refreshWbcUi();
            return;
        }
        publishModeCommand(current_wbc_state_.left_arm_state == State::ARM_ENABLED
                               ? "LEFT_ARM_DISABLE"
                               : "LEFT_ARM_ENABLE");
    }

    void OCS2FSMPanel::onRightArmToggled()
    {
        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        // 双臂耦合启用时，禁止调整左右臂禁用状态
        if (current_wbc_state_.bimanual_state == State::BIMANUAL_COUPLED)
        {
            RCLCPP_INFO(node_->get_logger(), "Right arm toggle blocked: bimanual coupling is enabled");
            refreshWbcUi();
            return;
        }
        publishModeCommand(current_wbc_state_.right_arm_state == State::ARM_ENABLED
                               ? "RIGHT_ARM_DISABLE"
                               : "RIGHT_ARM_ENABLE");
    }

    void OCS2FSMPanel::onBimanualToggled()
    {
        using State = arms_ros2_control_msgs::msg::WbcCurrentState;
        if (!capability_state_.has_bimanual_coupling) return;
        // 任意一侧手臂禁用时，禁止开启双臂耦合
        if (current_wbc_state_.left_arm_state != State::ARM_ENABLED ||
            current_wbc_state_.right_arm_state != State::ARM_ENABLED)
        {
            RCLCPP_INFO(node_->get_logger(), "Bimanual coupling blocked: one arm is disabled");
            refreshWbcUi();
            return;
        }
        publishModeCommand(current_wbc_state_.bimanual_state == State::BIMANUAL_COUPLED
                               ? "ARMS_INDEPENDENT"
                               : "ARMS_COUPLED");
    }

    void OCS2FSMPanel::onHomeJointReferenceToggled()
    {
        if (!capability_state_.has_home_joint_reference) return;
        publishModeCommand(current_wbc_state_.home_joint_reference_enabled
                               ? "HOME_JOINT_OFF"
                               : "HOME_JOINT_ON");
    }



} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::OCS2FSMPanel, rviz_common::Panel)
