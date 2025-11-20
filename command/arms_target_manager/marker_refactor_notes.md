# ArmsTargetManager 抽象记录

## 保留在 `ArmsTargetManager` 的函数列表

### 1. 构造函数 `ArmsTargetManager(...)`

**完整代码：**
```cpp
ArmsTargetManager::ArmsTargetManager(
    rclcpp::Node::SharedPtr node,
    const std::string& topicPrefix,
    bool dualArmMode,
    const std::string& frameId,
    const std::string& markerFixedFrame,
    double publishRate,
    const std::vector<int32_t>& disableAutoUpdateStates,
    double markerUpdateInterval,
    bool enableHeadControl,
    const std::string& headMarkerFrame,
    const std::string& headControllerName,
    const std::array<double, 3>& headMarkerPosition)
    : node_(std::move(node))
      , topic_prefix_(topicPrefix)
      , dual_arm_mode_(dualArmMode)
      , control_base_frame_(frameId)
      , marker_fixed_frame_(markerFixedFrame)
      , publish_rate_(publishRate)
      , current_mode_(MarkerState::SINGLE_SHOT)
      , current_controller_state_(2)
      , auto_update_enabled_(true)
      , disable_auto_update_states_(disableAutoUpdateStates)
      , last_marker_update_time_(node_->now())
      , marker_update_interval_(markerUpdateInterval)
      , enable_head_control_(enableHeadControl)
      , head_marker_frame_(headMarkerFrame)
      , head_controller_name_(headControllerName)
      , head_marker_position_(headMarkerPosition)
{
    left_pose_.position.x = 0.0;
    left_pose_.position.y = 0.5;
    left_pose_.position.z = 1.0;
    left_pose_.orientation.w = 1.0;
    left_pose_.orientation.x = 0.0;
    left_pose_.orientation.y = 0.0;
    left_pose_.orientation.z = 0.0;

    right_pose_.position.x = 0.0;
    right_pose_.position.y = -0.5;
    right_pose_.position.z = 1.0;
    right_pose_.orientation.w = 1.0;
    right_pose_.orientation.x = 0.0;
    right_pose_.orientation.y = 0.0;
    right_pose_.orientation.z = 0.0;

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "arms_target_manager", node_);
    left_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>(
        "left_target", 1);

    if (dual_arm_mode_)
    {
        right_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>(
            "right_target", 1);
    }

    // 初始化TF2 buffer和listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 初始化头部pose
    head_pose_.position.x = head_marker_position_[0];
    head_pose_.position.y = head_marker_position_[1];
    head_pose_.position.z = head_marker_position_[2];
    head_pose_.orientation.w = 1.0;
    head_pose_.orientation.x = 0.0;
    head_pose_.orientation.y = 0.0;
    head_pose_.orientation.z = 0.0;
}
```

- **依赖参数**：`node`, `topicPrefix`, `dualArmMode`, `frameId`, `markerFixedFrame`, `publishRate`, `disableAutoUpdateStates`, `markerUpdateInterval`, `enableHeadControl`, `headMarkerFrame`, `headControllerName`, `headMarkerPosition`
- **理由**：作为业务入口接收全局配置、控制模式、坐标系等上层逻辑；`InteractiveMarkerManager` 不关心这些业务参数

### 2. `initialize()`

**完整代码：**
```20:214:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::initialize()
{
    setupMenu();

    auto leftMarker = createMarker("left_arm_target", "left");
    server_->insert(leftMarker);

    auto leftCallback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        leftMarkerCallback(feedback);
    };
    server_->setCallback(leftMarker.name, leftCallback);

    left_menu_handler_->apply(*server_, leftMarker.name);

    if (dual_arm_mode_)
    {
        auto rightMarker = createMarker("right_arm_target", "right");
        server_->insert(rightMarker);

        auto rightCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            rightMarkerCallback(feedback);
        };
        server_->setCallback(rightMarker.name, rightCallback);

        right_menu_handler_->apply(*server_, rightMarker.name);
    }

    // 如果启用头部控制，初始化头部marker
    if (enable_head_control_)
    {
        setupHeadMenu();
        
        // 创建头部发布器
        std::string head_topic = "/" + head_controller_name_ + "/target_joint_position";
        head_joint_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            head_topic, 1);

        // 从 TF 获取 head_link2 的初始位置
        try
        {
            // 获取 head_link2 在 head_marker_frame_ 中的初始位置
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                head_marker_frame_, HEAD_LINK_NAME, tf2::TimePointZero);
            
            head_pose_.position.x = transform.transform.translation.x;
            head_pose_.position.y = transform.transform.translation.y;
            head_pose_.position.z = transform.transform.translation.z;
            
            RCLCPP_INFO(node_->get_logger(),
                       "Initialized head marker position from TF: [%.3f, %.3f, %.3f] (link: %s)",
                       head_pose_.position.x, head_pose_.position.y, head_pose_.position.z,
                       HEAD_LINK_NAME);
        }
        catch (const tf2::TransformException& ex)
        {
            // 如果 TF 转换失败，使用配置的固定位置
            RCLCPP_WARN(node_->get_logger(),
                       "无法从 TF 获取头部 link %s 的初始位置: %s，使用配置的固定位置 [%.3f, %.3f, %.3f]",
                       HEAD_LINK_NAME, ex.what(),
                       head_marker_position_[0], head_marker_position_[1], head_marker_position_[2]);
            // head_pose_.position 已经在构造函数中从 head_marker_position_ 初始化
        }

        auto headMarker = createHeadMarker();
        server_->insert(headMarker);

        auto headCallback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            headMarkerCallback(feedback);
        };
        server_->setCallback(headMarker.name, headCallback);

        head_menu_handler_->apply(*server_, headMarker.name);
    }

    updateMenuVisibility();

    left_end_effector_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "left_current_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
        {
            leftEndEffectorPoseCallback(msg);
        });

    if (dual_arm_mode_)
    {
        right_end_effector_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "right_current_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                rightEndEffectorPoseCallback(msg);
            });
    }

    // 如果启用头部控制，订阅关节状态以自动更新头部 marker
    if (enable_head_control_)
    {
        head_joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg)
            {
                headJointStateCallback(msg);
            });
    }

    server_->applyChanges();

    RCLCPP_INFO(node_->get_logger(),
                "ArmsTargetManager initialized. Mode: %s, Control Base Frame: %s, Marker Fixed Frame: %s, Publish Rate: %.1f Hz",
                dual_arm_mode_ ? "dual_arm" : "single_arm",
                control_base_frame_.c_str(),
                marker_fixed_frame_.c_str(),
                publish_rate_);

    RCLCPP_INFO(node_->get_logger(),
                "📍 Markers will be created in frame: %s",
                marker_fixed_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(),
                "🔄 Received current_pose will be transformed to marker frame: %s",
                marker_fixed_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(),
                "📤 Published target poses will be transformed to control base frame: %s",
                control_base_frame_.c_str());
}
```

- **依赖参数**：无（使用成员变量）
- **依赖成员**：`node_`, `server_`, `left_pose_`, `right_pose_`, `head_pose_`, `dual_arm_mode_`, `enable_head_control_`, `marker_fixed_frame_`, `tf_buffer_`, `left_pose_publisher_`, `right_pose_publisher_`, `head_joint_publisher_`
- **理由**：业务层初始化，决定何时创建 marker、绑定回调、订阅话题、应用菜单等，涉及业务逻辑的调度时机

### 3. `setMarkerPose(const std::string& armType, const geometry_msgs::msg::Point& position, const geometry_msgs::msg::Quaternion& orientation)`

**完整代码：**
```216:266:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::setMarkerPose(
    const std::string& armType,
    const geometry_msgs::msg::Point& position,
    const geometry_msgs::msg::Quaternion& orientation)
{
    geometry_msgs::msg::Pose* current_pose = nullptr;
    std::string marker_name;


    if (armType == "left")
    {
        current_pose = &left_pose_;
        marker_name = "left_arm_target";
    }
    else if (armType == "right" && dual_arm_mode_)
    {
        current_pose = &right_pose_;
        marker_name = "right_arm_target";
    }
    else
    {
        return; // 无效的手臂类型
    }

    current_pose->position = position;
    current_pose->orientation = orientation;

    // 更新marker
    if (server_)
    {
        server_->setPose(marker_name, *current_pose);
        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }

    // 在连续发布模式下，发送target pose（需要转换到control_base_frame_）
    if (current_mode_ == MarkerState::CONTINUOUS)
    {
        geometry_msgs::msg::Pose transformed_pose = transformPose(*current_pose, marker_fixed_frame_, control_base_frame_);
        if (armType == "left" && left_pose_publisher_)
        {
            left_pose_publisher_->publish(transformed_pose);
        }
        else if (armType == "right" && dual_arm_mode_ && right_pose_publisher_)
        {
            right_pose_publisher_->publish(transformed_pose);
        }
    }
}
```

- **依赖成员**：`left_pose_`/`right_pose_`, `server_`, `marker_fixed_frame_`, `control_base_frame_`, `current_mode_`, `left_pose_publisher_`/`right_pose_publisher_`, `shouldUpdateMarker()`, `transformPose()`
- **理由**：业务级 pose 设置与发布，涉及具体 topic 名、坐标转换、模式判断（连续/单次）、发布时机

### 4. `updateMarkerPoseIncremental(const std::string& armType, const std::array<double, 3>& positionDelta, const std::array<double, 3>& rpyDelta)`

**完整代码：**
```268:365:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::updateMarkerPoseIncremental(
    const std::string& armType,
    const std::array<double, 3>& positionDelta,
    const std::array<double, 3>& rpyDelta)
{
    // 检查是否在禁用状态，只有在禁用状态下才允许增量更新
    if (!isStateDisabled(current_controller_state_))
    {
        RCLCPP_DEBUG(node_->get_logger(), "🎮 Incremental update blocked - controller state %d is not disabled",
                     current_controller_state_);
        return;
    }

    geometry_msgs::msg::Pose* current_pose = nullptr;
    std::string marker_name;

    if (armType == "left")
    {
        current_pose = &left_pose_;
        marker_name = "left_arm_target";
    }
    else if (armType == "right" && dual_arm_mode_)
    {
        current_pose = &right_pose_;
        marker_name = "right_arm_target";
    }
    else
    {
        return; // 无效的手臂类型
    }

    // 更新位置
    current_pose->position.x += positionDelta[0];
    current_pose->position.y += positionDelta[1];
    current_pose->position.z += positionDelta[2];

    // 更新旋转（使用RPY增量）
    if (std::abs(rpyDelta[0]) > 0.001 || std::abs(rpyDelta[1]) > 0.001 || std::abs(rpyDelta[2]) > 0.001)
    {
        // 将当前四元数转换为Eigen
        Eigen::Quaterniond current_quat(
            current_pose->orientation.w,
            current_pose->orientation.x,
            current_pose->orientation.y,
            current_pose->orientation.z
        );

        // 创建旋转增量
        Eigen::AngleAxisd rollAngle(rpyDelta[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(rpyDelta[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rpyDelta[2], Eigen::Vector3d::UnitZ());

        // 组合旋转（ZYX顺序）
        Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;

        // 方案 B：左乘 - 旋转相对于全局坐标系（世界坐标系）
        current_quat = rotationIncrement * current_quat;
        current_quat.normalize();

        // 转换回geometry_msgs
        current_pose->orientation.w = current_quat.w();
        current_pose->orientation.x = current_quat.x();
        current_pose->orientation.y = current_quat.y();
        current_pose->orientation.z = current_quat.z();
    }

    // 更新marker
    if (server_)
    {
        server_->setPose(marker_name, *current_pose);
        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }

    // 在连续发布模式下，发送target pose（需要转换到control_base_frame_）
    if (current_mode_ == MarkerState::CONTINUOUS)
    {
        geometry_msgs::msg::Pose transformed_pose = transformPose(*current_pose, marker_fixed_frame_, control_base_frame_);
        if (armType == "left" && left_pose_publisher_)
        {
            left_pose_publisher_->publish(transformed_pose);
        }
        else if (armType == "right" && dual_arm_mode_ && right_pose_publisher_)
        {
            right_pose_publisher_->publish(transformed_pose);
        }
    }
}
```

- **依赖成员**：`left_pose_`/`right_pose_`, `current_controller_state_`, `isStateDisabled()`, `server_`, `marker_fixed_frame_`, `control_base_frame_`, `current_mode_`, `left_pose_publisher_`/`right_pose_publisher_`, `shouldUpdateMarker()`, `transformPose()`
- **理由**：增量更新逻辑，涉及状态机检查、局部/全局旋转策略、节流和 topic 发布，均为业务层决策

### 5. `getMarkerPose(const std::string& armType) const`

**完整代码：**
```368:388:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
geometry_msgs::msg::Pose ArmsTargetManager::getMarkerPose(const std::string& armType) const
{
    if (armType == "left")
    {
        return left_pose_;
    }
    if (armType == "right" && dual_arm_mode_)
    {
        return right_pose_;
    }

    geometry_msgs::msg::Pose zero_pose;
    zero_pose.position.x = 0.0;
    zero_pose.position.y = 0.0;
    zero_pose.position.z = 0.0;
    zero_pose.orientation.w = 1.0;
    zero_pose.orientation.x = 0.0;
    zero_pose.orientation.y = 0.0;
    zero_pose.orientation.z = 0.0;
    return zero_pose;
}
```

- **依赖成员**：`left_pose_`, `right_pose_`, `dual_arm_mode_`
- **理由**：业务状态查询接口，返回业务层缓存的 pose，供外部查询

### 6. `leftMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)`

**完整代码：**
```513:529:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::leftMarkerCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
    std::string source_frame_id = feedback->header.frame_id;

    // 转换pose到目标frame（配置的marker_fixed_frame_）
    geometry_msgs::msg::Pose transformed_pose = transformPose(feedback->pose, source_frame_id, marker_fixed_frame_);

    left_pose_ = transformed_pose;

    if (current_mode_ == MarkerState::CONTINUOUS)
    {
        // 发布时需要转换到control_base_frame_
        geometry_msgs::msg::Pose transformed_pose = transformPose(left_pose_, marker_fixed_frame_, control_base_frame_);
        left_pose_publisher_->publish(transformed_pose);
    }
}
```

- **依赖成员**：`left_pose_`, `marker_fixed_frame_`, `control_base_frame_`, `current_mode_`, `left_pose_publisher_`, `transformPose()`
- **理由**：Marker 交互回调逻辑，处理坐标转换、模式判断、发布目标 pose 等业务需求

### 7. `rightMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)`

**完整代码：**
```531:547:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::rightMarkerCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
    std::string source_frame_id = feedback->header.frame_id;

    // 转换pose到目标frame（配置的marker_fixed_frame_）
    geometry_msgs::msg::Pose transformed_pose = transformPose(feedback->pose, source_frame_id, marker_fixed_frame_);

    right_pose_ = transformed_pose;

    if (current_mode_ == MarkerState::CONTINUOUS)
    {
        // 发布时需要转换到control_base_frame_
        geometry_msgs::msg::Pose transformed_pose = transformPose(right_pose_, marker_fixed_frame_, control_base_frame_);
        right_pose_publisher_->publish(transformed_pose);
    }
}
```

- **依赖成员**：`right_pose_`, `marker_fixed_frame_`, `control_base_frame_`, `current_mode_`, `right_pose_publisher_`, `transformPose()`
- **理由**：同 `leftMarkerCallback`，处理右臂业务逻辑

### 8. `headMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)`

**完整代码：**
```1138:1153:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::headMarkerCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
    std::string source_frame_id = feedback->header.frame_id;

    // 转换pose到目标frame（配置的head_marker_frame_）
    geometry_msgs::msg::Pose transformed_pose = transformPose(feedback->pose, source_frame_id, head_marker_frame_);

    head_pose_ = transformed_pose;

    // 在连续发布模式下，发送头部目标关节位置
    if (current_mode_ == MarkerState::CONTINUOUS)
    {
        sendHeadTargetJointPosition();
    }
}
```

- **依赖成员**：`head_pose_`, `head_marker_frame_`, `current_mode_`, `sendHeadTargetJointPosition()`, `transformPose()`
- **理由**：头部 marker 交互回调，处理坐标转换、模式判断、发布关节角度等业务需求

### 9. `togglePublishMode()`

**完整代码：**
```550:565:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::togglePublishMode()
{
    if (current_mode_ == MarkerState::SINGLE_SHOT)
    {
        current_mode_ = MarkerState::CONTINUOUS;
    }
    else
    {
        current_mode_ = MarkerState::SINGLE_SHOT;
    }

    updateMarkerShape();
    updateMenuVisibility();

    server_->applyChanges();
}
```

- **依赖成员**：`current_mode_`, `updateMarkerShape()`, `updateMenuVisibility()`, `server_`
- **理由**：业务状态切换（单次/连续发布模式），触发 marker 重建和菜单更新

### 10. `getCurrentMode() const`

**完整代码：**
```567:570:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
MarkerState ArmsTargetManager::getCurrentMode() const
{
    return current_mode_;
}
```

- **依赖成员**：`current_mode_`
- **理由**：业务状态查询接口，返回当前发布模式，供其他组件（如 `VRInputHandler`）查询

### 11. `sendTargetPose()`

**完整代码：**
```572:583:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::sendTargetPose()
{
    // 将pose从marker_fixed_frame_转换到control_base_frame_后发布
    geometry_msgs::msg::Pose transformed_left_pose = transformPose(left_pose_, marker_fixed_frame_, control_base_frame_);
    left_pose_publisher_->publish(transformed_left_pose);

    if (dual_arm_mode_)
    {
        geometry_msgs::msg::Pose transformed_right_pose = transformPose(right_pose_, marker_fixed_frame_, control_base_frame_);
        right_pose_publisher_->publish(transformed_right_pose);
    }
}
```

- **依赖成员**：`left_pose_`, `right_pose_`, `dual_arm_mode_`, `marker_fixed_frame_`, `control_base_frame_`, `left_pose_publisher_`, `right_pose_publisher_`, `transformPose()`
- **理由**：目标发布逻辑，读取业务层缓存的 pose、做坐标转换、发布到具体话题，涉及话题名、发布器、业务层触发时机

### 12. `sendHeadTargetJointPosition()`

**完整代码：**
```1177:1197:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::sendHeadTargetJointPosition()
{
    if (!head_joint_publisher_)
    {
        RCLCPP_WARN(node_->get_logger(), "Head joint publisher not initialized");
        return;
    }

    // 从头部pose的orientation提取关节角度
    std::vector<double> joint_angles = quaternionToHeadJointAngles(head_pose_.orientation);

    // 创建并发布消息
    std_msgs::msg::Float64MultiArray msg;
    msg.data = joint_angles;

    head_joint_publisher_->publish(msg);

    RCLCPP_INFO(node_->get_logger(), 
               "Published head target joint angles: [%.3f, %.3f] (head_joint1, head_joint2)",
               joint_angles[0], joint_angles[1]);
}
```

- **依赖成员**：`head_pose_`, `head_joint_publisher_`, `quaternionToHeadJointAngles()`
- **理由**：头部目标发布逻辑，从 pose 提取关节角度并发布到具体话题，涉及业务层特定转换逻辑

### 13. `setupMenu()` / `setupHeadMenu()`

**完整代码：**
```586:630:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::setupMenu()
{
    left_menu_handler_ = std::make_shared<interactive_markers::MenuHandler>();

    auto leftSendCallback = [this](
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
    {
        sendTargetPose();
    };

    auto leftToggleCallback = [this](
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
    {
        togglePublishMode();
    };

    left_send_handle_ = left_menu_handler_->insert("发送目标", leftSendCallback);

    std::string leftToggleText = (current_mode_ == MarkerState::CONTINUOUS) ? "切换到单次发布" : "切换到连续发布";
    left_toggle_handle_ = left_menu_handler_->insert(leftToggleText, leftToggleCallback);

    if (dual_arm_mode_)
    {
        right_menu_handler_ = std::make_shared<interactive_markers::MenuHandler>();

        auto rightSendCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
        {
            sendTargetPose();
        };

        auto rightToggleCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
        {
            togglePublishMode();
        };

        right_send_handle_ = right_menu_handler_->insert("发送目标", rightSendCallback);

        std::string rightToggleText = (current_mode_ == MarkerState::CONTINUOUS) ? "切换到单次发布" : "切换到连续发布";
        right_toggle_handle_ = right_menu_handler_->insert(rightToggleText, rightToggleCallback);
    }
}
```

**setupHeadMenu 完整代码：**
```1155:1175:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::setupHeadMenu()
{
    head_menu_handler_ = std::make_shared<interactive_markers::MenuHandler>();

    auto headSendCallback = [this](
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
    {
        sendHeadTargetJointPosition();
    };

    auto headToggleCallback = [this](
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
    {
        togglePublishMode();
    };

    head_send_handle_ = head_menu_handler_->insert("发送目标", headSendCallback);

    std::string headToggleText = (current_mode_ == MarkerState::CONTINUOUS) ? "切换到单次发布" : "切换到连续发布";
    head_toggle_handle_ = head_menu_handler_->insert(headToggleText, headToggleCallback);
}
```

- **依赖成员**：`left_menu_handler_`/`right_menu_handler_`/`head_menu_handler_`, `current_mode_`, `sendTargetPose()`, `sendHeadTargetJointPosition()`, `togglePublishMode()`
- **理由**：菜单管理，虽然菜单项创建是通用操作，但菜单内容、回调绑定、可见性判断都依赖业务层状态和函数，保留在上层更合适

### 14. `updateMarkerShape()`

**完整代码：**
```632:663:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::updateMarkerShape()
{
    auto leftMarker = createMarker("left_arm_target", "left");
    server_->insert(leftMarker);

    auto leftCallback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        leftMarkerCallback(feedback);
    };
    server_->setCallback(leftMarker.name, leftCallback);
    left_menu_handler_->apply(*server_, leftMarker.name);

    if (dual_arm_mode_)
    {
        auto rightMarker = createMarker("right_arm_target", "right");
        server_->insert(rightMarker);

        auto rightCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            rightMarkerCallback(feedback);
        };
        server_->setCallback(rightMarker.name, rightCallback);
        right_menu_handler_->apply(*server_, rightMarker.name);
    }

    // 如果启用头部控制，也更新头部 marker
    if (enable_head_control_)
    {
        updateHeadMarkerShape();
    }
}
```

- **依赖成员**：`createMarker()`, `server_`, `leftMarkerCallback()`, `rightMarkerCallback()`, `dual_arm_mode_`, `enable_head_control_`, `updateHeadMarkerShape()`, `left_menu_handler_`, `right_menu_handler_`
- **理由**：Marker 重建/调度，根据模式或状态决定何时重新创建/插入左右臂与头部 marker 并绑定回调、菜单，此调度策略属于上层业务

### 15. `updateHeadMarkerShape()`

**完整代码：**
```799:818:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::updateHeadMarkerShape()
{
    if (!enable_head_control_)
    {
        return;
    }

    // 更新菜单以确保切换文本正确
    setupHeadMenu();

    auto headMarker = createHeadMarker();
    server_->insert(headMarker);

    auto headCallback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        headMarkerCallback(feedback);
    };
    server_->setCallback(headMarker.name, headCallback);
    head_menu_handler_->apply(*server_, headMarker.name);
}
```

- **依赖成员**：`createHeadMarker()`, `server_`, `headMarkerCallback()`, `setupHeadMenu()`, `head_menu_handler_`
- **理由**：头部 marker 重建/调度，业务层逻辑

### 16. `updateMenuVisibility()`

**完整代码：**
```665:712:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::updateMenuVisibility()
{
    setupMenu();

    left_menu_handler_->apply(*server_, "left_arm_target");
    if (dual_arm_mode_)
    {
        right_menu_handler_->apply(*server_, "right_arm_target");
    }

    if (current_mode_ == MarkerState::CONTINUOUS)
    {
        left_menu_handler_->setVisible(left_send_handle_, false);
        if (dual_arm_mode_)
        {
            right_menu_handler_->setVisible(right_send_handle_, false);
        }
        // 更新头部菜单可见性
        if (enable_head_control_)
        {
            head_menu_handler_->setVisible(head_send_handle_, false);
        }
    }
    else
    {
        left_menu_handler_->setVisible(left_send_handle_, true);
        if (dual_arm_mode_)
        {
            right_menu_handler_->setVisible(right_send_handle_, true);
        }
        // 更新头部菜单可见性
        if (enable_head_control_)
        {
            head_menu_handler_->setVisible(head_send_handle_, true);
        }
    }

    left_menu_handler_->reApply(*server_);
    if (dual_arm_mode_)
    {
        right_menu_handler_->reApply(*server_);
    }
    // 更新头部菜单
    if (enable_head_control_)
    {
        head_menu_handler_->reApply(*server_);
    }
}
```

- **依赖成员**：`setupMenu()`, `setupHeadMenu()`, `current_mode_`, `dual_arm_mode_`, `enable_head_control_`, `left_menu_handler_`, `right_menu_handler_`, `head_menu_handler_`, `server_`
- **理由**：菜单的应用与可见性控制，业务层决定何时应用/重新应用菜单以及各项可见性，根据模式、是否双臂/头控来隐藏"发送目标"

### 17. `setAutoUpdateEnabled(bool enable)` / `isAutoUpdateEnabled() const`

**完整代码：**
```747:755:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::setAutoUpdateEnabled(bool enable)
{
    auto_update_enabled_ = enable;
}

bool ArmsTargetManager::isAutoUpdateEnabled() const
{
    return auto_update_enabled_;
}
```

- **依赖成员**：`auto_update_enabled_`
- **理由**：自动更新控制接口，控制业务层的自动更新开关，与状态机、话题订阅等业务逻辑相关

### 18. `controlInputCallback(const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg)`

**完整代码：**
```757:797:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::controlInputCallback(const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg)
{
    int32_t new_state = msg->command;

    if (new_state == 0)
    {
        return;
    }

    if (new_state != current_controller_state_)
    {
        current_controller_state_ = new_state;
        
        // 如果切换到 MOVE 状态（command = 3）且启用了头部控制
        // 将当前头部位置作为目标位置发布，确保头部保持当前位置
        if (new_state == 3 && enable_head_control_ && head_joint_publisher_)
        {
            // 如果有缓存的关节角度，使用缓存的；否则从 head_pose_ 提取
            if (last_head_joint_angles_.size() == 2)
            {
                std_msgs::msg::Float64MultiArray msg;
                msg.data = last_head_joint_angles_;
                head_joint_publisher_->publish(msg);
                RCLCPP_INFO(node_->get_logger(),
                           "Entered MOVE state, published current head position as target: [%.3f, %.3f]",
                           last_head_joint_angles_[0], last_head_joint_angles_[1]);
            }
            else
            {
                // 如果没有缓存，从 head_pose_ 提取（可能不是最新的，但总比没有好）
                sendHeadTargetJointPosition();
                RCLCPP_INFO(node_->get_logger(),
                           "Entered MOVE state, published head position from marker as target");
            }
        }
        
        // 状态变化时重新创建marker
        updateMarkerShape();
        server_->applyChanges();
    }
}
```

- **依赖成员**：`current_controller_state_`, `enable_head_control_`, `head_joint_publisher_`, `last_head_joint_angles_`, `sendHeadTargetJointPosition()`, `updateMarkerShape()`, `server_`
- **理由**：控制器状态机回调，处理控制器状态变化（HOME/HOLD/MOVE 等），更新内部状态、触发 marker 重建、在特定状态下发布头部目标等

### 19. `leftEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)`

**完整代码：**
```820:836:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::leftEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    if (auto_update_enabled_ && !isStateDisabled(current_controller_state_))
    {
        // 将接收到的pose转换到marker_fixed_frame_下，使用最新的可用变换
        std::string source_frame_id = msg->header.frame_id;
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            msg->pose, source_frame_id, marker_fixed_frame_);
        left_pose_ = transformed_pose;
        server_->setPose("left_arm_target", left_pose_);

        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }
}
```

- **依赖成员**：`auto_update_enabled_`, `isStateDisabled()`, `current_controller_state_`, `left_pose_`, `marker_fixed_frame_`, `server_`, `shouldUpdateMarker()`, `transformPose()`
- **理由**：订阅与状态同步，把机器人实时位姿同步到 marker 的流程依赖具体话题和状态机，属于上层业务范围

### 20. `rightEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)`

**完整代码：**
```838:854:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::rightEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    if (auto_update_enabled_ && !isStateDisabled(current_controller_state_))
    {
        // 将接收到的pose转换到marker_fixed_frame_下，使用最新的可用变换
        std::string source_frame_id = msg->header.frame_id;
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            msg->pose, source_frame_id, marker_fixed_frame_);
        right_pose_ = transformed_pose;
        server_->setPose("right_arm_target", right_pose_);

        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }
}
```

- **依赖成员**：同 `leftEndEffectorPoseCallback`
- **理由**：同 `leftEndEffectorPoseCallback`，处理右臂

### 21. `headJointStateCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)`

**完整代码：**
```963:1043:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::headJointStateCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    if (!enable_head_control_)
    {
        return;
    }

    // 检查是否启用自动更新且当前状态不在禁用列表中
    if (!auto_update_enabled_ || isStateDisabled(current_controller_state_))
    {
        return;
    }

    // 从 joint_states 中查找 head_joint1 和 head_joint2
    double head_joint1_angle = 0.0;
    double head_joint2_angle = 0.0;
    bool found_joint1 = false;
    bool found_joint2 = false;

    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
        if (msg->name[i] == "head_joint1")
        {
            head_joint1_angle = msg->position[i];
            found_joint1 = true;
        }
        else if (msg->name[i] == "head_joint2")
        {
            head_joint2_angle = msg->position[i];
            found_joint2 = true;
        }

        if (found_joint1 && found_joint2)
        {
            break;
        }
    }

    // 如果找到了两个关节，更新头部 marker 的 orientation 和 position
    if (found_joint1 && found_joint2)
    {
        // 将关节角度转换为四元数（确保顺序：head_joint1, head_joint2）
        std::vector<double> head_joint_angles = {head_joint1_angle, head_joint2_angle};
        
        // 缓存最新的关节角度，用于状态切换时发布
        last_head_joint_angles_ = head_joint_angles;
        
        geometry_msgs::msg::Quaternion quat = headJointAnglesToQuaternion(head_joint_angles);

        // 更新头部 pose 的 orientation
        head_pose_.orientation = quat;

        // 从 TF 获取 head_link2 的实际位置并更新
        try
        {
            // 获取 head_link2 在 head_marker_frame_ 中的位置
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                head_marker_frame_, HEAD_LINK_NAME, tf2::TimePointZero);
            
            // 更新 marker 位置为 head_link2 的实际位置
            head_pose_.position.x = transform.transform.translation.x;
            head_pose_.position.y = transform.transform.translation.y;
            head_pose_.position.z = transform.transform.translation.z;
        }
        catch (const tf2::TransformException& ex)
        {
            // 如果 TF 转换失败，保持当前位置不变（使用固定位置或上次的位置）
            RCLCPP_DEBUG(node_->get_logger(),
                        "无法从 TF 获取头部 link %s 的位置: %s，保持当前位置",
                        HEAD_LINK_NAME, ex.what());
        }

        // 更新 marker
        server_->setPose("head_target", head_pose_);

        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }
}
```

- **依赖成员**：`enable_head_control_`, `auto_update_enabled_`, `isStateDisabled()`, `current_controller_state_`, `head_pose_`, `head_marker_frame_`, `tf_buffer_`, `last_head_joint_angles_`, `headJointAnglesToQuaternion()`, `server_`, `shouldUpdateMarker()`
- **理由**：订阅与状态同步，把机器人实时关节角同步到头部 marker 的流程依赖具体话题和状态机，属于上层业务范围

### 22. `isStateDisabled(int32_t state) const`

**完整代码：**
```857:861:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
bool ArmsTargetManager::isStateDisabled(int32_t state) const
{
    return std::find(disable_auto_update_states_.begin(), disable_auto_update_states_.end(), state) !=
        disable_auto_update_states_.end();
}
```

- **依赖成员**：`disable_auto_update_states_`
- **理由**：状态禁用判断，检查给定控制器状态是否在禁用列表中，用于决定是否禁用自动更新和 marker 交互性，属于业务层状态判断逻辑

### 23. `shouldUpdateMarker()`

**完整代码：**
```863:874:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
bool ArmsTargetManager::shouldUpdateMarker()
{
    auto now = node_->now();
    auto time_since_last_update = (now - last_marker_update_time_).seconds();

    if (time_since_last_update >= marker_update_interval_)
    {
        last_marker_update_time_ = now;
        return true;
    }
    return false;
}
```

- **依赖成员**：`node_`, `last_marker_update_time_`, `marker_update_interval_`
- **理由**：更新节流判断，实现更新节流机制，防止 marker 更新过于频繁，虽然类似通用节流逻辑，但继续保留在上层

### 24. `quaternionToHeadJointAngles(const geometry_msgs::msg::Quaternion& quaternion) const`

**完整代码：**
```913:928:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
std::vector<double> ArmsTargetManager::quaternionToHeadJointAngles(
    const geometry_msgs::msg::Quaternion& quaternion) const
{
    // 使用 tf2 的 getRPY 从 quaternion 提取欧拉角
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quaternion, tf_quat);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    
    // yaw (Z轴旋转) -> head_joint1
    // pitch (Y轴旋转) -> head_joint2
    // 注意：pitch 取反，使得向上转动 marker 时头部向上看
    // 忽略 roll (X轴旋转)
    return {yaw, -pitch};
}
```

- **依赖成员**：无（纯函数）
- **理由**：头部特定的转换逻辑，包含业务层特定的关节映射（`head_joint1` → yaw、`head_joint2` → pitch 且需取反），属于业务层的特定逻辑

### 25. `headJointAnglesToQuaternion(const std::vector<double>& joint_angles) const`

**完整代码：**
```930:961:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
geometry_msgs::msg::Quaternion ArmsTargetManager::headJointAnglesToQuaternion(
    const std::vector<double>& joint_angles) const
{
    if (joint_angles.size() < 2)
    {
        RCLCPP_WARN(node_->get_logger(), "Invalid joint angles size, expected 2, got %zu", joint_angles.size());
        geometry_msgs::msg::Quaternion quat;
        quat.w = 1.0;
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = 0.0;
        return quat;
    }

    // head_joint1 -> yaw (Z轴旋转)
    // head_joint2 -> pitch (Y轴旋转，需要取反)
    double yaw = joint_angles[0];
    double pitch = -joint_angles[1];  // 取反，与 quaternionToHeadJointAngles 对应
    double roll = 0.0;  // 忽略 roll

    // 使用 tf2 从 RPY 创建四元数
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    tf_quat.normalize();

    geometry_msgs::msg::Quaternion quat;
    quat.w = tf_quat.w();
    quat.x = tf_quat.x();
    quat.y = tf_quat.y();
    quat.z = tf_quat.z();
    return quat;
}
```

- **依赖成员**：无（纯函数）
- **理由**：同 `quaternionToHeadJointAngles`，头部特定的业务逻辑

## 计划抽象到 `InteractiveMarkerManager` 的函数列表

---

## ✅ 已完成：统一 `createMarker` 函数（在原文件中直接修改）

**完成日期**：已编译测试通过

**实现方式**：在 `ArmsTargetManager.cpp` 中直接修改 `createMarker()` 函数，使其支持 `armType == "head"` 的情况

### 主要修改内容：

1. **扩展 `createMarker()` 函数**：
   - 添加对 `armType == "head"` 的判断和处理
   - 当 `armType == "head"` 时，使用头部逻辑（箭头显示、yaw/pitch 控制）
   - 当 `armType == "left"` 或 `"right"` 时，使用原有的双臂逻辑

2. **统一坐标系**：
   - 头部和双臂都使用 `marker_fixed_frame_`（默认 `base_footprint`）
   - 删除了 `head_marker_frame_` 成员变量和构造函数参数
   - 所有 TF 查询和坐标转换都使用统一的 `marker_fixed_frame_`

3. **更新调用点**：
   - `initialize()` 中使用 `createMarker("head_target", "head")` 替代 `createHeadMarker()`
   - `updateHeadMarkerShape()` 中使用统一的 `createMarker()` 函数
   - 所有相关 TF 查询从 `head_marker_frame_` 改为 `marker_fixed_frame_`

4. **清理代码**：
   - 删除了 `createHeadMarker()` 函数（头文件声明和实现）
   - 删除了 `headMarkerFrame` 构造函数参数
   - 删除了 `head_marker_frame_` 成员变量
   - 更新了 `arms_target_manager_node.cpp` 中的相关调用

### 修改的文件：
- `ArmsTargetManager.h` - 删除 `createHeadMarker()` 声明和 `head_marker_frame_` 成员变量
- `ArmsTargetManager.cpp` - 扩展 `createMarker()` 函数，删除 `createHeadMarker()` 实现
- `arms_target_manager_node.cpp` - 删除 `head_marker_frame` 参数读取和传递

### 结果：
- ✅ 头部和双臂统一使用 `createMarker()` 函数创建
- ✅ 所有 marker 使用统一的坐标系（`base_footprint`）
- ✅ 代码更简洁，减少了冗余参数和函数
- ✅ 编译测试通过，功能正常

---

### 3. `createBoxMarker(const std::string& color) const`

**函数作用**：
- 创建一个基础的 **可视化 Marker**（`visualization_msgs::msg::Marker`）
- 返回一个立方体（CUBE）形状的 marker，用于在 RViz 中显示
- **注意**：这是简单的几何图形，没有交互功能

**完整代码：**
```444:473:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
visualization_msgs::msg::Marker ArmsTargetManager::createBoxMarker(const std::string& color) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    if (color == "blue")
    {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    else if (color == "red")
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
    }
    marker.color.a = 0.7;

    return marker;
}
```

- **当前依赖成员**：无（纯函数）
- **抽象后的接口设计**：
  - 输入参数：`color` (string), `scale` (double, 可选)
  - 返回：`visualization_msgs::msg::Marker`
- **理由**：基础可视化创建，仅处理 `visualization_msgs::msg::Marker` 颜色/尺寸的函数，可放入通用类供不同业务重用

### 4. `createSphereMarker(const std::string& color) const`

**函数作用**：
- 创建一个基础的 **可视化 Marker**（`visualization_msgs::msg::Marker`）
- 返回一个球体（SPHERE）形状的 marker，用于在 RViz 中显示
- **注意**：这是简单的几何图形，没有交互功能

**完整代码：**
```715:744:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
visualization_msgs::msg::Marker ArmsTargetManager::createSphereMarker(const std::string& color) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    if (color == "blue")
    {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    else if (color == "red")
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
    }
    marker.color.a = 0.7;

    return marker;
}
```

- **当前依赖成员**：无（纯函数）
- **抽象后的接口设计**：
  - 输入参数：`color` (string), `scale` (double, 可选)
  - 返回：`visualization_msgs::msg::Marker`
- **理由**：同 `createBoxMarker`，基础可视化创建

### 5. `createArrowMarker(const std::string& color) const`

**函数作用**：
- 创建一个基础的 **可视化 Marker**（`visualization_msgs::msg::Marker`）
- 返回一个箭头（ARROW）形状的 marker，用于在 RViz 中显示方向
- **注意**：这是简单的几何图形，没有交互功能

**完整代码：**
```1045:1080:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
visualization_msgs::msg::Marker ArmsTargetManager::createArrowMarker(const std::string& color) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x = 0.15;  // 箭头长度
    marker.scale.y = 0.03;  // 箭头宽度
    marker.scale.z = 0.03;  // 箭头高度

    if (color == "green")
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else if (color == "blue")
    {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    else if (color == "red")
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
    }
    marker.color.a = 0.7;

    return marker;
}
```

- **当前依赖成员**：无（纯函数）
- **抽象后的接口设计**：
  - 输入参数：`color` (string), `scale` (double, 可选)
  - 返回：`visualization_msgs::msg::Marker`
- **理由**：同 `createBoxMarker`，基础可视化创建

---

## 📌 重要区别：`createMarker()` vs `createBoxMarker/SphereMarker/ArrowMarker()`

### 层次关系

```
createMarker()
    ├── 创建 InteractiveMarker（完整的交互式 marker）
    ├── 添加交互控制（拖拽、旋转等）
    ├── 设置坐标系、位姿等
    └── 调用下面的函数创建可视化部分：
        ├── createBoxMarker()      ← 创建立方体显示
        ├── createSphereMarker()   ← 创建球体显示
        └── createArrowMarker()    ← 创建箭头显示
```

### 详细对比

| 特性 | `createMarker()` | `createBoxMarker/SphereMarker/ArrowMarker()` |
|------|------------------|-----------------------------------------------|
| **返回类型** | `InteractiveMarker` | `Marker` |
| **功能级别** | **高级** - 完整的交互式 marker | **低级** - 仅可视化几何图形 |
| **交互能力** | ✅ 支持拖拽、旋转等交互 | ❌ 不支持交互，仅显示 |
| **包含内容** | 包含 marker + 控制 + 交互逻辑 | 仅包含几何形状和颜色 |
| **使用场景** | 用于创建可以在 RViz 中操作的 marker | 用于创建 marker 的可视化部分 |
| **调用关系** | `createMarker()` **内部调用** `createBoxMarker()` 等 | 被 `createMarker()` **调用** |

### 代码示例

```cpp
// createMarker() 的调用（在 createMarker 函数内部）：
if (current_mode == MarkerState::CONTINUOUS)
{
    marker = createSphereMarker(armType == "left" ? "blue" : "red");  // 创建球体
}
else
{
    marker = createBoxMarker(armType == "left" ? "blue" : "red");      // 创建立方体
}

// 然后把这个 marker 添加到 InteractiveMarker 中：
visualization_msgs::msg::InteractiveMarkerControl boxControl;
boxControl.markers.push_back(marker);  // 将可视化 marker 添加到交互控制中
interactiveMarker.controls.push_back(boxControl);  // 添加到交互式 marker
```

### 类比说明

- **`createMarker()`** = 完整的"可交互的 3D 控件"（像 Windows 中的按钮，可以点击、拖拽）
- **`createBoxMarker()` 等** = 按钮的"外观/图标"（只是视觉上的图案，没有交互能力）

`createMarker()` 就像创建一个完整的交互式控件，而 `createBoxMarker()` 等函数只是创建这个控件的"皮肤"（外观）。

---

### 6. `addMovementControls(visualization_msgs::msg::InteractiveMarker& interactiveMarker) const`

**完整代码：**
```475:511:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
void ArmsTargetManager::addMovementControls(
    visualization_msgs::msg::InteractiveMarker& interactiveMarker) const
{
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);
}
```

- **当前依赖成员**：无（纯函数，修改输入参数）
- **抽象后的接口设计**：
  - 输入参数：`visualization_msgs::msg::InteractiveMarker&` (引用，会被修改)
  - 返回：无（void）
- **理由**：通用控制配置，为 marker 添加 6DOF 移动和旋转控制，可在 marker 类统一实现，上层只控制是否启用

### 7. `transformPose(const geometry_msgs::msg::Pose& pose, const std::string& sourceFrameId, const std::string& targetFrameId) const`

**完整代码：**
```876:911:ros2_ws/src/arms_ros2_control/command/arms_target_manager/src/ArmsTargetManager.cpp
geometry_msgs::msg::Pose ArmsTargetManager::transformPose(
    const geometry_msgs::msg::Pose& pose,
    const std::string& sourceFrameId,
    const std::string& targetFrameId) const
{
    // 如果源frame和目标frame相同，不需要转换
    if (sourceFrameId == targetFrameId)
    {
        return pose;
    }

    try
    {
        // 创建PoseStamped用于转换
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = sourceFrameId;
        pose_stamped.header.stamp = rclcpp::Time(0);  // 使用Time(0)表示使用最新变换
        pose_stamped.pose = pose;

        // 获取最新的变换并使用doTransform进行转换
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            targetFrameId, sourceFrameId, tf2::TimePointZero);
        
        // 使用doTransform进行转换
        geometry_msgs::msg::PoseStamped result_stamped;
        tf2::doTransform(pose_stamped, result_stamped, transform);
        return result_stamped.pose;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "无法将pose从 %s 转换到 %s: %s，使用原始pose",
                    sourceFrameId.c_str(), targetFrameId.c_str(), ex.what());
        return pose;
    }
}
```

- **当前依赖成员**：`tf_buffer_`, `node_`（用于日志）
- **抽象后的接口设计**：
  - 输入参数：`pose`, `sourceFrameId`, `targetFrameId`
  - 依赖：需要 `tf2_ros::Buffer` 或通过成员变量提供
  - 返回：`geometry_msgs::msg::Pose`
- **理由**：通用坐标转换，使用 TF2 在不同坐标系间转换 pose，这是通用的坐标转换工具；注意：`InteractiveMarkerManager` 中可能已经有类似的 `transformPose()` 函数，需要统一接口

## 保留的成员变量（业务层状态）

- `node_`: ROS2 节点指针
- `topic_prefix_`: Topic 前缀
- `dual_arm_mode_`: 是否双臂模式
- `control_base_frame_`: 控制坐标系
- `marker_fixed_frame_`: Marker 固定坐标系
- `publish_rate_`: 发布频率
- `current_mode_`: 当前发布模式（单次/连续）
- `current_controller_state_`: 当前控制器状态（HOME/HOLD/MOVE）
- `auto_update_enabled_`: 是否启用自动更新
- `disable_auto_update_states_`: 禁用自动更新的状态列表
- `last_marker_update_time_`: 上次 marker 更新时间（用于节流）
- `marker_update_interval_`: Marker 更新间隔
- `enable_head_control_`: 是否启用头部控制
- `head_marker_frame_`: 头部 marker 坐标系
- `head_controller_name_`: 头部控制器名称
- `head_marker_position_`: 头部 marker 初始位置
- `left_pose_`/`right_pose_`/`head_pose_`: 缓存的 pose（业务层状态）
- `server_`: InteractiveMarkerServer 指针
- `tf_buffer_`/`tf_listener_`: TF2 组件
- `left_pose_publisher_`/`right_pose_publisher_`: 左右臂目标位姿发布器
- `head_joint_publisher_`: 头部关节目标发布器
- `left_menu_handler_`/`right_menu_handler_`/`head_menu_handler_`: 菜单处理器
- `last_head_joint_angles_`: 缓存的头部关节角度

## 抽象后的交互方式

抽象后，`ArmsTargetManager` 使用 `InteractiveMarkerManager` 的方式：

1. **创建 marker**：通过 `createMarker(MarkerConfig)` 传入配置，获取 `InteractiveMarker`
2. **注册到 server**：上层控制 `server_->insert(marker)`
3. **设置回调**：上层控制 `server_->setCallback(marker_name, callback)`
4. **应用菜单**：上层控制 `menu_handler_->apply(*server_, marker_name)`
5. **更新 pose**：上层调用 `interactive_marker_manager->updateMarkerPose(marker_name, pose)`
6. **坐标转换**：使用 `interactive_marker_manager->transformPose(pose, source_frame, target_frame)`

这样的设计保持了职责分离：
- `InteractiveMarkerManager`：提供通用的 marker 操作工具
- `ArmsTargetManager`：负责业务逻辑、状态管理、话题发布、订阅回调等
