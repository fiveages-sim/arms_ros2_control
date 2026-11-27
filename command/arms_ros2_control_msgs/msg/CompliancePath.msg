# COMPLIANCE 连续路径：一次下发，控制器本地规划（速度曲线 + 时间参数化）并按时间回放，
# 同时把解析得到的目标速度作为位控轴前馈。下发后控制器不再依赖消息流，
# 通信抖动/丢帧不会进入跟踪误差。
#
# 两种模式（由 blend_radius 决定）：
#   折线（blend_radius = 0，推荐）：每段各自一条梯形曲线，段末速度归零 ——
#     角点停到 0 再转向。速度方向不突变，所以不产生法向加速度，也不存在
#     "切角"误差。代价是每个中间角点多花 v/a 秒。
#     注意：每个中间角点都会真的停住，所以只适合少量真实脚点，
#     不要把一段直线/圆弧离散成很多点发过来。
#   圆角（blend_radius > 0）：整条路径一条梯形曲线 + 角点圆弧过渡，角点不停。
#
# 收到本消息时：对应臂取消正在回放的路径，从当前 TCP 直线入段到路径起点，
# 然后按速度曲线走 loops 趟（开链路径按"往返"执行：第 2 趟反向走回来，
# 奇数趟停在末端、偶数趟回到起点），最后保持终点目标。
# 期间收到 left_target / left_target/stamped 等单点目标 → 立即抢占并取消路径。
#
# 会被拒绝的情况（控制器日志给出具体原因）：
#   - speed/accel 与 duration 同时缺省
#   - duration 反解出的速度超过控制器平移速度上限（前馈是开环量，不能事后再夹）
#   - blend_radius > 0 且 closed = false 且 loops > 1（一条梯形无法在换向处掉头）

std_msgs/Header header

# 角点：每点 7 个 double —— x, y, z [m], qx, qy, qz, qw（坐标系见 header.frame_id）
float64[] points

float64 blend_radius   # 圆角半径 [m]。
                       #   0 = 每段独立加减速：段末速度归零，角点停到 0 再转向。
                       #       速度方向不突变，法向加速度恒为 0 —— 走折线用这个。
                       #   >0 = 整条路径一条梯形曲线 + 角点圆弧过渡（角点不停）。
float64 speed          # 轨迹速度上限 [m/s]；duration>0 且此处 <=0 时用控制器默认
float64 accel          # 切向加减速 [m/s^2]；duration>0 且此处 <=0 时用控制器默认
float64 lat_accel      # 圆角处向心加速度上限 [m/s^2]；0 = 不限；仅 blend_radius>0 有效
                       # 实际速度 = min(speed, sqrt(lat_accel · min_radius))
float64 duration       # 总运行时间 [s]（含全部 loops，不含入段）；0 = 不用，按 speed/accel 规划
                       #   >0 时忽略 speed/accel 的绝对值，按 v→v/k、a→a/k^2 等比缩放
                       #   命中该时长（梯形形状不变，所以时长严格 k 倍）
                       #   反解后仍受控制器平移速度上限约束：超了就拒绝，不会偷偷夹
bool closed            # true = 闭合（4 个角点走成回路）；false = 只走一遍
uint32 loops           # 循环趟数；0 或 1 = 一趟
                       #   开链（closed=false）：往返，2 趟 = 去+回
                       #   闭合（closed=true）：绕圈，相邻两圈首尾相接
