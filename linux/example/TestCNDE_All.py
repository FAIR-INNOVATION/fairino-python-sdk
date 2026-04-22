"""
CNDE (Custom Network Data Export) 综合测试脚本
================================================
本脚本包含所有CNDE测试项，每个测试项可以独立运行

使用方法:
1. 独立运行单个测试: 取消该测试if块下的注释
   例如: 运行Test1 -> 取消 # if __name__ == "__main__": test1_xxx() 的注释
2. 运行全部测试: 取消底部 run_all_tests() 的注释

作者: 法奥意威
日期: 2026-04-11
"""

from fairino import Robot
from fairino.Robot import RobotState, SetRobotRealtimeStateConfig, DEFAULT_CNDE_STATES, AddRobotRealtimeState, DeleteRobotRealtimeState, SetRobotRealtimeStatePeriod
import time

# ==================== 全局配置参数 ====================
ROBOT_IP = '192.168.58.2'       # 机器人IP地址
TEST_DURATION = 10               # 默认测试持续时间（秒）


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def print_pkg_data(pkg, prefix="  "):
    """打印RobotStatePkg全部数据 - 使用反射遍历所有字段"""
    from fairino.Robot import RobotStatePkg
    import ctypes

    def print_struct_array(arr, name, pre):
        """展开打印结构体数组"""
        print(f"{pre}{name}: (长度: {len(arr)})")
        for i, item in enumerate(arr):
            if hasattr(item, '_fields_'):
                fields_str = []
                for fname, ftype in item._fields_:
                    fv = getattr(item, fname)
                    if isinstance(fv, float):
                        fields_str.append(f"{fname}={fv:.2f}")
                    else:
                        fields_str.append(f"{fname}={fv}")
                print(f"{pre}  [{i}] {', '.join(fields_str)}")
            else:
                print(f"{pre}  [{i}] {item}")

    print(f"{prefix}===== RobotStatePkg 完整数据 =====")

    if hasattr(pkg, '_fields_'):
        fields = pkg._fields_
    elif hasattr(RobotStatePkg, '_fields_'):
        fields = RobotStatePkg._fields_
    else:
        print(f"{prefix}错误: 无法获取字段定义")
        return

    for field_name, field_type in fields:
        try:
            value = getattr(pkg, field_name)
            if isinstance(value, ctypes.Array):
                if len(value) == 0:
                    print(f"{prefix}{field_name}: [] (空数组)")
                    continue
                # 检查数组元素类型
                first_elem = value[0]
                if isinstance(first_elem, (int, float)):
                    # 数值数组，格式化输出
                    if isinstance(first_elem, int):
                        array_str = ', '.join([str(x) for x in value])
                    else:
                        array_str = ', '.join([f'{x:.3f}' for x in value])
                    print(f"{prefix}{field_name}: [{array_str}]")
                elif hasattr(first_elem, '_fields_'):
                    # 结构体数组，展开显示
                    print_struct_array(value, field_name, prefix)
                else:
                    # 其他类型数组
                    print(f"{prefix}{field_name}: [{', '.join([str(x) for x in value])}]")
            elif hasattr(value, 'value'):
                actual_value = value.value
                if isinstance(actual_value, float):
                    print(f"{prefix}{field_name}: {actual_value:.6f}")
                else:
                    print(f"{prefix}{field_name}: {actual_value}")
            elif isinstance(value, (int, float)):
                if isinstance(value, float):
                    print(f"{prefix}{field_name}: {value:.6f}")
                else:
                    print(f"{prefix}{field_name}: {value}")
            else:
                print(f"{prefix}{field_name}: {value}")
        except Exception as e:
            print(f"{prefix}{field_name}: [访问错误: {e}]")

    print(f"{prefix}===== 共 {len(fields)} 个字段 =====")





# ==================== Test1: CNDE配置与数据获取测试 ====================
# 测试步骤:
# 1. 设置CNDE配置 (JointCurPos, ToolCurPos, 20ms周期)
# 2. 建立RPC连接
# 3. 打印机器人关节和TCP位姿数据
# 4. 获取时间戳并验证周期
# 5. 修改配置 (RobotMode, RbtEnableState, 10ms周期)
# 6. 验证新配置生效
#
# 预期: 
# - 数据正常更新
# - 周期与设置相符
# - 拖动机器人时数据实时变化
# - 配置修改后正常生效


def test1_cnde_config_and_data():
    """Test1: CNDE配置与数据获取测试 - 验证配置设置和数据实时性"""
    print_separator("Test1: CNDE配置与数据获取测试")

    # ===== 步骤1: 设置CNDE配置 (JointCurPos, ToolCurPos, 20ms) =====
    print("\n【步骤1】设置CNDE配置...")
    print("  配置字段: JointCurPos, ToolCurPos")
    print("  反馈周期: 20ms")

    custom_states = [
        RobotState.JointCurPos,   # 关节当前位置
        RobotState.ToolCurPos,    # 工具(TCP)当前位置
    ]

    rtn = SetRobotRealtimeStateConfig(custom_states, 20)
    if rtn != 0:
        print(f"✗ 配置设置失败，错误码: {rtn}")
        return None
    print("✓ CNDE配置设置成功")

    # ===== 步骤2: 建立RPC连接 =====
    print(f"\n【步骤2】建立RPC连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(0.5)  # 等待连接和数据接收

    # 验证配置
    config = robot.CNDEGetConfig()
    if config:
        states, period = config
        print(f"✓ 连接成功，当前配置: {len(states)} 个字段, 周期 {period}ms")
    else:
        print("✗ 无法获取CNDE配置")
        return robot

    # ===== 步骤3: 打印机器人关节和TCP位姿 =====
    print("\n【步骤3】打印机器人关节和TCP位姿...")
    print("  (提示: 可拖动机器人观察数据变化)")
    print("  按 Ctrl+C 停止数据打印")
    print("  (使用 Wireshark 抓包验证实际数据周期)\n")

    sample_count = 0
    try:
        while sample_count < 100:  # 采集100个样本
            pkg = robot.robot_state_pkg

            # 每10帧打印一次
            if sample_count % 10 == 0:
                print(f"--- 样本 #{sample_count} ---")
                print(f"  关节位置 (deg): [{', '.join([f'{x:.3f}' for x in pkg.jt_cur_pos])}]")
                print(f"  TCP位姿 (mm/deg): [{', '.join([f'{x:.3f}' for x in pkg.tl_cur_pos])}]")
                print(f"  当前帧计数: {pkg.frame_cnt}")
                print()

            sample_count += 1
            time.sleep(0.02)  # 20ms

    except KeyboardInterrupt:
        print("\n  用户中断数据打印")

    # 关闭连接
    robot.CloseRPC()
    time.sleep(1)

    # ===== 步骤4: 修改配置并验证 =====
    print("\n【步骤4】修改CNDE配置...")
    print("  新配置字段: RobotMode, RbtEnableState")
    print("  新反馈周期: 10ms")

    new_states = [
        RobotState.RobotMode,
        RobotState.RbtEnableState,
    ]

    # 设置新配置
    rtn = SetRobotRealtimeStateConfig(new_states, 10)
    if rtn != 0:
        print(f"✗ 新配置设置失败，错误码: {rtn}")
        return robot
    print("✓ 新配置设置成功")

    # 重新连接
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(0.5)

    # 验证新配置
    config = robot.CNDEGetConfig()
    if config:
        states, period = config
        print(f"✓ 当前配置: {[s.name for s in states]}")
        print(f"✓ 当前周期: {period}ms")

        if period == 10:
            print("✓ 配置修改验证通过 (周期已变为10ms)")
        else:
            print(f"⚠ 周期未生效 (期望10ms, 实际{period}ms)")

        # 打印新数据
        pkg = robot.robot_state_pkg
        print(f"\n【新配置数据】")
        print(f"  robot_mode: {pkg.robot_mode}")
        print(f"  rbtEnableState: {pkg.rbtEnableState}")
    else:
        print("✗ 无法获取新配置")

    print("\n✓ Test1 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test1_cnde_config_and_data()


# ==================== Test2: Add/Delete 状态字段测试 ====================
# 功能: 测试 AddRobotRealtimeState() 和 DeleteRobotRealtimeState()
# 测试步骤:
#   1. 使用 AddRobotRealtimeState() 添加 SpeedScaleManual 和 SpeedScaleAuto
#   2. 连接机器人，打印手动/自动模式全局速度
#   3. 在 WebApp 修改全局速度，观察 SDK 数据变化
#   4. 使用 DeleteRobotRealtimeState() 删除添加的字段
#   5. 重新连接，验证速度值是否为 0（字段不再更新）
#
# 预期结果:
#   - 添加字段后，SDK 能正常获取速度值
#   - WebApp 修改速度后，SDK 数据同步变化
#   - 删除字段后，速度值保持为 0（字段不再被 CNDE 更新）


def test2_add_delete_state():
    """Test2: Add/Delete 状态字段测试 - 验证动态添加和删除 CNDE 状态"""
    print_separator("Test2: Add/Delete 状态字段测试")

    # ===== 步骤1: 添加 SpeedScaleManual 和 SpeedScaleAuto 字段 =====
    print("\n【步骤1】使用 AddRobotRealtimeState() 添加速度比例字段...")
    print("  添加字段: SpeedScaleManual, SpeedScaleAuto")

    rtn = AddRobotRealtimeState([
        RobotState.SpeedScaleManual,
        RobotState.SpeedScaleAuto,
    ])

    if rtn != 0:
        print(f"✗ 添加字段失败，错误码: {rtn}")
        return None
    print("✓ 字段添加成功")

    # ===== 步骤2: 建立 RPC 连接并打印速度 =====
    print(f"\n【步骤2】建立 RPC 连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(0.5)  # 等待连接和数据接收

    # 验证配置
    config = robot.CNDEGetConfig()
    if config:
        states, period = config
        print(f"✓ 连接成功，当前配置: {len(states)} 个字段")
        # 检查是否包含添加的字段
        has_manual = RobotState.SpeedScaleManual in states
        has_auto = RobotState.SpeedScaleAuto in states
        if has_manual and has_auto:
            print("✓ 配置验证通过: SpeedScaleManual 和 SpeedScaleAuto 已添加")
        else:
            print(f"⚠ 配置验证警告: Manual={has_manual}, Auto={has_auto}")
    else:
        print("✗ 无法获取 CNDE 配置")

    # 打印速度数据
    print("\n【当前速度数据】(请在 WebApp 中修改全局速度观察变化)")
    print("  提示: 拖动机器人使能并切换手/自动模式，观察速度值")
    print("  按 Ctrl+C 停止数据打印\n")

    sample_count = 0
    try:
        while sample_count < 100:  # 采集 100 个样本 (约 10 秒，按 100ms 间隔)
            pkg = robot.robot_state_pkg
            print(f"  [{sample_count:3d}] SpeedScaleManual: {pkg.speedScaleManual:.2f}, "
                  f"SpeedScaleAuto: {pkg.speedScaleAuto:.2f}, "
                  f"Mode: {pkg.robot_mode}")
            sample_count += 1
            time.sleep(0.1)  # 100ms 间隔
    except KeyboardInterrupt:
        print("\n  用户中断数据打印")

    print(f"\n✓ 数据采集完成，共 {sample_count} 个样本")

    # ===== 步骤3: 断开连接 =====
    print("\n【步骤3】断开当前连接...")
    robot.CloseRPC()
    time.sleep(1.0)  # 等待CNDE完全关闭

    # ===== 步骤4: 删除添加的字段 =====
    print("\n【步骤4】使用 DeleteRobotRealtimeState() 删除速度比例字段...")
    rtn = DeleteRobotRealtimeState([
        RobotState.SpeedScaleManual,
        RobotState.SpeedScaleAuto,
    ])

    if rtn != 0:
        print(f"✗ 删除字段失败，错误码: {rtn}")
        return robot
    print("✓ 字段删除成功")

    # ===== 步骤5: 重新连接并验证字段值为 0 =====
    print(f"\n【步骤5】重新连接并验证删除后的字段值...")

    robot = Robot.RPC(ROBOT_IP)
    time.sleep(0.5)

    # 读取速度值
    pkg = robot.robot_state_pkg
    manual_speed = pkg.speedScaleManual
    auto_speed = pkg.speedScaleAuto

    print(f"\n  删除后 SpeedScaleManual: {manual_speed:.2f}")
    print(f"  删除后 SpeedScaleAuto: {auto_speed:.2f}")

    # 验证是否为 0
    if manual_speed == 0 and auto_speed == 0:
        print("\n✓ Test2 验证通过: 删除字段后速度值为 0")
    else:
        print(f"\n⚠ Test2 警告: 删除字段后速度值非零")

    print("\n✓ Test2 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test2_add_delete_state()


# ==================== Test3: 机器人本体状态反馈测试 ====================
def test3_robot_body_state():
    """
    Test3: 机器人本体相关状态反馈字段测试

    测试内容:
    1. 配置机器人本体相关状态字段
    2. 建立RPC连接
    3. 实时打印状态数据
    4. 支持拖动机器人、触发安全停止等操作观察数据变化
    """
    print("\n" + "=" * 60)
    print("Test3: 机器人本体状态反馈测试")
    print("=" * 60)

    # ===== 步骤1: 配置状态字段 =====
    print("\n【步骤1】配置机器人本体状态字段...")

    body_states = [
        RobotState.JointCurPos,           # 关节当前位置
        RobotState.ToolCurPos,            # TCP当前位姿
        RobotState.FlangeCurPos,          # 法兰当前位姿
        RobotState.ActualJointVel,        # 实际关节速度
        RobotState.ActualJointAcc,        # 实际关节加速度
        RobotState.TargetTCPCmpSpeed,     # 目标TCP合成速度
        RobotState.TargetTCPSpeed,        # 目标TCP速度
        RobotState.ActualTCPCmpSpeed,     # 实际TCP合成速度
        RobotState.ActualTCPSpeed,        # 实际TCP速度
        RobotState.ActualJointTorque,     # 实际关节力矩
        RobotState.SafetyStop0State,      # 安全停止0状态
        RobotState.SafetyStop1State,      # 安全停止1状态
        RobotState.JointDriverTorque,     # 关节驱动器力矩
        RobotState.JointDriverTemperature, # 关节驱动器温度
        RobotState.RobotTime,             # 机器人时间
        RobotState.TargetJointTorque,     # 目标关节力矩
        RobotState.WideVoltageCtrlBoxTemp, # 控制箱温度
        RobotState.WideVoltageCtrlBoxFanCurrent, # 控制箱风扇电流
        RobotState.TargetJointPos,        # 目标关节位置
        RobotState.TargetJointVel,        # 目标关节速度
        RobotState.TargetJointAcc,        # 目标关节加速度
        RobotState.TargetJointCurrent,    # 目标关节电流
        RobotState.ActualJointCurrent,    # 实际关节电流
        RobotState.ActualTCPForce,        # 实际TCP力
        RobotState.TargetTCPPos,          # 目标TCP位姿
        RobotState.SafetyBoxSingal,       # 安全箱信号
        RobotState.BtnBoxStopSignal,      # 按钮盒停止信号
    ]

    rtn = SetRobotRealtimeStateConfig(body_states, 8)  # 8ms周期
    if rtn != 0:
        print(f"✗ 配置失败，错误码: {rtn}")
        return None
    print(f"✓ 已配置 {len(body_states)} 个状态字段，周期 8ms")

    # ===== 步骤2: 建立连接 =====
    print(f"\n【步骤2】建立RPC连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(1)
    # 检查RPC连接状态（包括CNDE和XML-RPC）
    print(f"[调试] Robot.RPC.is_connect (类变量) = {Robot.RPC.is_connect}")
    print(f"[调试] robot.SDK_state = {robot.SDK_state}")

    # 双重检查连接状态
    is_connected = Robot.RPC.is_connect
    if not is_connected:
        print("✗✗✗ RPC连接失败（CNDE或XML-RPC连接异常），停止脚本运行 ✗✗✗")
        print(f"    is_connect={is_connected}, 预期继续=False")
        return None
    else:
        print(f"[调试] 连接检查通过，is_connect={is_connected}")

    print("✓ 连接建立成功")

    # ===== 步骤3: 打印状态数据 =====
    print("\n【步骤3】打印机器人本体状态数据...")
    print("  操作提示:")
    print("    - 拖动机器人观察位置/速度变化")
    print("    - 触发安全停止观察 SafetyStop 状态")
    print("    - 按 Ctrl+C 停止数据打印\n")

    sample_count = 0
    try:
        while sample_count < 500:  # 采集500个样本（约4秒）
            pkg = robot.robot_state_pkg

            # 每50帧打印一次（约400ms）
            if sample_count % 50 == 0:
                print(f"--- 样本 #{sample_count} ---")
                print(f"  关节位置 (deg): [{', '.join([f'{x:.3f}' for x in pkg.jt_cur_pos])}]")
                print(f"  TCP位姿 (mm/deg): [{', '.join([f'{x:.3f}' for x in pkg.tl_cur_pos])}]")
                print(f"  实际关节速度: [{', '.join([f'{x:.3f}' for x in pkg.actual_qd])}]")
                print(f"  实际关节力矩: [{', '.join([f'{x:.3f}' for x in pkg.jt_cur_tor])}]")
                print(f"  关节温度 (°C): [{', '.join([f'{x:.1f}' for x in pkg.jointDriverTemperature])}]")
                print(f"  控制箱温度: {pkg.wideVoltageCtrlBoxTemp:.1f}°C")
                print(f"  安全停止0: {pkg.safety_stop0_state}, 安全停止1: {pkg.safety_stop1_state}")
                print(f"  按钮盒停止信号: {pkg.btnBoxStopSignal}")
                print()

            sample_count += 1
            time.sleep(0.008)  # 8ms间隔

    except KeyboardInterrupt:
        print("\n  用户中断数据打印")

    print(f"\n✓ 数据采集完成，共 {sample_count} 个样本")

    # ===== 步骤4: 关闭连接 =====
    print("\n【步骤4】关闭连接...")
    robot.CloseRPC()
    time.sleep(1)

    print("\n✓ Test3 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test3_robot_body_state()


# ==================== Test4: 机器人运行状态反馈测试 ====================
def test4_robot_runtime_state():
    """
    Test4: 机器人运行相关状态反馈测试

    测试内容:
    1. 配置机器人运行相关状态字段
    2. 建立RPC连接
    3. 实时打印运行状态数据
    4. 支持切换手自动、触发DI、切换工具工件坐标系等操作
    """
    print("\n" + "=" * 60)
    print("Test4: 机器人运行状态反馈测试")
    print("=" * 60)

    # ===== 步骤1: 配置状态字段 =====
    print("\n【步骤1】配置机器人运行状态字段...")

    runtime_states = [
        RobotState.ProgramState,        # 程序状态
        RobotState.RobotState,          # 机器人状态
        RobotState.RobotMode,           # 机器人模式
        RobotState.MotionDone,          # 运动完成状态
        RobotState.McQueueLen,          # 运动队列长度
        RobotState.TrajectoryPnum,      # 轨迹点号
        RobotState.Tool,                # 工具号
        RobotState.User,                # 工件号
        RobotState.ClDgtOutputH,        # 控制箱数字输出高字节
        RobotState.ClDgtOutputL,        # 控制箱数字输出低字节
        RobotState.TlDgtOutputL,        # 工具数字输出
        RobotState.ClDgtInputH,          # 控制箱数字输入高字节
        RobotState.ClDgtInputL,          # 控制箱数字输入低字节
        RobotState.TlDgtInputL,          # 工具数字输入
        RobotState.ClAnalogInput,       # 控制箱模拟量输入
        RobotState.TlAnglogInput,       # 工具模拟量输入
        RobotState.RbtEnableState,      # 机器人使能状态
        RobotState.SoftwareUpgradeState, # 软件升级状态
        RobotState.WeldingBreakOffState, # 焊接断弧状态
        RobotState.ClAnalogOutput,      # 控制箱模拟量输出
        RobotState.TlAnalogOutput,      # 工具模拟量输出
        RobotState.ToolCoord,           # 工具坐标
        RobotState.WobjCoord,           # 工件坐标
        RobotState.ExtoolCoord,         # 外部工具坐标
        RobotState.ExAxisCoord,         # 外部轴坐标
        RobotState.Load,                # 负载
        RobotState.LoadCog,             # 负载重心
        RobotState.LastServoTarget,     # 上一次伺服目标
        RobotState.ServoJCmdNum,        # 伺服J命令数量
        RobotState.CollisionLevel,      # 碰撞等级
        RobotState.SpeedScaleManual,    # 手动速度比例
        RobotState.SpeedScaleAuto,      # 自动速度比例
        RobotState.LuaLineNum,          # Lua行号
        RobotState.AbnomalStop,         # 异常停止
        RobotState.CurrentLuaFileName,  # 当前Lua文件名
        RobotState.ProgramTotalLine,    # 程序总行数
        RobotState.WeldVoltage,         # 焊接电压
        RobotState.WeldCurrent,         # 焊接电流
        RobotState.WeldTrackVel,        # 焊接跟踪速度
        RobotState.UdpCmdState,         # UDP命令状态
        RobotState.WeldReadyState,      # 焊接就绪状态
    ]

    rtn = SetRobotRealtimeStateConfig(runtime_states, 8)  # 8ms周期
    if rtn != 0:
        print(f"✗ 配置失败，错误码: {rtn}")
        return None
    print(f"✓ 已配置 {len(runtime_states)} 个状态字段，周期 8ms")

    # ===== 步骤2: 建立连接 =====
    print(f"\n【步骤2】建立RPC连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(1)
    print("✓ 连接建立成功")

    # ===== 步骤3: 打印运行状态数据 =====
    print("\n【步骤3】打印机器人运行状态数据...")
    print("  操作提示:")
    print("    - 切换手自动模式观察 RobotMode 变化")
    print("    - 触发控制箱DI输入观察 ClDgtInput 变化")
    print("    - 切换工具工件坐标系观察 Tool/User 变化")
    print("    - 按 Ctrl+C 停止数据打印\n")

    sample_count = 0
    try:
        while sample_count < 500:  # 采集500个样本
            pkg = robot.robot_state_pkg

            # 每50帧打印一次
            if sample_count % 50 == 0:
                print(f"--- 样本 #{sample_count} ---")
                print(f"  程序状态: {pkg.program_state}, 机器人状态: {pkg.robot_state}, 模式: {pkg.robot_mode}")
                print(f"  运动完成: {pkg.motion_done}, 队列长度: {pkg.mc_queue_len}, 轨迹点: {pkg.trajectory_pnum}")
                print(f"  工具号: {pkg.tool}, 工件号: {pkg.user}")
                print(f"  DI高字节: {pkg.cl_dgt_input_h}, DI低字节: {pkg.cl_dgt_input_l}, 工具DI: {pkg.tl_dgt_input_l}")
                print(f"  DO高字节: {pkg.cl_dgt_output_h}, DO低字节: {pkg.cl_dgt_output_l}, 工具DO: {pkg.tl_dgt_output_l}")
                print(f"  模拟量输入: [{', '.join([f'{x:.2f}' for x in pkg.cl_analog_input])}]")
                print(f"  使能状态: {pkg.rbtEnableState}, 手动速度: {pkg.speedScaleManual:.2f}, 自动速度: {pkg.speedScaleAuto:.2f}")
                print(f"  Lua行号: {pkg.luaLineNum}, 异常停止: {pkg.abnomalStop}")
                print()

            sample_count += 1
            time.sleep(0.008)  # 8ms间隔

    except KeyboardInterrupt:
        print("\n  用户中断数据打印")

    print(f"\n✓ 数据采集完成，共 {sample_count} 个样本")

    # ===== 步骤4: 关闭连接 =====
    print("\n【步骤4】关闭连接...")
    robot.CloseRPC()
    time.sleep(1)

    print("\n✓ Test4 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test4_robot_runtime_state()


# ==================== Test5: 机器人外设状态反馈测试 ====================
def test5_robot_peripheral_state():
    """
    Test5: 机器人外设状态反馈测试

    测试内容:
    1. 配置机器人外设相关状态字段
    2. 建立RPC连接
    3. 实时打印外设状态数据
    4. 支持夹爪激活、运动及力传感器激活等操作
    """
    print("\n" + "=" * 60)
    print("Test5: 机器人外设状态反馈测试")
    print("=" * 60)

    # ===== 步骤1: 配置状态字段 =====
    print("\n【步骤1】配置机器人外设状态字段...")

    peripheral_states = [
        RobotState.FtSensorRawData,       # 力传感器原始数据
        RobotState.FtSensorData,          # 力传感器数据
        RobotState.FtSensorActive,        # 力传感器激活状态
        RobotState.GripperMotiondone,     # 夹爪运动完成
        RobotState.GripperFaultId,        # 夹爪故障ID
        RobotState.GripperFault,          # 夹爪故障
        RobotState.GripperActive,         # 夹爪激活
        RobotState.GripperPosition,       # 夹爪位置
        RobotState.GripperSpeed,          # 夹爪速度
        RobotState.GripperCurrent,        # 夹爪电流
        RobotState.GripperTemp,           # 夹爪温度
        RobotState.GripperVoltage,        # 夹爪电压
        RobotState.GripperRotNum,         # 旋转夹爪圈数
        RobotState.GripperRotSpeed,       # 旋转夹爪速度
        RobotState.GripperRotTorque,       # 旋转夹爪力矩
        RobotState.SmartToolState,        # 智能工具状态
        RobotState.ModbusMasterConnect,   # Modbus主站连接
        RobotState.ModbusSlaveConnect,    # Modbus从站连接
        RobotState.ForceSensorErrState,   # 力传感器错误状态
    ]

    rtn = SetRobotRealtimeStateConfig(peripheral_states, 8)  # 8ms周期
    if rtn != 0:
        print(f"✗ 配置失败，错误码: {rtn}")
        return None
    print(f"✓ 已配置 {len(peripheral_states)} 个状态字段，周期 8ms")

    # ===== 步骤2: 建立连接 =====
    print(f"\n【步骤2】建立RPC连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(1)

    # 检查RPC连接状态（包括CNDE和XML-RPC）
    print(f"[调试] Robot.RPC.is_connect (类变量) = {Robot.RPC.is_connect}")
    print(f"[调试] robot.SDK_state = {robot.SDK_state}")
    
    # 双重检查连接状态
    is_connected = Robot.RPC.is_connect
    if not is_connected:
        print("✗✗✗ RPC连接失败（CNDE或XML-RPC连接异常），停止脚本运行 ✗✗✗")
        print(f"    is_connect={is_connected}, 预期继续=False")
        return None
    else:
        print(f"[调试] 连接检查通过，is_connect={is_connected}")

    print("✓ 连接建立成功")

    # ===== 步骤3: 打印外设状态数据 =====
    print("\n【步骤3】打印机器人外设状态数据...")
    print("  操作提示:")
    print("    - 夹爪激活/运动观察 GripperActive/Position/Speed 变化")
    print("    - 力传感器激活观察 FtSensorActive 和 FtSensorData 变化")
    print("    - 旋转夹爪运动观察 GripperRotNum/RotSpeed 变化")
    print("    - 按 Ctrl+C 停止数据打印\n")

    sample_count = 0
    try:
        while sample_count < 5000:  # 采集500个样本
            pkg = robot.robot_state_pkg

            # 每50帧打印一次
            if sample_count % 50 == 0:
                print(f"--- 样本 #{sample_count} ---")
                # print(f"  力传感器激活: {pkg.ft_sensor_active}")
                # print(f"  力传感器数据: [{', '.join([f'{x:.3f}' for x in pkg.ft_sensor_data])}]")
                print(f"  夹爪激活: {pkg.gripper_active}, 完成: {pkg.gripper_motiondone}")
                print(f"  夹爪位置: {pkg.gripper_position}, 速度: {pkg.gripper_speed}, 电流: {pkg.gripper_current}")
                print(f"  夹爪温度: {pkg.gripper_temp}°C, 电压: {pkg.gripper_voltage}V")
                print(f"  旋转夹爪圈数: {pkg.gripperRotNum:.2f}, 速度: {pkg.gripperRotSpeed}, 力矩: {pkg.gripperRotTorque}")
                print(f"  夹爪故障: {pkg.gripper_fault}, 故障ID: {pkg.gripper_fault_id}")
                # print(f"  智能工具状态: {bin(pkg.smartToolState)}")
                # print(f"  Modbus主站: {pkg.modbusMasterConnect}, 从站: {pkg.modbusSlaveConnect}")
                # print(f"  力传感器错误: {pkg.forceSensorErrState}")
                print()

            sample_count += 1
            time.sleep(0.008)  # 8ms间隔

    except KeyboardInterrupt:
        print("\n  用户中断数据打印")

    print(f"\n✓ 数据采集完成，共 {sample_count} 个样本")

    # ===== 步骤4: 关闭连接 =====
    print("\n【步骤4】关闭连接...")
    robot.CloseRPC()
    time.sleep(1)

    print("\n✓ Test5 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test5_robot_peripheral_state()


# ==================== Test6: 扩展轴及扩展IO状态反馈测试 ====================
def test6_ext_axis_io_state():
    """
    Test6: 扩展轴及扩展IO状态反馈测试

    测试内容:
    1. 配置扩展轴和扩展IO状态字段
    2. 建立RPC连接
    3. 实时打印扩展轴和扩展IO状态数据
    4. 支持485/UDP扩展轴运动、扩展IO触发等操作
    """
    print("\n" + "=" * 60)
    print("Test6: 扩展轴及扩展IO状态反馈测试")
    print("=" * 60)

    # ===== 步骤1: 配置状态字段 =====
    print("\n【步骤1】配置扩展轴和扩展IO状态字段...")

    ext_states = [
        RobotState.AuxState,        # 辅助轴状态
        RobotState.ExtAxisStatus,   # 扩展轴状态
        RobotState.ExtDIState,      # 扩展数字输入
        RobotState.ExtDOState,      # 扩展数字输出
        RobotState.ExtAIState,      # 扩展模拟量输入
        RobotState.ExtAOState,      # 扩展模拟量输出
    ]

    rtn = SetRobotRealtimeStateConfig(ext_states, 8)  # 8ms周期
    if rtn != 0:
        print(f"✗ 配置失败，错误码: {rtn}")
        return None
    print(f"✓ 已配置 {len(ext_states)} 个状态字段，周期 8ms")

    # ===== 步骤2: 建立连接 =====
    print(f"\n【步骤2】建立RPC连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(1)
    print("✓ 连接建立成功")

    # ===== 步骤3: 打印扩展轴和扩展IO状态数据 =====
    print("\n【步骤3】打印扩展轴及扩展IO状态数据...")
    print("  操作提示:")
    print("    - 485扩展轴运动观察 AuxState 变化")
    print("    - UDP扩展轴运动观察 ExtAxisStatus 变化")
    print("    - 扩展IO触发观察 ExtDI/ExtDO 变化")
    print("    - 按 Ctrl+C 停止数据打印\n")

    sample_count = 0
    try:
        while sample_count < 5000:  # 采集5000个样本
            pkg = robot.robot_state_pkg

            # 每50帧打印一次
            if sample_count % 50 == 0:
                print(f"--- 样本 #{sample_count} ---")
                # 辅助轴状态（25个辅助轴）
                aux_active = [i for i, v in enumerate(pkg.aux_axis_state) if v != 0]
                print(f"  辅助轴状态: 活跃轴数量={len(aux_active)}, 状态列表={aux_active[:5]}...")
                # 扩展轴状态
                print(f"  扩展轴状态 (前4个轴):")
                for i, axis in enumerate(pkg.extAxisStatus[:4]):
                    print(f"    轴{i}: pos={axis.pos:.3f}, vel={axis.vel:.3f}, errorCode={axis.errorCode}, "
                          f"ready={axis.ready}, inPos={axis.inPos}, alarm={axis.alarm}, "
                          f"flerr={axis.flerr}, nlimit={axis.nlimit}, pLimit={axis.pLimit}")
                # # 扩展IO
                # print(f"  扩展DI: {list(pkg.extDIState)}")
                # print(f"  扩展DO: {list(pkg.extDOState)}")
                # print(f"  扩展AI: {list(pkg.extAIState)}")
                # print(f"  扩展AO: {list(pkg.extAOState)}")
                # print()

            sample_count += 1
            time.sleep(0.008)  # 8ms间隔

    except KeyboardInterrupt:
        print("\n  用户中断数据打印")

    print(f"\n✓ 数据采集完成，共 {sample_count} 个样本")

    # ===== 步骤4: 关闭连接 =====
    print("\n【步骤4】关闭连接...")
    robot.CloseRPC()
    time.sleep(1)

    print("\n✓ Test6 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test6_ext_axis_io_state()


# ==================== Test7: 机器人故障状态反馈测试 ====================
def test7_robot_fault_state():
    """
    Test7: 机器人故障状态反馈测试

    测试内容:
    1. 配置机器人故障相关状态字段
    2. 建立RPC连接
    3. 实时打印故障状态数据
    4. 支持触发碰撞、奇异位姿等操作观察故障状态变化
    """
    print("\n" + "=" * 60)
    print("Test7: 机器人故障状态反馈测试")
    print("=" * 60)

    # ===== 步骤1: 配置状态字段 =====
    print("\n【步骤1】配置机器人故障状态字段...")

    fault_states = [
        RobotState.EmergencyStop,       # 紧急停止
        RobotState.MainCode,            # 主错误码
        RobotState.SubCode,             # 子错误码
        RobotState.CollisionState,      # 碰撞状态
        RobotState.EndLuaErrCode,       # Lua错误码
        RobotState.TpdException,        # TPD异常
        RobotState.AlarmRebootRobot,    # 报警重启机器人
        RobotState.DragAlarm,           # 拖动报警
        RobotState.SafetyDoorAlarm,     # 安全门报警
        RobotState.SafetyPlaneAlarm,    # 安全平面报警
        RobotState.MotonAlarm,          # 运动报警
        RobotState.InterfaceAlarm,      # 接口报警
        RobotState.AlarmCheckEmergStopBtn,  # 急停按钮报警检查
        RobotState.TsTmCmdComError,     # 命令通信错误
        RobotState.TsTmStateComError,   # 状态通信错误
        RobotState.CtrlBoxError,        # 控制箱错误
        RobotState.SafetyDataState,     # 安全数据状态
        RobotState.CtrlOpenLuaErrCode,  # 打开Lua错误码
        RobotState.StrangePosFlag,      # 奇异位姿标志
        RobotState.Alarm,               # 报警
        RobotState.DriverAlarm,         # 驱动器报警
        RobotState.AliveSlaveNumError,  # 从站数量错误
        RobotState.SlaveComError,       # 从站通信错误
        RobotState.CmdPointError,       # 命令点错误
        RobotState.IOError,             # IO错误
        RobotState.GripperError,        # 夹爪错误
        RobotState.FileError,           # 文件错误
        RobotState.ParaError,           # 参数错误
        RobotState.ExaxisOutLimitError, # 扩展轴超限错误
        RobotState.DriverComError,      # 驱动器通信错误
        RobotState.DriverError,         # 驱动器错误
        RobotState.OutSoftLimitError,   # 超软限位错误
    ]

    rtn = SetRobotRealtimeStateConfig(fault_states, 8)  # 8ms周期
    if rtn != 0:
        print(f"✗ 配置失败，错误码: {rtn}")
        return None
    print(f"✓ 已配置 {len(fault_states)} 个状态字段，周期 8ms")

    # ===== 步骤2: 建立连接 =====
    print(f"\n【步骤2】建立RPC连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(1)
    print("✓ 连接建立成功")

    # ===== 步骤3: 打印故障状态数据 =====
    print("\n【步骤3】打印机器人故障状态数据...")
    print("  操作提示:")
    print("    - 触发碰撞观察 CollisionState/Alarm 变化")
    print("    - 进入奇异位姿观察 StrangePosFlag 变化")
    print("    - 触发急停观察 EmergencyStop 变化")
    print("    - 按 Ctrl+C 停止数据打印\n")

    sample_count = 0
    try:
        while sample_count < 10000:  # 采集500个样本
            pkg = robot.robot_state_pkg

            # 每100帧打印一次
            if sample_count % 100 == 0:
                print(f"--- 样本 #{sample_count} ---")
                print(f"  紧急停止: {pkg.EmergencyStop}, 主错误码: {pkg.main_code}, 子错误码: {pkg.sub_code}")
                print(f"  碰撞状态: {pkg.collisionState}, 奇异位姿: {pkg.strangePosFlag}")
                print(f"  报警: {pkg.alarm}, 驱动器报警: {pkg.driverAlarm}")
                print(f"  Lua错误码: {pkg.endLuaErrCode}, TPD异常: {pkg.tpdException}")
                print(f"  安全门报警: {pkg.safetyDoorAlarm}, 安全平面报警: {pkg.safetyPlaneAlarm}")
                print(f"  拖动报警: {pkg.dragAlarm}, 运动报警: {pkg.motonAlarm}")
                print(f"  从站通信错误: {list(pkg.slaveComError)}, 从站数量错误: {pkg.aliveSlaveNumError}")
                print(f"  IO错误: {pkg.IOError}, 夹爪错误: {pkg.gripperError}")
                print(f"  控制箱错误: {pkg.ctrlBoxError}, 超软限位: {pkg.outSoftLimitError}")
                print()

            sample_count += 1
            time.sleep(0.008)  # 8ms间隔

    except KeyboardInterrupt:
        print("\n  用户中断数据打印")

    print(f"\n✓ 数据采集完成，共 {sample_count} 个样本")

    # ===== 步骤4: 关闭连接 =====
    print("\n【步骤4】关闭连接...")
    robot.CloseRPC()
    time.sleep(1)

    print("\n✓ Test7 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
if __name__ == "__main__":
    test7_robot_fault_state()


# ==================== Test8: 错误码测试 ====================
def test8_error_code_validation():
    """
    Test8: 错误码验证测试

    测试内容:
    1. 正常配置（周期2000ms）
    2. 周期越界测试（7ms，低于最小值8ms）
    3. 空状态列表测试
    4. 添加已存在状态测试
    5. 删除不存在状态测试
    6. 周期设置越界测试（7ms和1001ms）
    """
    print("\n" + "=" * 60)
    print("Test8: 错误码验证测试")
    print("=" * 60)

    from fairino.Robot import RobotError

    results = []

    # ===== 步骤1: 正常配置（周期2000ms）=====
    print("\n【步骤1】正常配置测试（周期2000ms）...")
    states_normal = [RobotState.JointCurPos, RobotState.ToolCurPos]
    rtn = SetRobotRealtimeStateConfig(states_normal, 2000)
    results.append(("SetRobotRealtimeStateConfig(2000ms)", rtn, RobotError.ERR_PARAM_VALUE))
    print(f"  返回值: {rtn} (期望: {RobotError.ERR_PARAM_VALUE}) {'✓' if rtn == RobotError.ERR_PARAM_VALUE else '✗'}")
    time.sleep(1)
    # ===== 步骤2: 周期越界测试（7ms）=====
    print("\n【步骤2】周期越界测试（7ms，低于最小值8ms）...")
    rtn = SetRobotRealtimeStateConfig(states_normal, 7)
    results.append(("SetRobotRealtimeStateConfig(7ms)", rtn, RobotError.ERR_PARAM_VALUE))
    print(f"  返回值: {rtn} (期望: {RobotError.ERR_PARAM_VALUE}) {'✓' if rtn == RobotError.ERR_PARAM_VALUE else '✗'}")
    time.sleep(1)
    # ===== 步骤3: 空状态列表测试 =====
    print("\n【步骤3】空状态列表测试...")
    rtn = SetRobotRealtimeStateConfig([], 8)
    results.append(("SetRobotRealtimeStateConfig([])", rtn, RobotError.ERR_NEED_AT_LEAST_ONE_STATE))
    print(f"  返回值: {rtn} (期望: {RobotError.ERR_NEED_AT_LEAST_ONE_STATE}) {'✓' if rtn == RobotError.ERR_NEED_AT_LEAST_ONE_STATE else '✗'}")
    time.sleep(1)
    # 重新设置正常配置用于后续测试
    SetRobotRealtimeStateConfig([RobotState.JointCurPos, RobotState.ToolCurPos], 8)

    # ===== 步骤4: 添加已存在状态测试 =====
    print("\n【步骤4】添加已存在状态测试（JointCurPos已存在）...")
    rtn = AddRobotRealtimeState([RobotState.JointCurPos])
    results.append(("AddRobotRealtimeState(JointCurPos - 已存在)", rtn, RobotError.ERR_STATE_ALREADY_EXISTS))
    print(f"  返回值: {rtn} (期望: {RobotError.ERR_STATE_ALREADY_EXISTS}) {'✓' if rtn == RobotError.ERR_STATE_ALREADY_EXISTS else '✗'}")
    time.sleep(1)
    # ===== 步骤5: 删除不存在状态测试 =====
    print("\n【步骤5】删除不存在状态测试（CtrlBoxError）...")
    rtn = DeleteRobotRealtimeState([RobotState.CtrlBoxError])
    results.append(("DeleteRobotRealtimeState(CtrlBoxError - 不存在)", rtn, RobotError.ERR_STATE_INVALID))
    print(f"  返回值: {rtn} (期望: {RobotError.ERR_STATE_INVALID}) {'✓' if rtn == RobotError.ERR_STATE_INVALID else '✗'}")
    time.sleep(1)

    # ===== 打印汇总结果 =====
    print("\n" + "=" * 60)
    print("Test8 测试结果汇总")
    print("=" * 60)
    passed = 0
    failed = 0
    for name, actual, expected in results:
        status = "✓ 通过" if actual == expected else "✗ 失败"
        if actual == expected:
            passed += 1
        else:
            failed += 1
        print(f"  {status} | {name}")
        print(f"         实际值: {actual}, 期望值: {expected}")

    print(f"\n总计: {passed} 通过, {failed} 失败")
    print("\n✓ Test8 完成")
    return None


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test8_error_code_validation()


# ==================== Test9: 断线重连测试 ====================
def test9_connection_recovery():
    """
    Test9: CNDE断线重连测试

    测试内容:
    1. 配置CNDE状态（JointCurPos, ToolCurPos），周期20ms
    2. 建立RPC连接，循环打印机器人状态
    3. 拔掉网线，超时时间内插回，观察CNDE是否恢复
    4. 交换机断电再上电，观察CNDE是否恢复
    """
    print("\n" + "=" * 60)
    print("Test9: CNDE断线重连测试")
    print("=" * 60)

    # 配置CNDE状态
    SetRobotRealtimeStateConfig([RobotState.JointCurPos, RobotState.ToolCurPos], 20)

    # 建立RPC连接
    robot = Robot.RPC(ROBOT_IP)
    print("✓ 连接成功，开始打印...")
    print("  拔掉网线会暂停，插回后自动继续")
    print("  按 Ctrl+C 停止\n")

    # 设置重连参数
    robot.SetReConnectParam(True, 30, 5000)

    count = 0
    try:
        while True:
            try:
                err_j, joint_pos = robot.GetActualJointPosDegree()
                err_t, tcp_pos = robot.GetActualTCPPose()

                if err_j == 0 and err_t == 0:
                    count += 1
                    print(f"[{count:04d}] 关节: {[f'{x:7.2f}' for x in joint_pos]} | "
                          f"TCP: {[f'{x:7.2f}' for x in tcp_pos]}")
            except Exception:
                pass

            time.sleep(0.02)

    except KeyboardInterrupt:
        print(f"\n停止，共打印 {count} 条数据")

    robot.CloseRPC()
    return robot
# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test9_connection_recovery()

def test10_square_motion():
    """
    Test10: 正方形运动测试

    测试内容:
    1. 配置CNDE状态（JointCurPos, ToolCurPos, MotionDone），周期8ms
    2. 建立RPC连接
    3. 执行正方形MoveL运动：边长200mm，循环10次
    4. 每次运动后等待MotionDone完成信号
    5. 打印当前TCP位姿
    """
    print("\n" + "=" * 60)
    print("Test10: 正方形运动测试")
    print("=" * 60)

    # 1. 配置CNDE状态
    print("\n【步骤1】配置CNDE状态...")
    states = [
        RobotState.JointCurPos,   # 关节当前位置
        RobotState.ToolCurPos,    # 工具当前位置
        RobotState.MotionDone,    # 运动完成信号
    ]
    period_ms = 8
    ret = SetRobotRealtimeStateConfig(states, period_ms)
    print(f"  配置结果: {ret}")
    if ret != 0:
        print("✗ 配置失败")
        return None
    print(f"✓ 已配置 {len(states)} 个状态字段，周期 {period_ms}ms")

    # 2. 建立RPC连接
    print(f"\n【步骤2】建立RPC连接 ({ROBOT_IP})...")
    robot = Robot.RPC(ROBOT_IP)
    time.sleep(2)
    print("✓ RPC连接成功，CNDE已连接")

    # 3. 运动参数
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blend_r = -1.0  # 运动到位（阻塞），但我们将手动等待MotionDone
    exaxis_pos = [0.0, 0.0, 0.0, 0.0]
    search = 0
    offset_flag = 0
    offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    oacc = 100.0

    step = 100.0  # 边长100mm
    cycle = 0   # 循环次数

    print(f"\n【步骤3】开始正方形运动")
    print(f"  边长: {step}mm")
    print(f"  速度: {vel}mm/s，加速度: {acc}%")

    while True:
        print(f"\n---------- 第 {cycle} 次正方形运动 ----------")
        cycle = cycle + 1
        # 获取当前TCP位姿作为起点
        err_j, joint_pos = robot.GetActualJointPosDegree()
        err_t, tcp_pos = robot.GetActualTCPPose()

        if err_t != 0 or tcp_pos is None:
            print("  ✗ 获取当前位姿失败，退出")
            break

        print(f"  起点TCP: X={tcp_pos[0]:.2f}, Y={tcp_pos[1]:.2f}, Z={tcp_pos[2]:.2f}, "
              f"RX={tcp_pos[3]:.2f}, RY={tcp_pos[4]:.2f}, RZ={tcp_pos[5]:.2f}")

        start_x, start_y, start_z = tcp_pos[0], tcp_pos[1], tcp_pos[2]
        start_rx, start_ry, start_rz = tcp_pos[3], tcp_pos[4], tcp_pos[5]

        # 定义四个目标点（相对起点）
        target1 = [start_x + step, start_y, start_z, start_rx, start_ry, start_rz]  # 右
        target2 = [start_x + step, start_y + step, start_z, start_rx, start_ry, start_rz]  # 右上
        target3 = [start_x, start_y + step, start_z, start_rx, start_ry, start_rz]  # 左上
        target4 = [start_x, start_y, start_z, start_rx, start_ry, start_rz]  # 回到起点

        targets = [
            (target1, "右边"),
            (target2, "右上"),
            (target3, "左上"),
            (target4, "起点")
        ]

        # 执行四条边
        for target, name in targets:
            # 使用逆运动学求解关节位置
            err, jpos = robot.GetInverseKinRef(0, target, joint_pos)
            if err != 0:
                print(f"  ✗ 逆运动学求解失败: {err}")
                continue

            # 执行MoveL (使用关键字参数，明确对应每个参数)
            rtn = robot.MoveL(
                joint_pos=jpos,
                desc_pos=target,
                tool=tool,
                user=user,
                vel=vel,
                acc=acc,
                ovl=ovl,
                blendR=blend_r,
                blendMode=0,
                exaxis_pos=exaxis_pos,
                search=search,
                offset_flag=offset_flag,
                offset_pos=offset_pos,
                oacc=oacc
            )
            if rtn != 0:
                print(f"  ✗ MoveL指令失败: {rtn}")
                continue

            # 等待MotionDone完成
            timeout_ms = 10000  # 10秒超时
            interval_ms = 8   # 检查间隔8ms
            start_time = time.time()
            motion_done = False

            while (time.time() - start_time) * 1000 < timeout_ms:
                err_d, done = robot.GetRobotMotionDone()
                if err_d != 0:
                    print(f"  ✗ GetRobotMotionDone失败: {err_d}")
                    break
                if done == 1:
                    print(f"  ✓ 运动到位 [{name}]: X={target[0]:.1f}, Y={target[1]:.1f}, Z={target[2]:.1f}")
                    motion_done = True
                    break
                time.sleep(interval_ms / 1000.0)

            if not motion_done:
                print(f"  ✗ 等待运动完成超时 (>10s)")

        print(f"  第 {cycle} 次正方形运动完成")
        time.sleep(1)

    print("\n所有运动完成!")

    # 关闭连接
    print("\n【步骤4】关闭连接...")
    robot.CloseRPC()

    print("\n✓ Test10 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test10_square_motion()


# ==================== Test11: ServoJ运动测试 ====================
def test11_servoj_motion():
    """
    Test11: ServoJ运动测试（参考C++ servoj测试用例）
    功能: 测试ServoJ实时控制接口，监控运动队列和LastServoTarget
    """
    print("\n" + "=" * 60)
    print("Test11: ServoJ运动测试")
    print("=" * 60)

    # ===== 步骤1: 配置CNDE状态 =====
    print("\n【步骤1】配置CNDE状态...")
    states = [
        RobotState.JointCurPos,      # 关节当前位置
        RobotState.MotionDone,       # 运动完成标志
        RobotState.McQueueLen,       # 运动队列长度
        RobotState.LastServoTarget,  # 最后一次ServoJ目标位置
        RobotState.ServoJCmdNum,     # ServoJ命令计数
    ]
    rtn = SetRobotRealtimeStateConfig(states, 20)  # 20ms周期
    if rtn != 0:
        print(f"   ✗ CNDE配置失败: {rtn}")
        return None
    print(f"   ✓ CNDE配置成功: {len(states)} 个状态, 20ms周期")

    # ===== 步骤2: 连接机器人 =====
    print("\n【步骤2】连接机器人...")
    robot = Robot.RPC('192.168.58.2')
    if robot is None:
        print("   ✗ 机器人连接失败")
        return None
    print("   ✓ 机器人连接成功")

    # 设置重连参数
    robot.SetReConnectParam(True, 30, 500)
    print("   ✓ 重连参数已设置")

    # ===== 步骤3: 获取当前关节位置 =====
    print("\n【步骤3】获取初始关节位置...")
    flag = 0
    ret, joint_pos = robot.GetActualJointPosDegree(flag)
    if ret != 0:
        print(f"   ✗ 获取关节位置失败: {ret}")
        robot.CloseRPC()
        return robot

    # 扩展轴位置
    epos = [0.0, 0.0, 0.0, 0.0]

    # ServoJ参数
    vel = 0.0
    acc = 0.0
    cmdT = 0.008  # 8ms
    filterT = 0.0
    gain = 0.0
    count = 500
    dt = 0.1  # 每次增加0.1度
    cmdID = 0

    # ===== 步骤4: 开始ServoJ运动 =====
    print("\n【步骤4】开始ServoJ运动...")
    print(f"   参数: count={count}, dt={dt}, cmdT={cmdT}")
    print(f"   {'Count':>8} | {'J5 Target':>10} | {'QueueLen':>8} | {'MotionDone':>10}")
    print("   " + "-" * 50)

    # 获取包引用
    pkg = robot.robot_state_pkg

    robot.ServoMoveStart()
    cmdID += 1

    try:
        while count > 0:
            # 执行ServoJ
            ret = robot.ServoJ(joint_pos, epos, acc, vel, cmdT, filterT, gain, cmdID)
            if ret != 0:
                print(f"   ServoJ失败: {ret}")
                break

            # 更新关节位置（第5轴递增）
            joint_pos[4] += dt
            cmdID += 1
            count -= 1

            # 每50次打印一次状态
            if count % 50 == 0 or count < 55:
                # 获取实时状态
                last_pos = list(pkg.lastServoTarget)
                queue_len = pkg.mc_queue_len
                motion_done = pkg.motion_done
                servo_cmd_num = pkg.servoJCmdNum

                print(f"   {servo_cmd_num:>8} | {last_pos[4]:>10.3f} | {queue_len:>8} | {motion_done:>10}")

                # 当计数小于50时，清空运动队列并退出
                if count < 50:
                    print(f"\n   计数剩余 {count}，清空运动队列...")
                    robot.MotionQueueClear()

                    # 打印清空后的状态
                    time.sleep(0.1)
                    last_pos = list(pkg.lastServoTarget)
                    queue_len = pkg.mc_queue_len
                    servo_cmd_num = pkg.servoJCmdNum
                    print(f"   清空后: CmdNum={servo_cmd_num}, LastJ5={last_pos[4]:.3f}, Queue={queue_len}")
                    break

            time.sleep(cmdT)

        print("   " + "-" * 50)
        print(f"   ✓ ServoJ循环完成，剩余计数: {count}")

    except KeyboardInterrupt:
        print("\n   用户中断运动")
    except Exception as e:
        print(f"\n   运动异常: {e}")
    finally:
        robot.ServoMoveEnd()
        print("   ✓ Servo运动已结束")

    # ===== 步骤5: 关闭连接 =====
    print("\n【步骤5】关闭连接...")
    time.sleep(1)
    robot.CloseRPC()
    time.sleep(1)

    print("\n✓ Test11 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test11_servoj_motion()


# ==================== Test12: 艾灸头外设测试 ====================
def test12_moxibustion_head():
    """
    Test12: 艾灸头外设测试（参考C#测试用例）
    功能: 测试艾灸头通信，读取版本号、在位状态和温度数据
    """
    print("\n" + "=" * 60)
    print("Test12: 艾灸头外设测试")
    print("=" * 60)

    # ===== 步骤1: 配置CNDE状态 =====
    print("\n【步骤1】配置CNDE状态...")
    states = [
        RobotState.AxleGenComData,  # 末端通用透传数据（130字节）
    ]
    rtn = SetRobotRealtimeStateConfig(states, 20)  # 20ms周期
    if rtn != 0:
        print(f"   ✗ CNDE配置失败: {rtn}")
        return None
    print(f"   ✓ CNDE配置成功: {len(states)} 个状态, 20ms周期")

    # ===== 步骤2: 连接机器人 =====
    print("\n【步骤2】连接机器人...")
    robot = Robot.RPC('192.168.58.3')
    if robot is None:
        print("   ✗ 机器人连接失败")
        return None
    print("   ✓ 机器人连接成功")

    # 设置重连参数
    robot.SetReConnectParam(True, 30, 500)
    print("   ✓ 重连参数已设置")

    # ===== 步骤3: 艾灸头通信测试 =====
    print("\n【步骤3】艾灸头通信测试...")

    frame_count = 0
    max_frames = 20  # 采集20帧数据

    try:
        while frame_count < max_frames:
            # 获取实时状态包
            pkg = robot.robot_state_pkg

            print(f"\n========== 帧 {frame_count + 1} @ {time.strftime('%H:%M:%S')} ==========")

            # # 读取版本号（命令5）
            # version_cmd = [0] * 10
            # ret, rcv_data = robot.SndRcvAxleGenComCmdData(5, version_cmd, 10)
            # if ret != 0:
            #     print(f"   ✗ 读取版本号失败: {ret}")
            #     break
            #
            # # 解析版本号数据
            # if len(rcv_data) >= 9:
            #     hard_version = rcv_data[4]
            #     hard_code = rcv_data[5]
            #     soft_version_major = rcv_data[6]
            #     soft_version_minor = rcv_data[7]
            #     soft_code = rcv_data[8]
            #     print(f"   硬件版本: {hard_version}, 硬件代码: {hard_code}")
            #     print(f"   软件版本: {soft_version_major}.{soft_version_minor}, 软件代码: {soft_code}")
            #
            # time.sleep(1)
            #
            # # 读取艾灸头在位状态（命令6）
            # state_cmd = [0] * 6
            # ret, rcv_data = robot.SndRcvAxleGenComCmdData(6, state_cmd, 6)
            # if ret != 0:
            #     print(f"   ✗ 读取在位状态失败: {ret}")
            #     break
            #
            # if len(rcv_data) >= 5:
            #     moxi_state = rcv_data[4]
            #     print(f"   艾灸头在位状态: {moxi_state}")
            #
            # time.sleep(1)

            # 从CNDE实时数据解析艾灸头数据
            axle_data = pkg.axleGenComData

            # 检查数据有效性
            error_code = axle_data[0]
            data_len = axle_data[1]

            # 过滤异常包（检查包头0xAB 0xBA）
            if error_code != 0 or data_len == 0 or axle_data[2] != 0xAB or axle_data[3] != 0xBA:
                print(f"   ⚠ 数据包异常: error_code={error_code}, data_len={data_len}")
                print(f"      原始数据前20字节: {axle_data[:20]}")
            else:
                # 解析艾灸头数据
                cur_tem = axle_data[6]
                target_tem = axle_data[7]

                # genData1-6 是16位数据（高低字节组合）
                gen_data1 = (axle_data[8] << 8) | axle_data[9]
                gen_data2 = (axle_data[10] << 8) | axle_data[11]
                gen_data3 = (axle_data[12] << 8) | axle_data[13]
                gen_data4 = (axle_data[14] << 8) | axle_data[15]
                gen_data5 = (axle_data[16] << 8) | axle_data[17]
                gen_data6 = (axle_data[18] << 8) | axle_data[19]

                print(f"   错误码: {error_code}, 数据长度: {data_len}")
                print(f"   当前温度: {cur_tem}°C, 目标温度: {target_tem}°C")
                print(f"   数据1: {gen_data1}, 数据2: {gen_data2}, 数据3: {gen_data3}")
                print(f"   数据4: {gen_data4}, 数据5: {gen_data5}, 数据6: {gen_data6}")

            frame_count += 1
            time.sleep(0.5)

        print(f"\n   ✓ 采集完成，共 {frame_count} 帧")

    except KeyboardInterrupt:
        print("\n   用户中断测试")
    except Exception as e:
        print(f"\n   测试异常: {e}")
        import traceback
        traceback.print_exc()

    # ===== 步骤4: 关闭连接 =====
    print("\n【步骤4】关闭连接...")
    time.sleep(1)
    robot.CloseRPC()
    time.sleep(1)

    print("\n✓ Test12 完成")
    return robot


# 独立运行入口 (取消下面两行注释即可单独运行此测试)
# if __name__ == "__main__":
#     test12_moxibustion_head()


# ==================== 测试项占位区 ====================
# 以下测试项将逐个添加:
# ... (更多测试项)


# ==================== 全部测试运行入口 ====================
def run_all_tests():
    """运行全部测试"""
    print("\n" + "#" * 60)
    print("#" + "CNDE 综合测试套件".center(58) + "#")
    print("#" * 60)

    try:
        # 依次运行所有测试
        # robot = test1_basic_connection()
        # test2_default_config_data(robot)
        # ... 其他测试

        print("\n" + "=" * 60)
        print("全部测试完成!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    except Exception as e:
        print(f"\n测试异常: {e}")
        import traceback
        traceback.print_exc()


# 运行全部测试入口 (取消下面两行注释即可运行全部测试)
# if __name__ == "__main__":
#     run_all_tests()
