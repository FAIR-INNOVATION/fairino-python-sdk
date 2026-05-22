"""
CNDE (Custom Network Data Export) 测试用例
测试内容：
1. CNDE 自动连接验证（RPC初始化时自动连接）
2. 默认配置数据接收测试
3. 自定义配置测试（添加/删除状态）
4. 数据正确性验证
5. 周期设置测试
"""

import sys
sys.path.insert(0, r'f:\PythonSDK\SDKV2.0.5\SDKV2.2.5')
from fairino import Robot
from fairino.Robot import RobotState
import time

# ==================== 配置参数 ====================
ROBOT_IP = '192.168.58.2'  # 机器人IP地址
TEST_DURATION = 10  # 测试持续时间（秒）
SAMPLE_PERIOD = 100  # 数据采样周期（ms）


def print_separator(title):
    """打印分隔线"""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def test1_auto_connection():
    """
    测试1: CNDE自动连接验证
    RPC初始化时会自动连接CNDE（20005端口）
    """
    print_separator("测试1: CNDE自动连接验证")
    
    print(f"正在连接机器人 {ROBOT_IP} ...")
    robot = Robot.RPC(ROBOT_IP)
    
    # 检查CNDE客户端是否已创建
    if hasattr(robot, '_cnde_client') and robot._cnde_client is not None:
        print("✓ CNDE客户端已创建")
        
        # 获取当前配置
        config = robot.CNDEGetConfig()
        if config:
            states, period = config
            print(f"✓ CNDE默认配置: {len(states)} 个状态, 周期 {period}ms")
            print(f"  配置状态列表: {[s.name for s in states[:5]]} ... (共{len(states)}个)")
        else:
            print("✗ 无法获取CNDE配置")
    else:
        print("✗ CNDE客户端未创建")
    
    return robot


def test2_default_config_data_verify(robot):
    """
    测试2: 默认配置数据接收与正确性验证
    验证默认配置（到LastServoTarget）的数据接收和字段值
    """
    print_separator("测试2: 默认配置数据接收与正确性验证")
    
    # 获取当前配置
    config = robot.CNDEGetConfig()
    if not config:
        print("✗ 获取配置失败")
        return False
    
    states, period = config
    print(f"当前配置: {len(states)} 个状态, 周期 {period}ms")
    
    # 循环接收数据并验证
    print(f"\n开始接收数据 {TEST_DURATION} 秒...")
    start_time = time.time()
    sample_count = 0
    error_count = 0
    
    while time.time() - start_time < TEST_DURATION:
        # 获取机器人状态数据
        state_pkg = robot.robot_state_pkg
        
        try:
            # 验证基本状态字段
            print(f"\n--- 样本 #{sample_count + 1} ---")
            print(f"程序状态 (program_state): {state_pkg.program_state}")
            print(f"机器人状态 (robot_state): {state_pkg.robot_state}")
            print(f"主代码 (main_code): {state_pkg.main_code}")
            print(f"子代码 (sub_code): {state_pkg.sub_code}")
            print(f"机器人模式 (robot_mode): {state_pkg.robot_mode}")
            
            # 验证关节位置（6个关节）
            joint_pos = [state_pkg.jt_cur_pos[i] for i in range(6)]
            print(f"关节位置 (jt_cur_pos): {joint_pos}")
            
            # 验证工具位置（笛卡尔坐标）
            tool_pos = [state_pkg.tl_cur_pos[i] for i in range(6)]
            print(f"工具位置 (tl_cur_pos): {tool_pos}")
            
            # 验证工具和用户坐标系ID
            print(f"工具ID (tool): {state_pkg.tool}")
            print(f"用户坐标系ID (user): {state_pkg.user}")
            
            # 验证驱动器温度
            driver_temp = [state_pkg.jointDriverTemperature[i] for i in range(6)]
            print(f"驱动器温度 (jointDriverTemperature): {driver_temp}")
            
            # 数据合理性检查
            if state_pkg.program_state < 0 or state_pkg.program_state > 10:
                print(f"⚠ 程序状态值异常: {state_pkg.program_state}")
                error_count += 1
            
            # 检查关节位置范围（合理范围检查）
            for i, pos in enumerate(joint_pos):
                if abs(pos) > 360:  # 关节角度超过360度可能异常
                    print(f"⚠ 关节{i}位置值异常: {pos}")
                    error_count += 1
            
            sample_count += 1
            time.sleep(period / 1000.0)  # 按周期采样
            
        except Exception as e:
            print(f"✗ 数据解析错误: {e}")
            error_count += 1
            time.sleep(0.1)
    
    print(f"\n--- 测试结果 ---")
    print(f"✓ 成功接收样本数: {sample_count}")
    print(f"✓ 异常数据计数: {error_count}")
    print(f"✓ 数据有效率: {(sample_count - error_count) / max(sample_count, 1) * 100:.1f}%")
    
    return error_count == 0


def test3_custom_config(robot):
    """
    测试3: 自定义配置测试
    测试添加/删除状态，以及自定义周期
    """
    print_separator("测试3: 自定义配置测试")
    
    # 3.1 设置自定义周期
    print("\n3.1 测试设置自定义周期 (50ms)...")
    rtn = robot.CNDESetPeriod(50)
    if rtn == 0:
        print("✓ 周期设置成功")
        config = robot.CNDEGetConfig()
        if config:
            print(f"  新周期: {config[1]}ms")
    else:
        print(f"✗ 周期设置失败，错误码: {rtn}")
    
    # 等待配置生效
    time.sleep(0.5)
    
    # 3.2 添加单个状态
    print("\n3.2 测试添加单个状态 (ServoJCmdNum)...")
    rtn = robot.CNDEAddState(RobotState.ServoJCmdNum)
    if rtn == 0:
        print("✓ 状态添加成功")
    elif rtn == -4:
        print("⚠ 状态已存在（正常）")
    else:
        print(f"✗ 状态添加失败，错误码: {rtn}")
    
    # 3.3 删除单个状态
    print("\n3.3 测试删除单个状态 (LastServoTarget)...")
    rtn = robot.CNDEDeleteState(RobotState.LastServoTarget)
    if rtn == 0:
        print("✓ 状态删除成功")
    else:
        print(f"✗ 状态删除失败，错误码: {rtn}")
    
    # 获取更新后的配置
    config = robot.CNDEGetConfig()
    if config:
        states, period = config
        print(f"\n当前配置: {len(states)} 个状态, 周期 {period}ms")
        print(f"状态列表: {[s.name for s in states]}")
    
    # 3.4 恢复默认配置
    print("\n3.4 测试恢复默认配置...")
    from fairino.Robot import DEFAULT_CNDE_STATES
    rtn = robot.CNDESetStateConfig(DEFAULT_CNDE_STATES, 500)
    if rtn == 0:
        print("✓ 默认配置恢复成功")
    else:
        print(f"✗ 配置恢复失败，错误码: {rtn}")
    
    # 3.5 测试自定义完整配置
    print("\n3.5 测试自定义完整配置...")
    # 创建一个只包含基本状态的小配置
    custom_states = [
        RobotState.ProgramState,
        RobotState.RobotState,
        RobotState.JointCurPos,
        RobotState.ToolCurPos,
        RobotState.Tool,
        RobotState.User,
    ]
    rtn = robot.CNDESetStateConfig(custom_states, 200)
    if rtn == 0:
        print(f"✓ 自定义配置设置成功 (6个状态, 200ms周期)")
    else:
        print(f"✗ 自定义配置设置失败，错误码: {rtn}")
    
    # 接收几帧数据验证
    print("\n接收3帧数据验证...")
    for i in range(3):
        time.sleep(0.2)
        state_pkg = robot.robot_state_pkg
        print(f"  帧{i+1}: program_state={state_pkg.program_state}, "
              f"jt_cur_pos[0]={state_pkg.jt_cur_pos[0]:.2f}, "
              f"tool={state_pkg.tool}")
    
    # 恢复默认配置
    print("\n恢复默认配置...")
    from fairino.Robot import DEFAULT_CNDE_STATES
    robot.CNDESetStateConfig(DEFAULT_CNDE_STATES, 500)
    
    return True


def test4_stop_start(robot):
    """
    测试4: 停止/开始指令测试
    """
    print_separator("测试4: CNDE停止/开始指令测试")
    
    # 停止数据发送
    print("\n发送停止指令...")
    rtn = robot.CNDESendStop()
    if rtn == 0:
        print("✓ 停止指令发送成功")
    else:
        print(f"✗ 停止指令发送失败，错误码: {rtn}")
    
    time.sleep(1)
    
    # 开始数据发送
    print("\n发送开始指令...")
    rtn = robot.CNDESendStart()
    if rtn == 0:
        print("✓ 开始指令发送成功")
    else:
        print(f"✗ 开始指令发送失败，错误码: {rtn}")
    
    # 验证数据恢复
    print("\n验证数据恢复...")
    time.sleep(0.5)
    state_pkg = robot.robot_state_pkg
    print(f"  program_state: {state_pkg.program_state}")
    print(f"  robot_state: {state_pkg.robot_state}")
    
    return True


def test5_extended_states(robot):
    """
    测试5: 扩展状态测试（LastServoTarget之后的状态）
    """
    print_separator("测试5: 扩展状态测试")
    
    # 添加一些扩展状态
    extended_states = [
        RobotState.ProgramState,
        RobotState.RobotState,
        RobotState.JointCurPos,
        RobotState.ServoJCmdNum,        # 扩展状态
        RobotState.TargetJointPos,       # 扩展状态
        RobotState.LuaLineNum,            # 扩展状态
    ]
    
    print("设置包含扩展状态的配置...")
    rtn = robot.CNDESetStateConfig(extended_states, 100)
    if rtn != 0:
        print(f"✗ 扩展配置设置失败，错误码: {rtn}")
        return False
    
    print("✓ 扩展配置设置成功")
    print("\n接收5帧数据...")
    
    for i in range(5):
        time.sleep(0.1)
        state_pkg = robot.robot_state_pkg
        print(f"  帧{i+1}: prog={state_pkg.program_state}, "
              f"servoJCmdNum={state_pkg.servoJCmdNum}, "
              f"luaLineNum={state_pkg.luaLineNum}")
    
    # 恢复默认配置
    robot.CNDESetStateConfig(Robot.DEFAULT_CNDE_STATES, 500)
    
    return True


def main():
    """主测试流程"""
    print("\n" + "#" * 60)
    print("#" + " " * 58 + "#")
    print("#" + "     CNDE (Custom Network Data Export) 测试套件     ".center(58) + "#")
    print("#" + " " * 58 + "#")
    print("#" * 60)
    
    try:
        # 测试1: 自动连接
        robot = test1_auto_connection()
        
        # 测试2: 默认配置数据验证
        test2_default_config_data_verify(robot)
        
        # 测试3: 自定义配置
        test3_custom_config(robot)
        
        # 测试4: 停止/开始
        test4_stop_start(robot)
        
        # 测试5: 扩展状态
        test5_extended_states(robot)
        
        print_separator("所有测试完成")
        print("提示: CNDE连接在RPC实例销毁时会自动断开")
        
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    except Exception as e:
        print(f"\n\n测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n测试结束")


if __name__ == "__main__":
    main()
