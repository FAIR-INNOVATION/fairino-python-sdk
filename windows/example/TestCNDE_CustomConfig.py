"""
CNDE 自定义配置测试脚本（简化版）

使用方式：
1. 调用 SetRobotRealtimeStateConfig() 设置自定义配置（在RPC连接前）
2. 创建 Robot.RPC()，自动使用自定义配置建立CNDE连接
3. 接收数据验证

示例：
    from fairino.Robot import RobotState, SetRobotRealtimeStateConfig
    
    # 设置自定义配置（RPC连接前）
    SetRobotRealtimeStateConfig([
        RobotState.ProgramState,
        RobotState.RobotState,
        RobotState.JointCurPos,
    ], 100)
    
    # 创建RPC连接，自动使用上面的配置
    robot = Robot.RPC('192.168.58.2')
"""

import sys
import time

sys.path.append('..')
from fairino import Robot
from fairino.Robot import RobotState, SetRobotRealtimeStateConfig


def print_pkg_data(pkg, prefix="  "):
    """打印RobotStatePkg全部数据"""
    # 帧头信息
    print(f"{prefix}===== 帧头信息 =====")
    print(f"{prefix}frame_head: {pkg.frame_head} (0x{pkg.frame_head:04X})")
    print(f"{prefix}frame_cnt: {pkg.frame_cnt}")
    print(f"{prefix}data_len: {pkg.data_len}")
    print(f"{prefix}program_state: {pkg.program_state}")
    print(f"{prefix}robot_state: {pkg.robot_state}")
    print(f"{prefix}main_code: {pkg.main_code}, sub_code: {pkg.sub_code}")
    print(f"{prefix}robot_mode: {pkg.robot_mode}")
    
    # 机器人时间（新增）
    print(f"{prefix}===== 机器人时间 =====")
    print(f"{prefix}robot_time: {pkg.year}-{pkg.mouth:02d}-{pkg.day:02d} {pkg.hour:02d}:{pkg.minute:02d}:{pkg.second:02d}.{pkg.millisecond}")
    
    # # 关节位置和速度
    # print(f"{prefix}===== 关节位置 (deg) =====")
    # print(f"{prefix}jt_cur_pos: [{', '.join([f'{x:.3f}' for x in pkg.jt_cur_pos])}]")
    # print(f"{prefix}tl_cur_pos: [{', '.join([f'{x:.3f}' for x in pkg.tl_cur_pos])}]")
    # print(f"{prefix}flange_cur_pos: [{', '.join([f'{x:.3f}' for x in pkg.flange_cur_pos])}]")
    # print(f"{prefix}actual_qd: [{', '.join([f'{x:.3f}' for x in pkg.actual_qd])}]")
    # print(f"{prefix}actual_qdd: [{', '.join([f'{x:.3f}' for x in pkg.actual_qdd])}]")
    
    # # TCP速度
    # print(f"{prefix}===== TCP速度 =====")
    # print(f"{prefix}target_TCP_CmpSpeed: [{', '.join([f'{x:.3f}' for x in pkg.target_TCP_CmpSpeed])}]")
    # print(f"{prefix}target_TCP_Speed: [{', '.join([f'{x:.3f}' for x in pkg.target_TCP_Speed])}]")
    # print(f"{prefix}actual_TCP_CmpSpeed: [{', '.join([f'{x:.3f}' for x in pkg.actual_TCP_CmpSpeed])}]")
    # print(f"{prefix}actual_TCP_Speed: [{', '.join([f'{x:.3f}' for x in pkg.actual_TCP_Speed])}]")
    # print(f"{prefix}jt_cur_tor: [{', '.join([f'{x:.3f}' for x in pkg.jt_cur_tor])}]")
    
    # # 工具和用户
    # print(f"{prefix}===== 工具/用户 =====")
    # print(f"{prefix}tool: {pkg.tool}, user: {pkg.user}")
    
    # # 数字IO
    # print(f"{prefix}===== 数字IO =====")
    # print(f"{prefix}cl_dgt_output_h: {pkg.cl_dgt_output_h}, cl_dgt_output_l: {pkg.cl_dgt_output_l}")
    # print(f"{prefix}tl_dgt_output_l: {pkg.tl_dgt_output_l}")
    # print(f"{prefix}cl_dgt_input_h: {pkg.cl_dgt_input_h}, cl_dgt_input_l: {pkg.cl_dgt_input_l}")
    # print(f"{prefix}tl_dgt_input_l: {pkg.tl_dgt_input_l}")
    
    # # 模拟量IO
    # print(f"{prefix}===== 模拟量IO =====")
    # print(f"{prefix}cl_analog_input: [{pkg.cl_analog_input[0]}, {pkg.cl_analog_input[1]}]")
    # print(f"{prefix}tl_anglog_input: {pkg.tl_anglog_input}")
    # print(f"{prefix}cl_analog_output: [{pkg.cl_analog_output[0]}, {pkg.cl_analog_output[1]}]")
    # print(f"{prefix}tl_analog_output: {pkg.tl_analog_output}")
    
    # # 力矩传感器
    # print(f"{prefix}===== 力矩传感器 =====")
    # print(f"{prefix}ft_sensor_active: {pkg.ft_sensor_active}")
    # print(f"{prefix}ft_sensor_data: [{', '.join([f'{x:.3f}' for x in pkg.ft_sensor_data])}]")
    
    # # 状态信号
    # print(f"{prefix}===== 状态信号 =====")
    # print(f"{prefix}EmergencyStop: {pkg.EmergencyStop}")
    # print(f"{prefix}motion_done: {pkg.motion_done}")
    # print(f"{prefix}gripper_motiondone: {pkg.gripper_motiondone}")
    # print(f"{prefix}mc_queue_len: {pkg.mc_queue_len}")
    # print(f"{prefix}collisionState: {pkg.collisionState}")
    # print(f"{prefix}trajectory_pnum: {pkg.trajectory_pnum}")
    
    # # 夹爪
    # print(f"{prefix}===== 夹爪 =====")
    # print(f"{prefix}gripper_active: {pkg.gripper_active}")
    # print(f"{prefix}gripper_position: {pkg.gripper_position}")
    # print(f"{prefix}gripper_speed: {pkg.gripper_speed}")
    # print(f"{prefix}gripper_current: {pkg.gripper_current}")
    # print(f"{prefix}gripper_temp: {pkg.gripper_temp}, voltage: {pkg.gripper_voltage}")
    
    # # 旋转夹爪
    # print(f"{prefix}gripperRotNum: {pkg.gripperRotNum}")
    # print(f"{prefix}gripperRotSpeed: {pkg.gripperRotSpeed}")
    # print(f"{prefix}gripperRotTorque: {pkg.gripperRotTorque}")
    
    # # 扩展轴
    # print(f"{prefix}===== 扩展轴 =====")
    # print(f"{prefix}rbtEnableState: {pkg.rbtEnableState}")
    # print(f"{prefix}jointDriverTorque[0]: {pkg.jointDriverTorque[0]:.3f}")
    # print(f"{prefix}jointDriverTemperature[0]: {pkg.jointDriverTemperature[0]:.3f}")
    
    # # 坐标系
    # print(f"{prefix}===== 坐标系 =====")
    # print(f"{prefix}toolCoord: [{', '.join([f'{x:.3f}' for x in pkg.toolCoord])}]")
    # print(f"{prefix}wobjCoord: [{', '.join([f'{x:.3f}' for x in pkg.wobjCoord])}]")
    
    # # 负载
    # print(f"{prefix}===== 负载 =====")
    # print(f"{prefix}load: {pkg.load}")
    # print(f"{prefix}loadCog: [{', '.join([f'{x:.3f}' for x in pkg.loadCog])}]")
    
    # # 目标关节
    # print(f"{prefix}===== 目标关节 =====")
    # print(f"{prefix}targetJointPos: [{', '.join([f'{x:.3f}' for x in pkg.targetJointPos])}]")
    # print(f"{prefix}targetJointVel: [{', '.join([f'{x:.3f}' for x in pkg.targetJointVel])}]")
    # print(f"{prefix}targetJointAcc: [{', '.join([f'{x:.3f}' for x in pkg.targetJointAcc])}]")
    
    # # 其他
    # print(f"{prefix}===== 其他 =====")
    # print(f"{prefix}speedScaleManual: {pkg.speedScaleManual}")
    # print(f"{prefix}speedScaleAuto: {pkg.speedScaleAuto}")
    # print(f"{prefix}luaLineNum: {pkg.luaLineNum}")
    # print(f"{prefix}abnomalStop: {pkg.abnomalStop}")
    # print(f"{prefix}strangePosFlag: {pkg.strangePosFlag}")
    # print(f"{prefix}alarm: {pkg.alarm}")
    # print(f"{prefix}check_sum: {pkg.check_sum}")
    print(f"{prefix}socket_conn_timeout: {pkg.socket_conn_timeout}")
    print(f"{prefix}socket_read_timeout: {pkg.socket_read_timeout}")
    print(f"{prefix}ts_web_state_com_err: {pkg.ts_web_state_com_err}")

def main():
    print("=" * 60)
    print("CNDE 自定义配置测试（简化接口）")
    print("=" * 60)
    
    # 第1步：在RPC连接前配置自定义CNDE配置（一行代码搞定）
    print("\n1. 设置自定义CNDE配置（RPC连接前）...")
    
    custom_states = [
        RobotState.ProgramState,
        RobotState.RobotState,
        RobotState.MainCode,
        RobotState.SubCode,
        RobotState.RobotMode,
        # RobotState.JointCurPos,
        # RobotState.ToolCurPos,
        # RobotState.FlangeCurPos,
        # RobotState.ActualJointVel,
        # RobotState.ActualJointAcc,
        # RobotState.TargetTCPCmpSpeed,
        # RobotState.TargetTCPSpeed,
        # RobotState.ActualTCPCmpSpeed,
        # RobotState.ActualTCPSpeed,
        # RobotState.ActualJointTorque,
        # RobotState.Tool,
        # RobotState.User,
        # RobotState.ClDgtOutputH,
        # RobotState.ClDgtOutputL,
        # RobotState.TlDgtOutputL,
        # RobotState.ClDgtInputH,
        # RobotState.ClDgtInputL,
        # RobotState.TlDgtInputL,
        # RobotState.ClAnalogInput,
        # RobotState.TlAnglogInput,
        # RobotState.FtSensorRawData,
        # RobotState.FtSensorData,
        # RobotState.FtSensorActive,
        # RobotState.EmergencyStop,
        # RobotState.MotionDone,
        # RobotState.GripperMotiondone,
        # RobotState.McQueueLen,
        # RobotState.CollisionState,
        # RobotState.TrajectoryPnum,
        # RobotState.SafetyStop0State,
        # RobotState.SafetyStop1State,
        # RobotState.GripperFaultId,
        # RobotState.GripperFault,
        # RobotState.GripperActive,
        # RobotState.GripperPosition,
        # RobotState.GripperSpeed,
        # RobotState.GripperCurrent,
        # RobotState.GripperTemp,
        # RobotState.GripperVoltage,
        # RobotState.AuxState,
        # RobotState.ExtAxisStatus,
        # RobotState.ExtDIState,
        # RobotState.ExtDOState,
        # RobotState.ExtAIState,
        # RobotState.ExtAOState,
        # RobotState.RbtEnableState,
        # RobotState.JointDriverTorque,
        # RobotState.JointDriverTemperature,
        # RobotState.SoftwareUpgradeState,
        # RobotState.EndLuaErrCode,
        # RobotState.ClAnalogOutput,
        # RobotState.TlAnalogOutput,
        # RobotState.GripperRotNum,
        # RobotState.GripperRotSpeed,
        # RobotState.GripperRotTorque,
        # RobotState.WeldingBreakOffState,
        # RobotState.TargetJointTorque,
        # RobotState.SmartToolState,
        # RobotState.WideVoltageCtrlBoxTemp,
        # RobotState.WideVoltageCtrlBoxFanCurrent,
        # RobotState.ToolCoord,
        # RobotState.WobjCoord,
        # RobotState.ExtoolCoord,
        # RobotState.ExAxisCoord,
        # RobotState.Load,
        # RobotState.LoadCog,
        # RobotState.LastServoTarget,
        RobotState.RobotTime,
        RobotState.SocketConnTimeout,
        RobotState.SocketReadTimeout,
        RobotState.TsWebStateComErr,
    ]
    
    # 一行代码设置默认配置
    rtn = SetRobotRealtimeStateConfig(custom_states, 100)
    if rtn != 0:
        print(f"   ✗ 配置失败: {rtn}")
        return
    
    print(f"   配置状态: {[s.name for s in custom_states]}")
    
    # 第2步：创建RPC连接（自动使用上面设置的自定义配置）
    print("\n2. 连接机器人（RPC自动使用自定义配置连接CNDE）...")
    
    robot = Robot.RPC('192.168.58.2')
    
    # 验证当前配置
    current_config = robot.CNDEGetConfig()
    if current_config:
        states, period = current_config
        print(f"   ✓ CNDE当前配置: {len(states)} 个状态, {period}ms")
    
    # 第3步：接收数据验证（10次完整数据打印）
    print("\n3. 接收状态数据（10次完整数据打印）...")
    
    for i in range(10):
        pkg = robot.robot_state_pkg
        print(f"\n{'='*60}")
        print(f"第 {i+1}/10 帧数据 (frame_cnt={pkg.frame_cnt})")
        print(f"{'='*60}")
        print_pkg_data(pkg)
        time.sleep(0.2)
    
    print(f"\n   ✓ 共接收 10 帧数据")
    
    print("\n" + "=" * 60)
    print("测试完成！")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
