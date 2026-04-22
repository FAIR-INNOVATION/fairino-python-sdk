import enum
import xmlrpc.client
import os
import socket
import hashlib
import time
from datetime import datetime
import logging
from functools import wraps
from logging.handlers import RotatingFileHandler
from queue import Queue
import threading
import struct
import sys
import ctypes
from ctypes import *
from typing import Optional
from typing import Callable, Optional, List, Tuple, Dict
from dataclasses import dataclass
# from Cython.Compiler.Options import error_on_unknown_names

# ==================== CNDE帧类型定义 ====================
CNDE_FRAME_HEAD = 0x5A5A          # 帧头
CNDE_FRAME_TAIL = 0xA5A5          # 帧尾
CNDE_FRAME_TYPE_INPUT_CONFIG = 0x00      # 输入配置帧（控制配置）Client->Robot
CNDE_FRAME_TYPE_OUTPUT_CONFIG = 0x01     # 输出配置帧（状态配置）Client->Robot
CNDE_FRAME_TYPE_OUTPUT_START = 0x02      # CNDE输出启动 Client->Robot
CNDE_FRAME_TYPE_OUTPUT_STOP = 0x03       # CNDE输出停止 Client->Robot
CNDE_FRAME_TYPE_OUTPUT_DATA = 0x04       # 输出数据帧（状态数据）Robot->Client
CNDE_FRAME_TYPE_INPUT_DATA = 0x05        # 输入数据帧（控制数据）Client->Robot
CNDE_FRAME_TYPE_MESSAGE = 0x06           # 字符提示消息 Client<->Robot
CNDE_FRAME_TYPE_SET_VERSION = 0x07       # 设置机器人CNDE协议版本号 Client->Robot
CNDE_FRAME_TYPE_GET_VERSION = 0x08       # 获取机器人软固件版本 Client<->Robot

# 最大数据长度
CNDE_MAX_PKG_SIZE = 4096


# ==================== CNDE数据包结构 ====================
class CNDE_PKG:
    """CNDE数据包结构（表2-1）"""

    def __init__(self):
        self.head = CNDE_FRAME_HEAD  # 帧头 2字节
        self.count = 0  # 帧计数 1字节 (0~255)
        self.type = 0  # 帧类型 1字节 (0~8)
        self.len = 0  # 数据长度 2字节
        self.data = b""  # 数据内容
        self.tail = CNDE_FRAME_TAIL  # 帧尾 2字节

    def Clear(self):
        self.head = CNDE_FRAME_HEAD
        self.count = 0
        self.type = 0
        self.len = 0
        self.data = b""
        self.tail = CNDE_FRAME_TAIL


# ==================== CNDE帧处理函数 ====================
def Int16ToByte(val: int) -> bytes:
    """将16位整数转换为2字节小端序bytes"""
    return bytes([val & 0x00FF, (val & 0xFF00) >> 8])


def ByteToInt16(data: bytes) -> int:
    """将2字节小端序bytes转换为16位整数"""
    if len(data) < 2:
        return 0
    return (data[1] << 8) + data[0]


def CNDEPkgToFrame(pkg: CNDE_PKG) -> bytes:
    """将CNDE_PKG打包为帧字节流"""
    frame = bytearray()
    # 帧头
    frame.extend(Int16ToByte(CNDE_FRAME_HEAD))
    # 帧计数
    frame.append(pkg.count)
    # 帧类型
    frame.append(pkg.type)
    # 数据长度
    frame.extend(Int16ToByte(pkg.len))
    # 数据内容
    if pkg.data:
        frame.extend(pkg.data)
    # 帧尾
    frame.extend(Int16ToByte(CNDE_FRAME_TAIL))
    return bytes(frame)


def FrameToCNDEPkg(frame: bytes, pkg: CNDE_PKG) -> int:
    """
    将帧字节流解析为CNDE_PKG
    Returns:
        0: 成功
        -1: 帧长度不足
        -2: 数据长度不匹配
        -3: 帧头错误
        -4: 帧尾错误
    """
    if len(frame) < 8:
        return -1

    # 解析帧头
    head = ByteToInt16(frame[0:2])
    if head != CNDE_FRAME_HEAD:
        return -3

    # 帧计数
    pkg.count = frame[2]
    # 帧类型
    pkg.type = frame[3]
    # 数据长度
    pkg.len = ByteToInt16(frame[4:6])

    # 验证数据长度
    if pkg.len != len(frame) - 8:
        return -2

    # 提取数据
    if pkg.len > 0:
        pkg.data = frame[6:6 + pkg.len]
    else:
        pkg.data = b""

    # 验证帧尾
    tail_pos = 6 + pkg.len
    if len(frame) < tail_pos + 2:
        return -4
    tail = ByteToInt16(frame[tail_pos:tail_pos + 2])
    if tail != CNDE_FRAME_TAIL:
        return -4

    pkg.head = head
    pkg.tail = tail
    return 0


is_init =False
class ROBOT_AUX_STATE(Structure):
    _pack_ = 1
    _fields_ = [
        ("servoId", c_uint8),         # 伺服驱动器ID号
        ("servoErrCode", c_int),     # 伺服驱动器故障码
        ("servoState", c_int),       # 伺服驱动器状态
        ("servoPos", c_double),      # 伺服当前位置
        ("servoVel", c_float),       # 伺服当前速度
        ("servoTorque", c_float),    # 伺服当前转矩
    ]

class EXT_AXIS_STATUS(Structure):
    _pack_ = 1
    _fields_ = [
        ("pos", c_double),        # 扩展轴位置
        ("vel", c_double),        # 扩展轴速度
        ("errorCode", c_int),     # 扩展轴故障码
        ("ready", c_uint8),        # 伺服准备好
        ("inPos", c_uint8),        # 伺服到位
        ("alarm", c_uint8),        # 伺服报警
        ("flerr", c_uint8),        # 跟随误差
        ("nlimit", c_uint8),       # 到负限位
        ("pLimit", c_uint8),       # 到正限位
        ("mdbsOffLine", c_uint8),  # 驱动器485总线掉线
        ("mdbsTimeout", c_uint8),  # 控制卡与控制箱485通信超时
        ("homingStatus", c_uint8), # 扩展轴回零状态
    ]

class WELDING_BREAKOFF_STATE(Structure):
    _pack_ = 1
    _fields_ = [
        ("breakOffState", c_uint8),        # 焊接中断状态
        ("weldArcState", c_uint8),        # 焊接电弧中断状态
    ]

# ==================== 完整机器人状态结构体 ====================
class RobotStatePkg(Structure):
    """
    机器人状态反馈数据包
    """
    _pack_ = 1
    _fields_ = [
        # 帧头信息
        ("frame_head", c_uint16),           # 帧头，约定为0x5A5A
        ("frame_cnt", c_uint8),             # 帧计数，循环计数0-255
        ("data_len", c_uint16),             # 数据内容的长度
        ("program_state", c_uint8),         # 程序运行状态，1-停止；2-运行；3-暂停
        ("robot_state", c_uint8),             # 机器人运动状态，1-停止；2-运行；3-暂停；4-拖动
        ("main_code", c_int),               # 主故障码
        ("sub_code", c_int),                # 子故障码
        ("robot_mode", c_uint8),            # 机器人模式，1-手动模式；0-自动模式

        # 关节位置和速度
        ("jt_cur_pos", c_double * 6),       # 6个轴当前关节位置，单位deg
        ("tl_cur_pos", c_double * 6),       # 工具当前位置 [x,y,z,rx,ry,rz]
        ("flange_cur_pos", c_double * 6),   # 末端法兰当前位置 [x,y,z,rx,ry,rz]
        ("actual_qd", c_double * 6),        # 当前6个关节速度，单位deg/s
        ("actual_qdd", c_double * 6),       # 当前6个关节加速度，单位deg/s^2
        ("target_TCP_CmpSpeed", c_double * 2),  # TCP合成指令速度[位置mm/s,姿态deg/s]
        ("target_TCP_Speed", c_double * 6), # TCP指令速度[x,y,z,rx,ry,rz]
        ("actual_TCP_CmpSpeed", c_double * 2),  # TCP合成实际速度[位置mm/s,姿态deg/s]
        ("actual_TCP_Speed", c_double * 6), # TCP实际速度[x,y,z,rx,ry,rz]
        ("jt_cur_tor", c_double * 6),       # 6个轴当前扭矩，单位N·m

        # 工具和用户坐标系
        ("tool", c_int),                    # 应用的工具坐标系编号
        ("user", c_int),                    # 应用的工件坐标系编号

        # 数字IO
        ("cl_dgt_output_h", c_uint8),       # 控制箱数字量IO输出15-8
        ("cl_dgt_output_l", c_uint8),       # 控制箱数字量IO输出7-0
        ("tl_dgt_output_l", c_uint8),       # 工具数字量IO输出7-0，仅bit0-bit1有效
        ("cl_dgt_input_h", c_uint8),        # 控制箱数字量IO输入15-8
        ("cl_dgt_input_l", c_uint8),        # 控制箱数字量IO输入7-0
        ("tl_dgt_input_l", c_uint8),        # 工具数字量IO输入7-0，仅bit0-bit1有效

        # 模拟量IO 
        ("cl_analog_input", c_uint16 * 2),  # 控制箱模拟量输入[0],[1]
        ("tl_anglog_input", c_uint16),      # 工具模拟量输入

        # 力矩传感器
        ("ft_sensor_raw_data", c_double * 6),   # 力矩传感器原始数据
        ("ft_sensor_data", c_double * 6),      # 力矩传感器数据
        ("ft_sensor_active", c_uint8),          # 力矩传感器激活状态

        # 状态信号
        ("EmergencyStop", c_uint8),         # 急停标志，0-急停未按下，1-急停按下
        ("motion_done", c_int),             # 运动到位信号，1-到位，0-未到位
        ("gripper_motiondone", c_uint8),    # 夹爪运动完成信号，1-完成，0-未完成
        ("mc_queue_len", c_int),            # 运动指令队列长度
        ("collisionState", c_uint8),        # 碰撞检测，1-碰撞，0-无碰撞
        ("trajectory_pnum", c_int),         # 轨迹点编号
        ("safety_stop0_state", c_uint8),    # 安全停止信号SI0
        ("safety_stop1_state", c_uint8),    # 安全停止信号SI1

        # 夹爪信息
        ("gripper_fault_id", c_uint8),      # 错误夹爪号
        ("gripper_fault", c_uint16),        # 夹爪故障
        ("gripper_active", c_uint16),      # 夹爪激活状态
        ("gripper_position", c_uint8),      # 夹爪位置
        ("gripper_speed", c_int8),          # 夹爪速度
        ("gripper_current", c_int8),        # 夹爪电流
        ("gripper_temp", c_int),            # 夹爪温度
        ("gripper_voltage", c_int),         # 夹爪电压

        # 扩展轴状态
        ("aux_axis_state", ROBOT_AUX_STATE * 25),    # 485扩展轴状态 (25个)
        ("extAxisStatus", EXT_AXIS_STATUS * 4), # UDP扩展轴状态 (4个)

        # 扩展IO状态
        ("extDIState", c_uint16 * 8),       # 扩展DI输入
        ("extDOState", c_uint16 * 8),       # 扩展DO输出
        ("extAIState", c_uint16 * 4),        # 扩展AI输入
        ("extAOState", c_uint16 * 4),        # 扩展AO输出

        # 机器人和关节状态
        ("rbtEnableState", c_int),                  # 机器人使能状态
        ("jointDriverTorque", c_double * 6),        # 机器人关节驱动器扭矩
        ("jointDriverTemperature", c_double * 6),   # 机器人关节驱动器温度

        # 机器人时间
        #("robotTime", c_int * 7),             # 机器人系统时间 [year,month,day,hour,min,sec,ms]
        ("year", ctypes.c_uint16),  # 年
        ("mouth", ctypes.c_uint8),  # 月
        ("day", ctypes.c_uint8),  # 日
        ("hour", ctypes.c_uint8),  # 小时
        ("minute", ctypes.c_uint8),  # 分
        ("second", ctypes.c_uint8),  # 秒
        ("millisecond", ctypes.c_uint16),  # 毫秒

        ("softwareUpgradeState", c_int),      # 机器人软件升级状态
        ("endLuaErrCode", c_uint16),          # 末端LUA运行状态

        # 模拟量输出
        ("cl_analog_output", c_uint16 * 2), # 控制箱模拟量输出[0],[1]
        ("tl_analog_output", c_uint16),       # 工具模拟量输出

        # 旋转夹爪
        ("gripperRotNum", c_float),         # 旋转夹爪当前旋转圈数
        ("gripperRotSpeed", c_uint8),       # 旋转夹爪当前旋转速度百分比
        ("gripperRotTorque", c_uint8),      # 旋转夹爪当前旋转力矩百分比

        # 焊接中断状态 - 使用结构体
        ("weldingBreakOffState", WELDING_BREAKOFF_STATE),  # 焊接中断状态

        # 目标关节扭矩
        ("jt_tgt_tor", c_double * 6),       # 关节指令力矩

        ("smartToolState", c_int),          # SmartTool手柄按钮状态
        ("wideVoltageCtrlBoxTemp", c_float),        # 宽电压控制箱温度
        ("wideVoltageCtrlBoxFanCurrent", c_uint16), # 宽电压控制箱风扇电流(mA)

        # 坐标系数值
        ("toolCoord", c_double * 6),        # 当前工具坐标系数值；x,y,z,rx,ry,rz
        ("wobjCoord", c_double * 6),        # 当前工件坐标系数值；x,y,z,rx,ry,rz
        ("extoolCoord", c_double * 6),      # 当前外部工具坐标系数值；x,y,z,rx,ry,rz
        ("exAxisCoord", c_double * 6),      # 当前扩展轴坐标系数值；x,y,z,rx,ry,rz

        # 负载
        ("load", c_double),                 # 负载质量
        ("loadCog", c_double * 3),            # 负载质心

        # 伺服指令
        ("lastServoTarget", c_double * 6),  # 队列中最后一个ServoJ目标位置
        ("servoJCmdNum", c_int),            # servoJ指令计数

        # 目标关节数据
        ("targetJointPos", c_double * 6),   # 6个关节指令位置，单位°
        ("targetJointVel", c_double * 6),   # 6个关节指令速度，单位°/s
        ("targetJointAcc", c_double * 6),   # 6个关节指令加速度，单位°/s2
        ("targetJointCurrent", c_double * 6), # 6个关节指令电流，单位A
        ("actualJointCurrent", c_double * 6), # 6个关节当前电流，单位A
        ("actualTCPForce", c_double * 6),   # 机器人末端力矩Nm；x,y,z,rx,ry,rz
        ("targetTCPPos", c_double * 6),     # 机器人TCP指令位置mm；x,y,z,rx,ry,rz

        ("collisionLevel", c_uint8 * 6),    # 机器人碰撞等级
        ("speedScaleManual", c_double),     # 手动模式全局速度百分比
        ("speedScaleAuto", c_double),       # 自动模式全局速度百分比
        ("luaLineNum", c_int),              # 当前lua程序运行行号
        ("abnomalStop", c_uint8),           # 0-无异常；1-有异常
        ("currentLuaFileName", c_uint8 * 256),  # 当前运行lua程序名称
        ("programTotalLine", c_uint8),      # lua程序总行数
        ("safetyBoxSingal", c_uint8 * 6),   # 机器人按钮盒按钮状态

        # 焊接数据
        ("weldVoltage", c_double),          # 焊接电压 V
        ("weldCurrent", c_double),          # 焊接电流
        ("weldTrackVel", c_double),         # 焊缝跟踪速度 mm/s

        ("tpdException", c_uint8),            # TPD轨迹加载数量超限，0-未超限，1-超限
        ("alarmRebootRobot", c_uint8),      # 警告，1-松开急停按钮请断电重启控制箱，2-关节通讯异常请断电重启控制箱
        ("modbusMasterConnect", c_uint8),   # bit0-bit7位对应ModbusTCP的0-7主站连接状态
        ("modbusSlaveConnect", c_uint8),    # ModbusTCP从站连接状态
        ("btnBoxStopSignal", c_uint8),      # 按钮盒急停信号
        ("dragAlarm", c_uint8),             # 拖动警告
        ("safetyDoorAlarm", c_uint8),       # 安全门警告
        ("safetyPlaneAlarm", c_uint8),      # 进入安全墙警告
        ("motonAlarm", c_uint8),            # 运动警告
        ("interfaceAlarm", c_uint8),        # 进入干涉区警告
        ("udpCmdState", c_int),             # 20007端口UDP通讯连接状态
        ("weldReadyState", c_uint8),        # 焊机准备完成状态
        ("alarmCheckEmergStopBtn", c_uint8),    # 0-正常；1-通信异常，检查急停按钮是否松开
        ("tsTmCmdComError", c_uint8),       # 0-正常；1-扭矩指令通讯失败
        ("tsTmStateComError", c_uint8),     # 0-正常；1-扭矩状态通讯失败
        ("ctrlBoxError", c_int),            # 控制箱错误
        ("safetyDataState", c_uint8),       # 安全数据状态标志
        ("forceSensorErrState", c_uint8),   # 力传感器连接超时故障
        ("ctrlOpenLuaErrCode", c_uint8 * 4),  # 4个控制器外设协议错误码
        ("strangePosFlag", c_uint8),        # 当前处于奇异位姿标志
        ("alarm", c_uint8),                 # 警告
        ("driverAlarm", c_uint8),           # 驱动器报警轴号
        ("aliveSlaveNumError", c_uint8),    # 活动从站数量错误
        ("slaveComError", c_uint8 * 8),     # 从站错误状态
        ("cmdPointError", c_uint8),         # 指令点错误
        ("IOError", c_uint8),               # IO错误
        ("gripperError", c_uint8),          # 夹爪错误
        ("fileError", c_uint8),             # 文件错误
        ("paraError", c_uint8),             # 参数错误
        ("exaxisOutLimitError", c_uint8),   # 外部轴超出软限位错误
        ("driverComError", c_uint8 * 6),    # 与驱动器通信故障
        ("driverError", c_uint8),           # 驱动器通信故障轴号
        ("outSoftLimitError", c_uint8),     # 超出软限位故障
        ("axleGenComData", c_uint8 * 130),   # 轴通用通讯非周期数据
        ("socketConnTimeout", c_uint8),     # socket连接超时
        ("socketReadTimeout", c_uint8),     # socket读取超时
        ("tsWebStateComErr", c_uint8),      # TS_WEB状态通讯错误
        ("check_sum", c_uint16)          # 和校验
    ]

    # 兼容属性映射 
    _COMPAT_FIELDS = {
        # 关节和TCP位置/速度/加速度
        'actual_joint_pos': 'jt_cur_pos',
        'actual_TCP_pos': 'tl_cur_pos',
        'actual_flange_pos': 'flange_cur_pos',
        'actual_joint_vel': 'actual_qd',
        'actual_joint_acc': 'actual_qdd',
        'target_TCP_cmpvel': 'target_TCP_CmpSpeed',
        'target_TCP_vel': 'target_TCP_Speed',
        'actual_TCP_cmpvel': 'actual_TCP_CmpSpeed',
        'actual_TCP_vel': 'actual_TCP_Speed',
        'actual_joint_torque': 'jt_cur_tor',
        # 工具和用户
        'tool_id': 'tool',
        'wobj_id': 'user',
        # IO
        'cfg_DO_box': 'cl_dgt_output_h',
        'std_DO_box': 'cl_dgt_output_l',
        'cfg_DO_tool': 'tl_dgt_output_l',
        'cfg_DI_box': 'cl_dgt_input_h',
        'std_DI_box': 'cl_dgt_input_l',
        'cfg_DI_tool': 'tl_dgt_input_l',
        # 模拟量输入/输出（数组访问通过索引）
        'std_AI0_box': 'cl_analog_input',
        'std_AI1_box': 'cl_analog_input',
        'std_AI_tool': 'tl_anglog_input',
        'std_AO0_box': 'cl_analog_output',
        'std_AO1_box': 'cl_analog_output',
        'std_AO_tool': 'tl_analog_output',
        # 状态信号
        'emergency_stop': 'EmergencyStop',
        'gripper_motion_done': 'gripper_motiondone',
        'motion_queue_len': 'mc_queue_len',
        'collision_state': 'collisionState',
        # 扩展轴和IO
        'aux_axis_state': 'aux_axis_state',
        'exaxis_status': 'extAxisStatus',
        'ext_DI_state': 'extDIState',
        'ext_DO_state': 'extDOState',
        'ext_AI_state': 'extAIState',
        'ext_AO_state': 'extAOState',
        'rbt_enable_state': 'rbtEnableState',
        # 关节驱动器
        'joint_driver_torque': 'jointDriverTorque',
        'actual_joint_temp': 'jointDriverTemperature',
        'robot_time': 'robotTime',
        # 旋转夹爪
        'rotating_gripper_num': 'gripperRotNum',
        'rotating_gripper_speed': 'gripperRotSpeed',
        'rotating_gripper_tor': 'gripperRotTorque',
        # 焊接中断状态
        'weld_break_off_state': 'weldingBreakOffState',
        'weld_arc_state': 'weldingBreakOffState',
        # 其他
        'target_joint_torque': 'jt_tgt_tor',
        'smarttool_state': 'smartToolState',
        'tool_coord': 'toolCoord',
        'wobj_coord': 'wobjCoord',
        'exTool_coord': 'extoolCoord',
        'exAxis_coord': 'exAxisCoord',
        'payload': 'load',
        'pay_cog': 'loadCog',
        'last_servoJ_target': 'lastServoTarget',
        'servoJ_cmd_num': 'servoJCmdNum',
        'strange_pos_flag': 'strangePosFlag',
        'dr_alarm': 'driverAlarm',
        'socket_conn_timeout': 'socketConnTimeout',
        'socket_read_timeout': 'socketReadTimeout',
        'ts_web_state_com_err': 'tsWebStateComErr',
    }

    def __getattr__(self, name):
        """支持旧字段名访问"""
        if name in self._COMPAT_FIELDS:
            real_name = self._COMPAT_FIELDS[name]
            # 处理数组索引（如 std_AI0_box -> cl_analog_input[0]）
            if name == 'std_AI0_box':
                return getattr(self, real_name)[0]
            elif name == 'std_AI1_box':
                return getattr(self, real_name)[1]
            elif name == 'std_AO0_box':
                return getattr(self, real_name)[0]
            elif name == 'std_AO1_box':
                return getattr(self, real_name)[1]
            elif name == 'weld_break_off_state':
                return getattr(self, real_name).breakOffState
            elif name == 'weld_arc_state':
                return getattr(self, real_name).weldArcState
            return getattr(self, real_name)
        raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{name}'")

    def __setattr__(self, name, value):
        """支持旧字段名设置"""
        if name in self._COMPAT_FIELDS:
            real_name = self._COMPAT_FIELDS[name]
            # 处理数组索引
            if name == 'std_AI0_box':
                getattr(self, real_name)[0] = value
            elif name == 'std_AI1_box':
                getattr(self, real_name)[1] = value
            elif name == 'std_AO0_box':
                getattr(self, real_name)[0] = value
            elif name == 'std_AO1_box':
                getattr(self, real_name)[1] = value
            elif name == 'weld_break_off_state':
                getattr(self, real_name).breakOffState = value
            elif name == 'weld_arc_state':
                getattr(self, real_name).weldArcState = value
            else:
                super().__setattr__(real_name, value)
        else:
            super().__setattr__(name, value)


class BufferedFileHandler(RotatingFileHandler):
    def __init__(self, filename, mode='a', maxBytes=0, backupCount=0, encoding=None, delay=False):
        super().__init__(filename, mode, maxBytes, backupCount, encoding, delay)
        self.buffer = []

    def emit(self, record):
        # log_entry = self.format(record)  # 格式化日志记录
        # print(log_entry)  # 打印日志条目
        if RPC.log_output_model == 2:
            RPC.queue.put(record)
        else:
            self.buffer.append(record)
            if len(self.buffer) >= 50:
                for r in self.buffer:
                    super().emit(r)
                self.buffer = []


class LogWriterThread(threading.Thread):
    def __init__(self, queue, log_handler):
        super().__init__()
        self.queue = queue
        self.log_handler = log_handler
        self.daemon = True

    def run(self):
        while True:
            record = self.queue.get()
            if record is None:
                break
            log_entry = self.log_handler.format(record)
            self.log_handler.stream.write(log_entry + self.log_handler.terminator)
            self.log_handler.flush()


def calculate_file_md5(file_path):
    if not os.path.exists(file_path):
        raise ValueError(f"{file_path} 不存在")
    md5 = hashlib.md5()
    with open(file_path, 'rb') as file:
        while chunk := file.read(8192):  # Read in 8KB chunks
            md5.update(chunk)
    return md5.hexdigest()


def xmlrpc_timeout(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if RPC.is_connect == False:
            return -4
        else:
            result = func(self, *args, **kwargs)
            return result

    return wrapper


class RobotError:
    ERR_SUCCESS = 0

    ERR_TOO_MANY_STATES = -20                   #配置状态字段长度超限
    ERR_NEED_AT_LEAST_ONE_STATE = -19           #至少需要配置一个状态字段
    ERR_STATE_INVALID = -18                     #状态字段不存在
    ERR_STATE_ALREADY_EXISTS = -17              #状态字段重复配置
    ERR_SOCKET_RECV_FAILED=-16    #/* socket接收失败 */
    ERR_SOCKET_SEND_FAILED=-15    #/* socket发送失败 */
    ERR_FILE_OPEN_FAILED=-14    #/* 文件打开失败 */
    ERR_FILE_TOO_LARGE=-13    #/* 文件大小超限 */
    ERR_UPLOAD_FILE_ERROR=-12    #/* 上传文件异常 */
    ERR_FILE_NAME=-11    #/* 文件名称异常 */
    ERR_DOWN_LOAD_FILE_WRITE_FAILED=-10    #/* 下载文件写入失败 */
    ERR_DOWN_LOAD_FILE_CHECK_FAILED=-9     #/* 文件下载校验失败 */
    ERR_DOWN_LOAD_FILE_FAILED=-8     #/* 文件下载失败 */
    ERR_UPLOAD_FILE_NOT_FOUND=-7     #/* 上传文件存在 */
    ERR_SAVE_FILE_PATH_NOT_FOUND=-6     #/* 保存文件路径不存在 */
    ERR_NOT_FOUND_LUA_FILE = -5  # lua文件不存在
    ERR_RPC_ERROR = -4
    ERR_XMLRPC_COM_FAILED = -3
    ERR_SOCKET_COM_FAILED = -2
    ERR_OTHER = -1
    ERR_PARAM_VALUE=4                 #/* 参数值不在合理范围内 */

@dataclass
class UdpFrame:
    """UDP帧数据结构"""
    head: str = "/f/b"
    tail: str = "/b/f"
    count: int = 0
    cmd_id: int = 0
    content_len: int = 0
    content: str = ""

# 定义回调函数类型
UdpFrameCallback = Callable[[int, int, int, int, str], int]

def split_frame(data: str) -> List[str]:
    """
    从数据流中分割出完整的帧

    Args:
        data: 原始数据流（字符串）

    Returns:
        List[str]: 完整的帧列表
    """
    result = []
    pos = 0

    while pos < len(data):
        start = data.find("/f/b", pos)
        if start == -1:
            break

        end = data.find("/b/f", start)
        if end == -1:
            break

        # 提取完整帧
        result.append(data[start:end + 4])
        pos = end + 4

    return result


def unpack_frame(frame_str: str) -> UdpFrame:
    """
    解析帧字符串为UdpFrame结构
    格式: /f/bIII{count}III{cmd_id}III{content_len}III{content}III/b/f
    """
    frame = UdpFrame()

    # 1. 基本长度检查
    if len(frame_str) < 27:
        print(f"帧长度不足: {len(frame_str)} < 27")
        return frame

    # 2. 验证帧头帧尾
    if frame_str[:4] != "/f/b":
        print(f"帧头错误: {frame_str[:4]}")
        return frame

    if frame_str[-4:] != "/b/f":
        print(f"帧尾错误: {frame_str[-4:]}")
        return frame

    # 3. 去掉帧头帧尾
    data = frame_str[4:-4]

    # 4. 按"III"分割
    parts = data.split("III")

    # 格式应该是: ["", count, cmd_id, content_len, content, ""]
    # parts[0]是空（因为开头就是III），parts[-1]是空（因为结尾是III）

    if len(parts) < 6:
        print(f"分割字段数量不足: {len(parts)}")
        return frame

    # 5. 填充帧数据
    frame.head = "/f/b"
    frame.tail = "/b/f"

    try:
        # parts[1]是count, parts[2]是cmd_id, parts[3]是content_len, parts[4]是content
        frame.count = int(parts[1]) if parts[1] else 0
        frame.cmd_id = int(parts[2]) if parts[2] else 0
        frame.content_len = int(parts[3]) if parts[3] else 0
        frame.content = parts[4] if len(parts) > 4 else ""

    except ValueError as e:
        print(f"数据转换错误: {e}")
        return frame

    # 6. 验证内容长度
    if frame.content_len > 0 and len(frame.content) != frame.content_len:
        print(f"警告: 内容长度不匹配 - 声明={frame.content_len}, 实际={len(frame.content)}")

    return frame

def get_robot_lua_program_500_err_code(content: str) -> Tuple[int, int]:

    #获取lua程序500错误行号和错误码
    #Args:
    #   content: 错误内容字符串
    #Returns:
    #    Tuple[int, int]: (错误行号, Lua错误码)

    err_lin_num = 0
    lua_err_code = 0

    # 检查是否是lua错误
    lua_pos = content.find(".lua")
    if lua_pos == -1:
        return err_lin_num, lua_err_code

    # 找第一个冒号（文件名后的冒号）
    colon1 = content.find(':', lua_pos)
    if colon1 == -1:
        return err_lin_num, lua_err_code

    # 找第二个冒号（行号后的冒号）
    colon2 = content.find(':', colon1 + 1)
    if colon2 == -1:
        return err_lin_num, lua_err_code

    # 提取行号
    line_str = content[colon1 + 1:colon2]
    try:
        err_lin_num = int(line_str)
    except ValueError:
        pass

    # 找错误码
    errcode_pos = content.find("errcode", colon2)
    if errcode_pos == -1:
        return err_lin_num, lua_err_code

    # 提取错误码数字
    code_start = -1
    for i in range(errcode_pos + 7, len(content)):
        if content[i].isdigit():
            code_start = i
            break

    if code_start != -1:
        code_str = ""
        for i in range(code_start, len(content)):
            if content[i].isdigit():
                code_str += content[i]
            else:
                break
        try:
            lua_err_code = int(code_str)
        except ValueError:
            pass

    return err_lin_num, lua_err_code

class FrUdpClient:
    """
    FR UDP通信客户端类（内部实现，不对外暴露）
    通过20007端口创建UDP套接字
    发送：透传，不封装
    接收：解析成帧结构，通过回调返回给上层
    """

    def __init__(self, ip: str, port: int = 20007):
        """
        初始化FR UDP客户端
        Args:
            ip: 机器人IP地址
            port: UDP端口，默认20007
        """
        self.ip = ip
        self.port = port
        self.callback = None

        # UDP套接字
        self.udp_socket: Optional[socket.socket] = None

        # 线程控制
        self.stop_event = threading.Event()
        self.recv_thread: Optional[threading.Thread] = None

        # 接收缓冲区（字符串）
        self.recv_buffer = ""
        self.buffer_lock = threading.Lock()

        # 创建UDP套接字
        self._create_socket()

        print(f"FrUdpClient 初始化完成 - 目标IP: {ip}:{port}")

    def _create_socket(self):
        """创建UDP套接字"""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.settimeout(1.0)
            self.udp_socket.bind(('0.0.0.0', 0))  # 绑定到任意本地端口

            local_host, local_port = self.udp_socket.getsockname()

            print(f"UDP套接字创建成功 - 本地端口: {local_port}")

        except Exception as e:
            print(f"UDP套接字创建失败: {e}")
            self.udp_socket = None

    def start_recv_thread(self):
        """启动接收线程"""
        if self.recv_thread and self.recv_thread.is_alive():
            return

        self.stop_event.clear()
        self.recv_thread = threading.Thread(
            target=self._recv_thread_func,
            name="RobotUdpDataRecvThread",
            daemon=True
        )
        self.recv_thread.start()
        print("RobotUdpDataRecvThread 已启动")

    def stop_recv_thread(self):
        """停止接收线程"""
        self.stop_event.set()
        if self.recv_thread:
            self.recv_thread.join(timeout=3.0)

    def _recv_thread_func(self):
        """UDP接收线程函数"""
        while not self.stop_event.is_set():
            try:
                if not self.udp_socket:
                    time.sleep(1)
                    continue

                data, addr = self.udp_socket.recvfrom(65535)

                # 将接收到的字节数据转换为字符串
                try:
                    received_str = data.decode('utf-8')
                except UnicodeDecodeError:
                    print(f"收到非UTF-8数据，长度: {len(data)} 字节")
                    continue

                # 将接收到的字符串添加到缓冲区
                with self.buffer_lock:
                    self.recv_buffer += received_str

                    # 从缓冲区中提取并处理完整的帧
                    self._process_buffer()

            except socket.timeout:
                continue
            except Exception as e:
                if not self.stop_event.is_set():
                    print(f"UDP接收错误: {e}")
                time.sleep(0.1)

    def _process_buffer(self):
        """处理接收缓冲区中的帧数据"""
        # 从缓冲区中分割出完整的帧
        frames = split_frame(self.recv_buffer)

        if frames:
            # 找到最后一个完整帧的结束位置
            last_frame_end = self.recv_buffer.rfind("/b/f")
            if last_frame_end != -1:
                # 保留未完成的数据
                self.recv_buffer = self.recv_buffer[last_frame_end + 4:]
            else:
                self.recv_buffer = ""

            # 处理每个完整的帧
            for frame_str in frames:
                self._process_frame(frame_str)

    def _process_frame(self, frame_str: str):
        """
        处理单个帧 - 解析并通过回调返回给上层

        Args:
            frame_str: 帧字符串
        """
        # 解析帧
        frame = unpack_frame(frame_str)

        # 验证解析是否成功
        if frame.head != "/f/b" or frame.tail != "/b/f":
            print("帧解析失败")
            return
        # print(f"[DEBUG] 原始帧字符串: {frame_str}")
        #print(f"收到UDP帧 - 计数:{frame.count} 命令ID:{frame.cmd_id} 数据长度:{frame.content_len}")


        # 检查是否是Lua错误
        if frame.cmd_id == 500:
            err_lin_num, lua_err_code = get_robot_lua_program_500_err_code(frame.content)
            if err_lin_num != 0 or lua_err_code != 0:
                print(f"Lua程序错误 - 行号:{err_lin_num}, 错误码:{lua_err_code}")

        # 调用回调函数，将解析后的帧数据返回给上层
        if self.callback:
            try:
                # 回调函数格式: int callback(int srcType, int count, int cmdID, int daLen, string content)
                self.callback(0, frame.count, frame.cmd_id, frame.content_len, frame.content)

            except Exception as e:
                print(f"回调执行错误: {e}")

    def _verify_frame(self, frame):
        if len(frame) < 20 or \
                frame[:4] != "/f/b" or \
                frame[-4:] != "/b/f":
            return False

        data = frame[4:-4]

        parts = []
        start = 0
        separator = "III"

        for i in range(5):
            pos = data.find(separator, start)
            if pos == -1:
                return False
            parts.append(data[start:pos])
            start = pos + len(separator)
        parts.append(data[start:])

        try:
            return len(parts) == 6 and \
                all(parts[i] for i in range(1, 5)) and \
                int(parts[3]) == len(parts[4])
        except:
            return False

    def send_data(self, data: str) -> bool:
        """
        发送UDP数据 - 透传，不封装

        Args:
            data: 要发送的字符串数据

        Returns:
            bool: 是否发送成功
        """
        if not self.udp_socket:
            print("UDP套接字未创建")
            return False
        if not self._verify_frame(data):
            return RobotError.ERR_PARAM_VALUE
        try:
            encoded_data = data.encode('utf-8')
            sent = self.udp_socket.sendto(encoded_data, (self.ip, self.port))

            #(f"发送UDP数据 - 长度:{sent} 字节")
            return sent == len(encoded_data)

        except Exception as e:
            print(f"发送UDP数据失败: {e}")
            return RobotError.ERR_SOCKET_SEND_FAILED

    def set_callback(self, callback: UdpFrameCallback):
        """设置帧接收回调函数"""
        self.callback = callback
        print("回调函数已设置")

    def close(self):
        """关闭UDP套接字"""
        self.stop_recv_thread()

        if self.udp_socket:
            try:
                self.udp_socket.close()
            except Exception:
                pass
            finally:
                self.udp_socket = None
        print("UDP套接字已关闭")

    def __del__(self):
        """析构函数"""
        self.close()


# ==================== RobotState 枚举 ====================
class RobotState(enum.Enum):
    """CNDE状态类型枚举"""
    FrameHead = 0
    FrameCnt = 1
    DataLen = 2
    ProgramState = 3
    RobotState = 4
    MainCode = 5
    SubCode = 6
    RobotMode = 7
    JointCurPos = 8
    ToolCurPos = 9
    FlangeCurPos = 10
    ActualJointVel = 11
    ActualJointAcc = 12
    TargetTCPCmpSpeed = 13
    TargetTCPSpeed = 14
    ActualTCPCmpSpeed = 15
    ActualTCPSpeed = 16
    ActualJointTorque = 17
    Tool = 18
    User = 19
    ClDgtOutputH = 20
    ClDgtOutputL = 21
    TlDgtOutputL = 22
    ClDgtInputH = 23
    ClDgtInputL = 24
    TlDgtInputL = 25
    ClAnalogInput = 26
    TlAnglogInput = 27
    FtSensorRawData = 28
    FtSensorData = 29
    FtSensorActive = 30
    EmergencyStop = 31
    MotionDone = 32
    GripperMotiondone = 33
    McQueueLen = 34
    CollisionState = 35
    TrajectoryPnum = 36
    SafetyStop0State = 37
    SafetyStop1State = 38
    GripperFaultId = 39
    GripperFault = 40
    GripperActive = 41
    GripperPosition = 42
    GripperSpeed = 43
    GripperCurrent = 44
    GripperTemp = 45
    GripperVoltage = 46
    AuxState = 47
    ExtAxisStatus = 48
    ExtDIState = 49
    ExtDOState = 50
    ExtAIState = 51
    ExtAOState = 52
    RbtEnableState = 53
    JointDriverTorque = 54
    JointDriverTemperature = 55
    RobotTime = 56
    SoftwareUpgradeState = 57
    EndLuaErrCode = 58
    ClAnalogOutput = 59
    TlAnalogOutput = 60
    GripperRotNum = 61
    GripperRotSpeed = 62
    GripperRotTorque = 63
    WeldingBreakOffState = 64
    TargetJointTorque = 65
    SmartToolState = 66
    WideVoltageCtrlBoxTemp = 67
    WideVoltageCtrlBoxFanCurrent = 68
    ToolCoord = 69
    WobjCoord = 70
    ExtoolCoord = 71
    ExAxisCoord = 72
    Load = 73
    LoadCog = 74
    LastServoTarget = 75
    ServoJCmdNum = 76
    TargetJointPos = 77
    TargetJointVel = 78
    TargetJointAcc = 79
    TargetJointCurrent = 80
    ActualJointCurrent = 81
    ActualTCPForce = 82
    TargetTCPPos = 83
    CollisionLevel = 84
    SpeedScaleManual = 85
    SpeedScaleAuto = 86
    LuaLineNum = 87
    AbnomalStop = 88
    CurrentLuaFileName = 89
    ProgramTotalLine = 90
    SafetyBoxSingal = 91
    WeldVoltage = 92
    WeldCurrent = 93
    WeldTrackVel = 94
    TpdException = 95
    AlarmRebootRobot = 96
    ModbusMasterConnect = 97
    ModbusSlaveConnect = 98
    BtnBoxStopSignal = 99
    DragAlarm = 100
    SafetyDoorAlarm = 101
    SafetyPlaneAlarm = 102
    MotonAlarm = 103
    InterfaceAlarm = 104
    UdpCmdState = 105
    WeldReadyState = 106
    AlarmCheckEmergStopBtn = 107
    TsTmCmdComError = 108
    TsTmStateComError = 109
    CtrlBoxError = 110
    SafetyDataState = 111
    ForceSensorErrState = 112
    CtrlOpenLuaErrCode = 113
    StrangePosFlag = 114
    Alarm = 115
    DriverAlarm = 116
    AliveSlaveNumError = 117
    SlaveComError = 118
    CmdPointError = 119
    IOError = 120
    GripperError = 121
    FileError = 122
    ParaError = 123
    ExaxisOutLimitError = 124
    DriverComError = 125
    DriverError = 126
    OutSoftLimitError = 127
    AxleGenComData = 128
    SocketConnTimeout = 129
    SocketReadTimeout = 130
    TsWebStateComErr = 131
    CheckSum = 132


# ==================== CNDE状态映射表 ====================
CNDE_STATE_CONFIG = {
    RobotState.FrameHead: ("frame_head", "frame_head", "UINT16", "UINT16"),
    RobotState.FrameCnt: ("frame_cnt", "frame_cnt", "UINT8", "UINT8"),
    RobotState.DataLen: ("data_len", "data_len", "UINT16", "UINT16"),
    RobotState.ProgramState: ("program_state", "program_state", "UINT8", "UINT8"),
    RobotState.RobotState: ("robot_state", "robot_state", "UINT8", "UINT8"),
    RobotState.MainCode: ("main_code", "main_code", "INT32", "INT32"),
    RobotState.SubCode: ("sub_code", "sub_code", "INT32", "INT32"),
    RobotState.RobotMode: ("robot_mode", "robot_mode", "UINT8", "UINT8"),
    RobotState.JointCurPos: ("actual_joint_pos", "jt_cur_pos", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ToolCurPos: ("actual_TCP_pos", "tl_cur_pos", "DOUBLE_6", "DOUBLE_6"),
    RobotState.FlangeCurPos: ("actual_flange_pos", "flange_cur_pos", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ActualJointVel: ("actual_joint_vel", "actual_qd", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ActualJointAcc: ("actual_joint_acc", "actual_qdd", "DOUBLE_6", "DOUBLE_6"),
    RobotState.TargetTCPCmpSpeed: ("target_TCP_cmpvel", "target_TCP_CmpSpeed", "DOUBLE_2", "DOUBLE_2"),
    RobotState.TargetTCPSpeed: ("target_TCP_vel", "target_TCP_Speed", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ActualTCPCmpSpeed: ("actual_TCP_cmpvel", "actual_TCP_CmpSpeed", "DOUBLE_2", "DOUBLE_2"),
    RobotState.ActualTCPSpeed: ("actual_TCP_vel", "actual_TCP_Speed", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ActualJointTorque: ("actual_joint_torque", "jt_cur_tor", "DOUBLE_6", "DOUBLE_6"),
    RobotState.Tool: ("tool_id", "tool", "INT32", "INT32"),
    RobotState.User: ("wobj_id", "user", "INT32", "INT32"),
    RobotState.ClDgtOutputH: ("cfg_DO_box", "cl_dgt_output_h", "UINT8", "UINT8"),
    RobotState.ClDgtOutputL: ("std_DO_box", "cl_dgt_output_l", "UINT8", "UINT8"),
    RobotState.TlDgtOutputL: ("cfg_DO_tool", "tl_dgt_output_l", "UINT8", "UINT8"),
    RobotState.ClDgtInputH: ("cfg_DI_box", "cl_dgt_input_h", "UINT8", "UINT8"),
    RobotState.ClDgtInputL: ("std_DI_box", "cl_dgt_input_l", "UINT8", "UINT8"),
    RobotState.TlDgtInputL: ("cfg_DI_tool", "tl_dgt_input_l", "UINT8", "UINT8"),
    RobotState.ClAnalogInput: ("std_AI0_box,std_AI1_box", "cl_analog_input", "UINT16_2", "DOUBLE,DOUBLE"),
    RobotState.TlAnglogInput: ("std_AI_tool", "tl_anglog_input", "UINT16", "DOUBLE"),
    RobotState.FtSensorRawData: ("ft_sensor_raw_data", "ft_sensor_raw_data", "DOUBLE_6", "DOUBLE_6"),
    RobotState.FtSensorData: ("ft_sensor_data", "ft_sensor_data", "DOUBLE_6", "DOUBLE_6"),
    RobotState.FtSensorActive: ("ft_sensor_active", "ft_sensor_active", "UINT8", "UINT8"),
    RobotState.EmergencyStop: ("emergency_stop", "EmergencyStop", "UINT8", "UINT8"),
    RobotState.MotionDone: ("motion_done", "motion_done", "INT32", "INT32"),
    RobotState.GripperMotiondone: ("gripper_motion_done", "gripper_motiondone", "UINT8", "UINT8"),
    RobotState.McQueueLen: ("motion_queue_len", "mc_queue_len", "INT32", "INT32"),
    RobotState.CollisionState: ("collision_state", "collisionState", "UINT8", "UINT8"),
    RobotState.TrajectoryPnum: ("trajectory_pnum", "trajectory_pnum", "INT32", "INT32"),
    RobotState.SafetyStop0State: ("safety_stop0_state", "safety_stop0_state", "UINT8", "UINT8"),
    RobotState.SafetyStop1State: ("safety_stop1_state", "safety_stop1_state", "UINT8", "UINT8"),
    RobotState.GripperFaultId: ("gripper_fault_id", "gripper_fault_id", "UINT8", "UINT8"),
    RobotState.GripperFault: ("gripper_fault", "gripper_fault", "UINT16", "INT32"),
    RobotState.GripperActive: ("gripper_active", "gripper_active", "UINT16", "INT32"),
    RobotState.GripperPosition: ("gripper_position", "gripper_position", "UINT8", "UINT8"),
    RobotState.GripperSpeed: ("gripper_speed", "gripper_speed", "INT8", "INT32"),
    RobotState.GripperCurrent: ("gripper_current", "gripper_current", "INT8", "INT32"),
    RobotState.GripperTemp: ("gripper_temp", "gripper_temp", "INT32", "INT32"),
    RobotState.GripperVoltage: ("gripper_voltage", "gripper_voltage", "INT32", "INT32"),
    RobotState.AuxState: ("aux_axis_state", "aux_axis_state", "UINT8_25", "UINT8_25"),
    RobotState.ExtAxisStatus: ("exaxis_status", "extAxisStatus", "UINT8_116", "UINT8_116"),
    RobotState.ExtDIState: ("ext_DI_state", "extDIState", "UINT16_8", "UINT8_16"),
    RobotState.ExtDOState: ("ext_DO_state", "extDOState", "UINT16_8", "UINT8_16"),
    RobotState.ExtAIState: ("ext_AI_state", "extAIState", "UINT16_4", "INT32_4"),
    RobotState.ExtAOState: ("ext_AO_state", "extAOState", "UINT16_4", "INT32_4"),
    RobotState.RbtEnableState: ("rbt_enable_state", "rbtEnableState", "INT32", "INT32"),
    RobotState.JointDriverTorque: ("joint_driver_torque", "jointDriverTorque", "DOUBLE_6", "DOUBLE_6"),
    RobotState.JointDriverTemperature: ("actual_joint_temp", "jointDriverTemperature", "DOUBLE_6", "DOUBLE_6"),
    RobotState.RobotTime: ("robot_time", "year,mouth,day,hour,minute,second,millisecond", "UINT16,UINT8,UINT8,UINT8,UINT8,UINT8,UINT16", "INT32_7"),
    RobotState.SoftwareUpgradeState: ("software_upgrade_state", "softwareUpgradeState", "INT32", "INT32"),
    RobotState.EndLuaErrCode: ("end_lua_err_code", "endLuaErrCode", "UINT16", "INT32"),
    RobotState.ClAnalogOutput: ("std_AO0_box,std_AO1_box", "cl_analog_output", "UINT16_2", "DOUBLE,DOUBLE"),
    RobotState.TlAnalogOutput: ("std_AO_tool", "tl_analog_output", "UINT16", "DOUBLE"),
    RobotState.GripperRotNum: ("rotating_gripper_num", "gripperRotNum", "FLOAT", "DOUBLE"),
    RobotState.GripperRotSpeed: ("rotating_gripper_speed", "gripperRotSpeed", "UINT8", "UINT8"),
    RobotState.GripperRotTorque: ("rotating_gripper_tor", "gripperRotTorque", "UINT8", "UINT8"),
    RobotState.WeldingBreakOffState: ("weld_break_off_state,weld_arc_state", "weldingBreakOffState", "STRUCT", "UINT8,UINT8"),
    RobotState.TargetJointTorque: ("target_joint_torque", "jt_tgt_tor", "DOUBLE_6", "DOUBLE_6"),
    RobotState.SmartToolState: ("smarttool_state", "smartToolState", "INT32", "UINT32"),
    RobotState.WideVoltageCtrlBoxTemp: ("wide_voltage_ctrl_box_temp", "wideVoltageCtrlBoxTemp", "FLOAT", "DOUBLE"),
    RobotState.WideVoltageCtrlBoxFanCurrent: ("wide_voltage_ctrl_box_fan_current", "wideVoltageCtrlBoxFanCurrent", "UINT16", "INT32"),
    RobotState.ToolCoord: ("tool_coord", "toolCoord", "DOUBLE_6", "DOUBLE_6"),
    RobotState.WobjCoord: ("wobj_coord", "wobjCoord", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ExtoolCoord: ("exTool_coord", "extoolCoord", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ExAxisCoord: ("exAxis_coord", "exAxisCoord", "DOUBLE_6", "DOUBLE_6"),
    RobotState.Load: ("payload", "load", "DOUBLE", "DOUBLE"),
    RobotState.LoadCog: ("pay_cog", "loadCog", "DOUBLE_3", "DOUBLE_3"),
    RobotState.LastServoTarget: ("last_servoJ_target", "lastServoTarget", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ServoJCmdNum: ("servoJ_cmd_num", "servoJCmdNum", "INT32", "INT32"),
    RobotState.TargetJointPos: ("target_joint_pos", "targetJointPos", "DOUBLE_6", "DOUBLE_6"),
    RobotState.TargetJointVel: ("target_joint_vel", "targetJointVel", "DOUBLE_6", "DOUBLE_6"),
    RobotState.TargetJointAcc: ("target_joint_acc", "targetJointAcc", "DOUBLE_6", "DOUBLE_6"),
    RobotState.TargetJointCurrent: ("target_joint_current", "targetJointCurrent", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ActualJointCurrent: ("actual_joint_current", "actualJointCurrent", "DOUBLE_6", "DOUBLE_6"),
    RobotState.ActualTCPForce: ("actual_TCP_force", "actualTCPForce", "DOUBLE_6", "DOUBLE_6"),
    RobotState.TargetTCPPos: ("target_TCP_pos", "targetTCPPos", "DOUBLE_6", "DOUBLE_6"),
    RobotState.CollisionLevel: ("collision_level", "collisionLevel", "UINT8_6", "UINT8_6"),
    RobotState.SpeedScaleManual: ("speed_scaling_man", "speedScaleManual", "DOUBLE", "DOUBLE"),
    RobotState.SpeedScaleAuto: ("speed_scaling_auto", "speedScaleAuto", "DOUBLE", "DOUBLE"),
    RobotState.LuaLineNum: ("line_number", "luaLineNum", "INT32", "INT32"),
    RobotState.AbnomalStop: ("abnormal_stop", "abnomalStop", "UINT8", "UINT8"),
    RobotState.CurrentLuaFileName: ("cur_lua_file_name", "currentLuaFileName", "UINT8_256", "UINT8_256"),
    RobotState.ProgramTotalLine: ("prog_total_line", "programTotalLine", "UINT8", "UINT8"),
    RobotState.SafetyBoxSingal: ("safety_box_signal", "safetyBoxSingal", "UINT8_6", "UINT8_6"),
    RobotState.WeldVoltage: ("welding_voltage", "weldVoltage", "DOUBLE", "DOUBLE"),
    RobotState.WeldCurrent: ("welding_current", "weldCurrent", "DOUBLE", "DOUBLE"),
    RobotState.WeldTrackVel: ("welding_track_speed", "weldTrackVel", "DOUBLE", "DOUBLE"),
    RobotState.TpdException: ("tpd_exception", "tpdException", "UINT8", "UINT8"),
    RobotState.AlarmRebootRobot: ("alarm_reboot_robot", "alarmRebootRobot", "UINT8", "UINT8"),
    RobotState.ModbusMasterConnect: ("modbus_master_connect", "modbusMasterConnect", "UINT8", "UINT8"),
    RobotState.ModbusSlaveConnect: ("modbus_slave_connect", "modbusSlaveConnect", "UINT8", "UINT8"),
    RobotState.BtnBoxStopSignal: ("btn_box_stop_signal", "btnBoxStopSignal", "UINT8", "UINT8"),
    RobotState.DragAlarm: ("drag_alarm", "dragAlarm", "UINT8", "UINT8"),
    RobotState.SafetyDoorAlarm: ("safety_door_alarm", "safetyDoorAlarm", "UINT8", "UINT8"),
    RobotState.SafetyPlaneAlarm: ("safety_plane_alarm", "safetyPlaneAlarm", "UINT8", "UINT8"),
    RobotState.MotonAlarm: ("motion_alarm", "motonAlarm", "UINT8", "UINT8"),
    RobotState.InterfaceAlarm: ("interfere_alarm", "interfaceAlarm", "UINT8", "UINT8"),
    RobotState.UdpCmdState: ("udp_cmd_state", "udpCmdState", "INT32", "INT32"),
    RobotState.WeldReadyState: ("weld_ready_state", "weldReadyState", "UINT8", "UINT8"),
    RobotState.AlarmCheckEmergStopBtn: ("alarm_check_emerg_stop_btn", "alarmCheckEmergStopBtn", "UINT8", "UINT8"),
    RobotState.TsTmCmdComError: ("ts_tm_cmd_com_error", "tsTmCmdComError", "UINT8", "UINT8"),
    RobotState.TsTmStateComError: ("ts_tm_state_com_error", "tsTmStateComError", "UINT8", "UINT8"),
    RobotState.CtrlBoxError: ("ctrl_box_error", "ctrlBoxError", "INT32", "INT32"),
    RobotState.SafetyDataState: ("safety_data_state", "safetyDataState", "UINT8", "UINT8"),
    RobotState.ForceSensorErrState: ("force_sensor_err_state", "forceSensorErrState", "UINT8", "UINT8"),
    RobotState.CtrlOpenLuaErrCode: ("ctrl_open_lua_errcode", "ctrlOpenLuaErrCode", "UINT8_4", "UINT8_4"),
    RobotState.StrangePosFlag: ("strange_pos_flag", "strangePosFlag", "UINT8", "UINT8"),
    RobotState.Alarm: ("alarm", "alarm", "UINT8", "UINT8"),
    RobotState.DriverAlarm: ("dr_alarm", "driverAlarm", "UINT8", "UINT8"),
    RobotState.SocketConnTimeout: ("socket_conn_timeout", "socketConnTimeout", "UINT8", "UINT8"),
    RobotState.SocketReadTimeout: ("socket_read_timeout", "socketReadTimeout", "UINT8", "UINT8"),
    RobotState.TsWebStateComErr: ("ts_web_state_com_err", "tsWebStateComErr", "UINT8", "UINT8"),
    RobotState.AliveSlaveNumError: ("alive_slave_num_error", "aliveSlaveNumError", "UINT8", "UINT8"),
    RobotState.SlaveComError: ("slave_com_error", "slaveComError", "UINT8_8", "UINT8_8"),
    RobotState.CmdPointError: ("cmd_point_error", "cmdPointError", "UINT8", "UINT8"),
    RobotState.IOError: ("IO_error", "IOError", "UINT8", "UINT8"),
    RobotState.GripperError: ("gripper_error", "gripperError", "UINT8", "UINT8"),
    RobotState.FileError: ("file_error", "fileError", "UINT8", "UINT8"),
    RobotState.ParaError: ("para_error", "paraError", "UINT8", "UINT8"),
    RobotState.ExaxisOutLimitError: ("exaxis_out_slimit_error", "exaxisOutLimitError", "UINT8", "UINT8"),
    RobotState.DriverComError: ("dr_com_err", "driverComError", "UINT8_6", "UINT8_6"),
    RobotState.DriverError: ("dr_err", "driverError", "UINT8", "UINT8"),
    RobotState.OutSoftLimitError: ("out_sflimit_err", "outSoftLimitError", "UINT8", "UINT8"),
    RobotState.AxleGenComData: ("axle_gen_com_data", "axleGenComData", "UINT8_130", "UINT8_130"),
    RobotState.CheckSum: ("check_sum", "check_sum", "UINT16", "UINT16"),
}


# ==================== 默认CNDE配置（只包含到LastServoTarget的状态） ====================
DEFAULT_CNDE_STATES = [
    RobotState.ProgramState,
    RobotState.RobotState,
    RobotState.MainCode,
    RobotState.SubCode,
    RobotState.RobotMode,
    RobotState.JointCurPos,
    RobotState.ToolCurPos,
    RobotState.FlangeCurPos,
    RobotState.ActualJointVel,
    RobotState.ActualJointAcc,
    RobotState.TargetTCPCmpSpeed,
    RobotState.TargetTCPSpeed,
    RobotState.ActualTCPCmpSpeed,
    RobotState.ActualTCPSpeed,
    RobotState.ActualJointTorque,
    RobotState.Tool,
    RobotState.User,
    RobotState.ClDgtOutputH,
    RobotState.ClDgtOutputL,
    RobotState.TlDgtOutputL,
    RobotState.ClDgtInputH,
    RobotState.ClDgtInputL,
    RobotState.TlDgtInputL,
    RobotState.ClAnalogInput,
    RobotState.TlAnglogInput,
    RobotState.FtSensorRawData,
    RobotState.FtSensorData,
    RobotState.FtSensorActive,
    RobotState.EmergencyStop,
    RobotState.MotionDone,
    RobotState.GripperMotiondone,
    RobotState.McQueueLen,
    RobotState.CollisionState,
    RobotState.TrajectoryPnum,
    RobotState.SafetyStop0State,
    RobotState.SafetyStop1State,
    RobotState.GripperFaultId,
    RobotState.GripperFault,
    RobotState.GripperActive,
    RobotState.GripperPosition,
    RobotState.GripperSpeed,
    RobotState.GripperCurrent,
    RobotState.GripperTemp,
    RobotState.GripperVoltage,
    RobotState.AuxState,
    RobotState.ExtAxisStatus,
    RobotState.ExtDIState,
    RobotState.ExtDOState,
    RobotState.ExtAIState,
    RobotState.ExtAOState,
    RobotState.RbtEnableState,
    RobotState.JointDriverTorque,
    RobotState.JointDriverTemperature,
    RobotState.RobotTime,
    RobotState.SoftwareUpgradeState,
    RobotState.EndLuaErrCode,
    RobotState.ClAnalogOutput,
    RobotState.TlAnalogOutput,
    RobotState.GripperRotNum,
    RobotState.GripperRotSpeed,
    RobotState.GripperRotTorque,
    RobotState.WeldingBreakOffState,
    RobotState.TargetJointTorque,
    RobotState.SmartToolState,
    RobotState.WideVoltageCtrlBoxTemp,
    RobotState.WideVoltageCtrlBoxFanCurrent,
    RobotState.ToolCoord,
    RobotState.WobjCoord,
    RobotState.ExtoolCoord,
    RobotState.ExAxisCoord,
    RobotState.Load,
    RobotState.LoadCog,
    RobotState.LastServoTarget,
]


# 默认CNDE数据周期(ms)，可通过SetRobotRealtimeStateConfig修改
DEFAULT_CNDE_PERIOD = 8

# IP-specific配置存储 {ip: [RobotState列表]}
_ip_states: Dict[str, List[RobotState]] = {}


def SetRobotRealtimeStateConfig(states: List[RobotState], period: int = 500) -> int:
    """
    设置CNDE默认配置（在RPC连接前调用）
    
    使用示例:
        from fairino import Robot
        from fairino.Robot import RobotState, SetRobotRealtimeStateConfig
        
        # 设置自定义配置
        SetRobotRealtimeStateConfig([
            RobotState.ProgramState,
            RobotState.RobotState,
            RobotState.JointCurPos,
        ], 100)
        
        # 创建RPC连接，自动使用上面的配置
        robot = Robot.RPC('192.168.58.2')
    
    Args:
        states: RobotState枚举列表
        period: 数据周期(ms)，范围8-1000，默认8ms
    
    Returns:
        0-成功，其他-错误码
    """
    global DEFAULT_CNDE_STATES, DEFAULT_CNDE_PERIOD
    
    if not states:
        print("错误：至少需要一个状态")
        return RobotError.ERR_NEED_AT_LEAST_ONE_STATE
    if period < 8 or period > 1000:
        print("错误：周期必须在8-1000ms之间")
        return RobotError.ERR_PARAM_VALUE
    
    DEFAULT_CNDE_STATES = states.copy()
    DEFAULT_CNDE_PERIOD = period
    print(f"CNDE默认配置已设置: {len(states)} 个状态, {period}ms 周期")
    return 0


def AddRobotRealtimeState(states: List[RobotState], ip: str = None) -> int:
    """
    在配置基础上添加CNDE状态列表（支持动态维护和IP隔离）

    使用示例:
        from fairino import Robot
        from fairino.Robot import RobotState, AddRobotRealtimeState

        # 【全局配置】在默认配置基础上添加
        AddRobotRealtimeState([
            RobotState.FtSensorData,
            RobotState.GripperPosition,
        ])

        # 【IP隔离】为特定机器人添加状态
        AddRobotRealtimeState([RobotState.ServoJCmdNum], ip='192.168.58.2')

        # 创建RPC连接，各自使用对应配置
        robot1 = Robot.RPC('192.168.58.2')  # 默认+ServoJCmdNum
        robot2 = Robot.RPC('192.168.58.3')  # 仅默认配置

    Args:
        states: RobotState枚举列表，要添加的状态
        ip: 可选，指定机器人IP（用于多机器人隔离配置，不提供则修改全局配置）

    Returns:
        0-成功，其他-错误码
    """
    global DEFAULT_CNDE_STATES, _ip_states

    # 确定要操作的配置列表
    if ip:
        if ip not in _ip_states:
            # 第一次为该IP添加，从默认配置复制一份
            _ip_states[ip] = DEFAULT_CNDE_STATES.copy()
        target_states = _ip_states[ip]
    else:
        target_states = DEFAULT_CNDE_STATES

    added_count = 0
    for state in states:
        if state not in CNDE_STATE_CONFIG:
            print(f"错误：无效的状态 {state}")
            return RobotError.ERR_STATE_INVALID
        if state in target_states:
            print(f"错误：状态 {state} 已存在")
            return RobotError.ERR_STATE_ALREADY_EXISTS
        target_states.append(state)
        added_count += 1

    scope = f"IP[{ip}]" if ip else "全局"
    print(f"CNDE配置已添加 {added_count} 个状态（{scope}），当前共 {len(target_states)} 个状态")
    return 0


def DeleteRobotRealtimeState(states: List[RobotState], ip: str = None) -> int:
    """
    在配置基础上删除CNDE状态列表（支持动态维护和IP隔离）

    使用示例:
        from fairino import Robot
        from fairino.Robot import RobotState, DeleteRobotRealtimeState

        # 【全局配置】在默认配置基础上删除
        DeleteRobotRealtimeState([
            RobotState.FtSensorRawData,
            RobotState.ClAnglogInput,
        ])

        # 【IP隔离】删除特定机器人的状态
        DeleteRobotRealtimeState([RobotState.FtSensorRawData], ip='192.168.58.2')

        # 创建RPC连接
        robot = Robot.RPC('192.168.58.2')

    Args:
        states: RobotState枚举列表，要删除的状态
        ip: 可选，指定机器人IP（用于多机器人隔离配置，不提供则修改全局配置）

    Returns:
        0-成功，其他-错误码
    """
    global DEFAULT_CNDE_STATES, _ip_states

    # 确定要操作的配置列表
    if ip:
        if ip not in _ip_states:
            print(f"错误：IP {ip} 没有配置")
            return RobotError.ERR_STATE_INVALID
        target_states = _ip_states[ip]
    else:
        target_states = DEFAULT_CNDE_STATES

    removed_count = 0
    for state in states:
        if state not in CNDE_STATE_CONFIG:
            print(f"错误：无效的状态 {state}")
            return RobotError.ERR_STATE_INVALID
        if state not in target_states:
            print(f"错误：状态 {state} 不存在")
            return RobotError.ERR_STATE_INVALID
        if len(target_states) <= 1:
            print("错误：至少需要一个状态")
            return RobotError.ERR_NEED_AT_LEAST_ONE_STATE
        target_states.remove(state)
        removed_count += 1

    scope = f"IP[{ip}]" if ip else "全局"
    print(f"CNDE配置已删除 {removed_count} 个状态（{scope}），当前共 {len(target_states)} 个状态")
    return 0


def SetRobotRealtimeStatePeriod(period: int, ip: str = None) -> int:
    """
    设置CNDE状态反馈周期（支持全局或IP隔离）

    使用示例:
        from fairino import Robot
        from fairino.Robot import SetRobotRealtimeStatePeriod

        # 设置全局周期
        SetRobotRealtimeStatePeriod(100)

        # 为特定IP设置周期
        SetRobotRealtimeStatePeriod(50, ip='192.168.58.2')

    Args:
        period: 数据周期(ms)，范围8-1000
        ip: 可选，指定机器人IP（不提供则修改全局配置）

    Returns:
        0-成功，其他-错误码
    """
    global DEFAULT_CNDE_PERIOD

    if period < 8 or period > 1000:
        print(f"错误：周期必须在8-1000ms之间，当前值: {period}ms")
        return RobotError.ERR_PARAM_VALUE

    if ip:
        # IP-specific周期存储在 _ip_states 中，或需要新增存储
        # 目前简化处理：仅支持全局周期设置
        print(f"警告：IP-specific周期暂未实现，设置全局周期为 {period}ms")
    else:
        DEFAULT_CNDE_PERIOD = period
        print(f"CNDE默认周期已设置为 {period}ms")

    return 0


# ==================== 数据类型大小计算函数 ====================
def get_cnde_type_size(type_str: str) -> int:
    """获取CNDE数据类型大小
    支持：单类型(UINT8)、数组类型(DOUBLE_6)、多字段类型(DOUBLE,DOUBLE)
    """
    type_sizes = {
        "UINT8": 1, "INT8": 1,
        "UINT16": 2, "INT16": 2,
        "UINT32": 4, "INT32": 4,
        "FLOAT": 4, "DOUBLE": 8,
    }
    import re

    # 处理逗号分隔的多字段类型 (如 "DOUBLE,DOUBLE", "UINT8,UINT8")
    if ',' in type_str:
        total_size = 0
        for single_type in type_str.split(','):
            single_type = single_type.strip()
            size = get_cnde_type_size(single_type)  # 递归处理每个类型
            if size <= 0:
                return 0  # 如果任何类型无法解析，返回0表示错误
            total_size += size
        return total_size

    # 处理数组类型 (如 "DOUBLE_6", "INT32_7")
    array_match = re.match(r'(\w+)_(\d+)', type_str)
    if array_match:
        base_type = array_match.group(1)
        count = int(array_match.group(2))
        base_size = type_sizes.get(base_type, 0)
        if base_size <= 0:
            return 0
        return base_size * count

    # 单类型
    return type_sizes.get(type_str, 0)


# ==================== FRCNDEClient CNDE客户端类 ====================
class FRCNDEClient:
    """
    CNDE TCP客户端类
    通过20005端口与机器人建立TCP连接，使用CNDE协议接收状态数据
    """

    ERR_SUCCESS = 0
    ERR_NEED_AT_LEAST_ONE_STATE = -1
    ERR_STATE_INVALID = -2
    ERR_PARAM_VALUE = -3
    ERR_STATE_ALREADY_EXISTS = -4
    ERR_SOCKET_COM_FAILED = -5

    def __init__(self, robot_state_pkg: RobotStatePkg, com_err_flag: list, ip: str = None, rpc=None):
        self._robot_state_pkg = robot_state_pkg
        self._sock_com_err = com_err_flag
        self._rpc = rpc  # RPC实例引用，用于断线时触发重连
        self._tcp_socket = None
        self._ip = ip or "192.168.58.2"
        self._port = 20005
        self._robot_state_run_flag = False
        self._recv_thread = None
        self._stop_event = threading.Event()
        self._recv_mutex = threading.Lock()
        self._send_count = 0
        self._robot_state_period = DEFAULT_CNDE_PERIOD  # 使用全局默认周期
        self._config_states = []
        self._all_states = CNDE_STATE_CONFIG
        self._init_default_config(self._ip)

    def _init_default_config(self, ip: str = None):
        """初始化默认配置状态（优先使用IP-specific配置）"""
        global _ip_states

        if ip and ip in _ip_states:
            # 使用IP-specific配置
            self._config_states = _ip_states[ip].copy()
            print(f"CNDE配置从IP专属加载 ({ip}): {len(self._config_states)} 个状态")
        else:
            # 使用全局默认配置
            self._config_states = DEFAULT_CNDE_STATES.copy()

        self._robot_state_period = DEFAULT_CNDE_PERIOD

    def connect(self, ip: str = None, port: int = None) -> int:
        """连接到机器人CNDE端口"""
        if ip:
            self._ip = ip
        if port:
            self._port = port

        self._tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_socket.settimeout(3.0)

        try:
            self._tcp_socket.connect((self._ip, self._port))
        except Exception as e:
            print(f"CNDE连接失败: {e}")
            self._sock_com_err[0] = self.ERR_SOCKET_COM_FAILED
            return self.ERR_SOCKET_COM_FAILED

        self._robot_state_run_flag = True

        rtn = self._send_cnde_output_config()
        print(f"SendCNDEOutputConfig rtn is {rtn}")
        if rtn != 0:
            # 配置失败，关闭连接并清理状态
            self._robot_state_run_flag = False
            if self._tcp_socket:
                try:
                    self._tcp_socket.close()
                except Exception:
                    pass
                self._tcp_socket = None
            return rtn

        rtn = self._set_cnde_start()
        print(f"SetCNDEStart rtn is {rtn}")
        if rtn != 0:
            # 启动失败，关闭连接并清理状态
            self._robot_state_run_flag = False
            if self._tcp_socket:
                try:
                    self._tcp_socket.close()
                except Exception:
                    pass
                self._tcp_socket = None
            return rtn

        self._recv_thread = threading.Thread(target=self._recv_robot_state_thread, name="CNDERecvThread")
        self._recv_thread.daemon = True
        self._recv_thread.start()
        return 0

    def close(self) -> int:
        """关闭CNDE连接"""
        self._robot_state_run_flag = False
        self._stop_event.set()
        # 尝试发送停止帧，但忽略套接字已关闭的错误
        try:
            if self._tcp_socket:
                self._set_cnde_stop()
        except Exception:
            pass  # 连接已断开，忽略错误
        # 关闭TCP套接字
        if self._tcp_socket:
            try:
                self._tcp_socket.close()
            except Exception:
                pass
            self._tcp_socket = None
        # 等待接收线程结束
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=2.0)
        return 0

    def _send_data(self, data: bytes) -> int:
        """发送数据到TCP连接"""
        if not self._tcp_socket:
            return -1
        try:
            sent = self._tcp_socket.send(data)
            return sent if sent == len(data) else -1
        except OSError as e:
            # WinError 10038: 套接字已关闭，静默处理
            if e.winerror == 10038:
                return -1
            print(f"CNDE发送失败: {e}")
            return -1
        except Exception as e:
            print(f"CNDE发送失败: {e}")
            return -1

    def _recv_data(self, buf: bytearray, timeout: float = 1.0) -> int:
        """接收数据"""
        if not self._tcp_socket:
            return -1
        try:
            self._tcp_socket.settimeout(timeout)
            data = self._tcp_socket.recv(len(buf))
            if not data:
                return 0
            buf[:len(data)] = data
            return len(data)
        except socket.timeout:
            return 0
        except Exception as e:
            print(f"CNDE接收失败: {e}")
            return -1

    def _send_cnde_output_config(self) -> int:
        """发送CNDE输出配置帧"""
        pkg = CNDE_PKG()
        pkg.count = self._send_count
        self._send_count = (self._send_count + 1) % 256
        pkg.type = CNDE_FRAME_TYPE_OUTPUT_CONFIG

        config_data = bytearray()
        config_data.extend(Int16ToByte(self._robot_state_period))

        state_names = []
        for state in self._config_states:
            if state in self._all_states:
                cnde_name = self._all_states[state][0]
                state_names.append(cnde_name)

        state_names_str = ",".join(state_names)
        config_data.extend(state_names_str.encode('utf-8'))

        pkg.data = bytes(config_data)
        pkg.len = len(pkg.data)

        frame = CNDEPkgToFrame(pkg)
        print(f"发送CNDE配置: 周期={self._robot_state_period}ms, 状态数={len(self._config_states)}")
        print(f"配置列表: {state_names_str}")  # 调试：确认实际发送的状态名

        rtn = self._send_data(frame)
        if rtn <= 0:
            print("CNDE Send output config pkg Failed")
            return -1

        pkg_buf = bytearray(CNDE_MAX_PKG_SIZE)
        timeout_count = 5

        while self._robot_state_run_flag and timeout_count > 0:
            recv_len = self._recv_data(pkg_buf, 0.1)
            if recv_len < 0:
                self._sock_com_err[0] = self.ERR_SOCKET_COM_FAILED
                return -1
            elif recv_len == 0:
                timeout_count -= 1
                continue

            recv_pkg = CNDE_PKG()
            rtn = FrameToCNDEPkg(bytes(pkg_buf[:recv_len]), recv_pkg)
            if rtn != 0:
                timeout_count -= 1  # 帧解析失败也减少计数
                continue

            # # 调试：打印收到的所有帧
            # if recv_pkg.type == 6 and recv_pkg.data:
            #     try:
            #         data_str = recv_pkg.data.decode('utf-8', errors='replace')
            #         print(f"  [Config]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data='{data_str}'")
            #     except:
            #          print(f"  [Config]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data={recv_pkg.data[:min(20, len(recv_pkg.data))].hex()}")
            # else:
            #      print(f"  [Config]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data={recv_pkg.data[:min(20, len(recv_pkg.data))].hex() if recv_pkg.data else 'None'}")

            if recv_pkg.type == CNDE_FRAME_TYPE_MESSAGE:
                if len(recv_pkg.data) > 0 and recv_pkg.data[0] == 0x00:
                    print("CNDE配置成功")
                    return 0
                else:
                    # 解析数据内容，检查是否包含NOT_FOUND
                    data_str = ""
                    try:
                        if recv_pkg.data:
                            # 尝试解码数据（跳过第一个字节的状态码）
                            data_str = recv_pkg.data[1:].decode('utf-8', errors='replace') if len(recv_pkg.data) > 1 else ""
                    except Exception as e:
                        print(f"解析响应数据失败: {e}")

                    print(f"CNDE配置失败: data[0]={recv_pkg.data[0] if recv_pkg.data else 'None'}")
                    # if recv_pkg.data:
                    #      print(f"响应数据: {recv_pkg.data.hex()}")

                    # 检查是否包含NOT_FOUND（状态不存在）
                    if "NOT_FOUND" in data_str.upper():
                        print(f"检测到NOT_FOUND错误: {data_str}")
                        return RobotError.ERR_STATE_INVALID
                    return -2
            else:
                # 收到其他类型帧（如数据帧），减少超时计数
                timeout_count -= 1
        print("CNDE配置超时")
        return -3


    def _set_cnde_start(self) -> int:
        """发送CNDE开始帧"""
        pkg = CNDE_PKG()
        pkg.count = self._send_count
        self._send_count = (self._send_count + 1) % 256
        pkg.type = CNDE_FRAME_TYPE_OUTPUT_START
        pkg.len = 0

        frame = CNDEPkgToFrame(pkg)
        rtn = self._send_data(frame)
        if rtn <= 0:
            print("CNDE Send Start pkg Failed")
            return -1

        pkg_buf = bytearray(CNDE_MAX_PKG_SIZE)
        timeout_count = 50

        while self._robot_state_run_flag and timeout_count > 0:
            recv_len = self._recv_data(pkg_buf, 0.1)
            if recv_len < 0:
                self._sock_com_err[0] = self.ERR_SOCKET_COM_FAILED
                return -1
            elif recv_len == 0:
                timeout_count -= 1
                continue

            recv_pkg = CNDE_PKG()
            rtn = FrameToCNDEPkg(bytes(pkg_buf[:recv_len]), recv_pkg)
            if rtn != 0:
                timeout_count -= 1
                continue

            # # 调试：打印接收到的帧类型和内容
            # if recv_pkg.type == 6 and recv_pkg.data:
            #     try:
            #         data_str = recv_pkg.data.decode('utf-8', errors='replace')
            #         print(f"  [Start]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data='{data_str}'")
            #     except:
            #         print(f"  [Start]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data={recv_pkg.data[:min(20, len(recv_pkg.data))].hex()}")
            # else:
            #     print(f"  [Start]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data={recv_pkg.data[:min(20, len(recv_pkg.data))].hex() if recv_pkg.data else 'None'}")

            if recv_pkg.type == CNDE_FRAME_TYPE_MESSAGE:
                if len(recv_pkg.data) > 0 and recv_pkg.data[0] == 0x00:
                    print("CNDE开始成功")
                    return 0
                else:
                    print(f"CNDE开始失败: data[0]={recv_pkg.data[0] if recv_pkg.data else 'None'}")
                    return -2
            elif recv_pkg.type == CNDE_FRAME_TYPE_OUTPUT_DATA:
                # 机器人可能直接发送数据帧（表示已开始）
                print("CNDE开始成功（收到数据帧）")
                # 缓存数据帧供解析
                self._parse_cnde_state_data(recv_pkg.data)
                return 0
            else:
                timeout_count -= 1
        print("CNDE开始超时")
        return -3


    def _set_cnde_stop(self) -> int:
        """发送CNDE停止帧"""
        with self._recv_mutex:
            pkg = CNDE_PKG()
            pkg.count = self._send_count
            self._send_count = (self._send_count + 1) % 256
            pkg.type = CNDE_FRAME_TYPE_OUTPUT_STOP
            pkg.len = 0

            frame = CNDEPkgToFrame(pkg)
            rtn = self._send_data(frame)
            if rtn <= 0:
                # 套接字已关闭时不打印错误（正常关闭流程）
                if self._tcp_socket:
                    print("CNDE Send Stop pkg Failed")
                return -1

            pkg_buf = bytearray(CNDE_MAX_PKG_SIZE)
            timeout_count = 30

            while self._robot_state_run_flag and timeout_count > 0:
                recv_len = self._recv_data(pkg_buf, 0.1)
                if recv_len < 0:
                    return -1
                elif recv_len == 0:
                    # 超时，重新发送停止帧（与C++一致）
                    self._send_data(frame)
                    timeout_count -= 1
                    continue

                recv_pkg = CNDE_PKG()
                rtn = FrameToCNDEPkg(bytes(pkg_buf[:recv_len]), recv_pkg)
                if rtn != 0:
                    timeout_count -= 1
                    continue

                # 调试：打印接收到的帧
                # if recv_pkg.type == 6 and recv_pkg.data:
                #     try:
                #         data_str = recv_pkg.data.decode('utf-8', errors='replace')
                #         print(f"  [Stop]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data='{data_str}'")
                #     except:
                #         print(f"  [Stop]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data={recv_pkg.data[:min(20, len(recv_pkg.data))].hex()}")
                # else:
                #     print(f"  [Stop]收到帧: type={recv_pkg.type}, count={recv_pkg.count}, len={recv_pkg.len}, data={recv_pkg.data[:min(20, len(recv_pkg.data))].hex() if recv_pkg.data else 'None'}")

                if recv_pkg.type == CNDE_FRAME_TYPE_MESSAGE:
                    if len(recv_pkg.data) > 0 and recv_pkg.data[0] == 0x00:
                        print("CNDE停止成功")
                        return 0
                    else:
                        print("CNDE停止失败")
                        return -2
                else:
                    timeout_count -= 1
            print("CNDE停止超时")
            return -3

    def _recv_robot_state_thread(self):
        """CNDE状态数据接收线程"""
        pkg_buf = bytearray(CNDE_MAX_PKG_SIZE)

        while self._robot_state_run_flag and not self._stop_event.is_set():
            with self._recv_mutex:
                recv_len = self._recv_data(pkg_buf, 0.5)
                if recv_len < 0:
                    print("CNDE接收错误，通讯失败")
                    self._sock_com_err[0] = self.ERR_SOCKET_COM_FAILED
                    # 触发RPC重连机制
                    if self._rpc is not None:
                        print("CNDE断线，触发RPC重连...")
                        threading.Thread(target=self._rpc.reconnect, daemon=True).start()
                    break
                elif recv_len == 0:
                    continue

                recv_pkg = CNDE_PKG()
                rtn = FrameToCNDEPkg(bytes(pkg_buf[:recv_len]), recv_pkg)
                if rtn != 0:
                    continue

                if recv_pkg.type == CNDE_FRAME_TYPE_OUTPUT_DATA:
                    self._parse_cnde_state_data(recv_pkg.data)

        if self._tcp_socket:
            try:
                self._tcp_socket.close()
            except Exception:
                pass

    def _parse_cnde_state_data(self, data: bytes):
        """
        解析CNDE状态数据
        根据配置的状态列表，按顺序解析数据并填充到结构体对应字段
        """
        state_ptr_index = 0
        data_len = len(data)

        # print(f"\n[解析开始] 总数据长度: {data_len} 字节, 配置状态数: {len(self._config_states)}")

        for idx, state in enumerate(self._config_states):
            if state not in self._all_states:
                print(f"  [{idx}] 状态 {state} 不在配置映射中，跳过")
                continue

            config_info = self._all_states[state]
            cnde_name = config_info[0]
            struct_field = config_info[1]
            struct_type = config_info[2]  # Python结构体字段类型
            cnde_type = config_info[3]    # CNDE发送的数据类型

            cnde_size = get_cnde_type_size(cnde_type)
            # print(f"  [{idx}] {cnde_name}: field='{struct_field}', cnde_type='{cnde_type}', "
                #   f"struct_type='{struct_type}', size={cnde_size}, offset={state_ptr_index}")

            if cnde_size <= 0:
                print(f"      错误: cnde_size <= 0")
                continue
            if state_ptr_index + cnde_size > data_len:
                print(f"      错误: 数据不足 ({state_ptr_index} + {cnde_size} > {data_len})")
                break

            # 打印原始数据片段（前16字节或全部）
            data_slice = data[state_ptr_index:state_ptr_index + cnde_size]
            hex_preview = ' '.join(f'{b:02X}' for b in data_slice[:min(16, len(data_slice))])
            if len(data_slice) > 16:
                hex_preview += f" ... (共{len(data_slice)}字节)"
            # print(f"      原始数据: {hex_preview}")

            try:
                self._parse_and_set_field(data_slice, struct_field, cnde_type, struct_type, cnde_size)
                state_ptr_index += cnde_size
                # print(f"      解析完成，新偏移: {state_ptr_index}")
            except Exception as e:
                print(f"      解析失败: {e}")
                import traceback
                traceback.print_exc()
                state_ptr_index += cnde_size

        # print(f"[解析结束] 最终偏移: {state_ptr_index}/{data_len}")

    def _parse_and_set_field(self, data: bytes, field_name: str, cnde_type: str, struct_type: str, cnde_size: int):
        """
        解析数据并设置结构体字段
        """
        # 处理多字段逐一赋值（如 RobotTime: year,mouth,day,hour,minute,second,millisecond）
        if ',' in field_name:
            field_names = [f.strip() for f in field_name.split(',')]
            # struct_type可能是逗号分隔的类型列表，也可能是数组类型如INT32_7
            if ',' in struct_type:
                types = [t.strip() for t in struct_type.split(',')]
            elif '_' in struct_type:
                # 数组类型如INT32_7，提取基础类型INT32
                base_type, count = self._parse_array_type(struct_type)
                types = [base_type] * len(field_names)
            else:
                types = [struct_type] * len(field_names)

            # 从cnde_type提取基础类型和数组大小（如INT32_7 -> INT32, 7）
            if '_' in cnde_type:
                cnde_base_type, cnde_count = self._parse_array_type(cnde_type)
                cnde_elem_size = self._get_type_size(cnde_base_type)
            else:
                cnde_base_type = cnde_type
                cnde_elem_size = self._get_type_size(cnde_type)

            offset = 0
            for i, fname in enumerate(field_names):
                if not hasattr(self._robot_state_pkg, fname):
                    offset += cnde_elem_size
                    continue
                if i >= len(types):
                    break

                struct_t = types[i]
                # 从CNDE数据中提取值
                sub_data = data[offset:offset+cnde_elem_size] if offset < len(data) else b''
                value = self._parse_value(sub_data, cnde_base_type)

                field = getattr(self._robot_state_pkg, fname)
                # 类型转换：CNDE类型 -> 目标结构体类型
                if struct_t == "UINT16":
                    value = int(value) & 0xFFFF
                elif struct_t == "UINT8":
                    value = int(value) & 0xFF

                setattr(self._robot_state_pkg, fname, type(field)(value))
                offset += cnde_elem_size
            return

        if not hasattr(self._robot_state_pkg, field_name):
            return

        field = getattr(self._robot_state_pkg, field_name)

        # 优先处理结构体字段（如 weldingBreakOffState）
        if isinstance(field, Structure) or struct_type == "STRUCT":
            self._set_struct_field(data, field, cnde_type)
            return

        # 处理结构体数组（如 extAxisStatus: EXT_AXIS_STATUS * 4）
        if isinstance(field, ctypes.Array) and len(field) > 0 and isinstance(field[0], Structure):
            self._set_struct_array_field(data, field, cnde_type)
            return

        # 处理数组类型（如 DOUBLE_6, UINT16_2）
        if '_' in struct_type and not self._is_complex_type(struct_type):
            # 多字段映射到数组（如 CNDE发送DOUBLE,DOUBLE映射到UINT16[2]）
            if ',' in cnde_type and isinstance(field, ctypes.Array):
                self._set_split_array_field(data, field, field_name, cnde_type, struct_type)
            else:
                self._set_array_field(data, field, field_name, cnde_type, struct_type)
            return

        # 处理多字段映射到数组（struct_type不是数组类型但field是数组）
        if ',' in cnde_type and isinstance(field, ctypes.Array):
            self._set_split_array_field(data, field, field_name, cnde_type, struct_type)
            return

        # 单字段处理
        self._set_single_field(data, field, field_name, struct_type, cnde_type)

    def _is_complex_type(self, type_str: str) -> bool:
        """判断是否为复杂类型（包含多个下划线）"""
        parts = type_str.split('_')
        return len(parts) > 2 or (len(parts) == 2 and not parts[1].isdigit())

    def _set_array_field(self, data: bytes, field, field_name: str, cnde_type: str, struct_type: str):
        """设置数组字段（如 DOUBLE_6）"""
        # 解析数组大小
        base_type, count = self._parse_array_type(struct_type)

        # 解析CNDE类型
        cnde_base, cnde_count = self._parse_array_type(cnde_type) if '_' in cnde_type else (cnde_type, count)

        # 计算元素大小
        struct_elem_size = self._get_type_size(base_type)
        cnde_elem_size = self._get_type_size(cnde_base)

        if struct_elem_size <= 0 or cnde_elem_size <= 0:
            return

        for i in range(min(count, len(field))):
            sub_data = data[i*cnde_elem_size:(i+1)*cnde_elem_size] if i*cnde_elem_size < len(data) else b''
            value = self._parse_value(sub_data, cnde_base)

            # 类型转换
            if base_type == "DOUBLE" and cnde_base != "DOUBLE":
                value = float(value)
            elif base_type == "UINT16" and cnde_base == "DOUBLE":
                value = int(value) & 0xFFFF
            elif base_type == "INT32" and cnde_base in ["UINT16", "UINT8"]:
                value = int(value)

            field[i] = type(field[0])(value)

    def _set_split_array_field(self, data: bytes, field, field_name: str, cnde_type: str, struct_type: str):
        """处理CNDE多字段映射到数组（如 DOUBLE,DOUBLE -> UINT16[2]）"""
        cnde_types = cnde_type.split(',')

        # 解析结构体数组类型
        base_type, count = self._parse_array_type(struct_type) if '_' in struct_type else (struct_type, len(field))

        for i, ct in enumerate(cnde_types):
            if i >= len(field):
                break
            ct = ct.strip()
            elem_size = self._get_type_size(ct)
            sub_data = data[i*elem_size:(i+1)*elem_size] if i*elem_size < len(data) else b''
            value = self._parse_value(sub_data, ct)

            # 类型转换
            if base_type == "UINT16" and ct == "DOUBLE":
                value = int(value) & 0xFFFF
            elif base_type == "UINT8" and ct == "UINT8":
                value = int(value) & 0xFF

            field[i] = type(field[0])(value)

    def _set_struct_field(self, data: bytes, field: Structure, cnde_type: str):
        """设置结构体字段（如 weldingBreakOffState）"""
        if ',' in cnde_type:
            # 多字段映射到结构体子字段
            types = [t.strip() for t in cnde_type.split(',')]
            offset = 0
            for fname, ftype in field._fields_:
                if offset >= len(types):
                    break
                ct = types[offset]
                elem_size = self._get_type_size(ct)
                sub_data = data[offset*elem_size:(offset+1)*elem_size] if offset*elem_size < len(data) else b''
                value = self._parse_value(sub_data, ct)
                setattr(field, fname, type(getattr(field, fname))(value))
                offset += 1
        else:
            # 单类型映射，直接memcpy风格设置
            offset = 0
            for fname, ftype in field._fields_:
                elem_size = self._get_type_size(cnde_type)
                sub_data = data[offset:offset+elem_size] if offset < len(data) else b''
                value = self._parse_value(sub_data, cnde_type)
                setattr(field, fname, type(getattr(field, fname))(value))
                offset += elem_size

    def _set_struct_array_field(self, data: bytes, field: ctypes.Array, cnde_type: str):
        """设置结构体数组字段（如 extAxisStatus: EXT_AXIS_STATUS * 4）
        
        直接字节拷贝，将CNDE原始数据memcpy到结构体数组
        """
        import ctypes
        # 计算结构体数组总大小
        struct_size = ctypes.sizeof(field[0])
        array_size = struct_size * len(field)
        data_len = len(data)
        
        # 拷贝数据（取最小值，防止溢出）
        copy_size = min(array_size, data_len)
        
        # 使用 memmove 直接拷贝字节到结构体数组内存
        ctypes.memmove(ctypes.addressof(field), data[:copy_size], copy_size)

    def _set_single_field(self, data: bytes, field, field_name: str, struct_type: str, cnde_type: str):
        """设置单个字段的值（含类型转换）"""
        value = self._parse_value(data, cnde_type)

        # 类型转换逻辑
        if struct_type == "UINT16" and cnde_type == "DOUBLE":
            value = int(value) & 0xFFFF
        elif struct_type == "INT32" and cnde_type == "UINT8":
            value = int(value)
        elif struct_type == "INT32" and cnde_type == "UINT16":
            value = int(value)
        elif struct_type == "FLOAT" and cnde_type == "DOUBLE":
            value = float(value)
        elif struct_type == "UINT8" and cnde_type == "INT32":
            value = int(value) & 0xFF

        setattr(self._robot_state_pkg, field_name, type(field)(value))

    def _parse_array_type(self, type_str: str) -> tuple:
        """解析数组类型，返回(base_type, count)"""
        if '_' not in type_str:
            return type_str, 1
        parts = type_str.rsplit('_', 1)
        try:
            return parts[0], int(parts[1])
        except (ValueError, IndexError):
            return type_str, 1

    def _get_type_size(self, type_str: str) -> int:
        """获取数据类型大小（字节）"""
        if '_' in type_str:
            base, count = self._parse_array_type(type_str)
            return self._get_type_size(base) * count

        sizes = {
            "UINT8": 1, "INT8": 1,
            "UINT16": 2, "INT16": 2,
            "UINT32": 4, "INT32": 4,
            "FLOAT": 4, "DOUBLE": 8
        }
        return sizes.get(type_str, 0)

    def _parse_value(self, data: bytes, type_str: str):
        """根据类型解析数据"""
        if not data:
            return 0 if not type_str.startswith("FLOAT") and not type_str.startswith("DOUBLE") else 0.0

        type_str = type_str.strip()

        if type_str == "UINT8":
            return data[0] if data else 0
        elif type_str == "INT8":
            return struct.unpack('<b', data[:1])[0] if data else 0
        elif type_str == "UINT16":
            return struct.unpack('<H', data[:2])[0] if len(data) >= 2 else 0
        elif type_str == "INT16":
            return struct.unpack('<h', data[:2])[0] if len(data) >= 2 else 0
        elif type_str == "UINT32":
            return struct.unpack('<I', data[:4])[0] if len(data) >= 4 else 0
        elif type_str == "INT32":
            return struct.unpack('<i', data[:4])[0] if len(data) >= 4 else 0
        elif type_str == "FLOAT":
            return struct.unpack('<f', data[:4])[0] if len(data) >= 4 else 0.0
        elif type_str == "DOUBLE":
            return struct.unpack('<d', data[:8])[0] if len(data) >= 8 else 0.0
        return 0

    def set_cnde_state_config(self, states: List[RobotState], period: int) -> int:
        """设置CNDE状态配置
        
        注意：CNDE运行过程中禁止修改配置。
        如需自定义配置，请在RPC启动前完成配置。
        """
        if not states:
            return self.ERR_NEED_AT_LEAST_ONE_STATE
        if period < 8 or period > 1000:
            return self.ERR_PARAM_VALUE
        
        # 运行过程中禁止修改配置
        if self._robot_state_run_flag and self._tcp_socket:
            print("错误：CNDE运行中，禁止修改配置。请在RPC启动前完成自定义配置。")
            return -6  # 运行时配置错误
        
        self._config_states = states.copy()
        self._robot_state_period = period
        return 0

    def set_cnde_state_period(self, period: int) -> int:
        """设置CNDE状态数据周期"""
        if period < 8 or period > 1000:
            return self.ERR_PARAM_VALUE
        self._robot_state_period = period
        if self._robot_state_run_flag and self._tcp_socket:
            return self._send_cnde_output_config()
        return 0

    def get_cnde_state_config(self) -> tuple:
        """获取当前CNDE配置
        
        Returns:
            tuple: (配置状态列表, 数据周期ms)
        """
        return (self._config_states.copy(), self._robot_state_period)



class RPC():
    ip_address = "192.168.58.2"

    logger = None
    log_output_model = -1
    queue = Queue(maxsize=10000 * 1024)
    logging_thread = None
    is_connect = False
    ROBOT_CNDE_PORT = 20005
    ROBOT_UDP_PORT = 20007
    BUFFER_SIZE = 1024 * 1024
    thread = threading.Thread()
    UDPthread = threading.Thread()
    SDK_state = True

    sock_cli_state_state = False
    closeRPC_state = False
    reconnect_lock = False
    reconnect_flag = False
    g_sock_com_err = RobotError.ERR_SOCKET_COM_FAILED
    
    # 重连参数（默认值）
    _reconnect_enable = True
    _reconnect_max_retries = 30  # 默认最大重连次数
    _reconnect_period = 1000  # 默认1秒（单位ms）

    def __init__(self, ip="192.168.58.2"):
        self.lock = threading.Lock()
        self.ip_address = ip
        link = 'http://' + self.ip_address + ":20003"
        self.robot = xmlrpc.client.ServerProxy(link)

        self.sock_cli_state = None
        self.robot_realstate_exit = False
        self.robot_state_pkg = RobotStatePkg()

        self.stop_event = threading.Event()
        # 禁用20004端口（已用20005 CNDE完全取代）
        # self.connect_to_robot()
        # thread = threading.Thread(target=self.robot_state_routine_thread)
        # thread.daemon = True
        # thread.start()
        # time.sleep(1)
        print(self.robot)

        # 创建UDP客户端（内部实现）
        self._udp_client = FrUdpClient(ip)
        self._udp_client.start_recv_thread()

        # UDP帧计数器 (0-65535循环)
        self._udp_count = 0
        self._udp_count_lock = threading.Lock()

        # CNDE客户端（20005端口）- 完全取代20004端口功能
        print("使用20005 CNDE端口获取状态数据（已取代20004端口）")
        self._com_err_flag = [0]
        # 传入IP地址以支持多机器人隔离配置，传入self以支持断线重连
        self._cnde_client = FRCNDEClient(self.robot_state_pkg, self._com_err_flag, self.ip_address, self)
        cnde_ok = False
        xmlrpc_ok = False
        
        try:
            cnde_rtn = self._cnde_client.connect(self.ip_address, self.ROBOT_CNDE_PORT)
            if cnde_rtn == 0:
                print("CNDE自动连接成功")
                cnde_ok = True
            else:
                print(f"CNDE自动连接失败，错误码: {cnde_rtn}")
        except Exception as e:
            print(f"CNDE自动连接异常: {e}")

        try:
            socket.setdefaulttimeout(1)
            self.robot.GetControllerIP()
            print("XML-RPC连接测试成功")
            xmlrpc_ok = True
        except socket.timeout:
            print("XML-RPC connection timed out.")
        except socket.error as e:
            print("可能是网络故障，请检查网络连接。")
        except Exception as e:
            print("An error occurred during XML-RPC call:", e)
        finally:
            self.robot = None
            socket.setdefaulttimeout(None)
            self.robot = xmlrpc.client.ServerProxy(link)
        
        # 只有CNDE和XML-RPC都成功才设置is_connect = True
        if cnde_ok and xmlrpc_ok:
            RPC.is_connect = True
            print("[调试] RPC连接完全成功，is_connect = True")
        else:
            RPC.is_connect = False
            print(f"[调试] RPC连接失败 (CNDE:{cnde_ok}, XML-RPC:{xmlrpc_ok})，is_connect = False")
        
        self.robot = xmlrpc.client.ServerProxy(link)

    def SetUDPCmdRpyCallback(self, callback: UdpFrameCallback):
        """
        设置UDP帧接收回调函数
        Args:
            callback: 回调函数，格式为 int callback(int srcType, int count, int cmdID, int daLen, string content)
        """
        if hasattr(self, '_udp_client'):
            self._udp_client.set_callback(callback)

    def SendUDPFrame(self, data: str) -> bool:
        """
        发送UDP数据 - 透传，不封装
        Args:
            data: 要发送的字符串数据
        Returns:
            bool: 是否发送成功
        """
        if hasattr(self, '_udp_client'):
            return self._udp_client.send_data(data)
        return False

    def _get_next_udp_count(self):
        """
        获取下一个UDP帧计数，从0递增到65535循环
        Returns:
            int: 下一个计数
        """
        with self._udp_count_lock:
            current = self._udp_count
            self._udp_count = (self._udp_count + 1) % 65536  # 0-65535循环
            return current

    def connect_to_robot(self):
        """连接到机器人CNDE端口(20005)"""
        if self._cnde_client is None:
            return False
        try:
            rtn = self._cnde_client.connect(self.ip_address, self.ROBOT_CNDE_PORT)
            return rtn == 0
        except Exception as ex:
            print(f"CNDE连接失败: {ex}")
            return False
        
    def SetReConnectParam(self, enable: bool, maxRetries: int, period: int) -> int:
        """设置与机器人通讯重连参数
        
        Args:
            enable: 网络故障时使能重连 true-使能 false-不使能
            maxRetries: 最大重连次数
            period: 重连周期，单位ms
            
        Returns:
            错误码 0-成功
        """
        RPC._reconnect_enable = enable
        RPC._reconnect_max_retries = maxRetries
        RPC._reconnect_period = period
        print(f"设置重连参数: enable={enable}, maxRetries={maxRetries}, period={period}ms")
        return 0
    
    def reconnect(self):
        """自动重连"""
        # 如果重连被禁用，直接返回失败
        if not RPC._reconnect_enable:
            print("重连功能已禁用")
            self.SDK_state = False
            return False
        
        max_retries = RPC._reconnect_max_retries
        retry_interval = RPC._reconnect_period / 1000.0  # 转换为秒
        
        self.reconnect_flag = True
        
        for attempt in range(max_retries):
            print(f"尝试重新连接，第 {attempt + 1} 次 / 最大{max_retries}次")
            
            if self.connect_to_robot():
                print("重新连接成功")
                self.SDK_state = True
                self.reconnect_flag = False
                RPC.is_connect = True  # 重连成功时设置连接状态
                return True
            else:
                print(f"重新连接失败，等待 {retry_interval:.1f} 秒后重试...")
                time.sleep(retry_interval)

        print(f"已达到最大重连次数({max_retries}次)，连接失败")
        self.SDK_state = False
        return False

    # def robot_state_routine_thread(self):
    #     """处理机器人状态数据包的线程例程"""

    #     while not self.closeRPC_state:
    #         recvbuf = bytearray(self.BUFFER_SIZE)
    #         tmp_recvbuf = bytearray(self.BUFFER_SIZE)
    #         state_pkg = bytearray(self.BUFFER_SIZE)
    #         find_head_flag = False
    #         index = 0
    #         length = 0
    #         tmp_len = 0
    #         expected_length = self.BUFFER_SIZE  # 初始期望接收长度

    #         try:
    #             while not self.robot_realstate_exit and not self.stop_event.is_set():
    #                 recvbyte = self.sock_cli_state.recv_into(recvbuf)
    #                 # print(f"接收机器人状态字节 {recvbyte}")
    #                 # print("Python 结构体大小:", sizeof(self.robot_state_pkg))
    #                 if recvbyte <= 0:
    #                     self.sock_cli_state.close()
    #                     print("接收机器人状态字节 -1")
    #                     if not self.reconnect():
    #                         return
    #                     continue

    #                 # 处理临时缓冲区数据
    #                 if tmp_len > 0:
    #                     if tmp_len + recvbyte <= self.BUFFER_SIZE:
    #                         recvbuf[:tmp_len + recvbyte] = tmp_recvbuf[:tmp_len] + recvbuf[:recvbyte]
    #                         recvbyte += tmp_len
    #                         tmp_len = 0
    #                     else:
    #                         tmp_len = 0

    #                 i = 0
    #                 while i < recvbyte:
    #                     # 查找包头
    #                     if format(recvbuf[i], '02X') == "5A" and not find_head_flag:
    #                         if i + 4 < recvbyte and format(recvbuf[i + 1], '02X') == "5A":
    #                             find_head_flag = True
    #                             state_pkg[0] = recvbuf[i]
    #                             index = 1
    #                             length = (recvbuf[i + 4] << 8) | recvbuf[i + 3]

    #                             #检查长度是否超过预期
    #                             if length + 7 > expected_length:
    #                                 expected_length = length + 7
    #                                 # 需要接收更多数据
    #                                 tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
    #                                 tmp_len = recvbyte - i
    #                                 find_head_flag = False
    #                                 break

    #                             i += 1
    #                         else:
    #                             i += 1
    #                             continue

    #                     # 已找到包头，收集数据
    #                     elif find_head_flag and index < length + 5:
    #                         if i >= recvbyte:
    #                             break

    #                         state_pkg[index] = recvbuf[i]
    #                         index += 1
    #                         i += 1

    #                     # 检查校验和
    #                     elif find_head_flag and index >= length + 5:
    #                         if i + 1 < recvbyte:
    #                             checksum = sum(state_pkg[:index])
    #                             checkdata = (recvbuf[i + 1] << 8) | recvbuf[i]

    #                             if checksum == checkdata:
    #                                 self.robot_state_pkg = RobotStatePkg.from_buffer_copy(state_pkg[:sizeof(self.robot_state_pkg)])

    #                                 # print(f"@@@@@@{self.robot_state_pkg.toolCoord[0]}")
    #                                 find_head_flag = False
    #                                 index = 0
    #                                 length = 0
    #                                 expected_length = self.BUFFER_SIZE  # 重置期望长度
    #                                 i += 2
    #                             else:
    #                                 # 校验失败处理
    #                                 self.robot_state_pkg.jt_cur_pos[0] = 0
    #                                 self.robot_state_pkg.jt_cur_pos[1] = 0
    #                                 self.robot_state_pkg.jt_cur_pos[2] = 0
    #                                 find_head_flag = False
    #                                 index = 0
    #                                 length = 0
    #                                 i += 2
    #                         else:
    #                             # 数据不足，保存到临时缓冲区
    #                             tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
    #                             tmp_len = recvbyte - i
    #                             break
    #                     else:
    #                         i += 1

    #         except Exception as ex:
    #             if not self.closeRPC_state:
    #                 self.sock_cli_state.close()
    #                 self.sock_cli_state_state = False
    #                 self.SDK_state = False
    #                 # print("SDK读取机器人实时数据失败", ex)
    #                 self.reconnect()

    # ==================== CNDE公共接口 ====================
    def CNDESetStateConfig(self, states: List[RobotState], period: int) -> int:
        """
        设置CNDE状态订阅配置
        Args:
            states: RobotState枚举列表
            period: 数据周期(ms)，范围8-1000
        Returns:
            0-成功，其他-错误码
        """
        if self._cnde_client is None:
            print("CNDE未连接，请先调用CNDEConnect")
            return -1
        return self._cnde_client.set_cnde_state_config(states, period)

    def CNDEAddState(self, state: RobotState) -> int:
        """
        添加单个CNDE状态订阅
        Args:
            state: RobotState枚举值
        Returns:
            0-成功，其他-错误码
        """
        if self._cnde_client is None:
            print("CNDE未连接，请先调用CNDEConnect")
            return -1
        return self._cnde_client.add_cnde_state(state)

    def CNDEDeleteState(self, state: RobotState) -> int:
        """
        删除单个CNDE状态订阅
        Args:
            state: RobotState枚举值
        Returns:
            0-成功，其他-错误码
        """
        if self._cnde_client is None:
            print("CNDE未连接，请先调用CNDEConnect")
            return -1
        return self._cnde_client.delete_cnde_state(state)

    def CNDESetPeriod(self, period: int) -> int:
        """
        设置CNDE数据周期
        Args:
            period: 数据周期(ms)，范围8-1000
        Returns:
            0-成功，其他-错误码
        """
        if self._cnde_client is None:
            print("CNDE未连接，请先调用CNDEConnect")
            return -1
        return self._cnde_client.set_cnde_state_period(period)

    def CNDEGetConfig(self) -> tuple:
        """
        获取当前CNDE配置
        Returns:
            tuple: (配置状态列表, 数据周期)
        """
        if self._cnde_client is None:
            print("CNDE未连接，请先调用CNDEConnect")
            return None
        return self._cnde_client.get_cnde_state_config()

    def CNDESendStart(self) -> int:
        """
        发送CNDE开始指令
        Returns:
            0-成功，其他-错误码
        """
        if self._cnde_client is None:
            print("CNDE未连接，请先调用CNDEConnect")
            return -1
        return self._cnde_client._set_cnde_start()

    def CNDESendStop(self) -> int:
        """
        发送CNDE停止指令
        Returns:
            0-成功，其他-错误码
        """
        if self._cnde_client is None:
            print("CNDE未连接，请先调用CNDEConnect")
            return -1
        return self._cnde_client._set_cnde_stop()

    def CNDEConnect(self) -> int:
        """
        手动连接CNDE（RPC初始化时已自动连接，通常无需调用）
        Returns:
            0-成功，其他-错误码
        """
        if self._cnde_client is None:
            self._com_err_flag = [0]
            # 传入IP地址以支持多机器人隔离配置，传入self以支持断线重连
            self._cnde_client = FRCNDEClient(self.robot_state_pkg, self._com_err_flag, self.ip_address, self)
        return self._cnde_client.connect(self.ip_address, self.ROBOT_CNDE_PORT)

    def CNDEDisconnect(self) -> int:
        """
        断开CNDE连接
        Returns:
            0-成功
        """
        if self._cnde_client is None:
            return 0
        return self._cnde_client.close()

    def setup_logging(self, output_model=1, file_path="", file_num=5):
        """用于处理日志"""
        self.logger = logging.getLogger("RPCLogger")
        log_level = logging.DEBUG
        log_handler = None

        if not file_path:
            current_datetime = datetime.now()
            formatted_date = current_datetime.strftime("%Y%m%d")
            file_name = "fairino_" + formatted_date + ".log"
            file_path = os.path.join(os.getcwd(), file_name)  # 使用当前工作目录，如果没有提供路径的话
        else:
            file_path = os.path.abspath(file_path)  # 获取绝对路径

        # 检查目录是否存在
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            # print(f"Error: The directory '{directory}' does not exist. Logging setup aborted.")
            return -1  # 如果目录不存在，则返回错误码

        if output_model == 0:
            RPC.log_output_model = 0
            log_handler = RotatingFileHandler(file_path, maxBytes=50 * 1024, backupCount=file_num)
        elif output_model == 1:
            RPC.log_output_model = 1
            log_handler = BufferedFileHandler(file_path, mode='a', maxBytes=50 * 1024, backupCount=file_num)
        elif output_model == 2:
            RPC.log_output_model = 2
            log_handler = BufferedFileHandler(file_path, mode='a', maxBytes=50 * 1024, backupCount=file_num)
            self.start_logging_thread(log_handler)

        formatter = logging.Formatter('[%(levelname)s] [%(asctime)s pid:%(process)d]  %(message)s')
        if log_handler:
            log_handler.setFormatter(formatter)
            self.logger.addHandler(log_handler)
        else:
            print("Error: Log handler not created. Logging setup aborted.")

        return 0  # 如果日志记录设置成功，则返回成功码

    def start_logging_thread(self, log_handler):
        """创建线程进行日志存储"""
        logging_thread = LogWriterThread(RPC.queue, log_handler)
        RPC.logging_thread = logging_thread  # 存储日志线程的引用
        logging_thread.start()

    def join_logging_thread(self):
        """通知日志线程停止"""
        if RPC.logging_thread is not None:
            RPC.queue.put(None)  # 通知日志线程停止
            RPC.logging_thread.join()  # 等待日志线程完成

    def __del__(self):
        """垃圾回收器，类似于析构"""
        self.join_logging_thread()#停止日志线程

    def set_log_level(self, lvl):
        """设置日志记录等级"""
        levels = {1: logging.ERROR, 2: logging.WARNING, 3: logging.INFO, 4: logging.DEBUG}
        log_level = levels.get(lvl, logging.DEBUG)
        self.logger.setLevel(log_level)
        return log_level

    def log_call(func):
        """记录函数调用的日志操作"""
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            args_str = ', '.join(map(repr, args))
            kwargs_str = ', '.join([f"{key}={value}" for key, value in kwargs.items()])
            if (kwargs_str) == "":
                call_message = f"Calling {func.__name__}" + f"({args_str}" + ")."
            else:
                call_message = f"Calling {func.__name__}" + f"({args_str}" + "," + f"{kwargs_str})."

            self.log_info(call_message)
            result = func(self, *args, **kwargs)
            if isinstance(result, (list, tuple)) and len(result) > 0:
                if result[0] == 0:
                    self.log_debug(f"{func.__name__} returned: {result}.")
                else:
                    self.log_error(f"{func.__name__} Error occurred. returned: {result}")
            else:
                if result == 0:
                    self.log_debug(f"{func.__name__} returned: {result}.")
                else:
                    self.log_error(f"{func.__name__} Error occurred. returned: {result}")

            return result

        return wrapper

    def log_debug(self, message):
        """用于记录debug等级日志"""
        if self.logger:
            self.logger.debug(message)

    def log_info(self, message):
        """用于记录info等级日志"""
        if self.logger:
            self.logger.info(message)

    def log_warning(self, message):
        """用于记录warning等级日志"""
        if self.logger:
            self.logger.warning(message)

    def log_error(self, message):
        """用于记录errror等级日志"""
        if self.logger:
            self.logger.error(message)

    def send_message(self, message):
        """创建tcp连接发送消息"""
        # 创建一个TCP/IP套接字
        sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        port = 8080  # 固定端口号为8080
        try:
            # 连接到服务器
            sock1.connect((self.ip_address, 8080))
            # 发送数据
            sock1.sendall(message.encode('utf-8'))

            response = sock1.recv(1024).decode('utf-8')

            value =response.split('III')
            if len(value) ==6:
                if value[4] == "1":
                    return 0
                else:
                    print("error happended",value[4])
                    return -1
            else:
                return -1
        except Exception as e:
            print(f'An error occurred: {e}')

        finally:
            sock1.close()

    """2024.12.23"""
    """   
       @brief 安全代码获取
       @return 错误码 成功- 0, 失败-错误码
    """

    def GetSafetyCode(self):
        if (self.robot_state_pkg.safety_stop0_state == 1) or (self.robot_state_pkg.safety_stop1_state == 1):
            return 99
        return 0
    """2024.12.23"""
    """   
    ***************************************************************************机器人基础********************************************************************************************
    """

    """   
    @brief  查询 SDK 版本号
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） version SDK版本号
    """

    @log_call
    @xmlrpc_timeout
    def GetSDKVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = 0
        sdk = ["SDK:V2.2.5", "Robot:V3.9.5"]
        return error, sdk

    """   
    @brief  查询控制器 IP
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回） ip  控制器IP
    """

    @log_call
    @xmlrpc_timeout
    def GetControllerIP(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetControllerIP()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,0

    """   
    @brief  控制机器人手自动模式切换
    @param  [in] 必选参数 state：0-自动模式 1-手动模式
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def Mode(self, state):
        flag = True
        while self.reconnect_flag:
            time.sleep(0.1)

        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.Mode(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  控制机器人进入或退出拖动示教模式
    @param  [in] 必选参数 state：0-退出拖动示教模式, 1-进入拖动示教模式
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def DragTeachSwitch(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)  # 强制转换为int型
        flag = True
        while flag:
            try:
                error = self.robot.DragTeachSwitch(state)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  查询机器人是否处于拖动示教模式
    @param  [in] NULL
    @return 错误码 成功-0，失败-错误码
    @return 返回值（调用成功返回） state 0-非拖动示教模式，1-拖动示教模式
    """

    @log_call
    @xmlrpc_timeout
    def IsInDragTeach(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.IsInDragTeach()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  控制机器人上使能或下使能
    @param  [in] 必选参数 state：0-下使能, 1-上使能
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def RobotEnable(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)  # 强制转换为int型
        flag = True
        while flag:
            try:
                error = self.robot.RobotEnable(state)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    ***************************************************************************机器人运动********************************************************************************************
    """

    """   
    @brief  jog点动
    @param  [in] 必选参数 ref：0-关节点动,2-基坐标系点动,4-工具坐标系点动,8-工件坐标系点动
    @param  [in] 必选参数 nb：1-关节1(或x轴)，2-关节2(或y轴)，3-关节3(或z轴)，4-关节4(或绕x轴旋转)，5-关节5(或绕y轴旋转)，6-关节6(或绕z轴旋转)
    @param  [in] 必选参数 dir：0-负方向，1-正方向
    @param  [in] 必认参数 max_dis：单次点动最大角度/距离，单位 ° 或 mm
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 默认100
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StartJOG(self, ref, nb, dir, max_dis, vel=20.0, acc=100.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        ref = int(ref)  # 强制转换为int型
        nb = int(nb)  # 强制转换为int型
        dir = int(dir)  # 强制转换为int型
        max_dis = float(max_dis)  # 强制转换为float型
        vel = float(vel)  # 强制转换为float型
        acc = float(acc)  # 强制转换为float型
        flag = True
        while flag:
            try:
                error = self.robot.StartJOG(ref, nb, dir, vel, acc, max_dis)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  jog 点动减速停止
    @param  [in] 必选参数：1-关节点动停止,3-基坐标系点动停止,5-工具坐标系点动停止,9-工件坐标系点动停止
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StopJOG(self, ref):
        while self.reconnect_flag:
            time.sleep(0.1)
        ref = int(ref)  # 强制转换为int型
        flag = True
        while flag:
            try:
                error = self.robot.StopJOG(ref)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  jog 点动立即停止
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ImmStopJOG(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ImmStopJOG()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  关节空间运动(自动正/逆运动学计算)
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用正运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放,默认0.0 
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 blendT:[-1.0]-运动到位 (阻塞)，[0~500.0]-平滑时间 (非阻塞)，单位 [ms] 默认-1.0
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveJ(self, joint_pos, tool, user, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
              exaxis_pos=[0.0, 0.0, 0.0, 0.0], blendT=-1.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        exaxis_pos = list(map(float, exaxis_pos))
        blendT = float(blendT)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        if (desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0) and (
                desc_pos[4] == 0.0) and (desc_pos[5] == 0.0):  # 若未输入参数则调用正运动学求解
            ret = self.robot.GetForwardKin(joint_pos)  # 正运动学求解
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, exaxis_pos, blendT, offset_flag,
                                 offset_pos)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  笛卡尔空间直线运动(自动正/逆运动学计算)
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 ovl: 速度缩放因子[0~100]/物理速度（mm/s） 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @param  [in] 默认参数 blendMode 过渡方式；0-内切过渡；1-角点过渡
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 search:[0]-不焊丝寻位，[1]-焊丝寻位
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 oacc 加速度缩放因子[0-100]/物理加速度(mm/s2) 默认 100
    @param  [in] 默认参数 config 逆解关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解，默认-1
    @param  [in] 默认参数 velAccParamMode 速度加速度参数模式；0-百分比；1-物理速度(mm/s)加速度(mm/s2) 默认0
    @param  [in] 默认参数 overSpeedStrategy  超速处理策略，0-策略关闭；1-标准；2-超速时报错停止；3-自适应降速，默认为0
    @param  [in] 默认参数 speedPercent  允许降速阈值百分比[0-100]，默认10%
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveL(self, desc_pos, tool, user, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
              blendR=-1.0, blendMode = 0,exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0, offset_flag=0,
              offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],oacc = 100.0,config=-1,velAccParamMode=0,overSpeedStrategy=0,speedPercent=10):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        blendMode = int(blendMode)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        oacc = float(oacc)
        config = int(config)
        velAccParamMode = int(velAccParamMode)
        overSpeedStrategy = int(overSpeedStrategy)
        speedPercent = int(speedPercent)
        if (overSpeedStrategy > 0):
            error = self.robot.JointOverSpeedProtectStart(overSpeedStrategy, speedPercent)
            if error!=0:
                return error
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error1 = ret[0]
                return error1

        flag = True
        while flag:
            try:
                error1 = self.robot.MoveL([joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5], desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5], tool, user, vel, acc, ovl, blendR, blendMode, exaxis_pos[0],exaxis_pos[1],exaxis_pos[2],exaxis_pos[3], search,offset_flag, offset_pos[0],offset_pos[1],offset_pos[2],offset_pos[3],offset_pos[4],offset_pos[5],oacc,velAccParamMode])
                flag = False
            except socket.error as e:
                flag = True

        if (overSpeedStrategy > 0):
            error = self.robot.JointOverSpeedProtectEnd()
            if error!=0:
                return error

        return error1

    """   
    @brief  笛卡尔空间圆弧运动(自动正/逆运动学计算)
    @param  [in] 必选参数 desc_pos_p: 路径点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_p: 路径点工具号，[0~14]
    @param  [in] 必选参数 user_p: 路径点工件号，[0~14]
    @param  [in] 必选参数 desc_pos_t: 目标点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_t: 工具号，[0~14]
    @param  [in] 必选参数 user_t: 工件号，[0~14]
    @param  [in] 默认参数 joint_pos_p: 路径点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 joint_pos_t: 目标点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel_p: 路径点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_p: 路径点加速度百分比，[0~100] 暂不开放,默认0.0
    @param  [in] 默认参数 exaxis_pos_p: 路径点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 offset_flag_p: 路径点是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos_p: 路径点位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 vel_t: 目标点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_t: 目标点加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 exaxis_pos_t: 目标点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 offset_flag_t: 目标点是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos_t: 目标点位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子[0~100]/物理速度（mm/s） 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @param  [in] 默认参数 oacc 加速度缩放因子[0-100]/物理加速度(mm/s2) 默认 100
    @param  [in] 默认参数 config 逆解关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解，默认-1
    @param  [in] 默认参数 velAccParamMode 速度加速度参数模式；0-百分比；1-物理速度(mm/s)加速度(mm/s2) 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveC(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              vel_p=20.0, acc_p=100.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], offset_flag_p=0,
              offset_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              vel_t=20.0, acc_t=100.0, exaxis_pos_t=[0.0, 0.0, 0.0, 0.0], offset_flag_t=0,
              offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              ovl=100.0, blendR=-1.0,oacc=100.0,config=-1,velAccParamMode=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = int(tool_p)
        user_p = int(user_p)
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))
        offset_flag_p = int(offset_flag_p)
        offset_pos_p = list(map(float, offset_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = int(tool_t)
        user_t = int(user_t)
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))
        offset_flag_t = int(offset_flag_t)
        offset_pos_t = list(map(float, offset_pos_t))

        ovl = float(ovl)
        blendR = float(blendR)
        oacc = float(oacc)
        config = int(config)
        velAccParamMode = int(velAccParamMode)

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, config)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, config)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveC([joint_pos_p[0],joint_pos_p[1],joint_pos_p[2],joint_pos_p[3],joint_pos_p[4],joint_pos_p[5], desc_pos_p[0],desc_pos_p[1],desc_pos_p[2],desc_pos_p[3],desc_pos_p[4],desc_pos_p[5], tool_p, user_p, vel_p, acc_p, exaxis_pos_p[0],exaxis_pos_p[1],exaxis_pos_p[2],exaxis_pos_p[3], offset_flag_p,
                                         offset_pos_p[0],offset_pos_p[1],offset_pos_p[2],offset_pos_p[3],offset_pos_p[4],offset_pos_p[5], joint_pos_t[0],joint_pos_t[1],joint_pos_t[2],joint_pos_t[3],joint_pos_t[4],joint_pos_t[5], desc_pos_t[0],desc_pos_t[1],desc_pos_t[2],desc_pos_t[3],desc_pos_t[4],desc_pos_t[5], tool_t, user_t, vel_t, acc_t, exaxis_pos_t[0],exaxis_pos_t[1],exaxis_pos_t[2],exaxis_pos_t[3],
                                         offset_flag_t, offset_pos_t[0],offset_pos_t[1],offset_pos_t[2],offset_pos_t[3],offset_pos_t[4],offset_pos_t[5], ovl, blendR,oacc,velAccParamMode])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  笛卡尔空间整圆运动(自动正/逆运动学计算)
    @param  [in] 必选参数 desc_pos_p: 路径点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_p: 路径点工具号，[0~14]
    @param  [in] 必选参数 user_p: 路径点工件号，[0~14]
    @param  [in] 必选参数 desc_pos_t: 目标点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_t: 工具号，[0~14]
    @param  [in] 必选参数 user_t: 工件号，[0~14]
    @param  [in] 默认参数 joint_pos_p: 路径点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 joint_pos_t: 目标点关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel_p: 路径点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_p: 路径点加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 exaxis_pos_p: 路径点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 vel_t: 目标点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_t: 目标点加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 exaxis_pos_t: 目标点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子[0~100]/物理速度（mm/s） 默认100.0
    @param  [in] 默认参数 offset_flag: 是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 oacc: 加速度缩放因子[0-100]/物理加速度(mm/s2)，默认：100
    @param  [in] 默认参数 blendR: -1：阻塞；0~1000：平滑半径 默认：-1
    @param  [in] 默认参数 config 逆解关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解，默认-1
    @param  [in] 默认参数 velAccParamMode 速度加速度参数模式；0-百分比；1-物理速度(mm/s)加速度(mm/s2) 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               vel_p=20.0, acc_p=0.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=0.0,
               exaxis_pos_t=[0.0, 0.0, 0.0, 0.0],
               ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], oacc=100.0, blendR=-1,config=-1,velAccParamMode=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = int(tool_p)
        user_p = int(user_p)
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = int(tool_t)
        user_t = int(user_t)
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))

        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        oacc = float(oacc)
        blendR = float(blendR)
        config = int(config)
        velAccParamMode = int(velAccParamMode)

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, config)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, config)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error

        flag = True
        while flag:
            try:
                error = self.robot.Circle([joint_pos_p[0],joint_pos_p[1],joint_pos_p[2],joint_pos_p[3],joint_pos_p[4],joint_pos_p[5], desc_pos_p[0],desc_pos_p[1],desc_pos_p[2],desc_pos_p[3],desc_pos_p[4],desc_pos_p[5], tool_p, user_p, vel_p, acc_p, exaxis_pos_p[0],exaxis_pos_p[1],exaxis_pos_p[2],exaxis_pos_p[3],
                                           joint_pos_t[0],joint_pos_t[1],joint_pos_t[2],joint_pos_t[3],joint_pos_t[4],joint_pos_t[5], desc_pos_t[0],desc_pos_t[1],desc_pos_t[2],desc_pos_t[3],desc_pos_t[4],desc_pos_t[5],
                                          tool_t, user_t, vel_t, acc_t, exaxis_pos_t[0],exaxis_pos_t[1],exaxis_pos_t[2],exaxis_pos_t[3], ovl, offset_flag, offset_pos[0],offset_pos[1],offset_pos[2],offset_pos[3],offset_pos[4],offset_pos[5], oacc, blendR,velAccParamMode])
                flag = False
            except socket.error as e:
                flag = True
        return error

    # @log_call
    # @xmlrpc_timeout
    # def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #            joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #            vel_p=20.0, acc_p=0.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=0.0,
    #            exaxis_pos_t=[0.0, 0.0, 0.0, 0.0],
    #            ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
    #     while self.reconnect_flag:
    #         time.sleep(0.1)
    #     if self.GetSafetyCode() != 0:
    #         return self.GetSafetyCode()
    #     desc_pos_p = list(map(float, desc_pos_p))
    #     tool_p = float(int(tool_p))
    #     user_p = float(int(user_p))
    #     joint_pos_p = list(map(float, joint_pos_p))
    #     vel_p = float(vel_p)
    #     acc_p = float(acc_p)
    #     exaxis_pos_p = list(map(float, exaxis_pos_p))
    #
    #     desc_pos_t = list(map(float, desc_pos_t))
    #     tool_t = float(int(tool_t))
    #     user_t = float(int(user_t))
    #     joint_pos_t = list(map(float, joint_pos_t))
    #     vel_t = float(vel_t)
    #     acc_t = float(acc_t)
    #     exaxis_pos_t = list(map(float, exaxis_pos_t))
    #
    #     ovl = float(ovl)
    #     offset_flag = int(offset_flag)
    #     offset_pos = list(map(float, offset_pos))
    #
    #     if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
    #             and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
    #         retp = self.robot.GetInverseKin(0, desc_pos_p, -1)  # 逆运动学求解
    #         if retp[0] == 0:
    #             joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
    #         else:
    #             error = retp[0]
    #             return error
    #
    #     if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
    #             and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
    #         rett = self.robot.GetInverseKin(0, desc_pos_t, -1)  # 逆运动学求解
    #         if rett[0] == 0:
    #             joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
    #         else:
    #             error = rett[0]
    #             return error
    #
    #     flag = True
    #     while flag:
    #         try:
    #             error = self.robot.Circle(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p,
    #                                       joint_pos_t,
    #                                       desc_pos_t,
    #                                       [tool_t, user_t, vel_t, acc_t], exaxis_pos_t, ovl, offset_flag, offset_pos)
    #             flag = False
    #         except socket.error as e:
    #             flag = True
    #     return error

    """   
    @brief  笛卡尔空间螺旋线运动(自动正/逆运动学计算)
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 param:[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction, velAccMode]circle_num: 螺旋圈数，circle_angle: 螺旋倾角，
    rad_init: 螺旋初始半径，rad_add: 半径增量，rotaxis_add: 转轴方向增量，rot_direction: 旋转方向，0-顺时针，1-逆时针, velAccMode速度加速度参数模式：0-角速度恒定，1-线速度恒定
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 默认100.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 config 逆解关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解，默认-1
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSpiral(self, desc_pos, tool, user, param, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0,
                  exaxis_pos=[0.0, 0.0, 0.0, 0.0],
                  ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        param[0] = int(param[0])
        param[1] = float(param[1])
        param[2] = float(param[2])
        param[3] = float(param[3])
        param[4] = float(param[4])
        param[5] = int(param[5])
        param[6] = int(param[6])
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        exaxis_pos = list(map(float, exaxis_pos))
        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        config = int(config)

        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.NewSpiral([joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5],
                                              desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5],
                                              tool, user, vel, acc, exaxis_pos[0],exaxis_pos[1],exaxis_pos[2],exaxis_pos[3],
                                              ovl, offset_flag,
                                             offset_pos[0],offset_pos[1],offset_pos[2],offset_pos[3],offset_pos[4],offset_pos[5],
                                              float(param[0]),param[1],param[2],param[3],param[4],int(param[5]),param[6]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief  伺服运动开始，配合ServoJ、ServoCart指令使用
    @param  [in] cmdType 命令传输类型，0=XML-RPC，1=UDP透传
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveStart(self, cmdType=0):
        while self.reconnect_flag:
            time.sleep(0.1)

        if cmdType == 0:
            flag = True
            while flag:
                try:
                    error = self.robot.ServoMoveStart()
                    flag = False
                except socket.error as e:
                    flag = True
            return error

        elif cmdType == 1:
            try:
                # 获取下一个计数
                count = self._get_next_udp_count()

                # 构建UDP帧数据，内容直接为ServoMoveStart()
                cmd_str = "ServoMoveStart()"
                content_len = len(cmd_str)

                # 使用固定命令ID，
                cmd_id = 689
                udp_data = "/f/bIII{}III{}III{}III{}III/b/f".format(
                    count, cmd_id, content_len, cmd_str
                )

                #print("UDP发送 - 计数:{} 命令ID:{} 长度:{}".format(count, cmd_id, content_len))

                success = self.SendUDPFrame(udp_data)

                if success:
                    return 0
                else:
                    return -1

            except Exception as e:
                print("UDP发送异常: {}".format(e))
                return -2

        else:
            print("不支持的cmdType: {}".format(cmdType))
            return RobotError.ERR_PARAM_VALUE

    """   
    @brief  伺服运动结束，配合ServoJ、ServoCart指令使用
    @param  [in] cmdType 命令传输类型，0=XML-RPC，1=UDP透传
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveEnd(self, cmdType=0):
        while self.reconnect_flag:
            time.sleep(0.1)

        if cmdType == 0:
            flag = True
            while flag:
                try:
                    error = self.robot.ServoMoveEnd()
                    flag = False
                except socket.error as e:
                    flag = True
            return error

        elif cmdType == 1:
            try:
                # 获取下一个计数
                count = self._get_next_udp_count()

                # 构建UDP帧数据，内容直接为ServoMoveEnd()
                cmd_str = "ServoMoveEnd()"
                content_len = len(cmd_str)

                # 使用固定命令ID，
                cmd_id = 690
                udp_data = "/f/bIII{}III{}III{}III{}III/b/f".format(
                    count, cmd_id, content_len, cmd_str
                )

                #print("UDP发送 - 计数:{} 命令ID:{} 长度:{}".format(count, cmd_id, content_len))

                success = self.SendUDPFrame(udp_data)

                if success:
                    return 0
                else:
                    return -1

            except Exception as e:
                print("UDP发送异常: {}".format(e))
                return -2

        else:
            print("不支持的cmdType: {}".format(cmdType))
            return RobotError.ERR_PARAM_VALUE

    """   
    @brief  关节空间伺服模式运动
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 axisPos  外部轴位置,单位mm
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 cmdT: 指令下发周期，单位s，建议范围[0.001~0.0016], 默认为0.008
    @param  [in] 默认参数 filterT: 滤波时间，单位 [s]，暂不开放， 默认为0.0
    @param  [in] 默认参数 gain: 目标位置的比例放大器，暂不开放， 默认为0.0
    @param  [in] 默认参数 id: servoJ指令ID,默认为0
    @param  [in] cmdType 命令传输类型，0=XML-RPC，1=UDP透传
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJ(self, joint_pos, axisPos, acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0, id=0, cmdType=0):
        while self.reconnect_flag:
            time.sleep(0.1)

        """
        关节空间伺服运动
        Args:
            joint_pos: 关节位置列表
            axisPos: 轴位置列表
            acc: 加速度
            vel: 速度
            cmdT: 命令周期
            filterT: 滤波时间
            gain: 增益
            id: 命令ID
            cmdType: 命令传输类型，0=XML-RPC，1=UDP透传

        Returns:
            错误码
        """
        # 处理重连标志
        while self.reconnect_flag:
            time.sleep(0.1)

        # 安全检查
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()

        # 参数类型转换
        joint_pos = list(map(float, joint_pos))
        axisPos = list(map(float, axisPos))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        id = int(id)

        # 根据cmdType选择传输方式
        if cmdType == 0:
            # 使用原有的XML-RPC接口
            flag = True
            while flag:
                try:
                    error = self.robot.ServoJ(joint_pos, axisPos, acc, vel, cmdT, filterT, gain, id)
                    flag = False
                except socket.error as e:
                    flag = True
            return error

        elif cmdType == 1:
            # 使用UDP透传接口
            try:
                # 获取下一个计数
                count = self._get_next_udp_count()

                # 构建ServoJ命令字符串
                joint_str = ",".join(["{:.4f}".format(j) for j in joint_pos])
                axis_str = ",".join(["{:.4f}".format(a) for a in axisPos])

                # 构建完整命令
                cmd_str = "ServoJ({},{},{},{},{},{},{},{})".format(
                    joint_str, axis_str,
                    "{:.4f}".format(acc),
                    "{:.4f}".format(vel),
                    "{:.4f}".format(cmdT),
                    "{:.4f}".format(filterT),
                    "{:.4f}".format(gain),
                    id
                )

                # 构建UDP帧数据
                content_len = len(cmd_str)
                udp_data = "/f/bIII{}III{}III{}III{}III/b/f".format(
                    count, 376, content_len, cmd_str
                )

                #print("UDP发送 - 计数:{} 命令ID:{} 长度:{}".format(count, 376, content_len))

                # 通过UDP发送
                success = self.SendUDPFrame(udp_data)

                if success:
                    return 0
                else:
                    return -1

            except Exception as e:
                print("UDP发送异常: {}".format(e))
                return -2

        else:
            print("不支持的cmdType: {}".format(cmdType))
            return RobotError.ERR_PARAM_VALUE

    """   
    @brief  笛卡尔空间伺服模式运动
    @param  [in] 必选参数 mode:[0]-绝对运动 (基坐标系)，[1]-增量运动 (基坐标系)，[2]-增量运动(工具坐标系)
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位置/目标笛卡尔位置增量
    @param  [in] 必选参数 exaxis 扩展轴位置
    @param  [in] 默认参数 pos_gain: 位姿增量比例系数，仅在增量运动下生效，范围 [0~1], 默认为 [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 cmdT:指令下发周期，单位s，建议范围[0.001~0.0016], 默认为0.008
    @param  [in] 默认参数 filterT: 滤波时间，单位 [s]，暂不开放， 默认为0.0
    @param  [in] 默认参数 gain: 目标位置的比例放大器，暂不开放， 默认为0.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoCart(self, mode, desc_pos, exaxis, pos_gain=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], acc=0.0, vel=0.0, cmdT=0.008,
                  filterT=0.0, gain=0.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        mode = int(mode)
        desc_pos = list(map(float, desc_pos))
        exaxis = list(map(float, exaxis))
        pos_gain = list(map(float, pos_gain))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        flag = True
        while flag:
            try:
                error = self.robot.ServoCart(mode, desc_pos, pos_gain, exaxis, acc, vel, cmdT, filterT, gain)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  关节扭矩控制开始
    @param  [in] cmdType 命令传输类型，0=XML-RPC，1=UDP透传
    @return 错误码 成功-0  失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def ServoJTStart(self, cmdType=0):
        while self.reconnect_flag:
            time.sleep(0.1)

        if cmdType == 0:
            flag = True
            while flag:
                try:
                    error = self.robot.ServoJTStart()
                    flag = False
                except socket.error as e:
                    flag = True
            return error

        elif cmdType == 1:
            try:
                count = self._get_next_udp_count()
                cmd_str = "ServoJTStart()"
                content_len = len(cmd_str)
                cmd_id = 1199
                udp_data = "/f/bIII{}III{}III{}III{}III/b/f".format(
                    count, cmd_id, content_len, cmd_str
                )
                #print("UDP发送 - 计数:{} 命令ID:{} 长度:{}".format(count, cmd_id, content_len))
                success = self.SendUDPFrame(udp_data)
                if success:
                    return 0
                else:
                    return -1
            except Exception as e:
                print("UDP发送异常: {}".format(e))
                return -2
        else:
            print("不支持的cmdType: {}".format(cmdType))
            return RobotError.ERR_PARAM_VALUE

    """   
    @brief  关节扭矩控制
    @param  [in] 必选参数 torque j1~j6关节扭矩，单位Nm
    @param  [in] 必选参数 interval 指令周期，单位s，范围[0.001~0.008]
    @param  [in] 默认参数 checkFlag 检测策略 0-不限制；1-限制功率；2-限制速度；3-功率和速度同时限制,默认0
    @param  [in] 默认参数 jPowerLimit 关节最大功率限制(W)，默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 jVelLimit 关节最大速度(°/s)，默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 命令传输类型，0=XML-RPC，1=UDP透传
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJT(self, torque, interval, checkFlag=0, jPowerLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                jVelLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], cmdType=0):
        # 处理重连标志
        while self.reconnect_flag:
            time.sleep(0.1)

        # 安全检查
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()

        # 参数类型转换
        torque = list(map(float, torque))
        interval = float(interval)
        checkFlag = int(checkFlag)
        jPowerLimit = list(map(float, jPowerLimit))
        jVelLimit = list(map(float, jVelLimit))

        # 根据cmdType选择传输方式
        if cmdType == 0:
            # 使用原有的XML-RPC接口
            flag = True
            while flag:
                try:
                    error = self.robot.ServoJT(torque, interval, checkFlag, jPowerLimit, jVelLimit)
                    flag = False
                except socket.error as e:
                    flag = True
            return error

        elif cmdType == 1:
            # 使用UDP透传接口
            try:
                # 获取下一个计数
                count = self._get_next_udp_count()

                # 构建ServoJT命令字符串
                torque_str = "{" + ",".join(["{:.4f}".format(t) for t in torque]) + "}"
                power_str = "{" + ",".join(["{:.4f}".format(p) for p in jPowerLimit]) + "}"
                vel_str = "{" + ",".join(["{:.4f}".format(v) for v in jVelLimit]) + "}"

                # 构建完整命令
                cmd_str = "ServoJT({},{},{},{},{})".format(
                    torque_str,
                    "{:.4f}".format(interval),
                    checkFlag,
                    power_str,
                    vel_str
                )

                # 构建UDP帧数据
                content_len = len(cmd_str)
                udp_data = "/f/bIII{}III{}III{}III{}III/b/f".format(
                    count, 1200, content_len, cmd_str  # 假设ServoJT使用命令ID=100
                )

                #print("UDP发送 - 计数:{} 命令ID:{} 长度:{}".format(count, 100, content_len))

                # 通过UDP发送
                success = self.SendUDPFrame(udp_data)

                if success:
                    return 0
                else:
                    return -1

            except Exception as e:
                print("UDP发送异常: {}".format(e))
                return -2

        else:
            print("不支持的cmdType: {}".format(cmdType))
            return RobotError.ERR_PARAM_VALUE

    """   
    @brief  关节扭矩控制结束
    @param  [in] cmdType 命令传输类型，0=XML-RPC，1=UDP透传
    @return 错误码 成功-0  失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def ServoJTEnd(self, cmdType=0):
        while self.reconnect_flag:
            time.sleep(0.1)

        if cmdType == 0:
            flag = True
            while flag:
                try:
                    error = self.robot.ServoJTEnd()
                    flag = False
                except socket.error as e:
                    flag = True
            return error

        elif cmdType == 1:
            try:
                count = self._get_next_udp_count()
                cmd_str = "ServoJTEnd()"
                content_len = len(cmd_str)
                cmd_id = 1201
                udp_data = "/f/bIII{}III{}III{}III{}III/b/f".format(
                    count, cmd_id, content_len, cmd_str
                )
                #print("UDP发送 - 计数:{} 命令ID:{} 长度:{}".format(count, cmd_id, content_len))
                success = self.SendUDPFrame(udp_data)
                if success:
                    return 0
                else:
                    return -1
            except Exception as e:
                print("UDP发送异常: {}".format(e))
                return -2
        else:
            print("不支持的cmdType: {}".format(cmdType))
            return RobotError.ERR_PARAM_VALUE

    """   
    @brief  笛卡尔空间点到点运动
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位置/目标笛卡尔位置增量
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，默认为 20.0
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放,默认为 0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100，默认为 100.0
    @param  [in] 默认参数 blendT:[-1.0]-运动到位 (阻塞)，[0~500]-平滑时间 (非阻塞)，单位 [ms] 默认为 -1.0
    @param  [in] 默认参数 config: 关节配置，[-1]-参考当前关节位置求解，[0~7]-依据关节配置求解 默认为 -1
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveCart(self, desc_pos, tool, user, vel=20.0, acc=0.0, ovl=100.0, blendT=-1.0, config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendT = float(blendT)
        config = int(config)
        flag = True
        while flag:
            try:
                error = self.robot.MoveCart(desc_pos, tool, user, vel, acc, ovl, blendT, config)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  样条运动开始
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SplineStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SplineStart()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  样条运动 PTP(自动正/逆运动学计算)
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用正运动学求解返回值
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，默认为 20.0
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，默认为 100.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100，默认为 100.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SplinePTP(self, joint_pos, tool, user, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=100.0, ovl=100.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        if ((desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0)
                and (desc_pos[4] == 0.0) and (desc_pos[5] == 0.0)):  # 若未输入参数则调用正运动学求解
            ret = self.robot.GetForwardKin(joint_pos)  # 正运动学求解
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.SplinePTP(joint_pos, desc_pos, tool, user, vel, acc, ovl)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  样条运动结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SplineEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SplineEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  新样条运动开始
    @param  [in] 必选参数 type:0-圆弧过渡，1-给定点位路径点
    @param  [in] 默认参数 averageTime: 全局平均衔接时间（ms）默认为 2000
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSplineStart(self, type, averageTime=2000):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        averageTime = int(averageTime)
        flag = True
        while flag:
            try:
                error = self.robot.NewSplineStart(type, averageTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  新样条指令点(自动正/逆运动学计算)
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 lastFlag: 是否为最后一个点，0-否，1-是
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认为 100.0
    @param  [in] 默认参数 blendR: [0~1000]-平滑半径，单位 [mm] 默认0.0
    @param  [in] 默认参数 config: 逆解关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解,默认-1
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSplinePoint(self, desc_pos, tool, user, lastFlag, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.0,
                       acc=0.0, ovl=100.0, blendR=0.0, config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        lastFlag = int(lastFlag)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        config = int(config)
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.NewSplinePoint(joint_pos, desc_pos, tool, user, vel, acc, ovl, blendR, lastFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  新样条运动结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSplineEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.NewSplineEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  终止运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StopMotion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.StopMotion()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  暂停运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PauseMotion(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # error = self.robot.PauseMotion()
        self.send_message("/f/bIII0III103III5IIIPAUSEIII/b/f")
        return 0

        # return error

    """   
    @brief  恢复运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ResumeMotion(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        # error = self.robot.ResumeMotion()
        error = self.send_message("/f/bIII0III104III6IIIRESUMEIII/b/f")
        return error

    """   
    @brief  点位整体偏移开始
    @param  [in] 必选参数 flag:0-基坐标或工件坐标系下偏移，2-工具坐标系下偏移
    @param  [in] 必选参数 offset_pos: 偏移量，单位 [mm][°]。
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointsOffsetEnable(self, flag, offset_pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        offset_pos = list(map(float, offset_pos))
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.PointsOffsetEnable(flag, offset_pos)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  点位整体偏移结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointsOffsetDisable(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.PointsOffsetDisable()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************机器人IO********************************************************************************************
    """

    """   
    @brief  设置控制箱数字量输出
    @param  [in] 必选参数 id:io 编号，范围 [0~15]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 默认参数 smooth:0-不平滑，1-平滑 默认0
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetDO(self, id, status, smooth=0, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        status = int(status)
        smooth = int(smooth)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetDO(id, status, smooth, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置工具数字量输出
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 默认参数 smooth:0-不平滑，1-平滑 默认0
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolDO(self, id, status, smooth=0, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        status = int(status)
        smooth = int(smooth)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolDO(id, status, smooth, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置控制箱模拟量输出
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 value: 电流或电压值百分比，范围 [0~100%] 对应电流值 [0~20mA] 或电压 [0~10V]；
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAO(self, id, value, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        value = float(value)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetAO(id, value * 40.95, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置工具模拟量输出
    @param  [in] 必选参数 id:io 编号，范围 [0]
    @param  [in] 必选参数 value: 电流或电压值百分比，范围 [0~100%] 对应电流值 [0~20mA] 或电压 [0~10V]；
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolAO(self, id, value, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        value = float(value)
        block = int(block)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolAO(id, value * 40.95, block)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取控制箱数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0-15]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回）di: 0-低电平，1-高电平
    """

    @log_call
    # @xmlrpc_timeout
    def GetDI(self, id, block=0):
        id = int(id)
        block = int(block)
        # _error = self.robot.GetDI(id, block)
        # error = _error[0]
        # print(_error)
        # if _error[0] == 0:
        #     di = _error[1]
        #     return error, di
        # else:
        #     return error
        if 0 <= id < 8:
            level = (self.robot_state_pkg.cl_dgt_input_l & (0x01 << id)) >> id
            return 0, level
        elif 8 <= id < 16:
            id -= 8
            level = (self.robot_state_pkg.cl_dgt_input_h & (0x01 << id)) >> id
            return 0, level
        else:
            return -1,None


    """   
    @brief  获取工具数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）di: 0-低电平，1-高电平
    """

    @log_call
    # @xmlrpc_timeout
    def GetToolDI(self, id, block=0):
        id = int(id)
        block = int(block)
        # _error = self.robot.GetToolDI(id, block)
        # error = _error[0]
        # if _error[0] == 0:
        #     di = _error[1]
        #     return error, di
        # else:
        #     return error
        if 0 <= id < 2:
            id+=1
            level = (self.robot_state_pkg.tl_dgt_input_l & (0x01 << id)) >> id
            return 0,level
        else:
            return -1,None

    """   
    @brief  等待控制箱数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~15]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitDI(self, id, status, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitDI(id, status, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  等待控制箱多路数字量输入
    @param  [in] 必选参数 mode 0-多路与，1-多路或
    @param  [in] 必选参数 id  io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitMultiDI(self, mode, id, status, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        id = int(id)
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitMultiDI(mode, id, status, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  等待工具数字量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 status:0-关，1-开
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitToolDI(self, id, status, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        id = id+1 #控制器内部1对应di0,2对应di1
        status = int(status)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitToolDI(id, status, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取控制箱模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    """

    @log_call
    @xmlrpc_timeout
    def GetAI(self, id, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)

        id = int(id)
        block = int(block)
        # _error = self.robot.GetAI(id, block)
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error
        if 0 <= id < 2:
            return 0,self.robot_state_pkg.cl_analog_input[id] / 40.95
        else:
            return -1


    """   
    @brief  获取工具模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0]
    @param  [in] 默认参数 block:0-阻塞，1-非阻塞 默认0
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    """

    @log_call
    @xmlrpc_timeout
    def GetToolAI(self, id, block=0):
        while self.reconnect_flag:
            time.sleep(0.1)

        id = int(id)
        block = int(block)
        # _error = self.robot.GetToolAI(id, block)
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error
        return 0, self.robot_state_pkg.tl_anglog_input / 40.95

    """   
    @brief  获取机器人末端点记录按钮状态
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）按钮状态，0-按下，1-松开
    """

    @log_call
    @xmlrpc_timeout
    def GetAxlePointRecordBtnState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # while self.reconnect_flag:
        #     time.sleep(0.1)
        # flag = True
        # while flag:
        #     try:
        #         _error = self.robot.GetAxlePointRecordBtnState()
        #         flag = False
        #     except socket.error as e:
        #         flag = True
        #
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error,None
        return 0,(self.robot_state_pkg.tl_dgt_input_l & 0x10) >> 4


    """   
    @brief  获取机器人末端DO输出状态
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）do_state DO输出状态，do0~do1对应bit1~bit2,从bit0开始
    """

    @log_call
    @xmlrpc_timeout
    def GetToolDO(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.GetToolDO()
        # error = _error[0]
        # if _error[0] == 0:
        #     value = _error[1]
        #     return error, value
        # else:
        #     return error
        return 0,self.robot_state_pkg.tl_dgt_output_l

    """   
    @brief  获取机器人控制器DO输出状态
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）do_state_h DO输出状态，co0~co7对应bit0~bit7 do_state_l DO输出状态，do0~do7对应bit0~bit7
    """

    @log_call
    @xmlrpc_timeout
    def GetDO(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.GetDO()
        # error = _error[0]
        # if _error[0] == 0:
        #     do_state_h = _error[1]
        #     do_state_l = _error[2]
        #     return error, [do_state_h, do_state_l]
        # else:
        #     return error
        return 0, [self.robot_state_pkg.cl_dgt_output_h,self.robot_state_pkg.cl_dgt_output_l]

    """   
    @brief  等待控制箱模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0~1]
    @param  [in] 必选参数 sign:0-大于，1-小于
    @param  [in] 必选参数 value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitAI(self, id, sign, value, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        sign = int(sign)
        value = float(value)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitAI(id, sign, value*40.95, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  等待工具模拟量输入
    @param  [in] 必选参数 id:io 编号，范围 [0]
    @param  [in] 必选参数 sign:0-大于，1-小于
    @param  [in] 必选参数 value: 输入电流或电压值百分比，范围 [0~100] 对应电流值 [0~20mA] 或电压 [0~10V]
    @param  [in] 必选参数 maxtime: 最大等待时间，单位 [ms]
    @param  [in] 必选参数 opt: 超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitToolAI(self, id, sign, value, maxtime, opt):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        sign = int(sign)
        value = float(value)
        maxtime = int(maxtime)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.WaitToolAI(id, sign, value*40.95, maxtime, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************机器人常用设置********************************************************************************************
    """

    """   
    @brief  设置全局速度
    @param  [in] 必选参数 vel  速度百分比，范围[0~100]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetSpeed(self, vel):
        while self.reconnect_flag:
            time.sleep(0.1)
        vel = int(vel)
        flag = True
        while flag:
            try:
                error = self.robot.SetSpeed(vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置系统变量
    @param  [in] 必选参数 id：变量编号，范围 [1~20]
    @param  [in] 必选参数 value：变量值
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetSysVarValue(self, id, value):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        value = float(value)
        flag = True
        while flag:
            try:
                error = self.robot.SetSysVarValue(id, value)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置工具参考点-六点法
    @param  [in] 必选参数 point_num 点编号,范围[1~6] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算工具坐标系-六点法
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）tcp_pose [x,y,z,rx,ry,rz] 工具坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeTool(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeTool()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  设置工具参考点-四点法
    @param  [in] 必选参数 point_num 点编号,范围[1~4] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTcp4RefPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetTcp4RefPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算工具坐标系-四点法
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）tcp_pose [x,y,z,rx,ry,rz]  工具坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeTcp4(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeTcp4()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  设置工具坐标系
    @param  [in] 必选参数 id: 坐标系编号，范围 [1~15]
    @param  [in] 必选参数 t_coord:[x,y,z,rx,ry,rz]  工具中心点相对末端法兰中心位姿，单位 [mm][°]
    @param  [in] 必选参数 type:0-工具坐标系，1-传感器坐标系
    @param  [in] 必选参数 install: 安装位置，0-机器人末端，1-机器人外部
    @param  [in] 必选参数 toolID: 工具ID
    @param  [in] 必选参数 loadNum: 负载编号
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolCoord(self, id, t_coord, type, install, toolID, loadNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        t_coord = list(map(float, t_coord))
        type = int(type)
        install = int(install)
        toolID = int(toolID)
        loadNum = int(loadNum)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolCoord(id, t_coord, type, install, toolID, loadNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置工具坐标系列表
    @param  [in] 必选参数 id: 坐标系编号，范围 [1~15]
    @param  [in] 必选参数 t_coord:[x,y,z,rx,ry,rz]  工具中心点相对末端法兰中心位姿，单位 [mm][°]
    @param  [in] 必选参数 type:0-工具坐标系，1-传感器坐标系
    @param  [in] 必选参数 install: 安装位置，0-机器人末端，1-机器人外部
    @param  [in] 必选参数 loadNum: 负载编号
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolList(self, id, t_coord, type, install , loadNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        t_coord = list(map(float, t_coord))
        type = int(type)
        install = int(install)
        loadNum = int(loadNum)
        flag = True
        while flag:
            try:
                error = self.robot.SetToolList(id, t_coord, type, install, loadNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置外部工具参考点-三点法
    @param  [in] 必选参数 point_num 点编号,范围[1~3] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExTCPPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetExTCPPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算外部工具坐标系-三点法
    @param  [in] NULL
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）tcp_pose [x,y,z,rx,ry,rz] 外部工具坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeExTCF(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeExTCF()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  设置外部工具坐标系
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 etcp: [x,y,z,rx,ry,rz] 外部工具坐标系，单位 [mm][°]
    @param  [in] 必选参数 etool: [x,y,z,rx,ry,rz] 末端工具坐标系，单位 [mm][°]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExToolCoord(self, id, etcp, etool):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        etcp = list(map(float, etcp))
        etool = list(map(float, etool))
        flag = True
        while flag:
            try:
                error = self.robot.SetExToolCoord(id, etcp, etool)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置外部工具坐标系列表
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 etcp: [x,y,z,rx,ry,rz] 外部工具坐标系，单位 [mm][°]
    @param  [in] 必选参数 etool: [x,y,z,rx,ry,rz] 末端工具坐标系，单位 [mm][°]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExToolList(self, id, etcp, etool):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        etcp = list(map(float, etcp))
        etool = list(map(float, etool))
        flag = True
        while flag:
            try:
                error = self.robot.SetExToolList(id, etcp, etool)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置工件参考点-三点法
    @param  [in] 必选参数 point_num 点编号,范围[1~3] 
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjCoordPoint(self, point_num):
        while self.reconnect_flag:
            time.sleep(0.1)
        point_num = int(point_num)
        flag = True
        while flag:
            try:
                error = self.robot.SetWObjCoordPoint(point_num)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算工件坐标系
    @param  [in] method 计算方式 0：原点-x轴-z轴  1：原点-x轴-xy平面
    @param  [in] refFrame 参考坐标系
    @return 错误码 成功-0,  失败-错误码
    @return 返回值（调用成功返回）wobj_pose [x,y,z,rx,ry,rz] 工件坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ComputeWObjCoord(self, method, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeWObjCoord(method, refFrame)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  设置工件坐标系
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 coord: 工件坐标系相对于末端法兰中心位姿，单位 [mm][°]
    @param  [in] 必选参数 refFrame: 参考坐标系
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjCoord(self, id, coord, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        coord = list(map(float, coord))
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                error = self.robot.SetWObjCoord(id, coord, refFrame)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置工件坐标系列表
    @param  [in] 必选参数 id: 坐标系编号，范围 [0~14]
    @param  [in] 必选参数 coord: 工件坐标系相对于末端法兰中心位姿，单位 [mm][°]
    @param  [in] 必选参数 refFrame: 参考坐标系
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWObjList(self, id, coord, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        coord = list(map(float, coord))
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                error = self.robot.SetWObjList(id, coord, refFrame)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置末端负载重量
    @param  [in] 必选参数 loadNum 负载编号
    @param  [in] 必选参数 weight: 单位 [kg]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoadWeight(self, loadNum, weight):
        while self.reconnect_flag:
            time.sleep(0.1)
        loadNum = int(loadNum)
        weight = float(weight)
        flag = True
        while flag:
            try:
                error = self.robot.SetLoadWeight(loadNum,weight)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置机器人安装方式-固定安装
    @param  [in] 必选参数 method:0-正装，1-侧装，2-挂装
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotInstallPos(self, method):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotInstallPos(method)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置机器人安装角度
    @param  [in] 必选参数 yangle：倾斜角
    @param  [in] 必选参数 zangle：旋转角
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotInstallAngle(self, yangle, zangle):
        while self.reconnect_flag:
            time.sleep(0.1)
        yangle = float(yangle)
        zangle = float(zangle)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotInstallAngle(yangle, zangle)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置末端负载质心坐标
    @param  [in] 必选参数 x: 质心坐标，单位 [mm]
    @param  [in] 必选参数 y: 质心坐标，单位 [mm]
    @param  [in] 必选参数 z: 质心坐标，单位 [mm]
    @param  [in] 默认参数 loadNum 负载编号，默认0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoadCoord(self, x, y, z, loadNum = 0):
        while self.reconnect_flag:
            time.sleep(0.1)
        x = float(x)
        y = float(y)
        z = float(z)
        loadNum = int(loadNum)
        flag = True
        while flag:
            try:
                error = self.robot.SetLoadCoord(x, y, z, loadNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  等待指定时间
    @param  [in] 必选参数 t_ms: 单位 [ms]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitMs(self, t_ms):
        while self.reconnect_flag:
            time.sleep(0.1)
        t_ms = int(t_ms)
        flag = True
        while flag:
            try:
                error = self.robot.WaitMs(t_ms)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************机器人安全设置********************************************************************************************
    """

    """   
    @brief  设置碰撞等级
    @param  [in] 必选参数 mode:0-等级，1-百分比
    @param  [in] 必选参数 level=[j1,j2,j3,j4,j5,j6]: 碰撞阈值 mode=0时，范围：1-10 对应mode=1时，范围0-100%
    @param  [in] 必选参数 config:0-不更新配置文件，1-更新配置文件
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAnticollision(self, mode, level, config):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        level = list(map(float, level))
        config = int(config)
        flag = True
        while flag:
            try:
                error = self.robot.SetAnticollision(mode, level, config)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置碰撞后策略
    @param  [in] 必选参数 strategy：0-报错暂停，1-继续运行，2-报错停止，3-重力矩模式，4-震荡相应模式，5-碰撞回弹模式
    @param  [in] 默认参数 safeTime：安全停止时间[1000-2000]ms，默认为：1000
    @param  [in] 默认参数 safeDistance：安全停止距离[1-150]mm，默认为：100
    @param  [in] 默认参数 safeVel：安全停止速度[50-250]mm/s，默认为：250
    @param  [in] 默认参数 safetyMargin[6]：安全系数[1-10]，默认为：[10,10,10,10,10,10]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetCollisionStrategy(self, strategy,safeTime=1000,safeDistance=100,safeVel=250,safetyMargin=[10,10,10,10,10,10]):
        while self.reconnect_flag:
            time.sleep(0.1)
        strategy = int(strategy)
        safeTime = int(safeTime)
        safeDistance = int(safeDistance)
        safeVel = int(safeVel)
        safetyMargin = list(map(int, safetyMargin))
        flag = True
        while flag:
            try:
                error = self.robot.SetCollisionStrategy(strategy,safeTime,safeDistance,safeVel,safetyMargin)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置正限位
    @param  [in] 必选参数 p_limit=[j1,j2,j3,j4,j5,j6]：六个关节位置
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLimitPositive(self, p_limit):
        while self.reconnect_flag:
            time.sleep(0.1)
        p_limit = list(map(float, p_limit))
        flag = True
        while flag:
            try:
                error = self.robot.SetLimitPositive(p_limit)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置负限位
    @param  [in] 必选参数 n_limit=[j1,j2,j3,j4,j5,j6]：六个关节位置
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLimitNegative(self, n_limit):
        while self.reconnect_flag:
            time.sleep(0.1)
        n_limit = list(map(float, n_limit))
        flag = True
        while flag:
            try:
                error = self.robot.SetLimitNegative(n_limit)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  错误状态清除，只能清除可复位的错误
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ResetAllError(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ResetAllError()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  关节摩擦力补偿开关
    @param  [in] 必选参数 state：0-关，1-开
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FrictionCompensationOnOff(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.FrictionCompensationOnOff(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置关节摩擦力补偿系数-固定安装-正装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_level(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_level(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置关节摩擦力补偿系数-固定安装-侧装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_wall(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_wall(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置关节摩擦力补偿系数-固定安装-倒装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_ceiling(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_ceiling(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置关节摩擦力补偿系数-自由安装
    @param  [in] 必选参数 coeff=[j1,j2,j3,j4,j5,j6]：六个关节补偿系数
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFrictionValue_freedom(self, coeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        coeff = list(map(float, coeff))
        flag = True
        while flag:
            try:
                error = self.robot.SetFrictionValue_freedom(coeff)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************机器人状态查询********************************************************************************************
    """

    """   
    @brief  获取机器人安装角度
    @param  [in] NULL
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回）[yangle,zangle] yangle-倾斜角,zangle-旋转角
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotInstallAngle(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotInstallAngle()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error,None

    """   
    @brief  获取系统变量值
    @param  [in] id：系统变量编号，范围 [1~20]
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） var_value：系统变量值
    """

    @log_call
    @xmlrpc_timeout
    def GetSysVarValue(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSysVarValue(id)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  获取当前关节位置 (角度)
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointPosDegree(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        _error = self.robot.GetActualJointPosDegree(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error
        # return 0,[self.robot_state_pkg.jt_cur_pos[0],self.robot_state_pkg.jt_cur_pos[1],self.robot_state_pkg.jt_cur_pos[2],
        #           self.robot_state_pkg.jt_cur_pos[3],self.robot_state_pkg.jt_cur_pos[4],self.robot_state_pkg.jt_cur_pos[5]]
    """   
    @brief  获取关节当前位置 (弧度)
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointPosRadian(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetActualJointPosRadian(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  获取关节反馈速度-deg/s
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） speed=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointSpeedsDegree(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        # _error = self.robot.GetActualJointSpeedsDegree(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.actual_qd[0],self.robot_state_pkg.actual_qd[1],self.robot_state_pkg.actual_qd[2],
                  self.robot_state_pkg.actual_qd[3],self.robot_state_pkg.actual_qd[4],self.robot_state_pkg.actual_qd[5]]

    """   
    @brief  获取关节反馈加速度-deg/s^2
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） acc=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualJointAccDegree(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        # _error = self.robot.GetActualJointAccDegree(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.actual_qdd[0],self.robot_state_pkg.actual_qdd[1],self.robot_state_pkg.actual_qdd[2],
                  self.robot_state_pkg.actual_qdd[3],self.robot_state_pkg.actual_qdd[4],self.robot_state_pkg.actual_qdd[5]]

    """   
    @brief  获取TCP指令合速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回）[tcp_speed,ori_speed] tcp_speed 线性合速度 ori_speed 姿态合速度 
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetTCPCompositeSpeed(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetTargetTCPCompositeSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.target_TCP_CmpSpeed[0],self.robot_state_pkg.target_TCP_CmpSpeed[1]]

    """   
    @brief  获取TCP反馈合速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回）[tcp_speed,ori_speed] tcp_speed 线性合速度 ori_speed 姿态合速度 
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPCompositeSpeed(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetActualTCPCompositeSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error
        return 0, [self.robot_state_pkg.actual_TCP_CmpSpeed[0], self.robot_state_pkg.actual_TCP_CmpSpeed[1]]

    """   
    @brief  获取TCP指令速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞  默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） speed [x,y,z,rx,ry,rz]速度 mm/s
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetTCPSpeed(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetTargetTCPSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.target_TCP_Speed[0],self.robot_state_pkg.target_TCP_Speed[1],self.robot_state_pkg.target_TCP_Speed[2],
                  self.robot_state_pkg.target_TCP_Speed[3],self.robot_state_pkg.target_TCP_Speed[4],self.robot_state_pkg.target_TCP_Speed[5]]

    """   
    @brief  获取TCP反馈速度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞  默认1
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） speed [x,y,z,rx,ry,rz]速度
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPSpeed(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetActualTCPSpeed(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.actual_TCP_Speed[0],self.robot_state_pkg.actual_TCP_Speed[1],self.robot_state_pkg.actual_TCP_Speed[2],
                  self.robot_state_pkg.actual_TCP_Speed[3],self.robot_state_pkg.actual_TCP_Speed[4],self.robot_state_pkg.actual_TCP_Speed[5]]

    """   
    @brief  获取当前工具位姿
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） tcp_pose=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPPose(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        _error = self.robot.GetActualTCPPose(flag)
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error
        # return 0,[self.robot_state_pkg.tl_cur_pos[0],self.robot_state_pkg.tl_cur_pos[1],self.robot_state_pkg.tl_cur_pos[2],
        #           self.robot_state_pkg.tl_cur_pos[3],self.robot_state_pkg.tl_cur_pos[4],self.robot_state_pkg.tl_cur_pos[5]]

    """   
    @brief  获取当前工具坐标系编号
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） tool_id:工具坐标系编号
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPNum(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetActualTCPNum(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0,self.robot_state_pkg.tool

    """   
    @brief  获取当前工件坐标系编号 
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） wobj_id:工件坐标系编号
    """

    @log_call
    @xmlrpc_timeout
    def GetActualWObjNum(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetActualWObjNum(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0, self.robot_state_pkg.user

    """   
    @brief  获取当前末端法兰位姿
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） flange_pose=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetActualToolFlangePose(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetActualToolFlangePose(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.flange_cur_pos[0],self.robot_state_pkg.flange_cur_pos[1],self.robot_state_pkg.flange_cur_pos[2],
                  self.robot_state_pkg.flange_cur_pos[3],self.robot_state_pkg.flange_cur_pos[4],self.robot_state_pkg.flange_cur_pos[5]]
    """   
    @brief  逆运动学，笛卡尔位姿求解关节位置
    @param  [in] 必选参数 type:0-绝对位姿 (基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    @param  [in] 必选参数 desc_pose:[x,y,z,rx,ry,rz], 工具位姿，单位 [mm][°]
    @param  [in] 默认参数 config: 关节配置，[-1]-参考当前关节位置求解，[0~7]-依据关节配置求解 默认-1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKin(self, type, desc_pos, config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        config = int(config)
        flag = True
        while flag:
            try:
                _error = self.robot.GetInverseKin(type, desc_pos, config)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  逆运动学，工具位姿求解关节位置，参考指定关节位置求解
    @param  [in] 必选参数 type:0-绝对位姿 (基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    @param  [in] 必选参数 desc_pose:[x,y,z,rx,ry,rz], 工具位姿，单位 [mm][°]
    @param  [in] 必选参数 joint_pos_ref：[j1,j2,j3,j4,j5,j6]，关节参考位置，单位 [°]
    @return 错误码 成功- 0,joint_pos=[j1,j2,j3,j4,j5,j6] 失败-错误码
    @return 返回值（调用成功返回） joint_pos=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKinRef(self, type, desc_pos, joint_pos_ref):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        joint_pos_ref = list(map(float, joint_pos_ref))
        flag = True
        while flag:
            try:
                _error = self.robot.GetInverseKinRef(type, desc_pos, joint_pos_ref)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  逆运动学，工具位姿求解关节位置是否有解
    @param  [in] 必选参数 type:0-绝对位姿 (基坐标系)，1-相对位姿（基坐标系），2-相对位姿（工具坐标系）
    @param  [in] 必选参数 desc_pose:[x,y,z,rx,ry,rz], 工具位姿，单位 [mm][°]
    @param  [in] 必选参数 joint_pos_ref：[j1,j2,j3,j4,j5,j6]，关节参考位置，单位 [°]
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） result:“True”-有解，“False”-无解
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKinHasSolution(self, type, desc_pos, joint_pos_ref):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        joint_pos_ref = list(map(float, joint_pos_ref))
        flag = True
        while flag:
            try:
                _error = self.robot.GetInverseKinHasSolution(type, desc_pos, joint_pos_ref)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  正运动学，关节位置求解工具位姿
    @param  [in] 必选参数 joint_pos:[j1,j2,j3,j4,j5,j6]: 关节位置，单位 [°]
    @return 错误码 成功- 0,  失败-错误码
    @return 返回值（调用成功返回） desc_pos=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetForwardKin(self, joint_pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        joint_pos = list(map(float, joint_pos))
        flag = True
        while flag:
            try:
                _error = self.robot.GetForwardKin(joint_pos)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  获取当前关节转矩
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） torques=[j1,j2,j3,j4,j5,j6]
    """

    @log_call
    @xmlrpc_timeout
    def GetJointTorques(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = int(flag)
        # _error = self.robot.GetJointTorques(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.jt_cur_tor[0],self.robot_state_pkg.jt_cur_tor[1],self.robot_state_pkg.jt_cur_tor[2],
                  self.robot_state_pkg.jt_cur_tor[3],self.robot_state_pkg.jt_cur_tor[4],self.robot_state_pkg.jt_cur_tor[5]]

    """   
    @brief  获取当前负载的质量
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weight  单位 [kg]
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayload(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetTargetPayload(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  获取当前负载的质心
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）cog=[x,y,z]: 质心坐标，单位 [mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayloadCog(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetTargetPayloadCog(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]]
        else:
            return error,None

    """   
    @brief  获取当前工具坐标系
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）tcp_offset=[x,y,z,rx,ry,rz]: 相对位姿，单位 [mm][°]
    """

    @log_call
    @xmlrpc_timeout
    def GetTCPOffset(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetTCPOffset(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  获取当前工件坐标系
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）wobj_offset=[x,y,z,rx,ry,rz]: 相对位姿，单位 [mm][°]
    """

    @log_call
    @xmlrpc_timeout
    def GetWObjOffset(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetWObjOffset(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  获取关节软限位角度
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[j1min,j1max,j2min,j2max,j3min,j3max,j4min,j4max,j5min,j5max,j6min,j6max]: 轴 1~ 轴 6 关节负限位与正限位，单位 [mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetJointSoftLimitDeg(self, flag=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                _error = self.robot.GetJointSoftLimitDeg(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8],
                           _error[9], _error[10], _error[11], _error[12]]
        else:
            return error,None

    """   
    @brief  获取系统时间
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）t_ms: 单位 [ms]
    """

    @log_call
    @xmlrpc_timeout
    def GetSystemClock(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSystemClock()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  获取机器人当前关节配置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）config: 范围 [0~7]
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotCurJointsConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotCurJointsConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  获取默认速度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）vel: 单位 [mm/s]
    """

    @log_call
    @xmlrpc_timeout
    def GetDefaultTransVel(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetDefaultTransVel()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  查询机器人运动是否完成
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state:0-未完成，1-完成
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotMotionDone(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        return 0, self.robot_state_pkg.motion_done

    """   
    @brief  查询机器人错误码
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[maincode subcode] maincode 主错误码 subcode 子错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotErrorCode(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.GetRobotErrorCode()
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error
        return 0, [self.robot_state_pkg.main_code,self.robot_state_pkg.sub_code]

    """   
    @brief  查询机器人示教管理点位数据
    @param  [in] 必选参数 name  点位名
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data 点位数据[x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6,tool, wobj,speed,acc,e1,e2,e3,e4]
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotTeachingPoint(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotTeachingPoint(name)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            data =_error[1].split(',')
            if len(data)!= 20:
                self.log_error("get get Teaching Point size fail")
                return -1
            return error, [data[0],data[1], data[2], data[3], data[4], data[5], data[6],data[7],
                           data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15],
                           data[16], data[17], data[18], data[19] ]
        else:
            return error,None

    """   
    @brief  查询机器人运动队列缓存长度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）len  缓存长度
    """

    @log_call
    @xmlrpc_timeout
    def GetMotionQueueLength(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.GetMotionQueueLength()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0, self.robot_state_pkg.mc_queue_len

    """   
    @brief  获取机器人急停状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state 急停状态，0-非急停，1-急停
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotEmergencyStopState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.GetRobotEmergencyStopState()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0, self.robot_state_pkg.EmergencyStop

    """   
    @brief  获取安全停止信号
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[si0_state,si1_state] si0_state 安全停止信号SI0，0-无效，1-有效 si1_state 安全停止信号SI1，0-无效，1-有效
    """

    @log_call
    @xmlrpc_timeout
    def GetSafetyStopState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.GetSafetyStopState()
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error

        return 0, [self.robot_state_pkg.safety_stop0_state,self.robot_state_pkg.safety_stop1_state]

    """   
    @brief  获取SDK与机器人的通讯状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state 通讯状态，0-通讯正常，1-通讯异常
    """

    @log_call
    @xmlrpc_timeout
    def GetSDKComState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # while self.reconnect_flag:
        #     time.sleep(0.1)
        # flag = True
        # while flag:
        #     try:
        #         _error = self.robot.GetSDKComState()
        #         flag = False
        #     except socket.error as e:
        #         flag = True
        #
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2]]
        # else:
        #     return error,None
        g_sock_com_err = self.SDK_state
        if g_sock_com_err is True:
            return 0,0
        else:
            return 0,1


    """   
       @brief  获取SSH公钥
       @param  [in] NULL
       @return 错误码 成功-0，失败-错误码
       @return 返回值（调用成功返回） keygen 公钥
       """

    @log_call
    @xmlrpc_timeout
    def GetSSHKeygen(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSSHKeygen()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  下发SCP指令
    @param  [in] 必选参数 mode 0-上传（上位机->控制器），1-下载（控制器->上位机）
    @param  [in] 必选参数 sshname 上位机用户名
    @param  [in] 必选参数 sship 上位机ip地址
    @param  [in] 必选参数 usr_file_url 上位机文件路径
    @param  [in] 必选参数 robot_file_url 机器人控制器文件路径
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetSSHScpCmd(self, mode, sshname, sship, usr_file_url, robot_file_url):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        sshname = str(sshname)
        sship = str(sship)
        usr_file_url = str(usr_file_url)
        robot_file_url = str(robot_file_url)
        flag = True
        while flag:
            try:
                error = self.robot.SetSSHScpCmd(mode, sshname, sship, usr_file_url, robot_file_url)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算指定路径下文件的MD5值
    @param  [in] 必选参数 file_path 文件路径包含文件名，默认Traj文件夹路径为:"/fruser/traj/",如"/fruser/traj/trajHelix_aima_1.txt"
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回）md5 文件MD5值
    """

    @log_call
    @xmlrpc_timeout
    def ComputeFileMD5(self, file_path):
        while self.reconnect_flag:
            time.sleep(0.1)
        file_path = str(file_path)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeFileMD5(file_path)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  获取机器人版本信息
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） robotModel 机器人模型
    @return 返回值（调用成功返回） webVersion web版本
    @return 返回值（调用成功返回） controllerVersion 控制器版本
    """

    @log_call
    @xmlrpc_timeout
    def GetSoftwareVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSoftwareVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3]
        else:
            return error,None,None,None

    """   
    @brief  获取机器人硬件版本信息
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） ctrlBoxBoardVersion 控制箱版本
    @return 返回值（调用成功返回） driver1Version 
    @return 返回值（调用成功返回） driver2Version 
    @return 返回值（调用成功返回） driver3Version
    @return 返回值（调用成功返回） driver4Version
    @return 返回值（调用成功返回） driver5Version
    @return 返回值（调用成功返回） driver6Version
    @return 返回值（调用成功返回） endBoardVersion
    """

    @log_call
    @xmlrpc_timeout
    def GetSlaveHardVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveHardVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error,None,None,None,None,None,None,None,None

    @log_call
    @xmlrpc_timeout
    def GetHardwareversion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveHardVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error, None, None, None, None, None, None, None, None

    """   
    @brief  获取机器人固件版本信息
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） ctrlBoxBoardVersion 控制箱版本
    @return 返回值（调用成功返回） driver1Version 
    @return 返回值（调用成功返回） driver2Version 
    @return 返回值（调用成功返回） driver3Version
    @return 返回值（调用成功返回） driver4Version
    @return 返回值（调用成功返回） driver5Version
    @return 返回值（调用成功返回） driver6Version
    @return 返回值（调用成功返回） endBoardVersion
    """

    @log_call
    @xmlrpc_timeout
    def GetSlaveFirmVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveFirmVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error,None,None,None,None,None,None,None,None

    @log_call
    @xmlrpc_timeout
    def GetFirmwareVersion(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlaveFirmVersion()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error, None, None, None, None, None, None, None, None

    """   
    @brief  获取DH补偿参数
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） dhCompensation 机器人DH参数补偿值(mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
    """

    @log_call
    @xmlrpc_timeout
    def GetDHCompensation(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetDHCompensation()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    ***************************************************************************机器人轨迹复现********************************************************************************************
    """

    """   
    @brief  设置轨迹记录参数
    @param  [in] 必选参数 name：轨迹名
    @param  [in] 必选参数 period_ms：采样周期，固定值，2ms 或 4ms 或 8ms
    @param  [in] 默认参数 type：数据类型，1-关节位置 默认1
    @param  [in] 默认参数 di_choose：DI 选择,bit0~bit7 对应控制箱 DI0~DI7，bit8~bit9 对应末端DI0~DI1，0-不选择，1-选择 默认0
    @param  [in] 默认参数 do_choose：DO 选择,bit0~bit7 对应控制箱 DO0~DO7，bit8~bit9 对应末端 DO0~DO1，0-不选择，1-选择 默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDParam(self, name, period_ms, type=1, di_choose=0, do_choose=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        period_ms = int(period_ms)
        type = int(type)
        di_choose = int(di_choose)
        do_choose = int(do_choose)
        flag = True
        while flag:
            try:
                error = self.robot.SetTPDParam(type, name, period_ms, di_choose, do_choose)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  开始轨迹记录
    @param  [in] 必选参数 name：轨迹名
    @param  [in] 必选参数 period_ms：采样周期，固定值，2ms 或 4ms 或 8ms
    @param  [in] 默认参数 type：数据类型，1-关节位置 默认1
    @param  [in] 默认参数 di_choose：DI 选择,bit0~bit7 对应控制箱 DI0~DI7，bit8~bit9 对应末端DI0~DI1，0-不选择，1-选择 默认0
    @param  [in] 默认参数 do_choose：DO 选择,bit0~bit7 对应控制箱 DO0~DO7，bit8~bit9 对应末端 DO0~DO1，0-不选择，1-选择 默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDStart(self, name, period_ms, type=1, di_choose=0, do_choose=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        period_ms = int(period_ms)
        type = int(type)
        di_choose = int(di_choose)
        do_choose = int(do_choose)
        flag = True
        while flag:
            try:
                error = self.robot.SetTPDStart(type, name, period_ms, di_choose, do_choose)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  停止轨迹记录
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWebTPDStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SetWebTPDStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  删除轨迹记录
    @param  [in] 必选参数 name：轨迹名
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTPDDelete(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                error = self.robot.SetTPDDelete(name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  轨迹预加载
    @param  [in] 必选参数 name：轨迹名
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadTPD(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                error = self.robot.LoadTPD(name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取轨迹起始位姿
    @param  [in] name 轨迹文件名,不需要文件后缀
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）desc_pose [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetTPDStartPose(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTPDStartPose(name)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  轨迹复现
    @param  [in] 必选参数 name：轨迹名
    @param  [in] 必选参数 blend：是否平滑，0-不平滑，1-平滑
    @param  [in] 必选参数 ovl：速度缩放因子，范围 [0~100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveTPD(self, name, blend, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        name = str(name)
        blend = int(blend)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.MoveTPD(name, blend, ovl)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  轨迹预处理
    @param  [in] 必选参数 name：轨迹名 如/fruser/traj/trajHelix_aima_1.txt
    @param  [in] 必选参数 ovl 速度缩放百分比，范围[0~100]
    @param  [in] 默认参数 opt 1-控制点，默认为1
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadTrajectoryJ(self, name, ovl, opt=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        ovl = float(ovl)
        opt = int(opt)
        flag = True
        while flag:
            try:
                error = self.robot.LoadTrajectoryJ(name, ovl, opt)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  轨迹复现
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveTrajectoryJ(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.MoveTrajectoryJ()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取轨迹起始位姿
    @param  [in] 必选参数 name：轨迹名
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）desc_pose [x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def GetTrajectoryStartPose(self, name):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTrajectoryStartPose(name)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  获取轨迹点编号
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）pnum
    """

    @log_call
    @xmlrpc_timeout
    def GetTrajectoryPointNum(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTrajectoryPointNum()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  设置轨迹运行中的速度
    @param  [in] 必选参数 ovl 速度缩放百分比，范围[0~100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJSpeed(self, ovl, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        ovl = float(ovl)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJSpeed(ovl,mode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置轨迹运行中的力和扭矩
    @param  [in] 必选参数 ft [fx,fy,fz,tx,ty,tz]，单位N和Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceTorque(self, ft):
        while self.reconnect_flag:
            time.sleep(0.1)
        ft = list(map(float, ft))
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceTorque(ft)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置轨迹运行中的沿x方向的力
    @param  [in] 必选参数 fx 沿x方向的力，单位N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFx(self, fx):
        while self.reconnect_flag:
            time.sleep(0.1)
        fx = float(fx)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceFx(fx)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置轨迹运行中的沿y方向的力
    @param  [in] 必选参数 fy 沿y方向的力，单位N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFy(self, fy):
        while self.reconnect_flag:
            time.sleep(0.1)
        fy = float(fy)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceFy(fy)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置轨迹运行中的沿z方向的力
    @param  [in] 必选参数 fz 沿z方向的力，单位N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJForceFz(self, fz):
        while self.reconnect_flag:
            time.sleep(0.1)
        fz = float(fz)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJForceFy(fz)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置轨迹运行中的绕x轴的扭矩
    @param  [in] 必选参数 tx 绕x轴的扭矩，单位Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTx(self, tx):
        while self.reconnect_flag:
            time.sleep(0.1)
        tx = float(tx)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJTorqueTx(tx)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置轨迹运行中的绕y轴的扭矩
    @param  [in] 必选参数 ty 绕y轴的扭矩，单位Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTy(self, ty):
        while self.reconnect_flag:
            time.sleep(0.1)
        ty = float(ty)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJTorqueTx(ty)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置轨迹运行中的绕z轴的扭矩
    @param  [in] 必选参数 tz 绕z轴的扭矩，单位Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTrajectoryJTorqueTz(self, tz):
        while self.reconnect_flag:
            time.sleep(0.1)
        tz = float(tz)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJTorqueTx(tz)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************机器人WebAPP程序使用********************************************************************************************
    """

    """   
    @brief  设置开机自动加载默认的作业程序
    @param  [in] 必选参数 flag：0-开机不自动加载默认程序，1-开机自动加载默认程序
    @param  [in] 必选参数 program_name：作业程序名及路径，如“/fruser/movej.lua”，其中“/fruser/”为固定路径
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadDefaultProgConfig(self, flag, program_name):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        program_name = str(program_name)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.LoadDefaultProgConfig(flag, program_name)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  加载指定的作业程序
    @param  [in] 必选参数 program_name：作业程序名及路径，如“/fruser/movej.lua”，其中“/fruser/”为固定路径
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramLoad(self, program_name):
        while self.reconnect_flag:
            time.sleep(0.1)
        program_name = str(program_name)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramLoad(program_name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取当前机器人作业程序的执行行号
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）line_num
    """

    @log_call
    @xmlrpc_timeout
    def GetCurrentLine(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetCurrentLine()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  运行当前加载的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramRun(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.ProgramRun()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  暂停当前运行的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramPause(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramPause()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  恢复当前暂停的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramResume(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.ProgramResume()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  终止当前运行的作业程序
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ProgramStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取机器人作业程序执行状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）state:1-程序停止或无程序运行，2-程序运行中，3-程序暂停
    """

    @log_call
    @xmlrpc_timeout
    def GetProgramState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.GetProgramState()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
        return 0,self.robot_state_pkg.robot_state

    """   
    @brief  获取已加载的作业程序名
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）program_name
    """

    @log_call
    @xmlrpc_timeout
    def GetLoadedProgram(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetLoadedProgram()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    ***************************************************************************机器人外设********************************************************************************************
    """

    """   
    @brief  获取夹爪配置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[number,company,device,softversion] 
            number 夹爪编号
            company夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行 
            device  设备号，Robotiq(0-2F-85系列)，慧灵(0-NK系列,1-Z-EFG-100)，天机(0-TEG-110)，大寰(0-PGI-140)，知行(0-CTPM2F20)
            softvesion  软件版本号，暂不使用，默认为0
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetGripperConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1] + 1, _error[2] + 1, _error[3], _error[4]]
        else:
            return error,None

    """   
    @brief  激活夹爪
    @param  [in] 必选参数 index: 夹爪编号
    @param  [in] 必选参数 action:0-复位，1-激活
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ActGripper(self, index, action):
        while self.reconnect_flag:
            time.sleep(0.1)
        index = int(index)
        action = int(action)
        flag = True
        while flag:
            try:
                error = self.robot.ActGripper(index, action)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  控制夹爪
    @param  [in] 必选参数 index: 夹爪编号
    @param  [in] 必选参数 pos: 位置百分比，范围 [0~100]
    @param  [in] 必选参数 vel: 速度百分比，范围 [0~100]
    @param  [in] 必选参数 force: 力矩百分比，范围 [0~100]
    @param  [in] 必选参数 maxtime: 最大等待时间，范围 [0~30000]，单位 [ms]
    @param  [in] 必选参数 block:0-阻塞，1-非阻塞
    @param  [in] 必选参数 type 夹爪类型，0-平行夹爪；1-旋转夹爪
    @param  [in] 必选参数 rotNum 旋转圈数
    @param  [in] 必选参数 rotVel 旋转速度百分比[0-100]
    @param  [in] 必选参数 rotTorque 旋转力矩百分比[0-100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveGripper(self, index, pos, vel, force, maxtime, block, type, rotNum, rotVel, rotTorque):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        index = int(index)
        pos = int(pos)
        vel = int(vel)
        force = int(force)
        maxtime = int(maxtime)
        block = int(block)
        type = int(type)
        rotNum = float(rotNum)
        rotVel = int(rotVel)
        rotTorque = int(rotTorque)
        flag = True
        while flag:
            try:
                error = self.robot.MoveGripper(index, pos, vel, force, maxtime, block, type, rotNum, rotVel, rotTorque)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取夹爪运动状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[fault,status]：夹爪运动状态，fault:0-无错误，1-有错误；status:0-运动未完成，1-运动完成    
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperMotionDone(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetGripperMotionDone()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2]]
        else:
            return error,None

    """   
    @brief  配置夹爪
    @param  [in] 必选参数 company：夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行
    @param  [in] 必选参数 device：设备号，Robotiq(0-2F-85 系列)，慧灵 (0-NK 系列,1-Z-EFG-100)，天机 (0-TEG-110)，大寰 (0-PGI-140)，知行 (0-CTPM2F20)
    @param  [in] 默认参数 softversion：软件版本号，暂不使用，默认为 0
    @param  [in] 默认参数 bus：设备挂载末端总线位置，暂不使用，默认为 0；
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetGripperConfig(self, company, device, softversion=0, bus=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        company = int(company)
        device = int(device)
        softversion = int(softversion)
        bus = int(bus)
        flag = True
        while flag:
            try:
                error = self.robot.SetGripperConfig(company, device, softversion, bus)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算预抓取点-视觉
    @param  [in] 必选参数 desc_pos  抓取点笛卡尔位姿
    @param  [in] 必选参数 zlength   z轴偏移量
    @param  [in] 必选参数 zangle    绕z轴旋转偏移量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） pre_pos  预抓取点笛卡尔位姿
    """

    @log_call
    @xmlrpc_timeout
    def ComputePrePick(self, desc_pos, zlength, zangle):
        while self.reconnect_flag:
            time.sleep(0.1)
        desc_pos = list(map(float, desc_pos))
        zlength = float(zlength)
        zangle = float(zangle)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputePrePick(desc_pos, zlength, zangle)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  计算撤退点-视觉
    @param  [in] 必选参数 desc_pos  抓取点笛卡尔位姿
    @param  [in] 必选参数 zlength   z轴偏移量
    @param  [in] 必选参数 zangle    绕z轴旋转偏移量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） post_pos  撤退点笛卡尔位姿
    """

    @log_call
    @xmlrpc_timeout
    def ComputePostPick(self, desc_pos, zlength, zangle):
        while self.reconnect_flag:
            time.sleep(0.1)
        desc_pos = list(map(float, desc_pos))
        zlength = float(zlength)
        zangle = float(zangle)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputePostPick(desc_pos, zlength, zangle)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    ***************************************************************************机器人力控********************************************************************************************
    """

    """   
    @brief  获取力传感器配置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[number,company,device,softversion,bus]
            number 传感器编号
            company  力传感器厂商，17-坤维科技，19-航天十一院，20-ATI 传感器，21-中科米点，22-伟航敏芯
            device  设备号，坤维 (0-KWR75B)，航天十一院 (0-MCS6A-200-4)，ATI(0-AXIA80-M8)，中科米点 (0-MST2010)，伟航敏芯 (0-WHC6L-YB10A)
            softvesion  软件版本号，暂不使用，默认为0    
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_GetConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1] + 1, _error[2] + 1, _error[3], _error[4]]
        else:
            return error,None

    """   
    @brief  力传感器配置
    @param  [in] 必选参数 company：传感器厂商，17-坤维科技，19-航天十一院，20-ATI 传感器，21-中科米点，22-伟航敏芯，23-NBIT，24-鑫精诚(XJC)，26-NSR；
    @param  [in] 必选参数 device：设备号，坤维 (0-KWR75B)，航天十一院 (0-MCS6A-200-4)，ATI(0-AXIA80-M8)，中科米点 (0-MST2010)，伟航敏芯 (0-WHC6L-YB10A)，NBIT(0-XLH93003ACS)，鑫精诚XJC(0-XJC-6F-D82)，NSR(0-NSR-FTSensorA)；
    @param  [in] 默认参数 softversion：软件版本号，暂不使用，默认为 0
    @param  [in] 默认参数 bus：设备挂载末端总线位置，暂不使用，默认为 0；
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetConfig(self, company, device, softversion=0, bus=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        company = int(company)
        device = int(device)
        softversion = int(softversion)
        bus = int(bus)
        flag = True
        while flag:
            try:
                error = self.robot.FT_SetConfig(company, device, softversion, bus)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  力传感器激活
    @param  [in] 必选参数 state：0-复位，1-激活
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_Activate(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.FT_Activate(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  力传感器校零
    @param  [in] 必选参数 state：0-去除零点，1-零点矫正
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetZero(self, state):
        while self.reconnect_flag:
            time.sleep(0.1)
        state = int(state)
        flag = True
        while flag:
            try:
                error = self.robot.FT_SetZero(state)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置力传感器参考坐标系
    @param  [in] 必选参数 ref：0-工具坐标系，1-基坐标系
    @param  [in] 默认参数 coord：[x,y,z,rx,ry,rz] 自定义坐标系值,默认[0,0,0,0,0,0]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SetRCS(self, ref,coord=[0,0,0,0,0,0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        ref = int(ref)
        coord = list(map(float, coord))
        flag = True
        while flag:
            try:
                error = self.robot.FT_SetRCS(ref,coord)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  负载重量辨识计算
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weight-负载重量，单位 kg
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdIdenCompute(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_PdIdenCompute()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  负载重量辨识记录
    @param  [in] 必选参数 tool_id：传感器坐标系编号，范围 [1~14]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdIdenRecord(self, tool_id):
        while self.reconnect_flag:
            time.sleep(0.1)
        tool_id = int(tool_id)
        flag = True
        while flag:
            try:
                error = self.robot.FT_PdIdenRecord(tool_id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  负载质心辨识计算
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）cog=[cogx,cogy,cogz] ，负载质心，单位 mm
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdCogIdenCompute(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_PdCogIdenCompute()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]]
        else:
            return error,None

    """   
    @brief  负载质心辨识记录
    @param  [in] 必选参数 tool_id：传感器坐标系编号，范围 [0~14]
    @param  [in] 必选参数 index 点编号，范围[1~3]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_PdCogIdenRecord(self, tool_id, index):
        while self.reconnect_flag:
            time.sleep(0.1)
        tool_id = int(tool_id)
        index = int(index)
        flag = True
        while flag:
            try:
                error = self.robot.FT_PdCogIdenRecord(tool_id, index)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取参考坐标系下力/扭矩数据
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data=[fx,fy,fz,tx,ty,tz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetForceTorqueRCS(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.FT_GetForceTorqueRCS(0)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.ft_sensor_data[0],self.robot_state_pkg.ft_sensor_data[1],self.robot_state_pkg.ft_sensor_data[2],
                  self.robot_state_pkg.ft_sensor_data[3],self.robot_state_pkg.ft_sensor_data[4],self.robot_state_pkg.ft_sensor_data[5]]

    """   
    @brief  获取力传感器原始力/扭矩数据
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data=[fx,fy,fz,tx,ty,tz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_GetForceTorqueOrigin(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # _error = self.robot.FT_GetForceTorqueOrigin(0)
        # error = _error[0]
        # return error
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.ft_sensor_raw_data[0],self.robot_state_pkg.ft_sensor_raw_data[1],self.robot_state_pkg.ft_sensor_raw_data[2],
                  self.robot_state_pkg.ft_sensor_raw_data[3],self.robot_state_pkg.ft_sensor_raw_data[4],self.robot_state_pkg.ft_sensor_raw_data[5]]

    """   
    @brief  碰撞守护
    @param  [in] 必选参数 flag：0-关闭碰撞守护，1-开启碰撞守护；
    @param  [in] 必选参数 sensor_num：力传感器编号
    @param  [in] 必选参数 select：六个自由度是否检测碰撞 [fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    @param  [in] 必选参数 force_torque：碰撞检测力/力矩，单位 N 或 Nm
    @param  [in] 必选参数 max_threshold：最大阈值
    @param  [in] 必选参数 min_threshold：最小阈值
    力/力矩检测范围:(force_torque-min_threshold,force_torque+max_threshold)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_Guard(self, flag, sensor_num, select, force_torque, max_threshold, min_threshold):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        sensor_num = int(sensor_num)
        select = list(map(int, select))
        force_torque = list(map(float, force_torque))
        max_threshold = list(map(float, max_threshold))
        min_threshold = list(map(float, min_threshold))
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.FT_Guard(flag, sensor_num, select, force_torque, max_threshold, min_threshold)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  恒力控制
    @param  [in] 必选参数 flag：0-关闭碰撞守护，1-开启碰撞守护；
    @param  [in] 必选参数 sensor_id：力传感器编号
    @param  [in] 必选参数 select：[fx,fy,fz,mx,my,mz]六个自由度是否检测碰撞 ，0-不生效，1-生效
    @param  [in] 必选参数 ft：[fx,fy,fz,mx,my,mz]碰撞检测力/力矩，单位 N 或 Nm
    @param  [in] 必选参数 ft_pid：[f_p,f_i,f_d,m_p,m_i,m_d], 力PID参数，力矩PID参数
    @param  [in] 必选参数 adj_sign：自适应启停状态，0-关闭，1-开启
    @param  [in] 必选参数 ILC_sign: ILC 控制启停状态，0-停止，1-训练，2-实操
    @param  [in] 必选参数 max_dis：最大调整距离，单位mm
    @param  [in] 必选参数 max_ang：最大调整角度，单位deg
    @param  [in] 必选参数 M 质量参数
    @param  [in] 必选参数 B 阻尼参数
    @param  [in] 默认参数 threshold rx、ry启动阈值[0-10],默认0.2
    @param  [in] 默认参数 adjustCoeff rx、ry力矩调节系数[0-1],默认1
    @param  [in] 默认参数 polishRadio：打磨盘半径，单位mm
    @param  [in] 默认参数 filter_Sign 滤波开启标志 0-关；1-开，默认 0-关闭
    @param  [in] 默认参数 posAdapt_sign 姿态顺应开启标志 0-关；1-开，默认 0-关闭
    @param  [in] 默认参数 isNoBlock 阻塞标志，0-阻塞；1-非阻塞 默认0-阻塞
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_Control(self, flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M=None, B=None, threshold=[0.2,0.2], adjustCoeff=[1.0,1.0], polishRadio=0, filter_Sign=0, posAdapt_sign=0, isNoBlock=0):
        if M is None:
            M = [0, 0]
        if B is None:
            B = [0, 0]
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        sensor_id = int(sensor_id)
        select = list(map(int, select))
        ft = list(map(float, ft))
        ft_pid = list(map(float, ft_pid))
        adj_sign = int(adj_sign)
        ILC_sign = int(ILC_sign)
        max_dis = float(max_dis)
        max_ang = float(max_ang)


        M = list(map(float, M))
        B = list(map(float, B))
        threshold = list(map(float, threshold))
        adjustCoeff = list(map(float, adjustCoeff))
        polishRadio = float(polishRadio)
        filter_Sign = int(filter_Sign)
        posAdapt_sign = int(posAdapt_sign)
        isNoBlock = int(isNoBlock)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis,
                                              max_ang,polishRadio, filter_Sign, posAdapt_sign,[M[0],M[1],B[0],B[0],threshold[0],threshold[1],adjustCoeff[0],adjustCoeff[1]],isNoBlock)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  螺旋线探索
    @param  [in] 必选参数 rcs 参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 ft：力或力矩阈值 (0~100)，单位 N 或 Nm
    @param  [in] 默认参数 dr：每圈半径进给量，单位 mm 默认0.7
    @param  [in] 默认参数 max_t_ms：最大探索时间，单位 ms 默认 60000
    @param  [in] 默认参数 max_vel：线速度最大值，单位 mm/s 默认 5
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_SpiralSearch(self, rcs, ft, dr=0.7, max_t_ms=60000, max_vel=5):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        ft = float(ft)
        dr = float(dr)
        max_t_ms = float(max_t_ms)
        max_vel = float(max_vel)
        flag = True
        while flag:
            try:
                error = self.robot.FT_SpiralSearch(rcs, ft, dr, max_t_ms, max_vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  旋转插入
    @param  [in] 必选参数 rcs 参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 angVelRot 旋转角速度，单位deg/s
    @param  [in] 必选参数 ft  力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
    @param  [in] 必选参数 max_angle 最大旋转角度，单位deg
    @param  [in] 必选参数 orn 力/扭矩方向，1-沿z轴方向，2-绕z轴方向
    @param  [in] 必选参数 max_angAcc 最大旋转加速度，单位deg/s^2，暂不使用，默认为0
    @param  [in] 默认参数 rotorn：旋转方向，1-顺时针，2-逆时针 默认1
    @param  [in] 默认参数 strategy 未检测到力/力矩的处理策略，0-报错；1-警告，继续运动
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_RotInsertion(self, rcs, angVelRot, ft, max_angle, orn, max_angAcc=0, rotorn=1, strategy=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        angVelRot = float(angVelRot)
        ft = float(ft)
        max_angle = float(max_angle)
        orn = int(orn)
        max_angAcc = float(max_angAcc)
        rotorn = int(rotorn)
        strategy = int(strategy)
        flag = True
        while flag:
            try:
                error = self.robot.FT_RotInsertion(rcs, angVelRot, ft, max_angle, orn, max_angAcc, rotorn, strategy)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  直线插入
    @param  [in] 必选参数 rcs 参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 ft：力或力矩阈值 (0~100)，单位 N 或 Nm
    @param  [in] 必选参数 disMax：最大插入距离，单位 mm
    @param  [in] 必选参数 linorn：插入方向:0-负方向，1-正方向
    @param  [in] 默认参数 lin_v：直线速度，单位 mm/s 默认1
    @param  [in] 默认参数 lin_a：直线加速度，单位 mm/s^2，暂不使用 默认1
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_LinInsertion(self, rcs, ft, disMax, linorn, lin_v=1.0, lin_a=1.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        ft = float(ft)
        disMax = float(disMax)
        linorn = int(linorn)
        lin_v = float(lin_v)
        lin_a = float(lin_a)
        flag = True
        while flag:
            try:
                error = self.robot.FT_LinInsertion(rcs, ft, lin_v, lin_a, disMax, linorn)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算中间平面位置开始
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_CalCenterStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.FT_CalCenterStart()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算中间平面位置结束
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）pos=[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def FT_CalCenterEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.FT_CalCenterEnd()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  表面定位
    @param  [in] 必选参数 rcs：参考坐标系，0-工具坐标系，1-基坐标系
    @param  [in] 必选参数 dir：移动方向，1-正方向，2-负方向
    @param  [in] 必选参数 axis：移动轴，1-x，2-y，3-z
    @param  [in] 必选参数 disMax：最大探索距离，单位 mm
    @param  [in] 必选参数 ft：动作终止力阈值，单位 N
    @param  [in] 默认参数 lin_v：探索直线速度，单位 mm/s 默认3
    @param  [in] 默认参数 lin_a：探索直线加速度，单位 mm/s^2 默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_FindSurface(self, rcs, dir, axis, disMax, ft, lin_v=3.0, lin_a=0.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        dir = int(dir)
        axis = int(axis)
        ft = float(ft)
        lin_v = float(lin_v)
        lin_a = float(lin_a)
        flag = True
        while flag:
            try:
                error = self.robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, disMax, ft)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  柔顺控制关闭
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_ComplianceStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.FT_ComplianceStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  柔顺控制开启
    @param  [in] 必选参数 p: 位置调节系数或柔顺系数
    @param  [in] 必选参数 force：柔顺开启力阈值，单位 N
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_ComplianceStart(self, p, force):
        while self.reconnect_flag:
            time.sleep(0.1)
        p = float(p)
        force = float(force)
        flag = True
        while flag:
            try:
                error = self.robot.FT_ComplianceStart(p, force)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  负载辨识滤波初始化
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyDynFilterInit(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LoadIdentifyDynFilterInit()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  负载辨识变量初始化
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyDynVarInit(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LoadIdentifyDynVarInit()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  负载辨识主程序
    @param  [in] 必选参数 joint_torque 关节扭矩 j1-j6
    @param  [in] 必选参数 joint_pos 关节位置 j1-j6
    @param  [in] 必选参数 t 采样周期
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyMain(self, joint_torque, joint_pos, t):
        while self.reconnect_flag:
            time.sleep(0.1)
        joint_torque = list(map(float, joint_torque))
        joint_pos = list(map(float, joint_pos))
        t = float(t)
        flag = True
        while flag:
            try:
                error = self.robot.LoadIdentifyMain(joint_torque, joint_pos, t)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取负载辨识结果
    @param  [in] 必选参数 gain 重力项系数double[6]，离心项系数double[6] 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weight 负载重量
    @return 返回值（调用成功返回）cog 负载质心 [x,y,z]
    """

    @log_call
    @xmlrpc_timeout
    def LoadIdentifyGetResult(self, gain):
        while self.reconnect_flag:
            time.sleep(0.1)
        gain = list(map(float, gain))
        flag = True
        while flag:
            try:
                _error = self.robot.LoadIdentifyGetResult(gain)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], [_error[2], _error[3], _error[4]]
        else:
            return error,None,None

    """   
    ***************************************************************************传送带功能********************************************************************************************
    """

    """   
    @brief  传动带启动、停止
    @param  [in] 必选参数 status 状态，1-启动，0-停止 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorStartEnd(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorStartEnd(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  记录IO检测点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointIORecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorPointIORecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  记录A点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointARecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorPointARecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  记录参考点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorRefPointRecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorRefPointRecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  记录B点
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorPointBRecord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorPointBRecord()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  传送带工件IO检测
    @param  [in] 必选参数 max_t 最大检测时间，单位ms
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorIODetect(self, max_t):
        while self.reconnect_flag:
            time.sleep(0.1)
        max_t = int(max_t)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorIODetect(max_t)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取物体当前位置
    @param  [in] 必选参数  mode 1-跟踪抓取 2-跟踪运动 3-TPD跟踪
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorGetTrackData(self, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorGetTrackData(mode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  传动带跟踪开始
    @param  [in] 必选参数  status 状态，1-启动，0-停止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackStart(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorTrackStart(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  传动带跟踪停止
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorTrackEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  传动带参数配置
    @param  [in] 必选参数  param = [encChannel,resolution,lead,wpAxis,vision,speedRadio]  encChannel编码器通道 1-2,resolution 编码器分辨率 编码器旋转一圈脉冲个数,
    lead机械传动比 编码器旋转一圈传送带移动距离,wpAxis  工件坐标系编号 针对跟踪运动功能选择工件坐标系编号，跟踪抓取、TPD跟踪设为0,vision 是否配视觉  0 不配  1 配,
    speedRadio 速度比  针对传送带跟踪抓取速度范围为（1-100）  跟踪运动、TPD跟踪设置为1
    @param  [in] 必选参数 followType 跟踪运动类型，0-跟踪运动；1-追检运动
    @param  [in] 默认参数 startDis 追检抓取需要设置， 跟踪起始距离， -1：自动计算(工件到达机器人下方后自动追检)，单位mm， 默认值0
    @param  [in] 默认参数 endDis 追检抓取需要设置，跟踪终止距离， 单位mm， 默认值100
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorSetParam(self, param, followType, startDis=0, endDis=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        param = list(map(float, param))
        followType = int(followType)
        startDis = int(startDis)
        endDis = int(endDis)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorSetParam(param, followType, startDis, endDis)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  传动带抓取点补偿
    @param  [in] 必选参数 cmp 补偿位置 [x,y,z]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorCatchPointComp(self, cmp):
        while self.reconnect_flag:
            time.sleep(0.1)
        cmp = list(map(float, cmp))
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorCatchPointComp(cmp)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  直线运动
    @param  [in] 必选参数  name cvrCatchPoint和cvrRaisePoint
    @param  [in] 必选参数 tool 工具号
    @param  [in] 必选参数 wobj 工件号
    @param  [in] 默认参数 vel 速度 默认20
    @param  [in] 默认参数 acc 加速度 默认100
    @param  [in] 默认参数 ovl 速度缩放因子 默认100
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorTrackMoveL(self, name, tool, wobj, vel=20, acc=100, ovl=100, blendR=-1.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        name = str(name)
        tool = int(tool)
        wobj = int(wobj)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        flag = True
        while flag:
            try:
                error = self.robot.ConveyorTrackMoveL(name, tool, wobj, vel, acc, ovl, blendR, 0, 0)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    ***************************************************************************焊接功能********************************************************************************************
    """


    """   
    @brief  焊接开始 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 arcNum 焊机配置文件编号
    @param  [in] 必选参数 timeout 起弧超时时间
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ARCStart(self, ioType, arcNum, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)
        flag = True
        while flag:
            try:
                error = self.robot.ARCStart(ioType, arcNum, timeout)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  焊接结束 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 arcNum 焊机配置文件编号
    @param  [in] 必选参数 timeout 熄弧超时时间
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ARCEnd(self, ioType, arcNum, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)
        flag = True
        while flag:
            try:
                error = self.robot.ARCEnd(ioType, arcNum, timeout)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置焊接电流与输出模拟量对应关系 
    @param  [in] 必选参数 currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
    @param  [in] 必选参数 currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
    @param  [in] 必选参数 outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @param  [in] 必选参数 outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
    @param  [in] 必选参数 AOIndex 焊接电流模拟量输出端口
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrentRelation(self, currentMin, currentMax, outputVoltageMin, outputVoltageMax,AOIndex):
        while self.reconnect_flag:
            time.sleep(0.1)
        currentMin = float(currentMin)
        currentMax = float(currentMax)
        outputVoltageMin = float(outputVoltageMin)
        outputVoltageMax = float(outputVoltageMax)
        AOIndex =int(AOIndex)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrentRelation(currentMin, currentMax, outputVoltageMin, outputVoltageMax,AOIndex)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置焊接电压与输出模拟量对应关系 
    @param  [in] 必选参数 weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
    @param  [in] 必选参数 weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
    @param  [in] 必选参数 outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @param  [in] 必选参数 outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)    
    @param  [in] 必选参数 AOIndex 焊接电压模拟量输出端口
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltageRelation(self, weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax,AOIndex):
        while self.reconnect_flag:
            time.sleep(0.1)
        weldVoltageMin = float(weldVoltageMin)
        weldVoltageMax = float(weldVoltageMax)
        outputVoltageMin = float(outputVoltageMin)
        outputVoltageMax = float(outputVoltageMax)
        AOIndex =int(AOIndex)

        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltageRelation(weldVoltageMin, weldVoltageMax, outputVoltageMin, outputVoltageMax,AOIndex)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取焊接电流与输出模拟量对应关系 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
    @return 返回值（调用成功返回）currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
    @return 返回值（调用成功返回）outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @return 返回值（调用成功返回）outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
    @return 返回值（调用成功返回） AOIndex 焊接电压电流模拟量输出端口
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetCurrentRelation(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:
            flag = True
            while flag:
                try:
                    _error = self.robot.WeldingGetCurrentRelation()
                    flag = False
                except socket.error as e:
                    flag = True

            error = _error[0]
            if error == 0:
                return error, _error[1], _error[2], _error[3], _error[4], _error[5]
            return _error,None,None,None,None,None
        except Exception as e:
            return RobotError.ERR_RPC_ERROR,None,None,None,None,None

    """   
    @brief  获取焊接电压与输出模拟量对应关系 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
    @return 返回值（调用成功返回）weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
    @return 返回值（调用成功返回）outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
    @return 返回值（调用成功返回）outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
    @return 返回值（调用成功返回） AOIndex 焊接电压模拟量输出端口
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetVoltageRelation(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:
            flag = True
            while flag:
                try:
                    _error = self.robot.WeldingGetVoltageRelation()
                    flag = False
                except socket.error as e:
                    flag = True

            error = _error[0]
            if error == 0:
                return error, _error[1], _error[2], _error[3], _error[4], _error[5]
            return _error,None,None,None,None,None
        except Exception as e:
            return RobotError.ERR_RPC_ERROR,None,None,None,None,None

    """   
    @brief  设置焊接电流 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 float current 焊接电流值(A)
    @param  [in] 必选参数 AOIndex 焊接电流控制箱模拟量输出端口(0-1)
    @param  [in] 必选参数 blend 是否平滑 0-不平滑，1-平滑
    @return 错误码 成功- 0, 失败-
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrent(self, ioType, current, AOIndex,blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        current = float(current)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrent(ioType, current, AOIndex,blend)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置焊接电压 
    @param  [in] 必选参数 ioType io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 float voltage 焊接电压值(A)
    @param  [in] 必选参数 AOIndex 焊接电压控制箱模拟量输出端口(0-1)
    @param  [in] 必选参数 blend 是否平滑 0-不平滑，1-平滑
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltage(self, ioType, voltage, AOIndex,blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        voltage = float(voltage)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltage(ioType, voltage, AOIndex,blend)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置摆动参数 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @param  [in] 必选参数 int weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
    @param  [in] 必选参数 float weaveFrequency 摆动频率(Hz)
    @param  [in] 必选参数 int weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
    @param  [in] 必选参数 float weaveRange 摆动幅度(mm)
    @param  [in] 必选参数 float weaveLeftRange 垂直三角摆动左弦长度(mm)
    @param  [in] 必选参数 float weaveRightRange 垂直三角摆动右弦长度(mm)
    @param  [in] 必选参数 int additionalStayTime 垂直三角摆动垂三角点停留时间(ms)
    @param  [in] 必选参数 int weaveLeftStayTime 摆动左停留时间(ms)
    @param  [in] 必选参数 int weaveRightStayTime 摆动右停留时间(ms)
    @param  [in] 必选参数 int weaveCircleRadio 圆形摆动-回调比率(0-100%)
    @param  [in] 必选参数 int weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
    @param  [in] 必选参数 float weaveYawAngle 摆动方向方位角（绕摆动Z轴旋转），单位°,默认0
    @param  [in] 必选参数 float weaveRotAngle 摆动方向方位角（绕摆动X轴旋转），单位°,默认0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveSetPara(self, weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                     weaveLeftRange, weaveRightRange, additionalStayTime, weaveLeftStayTime,
                     weaveRightStayTime, weaveCircleRadio, weaveStationary,weaveYawAngle=0,weaveRotAngle=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        weaveType = int(weaveType)
        weaveFrequency = float(weaveFrequency)
        weaveIncStayTime = int(weaveIncStayTime)
        weaveRange = float(weaveRange)
        weaveLeftRange = float(weaveLeftRange)
        weaveRightRange = float(weaveRightRange)
        additionalStayTime = int(additionalStayTime)
        weaveLeftStayTime = int(weaveLeftStayTime)
        weaveRightStayTime = int(weaveRightStayTime)
        weaveCircleRadio = int(weaveCircleRadio)
        weaveStationary = int(weaveStationary)
        weaveYawAngle = float(weaveYawAngle)
        weaveRotAngle = float(weaveRotAngle)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                                                weaveLeftRange, weaveRightRange, additionalStayTime,
                                                weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio, weaveStationary,weaveYawAngle,weaveRotAngle)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  即时设置摆动参数 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @param  [in] 必选参数 int weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-
    @param  [in] 必选参数 float weaveFrequency 摆动频率(Hz)
    @param  [in] 必选参数 int weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
    @param  [in] 必选参数 float weaveRange 摆动幅度(mm)
    @param  [in] 必选参数 int weaveLeftStayTime 摆动左停留时间(ms)
    @param  [in] 必选参数 int weaveRightStayTime 摆动右停留时间(ms)
    @param  [in] 必选参数 int weaveCircleRadio 圆形摆动-回调比率(0-100%)
    @param  [in] 必选参数 int weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveOnlineSetPara(self, weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange, weaveLeftStayTime,
                           weaveRightStayTime, weaveCircleRadio, weaveStationary):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        weaveType = int(weaveType)
        weaveFrequency = float(weaveFrequency)
        weaveIncStayTime = int(weaveIncStayTime)
        weaveRange = float(weaveRange)
        weaveLeftStayTime = int(weaveLeftStayTime)
        weaveRightStayTime = int(weaveRightStayTime)
        weaveCircleRadio = int(weaveCircleRadio)
        weaveStationary = int(weaveStationary)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.WeaveOnlineSetPara(weaveNum, weaveType, weaveFrequency, weaveIncStayTime, weaveRange,
                                                          weaveLeftStayTime, weaveRightStayTime, weaveCircleRadio,
                                                          weaveStationary)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  摆动开始 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveStart(self, weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.WeaveStart(weaveNum)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  摆动结束 
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeaveEnd(self, weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.WeaveEnd(weaveNum)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  正向送丝 
    @param  [in] 必选参数 int ioType io类型  0-控制器IO；1-扩展IO
    @param  [in] 必选参数 int wireFeed 送丝控制  0-停止送丝；1-送丝
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetForwardWireFeed(self, ioType, wireFeed):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        wireFeed = int(wireFeed)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.SetForwardWireFeed(ioType, wireFeed)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  反向送丝 
    @param  [in] 必选参数 int ioType io类型  0-控制器IO；1-扩展IO
    @param  [in] 必选参数 int wireFeed 送丝控制  0-停止送丝；1-送丝
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetReverseWireFeed(self, ioType, wireFeed):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        wireFeed = int(wireFeed)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.SetReverseWireFeed(ioType, wireFeed)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR

    """   
    @brief  送气
    @param  [in] 必选参数 int ioType io类型  0-控制器IO；1-扩展IO
    @param  [in] 必选参数 int airControl 送气控制  0-停止送气；1-送气
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAspirated(self, ioType, airControl):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        airControl = int(airControl)
        try:
            flag = True
            while flag:
                try:
                    error = self.robot.SetAspirated(ioType, airControl)
                    flag = False
                except socket.error as e:
                    flag = True

            return error
        except Exception as e:
            return RobotError.ERR_RPC_ERROR


    """   
    @brief  段焊获取位置和姿态
    @param  [in]必选参数 startPos=[x,y,z,rx,ry,rz] 起始点坐标
    @param  [in]必选参数 endPos=[x,y,z,rx,ry,rz] 终止点坐标
    @param  [in]必选参数 startDistance 焊接点至起点的长度
    @return 错误码 成功- 0, 失败-错误码    
    @return 返回值（调用成功返回） weldPointDesc=[x,y,z,rx,ry,rz] 焊接点的笛卡尔坐标信息
    @return 返回值（调用成功返回） weldPointJoint=[j1,j2,j3,j4,j5,j6] 焊接点的关节坐标信息
    @return 返回值（调用成功返回） tool 工具号
    @return 返回值（调用成功返回） user 工件号
    """

    @log_call
    @xmlrpc_timeout
    def GetSegmentWeldPoint(self, startPos, endPos, startDistance):
        while self.reconnect_flag:
            time.sleep(0.1)
        startPos = list(map(float, startPos))
        endPos = list(map(float, endPos))
        startDistance = float(startDistance)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSegmentWeldPoint(startPos, endPos, startDistance)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            data = _error[1].split(',')
            if len(data) != 14:
                self.log_error("GetSegmentWeldPoint fail")
                return -1
            else:
                data = list(map(float,data))
                tool = int(data[12])
                work = int(data[13])
                return (error, [ data[0],data[1],data[3],data[4],data[4],data[5]],
                        [data[6],data[7],data[8],data[9],data[10],data[11]],tool, work)
        else:
            return error,None,None,None

    """   
    @brief  分段焊接启动
    @param  [in] 必选参数 startDesePos: 初始笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 endDesePos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 startJPos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 endJPos: 目标关节位置，单位 [°] 
    @param  [in] 必选参数 weldLength: 焊接长度，单位 [mm]
    @param  [in] 必选参数 noWeldLength: 非焊接长度，单位 [mm]    
    @param  [in] 必选参数 weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
    @param  [in] 必选参数 arcNum 焊机配置文件编号
    @param  [in] 必选参数 timeout 熄弧超时时间
    @param  [in] 必选参数 isWeave true-焊接 false-不焊接
    @param  [in] 必选参数 int weaveNum 摆焊参数配置编号
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 search:[0]-不焊丝寻位，[1]-焊丝寻位
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SegmentWeldStart(self, startDesePos, endDesePos, startJPos, endJPos, weldLength, noWeldLength, weldIOType,
                         arcNum, weldTimeout, isWeave, weaveNum, tool, user,
                         vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0, exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0,
                         offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)

        startDesePos = list(map(float, startDesePos))
        endDesePos = list(map(float, endDesePos))
        startJPos = list(map(float, startJPos))
        endJPos = list(map(float, endJPos))

        weldLength = float(weldLength)
        noWeldLength = float(noWeldLength)
        weldIOType = int(weldIOType)
        arcNum = int(arcNum)
        weldTimeout = int(weldTimeout)
        isWeave = bool(isWeave)
        weaveNum = int(weaveNum)
        tool = int(tool)
        user = int(user)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        rtn = 0
        # 获取起点到终点之间的距离和各方向角度余弦值
        # print("1",startDesePos,endDesePos)
        result = self.robot.GetSegWeldDisDir(startDesePos[0], startDesePos[1], startDesePos[2], endDesePos[0],
                                             endDesePos[1], endDesePos[2])
        # print("result",result)
        if result[0] != 0:
            return int(result[0])

        distance = result[1]
        endOffPos = list(offset_pos)

        rtn = self.robot.MoveJ(startJPos, startDesePos, tool, user, vel, acc, ovl, exaxis_pos, blendR, offset_flag,
                               offset_pos)
        # print("rtn1", rtn)
        if rtn != 0:
            return rtn

        weldNum = 0
        noWeldNum = 0
        i = 0
        while i < int(distance / (weldLength + noWeldLength)) * 2 + 2:
            if i % 2 == 0:
                weldNum += 1
                if weldNum * weldLength + noWeldNum * noWeldLength > distance:

                    rtn = self.robot.ARCStart(weldIOType, arcNum, weldTimeout)
                    # print("rtn2", rtn)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveStart(weaveNum)
                        if rtn != 0:
                            # print("rtn3", rtn)
                            return rtn

                    # getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos,endDesePos,weldNum* weldLength + noWeldNum * noWeldLength)
                    # # print("getsegmentrtn", getsegmentrtn)
                    # # print(startDesePos, endDesePos, weldNum * weldLength + noWeldNum * noWeldLength)
                    # # print("weldNum", weldNum, "weldLength", weldLength)
                    # # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    # if getsegmentrtn[0] != 0  :
                    #     return getsegmentrtn[0]
                    # data = getsegmentrtn[1].split(',')
                    # data = list(map(float, data))
                    # if len(data) != 14:
                    #     self.log_error("GetSegmentWeldPoint fail")
                    #     return -1
                    # tmpJoint = [data[0],data[1],data[2],data[3],data[4],data[5]]
                    # tmpWeldDesc = [data[6],data[7],data[8],data[9],data[10],data[11]]
                    # tmpTool = int(data[12])
                    # tmpUser = int(data[13])
                    rtn = self.robot.MoveL(endJPos,endDesePos, tool, user, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn3", rtn,endJPos,endDesePos)
                    if rtn != 0:
                        self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                        if isWeave:
                            rtn = self.robot.WeaveEnd(weaveNum)
                            # print("rtn4", rtn)
                            if rtn != 0:
                                return rtn
                        return rtn
                    rtn = self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                    # print("rtn5", rtn)
                    if rtn != 0:
                        break
                    if isWeave:
                        rtn = self.robot.WeaveEnd(weaveNum)
                        # print("rtn6", rtn)
                        if rtn != 0:
                            break

                else:
                    rtn = self.robot.ARCStart(weldIOType, arcNum, weldTimeout)
                    # print("rtn7", rtn)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveStart(weaveNum)
                        # print("rtn8", rtn)
                        if rtn != 0:
                            return rtn

                    getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos, endDesePos,
                                                                   weldNum * weldLength + noWeldNum * noWeldLength)
                    # print("rtn9", getsegmentrtn)
                    # print(startDesePos, endDesePos, weldNum * weldLength + noWeldNum * noWeldLength)
                    # print("weldNum", weldNum, "weldLength", weldLength)
                    # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    if getsegmentrtn[0] != 0:
                        return getsegmentrtn[0]
                    data = getsegmentrtn[1].split(',')
                    data = list(map(float, data))
                    if len(data) != 14:
                        self.log_error("GetSegmentWeldPoint fail")
                        return -1
                    tmpJoint = [data[0], data[1], data[2], data[3], data[4], data[5]]
                    tmpWeldDesc = [data[6], data[7], data[8], data[9], data[10], data[11]]
                    tmpTool = int(data[12])
                    tmpUser = int(data[13])
                    # print("tmpJoint",tmpJoint,tmpWeldDesc,tmpTool,tmpUser)
                    time.sleep(1)
                    nihao = self.robot.MoveL(tmpJoint, tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn10nihao", nihao)
                    if nihao != 0:
                        self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                        if isWeave:
                            rtn = self.robot.WeaveEnd(weaveNum)
                            # print("rtn11", rtn)
                            if rtn != 0:
                                return rtn
                        return rtn
                    rtn = self.robot.ARCEnd(weldIOType, arcNum, weldTimeout)
                    # print("rtn12", rtn)
                    if rtn != 0:
                        return rtn
                    if isWeave:
                        rtn = self.robot.WeaveEnd(weaveNum)
                        # print("rtn13", rtn)
                        if rtn != 0:
                            return rtn
            else:
                noWeldNum += 1
                if weldNum * weldLength + noWeldNum * noWeldLength > distance:
                    # getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos, endDesePos, weldNum* weldLength + noWeldNum * noWeldLength)
                    # # print("rtn14", getsegmentrtn)
                    # # print(startDesePos, endDesePos, weldNum * weldLength + noWeldNum * noWeldLength)
                    # # print("weldNum", weldNum, "weldLength", weldLength)
                    # # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    # if getsegmentrtn[0] != 0:
                    #     return getsegmentrtn[0]
                    # data = getsegmentrtn[1].split(',')
                    # data = list(map(float,data))
                    # if len(data) != 14:
                    #     self.log_error("GetSegmentWeldPoint fail")
                    #     return -1
                    # tmpJoint = [data[0], data[1], data[2], data[3], data[4], data[5]]
                    # tmpWeldDesc = [data[6], data[7], data[8], data[9], data[10], data[11]]
                    # tmpTool = int(data[12])
                    # tmpUser = int(data[13])
                    rtn = self.robot.MoveL(endJPos,endDesePos, tool, user, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn15", rtn,endJPos,endDesePos)
                    if rtn != 0:
                       return rtn
                    break
                else:
                    getsegmentrtn = self.robot.GetSegmentWeldPoint(startDesePos, endDesePos, weldNum* weldLength + noWeldNum * noWeldLength)
                    # print("rtn16", getsegmentrtn,startDesePos,endDesePos,weldNum* weldLength + noWeldNum * noWeldLength)

                    # print(startDesePos,endDesePos,weldNum* weldLength + noWeldNum * noWeldLength)
                    # print("weldNum",weldNum,"weldLength",weldLength)
                    # print("noWeldNum", noWeldNum, "noWeldLength", noWeldLength)
                    if getsegmentrtn[0] != 0:
                        return getsegmentrtn[0]
                    data = getsegmentrtn[1].split(',')
                    data = list(map(float, data))
                    if len(data) != 14:
                        self.log_error("GetSegmentWeldPoint fail")
                        return -1
                    tmpJoint = [data[0], data[1], data[2], data[3], data[4], data[5]]
                    tmpWeldDesc = [data[6], data[7], data[8], data[9], data[10], data[11]]
                    tmpTool = int(data[12])
                    tmpUser = int(data[13])
                    rtn = self.robot.MoveL(tmpJoint, tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR,0, exaxis_pos,
                                           search, 0, endOffPos)
                    # print("rtn17", rtn)
                    if rtn != 0:
                        return rtn
            i =i + 1
        return rtn

    """   
    @brief  分段焊接终止
    @param  [in] 必选参数 ioType：io类型 0-控制器IO； 1-扩展IO
    @param  [in] 必选参数 arcNum：焊机配置文件编号
    @param  [in] 必选参数 timeout：熄弧超时时间
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SegmentWeldEnd(self, ioType, arcNum, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        ioType = int(ioType)
        arcNum = int(arcNum)
        timeout = int(timeout)

        flag = True
        while flag:
            try:
                rtn = self.robot.SegmentWeldEnd(ioType, arcNum, timeout)
                flag = False
            except socket.error as e:
                flag = True

        return rtn

    """   
    @brief  初始化日志参数
    @param  [in]默认参数 output_model：输出模式，0-直接输出；1-缓冲输出；2-异步输出，默认1
    @param  [in]默认参数 file_path： 文件保存路径+名称，名称必须是xxx.log的形式，比如/home/fr/linux/fairino.log。
                    默认执行程序所在路径，默认名称fairino_ year+mouth+data.log(如:fairino_2024_03_13.log);
    @param  [in]默认参数 file_num：滚动存储的文件数量，1~20个，默认值为5。单个文件上限50M;
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoggerInit(self, output_model=1, file_path="", file_num=5):
        while self.reconnect_flag:
            time.sleep(0.1)

        return self.setup_logging(output_model, file_path, file_num)

    """   
    @brief  设置日志过滤等级
    @param  [in] 默认参数 lvl: 过滤等级值，值越小输出日志越少, 1-error, 2-warnning, 3-inform, 4-debug,默认值是1.
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoggerLevel(self, lvl=1):
        while self.reconnect_flag:
            time.sleep(0.1)

        lvl=int(lvl)
        log_level = self.set_log_level(lvl)
        return 0

    """   
    @brief  下载点位表数据库
    @param  [in] pointTableName 要下载的点位表名称    pointTable1.db
    @param  [in] saveFilePath 下载点位表的存储路径   C://test/
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointTableDownLoad(self, point_table_name, save_file_path):
        while self.reconnect_flag:
            time.sleep(0.1)

        if not os.path.exists(save_file_path):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND

        rtn = self.robot.PointTableDownload(point_table_name)
        if rtn == -1:
            return RobotError.ERR_UPLOAD_FILE_NOT_FOUND
        elif rtn != 0:
            return rtn
        port = 20011
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)
        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        total_buffer = bytearray(1024 * 1024 * 50)  # 50Mb
        total_size = 0
        recv_md5 = ""
        recv_size = 0
        find_head_flag = False
        while True:
            buffer = client.recv(1024)
            length = len(buffer)
            if length < 1:
                return RobotError.ERR_OTHER
            total_buffer[total_size:total_size + len(buffer)] = buffer

            total_size += len(buffer)
            if not find_head_flag and total_size > 4 and total_buffer[:4].decode('utf-8') == "/f/b":
                find_head_flag = True
            # 找到文件头后，提取文件大小和MD5校验码。文件大小的信息位于总数据的第5到第12个字节，MD5校验码的信息位于总数据的第13到第44个字节。
            if find_head_flag and total_size > 12 + 32:
                recv_size = int(total_buffer[4:12].decode('utf-8'))
                recv_md5 = total_buffer[12:44].decode('utf-8')
            # 接收到整个文件跳出循环
            if find_head_flag and total_size == recv_size:
                break
        if total_size == 0:
            return RobotError.ERR_OTHER
        file_buffer = total_buffer[12 + 32:total_size - 4]

        with open(os.path.join(save_file_path, point_table_name), 'wb') as file_writer:
            file_writer.write(file_buffer[:total_size - 16 - 32])

        check_md5 = calculate_file_md5(save_file_path + point_table_name)
        if check_md5 == recv_md5:
            client.send("SUCCESS".encode('utf-8'))
            return 0
        else:
            client.send("FAIL".encode('utf-8'))
            os.remove(os.path.join(save_file_path, point_table_name))
            return RobotError.ERR_OTHER

    """   
    @brief  上传点位表数据库
    @param  [in] pointTableFilePath 上传点位表的全路径名   C://test/pointTable1.db
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PointTableUpLoad(self, point_table_file_path):
        while self.reconnect_flag:
            time.sleep(0.1)

        MAX_UPLOAD_FILE_SIZE = 2 * 1024 * 1024  # 最大上传文件为2Mb
        # 判断上传文件是否存在
        if not os.path.exists(point_table_file_path):
            return RobotError.ERR_UPLOAD_FILE_NOT_FOUND

        file_info = os.path.getsize(point_table_file_path)
        total_size = file_info + 16 + 32
        if total_size > MAX_UPLOAD_FILE_SIZE:
            print("Files larger than 2 MB are not supported!")
            return -1

        point_table_name = os.path.basename(point_table_file_path)

        rtn = self.robot.PointTableUpload(point_table_name)
        if rtn != 0:
            return rtn

        port = 20010

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)

        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER

        client.settimeout(2)

        # client.receive_timeout = 2000
        # client.send_timeout = 2000

        send_md5 = calculate_file_md5(point_table_file_path)

        head_data = f"/f/b{total_size:08d}{send_md5}"
        num = client.send(head_data.encode('utf-8'))
        if num < 1:
            return RobotError.ERR_OTHER

        with open(point_table_file_path, 'rb') as fs:
            file_bytes = fs.read()

        num = client.send(file_bytes)
        if num < 1:
            return RobotError.ERR_OTHER
        end_data = "/b/f"
        num = client.send(end_data.encode('utf-8'))
        if num < 1:
            return RobotError.ERR_OTHER

        result_buf = client.recv(1024)
        if result_buf[:7].decode('utf-8') == "SUCCESS":
            return RobotError.ERR_SUCCESS
        else:
            return RobotError.ERR_OTHER

    """   
    @brief  点位表切换
    @param  [in] PointTableSwitch 要切换的点位表名称   "pointTable1.db",当点位表为空，即""时，表示将lua程序更新为未应用点位表的初始程序
    @return 错误码 成功-0   失败-错误码 
    @return 错误errorStr
    """

    @log_call
    @xmlrpc_timeout
    def PointTableSwitch(self, point_table_name):
        while self.reconnect_flag:
            time.sleep(0.1)

        rtn = self.robot.PointTableSwitch(point_table_name)  # 切换点位表
        if rtn != 0:
            if rtn == RobotError.ERR_UPLOAD_FILE_NOT_FOUND:
                error_str = "PointTable not Found!"
            else:
                error_str = "PointTable not Found!"
            return rtn, error_str
        return rtn

    """   
    @brief  点位表更新lua文件
    @param  [in] pointTableName 要切换的点位表名称   "pointTable1.db",当点位表为空，即""时，表示将lua程序更新为未应用点位表的初始程序
    @param  [in] luaFileName 要更新的lua文件名称   "testPointTable.lua"
    @return 错误码 成功-0   失败-错误码 
    @return 错误errorStr
    """

    @log_call
    @xmlrpc_timeout
    def PointTableUpdateLua(self, point_table_name, lua_file_name):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:

            rtn = self.robot.PointTableSwitch(point_table_name)  # 切换点位表
            if rtn != 0:
                if rtn == RobotError.ERR_UPLOAD_FILE_NOT_FOUND:
                    error_str = "PointTable not Found!"
                else:
                    error_str = "PointTable not Found!"
                return rtn, error_str

            time.sleep(0.3)  # 增加延时确保切换后后端确实收到切换后的点位表名称

            result = self.robot.PointTableUpdateLua(lua_file_name)
            error_str = result[1]
            if not error_str:
                error_str = "fail to update lua, please inspect pointtable"
            return result[0], error_str

        except Exception as e:
            return RobotError.ERR_RPC_ERROR, ""

    """   
    @brief  下载文件
    @param  [in] fileType 文件类型    0-lua文件
    @param  [in] fileName 文件名称    “test.lua”
    @param  [in] saveFilePath 保存文件路径    “C：//test/”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def __FileDownLoad(self, fileType, fileName, saveFilePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        if not os.path.exists(saveFilePath):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND
        rtn = self.robot.FileDownload(fileType, fileName)
        if rtn == -1:
            return RobotError.ERR_UPLOAD_FILE_NOT_FOUND
        elif rtn != 0:
            return rtn
        port = 20011
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2)
        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        total_buffer = bytearray(1024 * 1024 * 50)  # 50Mb
        total_size = 0
        recv_md5 = ""
        recv_size = 0
        find_head_flag = False
        while True:
            buffer = client.recv(1024)
            length = len(buffer)
            if length < 1:
                return RobotError.ERR_OTHER
            total_buffer[total_size:total_size + len(buffer)] = buffer
            total_size += len(buffer)
            if not find_head_flag and total_size > 4 and total_buffer[:4].decode('utf-8') == "/f/b":
                find_head_flag = True
            # 找到文件头后，提取文件大小和MD5校验码。文件大小的信息位于总数据的第5到第12个字节，MD5校验码的信息位于总数据的第13到第44个字节。
            # if find_head_flag and total_size > 12 + 32:
            if find_head_flag and total_size > 14 + 32:
                # recv_size = int(total_buffer[4:12].decode('utf-8'))
                recv_size = int(total_buffer[4:14].decode('utf-8'))
                # recv_md5 = total_buffer[12:44].decode('utf-8')
                recv_md5 = total_buffer[14:46].decode('utf-8')
            # 接收到整个文件跳出循环
            if find_head_flag and total_size == recv_size:
                break
        if total_size == 0:
            return RobotError.ERR_OTHER
        # file_buffer = total_buffer[12 + 32:total_size - 4]
        file_buffer = total_buffer[14 + 32:total_size - 4]
        with open(os.path.join(saveFilePath, fileName), 'wb') as file_writer:
            # file_writer.write(file_buffer[:total_size - 16 - 32])
            file_writer.write(file_buffer[:total_size - 16 - 32 - 2])
        check_md5 = calculate_file_md5(saveFilePath + fileName)
        if check_md5 == recv_md5:
            client.send("SUCCESS".encode('utf-8'))
            return 0
        else:
            client.send("FAIL".encode('utf-8'))
            os.remove(os.path.join(saveFilePath, fileName))
            # return RobotError.ERR_OTHER
            return RobotError.ERR_DOWN_LOAD_FILE_FAILED

    """   
    @brief  上传文件
    @param  [in] fileType 文件类型    0-lua文件
    @param  [in] filePath上传文件的全路径名    C://test/test.lua     
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def __FileUpLoad(self, fileType, filePath):

        while self.reconnect_flag:
            time.sleep(0.1)

        if not os.path.exists(filePath):
            return RobotError.ERR_UPLOAD_FILE_NOT_FOUND

        MAX_UPLOAD_FILE_SIZE = 500 * 1024 * 1024;  # 最大上传文件为500Mb
        file_info = os.path.getsize(filePath)
        total_size = file_info + 46 + 4
        if total_size > MAX_UPLOAD_FILE_SIZE:
            print("Files larger than 500 MB are not supported!")
            return -1
        file_name = os.path.basename(filePath)
        rtn = self.robot.FileUpload(fileType, file_name)
        if rtn != 0:
            return rtn

        port = 20010

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(20)

        try:
            client.connect((self.ip_address, port))
        except Exception as e:
            client.close()
            return RobotError.ERR_OTHER
        client.settimeout(20)

        send_md5 = calculate_file_md5(filePath)
        head_data = f"/f/b{total_size:10d}{send_md5}"
        num = client.send(head_data.encode('utf-8'))

        if num < 1:
            return RobotError.ERR_OTHER

        with open(filePath, "rb") as f:
            while True:
                data = f.read(2 * 1024 * 1024)
                if not data:  # 如果读取到文件末尾
                    end_data = "/b/f"
                    num = client.send(end_data.encode('utf-8'))  # 发送文件传输完成的标志
                    if num < 1:
                        return RobotError.ERR_OTHER
                    break  # 跳出循环
                num = client.send(data)  # 将读取的数据通过socket连接发送给客户端
                if num < 1:
                    return RobotError.ERR_OTHER
        time.sleep(0.5)
        result_buf = client.recv(1024)
        if result_buf[:7].decode('utf-8') == "SUCCESS":
            return RobotError.ERR_SUCCESS
        else:
            return RobotError.ERR_OTHER

    """   
    @brief  删除文件
    @param  [in] fileType 文件类型    0-lua文件
    @param  [in] fileName 文件名称    “test.lua”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def __FileDelete(self, fileType, fileName):
        while self.reconnect_flag:
            time.sleep(0.1)

        rtn = self.robot.FileDelete(fileType, fileName)
        return rtn

    """   
    @brief  下载Lua文件
    @param  [in] fileName 要下载的lua文件名“test.lua”
    @param  [in] savePath 保存文件本地路径“D://Down/”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LuaDownLoad(self, fileName, savePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.__FileDownLoad(0, fileName, savePath)
        return error

    """   
    @brief  上传Lua文件
    @param  [in] filePath上传文件的全路径名   C://test/test.lua  
    @return 错误码 成功-0  失败-错误码
    """

    def LuaUpload(self, filePath):
        error = self.__FileUpLoad(0, filePath)
        if error == 0:
            file_name = os.path.basename(filePath)
            _error = self.robot.LuaUpLoadUpdate(file_name)
            tmp_error = _error[0]
            if tmp_error == 0:
                return tmp_error
            else:
                return tmp_error, _error[1]
        else:
            return error

    """   
    @brief  删除Lua文件
    @param  [in] fileName 要删除的lua文件名“test.lua”
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LuaDelete(self, fileName):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.__FileDelete(0, fileName)
        return error

    """   
    @brief  获取当前所有lua文件名称
    @return 错误码 成功-0  失败-错误码
    @return 返回值（调用成功返回） lua_num lua文件数量
    @return 返回值（调用成功返回） luaNames lua文件名列表
    """

    @log_call
    @xmlrpc_timeout
    def GetLuaList(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        _error = self.robot.GetLuaList()
        # size = len(_error)
        error = _error[0]
        if _error[0] == 0:
            lua_num = _error[1]
            lua_name = _error[2].split(';')
            return error, lua_num, lua_name
        else:
            return error,None,None

    """   
    @brief  设置485扩展轴参数
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 int servoCompany 伺服驱动器厂商，1-戴纳泰克
    @param  [in] 必选参数 int servoModel 伺服驱动器型号，1-FD100-750C
    @param  [in] 必选参数 int servoSoftVersion 伺服驱动器软件版本，1-V1.0
    @param  [in] 必选参数 int servoResolution 编码器分辨率
    @param  [in] 必选参数 float axisMechTransRatio 机械传动比  
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetParam(self, servoId, servoCompany, servoModel, servoSoftVersion, servoResolution,
                         axisMechTransRatio):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        servoCompany = int(servoCompany)
        servoModel = int(servoModel)
        servoSoftVersion = int(servoSoftVersion)
        servoResolution = int(servoResolution)
        axisMechTransRatio = float(axisMechTransRatio)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetParam(servoId, servoCompany, servoModel, servoSoftVersion, servoResolution,
                                                    axisMechTransRatio)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取485扩展轴配置参数
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） int servoCompany 伺服驱动器厂商，1-戴纳泰克
    @return 返回值（调用成功返回） servoModel 伺服驱动器型号，1-FD100-750C 
    @return 返回值（调用成功返回） servoSoftVersion 伺服驱动器软件版本，1-V1.0
    @return 返回值（调用成功返回） int servoResolution 编码器分辨率
    @return 返回值（调用成功返回） float axisMechTransRatio 机械传动比
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetParam(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                _error = self.robot.AuxServoGetParam(servoId)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5]
        else:
            return error,None,None,None,None,None

    """   
    @brief  设置485扩展轴使能/去使能
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 int status 使能状态，0-去使能， 1-使能
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoEnable(self, servoId, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoEnable(servoId, status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置485扩展轴控制模式
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 mode 控制模式，0-位置模式，1-速度模式
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetControlMode(self, servoId, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetControlMode(servoId, mode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置485扩展轴目标位置(位置模式)
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 float pos 目标位置，mm或°
    @param  [in] 必选参数 float speed 目标速度，mm/s或°/s
    @param  [in] 必选参数 acc 加速度百分比[0-100] 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetPos(self, servoId, pos, speed,acc=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        pos = float(pos)
        speed = float(speed)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetTargetPos(servoId, pos, speed,acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置485扩展轴目标速度(速度模式)
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 float speed 目标速度，mm/s或°/s
    @param  [in] 必选参数 acc 加速度百分比[0-100] 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetSpeed(self, servoId, speed,acc):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        speed = float(speed)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetTargetSpeed(servoId, speed,acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置485扩展轴目标转矩(力矩模式)
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 float torque 目标力矩，Nm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoSetTargetTorque(self, servoId, torque):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        torque = float(torque)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetTargetTorque(servoId, torque)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置485扩展轴回零
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @param  [in] 必选参数 int mode 回零模式，1-当前位置回零；2-负限位回零；3-正限位回零
    @param  [in] 必选参数 float searchVel 回零速度，mm/s或°/s
    @param  [in] 必选参数 float latchVel 箍位速度，mm/s或°/s
    @param  [in] 必选参数 acc 加速度百分比[0-100] 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoHoming(self, servoId, mode, searchVel, latchVel,acc=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        mode = int(mode)
        searchVel = float(searchVel)
        latchVel = float(latchVel)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoHoming(servoId, mode, searchVel, latchVel,acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  清除485扩展轴错误信息
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoClearError(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoClearError(servoId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取485扩展轴伺服状态
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） servoErrCode 伺服驱动器故障码
    @return 返回值（调用成功返回） servoState 伺服驱动器状态 bit0:0-未使能；1-使能;  bit1:0-未运动；1-正在运动;bit2:0-正限位未触发，1-正限位触发；bit3:0-负限位未触发，1-负限位触发；   bit4 0-未定位完成；1-定位完成；  bit5：0-未回零；1-回零完成
    @return 返回值（调用成功返回） servoPos 伺服当前位置 mm或°
    @return 返回值（调用成功返回） servoSpeed 伺服当前速度 mm/s或°/s
    @return 返回值（调用成功返回） servoTorque 伺服当前转矩Nm
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetStatus(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                _error = self.robot.AuxServoGetStatus(servoId)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5]
        else:
            return error,None,None,None,None,None

    """   
    @brief  设置状态反馈中485扩展轴数据轴号
    @param  [in] 必选参数 int servoId 伺服驱动器ID，范围[1-16],对应从站ID 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AuxServosetStatusID(self, servoId):
        while self.reconnect_flag:
            time.sleep(0.1)
        servoId = int(servoId)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetStatusID(servoId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置机器人外设协议
    @param  [in] 必选参数 int protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExDevProtocol(self, protocol):
        while self.reconnect_flag:
            time.sleep(0.1)
        protocol = int(protocol)
        flag = True
        while flag:
            try:
                error = self.robot.SetExDevProtocol(protocol)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取机器人外设协议
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） int protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
    """

    @log_call
    @xmlrpc_timeout
    def GetExDevProtocol(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetExDevProtocol()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if _error[0] == 0:
            return error, _error[1]
        else:
            return error,None

    """   
    @brief  设置机器人加速度
    @param [in]必选参数 acc 机器人加速度百分比
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetOaccScale(self, acc):
        while self.reconnect_flag:
            time.sleep(0.1)
        acc = float(acc)
        flag = True
        while flag:
            try:
                error = self.robot.SetOaccScale(acc)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  控制箱AO飞拍开始
    @param [in]必选参数 AONum 控制箱AO编号
    @param [in]默认参数 maxTCPSpeed 最大TCP速度值[1-5000mm/s]，默认1000
    @param [in]默认参数 maxAOPercent 最大TCP速度值对应的AO百分比，默认100%
    @param [in]必选参数 zeroZoneCmp 死区补偿值AO百分比，整形，默认为20%，范围[0-100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveAOStart(self, AONum, maxTCPSpeed=1000, maxAOPercent=100, zeroZoneCmp=20):
        while self.reconnect_flag:
            time.sleep(0.1)
        AONum = int(AONum)
        maxTCPSpeed = int(maxTCPSpeed)
        maxAOPercent = int(maxAOPercent)
        zeroZoneCmp = int(zeroZoneCmp)
        flag = True
        while flag:
            try:
                error = self.robot.MoveAOStart(AONum, maxTCPSpeed, maxAOPercent, zeroZoneCmp)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  控制箱AO飞拍停止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveAOStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MoveAOStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  末端AO飞拍开始
    @param [in]必选参数 AONum 末端AO编号
    @param [in]必选参数 maxTCPSpeed 最大TCP速度值[1-5000mm/s]，默认1000
    @param [in]必选参数 maxAOPercent 最大TCP速度值对应的AO百分比，默认100%
    @param [in]必选参数 zeroZoneCmp 死区补偿值AO百分比，整形，默认为20%，范围[0-100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveToolAOStart(self, AONum, maxTCPSpeed=1000, maxAOPercent=100, zeroZoneCmp=20):
        while self.reconnect_flag:
            time.sleep(0.1)
        AONum = int(AONum)
        maxTCPSpeed = int(maxTCPSpeed)
        maxAOPercent = int(maxAOPercent)
        zeroZoneCmp = int(zeroZoneCmp)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToolAOStart(AONum, maxTCPSpeed, maxAOPercent, zeroZoneCmp)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  末端AO飞拍停止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveToolAOStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToolAOStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴通讯参数配置
    @param [in]必选参数 ip PLC IP地址
    @param [in]必选参数 port	端口号
    @param [in]必选参数 period 通讯周期(ms，暂不开放)
    @param [in]必选参数 lossPkgTime	丢包检测时间(ms)
    @param [in]必选参数 lossPkgNum	丢包次数
    @param [in]必选参数 disconnectTime	通讯断开确认时长
    @param [in]必选参数 reconnectEnable	通讯断开自动重连使能 0-不使能 1-使能
    @param [in]必选参数 reconnectPeriod	重连周期间隔(ms)
    @param [in]必选参数 reconnectNum	重连次数
    @param [in]必选参数 selfConnect 断电重启是否自动建立连接；0-不建立连接；1-建立连接
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevSetUDPComParam(self, ip, port, period, lossPkgTime, lossPkgNum, disconnectTime,
                             reconnectEnable, reconnectPeriod, reconnectNum,selfConnect):
        while self.reconnect_flag:
            time.sleep(0.1)
        ip = str(ip)
        port = int(port)
        period = int(period)
        period = 2  # 暂不开放，必须是2
        lossPkgTime = int(lossPkgTime)
        lossPkgNum = int(lossPkgNum)
        disconnectTime = int(disconnectTime)
        reconnectEnable = int(reconnectEnable)
        reconnectPeriod = int(reconnectPeriod)
        reconnectNum = int(reconnectNum)
        selfConnect = int(selfConnect)

        flag = True
        while flag:
            try:
                error = self.robot.ExtDevSetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime,
                                                        reconnectEnable, reconnectPeriod, reconnectNum,selfConnect)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取UDP扩展轴通讯参数
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）ip PLC IP地址
    @return 返回值（调用成功返回）port	端口号
    @return 返回值（调用成功返回）period 通讯周期(ms，暂不开放)
    @return 返回值（调用成功返回） lossPkgTime	丢包检测时间(ms)
    @return 返回值（调用成功返回） lossPkgNum	丢包次数
    @return 返回值（调用成功返回） disconnectTime	通讯断开确认时长
    @return 返回值（调用成功返回） reconnectEnable	通讯断开自动重连使能 0-不使能 1-使能
    @return 返回值（调用成功返回） reconnectPeriod	重连周期间隔(ms)
    @return 返回值（调用成功返回） reconnectNum	重连次数
    @param [out] selfConnect 重启控制箱后是否自动重连；0-不重连；1-重连
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevGetUDPComParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ExtDevGetUDPComParam()
                flag = False
            except socket.error as e:
                flag = True

        if _error[0] == 0:
            return _error[0], [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8],
                               _error[9],_error[10]]
        else:
            return _error[0],None

    """   
    @brief  加载UDP通信
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevLoadUDPDriver(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevLoadUDPDriver()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  卸载UDP通信
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevUnloadUDPDriver(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevUnloadUDPDriver()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴通信异常断开后恢复连接
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevUDPClientComReset(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevUDPClientComReset()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴通信异常断开后关闭通讯
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtDevUDPClientComClose(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtDevUDPClientComClose()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展机器人相对扩展轴位置
    @param [in]必选参数 installType 0-机器人安装在外部轴上，1-机器人安装在外部轴外
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotPosToAxis(self, installType):
        while self.reconnect_flag:
            time.sleep(0.1)
        installType = int(installType)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotPosToAxis(installType)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展轴系统DH参数配置
    @param [in]必选参数 axisConfig 外部轴构型，0-单自由度直线滑轨，1-两自由度L型变位机，2-三自由度，3-四自由度，4-单自由度变位机
    @param [in]必选参数  axisDHd1 外部轴DH参数d1 mm
    @param [in]必选参数  axisDHd2 外部轴DH参数d2 mm
    @param [in]必选参数  axisDHd3 外部轴DH参数d3 mm
    @param [in]必选参数  axisDHd4 外部轴DH参数d4 mm
    @param [in]必选参数  axisDHa1 外部轴DH参数a1 mm
    @param [in]必选参数  axisDHa2 外部轴DH参数a2 mm
    @param [in]必选参数  axisDHa3 外部轴DH参数a3 mm
    @param [in]必选参数  axisDHa4 外部轴DH参数a4 mm
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAxisDHParaConfig(self, axisConfig, axisDHd1, axisDHd2, axisDHd3, axisDHd4, axisDHa1, axisDHa2, axisDHa3,
                            axisDHa4):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisConfig = int(axisConfig)
        axisDHd1 = float(axisDHd1)
        axisDHd2 = float(axisDHd2)
        axisDHd3 = float(axisDHd3)
        axisDHd4 = float(axisDHd4)
        axisDHa1 = float(axisDHa1)
        axisDHa2 = float(axisDHa2)
        axisDHa3 = float(axisDHa3)
        axisDHa4 = float(axisDHa4)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxisDHParaConfig(axisConfig, axisDHd1, axisDHd2, axisDHd3, axisDHd4, axisDHa1, axisDHa2,
                                                       axisDHa3, axisDHa4)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴参数配置
    @param [in]必选参数 axisId 轴号[1-4]
    @param [in]必选参数 axisType 扩展轴类型 0-平移；1-旋转
    @param [in]必选参数 axisDirection 扩展轴方向 0-正向；1-反向
    @param [in]必选参数 axisMax 扩展轴最大位置 mm
    @param [in]必选参数 axisMin 扩展轴最小位置 mm
    @param [in]必选参数 axisVel 速度mm/s
    @param [in]必选参数 axisAcc 加速度mm/s2
    @param [in]必选参数 axisLead 导程mm
    @param [in]必选参数 encResolution 编码器分辨率
    @param [in]必选参数 axisOffect 焊缝起始点扩展轴偏移量
    @param [in]必选参数 axisCompany 驱动器厂家 1-禾川；2-汇川；3-松下
    @param [in]必选参数 axisModel 驱动器型号 1-禾川-SV-XD3EA040L-E，2-禾川-SV-X2EA150A-A，1-汇川-SV620PT5R4I，1-松下-MADLN15SG，2-松下-MSDLN25SG，3-松下-MCDLN35SG
    @param [in]必选参数 axisEncType 编码器类型  0-增量；1-绝对值
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisParamConfig(self, axisId, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead,
                           encResolution, axisOffect, axisCompany, axisModel, axisEncType):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisId = int(axisId)
        axisType = int(axisType)
        axisDirection = int(axisDirection)
        axisMax = float(axisMax)
        axisMin = float(axisMin)
        axisVel = float(axisVel)
        axisAcc = float(axisAcc)
        axisLead = float(axisLead)
        encResolution = int(encResolution)
        axisOffect = float(axisOffect)
        axisCompany = int(axisCompany)
        axisModel = int(axisModel)
        axisEncType = int(axisEncType)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisParamConfig(axisId, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc,
                                                      axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取扩展轴驱动器配置信息
    @param [in]必选参数 axisId 轴号[1-4]
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） axisCompany 驱动器厂家 1-禾川；2-汇川；3-松下
    @return 返回值（调用成功返回） axisModel 驱动器型号 1-禾川-SV-XD3EA040L-E，2-禾川-SV-X2EA150A-A，1-汇川-SV620PT5R4I，1-松下-MADLN15SG，2-松下-MSDLN25SG，3-松下-MCDLN35SG
    @return 返回值（调用成功返回） axisEncType 编码器类型  0-增量；1-绝对值
    """

    @log_call
    @xmlrpc_timeout
    def GetExAxisDriverConfig(self, axisId):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisId = int(axisId)
        flag = True
        while flag:
            try:
                error = self.robot.GetExAxisDriverConfig(axisId)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], [error[1], error[2], error[3]]
        else:
            return error

    """   
    @brief  设置扩展轴坐标系参考点-四点法
    @param [in]必选参数 pointNum 点编号[1-4]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSetRefPoint(self, pointNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisSetRefPoint(pointNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算扩展轴坐标系-四点法
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系值[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisComputeECoordSys(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisComputeECoordSys()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置标定参考点在变位机末端坐标系下位姿
    @param [in]必选参数 pos 位姿值[x,y,z,rx,ry,rz]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetRefPointInExAxisEnd(self, pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        pos = list(map(float, pos))
        flag = True
        while flag:
            try:
                error = self.robot.SetRefPointInExAxisEnd(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  变位机坐标系参考点设置
    @param [in]必选参数 pointNum 点编号[1-4]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def PositionorSetRefPoint(self, pointNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        flag = True
        while flag:
            try:
                error = self.robot.PositionorSetRefPoint(pointNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  变位机坐标系计算-四点法
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系值[x,y,z,rx,ry,rz]
    """

    @log_call
    @xmlrpc_timeout
    def PositionorComputeECoordSys(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.PositionorComputeECoordSys()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None

    """   
    @brief  应用扩展轴坐标系
    @param [in]必选参数 axisCoordNum 坐标系编号
    @param [in]必选参数 toolNum 工具号
    @param [in]必选参数 coord 坐标系值[x,y,z,rx,ry,rz]
    @param [in]必选参数 calibFlag 标定标志 0-否，1-是    
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisActiveECoordSys(self, axisCoordNum, toolNum, coord, calibFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisCoordNum = int(axisCoordNum)
        toolNum = int(toolNum)
        coord = list(map(float, coord))
        calibFlag = int(calibFlag)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisActiveECoordSys(axisCoordNum, toolNum, coord[0], coord[1], coord[2], coord[3],
                                                          coord[4], coord[5], calibFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴使能
    @param [in]必选参数 axisID 轴号[1-4]
    @param [in]必选参数 status 0-去使能；1-使能
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisServoOn(self, axisID, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisID = int(axisID)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisServoOn(axisID, status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴回零
    @param [in]必选参数 axisID 轴号[1-4]
    @param [in]必选参数 mode 回零方式 0当前位置回零，1负限位回零，2-正限位回零
    @param [in]必选参数 searchVel 寻零速度(mm/s)
    @param [in]必选参数 latchVel 寻零箍位速度(mm/s)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSetHoming(self, axisID, mode, searchVel, latchVel):
        while self.reconnect_flag:
            time.sleep(0.1)
        axisID = int(axisID)
        mode = int(mode)
        searchVel = float(searchVel)
        latchVel = float(latchVel)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisSetHoming(axisID, mode, searchVel, latchVel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴点动开始
    @param [in]必选参数 axisID 轴号[1-4]
    @param [in]必选参数 direction 转动方向 0-反向；1-正向
    @param [in]必选参数 vel 速度(mm/s)
    @param [in]必选参数 acc (加速度 mm/s2)
    @param [in]必选参数 maxDistance 最大点动距离
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisStartJog(self, axisID, direction, vel, acc, maxDistance):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        axisID = int(axisID)
        direction = int(direction)
        vel = float(vel)
        acc = float(acc)
        maxDistance = float(maxDistance)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisStartJog(6, axisID, direction, vel, acc, maxDistance)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴点动停止
    @param [in]必选参数 axisID 轴号[1-4]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisStopJog(self, axisID):
        while self.reconnect_flag:
            time.sleep(0.1)

        axisID = int(axisID)
        error =self.send_message("/f/bIII19III240III14IIIStopExtAxisJogIII/b/f")
        # error = self.robot.ExtAxisStartJog(7, axisID, 0, 0.0, 0.0, 0.0)
        return error

    """   
    @brief  设置扩展DO
    @param [in]必选参数 DONum DO编号
    @param [in]必选参数 bOpen 开关 True-开,False-关
    @param [in]必选参数 smooth 是否平滑 True -是, False -否
    @param [in]必选参数 block 是否阻塞 True -是, False -否
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxDO(self, DONum, bOpen, smooth, block):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        bOpen = bool(bOpen)
        smooth = bool(smooth)
        block = bool(block)
        open_flag = 1 if bOpen else 0
        smooth_flag = 1 if smooth else 0
        no_block_flag = 1 if block else 0
        print("open_flag",open_flag)
        print("smooth_flag", smooth_flag)
        print("no_block_flag", no_block_flag)
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxDO(DONum, open_flag, smooth_flag, no_block_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展AO
    @param [in]必选参数 AONum AO编号 
    @param [in]必选参数 value 模拟量值[0-4095]
    @param [in]必选参数 block 是否阻塞 True-是,False-否
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxAO(self, AONum, value, block):
        while self.reconnect_flag:
            time.sleep(0.1)
        AONum = int(AONum)
        value = float(value)
        block = bool(block)
        no_block_flag = 0 if block else 1
        value =value
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxAO(AONum, value, no_block_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展DI输入滤波时间
    @param [in]必选参数 filterTime 滤波时间(ms)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxDIFilterTime(self, filterTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        filterTime = int(filterTime)
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxDIFilterTime(filterTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展AI输入滤波时间
    @param [in]必选参数 AINum AI编号
    @param [in]必选参数 filterTime 滤波时间(ms)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAuxAIFilterTime(self, AINum,filterTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        AINum = int(AINum)
        filterTime = int(filterTime)
        flag = True
        while flag:
            try:
                error = self.robot.SetAuxAIFilterTime(AINum,filterTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  等待扩展DI输入
    @param [in]必选参数 DINum DI编号
    @param [in]必选参数 bOpen 开关 True-开,False-关
    @param [in]必选参数 time 最大等待时间(ms)
    @param [in]必选参数 errorAlarm 是否继续运动 True-是,False-否
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitAuxDI(self, DINum, bOpen, time, errorAlarm):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        bOpen = bool(bOpen)
        open_flag = 0 if bOpen else 1
        time = int(time)
        errorAlarm = bool(errorAlarm)
        errorAlarm_flag = 0 if errorAlarm else 1
        flag = True
        while flag:
            try:
                error = self.robot.WaitAuxDI(DINum, open_flag, time, errorAlarm_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  等待扩展AI输入
    @param [in]必选参数 AINum AI编号
    @param [in]必选参数 sign 0-大于；1-小于
    @param [in]必选参数 value AI值
    @param [in]必选参数 time 最大等待时间(ms)
    @param [in]必选参数 errorAlarm 是否继续运动 True-是,False-否
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitAuxAI(self, AINum, sign, value, time, errorAlarm):
        while self.reconnect_flag:
            time.sleep(0.1)
        AINum = int(AINum)
        sign = int(sign)
        value = int(value)
        time = int(time)
        errorAlarm = bool(errorAlarm)
        errorAlarm_flag = 0 if errorAlarm else 1
        flag = True
        while flag:
            try:
                error = self.robot.WaitAuxAI(AINum, sign, value, time, errorAlarm_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取扩展DI值
    @param [in]必选参数 DINum DI编号
    @param [in]必选参数 isNoBlock 是否阻塞 True-阻塞 false-非阻塞
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） isOpen 0-关；1-开
    """

    @log_call
    @xmlrpc_timeout
    def GetAuxDI(self, DINum, isNoBlock):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        isNoBlock = bool(isNoBlock)
        isNoBlock_flag = 0 if isNoBlock else 1
        flag = True
        while flag:
            try:
                error = self.robot.GetAuxDI(DINum, isNoBlock_flag)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1]
        else:
            return error

    """   
    @brief  获取扩展AI值
    @param [in]必选参数 AINum AI编号
    @param [in]必选参数 isNoBlock 是否阻塞 True-阻塞 False-非阻塞
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） value 输入值
    """

    @log_call
    @xmlrpc_timeout
    def GetAuxAI(self, AINum, isNoBlock):
        while self.reconnect_flag:
            time.sleep(0.1)
        AINum = int(AINum)
        isNoBlock = bool(isNoBlock)
        isNoBlock_flag = 0 if isNoBlock else 1
        flag = True
        while flag:
            try:
                error = self.robot.GetAuxAI(AINum, isNoBlock_flag)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1]
        else:
            return error

    """   
    @brief  UDP扩展轴运动
    @param [in]必选参数 pos 目标位置 轴 1 位置 ~ 轴 4 位置[exaxis[0],exaxis[1],exaxis[2],exaxis[3]]
    @param [in]必选参数 ovl 速度百分比
    @param [in]必选参数 blend 平滑参数(mm或ms)，-1,等待运动完成
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisMove(self, pos, ovl, blend=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        pos = list(map(float, pos))
        ovl = float(ovl)
        blend = float(blend)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisMoveJ(0, pos[0], pos[1], pos[2], pos[3], ovl, blend)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴与机器人关节运动同步运动(自动正/逆运动学计算)
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放,默认0.0 
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0    
    @param  [in] 默认参数 blendT:[-1.0]-运动到位 (阻塞)，[0~500.0]-平滑时间 (非阻塞)，单位 [ms] 默认-1.0
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveJ(self, joint_pos, tool, user, exaxis_pos, desc_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
                         blendT=-1.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        tool = int(tool)
        user = int(user)
        desc_pos = list(map(float, desc_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        exaxis_pos = list(map(float, exaxis_pos))
        blendT = float(blendT)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        if (desc_pos[0] == 0.0) and (desc_pos[1] == 0.0) and (desc_pos[2] == 0.0) and (desc_pos[3] == 0.0) and (
                desc_pos[4] == 0.0) and (desc_pos[5] == 0.0):  # 若未输入参数则调用正运动学求解
            ret = self.robot.GetForwardKin(joint_pos)  # 正运动学求解
            if ret[0] == 0:
                desc_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos[0], exaxis_pos[1], exaxis_pos[2], exaxis_pos[3], ovl, blendT)
        if error != 0:
            return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveJ(joint_pos, desc_pos, tool, user, vel, acc, ovl, exaxis_pos, blendT, offset_flag,
                                         offset_pos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴与机器人直线运动同步运动(自动正/逆运动学计算)
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°] 
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0    
    @param  [in] 默认参数 search:[0]-不焊丝寻位，[1]-焊丝寻位
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 config 逆解关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解，默认-1
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveL(self, desc_pos, tool, user, exaxis_pos, joint_pos, vel=20.0, acc=0.0, ovl=100.0,
                         blendR=-1.0, search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        blendR = float(blendR)
        exaxis_pos = list(map(float, exaxis_pos))
        search = int(search)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))
        config = int(config)

        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, config)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos[0], exaxis_pos[1], exaxis_pos[2], exaxis_pos[3], ovl, blendR)
        if error != 0:
            return error
        flag = True
        while flag:
            try:
                error = self.MoveL(joint_pos=joint_pos,desc_pos=desc_pos,tool= tool,user= user,vel= vel, acc=acc,ovl= ovl,blendR= blendR,blendMode=0,exaxis_pos= exaxis_pos,search= search,
                                         offset_flag=offset_flag,offset_pos= offset_pos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴与机器人圆弧运动同步运动(自动正/逆运动学计算)
    @param  [in] 必选参数 joint_pos_p: 路径点关节位置，单位 [°] 
    @param  [in] 必选参数 desc_pos_p: 路径点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_p: 路径点工具号，[0~14]
    @param  [in] 必选参数 user_p: 路径点工件号，[0~14]
    @param  [in] 必选参数 exaxis_pos_p: 路径点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 必选参数 joint_pos_t: 目标点关节位置，单位 [°] 
    @param  [in] 必选参数 desc_pos_t: 目标点笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool_t: 工具号，[0~14]
    @param  [in] 必选参数 user_t: 工件号，[0~14]
    @param  [in] 必选参数 exaxis_pos_t: 目标点外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]    
    @param  [in] 默认参数 vel_p: 路径点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_p: 路径点加速度百分比，[0~100] 暂不开放,默认0.0    
    @param  [in] 默认参数 offset_flag_p: 路径点是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos_p: 路径点位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 vel_t: 目标点速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc_t: 目标点加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 offset_flag_t: 目标点是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos_t: 目标点位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @param  [in] 默认参数 config 逆解关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解，默认-1
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveC(self, desc_pos_p, tool_p, user_p, exaxis_pos_p, desc_pos_t, tool_t,
                         user_t, exaxis_pos_t,
                         joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         vel_p=20.0, acc_p=100.0, offset_flag_p=0,
                         offset_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         vel_t=20.0, acc_t=100.0, offset_flag_t=0,
                         offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         ovl=100.0, blendR=-1.0,config=-1):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos_p = list(map(float, desc_pos_p))
        tool_p = float(int(tool_p))
        user_p = float(int(user_p))
        joint_pos_p = list(map(float, joint_pos_p))
        vel_p = float(vel_p)
        acc_p = float(acc_p)
        exaxis_pos_p = list(map(float, exaxis_pos_p))
        offset_flag_p = int(offset_flag_p)
        offset_pos_p = list(map(float, offset_pos_p))

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = float(int(tool_t))
        user_t = float(int(user_t))
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))
        offset_flag_t = int(offset_flag_t)
        offset_pos_t = list(map(float, offset_pos_t))

        ovl = float(ovl)
        blendR = float(blendR)
        config = int(config)

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, config)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, config)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos_t[0], exaxis_pos_t[1], exaxis_pos_t[2], exaxis_pos_t[3], ovl, blendR)
        if error != 0:
            return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveC(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p, offset_flag_p,
                                         offset_pos_p, joint_pos_t, desc_pos_t, [tool_t, user_t, vel_t, acc_t], exaxis_pos_t,
                                         offset_flag_t, offset_pos_t, ovl, blendR)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  焊丝寻位开始
    @param [in]必选参数 refPos  1-基准点 2-接触点
    @param [in]必选参数 searchVel   寻位速度 %
    @param [in]必选参数 searchDis  寻位距离 mm
    @param [in]必选参数 autoBackFlag 自动返回标志，0-不自动；-自动
    @param [in]必选参数 autoBackVel  自动返回速度 %
    @param [in]必选参数 autoBackDis  自动返回距离 mm
    @param [in]必选参数 offectFlag  1-带偏移量寻位；2-示教点寻位
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WireSearchStart(self, refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        refPos = int(refPos)
        searchVel = float(searchVel)
        searchDis = int(searchDis)
        autoBackFlag = int(autoBackFlag)
        autoBackVel = float(autoBackVel)
        autoBackDis = int(autoBackDis)
        offectFlag = int(offectFlag)
        flag = True
        while flag:
            try:
                error = self.robot.WireSearchStart(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  焊丝寻位结束
    @param [in]必选参数 refPos  1-基准点 2-接触点
    @param [in]必选参数 searchVel   寻位速度 %
    @param [in]必选参数 searchDis  寻位距离 mm
    @param [in]必选参数 autoBackFlag 自动返回标志，0-不自动；-自动
    @param [in]必选参数 autoBackVel  自动返回速度 %
    @param [in]必选参数 autoBackDis  自动返回距离 mm
    @param [in]必选参数 offectFlag  1-带偏移量寻位；2-示教点寻位
    @return 错误码 成功- 0, 失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def WireSearchEnd(self, refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        refPos = int(refPos)
        searchVel = float(searchVel)
        searchDis = int(searchDis)
        autoBackFlag = int(autoBackFlag)
        autoBackVel = float(autoBackVel)
        autoBackDis = int(autoBackDis)
        offectFlag = int(offectFlag)
        flag = True
        while flag:
            try:
                error = self.robot.WireSearchEnd(refPos,searchVel,searchDis,autoBackFlag,autoBackVel,autoBackDis,offectFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  计算焊丝寻位偏移量
    @param  [in]必选参数 seamType  焊缝类型
    @param  [in]必选参数 method   计算方法
    @param  [in]必选参数 varNameRef 基准点1-6，“#”表示无点变量
    @param  [in]必选参数 varNameRes 接触点1-6，“#”表示无点变量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） offectFlag 0-偏移量直接叠加到指令点；1-偏移量需要对指令点进行坐标变换
    @return 返回值（调用成功返回） offect 偏移位姿[x, y, z, a, b, c]
    """
    @log_call
    @xmlrpc_timeout
    def GetWireSearchOffset(self, seamType, method,varNameRef,varNameRes):
        while self.reconnect_flag:
            time.sleep(0.1)
        seamType = int(seamType)
        method = int(method)
        if(len(varNameRes)!=6):
            return 4
        if(len(varNameRes)!=6):
            return 4
        varNameRef = list(map(str, varNameRef))
        varNameRes = list(map(str, varNameRes))

        flag = True
        while flag:
            try:
                _error = self.robot.GetWireSearchOffset(seamType, method, varNameRef[0], varNameRef[1], varNameRef[2], varNameRef[3], varNameRef[4], varNameRef[5],
                                                        varNameRes[0], varNameRes[1], varNameRes[2], varNameRes[3], varNameRes[4], varNameRes[5])
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], [_error[2], _error[3], _error[4], _error[5], _error[6], _error[7]]
        else:
            return error,None,None

    """   
    @brief  等待焊丝寻位完成
    @param  [in]必选参数 varName  接触点名称 “RES0” ~ “RES99”
    @return 错误码 成功- 0, 失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def WireSearchWait(self,varname):
        while self.reconnect_flag:
            time.sleep(0.1)
        varname=str(varname)
        flag = True
        while flag:
            try:
                error = self.robot.WireSearchWait(varname)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  焊丝寻位接触点写入数据库
    @param  [in]必选参数 varName  接触点名称 “RES0” ~ “RES99”
    @param  [in]必选参数 pos  接触点数据[x, y, x, a, b, c]
    @return 错误码 成功- 0, 失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def SetPointToDatabase(self,varName,pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        varName = str(varName)
        pos = list(map(float,pos))

        flag = True
        while flag:
            try:
                error = self.robot.SetPointToDatabase(varName,pos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  电弧跟踪控制
    @param  [in]必选参数 flag 开关，0-关；1-开
    @param  [in]必选参数 delayTime 滞后时间，单位ms
    @param  [in]必选参数 isLeftRight 左右偏差补偿 0-关闭，1-开启
    @param  [in]必选参数 klr 左右调节系数(灵敏度)
    @param  [in]必选参数 tStartLr 左右开始补偿时间cyc
    @param  [in]必选参数 stepMaxLr 左右每次最大补偿量 mm
    @param  [in]必选参数 sumMaxLr 左右总计最大补偿量 mm
    @param  [in]必选参数 isUpLow 上下偏差补偿 0-关闭，1-开启
    @param  [in]必选参数 kud 上下调节系数(灵敏度)
    @param  [in]必选参数 tStartUd 上下开始补偿时间cyc
    @param  [in]必选参数 stepMaxUd 上下每次最大补偿量 mm
    @param  [in]必选参数 sumMaxUd 上下总计最大补偿量
    @param  [in]必选参数 axisSelect 上下坐标系选择，0-摆动；1-工具；2-基座
    @param  [in]必选参数 referenceType 上下基准电流设定方式，0-反馈；1-常数
    @param  [in]必选参数 referSampleStartUd 上下基准电流采样开始计数(反馈)，cyc
    @param  [in]必选参数 referSampleCountUd 上下基准电流采样循环计数(反馈)，cyc
    @param  [in]必选参数 referenceCurrent 上下基准电流mA
    @param  [in]必选参数 offsetType 偏置跟踪类型，0-不偏置；1-采样；2-百分比
    @param  [in]必选参数 offsetParameter 偏置参数；采样(偏置采样开始时间，默认采一周期)；百分比(偏置百分比(-100 ~ 100))
    @return 错误码 成功- 0, 失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceControl(self,flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd,
                            sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent, offsetType, offsetParameter):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        delaytime = float(delaytime)
        isLeftRight = int(isLeftRight)
        klr = float(klr)
        tStartLr = float(tStartLr)
        stepMaxLr = float(stepMaxLr)
        sumMaxLr = float(sumMaxLr)
        isUpLow = int(isUpLow)
        kud = float(kud)
        tStartUd = float(tStartUd)
        stepMaxUd = float(stepMaxUd)
        sumMaxUd = float(sumMaxUd)
        axisSelect = int(axisSelect)
        referenceType = int(referenceType)
        referSampleStartUd = float(referSampleStartUd)
        referSampleCountUd = float(referSampleCountUd)
        referenceCurrent = float(referenceCurrent)
        offsetType = int(offsetType)
        offsetParameter = int(offsetParameter)

        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.ArcWeldTraceControl(flag,delaytime, isLeftRight, [klr, tStartLr, stepMaxLr, sumMaxLr], isUpLow, [kud, tStartUd, stepMaxUd,
                                                       sumMaxUd], axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent, offsetType, offsetParameter)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """   
    @brief  电弧跟踪AI通带选择
    @param  [in]必选参数 channel 电弧跟踪AI通带选择,[0-3]
    @return 错误码 成功- 0, 失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceExtAIChannelConfig(self,channel):
        while self.reconnect_flag:
            time.sleep(0.1)
        channel = int(channel)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceExtAIChannelConfig(channel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  力传感器辅助拖动
    @param  [in]必选参数 status 控制状态，0-关闭；1-开启
    @param  [in]必选参数 asaptiveFlag 自适应开启标志，0-关闭；1-开启
    @param  [in]必选参数 interfereDragFlag 干涉区拖动标志，0-关闭；1-开启
    @param  [in]必选参数 ingularityConstraintsFlag 奇异点策略，0-规避；1-穿越
    @param  [in]必选参数 forceCollisionFlag 辅助拖动时机器人碰撞检测标志；0-关闭；1-开启
    @param  [in]必选参数 M=[m1,m2,m3,m4,m5,m6] 惯性系数 
    @param  [in]必选参数 B=[b1,b2,b3,b4,b5,b6] 阻尼系数
    @param  [in]必选参数 K=[k1,k2,k3,k4,k5,k6] 刚度系数
    @param  [in]必选参数 F=[f1,f2,f3,f4,f5,f6] 拖动六维力阈值
    @param  [in]必选参数 Fmax 最大拖动力限制
    @param  [in]必选参数 Vmax 最大关节速度限制
    @return 错误码 成功- 0, 失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def EndForceDragControl(self, status, asaptiveFlag, interfereDragFlag, ingularityConstraintsFlag, forceCollisionFlag, M, B, K, F, Fmax, Vmax):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        asaptiveFlag = int(asaptiveFlag)
        interfereDragFlag = int(interfereDragFlag)
        ingularityConstraintsFlag = int(ingularityConstraintsFlag)
        M = list(map(float,M))
        B = list(map(float,B))
        K = list(map(float,K))
        F = list(map(float,F))
        Fmax = float(Fmax)
        Vmax = float(Vmax)
        forceCollisionFlag = int(forceCollisionFlag)
        flag = True
        while flag:
            try:
                error = self.robot.EndForceDragControl(status, asaptiveFlag, interfereDragFlag, ingularityConstraintsFlag,forceCollisionFlag, M, B, K, F, Fmax, Vmax)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  报错清除后力传感器自动开启
    @param  [in]必选参数 status 控制状态，0-关闭；1-开启
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetForceSensorDragAutoFlag(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.SetForceSensorDragAutoFlag(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置六维力和关节阻抗混合拖动开关及参数
    @param  [in]必选参数 status 控制状态，0-关闭；1-开启
    @param  [in]必选参数 impedanceFlag 阻抗开启标志，0-关闭；1-开启
    @param  [in]必选参数 lamdeDain 拖动增益
    @param  [in]必选参数 KGain 刚度增益
    @param  [in]必选参数 BGain 阻尼增益
    @param  [in]必选参数 dragMaxTcpVel 拖动末端最大线速度限制
    @param  [in]必选参数 dragMaxTcpOriVel 拖动末端最大角速度限制
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ForceAndJointImpedanceStartStop(self,status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        impedanceFlag = int(impedanceFlag)
        if((len(lamdeDain)!=6)or(len(KGain)!=6)or(len(BGain)!=6)):
            return 4
        lamdeDain = list(map(float,lamdeDain))
        KGain = list(map(float,KGain))
        BGain = list(map(float,BGain))
        dragMaxTcpVel = float(dragMaxTcpVel)
        dragMaxTcpOriVel = float(dragMaxTcpOriVel)
        flag = True
        while flag:
            try:
                error = self.robot.ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  获取力传感器拖动开关状态
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） dragState 力传感器辅助拖动控制状态，0-关闭；1-开启
    @return 返回值（调用成功返回） sixDimensionalDragState 六维力辅助拖动控制状态，0-关闭；1-开启
    """

    @log_call
    @xmlrpc_timeout
    def GetForceAndTorqueDragState(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetForceAndTorqueDragState()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        else:
            return error,None,None

    """   
    @brief  设置力传感器下负载重量
    @param  [in]必选参数 weight 负载重量 kg
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetForceSensorPayload(self,weight):
        while self.reconnect_flag:
            time.sleep(0.1)
        weight = float(weight)
        flag = True
        while flag:
            try:
                error = self.robot.SetForceSensorPayload(weight)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置力传感器下负载重量
    @param  [in]必选参数 x 负载质心x mm 
    @param  [in]必选参数 y 负载质心y mm 
    @param  [in]必选参数 z 负载质心z mm 
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetForceSensorPayloadCog(self,x,y,z):
        while self.reconnect_flag:
            time.sleep(0.1)
        x = float(x)
        y = float(y)
        z = float(z)
        flag = True
        while flag:
            try:
                error = self.robot.SetForceSensorPayloadCog(x,y,z)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取力传感器下负载重量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） weight 负载重量 kg
    """

    @log_call
    @xmlrpc_timeout
    def GetForceSensorPayload(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetForceSensorPayload()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None


    """   
    @brief  获取力传感器下负载质心
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） x 负载质心x mm 
    @return 返回值（调用成功返回） y 负载质心y mm 
    @return 返回值（调用成功返回） z 负载质心z mm 
    """

    @log_call
    @xmlrpc_timeout
    def GetForceSensorPayloadCog(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetForceSensorPayloadCog()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3]
        else:
            return error,None,None,None

    """   
    @brief  力传感器自动校零
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） weight 传感器质量 kg 
    @return 返回值（调用成功返回） pos=[x,y,z] 传感器质心 mm
    """
    @log_call
    @xmlrpc_timeout
    def ForceSensorAutoComputeLoad(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        rtn = self.ForceSensorSetSaveDataFlag(1)
        if rtn!=0:
            return rtn,None,None
        error =self.GetActualJointPosDegree()
        start_joint = error[1]
        error = self.GetActualJointPosDegree()
        if error[0]==0:
            joint =error[1]
            if joint[2]<0:
                joint[3] = joint[3] + 90
            else:
                joint[3] = joint[3] - 90
            rtn = self.MoveJ(joint,0,0,vel=10)
            if rtn!=0:
                return rtn,None,None
        else:
            return error,None,None

        rtn = self.ForceSensorSetSaveDataFlag(2)
        if rtn!=0:
            return rtn,None,None

        error = self.GetActualJointPosDegree()
        if error[0] == 0:
            joint = error[1]
            if joint[5] < 0:
                joint[5] = joint[5] + 90
            else:
                joint[5] = joint[5] - 90
            rtn = self.MoveJ(joint, 0, 0,vel=10)
            if rtn != 0:
                return rtn,None,None
        else:
            return error,None,None

        rtn = self.ForceSensorSetSaveDataFlag(3)
        if rtn!=0:
            return rtn,None,None

        _error = self.robot.ForceSensorComputeLoad()
        error = _error[0]
        self.MoveJ(start_joint,0,0,vel=10)
        if error == 0:
            return error, _error[1],[_error[2],_error[3],_error[4]]
        else:
            return error,None,None

    """   
    @brief  传感器自动校零数据记录
    @param  [in]必选参数 recordCount 记录数据个数 1-3
    @return 错误码 成功- 0, 失败-错误码
    """
    @log_call
    @xmlrpc_timeout
    def ForceSensorSetSaveDataFlag(self,recordCount):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ForceSensorSetSaveDataFlag(recordCount)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  传感器自动校零计算
    @return 错误码 成功- 0, 失败-错误码    
    @return 返回值（调用成功返回） weight 传感器质量 kg 
    @return 返回值（调用成功返回） pos=[x,y,z] 传感器质心 mm
    """

    @log_call
    @xmlrpc_timeout
    def ForceSensorComputeLoad(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.ForceSensorComputeLoad()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1],[_error[2],_error[3],_error[4]]
        else:
            return error,None,None

    """   
    @brief  末端传感器配置
    @param  [in]必选参数 idCompany 厂商，18-JUNKONG；25-HUIDE
    @param  [in]必选参数 idDevice 类型，0-JUNKONG/RYR6T.V1.0
    @param  [in]必选参数 idSoftware 软件版本，0-J1.0/HuiDe1.0(暂未开放)
    @param  [in]必选参数 idBus 挂载位置，1-末端1号口；2-末端2号口...8-末端8号口(暂未开放)
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorConfig(self,idCompany, idDevice, idSoftware, idBus):
        while self.reconnect_flag:
            time.sleep(0.1)
        idCompany = int(idCompany)
        idDevice = int(idDevice)
        idSoftware = int(idSoftware)
        idBus = int(idBus)

        flag = True
        while flag:
            try:
                error = self.robot.AxleSensorConfig(idCompany, idDevice, idSoftware, idBus)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取末端传感器配置    
    @return 错误码 成功- 0, 失败-错误码  
    @return 返回值（调用成功返回） idCompany 厂商，18-JUNKONG；25-HUIDE
    @return 返回值（调用成功返回） idDevice 类型，0-JUNKONG/RYR6T.V1.0
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorConfigGet(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.AxleSensorConfigGet()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        else:
            return error,None,None

    """   
    @brief  末端传感器激活
    @param  [in]必选参数 actFlag 0-复位；1-激活
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorActivate(self,actFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        actFlag = int(actFlag)
        flag = True
        while flag:
            try:
                error = self.robot.AxleSensorActivate(actFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  末端传感器寄存器写入
    @param  [in]必选参数 devAddr  设备地址编号 0-255
    @param  [in]必选参数 regHAddr 寄存器地址高8位
    @param  [in]必选参数 regLAddr 寄存器地址低8位
    @param  [in]必选参数 regNum  寄存器个数 0-255
    @param  [in]必选参数 data1 写入寄存器数值1
    @param  [in]必选参数 data2 写入寄存器数值2
    @param  [in]必选参数 isNoBlock 是否阻塞 0-阻塞；1-非阻塞
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def AxleSensorRegWrite(self,devAddr, regHAddr, regLAddr, regNum, data1, data2, isNoBlock):
        while self.reconnect_flag:
            time.sleep(0.1)
        devAddr = int(devAddr)
        regHAddr = int(regHAddr)
        regLAddr = int(regLAddr)
        regNum = int(regNum)
        data1 = int(data1)
        data2 = int(data2)
        isNoBlock = int(isNoBlock)
        flag = True
        while flag:
            try:
                error = self.robot.AxleSensorRegWrite(devAddr, regHAddr, regLAddr, regNum, data1, data2, isNoBlock)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置控制箱DO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @param  [in]必选参数 reloadFlag 暂停恢复后是否重加载，0-不加载；1-加载
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetCtlBoxDO(self,resetFlag,reloadFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        reloadFlag = int(reloadFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetCtlBoxDO(resetFlag,reloadFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置控制箱AO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @param  [in]必选参数 reloadFlag 暂停恢复后是否重加载，0-不加载；1-加载
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetCtlBoxAO(self,resetFlag,reloadFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        reloadFlag = int(reloadFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetCtlBoxAO(resetFlag,reloadFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置末端工具DO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @param  [in]必选参数 reloadFlag 暂停恢复后是否重加载，0-不加载；1-加载
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetAxleDO(self,resetFlag,reloadFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        reloadFlag = int(reloadFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetAxleDO(resetFlag,reloadFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置末端工具AO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @param  [in]必选参数 reloadFlag 暂停恢复后是否重加载，0-不加载；1-加载
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetAxleAO(self,resetFlag,reloadFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        reloadFlag = int(reloadFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetAxleAO(resetFlag,reloadFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展DO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @param  [in]必选参数 reloadFlag 暂停恢复后是否重加载，0-不加载；1-加载
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetExtDO(self,resetFlag,reloadFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        reloadFlag = int(reloadFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetExtDO(resetFlag,reloadFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展AO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @param  [in]必选参数 reloadFlag 暂停恢复后是否重加载，0-不加载；1-加载
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetExtAO(self,resetFlag,reloadFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        reloadFlag = int(reloadFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetExtAO(resetFlag,reloadFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置SmartTool停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @param  [in]必选参数 reloadFlag 暂停恢复后是否重加载，0-不加载；1-加载
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetSmartToolDO(self,resetFlag,reloadFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        reloadFlag = int(reloadFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetSmartToolDO(resetFlag,reloadFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  仿真摆动开始
    @param  [in]必选参数 weaveNum  摆动参数编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveStartSim(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveStartSim(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  仿真摆动结束
    @param  [in]必选参数 weaveNum  摆动参数编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveEndSim(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveEndSim(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  开始轨迹检测预警(不运动)
    @param  [in]必选参数 weaveNum  摆动参数编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveInspectStart(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveInspectStart(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  结束轨迹检测预警(不运动)
    @param  [in]必选参数 weaveNum  摆动参数编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def WeaveInspectEnd(self,weaveNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveNum = int(weaveNum)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveInspectEnd(weaveNum)
                flag = False
            except socket.error as e:
                flag = True

        return error



    """   
    @brief  设置焊接工艺曲线参数
    @param  [in]必选参数 id 焊接工艺编号(1-99)
    @param  [in]必选参数 startCurrent 起弧电流(A)
    @param  [in]必选参数 startVoltage 起弧电压(V)
    @param  [in]必选参数 startTime 起弧时间(ms)
    @param  [in]必选参数 weldCurrent 焊接电流(A)
    @param  [in]必选参数 weldVoltage 焊接电压(V)
    @param  [in]必选参数 endCurrent 收弧电流(A)
    @param  [in]必选参数 endVoltage 收弧电压(V)
    @param  [in]必选参数 endTime 收弧时间(ms)
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetProcessParam(self, id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent,
                               endVoltage, endTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        startCurrent = float(startCurrent)
        startVoltage = float(startVoltage)
        startTime = float(startTime)
        weldCurrent = float(weldCurrent)
        weldVoltage = float(weldVoltage)
        endCurrent = float(endCurrent)
        endVoltage = float(endVoltage)
        endTime = float(endTime)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage,
                                                          endCurrent, endVoltage, endTime)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取焊接工艺曲线参数    
    @param  [in]必选参数 id 焊接工艺编号(1-99)
    @return 错误码 成功- 0, 失败-错误码  
    @return 返回值（调用成功返回） startCurrent 起弧电流(A)
    @return 返回值（调用成功返回） startVoltage 起弧电压(V)
    @return 返回值（调用成功返回） startTime 起弧时间(ms)
    @return 返回值（调用成功返回） weldCurrent 焊接电流(A)
    @return 返回值（调用成功返回） weldVoltage 焊接电压(V)
    @return 返回值（调用成功返回） endCurrent 收弧电流(A)
    @return 返回值（调用成功返回） endVoltage 收弧电压(V)
    @return 返回值（调用成功返回） endTime 收弧时间(ms)
    """

    @log_call
    @xmlrpc_timeout
    def WeldingGetProcessParam(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.WeldingGetProcessParam(id)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3], _error[4], _error[5], _error[6], _error[7], _error[8]
        else:
            return error,None,None,None,None,None,None,None,None

    """   
    @brief  扩展IO-配置焊机气体检测信号
    @param  [in]必选参数 DONum  气体检测信号扩展DO编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetAirControlExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetAirControlExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  扩展IO-配置焊机起弧信号
    @param  [in]必选参数 DONum  焊机起弧信号扩展DO编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetArcStartExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetArcStartExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  扩展IO-配置焊机反向送丝信号
    @param  [in]必选参数 DONum  反向送丝信号扩展DO编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetWireReverseFeedExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWireReverseFeedExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  扩展IO-配置焊机正向送丝信号
    @param  [in]必选参数 DONum  正向送丝信号扩展DO编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetWireForwardFeedExtDoNum(self,DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWireForwardFeedExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  扩展IO-配置焊机起弧成功信号
    @param  [in]必选参数 DINum  起弧成功信号扩展DI编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetArcDoneExtDiNum(self,DINum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        flag = True
        while flag:
            try:
                error = self.robot.SetArcDoneExtDiNum(DINum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  扩展IO-配置焊机准备信号
    @param  [in]必选参数 DINum  焊机准备信号扩展DI编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetWeldReadyExtDiNum(self,DINum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DINum = int(DINum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWeldReadyExtDiNum(DINum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  扩展IO-配置焊接中断恢复信号
    @param  [in]必选参数 reWeldDINum  焊接中断后恢复焊接信号扩展DI编号
    @param  [in]必选参数 abortWeldDINum  焊接中断后退出焊接信号扩展DI编号
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetExtDIWeldBreakOffRecover(self,reWeldDINum, abortWeldDINum):
        while self.reconnect_flag:
            time.sleep(0.1)
        reWeldDINum = int(reWeldDINum)
        abortWeldDINum = int(abortWeldDINum)
        flag = True
        while flag:
            try:
                error = self.robot.SetExtDIWeldBreakOffRecover(reWeldDINum, abortWeldDINum)
                flag = False
            except socket.error as e:
                flag = True

        return error



    """   
    @brief  设置机器人碰撞检测方法
    @param  [in]必选参数 method 碰撞检测方法：0-电流模式；1-双编码器；2-电流和双编码器同时开启
    @param  [in]必选参数 thresholdMode 碰撞等级阈值方式；0-碰撞等级固定阈值方式；1-自定义碰撞检测阈值
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetCollisionDetectionMethod(self,method,thresholdMode):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        thresholdMode = int(thresholdMode)
        flag = True
        while flag:
            try:
                error = self.robot.SetCollisionDetectionMethod(method,thresholdMode)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置静态下碰撞检测开始关闭
    @param  [in]必选参数 status 0-关闭；1-开启
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetStaticCollisionOnOff(self,status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.SetStaticCollisionOnOff(status)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  关节扭矩功率检测
    @param  [in]必选参数 status 0-关闭；1-开启
    @param  [in]必选参数 power 设定最大功率(W)
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetPowerLimit(self,status, power):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        power = float(power)
        flag = True
        while flag:
            try:
                error = self.robot.SetPowerLimit(status, power)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置机器人 20004 端口反馈周期
    @param  [in]必选参数 period 机器人 20004 端口反馈周期(ms)
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetRobotRealtimeStateSamplePeriod(self,period):
        while self.reconnect_flag:
            time.sleep(0.1)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotRealtimeStateSamplePeriod(period)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取机器人 20004 端口反馈周期
    @param  [in]NULL
    @return 错误码 成功- 0, 失败-错误码    
    @return 返回值（调用成功返回） period 机器人 20004 端口反馈周期(ms)
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotRealtimeStateSamplePeriod(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotRealtimeStateSamplePeriod()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        else:
            return error,None


    """   
    @brief  获取关节驱动器当前扭矩
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data=[j1,j2,j3,j4,j5,j6] 关节扭矩    [fx,fy,fz,tx,ty,tz]
    """
    @log_call
    @xmlrpc_timeout
    def GetJointDriverTorque(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0,[self.robot_state_pkg.jointDriverTorque[0],self.robot_state_pkg.jointDriverTorque[1],self.robot_state_pkg.jointDriverTorque[2],
                  self.robot_state_pkg.jointDriverTorque[3],self.robot_state_pkg.jointDriverTorque[4],self.robot_state_pkg.jointDriverTorque[5]]


    """   
    @brief  获取关节驱动器当前温度
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）data=[t1,t2,t3,t4,t5,t6]
    """
    @log_call
    @xmlrpc_timeout
    def GetJointDriverTemperature (self):
        return 0,[self.robot_state_pkg.jointDriverTemperature [0],self.robot_state_pkg.jointDriverTemperature [1],self.robot_state_pkg.jointDriverTemperature[2],
                  self.robot_state_pkg.jointDriverTemperature [3],self.robot_state_pkg.jointDriverTemperature[4],self.robot_state_pkg.jointDriverTemperature[5]]



    """   
    @brief  电弧追踪 + 多层多道补偿开启
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceReplayStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceReplayStart()
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  电弧追踪 + 多层多道补偿关闭
    @param  [in] 必选参数 NULL
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceReplayEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceReplayEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  偏移量坐标变化-多层多道焊
    @param  [in] pointo 基准点笛卡尔位姿
    @param  [in] pointX 基准点X向偏移方向点笛卡尔位姿
    @param  [in] pointZ 基准点Z向偏移方向点笛卡尔位姿
    @param  [in] dx x方向偏移量(mm)
    @param  [in] dz z方向偏移量(mm)
    @param  [in] dry 绕y轴偏移量(°)
    @return 错误码 成功- 0, 失败-错误码    
    @return 返回值（调用成功返回） offset 计算结果偏移量
    """

    @log_call
    @xmlrpc_timeout
    def MultilayerOffsetTrsfToBase(self,pointo,pointX,pointZ,dx,dz,dry):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointo =list(map(float,pointo))
        pointX = list(map(float, pointX))
        pointZ = list(map(float, pointZ))
        dx = float(dx)
        dz = float(dz)
        dry = float(dry)
        flag = True
        while flag:
            try:
                _error = self.robot.MultilayerOffsetTrsfToBase(pointo[0],pointo[1],pointo[2],
                                                               pointX[0],pointX[1],pointX[2],pointZ[0],pointZ[1],pointZ[2],dx,dz,dry)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        else:
            return error,None


    """   
    @brief  指定姿态速度开启
    @param  [in]必选参数 ratio 姿态速度百分比[0-300]
    @return 错误码 成功- 0, 失败-错误码    
    """
    @log_call
    @xmlrpc_timeout
    def AngularSpeedStart(self, ratio):
        while self.reconnect_flag:
            time.sleep(0.1)
        ratio = int(ratio)
        flag = True
        while flag:
            try:
                error = self.robot.AngularSpeedStart(ratio)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  指定姿态速度关闭
    @return 错误码 成功- 0, 失败-错误码    
    """
    @log_call
    @xmlrpc_timeout
    def AngularSpeedEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.AngularSpeedEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  机器人软件升级
    @param  [in]必选参数  filePath 软件升级包全路径
    @param  [in]必选参数 block 是否阻塞至升级完成 true:阻塞；false:非阻塞
    @return 错误码 成功- 0, 失败-错误码    
    """
    @log_call
    @xmlrpc_timeout
    def SoftwareUpgrade(self,filePath, block):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.__FileUpLoad(1,filePath)

        print("__FileUpLoad", error)
        if 0==error:
            self.log_info("Software Upload success!")
            error =self.robot.SoftwareUpgrade()
            if 0!=error:
                return error
            if block:
                upgradeState = -1
                time.sleep(0.5)
                upgradeState = self.GetSoftwareUpgradeState()
                if upgradeState == 0:
                    self.log_error("software upgrade not start")
                    return -1
                while (upgradeState > 0 and upgradeState < 100):
                    time.sleep(0.5)
                    upgradeState = self.GetSoftwareUpgradeState()
                    # print("upgradeState",upgradeState,"%")
                if upgradeState == 100:
                    error = 0
                else:
                    error = upgradeState
            return error
        else:
            self.log_error("execute SoftwareUpgrade fail.")
            return error


    """   
    @brief  获取机器人软件升级状态
    @return 错误码 成功- 0, 失败-错误码   
    @return 返回值（调用成功返回） state 机器人软件包升级状态 0：空闲中或上传升级包中，1~100：升级完成百分比，-1：升级软件失败，-2：校验失败，-3：版本校验失败，-4：解压失败，-5：用户配置升级失败，-6：外设配置升级失败，-7：扩展轴配置升级失败，-8：机器人配置升级失败，-9：DH参数配置升级失败
    """
    @log_call
    @xmlrpc_timeout
    def GetSoftwareUpgradeState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.robot_state_pkg.softwareUpgradeState
        return error

    """   
    @brief  设置485扩展轴运动加减速度
    @param  [in]必选参数  acc 485扩展轴运动加速度
    @param  [in]必选参数 dec 485扩展轴运动减速度
    @return 错误码 成功- 0, 失败-错误码    
    """
    @log_call
    @xmlrpc_timeout
    def AuxServoSetAcc(self,acc,dec):
        while self.reconnect_flag:
            time.sleep(0.1)
        acc = float(acc)
        dec = float(dec)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetAcc(acc,dec)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置485扩展轴急停加减速度
    @param  [in]必选参数  acc 485扩展轴急停加速度
    @param  [in]必选参数 dec 485扩展轴急停减速度
    @return 错误码 成功- 0, 失败-错误码    
    """
    @log_call
    @xmlrpc_timeout
    def AuxServoSetEmergencyStopAcc(self,acc,dec):
        while self.reconnect_flag:
            time.sleep(0.1)
        acc = float(acc)
        dec = float(dec)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoSetEmergencyStopAcc(acc,dec)
                flag = False
            except socket.error as e:
                flag = True

        return error


    """   
    @brief  获取485扩展轴急停加减速度
    @return 错误码 成功- 0, 失败-错误码        
    @return 返回值（调用成功返回） acc 485扩展轴急停加速度   
    @return 返回值（调用成功返回） dec 485扩展轴急停减速度
    """
    @log_call
    @xmlrpc_timeout
    def AuxServoGetEmergencyStopAcc(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoGetEmergencyStopAcc()
                flag = False
            except socket.error as e:
                flag = True

        if error[0]==0:
            return error[0],error[1],error[2]
        else:
            return error

    """   
    @brief  获取485扩展轴运动加减速度
    @return 错误码 成功- 0, 失败-错误码        
    @return 返回值（调用成功返回） acc 485扩展轴运动加速度 
    @return 返回值（调用成功返回） dec 485扩展轴运动减速度
    """

    @log_call
    @xmlrpc_timeout
    def AuxServoGetAcc(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.AuxServoGetAcc()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1], error[2]
        else:
            return error

    """   
    @brief  获取末端通讯参数
    @return 错误码 成功- 0, 失败-错误码        
    @return 返回值（调用成功返回） baudRate 波特率：支持 1-9600，2-14400，3-19200，4-38400，5-56000，6-67600，7-115200，8-128000；
    @return 返回值（调用成功返回） dataBit 数据位：数据位支持（8,9），目前常用为 8
    @return 返回值（调用成功返回） stopBit 停止位：1-1，2-0.5，3-2，4-1.5，目前常用为 1
    @return 返回值（调用成功返回） verify 校验位：0-None，1-Odd，2-Even,目前常用为 0；
    @return 返回值（调用成功返回） timeout 超时时间：1~1000ms，此值需要结合外设搭配设置合理的时间参数
    @return 返回值（调用成功返回） timeoutTimes  超时次数：1~10，主要进行超时重发，减少偶发异常提高用户体验
    @return 返回值（调用成功返回） period 周期性指令时间间隔：1~1000ms，主要用于周期性指令每次下发的时间间隔
    """

    @log_call
    @xmlrpc_timeout
    def GetAxleCommunicationParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleCommunicationParam()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1], error[2], error[3], error[4], error[5], error[6], error[7]
        else:
            return error

    """   
    @brief  设置末端通讯参数
    @param  [in]  baudRate 波特率：支持 1-9600，2-14400，3-19200，4-38400，5-56000，6-67600，7-115200，8-128000；
    @param  [in]  dataBit 数据位：数据位支持（8,9），目前常用为 8
    @param  [in]  stopBit 停止位：1-1，2-0.5，3-2，4-1.5，目前常用为 1
    @param  [in]  verify 校验位：0-None，1-Odd，2-Even,目前常用为 0；
    @param  [in]  timeout 超时时间：1~1000ms，此值需要结合外设搭配设置合理的时间参数
    @param  [in]  timeoutTimes  超时次数：1~10，主要进行超时重发，减少偶发异常提高用户体验
    @param  [in]  period 周期性指令时间间隔：1~1000ms，主要用于周期性指令每次下发的时间间隔
    @return 错误码 成功- 0, 失败-错误码        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleCommunicationParam(self,baudRate,dataBit,stopBit,verify,timeout,timeoutTimes,period):
        while self.reconnect_flag:
            time.sleep(0.1)
        baudRate = int (baudRate)
        dataBit = int (dataBit)
        stopBit = int (stopBit)
        verify = int (verify)
        timeout = int (timeout)
        timeoutTimes = int (timeoutTimes)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleCommunicationParam(baudRate,dataBit,stopBit,verify,timeout,timeoutTimes,period)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置末端文件传输类型
    @param  [in] type 1-MCU升级文件；2-LUA文件
    @return 错误码 成功- 0, 失败-错误码        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleFileType(self,type):
        while self.reconnect_flag:
            time.sleep(0.1)
        type=int(type)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleFileType(type)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置启用末端LUA执行
    @param  [in] enable 0-不启用；1-启用
    @return 错误码 成功- 0, 失败-错误码        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleLuaEnable(self,enable):
        while self.reconnect_flag:
            time.sleep(0.1)
        enable=int(enable)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleLuaEnable(enable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  末端LUA文件异常错误恢复
    @param  [in] status 0-不恢复；1-恢复
    @return 错误码 成功- 0, 失败-错误码        
    """
    @log_call
    @xmlrpc_timeout
    def SetRecoverAxleLuaErr(self,enable):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SetRecoverAxleLuaErr(enable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取末端LUA执行使能状态
    @return 错误码 成功- 0, 失败-错误码   
    @return 返回值（调用成功返回） enable 0-不启用；1-启用
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaEnableStatus(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaEnableStatus()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1]
        else:
            return error

    """   
    @brief  设置末端LUA末端设备启用类型
    @param  [in] forceSensorEnable 力传感器启用状态，0-不启用；1-启用
    @param  [in] gripperEnable 夹爪启用状态，0-不启用；1-启用
    @param  [in] IOEnable IO设备启用状态，0-不启用；1-启用
    @return 错误码 成功- 0, 失败-错误码        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleLuaEnableDeviceType(self,forceSensorEnable,gripperEnable,IOEnable):
        while self.reconnect_flag:
            time.sleep(0.1)
        forceSensorEnable = int(forceSensorEnable)
        gripperEnable = int(gripperEnable)
        IOEnable = int(IOEnable)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleLuaEnableDeviceType(forceSensorEnable,gripperEnable,IOEnable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  获取末端LUA末端设备启用类型
    @return 错误码 成功- 0, 失败-错误码   
    @return 返回值（调用成功返回） forceSensorEnable 力传感器启用状态，0-不启用；1-启用
    @return 返回值（调用成功返回） gripperEnable 夹爪启用状态，0-不启用；1-启用
    @return 返回值（调用成功返回） IOEnable IO设备启用状态，0-不启用；1-启用
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaEnableDeviceType(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaEnableDeviceType()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            return error[0], error[1], error[2], error[3]
        else:
            return error

    """   
    @brief  获取当前配置的末端设备
    @return 错误码 成功- 0, 失败-错误码   
    @return 返回值（调用成功返回） forceSensorEnable[8] 力传感器启用状态，0-不启用；1-启用
    @return 返回值（调用成功返回） gripperEnable[8] 夹爪启用状态，0-不启用；1-启用
    @return 返回值（调用成功返回） IOEnable[8]  IO设备启用状态，0-不启用；1-启用
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaEnableDevice(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaEnableDevice()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            par= error[1].split(',')
            if 24 != len(par):
                self.log_error("GetAxleLuaEnableDevice fail")
                return -1,None,None,None
            else:
                print(par)
                return (error[0], [par[0],par[1], par[2], par[3], par[4], par[5], par[6], par[7]],
                        [par[8], par[9], par[10], par[11], par[12], par[13], par[14], par[15]],
                        [par[16], par[17], par[18], par[19], par[20], par[21], par[22], par[23]])
        else:
            return error,None,None,None

    """   
    @brief  设置启用夹爪动作控制功能
    @param  [in] id 夹爪设备编号
    @param  [in] func 0-夹爪使能；1-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩,12-15预留
    @return 错误码 成功- 0, 失败-错误码        
    """
    @log_call
    @xmlrpc_timeout
    def SetAxleLuaGripperFunc(self,id,func):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        func = list(map(int, func))
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleLuaGripperFunc(id,func)
                flag = False
            except socket.error as e:
                flag = True

        # error = self.robot.SetAxleLuaGripperFunc(id,func)
        return error

    """   
    @brief  获取启用夹爪动作控制功能
    @param  [in] id 夹爪设备编号
    @return 错误码 成功- 0, 失败-错误码   
    @return 返回值（调用成功返回） func 0-夹爪使能；1-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleLuaGripperFunc(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id=int(id)
        flag = True
        while flag:
            try:
                error = self.robot.GetAxleLuaGripperFunc(id)
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            par = error[1].split(',')
            print(len(par))
            if 16 != len(par):
                self.log_error("GetAxleLuaEnableDevice fail")
                return -1
            else:
                return (error[0], [par[0],par[1],par[2], par[3], par[4], par[5], par[6], par[7], par[8],
                        par[9], par[10], par[11], par[12], par[13], par[14], par[15]])
        else:
            return error

    """   
    @brief  设置控制器外设协议LUA文件名
    @param  [in]  id 协议编号
    @param  [in] name lua文件名称 “CTRL_LUA_test.lua”
    @return 错误码 成功- 0, 失败-错误码        
    """
    @log_call
    @xmlrpc_timeout
    def SetCtrlOpenLUAName(self,id,name):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        name = str(name)
        flag = True
        while flag:
            try:
                error = self.robot.SetCtrlOpenLUAName(id,name)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取控制箱开放协议Lua文件名称列表
    @param [out] name 开放协议Lua文件名称数组
    @return 错误码
    """
    @log_call
    @xmlrpc_timeout
    def GetCtrlOpenLUAName(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetCtrlOpenLUAName()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            paramStr = str(_error[1])
            # Split the string by comma to get file names
            name = paramStr.split(',')
            return error, name
        else:
            return error, None

    """   
    @brief  加载控制器LUA协议
    @param  [in] id 控制器LUA协议编号
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def LoadCtrlOpenLUA(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                error = self.robot.LoadCtrlOpenLUA(id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief   卸载控制器LUA协议
    @param  [in] id 控制器LUA协议编号
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def UnloadCtrlOpenLUA(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                error = self.robot.UnloadCtrlOpenLUA(id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置控制器LUA协议错误码
    @param  [in] id 控制器LUA协议编号
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def SetCtrlOpenLuaErrCode(self,id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                error = self.robot.SetCtrlOpenLuaErrCode(id)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  机器人Ethercat从站文件写入
    @param  [in] type 从站文件类型，1-升级从站文件；2-升级从站配置文件
    @param  [in] slaveID 从站号
    @param  [in] fileName 上传文件名
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def SlaveFileWrite(self,type,slaveID,fileName):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        slaveID = int(slaveID)
        fileName =str(fileName)
        flag = True
        while flag:
            try:
                error = self.robot.SlaveFileWrite(type,slaveID,fileName)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  机器人Ethercat从站进入boot模式
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def SetSysServoBootMode(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SetSysServoBootMode()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  上传末端Lua开放协议文件
    @param  [in] filePath 本地lua文件路径名 ".../AXLE_LUA_End_DaHuan.lua"
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def AxleLuaUpload(self,filePath):

        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.__FileUpLoad(10,filePath)
        file_name = "/tmp/" + os.path.basename(filePath)
        # file_name = os.path.basename(filePath)
        if 0!= error :
            return error
        else:
            rtn = self.SetAxleFileType(2)
            if(rtn!=0):
                return -1
            rtn = self.SetSysServoBootMode()
            if(rtn!=0):
                return -1
            rtn = self.SlaveFileWrite(1,7,file_name)
            if(rtn!=0):
                return -1
            return rtn


    """   
    ***************************************************************************新增********************************************************************************************
    """

    """   
    @brief  可移动装置使能
    @param  [in] enable 使能状态，0-去使能， 1-使能
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def TractorEnable(self, enable):
        while self.reconnect_flag:
            time.sleep(0.1)
        enable = int(enable)
        flag = True
        while flag:
            try:
                error = self.robot.TractorEnable(enable)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  可移动装置回零
    @return 错误码 成功- 0, 失败-错误码     
    """

    @log_call
    @xmlrpc_timeout
    def TractorHoming(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.TractorHoming()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 可移动装置直线运动
    @param [in] distance 直线运动距离（mm）
    @param [in] vel 直线运动速度百分比（0-100）
    @return 错误码 成功- 0, 失败-错误码  
    """

    @log_call
    @xmlrpc_timeout
    def TractorMoveL(self,distance,vel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        distance = float(distance)
        vel = float(vel)
        flag = True
        while flag:
            try:
                error = self.robot.TractorMoveL(distance,vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 可移动装置圆弧运动
    @param [in] radio 圆弧运动半径（mm）
    @param [in] angle 圆弧运动角度（°）
    @param [in] vel 圆弧运动速度百分比（0-100）
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def TractorMoveC(self,radio, angle, vel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        radio = float(radio)
        angle = float(angle)
        vel = float(vel)
        flag = True
        while flag:
            try:
                error = self.robot.TractorMoveC(radio, angle, vel)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 可移动装置停止运动
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def TractorStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ProgramStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 设置焊丝寻位扩展IO端口
    @param [in] searchDoneDINum 焊丝寻位成功DO端口(0-127)
    @param [in] searchStartDONum 焊丝寻位启停控制DO端口(0-127)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def SetWireSearchExtDIONum(self,searchDoneDINum,searchStartDONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        searchDoneDINum = int(searchDoneDINum)
        searchStartDONum = int(searchStartDONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWireSearchExtDIONum(searchDoneDINum,searchStartDONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 设置焊机控制模式扩展DO端口
    @param [in] DONum 焊机控制模式DO端口(0-127)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def SetWeldMachineCtrlModeExtDoNum(self, DONum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DONum = int(DONum)
        flag = True
        while flag:
            try:
                error = self.robot.SetWeldMachineCtrlModeExtDoNum(DONum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 设置焊机控制模式
    @param [in] mode 焊机控制模式;焊机控制模式;0-直流一元模式；1-脉冲一元模式；2-JOB模式；3-近控模式；4-分别模式；5-CC/CV模式；6-TIG；7-CMT
    @param [in] ioType 控制类型；0-控制箱IO；1-数字通信协议(UDP);2-数字通信协议(ModbusTCP)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def SetWeldMachineCtrlMode(self, mode, ioType=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        ioType = int(ioType)
        param = [ioType , mode]
        flag = True
        while flag:
            try:
                error = self.robot.SetWeldMachineCtrlMode(param)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 关闭RPC
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def CloseRPC(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        # 设置停止事件以通知线程停止
        self.stop_event.set()

        # 关闭CNDE连接（关键：必须先关闭CNDE再关闭RPC）
        if hasattr(self, '_cnde_client') and self._cnde_client is not None:
            print("关闭CNDE连接...")
            self._cnde_client.close()
            self._cnde_client = None

        # 如果线程仍在运行，则等待其结束
        # if self.thread.is_alive():
        #     self.thread.join()

        # 清理 XML-RPC 代理
        if self.robot is not None:
            self.robot = None  # 将代理设置为 None，释放资源
            if self.sock_cli_state is not None:
                self.sock_cli_state.close()
                self.sock_cli_state = None
            self.robot_state_pkg = None
            self.closeRPC_state = True
            # self.robot_realstate_exit = False

        # 如果线程仍在运行，则等待其结束
        if self.thread.is_alive():
            self.thread.join()


        print("RPC connection closed.")
        return

    """   
    @brief 记录示教点
    @param [in] name 示教点名称
    @param [in] update_allprogramfile 是否覆盖 0-不覆盖 1-覆盖
    @return 错误码 成功- 0, 失败-错误码
    """

    # @log_call
    # @xmlrpc_timeout
    #
    # def SavePoint(self,name,update_allprogramfile=0):
    #     name = str(name)
    #     update_allprogramfile = int(update_allprogramfile)
    #     error = self.robot.save_point(name,update_allprogramfile)
    #     return error

    """   
    @brief 开始奇异位姿保护
    @param [in] protectMode 奇异保护模式，0：关节模式；1-笛卡尔模式
    @param [in] minShoulderPos 肩奇异调整范围(mm), 默认100.0
    @param [in] minElbowPos 肘奇异调整范围(mm), 默认50.0
    @param [in] minWristPos 腕奇异调整范围(°), 默认10.0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def SingularAvoidStart(self, protectMode, minShoulderPos=100,minElbowPos=50,minWristPos=10):
        while self.reconnect_flag:
            time.sleep(0.1)
        protectMode = int(protectMode)
        minShoulderPos = float(minShoulderPos)
        minElbowPos = float(minElbowPos)
        minWristPos = float(minWristPos)
        flag = True
        while flag:
            try:
                error = self.robot.SingularAvoidStart(protectMode, minShoulderPos,minElbowPos,minWristPos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief 停止奇异位姿保护
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def SingularAvoidEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.SingularAvoidEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
        @brief 获取旋转夹爪的旋转圈数
        @return 错误码 成功- 0, 失败-错误码
        @return 返回值（调用成功返回） fault 0-无错误，1-有错误
        @return 返回值（调用成功返回） num 旋转圈数
    """

    @log_call
    @xmlrpc_timeout

    def GetGripperRotNum(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0,self.robot_state_pkg.gripper_fault,self.robot_state_pkg.gripperRotNum

    """   
        @brief 获取旋转夹爪的旋转速度百分比
        @return 错误码 成功- 0, 失败-错误码
        @return 返回值（调用成功返回） fault 0-无错误，1-有错误
        @return 返回值（调用成功返回） speed 旋转速度百分比
    """

    @log_call
    @xmlrpc_timeout

    def GetGripperRotSpeed(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripperRotSpeed

    """   
        @brief 获取旋转夹爪的旋转力矩百分比
        @return 错误码 成功- 0, 失败-错误码
        @return 返回值（调用成功返回） fault 0-无错误，1-有错误
        @return 返回值（调用成功返回） torque 旋转力矩百分比
    """

    @log_call
    @xmlrpc_timeout

    def GetGripperRotTorque(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripperRotTorque

    """   
       @brief 开始Ptp运动FIR滤波
       @param  [in] maxAcc 最大加速度极值(deg/s2)
       @param  [in] maxJek 统一关节急动度极值(deg/s3)
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def PtpFIRPlanningStart(self, maxAcc,maxJek):
        while self.reconnect_flag:
            time.sleep(0.1)
        maxAcc = float(maxAcc)
        maxJek = float(maxJek)
        flag = True
        while flag:
            try:
                error = self.robot.PtpFIRPlanningStart(maxAcc,maxJek)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
       @brief 关闭Ptp运动FIR滤波
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def PtpFIRPlanningEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.PtpFIRPlanningEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
       @brief 上传轨迹J文件
       @param  [in] filePath 上传轨迹文件的全路径名   C://test/testJ.txt
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def TrajectoryJUpLoad(self,filePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.__FileUpLoad(20, filePath)
        return error

    """2024.12.16"""
    """   
       @brief 删除轨迹J文件
       @param  [in] filePath 删除轨迹文件的全路径名   C://test/testJ.txt
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def TrajectoryJDelete(self, fileName):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.__FileDelete(20, fileName)
        return error

    """2024.12.18"""
    """   
       @brief 开始LIN、ARC运动FIR滤波
       @param  [in] maxAccLin 线加速度极值(mm/s2)
       @param  [in] maxAccDeg 角加速度极值(deg/s2)
       @param  [in] maxJerkLin 线加加速度极值(mm/s3)
       @param  [in] maxJerkDeg 角加加速度极值(deg/s3)
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LinArcFIRPlanningStart(self, maxAccLin, maxAccDeg, maxJerkLin, maxJerkDeg):
        while self.reconnect_flag:
            time.sleep(0.1)
        maxAccLin = float(maxAccLin)
        maxAccDeg = float(maxAccDeg)
        maxJerkLin = float(maxJerkLin)
        maxJerkDeg = float(maxJerkDeg)
        flag = True
        while flag:
            try:
                error = self.robot.LinArcFIRPlanningStart(maxAccLin, maxAccDeg, maxJerkLin, maxJerkDeg)
                flag = False
            except socket.error as e:
                flag = True

        return  error

    """   
       @brief 关闭LIN、ARC运动FIR滤波
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LinArcFIRPlanningEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LinArcFIRPlanningEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.08"""
    """   
       @brief 工具坐标系转换开始
       @param  [in] toolNum 工具坐标系编号[0-14]
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def ToolTrsfStart(self, toolNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        toolNum = int(toolNum)
        flag = True
        while flag:
            try:
                error = self.robot.ToolTrsfStart(toolNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
       @brief 工具坐标系转换结束
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def ToolTrsfEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ToolTrsfEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.08"""
    """3.7.8"""
    """
       @brief 根据点位信息计算工具坐标系
       @param  [in] method 计算方法；0-四点法；1-六点法
       @param  [in] pos 关节位置组，四点法时数组长度为4个，六点法时数组长度为6个
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回） tcp_offset=[x,y,z,rx,ry,rz]: 根据点位信息计算得到的工具坐标系，单位 [mm][°]
    """

    @log_call
    @xmlrpc_timeout

    def ComputeToolCoordWithPoints(self, method, pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        param = {}
        param[0] = pos[0]
        param[1] = pos[1]
        param[2] = pos[2]
        param[3] = pos[3]

        if method == 0:  # 四点法
            param[4] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            param[5] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:  # 六点法
            param[4] = pos[4]
            param[5] = pos[5]
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeToolCoordWithPoints(method, param[0], param[1], param[2], param[3], param[4], param[5])
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error,None

    """
       @brief 根据点位信息计算工件坐标系
       @param  [in] method 计算方法；0：原点-x轴-z轴  1：原点-x轴-xy平面
       @param  [in] pos 三个TCP位置组
       @param  [in] refFrame 参考坐标系
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回） wobj_offset=[x,y,z,rx,ry,rz]: 根据点位信息计算得到的工件坐标系，单位 [mm][°]
    """

    @log_call
    @xmlrpc_timeout

    def ComputeWObjCoordWithPoints(self, method, pos, refFrame):
        while self.reconnect_flag:
            time.sleep(0.1)
        method = int(method)
        param = {}
        param[0] = pos[0]
        param[1] = pos[1]
        param[2] = pos[2]
        refFrame = int(refFrame)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeWObjCoordWithPoints(method, param[0], param[1], param[2], refFrame)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error,None

    """
       @brief 设置机器人焊接电弧意外中断检测参数
       @param  [in] checkEnable 是否使能检测；0-不使能；1-使能
       @param  [in] arcInterruptTimeLength 电弧中断确认时长(ms)
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def WeldingSetCheckArcInterruptionParam(self, checkEnable, arcInterruptTimeLength):
        while self.reconnect_flag:
            time.sleep(0.1)
        checkEnable = int(checkEnable)
        arcInterruptTimeLength = int(arcInterruptTimeLength)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCheckArcInterruptionParam(checkEnable, arcInterruptTimeLength)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 获取机器人焊接电弧意外中断检测参数
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回） checkEnable 是否使能检测；0-不使能；1-使能
       @return 返回值（调用成功返回） arcInterruptTimeLength 电弧中断确认时长(ms)
    """

    @log_call
    @xmlrpc_timeout

    def WeldingGetCheckArcInterruptionParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.WeldingGetCheckArcInterruptionParam()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        return error,None,None

    """
       @brief 设置机器人焊接中断恢复参数
       @param  [in] enable 是否使能焊接中断恢复
       @param  [in] length 焊缝重叠距离(mm)
       @param  [in] velocity 机器人回到再起弧点速度百分比(0-100)
       @param  [in] moveType 机器人运动到再起弧点方式；0-LIN；1-PTP
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def WeldingSetReWeldAfterBreakOffParam(self, enable, length, velocity, moveType):
        while self.reconnect_flag:
            time.sleep(0.1)
        enable = int(enable)
        length = float(length)
        velocity = float(velocity)
        moveType = int(moveType)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetReWeldAfterBreakOffParam(enable, length, velocity, moveType)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 获取机器人焊接中断恢复参数
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回） enable 是否使能焊接中断恢复
       @return 返回值（调用成功返回） length 焊缝重叠距离(mm)
       @return 返回值（调用成功返回） velocity 机器人回到再起弧点速度百分比(0-100)
       @return 返回值（调用成功返回） moveType 机器人运动到再起弧点方式；0-LIN；1-PTP
    """

    @log_call
    @xmlrpc_timeout

    def WeldingGetReWeldAfterBreakOffParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.WeldingGetReWeldAfterBreakOffParam()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3], _error[4]
        return error,None,None,None,None

    """
       @brief 设置机器人焊接中断后恢复焊接
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def WeldingStartReWeldAfterBreakOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingStartReWeldAfterBreakOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 设置机器人焊接中断后退出焊接
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def WeldingAbortWeldAfterBreakOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingAbortWeldAfterBreakOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.09"""
    """
       @brief 
       @param  [in] status
       @param  [in] delayMode
       @param  [in] delayTime
       @param  [in] delayDisExAxisNum
       @param  [in] delayDis
       @param  [in] sensitivePara
       @param  [in] speed
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LaserSensorRecord(self, status, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        delayMode = int(delayMode)
        delayTime = int(delayTime)
        delayDisExAxisNum = int(delayDisExAxisNum)
        delayDis = float(delayDis)
        sensitivePara = float(sensitivePara)
        speed = float(speed)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorRecord(status, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @param  [in] weldId
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingLaserOn(self, weldId):
        while self.reconnect_flag:
            time.sleep(0.1)
        weldId = int(weldId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingLaserOn(weldId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingLaserOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingLaserOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @param  [in] coordId
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingTrackOn(self, coordId):
        while self.reconnect_flag:
            time.sleep(0.1)
        coordId = int(coordId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingTrackOn(coordId)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingTrackOff(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingTrackOff()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 
       @param  [in] direction
       @param  [in] directionPoint
       @param  [in] vel
       @param  [in] distance
       @param  [in] timeout
       @param  [in] posSensorNum
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingSearchStart(self, direction, directionPoint, vel, distance, timeout, posSensorNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        direction = int(direction)
        directionPoint = list(map(float, directionPoint))
        vel = int(vel)
        distance = int(distance)
        timeout = int(timeout)
        posSensorNum = int(posSensorNum)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStart(direction, directionPoint[0], directionPoint[1], directionPoint[2], vel, distance, timeout, posSensorNum)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 激光寻位结束
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LaserTrackingSearchStop(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStop()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.01.24"""
    """3.7.9"""
    """
       @brief 摆动渐变开始
       @param  [in] weaveChangeFlag 摆动编号 1-变摆动参数；2-变摆动参数+焊接速度
       @param  [in] weaveNum 摆动编号
       @param  [in] velStart 焊接开始速度，(cm/min)
       @param  [in] velEnd 焊接结束速度，(cm/min)
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def WeaveChangeStart(self, weaveChangeFlag, weaveNum, velStart, velEnd):
        while self.reconnect_flag:
            time.sleep(0.1)
        weaveChangeFlag = int(weaveChangeFlag)
        weaveNum = int(weaveNum)
        velStart = float(velStart)
        velEnd = float(velEnd)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveChangeStart(weaveChangeFlag, weaveNum, velStart, velEnd)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 摆动渐变结束
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def WeaveChangeEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.WeaveChangeEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.02.20"""
    """3.8.0"""
    """
       @brief  轨迹预处理(轨迹前瞻)
       @param  [in] name  轨迹文件名
       @param  [in] mode 采样模式，0-不进行采样；1-等数据间隔采样；2-等误差限制采样
       @param  [in] errorLim 误差限制，使用直线拟合生效
       @param  [in] type 平滑方式，0-贝塞尔平滑
       @param  [in] precision 平滑精度，使用贝塞尔平滑时生效
       @param  [in] vamx 设定的最大速度，mm/s
       @param  [in] amax 设定的最大加速度，mm/s2
       @param  [in] jmax 设定的最大加加速度，mm/s3
       @param  [in] flag 匀速前瞻开启开关 0-不开启；1-开启
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LoadTrajectoryLA(self, name, mode, errorLim, type, precision, vamx, amax, jmax, flag):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        name =str(name)
        mode = int(mode)
        errorLim = float(errorLim)
        type = int(type)
        precision = float(precision)
        vamx = float(vamx)
        amax = float(amax)
        jmax = float(jmax)
        flag = int(flag)

        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.LoadTrajectoryLA(name, mode, errorLim, type, precision, vamx, amax, jmax, flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """
       @brief 轨迹复现(轨迹前瞻)
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def MoveTrajectoryLA(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.MoveTrajectoryLA()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.02.25"""
    """
      @brief  自定义碰撞检测阈值功能开始，设置关节端和TCP端的碰撞检测阈值
      @param  [in] flag 1-仅关节检测开启；2-仅TCP检测开启；3-关节和TCP检测同时开启
      @param  [in] jointDetectionThreshould 关节碰撞检测阈值 j1-j6
      @param  [in] tcpDetectionThreshould TCP碰撞检测阈值，xyzabc
      @param  [in] block 0-非阻塞；1-阻塞
      @return  错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def CustomCollisionDetectionStart(self, flag, jointDetectionThreshould, tcpDetectionThreshould, block):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = int(flag)
        jointDetectionThreshould = list(map(float, jointDetectionThreshould))
        tcpDetectionThreshould = list(map(float, tcpDetectionThreshould))
        block = int(block)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.CustomCollisionDetectionStart(flag, jointDetectionThreshould, tcpDetectionThreshould, block)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True

        return error

    """
       @brief 自定义碰撞检测阈值功能关闭
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def CustomCollisionDetectionEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.CustomCollisionDetectionEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.03.19"""
    """3.8.1"""
    """
       @brief 获取机器人状态
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回）robot_state_pkg 机器人状态结构体
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotRealTimeState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0,self.robot_state_pkg

    """   
    @brief  停止运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StopMove(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.send_message("/f/bIII0III102III4IIIStopIII/b/f")
        return error

    """2025.03.28"""
    """
       @brief 加速度平滑开启
       @param  [in] saveFlag 是否断电保存
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AccSmoothStart(self, saveFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        saveFlag = bool(saveFlag)
        saveFlag_flag = 1 if saveFlag else 0
        saveFlag_flag = int(saveFlag_flag)
        flag = True
        while flag:
            try:
                error = self.robot.AccSmoothStart(saveFlag_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 加速度平滑关闭
       @param  [in] saveFlag 是否断电保存
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AccSmoothEnd(self, saveFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        saveFlag = bool(saveFlag)
        saveFlag_flag = 1 if saveFlag else 0
        saveFlag_flag = int(saveFlag_flag)

        flag = True
        while flag:
            try:
                error = self.robot.AccSmoothEnd(saveFlag_flag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """2025.04.03"""
    """
       @brief 控制器日志下载
       @param  [in] savePath 保存文件路径"D://zDown/"
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def RbLogDownload(self, savePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:
            error = self.robot.RbLogDownloadPrepare()
            if error == 0:
                savePath = str(savePath)
                fileName = "rblog.tar.gz"
                try:
                    error = self.__FileDownLoad(1, fileName, savePath)
                    return error
                except socket.error as e:
                    return RobotError.ERR_DOWN_LOAD_FILE_FAILED
            else:
                return error
        except socket.error as e:
            return RobotError.ERR_RPC_ERROR

    """
       @brief 所有数据源下载
       @param  [in] savePath 保存文件路径"D://zDown/"
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def AllDataSourceDownload(self, savePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:
            error = self.robot.AllDataSourceDownloadPrepare()
            if error == 0:
                savePath = str(savePath)
                fileName = "alldatasource.tar.gz"
                try:
                    error = self.__FileDownLoad(2, fileName, savePath)
                    return error
                except socket.error as e:
                    return RobotError.ERR_DOWN_LOAD_FILE_FAILED
            else:
                return error
        except socket.error as e:
            return RobotError.ERR_RPC_ERROR

    """
       @brief 数据备份包下载
       @param  [in] savePath 保存文件路径"D://zDown/"
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def DataPackageDownload(self, savePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        try:
            error = self.robot.DataPackageDownloadPrepare()
            if error == 0:
                savePath = str(savePath)
                fileName = "fr_user_data.tar.gz"
                try:
                    error = self.__FileDownLoad(3, fileName, savePath)
                    return error
                except socket.error as e:
                    return RobotError.ERR_DOWN_LOAD_FILE_FAILED
            else:
                return error
        except socket.error as e:
            return RobotError.ERR_RPC_ERROR

    """
       @brief 获取控制箱SN码
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回） SNCode 控制箱SN码
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotSN(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotSN()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1]
        return error,None

    """
       @brief 关闭机器人操作系统
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ShutDownRobotOS(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.ShutDownRobotOS()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 传送带通讯输入检测
       @param  [in] timeout 等待超时时间ms
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorComDetect(self, timeout):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        timeout = int(timeout)
        flag = True
        while flag:
            try:
                # error = self.robot.ConveryComDetect(timeout)
                error = self.robot.ConveyorComDetect(timeout)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
       @brief 传送带通讯输入检测触发
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ConveyorComDetectTrigger(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        error = self.send_message("/f/bIII0III1149III25IIIConveyorComDetectTrigger()III/b/f")
        return error
        # while self.reconnect_flag:
        #     time.sleep(0.1)
        # if self.GetSafetyCode() != 0:
        #     return self.GetSafetyCode()
        #
        # flag = True
        # while flag:
        #     try:
        #         error = self.robot.ConveryComDetectTrigger()
        #         # error = self.robot.ConveyorComDetectTrigger()
        #         flag = False
        #     except socket.error as e:
        #         flag = True
        #
        # return error

    """2025.04.14"""
    """3.8.2"""
    """
       @brief 电弧跟踪焊机电流反馈AI通道选择
       @param  [in] channel 通道；0-扩展AI0；1-扩展AI1；2-扩展AI2；3-扩展AI3；4-控制箱AI0；5-控制箱AI1
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceAIChannelCurrent(self, channel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        channel = int(channel)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceAIChannelCurrent(channel)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief 电弧跟踪焊机电压反馈AI通道选择
       @param  [in] channel 通道；0-扩展AI0；1-扩展AI1；2-扩展AI2；3-扩展AI3；4-控制箱AI0；5-控制箱AI1
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceAIChannelVoltage(self, channel):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        channel = int(channel)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceAIChannelVoltage(channel)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief 电弧跟踪焊机电流反馈转换参数
       @param  [in] AILow AI通道下限，默认值0V，范围[0-10V]
       @param  [in] AIHigh AI通道上限，默认值10V，范围[0-10V]
       @param  [in] currentLow AI通道下限对应焊机电流值，默认值0V，范围[0-200V]
       @param  [in] currentHigh AI通道上限对应焊机电流值，默认值100V，范围[0-200V]
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceCurrentPara(self, AILow=0, AIHigh=10, currentLow=0, currentHigh=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        AILow = float(AILow)
        AIHigh = float(AIHigh)
        currentLow = float(currentLow)
        currentHigh = float(currentHigh)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceCurrentPara(AILow, AIHigh, currentLow, currentHigh)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief 电弧跟踪焊机电压反馈转换参数
       @param  [in] AILow AI通道下限，默认值0V，范围[0-10V]
       @param  [in] AIHigh AI通道上限，默认值10V，范围[0-10V]
       @param  [in] voltageLow AI通道下限对应焊机电压值，默认值0V，范围[0-200V]
       @param  [in] voltageHigh AI通道上限对应焊机电压值，默认值100V，范围[0-200V]
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ArcWeldTraceVoltagePara(self, AILow=0, AIHigh=10, voltageLow=0, voltageHigh=100):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        AILow = float(AILow)
        AIHigh = float(AIHigh)
        voltageLow = float(voltageLow)
        voltageHigh = float(voltageHigh)
        flag = True
        while flag:
            try:
                error = self.robot.ArcWeldTraceVoltagePara(AILow, AIHigh, voltageLow, voltageHigh)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.04.16"""
    """
       @brief 设置焊接电压渐变开始
       @param  [in] IOType 控制类型；0-控制箱IO；1-数字通信协议(UDP);2-数字通信协议(ModbusTCP)
       @param  [in] voltageStart 起始焊接电压(V)
       @param  [in] voltageEnd 终止焊接电压(V)
       @param  [in] AOIndex 控制箱AO端口号(0-1)
       @param  [in] blend 是否平滑 0-不平滑；1-平滑
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltageGradualChangeStart(self, IOType, voltageStart, voltageEnd, AOIndex, blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        IOType = int(IOType)
        voltageStart = float(voltageStart)
        voltageEnd = float(voltageEnd)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltageGradualChangeStart(IOType, voltageStart, voltageEnd, AOIndex, blend)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief 设置焊接电压渐变结束
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetVoltageGradualChangeEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetVoltageGradualChangeEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief 设置焊接电流渐变开始
       @param  [in] IOType 控制类型；0-控制箱IO；1-数字通信协议(UDP);2-数字通信协议(ModbusTCP)
       @param  [in] currentStart 起始焊接电流(A)
       @param  [in] currentEnd 终止焊接电流(A)
       @param  [in] AOIndex 控制箱AO端口号(0-1)
       @param  [in] blend 是否平滑 0-不平滑；1-平滑
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrentGradualChangeStart(self, IOType, currentStart, currentEnd, AOIndex, blend):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        IOType = int(IOType)
        currentStart = float(currentStart)
        currentEnd = float(currentEnd)
        AOIndex = int(AOIndex)
        blend = int(blend)
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrentGradualChangeStart(IOType, currentStart, currentEnd, AOIndex, blend)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
       @brief 设置焊接电流渐变结束
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WeldingSetCurrentGradualChangeEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                error = self.robot.WeldingSetCurrentGradualChangeEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.04.27"""
    """
       @brief 获取SmartTool按钮状态
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回） state SmartTool手柄按钮状态;(bit0:0-通信正常；1-通信掉线；bit1-撤销操作；bit2-清空程序；bit3-A键；bit4-B键；bit5-C键；bit6-D键；bit7-E键；bit8-IO键；bit9-手自动；bit10开始)
    """

    @log_call
    @xmlrpc_timeout
    def GetSmarttoolBtnState(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0,self.robot_state_pkg.smartToolState

    """2025.05.08"""
    """
       @brief 获取扩展轴坐标系
       @return 错误码 成功- 0, 失败-错误码
       @return 返回值（调用成功返回） coord 扩展轴坐标系
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisGetCoord(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        flag = True
        while flag:
            try:
                _error = self.robot.ExtAxisGetCoord()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """2025.06.06"""
    """   
    @brief  获取夹爪激活状态
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） fault 0-无错误，1-有错误
    @return 返回值（调用成功返回） gripper_active bit0~bit15对应夹爪编号0~15，bit=0为未激活，bit=1为激活
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperActivateStatus(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault,self.robot_state_pkg.gripper_active

    """   
    @brief  获取夹爪位置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） fault 0-无错误，1-有错误
    @return 返回值（调用成功返回） position 位置百分比，范围0~100% 
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurPosition(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_position

    """   
    @brief  获取夹爪电流
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） fault 0-无错误，1-有错误
    @return 返回值（调用成功返回） current 电流百分比，范围0~100%
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurCurrent(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_current

    """   
    @brief  获取夹爪电压
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） fault 0-无错误，1-有错误
    @return 返回值（调用成功返回） voltage 电压,单位0.1V
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperVoltage(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_voltage

    """   
    @brief  获取夹爪温度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） fault 0-无错误，1-有错误
    @return 返回值（调用成功返回） temp 温度，单位℃
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperTemp(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_tmp

    """   
    @brief  获取夹爪速度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） fault 0-无错误，1-有错误
    @return 返回值（调用成功返回） speed 速度百分比，范围0~100%
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurSpeed(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_speed

    """2025.06.24"""
    """3.8.3"""
    """
    @brief 设置宽电压控制箱温度及风扇转速监控参数
    @param  [in] enable 0-不使能监测；1-使能监测
    @param  [in] period 监测周期(s),范围1-100
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetWideBoxTempFanMonitorParam(self, enable, period):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        enable = int(enable)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.SetWideBoxTempFanMonitorParam(enable, period)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取宽电压控制箱温度及风扇转速监控参数
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） enable 0-不使能监测；1-使能监测
    @return 返回值（调用成功返回） period 监测周期(s),范围1-100
    """

    @log_call
    @xmlrpc_timeout
    def GetWideBoxTempFanMonitorParam(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetWideBoxTempFanMonitorParam()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2]
        return error, None, None

    """2025.07.04"""
    """3.8.4"""

    """
    @brief 设置焦点标定点
    @param  [in] pointNum 焦点标定点编号 1-8
    @param  [in] point 标定点坐标
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFocusCalibPoint(self, pointNum, point):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        point = list(map(float, point))
        flag = True
        while flag:
            try:
                error = self.robot.SetFocusCalibPoint(pointNum,point[0],point[1],point[2],point[3],point[4],point[5])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 计算焦点标定结果
    @param  [in] pointNum 标定点个数
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） resultPos 标定结果XYZ
    @return 返回值（调用成功返回） accuracy 标定精度误差
    """

    @log_call
    @xmlrpc_timeout
    def ComputeFocusCalib(self, pointNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        pointNum = int(pointNum)
        flag = True
        while flag:
            try:
                _error = self.robot.ComputeFocusCalib(pointNum)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3]], _error[4]
        return error, None, None

    """
    @brief 开启焦点跟随
    @param  [in] kp 比例参数，默认50.0
    @param  [in] kpredict 前馈参数，默认19.0
    @param  [in] aMax 最大角加速度限制，默认1440°/s^2
    @param  [in] vMax 最大角速度限制，默认180°/s
    @param  [in] type 锁定X轴指向(0-参考输入矢量；1-水平；2-垂直)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FocusStart(self, kp=50.0, kpredic=19.0, aMax=1440, vMax=180, type=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        kp = float(kp)
        kpredic = float(kpredic)
        aMax = float(aMax)
        vMax = float(vMax)
        type = int(type)
        flag = True
        while flag:
            try:
                error = self.robot.FocusStart(kp, kpredic, aMax, vMax, type)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 停止焦点跟随
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FocusEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.FocusEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 设置焦点坐标
    @param  [in] pos 焦点坐标XYZ
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetFocusPosition(self, pos):
        while self.reconnect_flag:
            time.sleep(0.1)
        pos = list(map(float, pos))
        flag = True
        while flag:
            try:
                error = self.robot.SetFocusPosition(pos[0],pos[1],pos[2])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.07.08"""
    """
    @brief 设置编码器升级
    @param  [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetEncoderUpgrade(self, path):
        while self.reconnect_flag:
            time.sleep(0.1)
        path = str(path)
        flag = True
        while flag:
            try:
                error = self.robot.SetEncoderUpgrade(path)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 设置关节固件升级
    @param  [in] type 升级文件类型；1-升级固件；2-升级从站配置文件
    @param  [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetJointFirmwareUpgrade(self, type, path):
        while self.reconnect_flag:
            time.sleep(0.1)

        type = int(type)
        path = str(path)
        errcode = self.__FileUpLoad(2, path)
        if errcode == 0:
            file_name = "/tmp/" + os.path.basename(path)
            for joint_num in range(1, 7):
                errcode = self.SlaveFileWrite(1, joint_num, file_name)
                if errcode != 0:
                    return errcode
        return errcode

    """
    @brief 设置控制箱固件升级
    @param  [in] type 升级文件类型；1-升级固件；2-升级从站配置文件
    @param  [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetCtrlFirmwareUpgrade(self, type, path):
        while self.reconnect_flag:
            time.sleep(0.1)

        type = int(type)
        path = str(path)
        errcode = self.__FileUpLoad(2, path)
        if errcode == 0:
            file_name = "/tmp/" + os.path.basename(path)
            errcode = self.SlaveFileWrite(type, 0, file_name)
            return errcode
        return errcode

    """
    @brief 设置末端固件升级
    @param  [in] type 升级文件类型；1-升级固件；2-升级从站配置文件
    @param  [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetEndFirmwareUpgrade(self, type, path):
        while self.reconnect_flag:
            time.sleep(0.1)

        type = int(type)
        path = str(path)
        errcode = self.__FileUpLoad(2, path)
        if errcode == 0:
            file_name = "/tmp/" + os.path.basename(path)
            errcode = self.SlaveFileWrite(type, 7, file_name)
            return errcode
        return errcode

    """
    @brief 关节全参数配置文件升级
    @param  [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def JointAllParamUpgrade(self, path):
        while self.reconnect_flag:
            time.sleep(0.1)

        path = str(path)
        errcode = self.__FileUpLoad(5, path)
        if errcode == 0:
            error = self.robot.JointAllParamUpgrade()
            return error
        return errcode

    """2025.07.21"""
    """3.8.5"""
    """
    @brief 激光传感器记录点
    @param  [in] coordID 激光传感器坐标系
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） joint 激光传感器识别点关节位置
    @return 返回值（调用成功返回） desc 激光传感器识别点笛卡尔位置
    @return 返回值（调用成功返回） exaxis 激光传感器识别点扩展轴位置
    """

    @log_call
    @xmlrpc_timeout
    def LaserRecordPoint(self, coordID):
        while self.reconnect_flag:
            time.sleep(0.1)
        coordID = int(coordID)
        flag = True
        while flag:
            try:
                _error = self.robot.LaserRecordPoint(coordID,0,100)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            param_str = str(_error[1])
            par_s = param_str.split(',')
            if len(par_s) != 16:
                return -1, None, None, None
            return (error, [float(par_s[0]),float(par_s[1]),float(par_s[2]),float(par_s[3]),float(par_s[4]),float(par_s[5])],
                    [float(par_s[6]),float(par_s[7]),float(par_s[8]),float(par_s[9]),float(par_s[10]),float(par_s[11])],
                    [float(par_s[12]),float(par_s[13]),float(par_s[14]),float(par_s[15])])
        return error, None, None, None

    """2025.08.07"""
    """
    @brief 设置扩展轴与机器人同步运动策略
    @param  [in] strategy 策略；0-以机器人为主；1-扩展轴与机器人同步
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExAxisRobotPlan(self, strategy):
        while self.reconnect_flag:
            time.sleep(0.1)
        strategy = int(strategy)
        flag = True
        while flag:
            try:
                error = self.robot.SetExAxisRobotPlan(strategy)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.08.12"""
    """
    @brief 获取从站板卡参数
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） type  0-Ethercat，1-CClink, 3-Ethercat, 4-EIP
    @return 返回值（调用成功返回） version  协议版本
    @return 返回值（调用成功返回） connState  0-未连接 1-已连接
    """

    @log_call
    @xmlrpc_timeout
    def GetFieldBusConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetFieldBusConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[2], _error[3], _error[4]
        return error, None, None, None

    """
    @brief 写入从站DO
    @param  [in] DOIndex  DO编号
    @param  [in] wirteNum  写入的数量
    @param  [in] status[8] 写入的数值，最多写8个
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWriteDO(self, DOIndex, wirteNum, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        DOIndex = int(DOIndex)
        wirteNum = int(wirteNum)
        status = list(map(int, status))
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWriteDO(DOIndex, wirteNum, status)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 写入从站AO
    @param  [in] AOIndex  AO编号
    @param  [in] wirteNum  写入的数量
    @param  [in] status[8] 写入的数值，最多写8个
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWriteAO(self, AOIndex, wirteNum, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        AOIndex = int(AOIndex)
        wirteNum = int(wirteNum)
        status = list(map(int, status))
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWriteAO(AOIndex, wirteNum, status)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 读取从站DI
    @param  [in] DOIndex  DI编号
    @param  [in] readeNum  读取的数量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） status[8] 读取到的数值，最多读8个
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveReadDI(self, DOIndex, readeNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        DOIndex = int(DOIndex)
        readeNum = int(readeNum)
        flag = True
        while flag:
            try:
                _error = self.robot.FieldBusSlaveReadDI(DOIndex, readeNum)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1:readeNum+1]
        return error, None

    """
    @brief 读取从站AI
    @param  [in] AOIndex  AI编号
    @param  [in] readeNum  读取的数量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） status[8] 读取到的数值，最多读8个
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveReadAI(self, AOIndex, readeNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        AOIndex = int(AOIndex)
        readeNum = int(readeNum)
        flag = True
        while flag:
            try:
                _error = self.robot.FieldBusSlaveReadAI(AOIndex, readeNum)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1:readeNum + 1]
        return error, None

    """
    @brief 等待扩展DI输入
    @param  [in] DIIndex DI编号
    @param  [in] status 0-低电平；1-高电平
    @param  [in] waitMs 最大等待时间(ms)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWaitDI(self, DIIndex, status, waitMs):
        while self.reconnect_flag:
            time.sleep(0.1)
        DIIndex = int(DIIndex)
        status = int(status)
        waitMs = int(waitMs)
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWaitDI(DIIndex, status, waitMs)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 等待扩展AI输入
    @param  [in] AIIndex AI编号
    @param  [in] waitType 0-大于；1-小于
    @param  [in] value AI值
    @param  [in] waitMs 最大等待时间(ms)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FieldBusSlaveWaitAI(self, AIIndex, waitType, value, waitMs):
        while self.reconnect_flag:
            time.sleep(0.1)
        AIIndex = int(AIIndex)
        waitType = int(waitType)
        value = float(value)
        waitMs = int(waitMs)
        flag = True
        while flag:
            try:
                error = self.robot.FieldBusSlaveWaitAI(AIIndex, waitType, value, waitMs)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 控制阵列式吸盘
    @param  [in] slaveID 从站号
    @param  [in] len 长度
    @param  [in] ctrlValue 控制值 1-按最大真空度吸取 2-按设定真空度吸取 3-停止吸取
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetSuckerCtrl(self, slaveID, len, ctrlValue):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        len = int(len)
        ctrlValue = list(map(int, ctrlValue))
        flag = True
        while flag:
            try:
                error = self.robot.SetSuckerCtrl(slaveID, len, ctrlValue)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取阵列式吸盘状态
    @param  [in] slaveID 从站号
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） state 吸附状态 0-释放物体 1-检测到工件吸附成功 2-没有吸附到物体 3-物体脱离
    @return 返回值（调用成功返回） pressValue 当前真空度 单位kpa
    @return 返回值（调用成功返回） error 吸盘当前的错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetSuckerState(self, slaveID):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSuckerState(slaveID)
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            return error, _error[1], _error[2], _error[3]
        return error, None, None, None

    """
    @brief 等待吸盘状态
    @param  [in] slaveID 从站号
    @param  [in] state 吸附状态 0-释放物体 1-检测到工件吸附成功 2-没有吸附到物体 3-物体脱离
    @param  [in] ms 等待最大时间
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def WaitSuckerState(self, slaveID, state, ms):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        state = int(state)
        ms = int(ms)
        flag = True
        while flag:
            try:
                error = self.robot.WaitSuckerState(slaveID, state, ms)
                flag = False
            except socket.error as e:
                flag = True
        return error


    """
    @brief 上传开放协议的Lua文件
    @param  [in] filePath 本地开放协议lua文件路径名
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def OpenLuaUpload(self, filePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        filePath =str(filePath)
        errcode = self.__FileUpLoad(11, filePath)
        if errcode == 0:
            pos = filePath.rfind('/')
            if pos == -1:
                return RobotError.ERR_FILE_NAME
            filename = filePath[pos + 1:]  # 获取文件名部分
            error = self.robot.CtrlOpenLuaUpLoadCheck(filename)
            return error
        return errcode

    """3.8.6"""
    """2025.09.03"""
    """
    @brief 设置拖动开启前负载力检测
    @param  [in] flag 0-关闭；1-开启
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetTorqueDetectionSwitch(self, flag):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.SetTorqueDetectionSwitch(flag)
                flag_tmp = False
            except socket.error as e:
                flag_tmp = True
        return error

    # """2025.09.03"""
    # """
    # @brief 设置拖动开启前负载力检测
    # @param  [in] flag 0-关闭；1-开启
    # @return 错误码 成功- 0, 失败-错误码
    # """
    #
    # @log_call
    # @xmlrpc_timeout
    # def SetTorqueDetectionSwitch(self, flag):
    #     while self.reconnect_flag:
    #         time.sleep(0.1)
    #     flag = int(flag)
    #     flag_tmp = True
    #     while flag_tmp:
    #         try:
    #             error = self.robot.SetTorqueDetectionSwitch(flag)
    #             flag_tmp = False
    #         except socket.error as e:
    #             flag_tmp = True
    #     return error

    """2025.09.05"""
    """
    @brief 激光外设打开关闭函数
    @param  [in] OnOff 0-关闭 1-打开
    @param  [in] weldId 焊缝ID 默认为0
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingLaserOnOff(self, OnOff, weldId=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        OnOff = int(OnOff)
        weldId = int(weldId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingLaserOnOff(OnOff, weldId)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光跟踪开始结束函数
    @param  [in] OnOff 0-结束 1-开始
    @param  [in] coordId 激光外设工具坐标系编号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingTrackOnOff(self, OnOff, coordId):
        while self.reconnect_flag:
            time.sleep(0.1)
        OnOff = int(OnOff)
        coordId = int(coordId)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingTrackOnOff(OnOff, coordId)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光寻位-固定方向
    @param  [in] direction 0-x+ 1-x- 2-y+ 3-y- 4-z+ 5-z-
    @param  [in] vel 速度 单位%
    @param  [in] distance 最大寻位距离 单位mm
    @param  [in] timeout 寻位超时时间 单位ms
    @param  [in] posSensorNum 激光标定的工具坐标编号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSearchStart_xyz(self, direction, vel, distance, timeout, posSensorNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        direction = int(direction)
        vel = int(vel)
        distance = int(distance)
        timeout = int(timeout)
        posSensorNum = int(posSensorNum)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStart_xyz(direction, vel, distance, timeout, posSensorNum)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光寻位-任意方向
    @param  [in] directionPoint 寻位输入的点的xyz左边,[x,y,z]
    @param  [in] vel 速度 单位%
    @param  [in] distance 最大寻位距离 单位mm
    @param  [in] timeout 寻位超时时间 单位ms
    @param  [in] posSensorNum 激光标定的工具坐标编号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSearchStart_point(self, directionPoint, vel, distance, timeout, posSensorNum):
        while self.reconnect_flag:
            time.sleep(0.1)
        directionPoint = list(map(float, directionPoint))
        vel = int(vel)
        distance = int(distance)
        timeout = int(timeout)
        posSensorNum = int(posSensorNum)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSearchStart_point(6, vel, distance, timeout, posSensorNum, directionPoint[0], directionPoint[1], directionPoint[2])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光IP配置
    @param  [in] ip 激光外设的ip地址
    @param  [in] port 激光外设的端口号
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSensorConfig(self, ip, port):
        while self.reconnect_flag:
            time.sleep(0.1)
        ip =str (ip)
        port = int(port)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSensorConfig(ip, port)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光外设采样周期配置
    @param  [in] period 激光外设采样周期 单位ms
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserTrackingSensorSamplePeriod(self, period):
        while self.reconnect_flag:
            time.sleep(0.1)
        period = int(period)
        flag = True
        while flag:
            try:
                error = self.robot.LaserTrackingSensorSamplePeriod(period)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光外设驱动加载
    @param  [in] type 激光外设驱动的协议类型 101-睿牛 102-创想 103-全视 104-同舟 105-奥太
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoadPosSensorDriver(self, type):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        flag = True
        while flag:
            try:
                error = self.robot.LoadPosSensorDriver(type)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光外设驱动卸载
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def UnLoadPosSensorDriver(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.UnLoadPosSensorDriver()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光焊缝轨迹记录
    @param  [in] status 0-停止记录 1-实时跟踪  2-开始记录
    @param  [in] delayTime 延时时间 单位ms
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserSensorRecord1(self, status, delayTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        delayTime = int(delayTime)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorRecord1(status, delayTime)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光焊缝轨迹复现
    @param  [in] delayTime 延时时间 单位ms
    @param  [in] speed 速度 单位%
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserSensorReplay(self, delayTime, speed):
        while self.reconnect_flag:
            time.sleep(0.1)
        delayTime = int(delayTime)
        speed = float(speed)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorReplay(3, delayTime, speed)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 激光跟踪复现
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveLTR(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MoveLTR(0)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 运动到焊缝记录的起点
    @param  [in] moveType 0-PTP 1-LIN
    @param  [in] ovl 速度 单位%
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveToLaserRecordStart(self, moveType, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        moveType = int(moveType)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToLaserRecordStart(moveType, ovl)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 运动到焊缝记录的终点
    @param  [in] moveType 0-PTP 1-LIN
    @param  [in] ovl 速度 单位%
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveToLaserRecordEnd(self, moveType, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        moveType = int(moveType)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToLaserRecordEnd(moveType, ovl)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 运动到激光传感器寻位点
    @param  [in] moveFlag 运动类型：0-PTP；1-LIN
    @param  [in] ovl 速度缩放因子，0-100
    @param  [in] dataFlag 焊缝缓存数据选择：0-执行规划数据；1-执行记录数据
    @param  [in] plateType 板材类型：0-波纹板；1-瓦楞板；2-围栏板；3-油桶；4-波纹甲壳钢
    @param  [in] trackOffectType 激光传感器偏移类型：0-不偏移；1-基坐标系偏移；2-工具坐标系偏移；3-激光传感器原始数据偏移
    @param  [in] offset 偏移量
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveToLaserSeamPos(self, moveFlag, ovl, dataFlag, plateType, trackOffectType, offset):
        while self.reconnect_flag:
            time.sleep(0.1)
        moveFlag = int(moveFlag)
        ovl = float(ovl)
        plateType = int(plateType)
        trackOffectType = int(trackOffectType)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                error = self.robot.MoveToLaserSeamPos([moveFlag, ovl, dataFlag, plateType, trackOffectType, offset[0], offset[1],offset[2], offset[3], offset[4],offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取激光传感器寻位点坐标信息
    @param [in] trackOffectType 激光传感器偏移类型：0-不偏移；1-基坐标系偏移；2-工具坐标系偏移；3-激光传感器原始数据偏移
    @param [in] offset 偏移量
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） jPos 关节位置[°]
    @return 返回值（调用成功返回） descPos 笛卡尔位置[mm]
    @return 返回值（调用成功返回） tool 工具坐标系
    @return 返回值（调用成功返回） user 工件坐标系
    @return 返回值（调用成功返回） exaxis 扩展轴位置[mm]
    """

    @log_call
    @xmlrpc_timeout
    def GetLaserSeamPos(self, trackOffectType, offset):
        while self.reconnect_flag:
            time.sleep(0.1)
        trackOffectType = int(trackOffectType)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                _error = self.robot.GetLaserSeamPos([trackOffectType, offset[0], offset[1], offset[2], offset[3],offset[4], offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            paramStr = str(_error[1])
            # print(f"{paramStr}\n")

            parS = paramStr.split(',')
            if len(parS) != 20:
                return -1, None, None, None, None, None
            return (error, [float(parS[0]),float(parS[1]),float(parS[2]),float(parS[3]),float(parS[4]),float(parS[5])],
                    [float(parS[6]),float(parS[7]),float(parS[8]),float(parS[9]),float(parS[10]),float(parS[11])],
                    int(parS[12]), int(parS[13]),
                    [float(parS[16]),float(parS[17]),float(parS[18]),float(parS[19])])
        return error, None, None, None, None, None

    """2025.09.12"""
    """
    @brief 阻抗启停控制
    @param [in] status 0：关闭；1-开启
    @param [in] workSpace 0-关节空间；1-迪卡尔空间
    @param [in] forceThreshold 触发力阈值(N)
    @param [in] m 质量参数
    @param [in] b 阻尼参数
    @param [in] k 刚度参数
    @param [in] maxV 最大线速度(mm/s)
    @param [in] maxVA 最大线加速度(mm/s2)
    @param [in] maxW 最大角速度(°/s)
    @param [in] maxWA 最大角加速度(°/s2)
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ImpedanceControlStartStop(self, status, workSpace, forceThreshold, m, b, k, maxV, maxVA, maxW, maxWA):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        workSpace = int(workSpace)
        forceThreshold = list(map(float, forceThreshold))
        m = list(map(float, m))
        b = list(map(float, b))
        k = list(map(float, k))
        maxV = float(maxV)
        maxVA = float(maxVA)
        maxW = float(maxW)
        maxWA = float(maxWA)
        flag = True
        while flag:
            try:
                error = self.robot.ImpedanceControlStartStop([status, workSpace,
                                                     forceThreshold[0], forceThreshold[1], forceThreshold[2], forceThreshold[3], forceThreshold[4], forceThreshold[5]
                                                     ,m[0], m[1], m[2], m[3], m[4], m[5]
                                                     ,b[0], b[1], b[2], b[3], b[4], b[5]
                                                     ,k[0], k[1], k[2], k[3], k[4], k[5]
                                                     ,maxV, maxVA, maxW, maxWA])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.09.17"""
    """
    @brief 根据编号获取工具坐标系
    @param [in] id 工具坐标系编号
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetToolCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetToolCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief 根据编号获取工件坐标系
    @param [in] id 工件坐标系编号
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetWObjCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetWObjCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief 根据编号获取外部工具坐标系
    @param [in] id 外部工具坐标系编号
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetExToolCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetExToolCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief 根据编号获取扩展轴坐标系
    @param [in] id 扩展轴坐标系编号
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetExAxisCoordWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetExAxisCoordWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        return error, None

    """
    @brief 根据编号获取负载质量及质心
    @param [in] id 扩展轴坐标系编号
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） weight 负载质量
    @return 返回值（调用成功返回） cog 负载质心
    """

    @log_call
    @xmlrpc_timeout
    def GetTargetPayloadWithID(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.GetTargetPayloadWithID(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, _error[1], [_error[2], _error[3], _error[4]]
        return error, None, None

    """
    @brief 获取当前工具坐标系
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetCurToolCoord(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, [self.robot_state_pkg.toolCoord[0],
                   self.robot_state_pkg.toolCoord[1],
                   self.robot_state_pkg.toolCoord[2],
                   self.robot_state_pkg.toolCoord[3],
                   self.robot_state_pkg.toolCoord[4],
                   self.robot_state_pkg.toolCoord[5]]

    """
    @brief 获取当前工件坐标系
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetCurWObjCoord(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, [self.robot_state_pkg.wobjCoord[0],
                   self.robot_state_pkg.wobjCoord[1],
                   self.robot_state_pkg.wobjCoord[2],
                   self.robot_state_pkg.wobjCoord[3],
                   self.robot_state_pkg.wobjCoord[4],
                   self.robot_state_pkg.wobjCoord[5]]

    """
    @brief 获取当前外部工具坐标系
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetCurExToolCoord(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, [self.robot_state_pkg.extoolCoord[0],
                   self.robot_state_pkg.extoolCoord[1],
                   self.robot_state_pkg.extoolCoord[2],
                   self.robot_state_pkg.extoolCoord[3],
                   self.robot_state_pkg.extoolCoord[4],
                   self.robot_state_pkg.extoolCoord[5]]

    """
    @brief 获取当前扩展轴坐标系
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） coord 坐标系数值
    """

    @log_call
    @xmlrpc_timeout
    def GetCurExAxisCoord(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        return 0, [self.robot_state_pkg.exAxisCoord[0],
                   self.robot_state_pkg.exAxisCoord[1],
                   self.robot_state_pkg.exAxisCoord[2],
                   self.robot_state_pkg.exAxisCoord[3],
                   self.robot_state_pkg.exAxisCoord[4],
                   self.robot_state_pkg.exAxisCoord[5]]

    """2025.09.18"""
    """
    @brief 设置自定义摆动参数
    @param [in] id 自定义摆动编号：0-2
    @param [in] pointNum 摆动点位个数 0-10
    @param [in] point 移动端点数据x,y,z
    @param [in] stayTime 摆动停留时间ms
    @param [in] frequency 摆动频率 Hz
    @param [in] incStayType 等待模式：0-周期不包含等待时间；1-周期包含等待时间
    @param [in] stationary 摆动位置等待：0-等待时间内继续运动；1-等待时间内位置静止
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def CustomWeaveSetPara(self, id, pointNum, point, stayTime, frequency, incStayType, stationary):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        pointNum = int(pointNum)
        point = list(map(float, point))
        stayTime = list(map(float,stayTime))
        frequency = float(frequency)
        incStayType = int(incStayType)
        stationary = int(stationary)
        flag = True
        while flag:
            try:
                error = self.robot.CustomWeaveSetPara([id, pointNum,
                                                       point[0],point[1],point[2],point[3],point[4],point[5],point[6],point[7],point[8],point[9],
                                                       point[10],point[11],point[12],point[13],point[14],point[15],point[16],point[17],point[18],point[19],
                                                       point[20],point[21],point[22],point[23],point[24],point[25],point[26],point[27],point[28],point[29],
                                                       stayTime[0],stayTime[1],stayTime[2],stayTime[3],stayTime[4],stayTime[5],stayTime[6],stayTime[7],stayTime[8],stayTime[9],
                                                       frequency,incStayType,stationary])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取自定义摆动参数
    @param [in] id 自定义摆动编号：0-2
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） pointNum 摆动点位个数 0-10
    @return 返回值（调用成功返回） point 移动端点数据x,y,z
    @return 返回值（调用成功返回） stayTime 摆动停留时间ms
    @return 返回值（调用成功返回） frequency 摆动频率 Hz
    @return 返回值（调用成功返回） incStayType 等待模式：0-周期不包含等待时间；1-周期包含等待时间
    @return 返回值（调用成功返回） stationary 摆动位置等待：0-等待时间内继续运动；1-等待时间内位置静止
    """

    @log_call
    @xmlrpc_timeout
    def CustomWeaveGetPara(self, id):
        while self.reconnect_flag:
            time.sleep(0.1)
        id = int(id)
        flag = True
        while flag:
            try:
                _error = self.robot.CustomWeaveGetPara(id)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            paramStr = str(_error[1])
            # print(f"{paramStr}\n")

            parS = paramStr.split(',')
            if len(parS) != 44:
                return -1, None, None, None, None, None, None
            return (error, int(parS[0]), [float(parS[1]),  float(parS[2]),  float(parS[3]),
                                          float(parS[4]),  float(parS[5]),  float(parS[6]),
                                          float(parS[7]),  float(parS[8]),  float(parS[9]),
                                          float(parS[10]), float(parS[11]), float(parS[12]),
                                          float(parS[13]), float(parS[14]), float(parS[15]),
                                          float(parS[16]), float(parS[17]), float(parS[18]),
                                          float(parS[19]), float(parS[20]), float(parS[21]),
                                          float(parS[22]), float(parS[23]), float(parS[24]),
                                          float(parS[25]), float(parS[26]), float(parS[27]),
                                          float(parS[28]), float(parS[29]), float(parS[30])],
                                          [float(parS[31]), float(parS[32]), float(parS[33]),
                                           float(parS[34]), float(parS[35]), float(parS[36]),
                                           float(parS[37]), float(parS[38]), float(parS[39]),
                                           float(parS[40])],
                                          float(parS[41]),int(parS[42]), int(parS[43]))
        return error, None, None, None, None, None, None

    """2025.09.19"""
    """
    @brief 机器人操作系统升级(LA控制箱)
    @param [in] filePath 操作系统升级包全路径
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def KernelUpgrade(self, filePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        filePath = str(filePath)
        errcode = self.__FileUpLoad(6, filePath)
        if errcode == 0:
            try:
                result = self.robot.KernelUpgrade()
                if result is not None and hasattr(result, '__len__') and len(result) == 0:
                    # print("警告: 内核升级调用成功，但返回空数据")
                    return 0
                return result if result is not None else 0
            except xmlrpc.client.Fault as e:
                if "array has only 0 items" in str(e):
                    # print("内核升级指令已发送，但服务器返回空响应")
                    return 0
                else:
                    # print(f"内核升级失败: {e}")
                    return -1
        return errcode

    """
    @brief 获取机器人操作系统升级结果(LA控制箱)
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） result 升级结果：0:成功；-1:失败
    """

    @log_call
    @xmlrpc_timeout
    def GetKernelUpgradeResult(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetKernelUpgradeResult()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, _error[1]
        return error, None

    """3.8.7"""
    """2025.10.11"""
    """
    @brief 关节扭矩传感器灵敏度标定功能开启
    @param [in] status 0-关闭；1-开启
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def JointSensitivityEnable(self, status):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        flag = True
        while flag:
            try:
                error = self.robot.JointSensitivityEnable([status])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取关节扭矩传感器灵敏度标定结果
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） calibResult j1~j6关节灵敏度[0-1]
    @return 返回值（调用成功返回） linearityn j1~j6关节线性度[0-1]
    """

    @log_call
    @xmlrpc_timeout
    def JointSensitivityCalibration(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.JointSensitivityCalibration()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]], [_error[7], _error[8], _error[9], _error[10], _error[11], _error[12]]
        return error, None, None

    """
    @brief 关节扭矩传感器灵敏度数据采集
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def JointSensitivityCollect(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.JointSensitivityCollect()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """2025.10.15"""
    """
    @brief 清空运动指令队列
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MotionQueueClear(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MotionQueueClear()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取机器人8个从站端口错误帧数
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） inRecvErr 输入接收错误帧数
    @return 返回值（调用成功返回） inCRCErr 输入CRC错误帧数
    @return 返回值（调用成功返回） inTransmitErr 输入转发错误帧数
    @return 返回值（调用成功返回） inLinkErr 输入链接错误帧数
    @return 返回值（调用成功返回） outRecvErr 输出接收错误帧数
    @return 返回值（调用成功返回） outCRCErr 输出CRC错误帧数
    @return 返回值（调用成功返回） outTransmitErr 输出转发错误帧数
    @return 返回值（调用成功返回） outLinkErr 输出链接错误帧数
    """

    @log_call
    @xmlrpc_timeout
    def GetSlavePortErrCounter(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetSlavePortErrCounter()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            paramStr = str(_error[1])
            # print(f"{paramStr}\n")

            parS = paramStr.split(',')
            if len(parS) != 64:
                return -1, None, None, None, None, None, None, None, None
            return (error,
                    [int(parS[0]), int(parS[4]), int(parS[8]), int(parS[12]), int(parS[16]), int(parS[20]), int(parS[24]), int(parS[28])],
                    [int(parS[1]), int(parS[5]), int(parS[9]), int(parS[13]), int(parS[17]), int(parS[21]), int(parS[25]), int(parS[29])],
                    [int(parS[2]), int(parS[6]), int(parS[10]), int(parS[14]), int(parS[18]), int(parS[22]), int(parS[26]), int(parS[30])],
                    [int(parS[3]), int(parS[7]), int(parS[11]), int(parS[15]), int(parS[19]), int(parS[23]), int(parS[27]), int(parS[31])],
                    [int(parS[32]), int(parS[36]), int(parS[40]), int(parS[44]), int(parS[48]), int(parS[52]), int(parS[56]), int(parS[60])],
                    [int(parS[33]), int(parS[37]), int(parS[41]), int(parS[45]), int(parS[49]), int(parS[53]), int(parS[57]), int(parS[61])],
                    [int(parS[34]), int(parS[38]), int(parS[42]), int(parS[46]), int(parS[50]), int(parS[54]), int(parS[58]), int(parS[62])],
                    [int(parS[35]), int(parS[39]), int(parS[43]), int(parS[47]), int(parS[51]), int(parS[55]), int(parS[59]), int(parS[63])])
        return error, None, None, None, None, None, None, None, None

    """
    @brief 从站端口错误帧清零
    @param [in] slaveID 从站编号0~7
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SlavePortErrCounterClear(self, slaveID):
        while self.reconnect_flag:
            time.sleep(0.1)
        slaveID = int(slaveID)
        flag = True
        while flag:
            try:
                error = self.robot.SlavePortErrCounterClear(slaveID)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 设置各轴速度前馈系数
    @param [in] radio 各轴速度前馈系数
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetVelFeedForwardRatio(self, radio):
        while self.reconnect_flag:
            time.sleep(0.1)
        radio = list(map(float,radio))
        flag = True
        while flag:
            try:
                error = self.robot.SetVelFeedForwardRatio([radio[0],radio[1],radio[2],radio[3],radio[4],radio[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取各轴速度前馈系数
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） radio 各轴速度前馈系数
    """

    @log_call
    @xmlrpc_timeout
    def GetVelFeedForwardRatio(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.GetVelFeedForwardRatio()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3]), float(_error[4]), float(_error[5]), float(_error[6])]
        return error, None

    """
    @brief 机器人MCU日志生成
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def RobotMCULogCollect(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.RobotMCULogCollect()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """3.9.0"""
    """2025.11.03"""
    """
    @brief 移动到相贯线起始点
    @param  [in] 必选参数 mainPoint 主管6个示教点的笛卡尔位姿
    @param  [in] 必选参数 piecePoint 辅管6个示教点的笛卡尔位姿
    @param  [in] 必选参数 tool 工具坐标系编号
    @param  [in] 必选参数 wobj 工件坐标系编号
    @param  [in] 必选参数 vel 速度百分比
    @param  [in] 必选参数 acc 加速度百分比 
    @param  [in] 必选参数 ovl 速度缩放因子
    @param  [in] 必选参数 oacc 加速度缩放因子
    @param  [in] 必选参数 moveType 运动类型; 0-PTP；1-LIN
    @param  [in] 默认参数 mainExaxisPos 主管6个示教点扩展轴位置,默认[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] 默认参数 pieceExaxisPos 拼接管6个示教点扩展轴位置,默认[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] 默认参数 extAxisFlag 是否启用扩展轴；0-不启用；1-启用
    @param  [in] 默认参数 exaxisPos 起点扩展轴位置[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 moveDirection 运动方向；0-顺时针；1-逆时针
    @param  [in] 默认参数 offset 偏移量
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveToIntersectLineStart(self, mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveType,mainExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],
                                 pieceExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],extAxisFlag=0,
                                 exaxisPos=[0.0,0.0,0.0,0.0],moveDirection=0,offset=[0.0,0.0,0.0,0.0,0.0,0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        mainPoint0 = list(map(float, mainPoint[0]))
        mainPoint1 = list(map(float, mainPoint[1]))
        mainPoint2 = list(map(float, mainPoint[2]))
        mainPoint3 = list(map(float, mainPoint[3]))
        mainPoint4 = list(map(float, mainPoint[4]))
        mainPoint5 = list(map(float, mainPoint[5]))
        mainExaxisPos0 = list(map(float, mainExaxisPos[0]))
        mainExaxisPos1 = list(map(float, mainExaxisPos[1]))
        mainExaxisPos2 = list(map(float, mainExaxisPos[2]))
        mainExaxisPos3 = list(map(float, mainExaxisPos[3]))
        mainExaxisPos4 = list(map(float, mainExaxisPos[4]))
        mainExaxisPos5 = list(map(float, mainExaxisPos[5]))
        piecePoint0 = list(map(float, piecePoint[0]))
        piecePoint1 = list(map(float, piecePoint[1]))
        piecePoint2 = list(map(float, piecePoint[2]))
        piecePoint3 = list(map(float, piecePoint[3]))
        piecePoint4 = list(map(float, piecePoint[4]))
        piecePoint5 = list(map(float, piecePoint[5]))
        pieceExaxisPos0 = list(map(float, pieceExaxisPos[0]))
        pieceExaxisPos1 = list(map(float, pieceExaxisPos[1]))
        pieceExaxisPos2 = list(map(float, pieceExaxisPos[2]))
        pieceExaxisPos3 = list(map(float, pieceExaxisPos[3]))
        pieceExaxisPos4 = list(map(float, pieceExaxisPos[4]))
        pieceExaxisPos5 = list(map(float, pieceExaxisPos[5]))
        extAxisFlag = int(extAxisFlag)
        exaxisPos = list(map(float, exaxisPos))

        tool = int(tool)
        wobj = int(wobj)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        oacc = float(oacc)
        moveType = int(moveType)
        moveDirection = int(moveDirection)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                error = self.robot.MoveToIntersectLineStart([mainPoint0[0], mainPoint0[1], mainPoint0[2], mainPoint0[3], mainPoint0[4], mainPoint0[5],
                                                             mainPoint1[0], mainPoint1[1], mainPoint1[2], mainPoint1[3], mainPoint1[4], mainPoint1[5],
                                                             mainPoint2[0], mainPoint2[1], mainPoint2[2], mainPoint2[3], mainPoint2[4], mainPoint2[5],
                                                             mainPoint3[0], mainPoint3[1], mainPoint3[2], mainPoint3[3], mainPoint3[4], mainPoint3[5],
                                                             mainPoint4[0], mainPoint4[1], mainPoint4[2], mainPoint4[3], mainPoint4[4], mainPoint4[5],
                                                             mainPoint5[0], mainPoint5[1], mainPoint5[2], mainPoint5[3], mainPoint5[4], mainPoint5[5],
                                                             mainExaxisPos0[0], mainExaxisPos0[1], mainExaxisPos0[2], mainExaxisPos0[3],
                                                             mainExaxisPos1[0], mainExaxisPos1[1], mainExaxisPos1[2], mainExaxisPos1[3],
                                                             mainExaxisPos2[0], mainExaxisPos2[1], mainExaxisPos2[2], mainExaxisPos2[3],
                                                             mainExaxisPos3[0], mainExaxisPos3[1], mainExaxisPos3[2], mainExaxisPos3[3],
                                                             mainExaxisPos4[0], mainExaxisPos4[1], mainExaxisPos4[2], mainExaxisPos4[3],
                                                             mainExaxisPos5[0], mainExaxisPos5[1], mainExaxisPos5[2], mainExaxisPos5[3],
                                                             piecePoint0[0],piecePoint0[1],piecePoint0[2],piecePoint0[3],piecePoint0[4],piecePoint0[5],
                                                             piecePoint1[0],piecePoint1[1],piecePoint1[2],piecePoint1[3],piecePoint1[4],piecePoint1[5],
                                                             piecePoint2[0],piecePoint2[1],piecePoint2[2],piecePoint2[3],piecePoint2[4],piecePoint2[5],
                                                             piecePoint3[0],piecePoint3[1],piecePoint3[2],piecePoint3[3],piecePoint3[4],piecePoint3[5],
                                                             piecePoint4[0],piecePoint4[1],piecePoint4[2],piecePoint4[3],piecePoint4[4],piecePoint4[5],
                                                             piecePoint5[0],piecePoint5[1],piecePoint5[2],piecePoint5[3],piecePoint5[4],piecePoint5[5],
                                                             pieceExaxisPos0[0], pieceExaxisPos0[1], pieceExaxisPos0[2], pieceExaxisPos0[3],
                                                             pieceExaxisPos1[0], pieceExaxisPos1[1], pieceExaxisPos1[2], pieceExaxisPos1[3],
                                                             pieceExaxisPos2[0], pieceExaxisPos2[1], pieceExaxisPos2[2], pieceExaxisPos2[3],
                                                             pieceExaxisPos3[0], pieceExaxisPos3[1], pieceExaxisPos3[2], pieceExaxisPos3[3],
                                                             pieceExaxisPos4[0], pieceExaxisPos4[1], pieceExaxisPos4[2], pieceExaxisPos4[3],
                                                             pieceExaxisPos5[0], pieceExaxisPos5[1], pieceExaxisPos5[2], pieceExaxisPos5[3],
                                                             extAxisFlag,
                                                             exaxisPos[0], exaxisPos[1], exaxisPos[2], exaxisPos[3],
                                                             tool, wobj, vel, acc, ovl, oacc, moveType,moveDirection,
                                                             offset[0],offset[1],offset[2],offset[3],offset[4],offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 相贯线运动
    @param  [in] 必选参数 mainPoint 主管6个示教点的笛卡尔位姿
    @param  [in] 必选参数 piecePoint 辅管6个示教点的笛卡尔位姿
    @param  [in] 必选参数 tool 工具坐标系编号
    @param  [in] 默认参数 wobj 工件坐标系编号
    @param  [in] 默认参数 vel 速度百分比
    @param  [in] 默认参数 acc 加速度百分比 
    @param  [in] 默认参数 ovl 速度缩放因子
    @param  [in] 默认参数 oacc 加速度缩放因子
    @param  [in] 默认参数 moveDirection 运动方向; 0-顺时针；1-逆时针
    @param  [in] 默认参数 mainExaxisPos 主管6个示教点扩展轴位置,默认[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] 默认参数 pieceExaxisPos 拼接管6个示教点扩展轴位置,默认[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] 默认参数 extAxisFlag 是否启用扩展轴；0-不启用；1-启用
    @param  [in] 默认参数 exaxisPos 起点扩展轴位置[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    @param  [in] 默认参数 offset 偏移量
    @return 错误码 成功- 0, 失败-错误码
    """


    @log_call
    @xmlrpc_timeout
    def MoveIntersectLine(self, mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveDirection,mainExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],
                                 pieceExaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],extAxisFlag=0,
                                 exaxisPos=[[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]],offset=[0.0,0.0,0.0,0.0,0.0,0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        mainPoint0 = list(map(float, mainPoint[0]))
        mainPoint1 = list(map(float, mainPoint[1]))
        mainPoint2 = list(map(float, mainPoint[2]))
        mainPoint3 = list(map(float, mainPoint[3]))
        mainPoint4 = list(map(float, mainPoint[4]))
        mainPoint5 = list(map(float, mainPoint[5]))
        mainExaxisPos0 = list(map(float, mainExaxisPos[0]))
        mainExaxisPos1 = list(map(float, mainExaxisPos[1]))
        mainExaxisPos2 = list(map(float, mainExaxisPos[2]))
        mainExaxisPos3 = list(map(float, mainExaxisPos[3]))
        mainExaxisPos4 = list(map(float, mainExaxisPos[4]))
        mainExaxisPos5 = list(map(float, mainExaxisPos[5]))
        piecePoint0 = list(map(float, piecePoint[0]))
        piecePoint1 = list(map(float, piecePoint[1]))
        piecePoint2 = list(map(float, piecePoint[2]))
        piecePoint3 = list(map(float, piecePoint[3]))
        piecePoint4 = list(map(float, piecePoint[4]))
        piecePoint5 = list(map(float, piecePoint[5]))
        pieceExaxisPos0 = list(map(float, pieceExaxisPos[0]))
        pieceExaxisPos1 = list(map(float, pieceExaxisPos[1]))
        pieceExaxisPos2 = list(map(float, pieceExaxisPos[2]))
        pieceExaxisPos3 = list(map(float, pieceExaxisPos[3]))
        pieceExaxisPos4 = list(map(float, pieceExaxisPos[4]))
        pieceExaxisPos5 = list(map(float, pieceExaxisPos[5]))
        extAxisFlag = int(extAxisFlag)
        exaxisPos0 = list(map(float, exaxisPos[0]))
        exaxisPos1 = list(map(float, exaxisPos[1]))
        exaxisPos2 = list(map(float, exaxisPos[2]))
        exaxisPos3 = list(map(float, exaxisPos[3]))
        tool = int(tool)
        wobj = int(wobj)
        vel = float(vel)
        acc = float(acc)
        ovl = float(ovl)
        oacc = float(oacc)
        moveDirection = int(moveDirection)
        offset = list(map(float, offset))
        flag = True
        while flag:
            try:
                error = self.robot.MoveIntersectLine(
                    [mainPoint0[0], mainPoint0[1], mainPoint0[2], mainPoint0[3], mainPoint0[4], mainPoint0[5],
                                                             mainPoint1[0], mainPoint1[1], mainPoint1[2], mainPoint1[3], mainPoint1[4], mainPoint1[5],
                                                             mainPoint2[0], mainPoint2[1], mainPoint2[2], mainPoint2[3], mainPoint2[4], mainPoint2[5],
                                                             mainPoint3[0], mainPoint3[1], mainPoint3[2], mainPoint3[3], mainPoint3[4], mainPoint3[5],
                                                             mainPoint4[0], mainPoint4[1], mainPoint4[2], mainPoint4[3], mainPoint4[4], mainPoint4[5],
                                                             mainPoint5[0], mainPoint5[1], mainPoint5[2], mainPoint5[3], mainPoint5[4], mainPoint5[5],
                                                             mainExaxisPos0[0], mainExaxisPos0[1], mainExaxisPos0[2], mainExaxisPos0[3],
                                                             mainExaxisPos1[0], mainExaxisPos1[1], mainExaxisPos1[2], mainExaxisPos1[3],
                                                             mainExaxisPos2[0], mainExaxisPos2[1], mainExaxisPos2[2], mainExaxisPos2[3],
                                                             mainExaxisPos3[0], mainExaxisPos3[1], mainExaxisPos3[2], mainExaxisPos3[3],
                                                             mainExaxisPos4[0], mainExaxisPos4[1], mainExaxisPos4[2], mainExaxisPos4[3],
                                                             mainExaxisPos5[0], mainExaxisPos5[1], mainExaxisPos5[2], mainExaxisPos5[3],
                                                             piecePoint0[0],piecePoint0[1],piecePoint0[2],piecePoint0[3],piecePoint0[4],piecePoint0[5],
                                                             piecePoint1[0],piecePoint1[1],piecePoint1[2],piecePoint1[3],piecePoint1[4],piecePoint1[5],
                                                             piecePoint2[0],piecePoint2[1],piecePoint2[2],piecePoint2[3],piecePoint2[4],piecePoint2[5],
                                                             piecePoint3[0],piecePoint3[1],piecePoint3[2],piecePoint3[3],piecePoint3[4],piecePoint3[5],
                                                             piecePoint4[0],piecePoint4[1],piecePoint4[2],piecePoint4[3],piecePoint4[4],piecePoint4[5],
                                                             piecePoint5[0],piecePoint5[1],piecePoint5[2],piecePoint5[3],piecePoint5[4],piecePoint5[5],
                                                             pieceExaxisPos0[0], pieceExaxisPos0[1], pieceExaxisPos0[2], pieceExaxisPos0[3],
                                                             pieceExaxisPos1[0], pieceExaxisPos1[1], pieceExaxisPos1[2], pieceExaxisPos1[3],
                                                             pieceExaxisPos2[0], pieceExaxisPos2[1], pieceExaxisPos2[2], pieceExaxisPos2[3],
                                                             pieceExaxisPos3[0], pieceExaxisPos3[1], pieceExaxisPos3[2], pieceExaxisPos3[3],
                                                             pieceExaxisPos4[0], pieceExaxisPos4[1], pieceExaxisPos4[2], pieceExaxisPos4[3],
                                                             pieceExaxisPos5[0], pieceExaxisPos5[1], pieceExaxisPos5[2], pieceExaxisPos5[3],
                                                             extAxisFlag,
                                                             exaxisPos0[0], exaxisPos0[1], exaxisPos0[2], exaxisPos0[3],
                                                             exaxisPos1[0], exaxisPos1[1], exaxisPos1[2], exaxisPos1[3],
                                                             exaxisPos2[0], exaxisPos2[1], exaxisPos2[2], exaxisPos2[3],
                                                             exaxisPos3[0], exaxisPos3[1], exaxisPos3[2], exaxisPos3[3],
                     tool, wobj, vel, acc, ovl, oacc, moveDirection,
                                                             offset[0],offset[1],offset[2],offset[3],offset[4],offset[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """3.9.0"""
    """2025.11.20"""
    """
    @brief 获取关节扭矩传感器迟滞误差
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） hysteresisError j1~j6关节迟滞误差
    """

    @log_call
    @xmlrpc_timeout
    def JointHysteresisError(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.JointHysteresisError()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3]), float(_error[4]), float(_error[5]), float(_error[6])]
        return error, None


    """
    @brief 获取关节扭矩传感器重复精度
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） repeatability j1~j6关节重复精度
    """

    @log_call
    @xmlrpc_timeout
    def JointRepeatability(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.JointRepeatability()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3]), float(_error[4]), float(_error[5]), float(_error[6])]
        return error, None

    """
    @brief 设置关节力传感器参数
    @param  [in] 必选参数 M J1-J6质量系数[]
    @param  [in] 必选参数 B J1-J6阻尼系数[]
    @param  [in] 必选参数 K J1-J6刚度系数[]
    @param  [in] 默认参数 threshold 力控制阈值，Nm
    @param  [in] 默认参数 sensitivity 灵敏度,Nm/V,[]
    @param  [in] 默认参数 setZeroFlag 功能开启标志位；0-关闭；1-开启；2-位置1记录零点；3-位置2记录零点
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAdmittanceParams(self, M, B, K, threshold, sensitivity, setZeroFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        M = list(map(float, M))
        B = list(map(float, B))
        K = list(map(float, K))
        threshold = list(map(float, threshold))
        sensitivity = list(map(float, sensitivity))
        setZeroFlag = int(setZeroFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetAdmittanceParams(
                    [M[0], M[1], M[2], M[3], M[4], M[5],
                     B[0], B[1], B[2], B[3], B[4], B[5],
                     K[0], K[1], K[2], K[3], K[4], K[5],
                     threshold[0], threshold[1], threshold[2], threshold[3], threshold[4], threshold[5],
                     sensitivity[0], sensitivity[1], sensitivity[2], sensitivity[3], sensitivity[4], sensitivity[5],
                     setZeroFlag])
                flag = False
            except socket.error as e:
                flag = True
        return error

    """3.9.1"""
    """2025.12.01"""
    """
    @brief 开启力矩补偿功能及补偿系数
    @param  [in] 必选参数 status 开关，0-关闭；1-开启
    @param  [in] 必选参数 torqueCoeff J1-J6力矩补偿系数[0-1]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SerCoderCompenParams(self, status, torqueCoeff):
        while self.reconnect_flag:
            time.sleep(0.1)
        status = int(status)
        torqueCoeff = list(map(float, torqueCoeff))
        flag = True
        while flag:
            try:
                error = self.robot.SerCoderCompenParams([status,torqueCoeff[0],torqueCoeff[1],torqueCoeff[2],torqueCoeff[3],torqueCoeff[4],torqueCoeff[5]])
                flag = False
            except socket.error as e:
                flag = True
        return error


    """3.9.2"""
    """2025.12.30"""
    """
    @brief 光电传感器TCP标定-计算工具RPY
    @param  [in] 必选参数 机器人笛卡尔位置
    @param  [in] 必选参数 Etool 当前工具坐标系数值
    @param  [in] 必选参数 senser 当前传感器坐标系数值(暂未开放)
    @param  [in] 必选参数 radius 圆周运动半径mm(暂未开放)
    @param  [in] 必选参数 dz 沿基座标系z轴负方向运动距离；当dz = 10000时，函数直接返回工具RPY
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） TCPRPY 工具RPY数值
    """

    @log_call
    @xmlrpc_timeout
    def TCPComputeRPY(self, Btool, Etool, sensor, radius, dz):
        while self.reconnect_flag:
            time.sleep(0.1)
        Btool = list(map(float, Btool))
        Etool = list(map(float, Etool))
        sensor = list(map(float, sensor))
        radius = float(radius)
        dz = float(dz)
        flag = True
        while flag:
            try:
                _error = self.robot.TCPComputeRPY([Btool[0],Btool[1],Btool[2],Btool[3],Btool[4],Btool[5],
                                                   Etool[0],Etool[1],Etool[2],Etool[3],Etool[4],Etool[5],
                                                   sensor[0],sensor[1],sensor[2],sensor[3],sensor[4],sensor[5],
                                                   radius,dz,0.0,0.0,0.0,0.0])
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3])]
        return error, None

    """
    @brief 光电传感器TCP标定-计算工具XYZ
    @param  [in] 必选参数 select 0-计算工具TCP；1-计算传感器原点；2-计算传感器姿态；3-直接返回工具TCP；4-记录当前工件坐标系和工具坐标系
    @param  [in] 必选参数 originDirection 0-X方向；1-Y方向；2-Z方向
    @param  [in] 必选参数 pos1 机器人笛卡尔位置1
    @param  [in] 必选参数 pos2 机器人笛卡尔位置2
    @param  [in] 必选参数 pos3 机器人笛卡尔位置3
    @param  [in] 必选参数 pos4 机器人笛卡尔位置4
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） TCP 工具XYZ数值
    """

    @log_call
    @xmlrpc_timeout
    def TCPComputeXYZ(self, select, originDirection, pos1, pos2, pos3, pos4):
        while self.reconnect_flag:
            time.sleep(0.1)
        select = int(select)
        originDirection = float(originDirection)
        pos1 = list(map(float, pos1))
        pos2 = list(map(float, pos2))
        pos3 = list(map(float, pos3))
        pos4 = list(map(float, pos4))
        flag = True
        while flag:
            try:
                _error = self.robot.TCPComputeXYZ([select,originDirection,
                                                   pos1[0],pos1[1],pos1[2],
                                                   pos2[0],pos2[1],pos2[2],
                                                   pos3[0],pos3[1],pos3[2],
                                                   pos4[0],pos4[1],pos4[2]])
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3])]
        return error, None

    """
    @brief 光电传感器TCP标定-开始记录末端法兰中心位置
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def TCPRecordFlangePosStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.TCPRecordFlangePosStart()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 光电传感器TCP标定-停止记录末端法兰中心位置
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def TCPRecordFlangePosEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.TCPRecordFlangePosEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 光电传感器TCP标定-获取末端工具中心点位置
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） TCP 工具中心点位置(x,y,z)
    """

    @log_call
    @xmlrpc_timeout
    def TCPGetRecordFlangePos(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                _error = self.robot.TCPGetRecordFlangePos()
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3])]
        return error, None

    """
    @brief 光电传感器TCP标定
    @param  [in] 必选参数 luaPath 自动标定lua程序路径：QX版本机器人-"/fruser/FR_CalibrateTheToolTcp.lua";LA版本机器人-"/usr/local/etc/controller/lua/FR_CalibrateTheToolTcp.lua"
    @param  [in] 必选参数 offsetX 示教点偏移(x,y,z)mm
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） TCP 工具XYZ数值
    """

    @log_call
    @xmlrpc_timeout
    def PhotoelectricSensorTCPCalibration(self, luaPath, offsetX):
        while self.reconnect_flag:
            time.sleep(0.1)
        luaPath = str(luaPath)
        offsetX = list(map(float, offsetX))
        flag = True
        while flag:
            try:
                _error = self.SetSysVarValue(1,offsetX[0])
                if _error != 0:
                    return _error
                _error = self.SetSysVarValue(2,offsetX[1])
                if _error != 0:
                    return _error
                _error = self.SetSysVarValue(3,offsetX[2])
                if _error != 0:
                    return _error
                _error = self.Mode(0)
                if _error != 0:
                    return _error
                _error = self.ProgramLoad(luaPath)
                if _error != 0:
                    return _error
                _error = self.ProgramRun()
                if _error != 0:
                    return _error
                time.sleep(2)
                while self.robot_state_pkg.program_state != 1:
                    time.sleep(0.2)
                _error,TCPxyz = self.TCPComputeXYZ(3,0,
                                                   [0,0,0],
                                                   [0,0,0],
                                                   [0,0,0],
                                                   [0,0,0])
                if _error != 0:
                    return _error
                _error,TCPrpy = self.TCPComputeRPY([0,0,0,0,0,0],
                                                   [0,0,0,0,0,0],
                                                   [0,0,0,0,0,0],
                                                   1.0,1000)
                if _error != 0:
                    return _error
                flag = False
            except socket.error as e:
                flag = True
        error = _error
        if error == 0:
            return error, [float(TCPxyz[0]), float(TCPxyz[1]), float(TCPxyz[2]),float(TCPrpy[0]), float(TCPrpy[1]), float(TCPrpy[2])]
        return error, None

    """2026.01.08"""
    """
    @brief 激光焊缝轨迹复现
    @param  [in] 必选参数 delayMode 模式 0-延时时间 1-延时距离
    @param  [in] 必选参数 delayTime 延时时间 单位ms
    @param  [in] 必选参数 delayDisExAxisNum 扩展轴编号
    @param  [in] 必选参数 delayDis 延时距离 单位mm
    @param  [in] 必选参数 sensitivePara 补偿灵敏系数
    @param  [in] 必选参数 trackMode 定点跟踪类型。0-扩展轴异步运动；1-机器人
    @param  [in] 必选参数 triggerMode 定点跟踪触发方式。0-跟踪时长；1-IO
    @param  [in] 必选参数 runTime 机器人定点跟踪时长(s)
    @param  [in] 必选参数 speed 速度 单位%
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LaserSensorRecordandReplay(self, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, trackMode, triggerMode, runTime, speed):
        while self.reconnect_flag:
            time.sleep(0.1)
        delayMode = int(delayMode)
        delayTime = int(delayTime)
        delayDisExAxisNum = int(delayDisExAxisNum)
        delayDis = float(delayDis)
        sensitivePara = float(sensitivePara)
        trackMode = int(trackMode)
        triggerMode = int(triggerMode)
        runTime = float(runTime)
        speed = float(speed)
        flag = True
        while flag:
            try:
                error = self.robot.LaserSensorRecordandReplay(4, delayTime, speed, delayMode, delayDisExAxisNum, delayDis, sensitivePara, trackMode, triggerMode, runTime)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 原地空运动
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveStationary(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.MoveStationary()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """3.9.3"""
    """2026.01.29"""
    """
    @brief 逆运动学求解，笛卡尔空间包含扩展轴位置
    @param  [in] 必选参数 type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
    @param  [in] 必选参数 desc_pos 笛卡尔位姿
    @param  [in] 必选参数 exaxis 扩展轴位置
    @param  [in] 必选参数 tool 工具号
    @param  [in] 必选参数 workPiece 工件号
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） joint_pos 关节位置
    """

    @log_call
    @xmlrpc_timeout
    def GetInverseKinExaxis(self, type, desc_pos, exaxis, tool, workPiece):
        while self.reconnect_flag:
            time.sleep(0.1)
        type = int(type)
        desc_pos = list(map(float, desc_pos))
        exaxis = list(map(float, exaxis))
        tool = int(tool)
        workPiece = int(workPiece)
        flag = True
        while flag:
            try:
                _error = self.robot.GetInverseKinExaxis(type,
                                                        [desc_pos[0],desc_pos[1],desc_pos[2],
                                                         desc_pos[3],desc_pos[4],desc_pos[5]],
                                                        [exaxis[0],exaxis[1],exaxis[2],exaxis[3]],
                                                        tool,workPiece)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            return error, [float(_error[1]), float(_error[2]), float(_error[3]),float(_error[4]), float(_error[5]), float(_error[6])]
        return error, None

    """3.9.4"""
    """2026.02.28"""
    """
    @brief 运动到TPD轨迹记录起点
    @param  [in] 必选参数 name 轨迹文件名
    @param  [in] 必选参数 moveType 运动类型；0-PTP; 1-LIN
    @param  [in] 必选参数 ovl 速度缩放百分比，范围[0~100]
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveToTPDStart(self, name, moveType, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        name = str(name)
        moveType = int(moveType)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.MoveToTPDStart(name, moveType, ovl)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 开启末端通用透传功能
    @param [in] 使能，0-关闭，1-开启
    @return  错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetAxleGenComEnable(self, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.SetAxleGenComEnable(mode)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 按长度获取周期数据
    @param [in] len，返回的长度
    @return  错误码
    """
    @log_call
    @xmlrpc_timeout
    def GetAxleGenComCycleData(self, len):
        while self.reconnect_flag:
            time.sleep(0.1)
        len = int(len)
        cycle_data = [0] * len
        flag = True
        while flag:
            try:
                _error = self.robot.GetAxleGenComCycleData(len)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            for i in range(len):
                cycle_data[i] = int(_error[i + 1])
            return error, cycle_data
        return error

    """
    @brief 末端发送非周期数据并等待应答
    @param [in] len_snd，发送的长度
    @param [in] sndBuff[]，发送数据
    @param [in] len_rcv，选择接受的长度
    @param [out] rcvBuff[]，应答的数据
    @return  错误码
    """
    @log_call
    @xmlrpc_timeout
    def SndRcvAxleGenComCmdData(self, len_snd, sndBuff, len_rcv):
        while self.reconnect_flag:
            time.sleep(0.1)
        len_snd = int(len_snd)
        sndBuff = list(map(int, sndBuff))
        len_rcv = int(len_rcv)
        rcv_data = [0] * len_rcv
        flag = True
        while flag:
            try:
                _error = self.robot.SndRcvAxleGenComCmdData(len_snd, sndBuff, len_rcv)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            for i in range(len_rcv):
                rcv_data[i] = int(_error[i + 1])
            return error, rcv_data
        return error,None

    """
    @brief 设置端口通讯断开时停止机器人运行
    @param [in] portID 端口编号 0-8080；1-8083；2-20002；3-20004
    @param [in] enable 0-关闭；1-开启
    @param [in] confirmTime 通讯中断确认时长(ms)[0-5000]
    @return  错误码
    """
    @log_call
    @xmlrpc_timeout
    def SetRobotStopOnComDisc(self, portID, enable, confirmTime):
        while self.reconnect_flag:
            time.sleep(0.1)
        portID = int(portID)
        enable = int(enable)
        confirmTime = int(confirmTime)
        # 将三个参数合并成一个数组
        params = [portID, enable, confirmTime]
        flag = True
        while flag:
            try:
                error = self.robot.SetRobotStopOnComDisc(params)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 获取端口通讯断开时停止机器人运行参数
    @param [in] portID 端口编号 0-8080；1-8083；2-20002；3-20004
    @param [out] enable 0-关闭；1-开启
    @param [out] confirmTime 通讯中断确认时长(ms)[0-5000]
    @return  错误码
    """
    @log_call
    @xmlrpc_timeout
    def GetRobotStopOnComDisc(self, portID):
        while self.reconnect_flag:
            time.sleep(0.1)
        portID = int(portID)
        # 将portID作为数组传入
        params = [portID]
        enable = 0
        confirmTime = 0
        flag = True
        while flag:
            try:
                _error = self.robot.GetRobotStopOnComDisc(params)
                flag = False
            except socket.error as e:
                flag = True
        error = _error[0]
        if error == 0:
            enable = int(_error[1])
            confirmTime = int(_error[2])
            return error, enable, confirmTime
        return error, None, None

    """
    @brief 设置安全速度参数
    @param [in] enable 0-关；1-手动模式启用；2-所有模式启用(不支持自动限速)
    @param [in] maxTCPVel 限制最大TCP速度;[0-1000]mm/s
    @param [in] strategy 超速后策略；0-停止报警；1-自动限速；2-停止报警并去使能
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetVelReducePara(self, enable, maxTCPVel, strategy):
        while self.reconnect_flag:
            time.sleep(0.1)

        enable = int(enable)
        maxTCPVel = float(maxTCPVel)
        strategy = int(strategy)

        # 参数校验：当enable==2且strategy==1时返回参数错误
        if enable == 2 and strategy == 1:
            return RobotError.ERR_PARAM_VALUE

        # 将三个参数合并成一个数组
        params = [enable, maxTCPVel, strategy]
        flag = True
        while flag:
            try:
                error = self.robot.SetVelReducePara(params)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 定点摆动开始
    @param [in] weaveNum 摆动编号[0-7]
    @param [in] mode 0-工具坐标系；1-参考点
    @param [in] refPoint 参考点笛卡尔坐标[x,y,z,a,b,c]
    @param [in] weaveTime 摆动时间[s]
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def OriginPointWeaveStart(self, weaveNum, mode, refPoint, weaveTime):
        while self.reconnect_flag:
            time.sleep(0.1)

        weaveNum = int(weaveNum)
        mode = int(mode)
        refPoint = list(map(float, refPoint))
        weaveTime = float(weaveTime)

        # 将参考点坐标拆分成6个参数
        params = [
            weaveNum,
            mode,
            refPoint[0],
            refPoint[1],
            refPoint[2],
            refPoint[3],
            refPoint[4],
            refPoint[5],
            weaveTime
        ]

        flag = True
        while flag:
            try:
                error = self.robot.OriginPointWeaveStart(params)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 定点摆动结束
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def OriginPointWeaveEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                error = self.robot.OriginPointWeaveEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """
    @brief 设置可配置CI端口功能
    @param [in] config CI0-CI7功能编码数组,0-无;1-起弧成功;2-焊机准备;3-传送带检测;4-暂停;5-恢复;6-启动;7-停止;
      58-暂停/恢复;9-启动/停止;10-脚踏拖动;11-移至作业原点;12-手自动切换;
      613-焊丝寻位成功;14-运动中断;15-启动主程序;16-启动倒带;17-启动确认;
      718-光电检测信号X;19-光电检测信号Y;20-外部急停输入信号1;21-外部急停输入信号2;
      822-一级缩减模式;23-二级缩减模式;24-三级缩减模式(停止);25-恢复焊接;26-终止焊接;
      927-辅助拖动开启;28-辅助拖动关闭;29-辅助拖动开启/关闭;30-清除所有错误;
      1031-手自动切换(高低电平);32-使能;33-去使能;34-使能/去使能(上升下降沿);35-定点跟踪开始/结束
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetDIConfig(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        # 将config数组转换为整数列表
        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetDIConfig(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取控制箱可配置CI端口功能
    @param [out] config CI0-CI7功能编码数组,0-无;1-起弧成功;2-焊机准备;3-传送带检测;4-暂停;5-恢复;6-启动;7-停止;
      58-暂停/恢复;9-启动/停止;10-脚踏拖动;11-移至作业原点;12-手自动切换;
      613-焊丝寻位成功;14-运动中断;15-启动主程序;16-启动倒带;17-启动确认;
      718-光电检测信号X;19-光电检测信号Y;20-外部急停输入信号1;21-外部急停输入信号2;
      822-一级缩减模式;23-二级缩减模式;24-三级缩减模式(停止);25-恢复焊接;26-终止焊接;
      927-辅助拖动开启;28-辅助拖动关闭;29-辅助拖动开启/关闭;30-清除所有错误;
      1031-手自动切换(高低电平);32-使能;33-去使能;34-使能/去使能(上升下降沿);35-定点跟踪开始/结束
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetDIConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetDIConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(8)]
            return error, config
        else:
            return error, None

    """
    @brief 设置可配置CO端口功能
    @param [in] config CO0-CO7功能编码数组,0-无;1-机器人报错;2-机器人运动中;3-喷涂启停;4-喷涂清枪;5-送气信号;6-起弧信号;7-点动送丝;
      58-反向送丝;9-JOB输入口1;10-JOB输入口2;11-JOB输入口3;12-传送带启停控制;13-机器人暂停中;14-到达作业原点;
      615-到达干涉区;16-焊丝寻位启停控制;17-机器人启动完成;18-程序启动停止;19-自动手动模式;20-急停输出信号1-安全;
      721-急停输出信号2-安全;22-LUA脚本程序运行停止;23-安全状态输出-安全;24-保护性停止状态输出-安全;
      825-机器人运动中-安全;26-机器人缩减模式-安全;27-机器人非缩减模式-安全;28-机器人非停止;29-机器人报错-指令点错误;
      930-机器人报错-驱动器错误;31-机器人报错-超出软限位错误;32-机器人报错-碰撞错误;33-机器人报错-活动从站数量错误;
      1034-机器人报错-从站错误;35-机器人报错-IO错误;36-机器人报错-夹爪错误;37-机器人报错-文件错误;38-机器人报错-奇异位姿错误;
      1139-机器人报错-驱动器通信错误;40-机器人报错-参数错误;41-机器人报错-外部轴超出软限位错误;42-机器人警告-警告;
      1243-机器人警告-安全门警告;44-机器人警告-运动警告;45-机器人警告-干涉区警告;46-机器人警告-安全墙警告;
      1347-使能状态;48-断线自动抬升中;49-立方体1干涉警告;50-立方体2干涉警告;51-立方体3干涉警告;52-立方体4干涉警告;
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetDOConfig(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetDOConfig(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取可配置CO端口功能
    @param [out] config CO0-CO7功能编码数组,0-无;1-机器人报错;2-机器人运动中;3-喷涂启停;4-喷涂清枪;5-送气信号;6-起弧信号;7-点动送丝;
      58-反向送丝;9-JOB输入口1;10-JOB输入口2;11-JOB输入口3;12-传送带启停控制;13-机器人暂停中;14-到达作业原点;
      615-到达干涉区;16-焊丝寻位启停控制;17-机器人启动完成;18-程序启动停止;19-自动手动模式;20-急停输出信号1-安全;
      721-急停输出信号2-安全;22-LUA脚本程序运行停止;23-安全状态输出-安全;24-保护性停止状态输出-安全;
      825-机器人运动中-安全;26-机器人缩减模式-安全;27-机器人非缩减模式-安全;28-机器人非停止;29-机器人报错-指令点错误;
      930-机器人报错-驱动器错误;31-机器人报错-超出软限位错误;32-机器人报错-碰撞错误;33-机器人报错-活动从站数量错误;
      1034-机器人报错-从站错误;35-机器人报错-IO错误;36-机器人报错-夹爪错误;37-机器人报错-文件错误;38-机器人报错-奇异位姿错误;
      1139-机器人报错-驱动器通信错误;40-机器人报错-参数错误;41-机器人报错-外部轴超出软限位错误;42-机器人警告-警告;
      1243-机器人警告-安全门警告;44-机器人警告-运动警告;45-机器人警告-干涉区警告;46-机器人警告-安全墙警告;
      1347-使能状态;48-断线自动抬升中;49-立方体1干涉警告;50-立方体2干涉警告;51-立方体3干涉警告;52-立方体4干涉警告;
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetDOConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetDOConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(8)]
            return error, config
        else:
            return error, None

    """
    @brief 设置末端可配置End-CI端口功能
    @param [in] config End CI0-CI1功能编码数组,0-无;1-拖动示教工具开关;2-点记录信号;3-手自动切换（脉冲信号）;4-TPD记录启动/停止;5-暂停运动;
      56-恢复运动;7-启动;8-停止;9-暂停/恢复;10-启动/停止;11-力传感器辅助拖动开启;12-力传感器辅助拖动关闭;
      613-力传感器辅助拖动开启/关闭;14-激光检测信号X;15-激光检测信号Y;16-PTP运动至作业原点;17-运动中断，根据信号停止当前运动;
      718-启动主程序;19-启动倒带;20-启动确认;21-恢复焊接;22-终止焊接;23-清除错误;24-手自动切换（高低电平）
      825-使能;26-去使能;27-使能/去使能;28-激光伺服跟踪启停信号;
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolDIConfig(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetToolDIConfig(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取末端可配置End-CI端口功能
    @param [out] config End CI0-CI1功能编码数组,0-无;1-拖动示教工具开关;2-点记录信号;3-手自动切换（脉冲信号）;4-TPD记录启动/停止;5-暂停运动;
      56-恢复运动;7-启动;8-停止;9-暂停/恢复;10-启动/停止;11-力传感器辅助拖动开启;12-力传感器辅助拖动关闭;
      613-力传感器辅助拖动开启/关闭;14-激光检测信号X;15-激光检测信号Y;16-PTP运动至作业原点;17-运动中断，根据信号停止当前运动;
      718-启动主程序;19-启动倒带;20-启动确认;21-恢复焊接;22-终止焊接;23-清除错误;24-手自动切换（高低电平）
      825-使能;26-去使能;27-使能/去使能;28-激光伺服跟踪启停信号;
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetToolDIConfig(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetToolDIConfig()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(2)]
            return error, config
        else:
            return error, None

    """
    @brief 设置控制箱可配置CI有效状态
    @param [in] config CI0-CI7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetDIConfigLevel(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetDIConfigLevel(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取控制箱可配置CI有效状态
    @param [out] config CI0-CI7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetDIConfigLevel(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetDIConfigLevel()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(8)]
            return error, config
        else:
            return error, None

    """
    @brief 设置控制箱可配置CO有效状态
    @param [in] config CO0-CO7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetDOConfigLevel(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetDOConfigLevel(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取控制箱可配置CO有效状态
    @param [out] config CO0-CO7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetDOConfigLevel(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetDOConfigLevel()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(8)]
            return error, config
        else:
            return error, None

    """
    @brief 设置末端可配置CI有效状态
    @param [in] config CI0-CI1端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetToolDIConfigLevel(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetToolDIConfigLevel(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取末端可配置CI有效状态
    @param [out] config CI0-CI1端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetToolDIConfigLevel(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetToolDIConfigLevel()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(2)]
            return error, config
        else:
            return error, None

    """
    @brief 设置控制箱标准DI有效状态
    @param [in] config DI0-DI7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetStandardDILevel(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetStandardDILevel(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取控制箱标准DI有效状态
    @param [out] config DI0-DI7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetStandardDILevel(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetStandardDILevel()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(8)]
            return error, config
        else:
            return error, None

    """
    @brief 设置控制箱标准DO有效状态
    @param [in] config DO0-DO7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetStandardDOLevel(self, config):
        while self.reconnect_flag:
            time.sleep(0.1)

        config = [int(x) for x in config]
        params = config

        flag = True
        while flag:
            try:
                error = self.robot.SetStandardDOLevel(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 获取控制箱标准DO有效状态
    @param [out] config DO0-DO7端口有效状态数组；0-高电平有效；1-低电平有效
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetStandardDOLevel(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        flag = True
        while flag:
            try:
                _error = self.robot.GetStandardDOLevel()
                flag = False
            except socket.error as e:
                flag = True

        error = _error[0]
        if error == 0:
            config = [int(_error[i + 1]) for i in range(8)]
            return error, config
        else:
            return error, None

    """
    @brief UDP扩展轴定位完成时间设置
    @param [in] time 定位完成时间[ms]
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetExAxisCmdDoneTime(self, time):
        while self.reconnect_flag:
            time.sleep(0.1)

        time = float(time)
        params = [time]

        flag = True
        while flag:
            try:
                error = self.robot.SetExAxisCmdDoneTime(params)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 下载开放协议Lua文件
    @param [in] fileName 开放协议文件名称“CtrlDev_XXX.lua”
    @param [in] savePath 开放协议保存文件路径
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def OpenLuaDownload(self, fileName, savePath):
        while self.reconnect_flag:
            time.sleep(0.1)

        # 参数检查
        if len(fileName) == 0:
            return RobotError.ERR_UPLOAD_FILE_NOT_FOUND

        # 检查文件扩展名
        _file_name = fileName.split('.')
        if len(_file_name) == 2 and _file_name[1] == "lua":
            print(f"download open lua.")
        else:
            return RobotError.ERR_FILE_NAME

        # 调用文件下载函数
        errcode = self.__FileDownLoad(11, fileName, savePath)
        return errcode

    """
    @brief 设置用户自定义机器人末端灯色
    @param [in] r 末端红灯控制；0-灭；1-亮
    @param [in] g 末端绿灯控制；0-灭；1-亮
    @param [in] b 末端蓝灯控制；0-灭；1-亮
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetUserLEDColor(self, r, g, b):
        while self.reconnect_flag:
            time.sleep(0.1)

        # 将bool值转换为int (True->1, False->0)
        r = 1 if r else 0
        g = 1 if g else 0
        b = 1 if b else 0

        flag = True
        while flag:
            try:
                error = self.robot.SetUserLEDColor(r, g, b)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """
    @brief 删除开放协议Lua文件
    @param [in] fileName 要删除的开放协议lua文件名“CtrlDev_XXX.lua”
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def OpenLuaDelete(self, fileName):
        while self.reconnect_flag:
            time.sleep(0.1)

        errcode = self.__FileDelete(11, fileName)
        return errcode

    """
    @brief 删除所有开放协议Lua文件
    @return 错误码
    """

    @log_call
    @xmlrpc_timeout
    def AllOpenLuaDelete(self):
        while self.reconnect_flag:
            time.sleep(0.1)

        errcode = self.__FileDelete(12, "openluas")
        return errcode