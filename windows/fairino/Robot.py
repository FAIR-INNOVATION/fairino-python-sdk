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

# from Cython.Compiler.Options import error_on_unknown_names

is_init =False
class ROBOT_AUX_STATE(Structure):
    _pack_ = 1
    _fields_ = [
        ("servoId", c_byte),         # 伺服驱动器ID号
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
        ("ready", c_byte),        # 伺服准备好
        ("inPos", c_byte),        # 伺服到位
        ("alarm", c_byte),        # 伺服报警
        ("flerr", c_byte),        # 跟随误差
        ("nlimit", c_byte),       # 到负限位
        ("pLimit", c_byte),       # 到正限位
        ("mdbsOffLine", c_byte),  # 驱动器485总线掉线
        ("mdbsTimeout", c_byte),  # 控制卡与控制箱485通信超时
        ("homingStatus", c_byte), # 扩展轴回零状态
    ]

class WELDING_BREAKOFF_STATE(Structure):
    _pack_ = 1
    _fields_ = [
        ("breakOffState", ctypes.c_uint8),        # 焊接中断状态
        ("weldArcState", ctypes.c_uint8),        # 焊接电弧中断状态
    ]

"""   
@brief  机器人状态反馈数据包
"""
class RobotStatePkg(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_head", c_uint16),      # 帧头 0x5A5A
        ("frame_cnt", c_byte),         # 帧计数
        ("data_len", c_uint16),        # 数据长度
        ("program_state", c_byte),     # 程序运行状态，1-停止；2-运行；3-暂停
        ("robot_state", c_byte),       # 机器人运动状态，1-停止；2-运行；3-暂停；4-拖动
        ("main_code", c_int),          # 主故障码
        ("sub_code", c_int),           # 子故障码
        ("robot_mode", c_byte),        # 机器人模式，0-自动模式；1-手动模式
        ("jt_cur_pos", c_double * 6),  # 机器人当前关节位置，假设有6个关节
        ("tl_cur_pos", ctypes.c_double * 6),  # 工具当前位姿
        ("flange_cur_pos", ctypes.c_double * 6),  # 末端法兰当前位姿
        ("actual_qd", ctypes.c_double * 6),  # 机器人当前关节速度
        ("actual_qdd", ctypes.c_double * 6),  # 机器人当前关节加速度
        ("target_TCP_CmpSpeed", ctypes.c_double * 2),  # 机器人TCP合成指令速度
        ("target_TCP_Speed", ctypes.c_double * 6),  # 机器人TCP指令速度
        ("actual_TCP_CmpSpeed", ctypes.c_double * 2),  # 机器人TCP合成实际速度
        ("actual_TCP_Speed", ctypes.c_double * 6),  # 机器人TCP实际速度
        ("jt_cur_tor", ctypes.c_double * 6),  # 当前扭矩
        ("tool", ctypes.c_int),  # 工具号
        ("user", ctypes.c_int),  # 工件号
        ("cl_dgt_output_h", ctypes.c_byte),  # 数字输出15-8
        ("cl_dgt_output_l", ctypes.c_byte),  # 数字输出7-0
        ("tl_dgt_output_l", ctypes.c_byte),  # 工具数字输出7-0(仅bit0-bit1有效)
        ("cl_dgt_input_h", ctypes.c_byte),  # 数字输入15-8
        ("cl_dgt_input_l", ctypes.c_byte),  # 数字输入7-0
        ("tl_dgt_input_l", ctypes.c_byte),  # 工具数字输入7-0(仅bit0-bit1有效)
        ("cl_analog_input", ctypes.c_uint16 * 2),  # 控制箱模拟量输入
        ("tl_anglog_input", ctypes.c_uint16),  # 工具模拟量输入
        ("ft_sensor_raw_data", ctypes.c_double * 6),  # 力/扭矩传感器原始数据
        ("ft_sensor_data", ctypes.c_double * 6),  # 力/扭矩传感器数据
        ("ft_sensor_active", ctypes.c_byte),  # 力/扭矩传感器激活状态， 0-复位，1-激活
        ("EmergencyStop", ctypes.c_byte),  # 急停标志
        ("motion_done", ctypes.c_int),  # 到位信号
        ("gripper_motiondone", ctypes.c_byte),  # 夹爪运动完成信号
        ("mc_queue_len", ctypes.c_int),  # 运动队列长度
        ("collisionState", ctypes.c_byte),  # 碰撞检测，1-碰撞；0-无碰撞
        ("trajectory_pnum", ctypes.c_int),  # 轨迹点编号
        ("safety_stop0_state", ctypes.c_byte),  # 安全停止信号SI0
        ("safety_stop1_state", ctypes.c_byte),  # 安全停止信号SI1
        ("gripper_fault_id", ctypes.c_byte),  # 错误夹爪号
        ("gripper_fault", ctypes.c_uint16),  # 夹爪故障
        ("gripper_active", ctypes.c_uint16),  # 夹爪激活状态
        ("gripper_position", ctypes.c_byte),  # 夹爪位置
        ("gripper_speed", ctypes.c_byte),  # 夹爪速度
        ("gripper_current", ctypes.c_byte),  # 夹爪电流
        ("gripper_tmp", ctypes.c_int),  # 夹爪温度
        ("gripper_voltage", ctypes.c_int),  # 夹爪电压
        ("auxState", ROBOT_AUX_STATE),  # 485扩展轴状态
        ("extAxisStatus", EXT_AXIS_STATUS*4),  # UDP扩展轴状态
        ("extDIState", c_uint16*8),  # 扩展DI输入
        ("extDOState", c_uint16*8),  # 扩展DO输出
        ("extAIState", c_uint16*4),  # 扩展AI输入
        ("extAOState", c_uint16*4),  # 扩展AO输出
        ("rbtEnableState", ctypes.c_int),  # 机器人使能状态
        ("jointDriverTorque", ctypes.c_double * 6),  # 关节驱动器当前扭矩
        ("jointDriverTemperature", ctypes.c_double * 6),  # 关节驱动器当前温度
        ("year", ctypes.c_uint16),  # 年
        ("mouth", ctypes.c_uint8),  # 月
        ("day", ctypes.c_uint8),  # 日
        ("hour", ctypes.c_uint8),  # 小时
        ("minute", ctypes.c_uint8),  # 分
        ("second", ctypes.c_uint8),  # 秒
        ("millisecond", ctypes.c_uint16),  # 毫秒
        ("softwareUpgradeState", ctypes.c_int),  # 机器人软件升级状态
        ("endLuaErrCode", ctypes.c_uint16),  # 末端LUA运行状态
        ("cl_analog_output", ctypes.c_uint16 * 2),  # 控制箱模拟量输出
        ("tl_analog_output", ctypes.c_uint16),  # 工具模拟量输出
        ("gripperRotNum", ctypes.c_float),  # 旋转夹爪当前旋转圈数
        ("gripperRotSpeed", ctypes.c_uint8),  # 旋转夹爪当前旋转速度百分比
        ("gripperRotTorque", ctypes.c_uint8),  # 旋转夹爪当前旋转力矩百分比
        ("weldingBreakOffState", WELDING_BREAKOFF_STATE), # 焊接中断状态
        ("jt_tgt_tor", ctypes.c_double * 6),  # 关节指令力矩
        ("smartToolState", ctypes.c_uint16),  # SmartTool手柄按钮状态
        ("check_sum", c_ushort)]  # 校验和


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
        if RPC.is_conect == False:
            return -4
        else:
            result = func(self, *args, **kwargs)
            return result

    return wrapper


class RobotError:
    ERR_SUCCESS = 0
    ERR_POINTTABLE_NOTFOUND = -7  # 上传文件不存在
    # ERR_SAVE_FILE_PATH_NOT_FOUND = -6  # 保存文件路径不存在
    ERR_NOT_FOUND_LUA_FILE = -5  # lua文件不存在
    ERR_RPC_ERROR = -4
    ERR_SOCKET_COM_FAILED = -2
    ERR_OTHER = -1
    ERROR_RECONN = -8
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


class RPC():
    ip_address = "192.168.58.2"

    logger = None
    log_output_model = -1
    queue = Queue(maxsize=10000 * 1024)
    logging_thread = None
    is_conect = True
    ROBOT_REALTIME_PORT = 20004
    # BUFFER_SIZE = 1024 * 8
    BUFFER_SIZE = 1024 * 1024
    thread=  threading.Thread()
    SDK_state=True

    sock_cli_state_state = False
    closeRPC_state = False
    reconnect_lock = False
    reconnect_flag = False
    g_sock_com_err = RobotError.ERROR_RECONN


    def __init__(self, ip="192.168.58.2"):
        self.lock = threading.Lock()  # 增加锁
        self.ip_address = ip
        link = 'http://' + self.ip_address + ":20003"
        self.robot = xmlrpc.client.ServerProxy(link)#xmlrpc连接机器人20003端口，用于发送机器人指令数据帧

        self.sock_cli_state = None
        self.robot_realstate_exit = False
        self.robot_state_pkg = RobotStatePkg#机器人状态数据

        self.stop_event = threading.Event()  # 停止事件
        self.connect_to_robot()
        thread= threading.Thread(target=self.robot_state_routine_thread)#创建线程循环接收机器人状态数据
        thread.daemon = True
        thread.start()
        time.sleep(1)
        print(self.robot)


        try:
            # 调用 XML-RPC 方法
            socket.setdefaulttimeout(1)
            self.robot.GetControllerIP()
        except socket.timeout:
            print("XML-RPC connection timed out.")
            RPC.is_conect = False

        except socket.error as e:
            print("可能是网络故障，请检查网络连接。")
            RPC.is_conect = False
        except Exception as e:
            print("An error occurred during XML-RPC call:", e)
            RPC.is_conect = False
        finally:
            # 恢复默认超时时间
            self.robot = None
            socket.setdefaulttimeout(None)
            self.robot = xmlrpc.client.ServerProxy(link)

    def connect_to_robot(self):
        """连接到机器人的实时端口"""
        # print("SDK连接机器人")
        self.sock_cli_state = socket.socket(socket.AF_INET, socket.SOCK_STREAM)#套接字连接机器人20004端口，用于实时更新机器人状态数据
        self.sock_cli_state.settimeout(0.03)  # 设置超时时间为 0.05 秒
        try:
            self.sock_cli_state.connect((self.ip_address, self.ROBOT_REALTIME_PORT))
            self.sock_cli_state_state = True
        except socket.timeout:
            print("连接超时，请检查网络连接。")
            self.sock_cli_state_state = False
            return False
        except Exception as ex:
            self.sock_cli_state_state = False
            print("SDK连接机器人实时端口失败", ex)
            return False
        return True

    def reconnect(self):
        """自动重连"""
        max_retries = 1000
        retry_interval = 2  # 2秒
        # with self.lock:  # 加锁
        # RPC.is_conect = False
        # print("断联")
        self.reconnect_flag = True
        for attempt in range(max_retries):
            # print(f"尝试重新连接，第 {attempt + 1} 次")
            # print(f"尝试重新连接")
            # 确保 self.sock_cli_state 是新的 socket 对象
            if self.sock_cli_state:
                self.sock_cli_state.close()  # 关闭旧的 socket
                self.sock_cli_state = None  # 重置为 None
            # 重新初始化 XML-RPC 连接
            # self.robot = None
            # link = 'http://' + self.ip_address + ":20003"
            # self.robot = xmlrpc.client.ServerProxy(link)
            # 尝试连接

            if self.connect_to_robot():
                # print("重新连接成功")
                self.SDK_state = True
                self.reconnect_flag = False
                return True
                # 验证 XML-RPC 连接
                # try:
                #     time.sleep(1)
                #     self.Mode(0)  # 调用一个简单的 XML-RPC 方法
                #     time.sleep(1)
                #     self.Mode(1)  # 调用一个简单的 XML-RPC 方法
                #     time.sleep(1)
                #     # self.robot.Mode(0)  # 调用一个简单的 XML-RPC 方法
                #     # time.sleep(1)
                #     # self.robot.Mode(1)  # 调用一个简单的 XML-RPC 方法
                #     # time.sleep(1)
                #     # self.Mode(0)  # 调用一个简单的 XML-RPC 方法
                #     print("XML-RPC 连接验证成功")
                #     self.reconnect_flag = False
                #     # RPC.is_conect = True
                #     return True
                # except Exception as ex:
                #     print("XML-RPC 连接验证失败:", ex)
                #     self.SDK_state = False
            else:
                print(f"重新连接失败，等待 {retry_interval} 秒后重试...")
                time.sleep(retry_interval)

        print("已达到最大重连次数，连接失败")
        self.SDK_state = False
        return False
        #
        # print("自动重连机制")
        # for i in range(1,6):
        #     print("---")
        #     time.sleep(2)
        #     try:
        #         self.sock_cli_state = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #         self.sock_cli_state.connect((self.ip_address, self.ROBOT_REALTIME_PORT))
        #         self.sock_cli_state_state = True
        #     except Exception as ex:
        #         self.sock_cli_state_state = False
        #         print("SDK连接机器人实时端口失败", ex)
        #     if self.sock_cli_state_state:
        #         # self.sock_cli_state_state = True
        #         return


    def robot_state_routine_thread_old(self):
        """处理机器人状态数据包的线程例程"""

        while not self.closeRPC_state:
            recvbuf = bytearray(self.BUFFER_SIZE)
            tmp_recvbuf = bytearray(self.BUFFER_SIZE)
            state_pkg = bytearray(self.BUFFER_SIZE)
            find_head_flag = False
            index = 0
            length = 0
            tmp_len = 0
            # if not self.sock_cli_state_state:
            #     if not self.connect_to_robot():
            #         return


            try:
                # while not self.robot_realstate_exit:
                while not self.robot_realstate_exit and not self.stop_event.is_set():
                    recvbyte = self.sock_cli_state.recv_into(recvbuf)
                    # timestamp_ms = int(datetime.now().timestamp() * 1000)
                    # print("当前时间戳（毫秒级）:", timestamp_ms)
                    if recvbyte <= 0:
                        self.sock_cli_state.close()
                        print("接收机器人状态字节 -1")
                        # self.reconnect()
                        return
                    else:
                        if tmp_len > 0:
                            if tmp_len + recvbyte <= self.BUFFER_SIZE:
                                recvbuf = tmp_recvbuf[:tmp_len] + recvbuf[:recvbyte]
                                recvbyte += tmp_len
                                tmp_len = 0
                            else:
                                tmp_len = 0

                        for i in range(recvbyte):
                            if format(recvbuf[i], '02X') == "5A" and not find_head_flag:
                                if i + 4 < recvbyte:
                                    if format(recvbuf[i+1], '02X') == "5A":
                                        find_head_flag = True
                                        state_pkg[0] = recvbuf[i]
                                        index += 1
                                        length = length | recvbuf[i + 4]
                                        length = length << 8
                                        length = length | recvbuf[i + 3]
                                    else:
                                        continue
                                else:
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    break
                            elif find_head_flag and index < length + 5:
                                state_pkg[index] = recvbuf[i]
                                index += 1
                            elif find_head_flag and index >= length + 5:
                                if i + 1 < recvbyte:
                                    checksum = sum(state_pkg[:index])
                                    checkdata = 0
                                    checkdata = checkdata | recvbuf[i + 1]
                                    checkdata = checkdata << 8
                                    checkdata = checkdata | recvbuf[i]

                                    if checksum == checkdata:
                                        self.robot_state_pkg = RobotStatePkg.from_buffer_copy(recvbuf)
                                        find_head_flag = False
                                        index = 0
                                        length = 0
                                        i += 1
                                    else:
                                        # print(checksum,":",checkdata,"===========================")
                                        self.robot_state_pkg.jt_cur_pos[0] = 0
                                        self.robot_state_pkg.jt_cur_pos[1] = 0
                                        self.robot_state_pkg.jt_cur_pos[2] = 0
                                        find_head_flag = False
                                        index = 0
                                        length = 0
                                        i += 1
                                else:
                                    print("4.2")
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    break
                            else:
                                continue
            except Exception as ex:
                if not self.closeRPC_state:
                    self.sock_cli_state.close()
                    self.sock_cli_state_state = False
                    self.SDK_state=False
                    # self.reconnect()
                    # print("SDK读取机器人实时数据失败", ex)
                    self.reconnect()

    def robot_state_routine_thread(self):
        """处理机器人状态数据包的线程例程"""

        while not self.closeRPC_state:
            recvbuf = bytearray(self.BUFFER_SIZE)
            tmp_recvbuf = bytearray(self.BUFFER_SIZE)
            state_pkg = bytearray(self.BUFFER_SIZE)
            find_head_flag = False
            index = 0
            length = 0
            tmp_len = 0
            expected_length = self.BUFFER_SIZE  # 初始期望接收长度

            try:
                while not self.robot_realstate_exit and not self.stop_event.is_set():
                    recvbyte = self.sock_cli_state.recv_into(recvbuf)

                    if recvbyte <= 0:
                        self.sock_cli_state.close()
                        print("接收机器人状态字节 -1")
                        if not self.reconnect():
                            return
                        continue

                    # 处理临时缓冲区数据
                    if tmp_len > 0:
                        if tmp_len + recvbyte <= self.BUFFER_SIZE:
                            recvbuf[:tmp_len + recvbyte] = tmp_recvbuf[:tmp_len] + recvbuf[:recvbyte]
                            recvbyte += tmp_len
                            tmp_len = 0
                        else:
                            tmp_len = 0

                    i = 0
                    while i < recvbyte:
                        # 查找包头
                        if format(recvbuf[i], '02X') == "5A" and not find_head_flag:
                            if i + 4 < recvbyte and format(recvbuf[i + 1], '02X') == "5A":
                                find_head_flag = True
                                state_pkg[0] = recvbuf[i]
                                index = 1
                                length = (recvbuf[i + 4] << 8) | recvbuf[i + 3]

                                # 检查长度是否超过预期
                                if length + 7 > expected_length:
                                    expected_length = length + 7
                                    # 需要接收更多数据
                                    tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                    tmp_len = recvbyte - i
                                    find_head_flag = False
                                    break

                                i += 1
                            else:
                                i += 1
                                continue

                        # 已找到包头，收集数据
                        elif find_head_flag and index < length + 5:
                            if i >= recvbyte:
                                break

                            state_pkg[index] = recvbuf[i]
                            index += 1
                            i += 1

                        # 检查校验和
                        elif find_head_flag and index >= length + 5:
                            if i + 1 < recvbyte:
                                checksum = sum(state_pkg[:index])
                                checkdata = (recvbuf[i + 1] << 8) | recvbuf[i]

                                if checksum == checkdata:
                                    self.robot_state_pkg = RobotStatePkg.from_buffer_copy(state_pkg[:index])
                                    find_head_flag = False
                                    index = 0
                                    length = 0
                                    expected_length = self.BUFFER_SIZE  # 重置期望长度
                                    i += 2
                                else:
                                    # 校验失败处理
                                    self.robot_state_pkg.jt_cur_pos[0] = 0
                                    self.robot_state_pkg.jt_cur_pos[1] = 0
                                    self.robot_state_pkg.jt_cur_pos[2] = 0
                                    find_head_flag = False
                                    index = 0
                                    length = 0
                                    i += 2
                            else:
                                # 数据不足，保存到临时缓冲区
                                tmp_recvbuf[:recvbyte - i] = recvbuf[i:recvbyte]
                                tmp_len = recvbyte - i
                                break
                        else:
                            i += 1

            except Exception as ex:
                if not self.closeRPC_state:
                    self.sock_cli_state.close()
                    self.sock_cli_state_state = False
                    self.SDK_state = False
                    # print("SDK读取机器人实时数据失败", ex)
                    self.reconnect()

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
        error = 0
        sdk = ["SDK:V2.1.2", "Robot:V3.8.2"]
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
    @brief  关节空间运动
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
    @brief  笛卡尔空间直线运动
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 暂不开放 默认0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
    @param  [in] 默认参数 blendMode 过渡方式；0-内切过渡；1-角点过渡
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 search:[0]-不焊丝寻位，[1]-焊丝寻位
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 overSpeedStrategy  超速处理策略，0-策略关闭；1-标准；2-超速时报错停止；3-自适应降速，默认为0
    @param  [in] 默认参数 speedPercent  允许降速阈值百分比[0-100]，默认10%
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def MoveL(self, desc_pos, tool, user, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0, ovl=100.0,
              blendR=-1.0, blendMode = 0,exaxis_pos=[0.0, 0.0, 0.0, 0.0], search=0, offset_flag=0,
              offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],overSpeedStrategy=0,speedPercent=10):
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
        overSpeedStrategy = int(overSpeedStrategy)
        speedPercent = int(speedPercent)
        if (overSpeedStrategy > 0):
            error = self.robot.JointOverSpeedProtectStart(overSpeedStrategy, speedPercent)
            if error!=0:
                return error
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, -1)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error1 = ret[0]
                return error1

        flag = True
        while flag:
            try:
                error1 = self.robot.MoveL(joint_pos, desc_pos, tool, user, vel, acc, ovl, blendR, blendMode, exaxis_pos, search,offset_flag, offset_pos)
                flag = False
            except socket.error as e:
                flag = True

        if (overSpeedStrategy > 0):
            error = self.robot.JointOverSpeedProtectEnd()
            if error!=0:
                return error

        return error1

    """   
    @brief  笛卡尔空间圆弧运动
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
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 blendR:[-1.0]-运动到位 (阻塞)，[0~1000]-平滑半径 (非阻塞)，单位 [mm] 默认-1.0
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
              ovl=100.0, blendR=-1.0):
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

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, -1)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, -1)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
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
    @brief  笛卡尔空间整圆运动
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
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 offset_flag: 是否偏移[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @param  [in] 必选参数 oacc: 加速度百分比
    @param  [in] 必选参数 blendR: -1：阻塞；0~1000：平滑半径
    @return 错误码 成功-0  失败-错误码
    """

    # @log_call
    # @xmlrpc_timeout
    # def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, oacc, blendR, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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
    #     offset_flag = float(int(offset_flag))
    #     offset_pos = list(map(float, offset_pos))
    #
    #     oacc = float(oacc)
    #     blendR = float(blendR)
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
    #             error = self.robot.Circle(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p, joint_pos_t,
    #                                       desc_pos_t,
    #                                       [tool_t, user_t, vel_t, acc_t], exaxis_pos_t, [ovl, offset_flag], offset_pos, [oacc, blendR])
    #             flag = False
    #         except socket.error as e:
    #             flag = True
    #     return error

    @log_call
    @xmlrpc_timeout
    def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               joint_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               vel_p=20.0, acc_p=0.0, exaxis_pos_p=[0.0, 0.0, 0.0, 0.0], vel_t=20.0, acc_t=0.0,
               exaxis_pos_t=[0.0, 0.0, 0.0, 0.0],
               ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
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

        desc_pos_t = list(map(float, desc_pos_t))
        tool_t = float(int(tool_t))
        user_t = float(int(user_t))
        joint_pos_t = list(map(float, joint_pos_t))
        vel_t = float(vel_t)
        acc_t = float(acc_t)
        exaxis_pos_t = list(map(float, exaxis_pos_t))

        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, -1)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, -1)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error

        flag = True
        while flag:
            try:
                error = self.robot.Circle(joint_pos_p, desc_pos_p, [tool_p, user_p, vel_p, acc_p], exaxis_pos_p,
                                          joint_pos_t,
                                          desc_pos_t,
                                          [tool_t, user_t, vel_t, acc_t], exaxis_pos_t, ovl, offset_flag, offset_pos)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  笛卡尔空间螺旋线运动
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 param:[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction]circle_num: 螺旋圈数，circle_angle: 螺旋倾角，
    rad_init: 螺旋初始半径，rad_add: 半径增量，rotaxis_add: 转轴方向增量，rot_direction: 旋转方向，0-顺时针，1-逆时针
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel：速度百分比，[0~100] 默认20.0
    @param  [in] 默认参数 acc：加速度百分比，[0~100] 默认100.0
    @param  [in] 默认参数 exaxis_pos: 外部轴 1 位置 ~ 外部轴 4 位置 默认[0.0,0.0,0.0,0.0]
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认100.0
    @param  [in] 默认参数 offset_flag:[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0
    @param  [in] 默认参数 offset_pos: 位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSpiral(self, desc_pos, tool, user, param, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=20.0, acc=0.0,
                  exaxis_pos=[0.0, 0.0, 0.0, 0.0],
                  ovl=100.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        desc_pos = list(map(float, desc_pos))
        tool = int(tool)
        user = int(user)
        param[0] = float(param[0])
        param[1] = float(param[1])
        param[2] = float(param[2])
        param[3] = float(param[3])
        param[4] = float(param[4])
        param[5] = float(param[5])
        joint_pos = list(map(float, joint_pos))
        vel = float(vel)
        acc = float(acc)
        exaxis_pos = list(map(float, exaxis_pos))
        ovl = float(ovl)
        offset_flag = int(offset_flag)
        offset_pos = list(map(float, offset_pos))

        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, -1)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        flag = True
        while flag:
            try:
                error = self.robot.NewSpiral(joint_pos, desc_pos, tool, user, vel, acc, exaxis_pos, ovl, offset_flag,
                                             offset_pos, param)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  伺服运动开始，配合ServoJ、ServoCart指令使用
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoMoveStart()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  伺服运动结束，配合ServoJ、ServoCart指令使用
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoMoveEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoMoveEnd()
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  关节空间伺服模式运动
    @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
    @param  [in] 必选参数 axisPos  外部轴位置,单位mm
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 cmdT: 指令下发周期，单位s，建议范围[0.001~0.0016], 默认为0.008
    @param  [in] 默认参数 filterT: 滤波时间，单位 [s]，暂不开放， 默认为0.0
    @param  [in] 默认参数 gain: 目标位置的比例放大器，暂不开放， 默认为0.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJ(self, joint_pos,axisPos, acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        joint_pos = list(map(float, joint_pos))
        axisPos = list(map(float, axisPos))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        flag = True
        while flag:
            try:
                error = self.robot.ServoJ(joint_pos,axisPos,acc, vel, cmdT, filterT, gain)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  笛卡尔空间伺服模式运动
    @param  [in] 必选参数 mode:[0]-绝对运动 (基坐标系)，[1]-增量运动 (基坐标系)，[2]-增量运动(工具坐标系)
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位置/目标笛卡尔位置增量
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
    def ServoCart(self, mode, desc_pos, pos_gain=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], acc=0.0, vel=0.0, cmdT=0.008,
                  filterT=0.0, gain=0.0):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        mode = int(mode)
        desc_pos = list(map(float, desc_pos))
        pos_gain = list(map(float, pos_gain))
        acc = float(acc)
        vel = float(vel)
        cmdT = float(cmdT)
        filterT = float(filterT)
        gain = float(gain)
        flag = True
        while flag:
            try:
                error = self.robot.ServoCart(mode, desc_pos, pos_gain, acc, vel, cmdT, filterT, gain)
                flag = False
            except socket.error as e:
                flag = True
        return error

    """   
    @brief  关节扭矩控制开始
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJTStart(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoJTStart()
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  关节扭矩控制
    @param  [in] 必选参数 torque j1~j6关节扭矩，单位Nm
    @param  [in] 必选参数 interval 指令周期，单位s，范围[0.001~0.008]
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJT(self, torque, interval):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        torque = list(map(float, torque))
        interval = float(interval)
        flag = True
        while flag:
            try:
                error = self.robot.ServoJT(torque, interval)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  关节扭矩控制结束
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ServoJTEnd(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.ServoJTEnd()
                flag = False
            except socket.error as e:
                flag = True

        return error

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
    @brief  样条运动 PTP
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
    @brief  新样条指令点
    @param  [in] 必选参数 desc_pos: 目标笛卡尔位姿，单位 [mm][°]
    @param  [in] 必选参数 tool: 工具号，[0~14]
    @param  [in] 必选参数 user: 工件号，[0~14]
    @param  [in] 必选参数 lastFlag: 是否为最后一个点，0-否，1-是
    @param  [in] 默认参数 joint_pos: 目标关节位置，单位 [°] 默认初值为[0.0,0.0,0.0,0.0,0.0,0.0]，默认值调用逆运动学求解返回值
    @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
    @param  [in] 默认参数 ovl: 速度缩放因子，[0~100] 默认为 100.0
    @param  [in] 默认参数 blendR: [0~1000]-平滑半径，单位 [mm] 默认0.0
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def NewSplinePoint(self, desc_pos, tool, user, lastFlag, joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=0.0,
                       acc=0.0, ovl=100.0, blendR=0.0):
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
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, -1)  # 逆运动学求解
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
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoadCoord(self, x, y, z):
        while self.reconnect_flag:
            time.sleep(0.1)
        x = float(x)
        y = float(y)
        z = float(z)
        flag = True
        while flag:
            try:
                error = self.robot.SetLoadCoord(x, y, z)
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
        flag = int(flag)
        # _error = self.robot.GetActualJointPosDegree(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.jt_cur_pos[0],self.robot_state_pkg.jt_cur_pos[1],self.robot_state_pkg.jt_cur_pos[2],
                  self.robot_state_pkg.jt_cur_pos[3],self.robot_state_pkg.jt_cur_pos[4],self.robot_state_pkg.jt_cur_pos[5]]
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
        flag = int(flag)
        # _error = self.robot.GetActualTCPPose(flag)
        # error = _error[0]
        # if error == 0:
        #     return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
        # else:
        #     return error
        return 0,[self.robot_state_pkg.tl_cur_pos[0],self.robot_state_pkg.tl_cur_pos[1],self.robot_state_pkg.tl_cur_pos[2],
                  self.robot_state_pkg.tl_cur_pos[3],self.robot_state_pkg.tl_cur_pos[4],self.robot_state_pkg.tl_cur_pos[5]]

    """   
    @brief  获取当前工具坐标系编号
    @param  [in] 默认参数 flag：0-阻塞，1-非阻塞 默认1
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回） tool_id:工具坐标系编号
    """

    @log_call
    @xmlrpc_timeout
    def GetActualTCPNum(self, flag=1):
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
        # _error = self.robot.GetRobotMotionDone()
        # error = _error[0]
        # if error == 0:
        #     return error, _error[1]
        # else:
        #     return error
            return 0,self.robot_state_pkg.motion_done
    """   
    @brief  查询机器人错误码
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[maincode subcode] maincode 主错误码 subcode 子错误码
    """

    @log_call
    @xmlrpc_timeout
    def GetRobotErrorCode(self):
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
    def SetTrajectoryJSpeed(self, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.SetTrajectoryJSpeed(ovl)
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
    @param  [in] 必选参数 sensor_num：力传感器编号
    @param  [in] 必选参数 select：[fx,fy,fz,mx,my,mz]六个自由度是否检测碰撞 ，0-不生效，1-生效
    @param  [in] 必选参数 force_torque：[fx,fy,fz,mx,my,mz]碰撞检测力/力矩，单位 N 或 Nm
    @param  [in] 必选参数 gain：[f_p,f_i,f_d,m_p,m_i,m_d], 力PID参数，力矩PID参数
    @param  [in] 必选参数 adj_sign：自适应启停状态，0-关闭，1-开启
    @param  [in] 必选参数 ILC_sign: ILC 控制启停状态，0-停止，1-训练，2-实操
    @param  [in] 必选参数 max_dis：最大调整距离，单位mm
    @param  [in] 必选参数 max_ang：最大调整角度，单位deg
    @param  [in] 必选参数 r：打磨盘半径，单位mm
    @param  [in] 默认参数 filter_Sign 滤波开启标志 0-关；1-开，默认 0-关闭
    @param  [in] 默认参数 posAdapt_sign 姿态顺应开启标志 0-关；1-开，默认 0-关闭
    @param  [in] 默认参数 isNoBlock 阻塞标志，0-阻塞；1-非阻塞 默认0-阻塞
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_Control(self, flag, sensor_num, select, force_torque, gain, adj_sign, ILC_sign, max_dis, max_ang,filter_Sign=0, posAdapt_sign=0,isNoBlock=0):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = int(flag)
        sensor_num = int(sensor_num)
        select = list(map(int, select))
        force_torque = list(map(float, force_torque))
        gain = list(map(float, gain))
        adj_sign = int(adj_sign)
        ILC_sign = int(ILC_sign)
        max_dis = float(max_dis)
        max_ang = float(max_ang)

        filter_Sign = int(filter_Sign)
        posAdapt_sign = int(posAdapt_sign)
        isNoBlock = int(isNoBlock)
        flag_tmp = True
        while flag_tmp:
            try:
                error = self.robot.FT_Control(flag, sensor_num, select, force_torque, gain, adj_sign, ILC_sign, max_dis,
                                              max_ang, filter_Sign, posAdapt_sign,isNoBlock)
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
    @param  [in] 必选参数 ft：力或力矩阈值 (0~100)，单位 N 或 Nm
    @param  [in] 必选参数 orn 力/扭矩方向，1-沿z轴方向，2-绕z轴方向
    @param  [in] 默认参数 angVelRot：旋转角速度，单位 °/s  默认 3
    @param  [in] 默认参数 angleMax：最大旋转角度，单位 ° 默认 45
    @param  [in] 默认参数 angAccmax：最大旋转加速度，单位 °/s^2，暂不使用 默认0
    @param  [in] 默认参数 rotorn：旋转方向，1-顺时针，2-逆时针 默认1
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def FT_RotInsertion(self, rcs, ft, orn, angVelRot=3, angleMax=45, angAccmax=0, rotorn=1):
        while self.reconnect_flag:
            time.sleep(0.1)
        rcs = int(rcs)
        ft = float(ft)
        orn = int(orn)
        angVelRot = float(angVelRot)
        angleMax = float(angleMax)
        angAccmax = float(angAccmax)
        rotorn = int(rotorn)
        flag = True
        while flag:
            try:
                error = self.robot.FT_RotInsertion(rcs, angVelRot, ft, angleMax, orn, angAccmax, rotorn)
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
                    默认执行程序所在路径，默认名称fairino_ year+month+data.log(如:fairino_2024_03_13.log);
    @param  [in]默认参数 file_num：滚动存储的文件数量，1~20个，默认值为5。单个文件上限50M;
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def LoggerInit(self, output_model=1, file_path="", file_num=5):
        return self.setup_logging(output_model, file_path, file_num)

    """   
    @brief  设置日志过滤等级
    @param  [in] 默认参数 lvl: 过滤等级值，值越小输出日志越少, 1-error, 2-warnning, 3-inform, 4-debug,默认值是1.
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def SetLoggerLevel(self, lvl=1):
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
        if not os.path.exists(save_file_path):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND

        rtn = self.robot.PointTableDownload(point_table_name)
        if rtn == -1:
            return RobotError.ERR_POINTTABLE_NOTFOUND
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
        rtn = self.robot.PointTableSwitch(point_table_name)  # 切换点位表
        if rtn != 0:
            if rtn == RobotError.ERR_POINTTABLE_NOTFOUND:
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
        try:

            rtn = self.robot.PointTableSwitch(point_table_name)  # 切换点位表
            if rtn != 0:
                if rtn == RobotError.ERR_POINTTABLE_NOTFOUND:
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
        if not os.path.exists(saveFilePath):
            return RobotError.ERR_SAVE_FILE_PATH_NOT_FOUND
        rtn = self.robot.FileDownload(fileType, fileName)
        if rtn == -1:
            return RobotError.ERR_POINTTABLE_NOTFOUND
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

        if not os.path.exists(filePath):
            return RobotError.ERR_POINTTABLE_NOTFOUND

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
                               _error[9]]
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
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisMove(self, pos, ovl):
        while self.reconnect_flag:
            time.sleep(0.1)
        if self.GetSafetyCode() != 0:
            return self.GetSafetyCode()
        pos = list(map(float, pos))
        ovl = float(ovl)
        flag = True
        while flag:
            try:
                error = self.robot.ExtAxisMoveJ(0, pos[0], pos[1], pos[2], pos[3], ovl)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴与机器人关节运动同步运动
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
    def ExtAxisSyncMoveJ(self, joint_pos, desc_pos, tool, user, exaxis_pos, vel=20.0, acc=0.0, ovl=100.0,
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
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos[0], exaxis_pos[1], exaxis_pos[2], exaxis_pos[3], ovl)
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
    @brief  UDP扩展轴与机器人直线运动同步运动
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
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveL(self, joint_pos, desc_pos, tool, user, exaxis_pos, vel=20.0, acc=0.0, ovl=100.0,
                         blendR=-1.0, search=0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
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
        if ((joint_pos[0] == 0.0) and (joint_pos[1] == 0.0) and (joint_pos[2] == 0.0) and (joint_pos[3] == 0.0)
                and (joint_pos[4] == 0.0) and (joint_pos[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            ret = self.robot.GetInverseKin(0, desc_pos, -1)  # 逆运动学求解
            if ret[0] == 0:
                joint_pos = [ret[1], ret[2], ret[3], ret[4], ret[5], ret[6]]
            else:
                error = ret[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos[0], exaxis_pos[1], exaxis_pos[2], exaxis_pos[3], ovl)
        if error != 0:
            return error
        flag = True
        while flag:
            try:
                error = self.robot.MoveL(joint_pos, desc_pos, tool, user, vel, acc, ovl, blendR, exaxis_pos, search,
                                         offset_flag, offset_pos)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  UDP扩展轴与机器人圆弧运动同步运动
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
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def ExtAxisSyncMoveC(self, joint_pos_p, desc_pos_p, tool_p, user_p, exaxis_pos_p, joint_pos_t, desc_pos_t, tool_t,
                         user_t, exaxis_pos_t,
                         vel_p=20.0, acc_p=100.0, offset_flag_p=0,
                         offset_pos_p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         vel_t=20.0, acc_t=100.0, offset_flag_t=0,
                         offset_pos_t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         ovl=100.0, blendR=-1.0):
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

        if ((joint_pos_p[0] == 0.0) and (joint_pos_p[1] == 0.0) and (joint_pos_p[2] == 0.0) and (joint_pos_p[3] == 0.0)
                and (joint_pos_p[4] == 0.0) and (joint_pos_p[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            retp = self.robot.GetInverseKin(0, desc_pos_p, -1)  # 逆运动学求解
            if retp[0] == 0:
                joint_pos_p = [retp[1], retp[2], retp[3], retp[4], retp[5], retp[6]]
            else:
                error = retp[0]
                return error

        if ((joint_pos_t[0] == 0.0) and (joint_pos_t[1] == 0.0) and (joint_pos_t[2] == 0.0) and (joint_pos_t[3] == 0.0)
                and (joint_pos_t[4] == 0.0) and (joint_pos_t[5] == 0.0)):  # 若未输入参数则调用逆运动学求解
            rett = self.robot.GetInverseKin(0, desc_pos_t, -1)  # 逆运动学求解
            if rett[0] == 0:
                joint_pos_t = [rett[1], rett[2], rett[3], rett[4], rett[5], rett[6]]
            else:
                error = rett[0]
                return error
        error = self.robot.ExtAxisMoveJ(1, exaxis_pos_t[0], exaxis_pos_t[1], exaxis_pos_t[2], exaxis_pos_t[3], ovl)
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
    def EndForceDragControl(self, status, asaptiveFlag, interfereDragFlag, ingularityConstraintsFlag, M, B, K, F, Fmax, Vmax):
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
        flag = True
        while flag:
            try:
                error = self.robot.EndForceDragControl(status, asaptiveFlag, interfereDragFlag, ingularityConstraintsFlag, M, B, K, F, Fmax, Vmax)
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
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetCtlBoxDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetCtlBoxDO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置控制箱AO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetCtlBoxAO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetCtlBoxAO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置末端工具DO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetAxleDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetAxleDO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置末端工具AO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetAxleAO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetAxleAO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展DO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetExtDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetExtDO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置扩展AO停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetExtAO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetExtAO(resetFlag)
                flag = False
            except socket.error as e:
                flag = True

        return error

    """   
    @brief  设置SmartTool停止/暂停后输出是否复位
    @param  [in]必选参数 resetFlag  0-不复位；1-复位
    @return 错误码 成功- 0, 失败-错误码    
    """

    @log_call
    @xmlrpc_timeout
    def SetOutputResetSmartToolDO(self,resetFlag):
        while self.reconnect_flag:
            time.sleep(0.1)
        resetFlag = int(resetFlag)
        flag = True
        while flag:
            try:
                error = self.robot.SetOutputResetSmartToolDO(resetFlag)
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
    @brief  获取当前配置的控制器外设协议LUA文件名
    @return 错误码 成功- 0, 失败-错误码     
    @return 返回值（调用成功返回） name[4] lua文件名称 “CTRL_LUA_test.lua”
    """
    @log_call
    @xmlrpc_timeout
    def GetCtrlOpenLUAName(self):
        while self.reconnect_flag:
            time.sleep(0.1)
        flag = True
        while flag:
            try:
                error = self.robot.GetCtrlOpenLUAName()
                flag = False
            except socket.error as e:
                flag = True

        if error[0] == 0:
            par = error[2].split(',')
            if 4 != sizeof(par):
                self.log_error("GetCtrlOpenLUAName fail")
                return -1
            else:
                return error[0], [error[1], error[2], error[3], error[4]]
        else:
            return error

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
    @param [in] mode 焊机控制模式;0-一元化
    @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def SetWeldMachineCtrlMode(self, mode):
        while self.reconnect_flag:
            time.sleep(0.1)
        mode = int(mode)
        flag = True
        while flag:
            try:
                error = self.robot.SetWeldMachineCtrlMode(mode)
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
        # 设置停止事件以通知线程停止
        self.stop_event.set()

        # 如果线程仍在运行，则等待其结束
        # if self.thread.is_alive():
        #     self.thread.join()

        # 清理 XML-RPC 代理
        if self.robot is not None:
            self.robot = None  # 将代理设置为 None，释放资源
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
                error = self.robot.LaserTrackingSearchStart(direction, directionPoint, vel, distance, timeout, posSensorNum)
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
       @return 错误码 成功- 0, 失败-错误码
    """

    @log_call
    @xmlrpc_timeout

    def LoadTrajectoryLA(self, name, mode, errorLim, type, precision, vamx, amax, jmax):
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
        flag = True
        while flag:
            try:
                error = self.robot.LoadTrajectoryLA(name, mode, errorLim, type, precision, vamx, amax, jmax)
                flag = False
            except socket.error as e:
                flag = True

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
        return 0,self.robot_state_pkg

    """   
    @brief  停止运动
    @param  [in] NULL
    @return 错误码 成功-0  失败-错误码
    """

    @log_call
    @xmlrpc_timeout
    def StopMove(self):
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
        saveFlag_flag = 0 if saveFlag else 1
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
        saveFlag_flag = 0 if saveFlag else 1
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
    @return 返回值（调用成功返回）[fault,status]：夹爪激活状态，fault:0-无错误，1-有错误；status:bit0~bit15对应夹爪编号0~15，bit=0为未激活，bit=1为激活    
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperActivateStatus(self):
        return 0, [self.robot_state_pkg.gripper_fault,self.robot_state_pkg.gripper_active]

    """   
    @brief  获取夹爪位置
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[fault,position]：夹爪激活状态，fault:0-无错误，1-有错误；position:位置百分比，范围0~100% 
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurPosition(self):
        return 0, [self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_position]

    """   
    @brief  获取夹爪电流
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[fault,current]：夹爪激活状态，fault:0-无错误，1-有错误；current:电流百分比，范围0~100%
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurCurrent(self):
        return 0, [self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_current]

    """   
    @brief  获取夹爪电压
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[fault, voltage]：夹爪激活状态，fault:0-无错误，1-有错误； voltage:电压,单位0.1V
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperVoltage(self):
        return 0, [self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_voltage]

    """   
    @brief  获取夹爪温度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[fault, temp]：夹爪激活状态，fault:0-无错误，1-有错误； temp:温度，单位℃
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperTemp(self):
        return 0, [self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_tmp]

    """   
    @brief  获取夹爪速度
    @param  [in] NULL
    @return 错误码 成功- 0, 失败-错误码
    @return 返回值（调用成功返回）[fault, speed]：夹爪激活状态，fault:0-无错误，1-有错误； speed:速度
    """

    @log_call
    @xmlrpc_timeout
    def GetGripperCurSpeed(self):
        return 0, [self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripper_speed]
