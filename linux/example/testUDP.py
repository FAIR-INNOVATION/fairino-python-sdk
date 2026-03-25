from time import sleep
from fairino import Robot
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')


def TestUDP():
    # 设置简单的回调
    def callback(src_type, count, cmd_id, data_len, content):
        print("收到回复: cmd_id={} count={} data_len={} content={}".format(cmd_id, count, data_len, content))
        return 0

    robot.set_udp_cmd_rpy_callback(callback)

    # ========== 测试1: 直接发送UDP数据 ==========
    # print("\n=== 测试1: 直接发送UDP数据 ===")
    # test_data = "/f/bIII2III376III75IIIServoJ(-110.8788,-208.2472,77.6644,-49.4176,200.8790,135.0003,0,0,0.01,0,0)III/b/f"
    # print("发送数据: {}".format(test_data[:50] + "..."))
    # success = robot.send_udp_data(test_data)
    # print("发送结果: {}".format(success))
    # time.sleep(2)

    # ========== 测试2: ServoMoveStart UDP方式 ==========
    print("\n=== 测试2: ServoMoveStart UDP方式 ===")
    result = robot.ServoMoveStart(cmdType=2)
    print("ServoMoveStart返回结果: {}".format(result))
    time.sleep(1)

    # ========== 测试3: ServoJ UDP方式循环发送 ==========
    print("\n=== 测试3: ServoJ UDP方式循环5次 ===")
    joint_pos = [-110.8788, -208.2472, 77.6644, -49.4176, 200.8790, 135.0003]
    axisPos = [0, 0, 0, 0, 0, 0]

    for i in range(5):
        print("第{}次ServoJ UDP调用".format(i + 1))
        result = robot.ServoJ(
            joint_pos=joint_pos,
            axisPos=axisPos,
            acc=0.01,
            vel=0.0,
            cmdT=0.008,
            filterT=0.0,
            gain=0.0,
            id=0,
            cmdType=2
        )
        print("返回结果: {}".format(result))
        time.sleep(1)

    # ========== 测试4: ServoMoveEnd UDP方式 ==========
    print("\n=== 测试4: ServoMoveEnd UDP方式 ===")
    result = robot.ServoMoveEnd(cmdType=2)
    print("ServoMoveEnd返回结果: {}".format(result))
    time.sleep(1)

    # # ========== 测试5: ServoJTStart UDP方式 ==========
    # print("\n=== 测试5: ServoJTStart UDP方式 ===")
    # result = robot.ServoJTStart(cmdType=1)
    # print("ServoJTStart返回结果: {}".format(result))
    # time.sleep(1)
    #
    # # ========== 测试6: ServoJT UDP方式 ==========
    # print("\n=== 测试6: ServoJT UDP方式 ===")
    # torque = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # 力矩值
    # interval = 0.008  # 间隔时间
    #
    # result = robot.ServoJT(
    #     torque=torque,
    #     interval=interval,
    #     checkFlag=0,
    #     jPowerLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #     jVelLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    #     cmdType=1
    # )
    # print("ServoJT返回结果: {}".format(result))
    # time.sleep(1)
    #
    # # ========== 测试7: ServoJTEnd UDP方式 ==========
    # print("\n=== 测试7: ServoJTEnd UDP方式 ===")
    # result = robot.ServoJTEnd(cmdType=1)
    # print("ServoJTEnd返回结果: {}".format(result))
    # time.sleep(1)
    #
    # # ========== 测试8: 组合测试 - 完整的力矩控制流程 ==========
    # print("\n=== 测试8: 完整的力矩控制流程 ===")
    #
    # # 8.1 开始力矩控制
    # print("8.1 ServoJTStart")
    # robot.ServoJTStart(cmdType=1)
    # time.sleep(1)
    #
    # # 8.2 发送多次力矩指令
    # for i in range(3):
    #     print("8.2 第{}次ServoJT".format(i + 1))
    #     robot.ServoJT(
    #         torque=[0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
    #         interval=0.008,
    #         cmdType=1
    #     )
    #     time.sleep(1)
    #
    # # 8.3 结束力矩控制
    # print("8.3 ServoJTEnd")
    # robot.ServoJTEnd(cmdType=1)
    # time.sleep(1)

    # 关闭连接
    robot.CloseRPC()
    print("\n=== 所有测试完成，连接已关闭 ===")


# 调用测试函数
TestUDP()