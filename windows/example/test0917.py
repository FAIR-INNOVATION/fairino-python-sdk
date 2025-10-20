from time import sleep
from fairino import Robot
import fairino
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time
def TestCoord1(self):
    id = 0
    while True:
        weight = 0.0
        cog = [0.0] * 3
        rtn, weight, cog = robot.GetTargetPayloadWithID(id)
        print(f"GetTargetPayloadWithID {id},{weight},{cog[0]},{cog[1]},{cog[2]}")

        id += 1
        if id > 19:
            id = 1
        time.sleep(0.2)

    # toolCoord = [0.0] * 6
    # rtn, toolCoord = robot.GetToolCoordWithID(id)
    # print(f"GetToolCoordWithID {id},{toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
    #
    # wobjCoord = [0.0] * 6
    # rtn, wobjCoord = robot.GetWObjCoordWithID(id)
    # print(f"GetWObjCoordWithID {id},{wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
    #
    # extoolCoord = [0.0] * 6
    # rtn, extoolCoord = robot.GetExToolCoordWithID(id)
    # print(f"GetExToolCoordWithID {id},{extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
    #
    # exAxisCoord = [0.0] * 6
    # rtn, exAxisCoord = robot.GetExAxisCoordWithID(id)
    # print(f"GetExAxisCoordWithID {id},{exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
    #
    # weight = 0.0
    # cog = [0.0] * 3
    # rtn, weight, cog = robot.GetTargetPayloadWithID(id)
    # print(f"GetTargetPayloadWithID {id},{weight},{cog[0]},{cog[1]},{cog[2]}")
    #
    # toolCoord = [0.0] * 6
    # rtn.toolCoord = robot.GetCurToolCoord()
    # print(f"GetCurToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
    #
    # wobjCoord = [0.0] * 6
    # rtn.wobjCoord = robot.GetCurWObjCoord()
    # print(f"GetCurWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
    #
    # extoolCoord = [0.0] * 6
    # rtn.extoolCoord = robot.GetCurExToolCoord()
    # print(f"GetExToolCoordWithID {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
    #
    # exAxisCoord = [0.0] * 6
    # rtn.exAxisCoord = robot.GetCurExAxisCoord()
    # print(f"GetCurExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
    #
    # weightT = 0.0
    # cogT = [0.0] * 3
    # rtn, weightT = robot.GetTargetPayload(0)
    # rtn, cogT = robot.GetTargetPayloadCog(0)
    # print(f"GetTargetPayload {weightT},{cogT[0]},{cogT[1]},{cogT[2]}")
    #
    # coordSet = [0.0, 10.0, 2.0, 3.0, 4.0, 5.0]
    # robot.SetToolCoord(2, coordSet, 0, 0, 1, 0)

    robot.CloseRPC()



def TestCoord(self):
    id = 1
    toolCoord = [0.0] * 6
    extoolCoord = [0.0] * 6
    wobjCoord = [0.0] * 6
    exAxisCoord = [0.0] * 6

    for i in range(100):
        print(f"当前ID为:{id}")
        coordSet0 = [0.0] * 6
        coordSet = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        etcp = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        etool = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        cog = [1.0, 2.0, 3.0]

        # if i % 2 == 0:
        #     robot.SetToolCoord(id, coordSet, 0, 0, 1, 0)
        #     time.sleep(0.1)
        #     rtn, toolCoord = robot.GetCurToolCoord()
        #     print(
        #         f"GetToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
        #     robot.SetWObjCoord(id, coordSet, 0)
        #     time.sleep(0.1)
        #     rtn, wobjCoord = robot.GetCurWObjCoord()
        #     print(
        #         f"GetWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
        #     robot.ExtAxisActiveECoordSys(id, 1, coordSet, 1)  # 将标定结果应用到扩展轴坐标系
        #     time.sleep(0.1)
        #     rtn, exAxisCoord = robot.GetCurExAxisCoord()
        #     print(
        #         f"GetExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
        #     rtn = robot.SetExToolCoord(id, etcp, etool)
        #     time.sleep(0.1)
        #     rtn, extoolCoord = robot.GetCurExToolCoord()
        #     print(
        #         f"GetExToolCoord {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
        #     rtn = robot.SetLoadWeight(id, 1.5)
        #     time.sleep(0.1)
        #     rtn = robot.SetLoadCoord(cog[0], cog[1], cog[2], id)
        #     time.sleep(0.1)
        #     weight = 0.0
        #     getCog = [0.0] * 3
        #     rtn, weight = robot.GetTargetPayload(0)
        #     rtn, getCog = robot.GetTargetPayloadCog(0)
        #     print(f"GetTargetPayload {weight},{getCog[0]},{getCog[1]},{getCog[2]}")
        # else:
        #     robot.SetToolCoord(id, coordSet0, 0, 0, 1, 0)
        #     time.sleep(0.1)
        #     rtn, toolCoord = robot.GetCurToolCoord()
        #     print(
        #         f"GetToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
        #     robot.SetWObjCoord(id, coordSet0, 0)
        #     time.sleep(0.1)
        #     rtn, wobjCoord = robot.GetCurWObjCoord()
        #     print(
        #         f"GetWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
        #     robot.ExtAxisActiveECoordSys(id, 1, coordSet0, 1)  # 将标定结果应用到扩展轴坐标系
        #     time.sleep(0.1)
        #     rtn, exAxisCoord = robot.GetCurExAxisCoord()
        #     print(
        #         f"GetExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
        #     rtn = robot.SetExToolCoord(id, coordSet0, coordSet0)
        #     time.sleep(0.1)
        #     rtn, extoolCoord = robot.GetCurExToolCoord()
        #     print(
        #         f"GetExToolCoord {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
        #     rtn = robot.SetLoadWeight(id, 0)
        #     time.sleep(0.1)
        #     rtn = robot.SetLoadCoord(coordSet0[0], coordSet0[1], coordSet0[2], id)  # 取前3个元素作为坐标
        #     time.sleep(0.1)
        #     weight = 0.0
        #     getCog = [0.0] * 3
        #     rtn, weight = robot.GetTargetPayload(0)
        #     rtn, getCog = robot.GetTargetPayloadCog(0)
        #     print(f"GetTargetPayload {weight},{getCog[0]},{getCog[1]},{getCog[2]}")




        if i % 2 == 0:
            robot.SetToolCoord(id, coordSet, 0, 0, 1, 0)
            time.sleep(0.1)
            robot.SetWObjCoord(id, coordSet, 0)
            time.sleep(0.1)
            robot.ExtAxisActiveECoordSys(id, 1, coordSet, 1)  # 将标定结果应用到扩展轴坐标系
            time.sleep(0.1)
            rtn = robot.SetExToolCoord(id, etcp, etool)
            time.sleep(0.1)
            rtn = robot.SetLoadWeight(id, 1.5)
            time.sleep(0.1)
            rtn = robot.SetLoadCoord(cog[0],cog[1],cog[2],id)
            time.sleep(0.1)
        else:
            robot.SetToolCoord(id, coordSet0, 0, 0, 1, 0)
            time.sleep(0.1)
            robot.SetWObjCoord(id, coordSet0, 0)
            time.sleep(0.1)
            robot.ExtAxisActiveECoordSys(id, 1, coordSet0, 1)  # 将标定结果应用到扩展轴坐标系
            time.sleep(0.1)
            rtn = robot.SetExToolCoord(id, coordSet0, coordSet0)
            time.sleep(0.1)
            rtn = robot.SetLoadWeight(id, 0)
            time.sleep(0.1)
            rtn = robot.SetLoadCoord(coordSet0[0],coordSet0[1],coordSet0[2] , id)  # 取前3个元素作为坐标
            time.sleep(0.1)



        # rtn, toolCoord = robot.GetCurToolCoord()
        # print(
        #     f"GetToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
        #
        # rtn, wobjCoord = robot.GetCurWObjCoord()
        # print(
        #     f"GetWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
        #
        # rtn, extoolCoord = robot.GetCurExToolCoord()
        # print(
        #     f"GetExToolCoord {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
        #
        # rtn, exAxisCoord = robot.GetCurExAxisCoord()
        # print(
        #     f"GetExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
        #
        # weight = 0.0
        # getCog = [0.0] * 3
        # rtn, weight = robot.GetTargetPayload(0)
        # rtn, getCog = robot.GetTargetPayloadCog(0)
        # print(f"GetTargetPayload {weight},{getCog[0]},{getCog[1]},{getCog[2]}")



        # rtn, toolCoord = robot.GetToolCoordWithID(id)
        # print(
        #     f"GetToolCoordWithID {id},{toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
        #
        # rtn, wobjCoord = robot.GetWObjCoordWithID(id)
        # print(
        #     f"GetWObjCoordWithID {id},{wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
        #
        # rtn, extoolCoord = robot.GetExToolCoordWithID(id)
        # print(
        #     f"GetExToolCoordWithID {id},{extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
        #
        # rtn, exAxisCoord = robot.GetExAxisCoordWithID(id)
        # print(
        #     f"GetExAxisCoordWithID {id},{exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
        #
        # weight = 0.0
        # getCog = [0.0] * 3
        # rtn, weight, getCog = robot.GetTargetPayloadWithID(id)
        # print(f"GetTargetPayloadWithID {id},{weight},{getCog[0]},{getCog[1]},{getCog[2]}")
        #
        # if i % 1 == 0 and i > 1:
        #     id = id + 1

        time.sleep(0.5)
        print(f"times {i}")

TestCoord(robot)

# if i % 2 == 0:
#     robot.SetToolCoord(id, coordSet, 0, 0, 1, 0)
#     time.sleep(0.1)
#     rtn, toolCoord = robot.GetCurToolCoord()
#     print(
#         f"GetToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
#     robot.SetWObjCoord(id, coordSet, 0)
#     time.sleep(0.1)
#     rtn, wobjCoord = robot.GetCurWObjCoord()
#     print(
#         f"GetWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
#     robot.ExtAxisActiveECoordSys(id, 1, coordSet, 1)  # 将标定结果应用到扩展轴坐标系
#     time.sleep(0.1)
#     rtn, exAxisCoord = robot.GetCurExAxisCoord()
#     print(
#         f"GetExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
#     rtn = robot.SetExToolCoord(id, etcp, etool)
#     time.sleep(0.1)
#     rtn, extoolCoord = robot.GetCurExToolCoord()
#     print(
#         f"GetExToolCoord {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
#     rtn = robot.SetLoadWeight(id, 1.5)
#     time.sleep(0.1)
#     rtn = robot.SetLoadCoord(cog[0], cog[1], cog[2], id)
#     time.sleep(0.1)
#     weight = 0.0
#     getCog = [0.0] * 3
#     rtn, weight = robot.GetTargetPayload(0)
#     rtn, getCog = robot.GetTargetPayloadCog(0)
#     print(f"GetTargetPayload {weight},{getCog[0]},{getCog[1]},{getCog[2]}")
# else:
#     robot.SetToolCoord(id, coordSet0, 0, 0, 1, 0)
#     time.sleep(0.1)
#     rtn, toolCoord = robot.GetCurToolCoord()
#     print(
#         f"GetToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
#     robot.SetWObjCoord(id, coordSet0, 0)
#     time.sleep(0.1)
#     rtn, wobjCoord = robot.GetCurWObjCoord()
#     print(
#         f"GetWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
#     robot.ExtAxisActiveECoordSys(id, 1, coordSet0, 1)  # 将标定结果应用到扩展轴坐标系
#     time.sleep(0.1)
#     rtn, exAxisCoord = robot.GetCurExAxisCoord()
#     print(
#         f"GetExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
#     rtn = robot.SetExToolCoord(id, coordSet0, coordSet0)
#     time.sleep(0.1)
#     rtn, extoolCoord = robot.GetCurExToolCoord()
#     print(
#         f"GetExToolCoord {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
#     rtn = robot.SetLoadWeight(id, 0)
#     time.sleep(0.1)
#     rtn = robot.SetLoadCoord(coordSet0[0], coordSet0[1], coordSet0[2], id)  # 取前3个元素作为坐标
#     time.sleep(0.1)
#     weight = 0.0
#     getCog = [0.0] * 3
#     rtn, weight = robot.GetTargetPayload(0)
#     rtn, getCog = robot.GetTargetPayloadCog(0)
#     print(f"GetTargetPayload {weight},{getCog[0]},{getCog[1]},{getCog[2]}")




# id = 1
# toolCoord = [0.0] * 6
# extoolCoord = [0.0] * 6
# wobjCoord = [0.0] * 6
# exAxisCoord = [0.0] * 6
# for i in range(100):
#     print(f"当前ID为:{id}")
#     coordSet0 = [0.0] * 6
#     coordSet = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
#     etcp = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
#     etool = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
#     cog = [1.0, 2.0, 3.0]
#     if i % 2 == 0:
#         robot.SetToolCoord(id, coordSet, 0, 0, 1, 0)
#         time.sleep(0.1)
#         robot.SetWObjCoord(id, coordSet, 0)
#         time.sleep(0.1)
#         robot.ExtAxisActiveECoordSys(id, 1, coordSet, 1)
#         time.sleep(0.1)
#         rtn = robot.SetExToolCoord(id, etcp, etool)
#         time.sleep(0.1)
#         rtn = robot.SetLoadWeight(id, 1.5)
#         time.sleep(0.1)
#         rtn = robot.SetLoadCoord(cog[0],cog[1],cog[2],id)
#         time.sleep(0.1)
#     else:
#         robot.SetToolCoord(id, coordSet0, 0, 0, 1, 0)
#         time.sleep(0.1)
#         robot.SetWObjCoord(id, coordSet0, 0)
#         time.sleep(0.1)
#         robot.ExtAxisActiveECoordSys(id, 1, coordSet0, 1)
#         time.sleep(0.1)
#         rtn = robot.SetExToolCoord(id, coordSet0, coordSet0)
#         time.sleep(0.1)
#         rtn = robot.SetLoadWeight(id, 0)
#         time.sleep(0.1)
#         rtn = robot.SetLoadCoord(coordSet0[0],coordSet0[1],coordSet0[2] , id)
#         time.sleep(0.1)
#     rtn, toolCoord = robot.GetCurToolCoord()
#     print(f"GetToolCoord {toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
#     rtn, wobjCoord = robot.GetCurWObjCoord()
#     print(f"GetWObjCoord {wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
#     rtn, extoolCoord = robot.GetCurExToolCoord()
#     print(f"GetExToolCoord {extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
#     rtn, exAxisCoord = robot.GetCurExAxisCoord()
#     print(f"GetExAxisCoord {exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
#     weight = 0.0
#     getCog = [0.0] * 3
#     rtn, weight = robot.GetTargetPayload(0)
#     rtn, getCog = robot.GetTargetPayloadCog(0)
#     print(f"GetTargetPayload {weight},{getCog[0]},{getCog[1]},{getCog[2]}")
#
#     rtn, toolCoord = robot.GetToolCoordWithID(id)
#     print(f"GetToolCoordWithID {id},{toolCoord[0]},{toolCoord[1]},{toolCoord[2]},{toolCoord[3]},{toolCoord[4]},{toolCoord[5]}")
#     rtn, wobjCoord = robot.GetWObjCoordWithID(id)
#     print(f"GetWObjCoordWithID {id},{wobjCoord[0]},{wobjCoord[1]},{wobjCoord[2]},{wobjCoord[3]},{wobjCoord[4]},{wobjCoord[5]}")
#     rtn, extoolCoord = robot.GetExToolCoordWithID(id)
#     print(f"GetExToolCoordWithID {id},{extoolCoord[0]},{extoolCoord[1]},{extoolCoord[2]},{extoolCoord[3]},{extoolCoord[4]},{extoolCoord[5]}")
#     rtn, exAxisCoord = robot.GetExAxisCoordWithID(id)
#     print(f"GetExAxisCoordWithID {id},{exAxisCoord[0]},{exAxisCoord[1]},{exAxisCoord[2]},{exAxisCoord[3]},{exAxisCoord[4]},{exAxisCoord[5]}")
#     weight = 0.0
#     getCog = [0.0] * 3
#     rtn, weight, getCog = robot.GetTargetPayloadWithID(id)
#     print(f"GetTargetPayloadWithID {id},{weight},{getCog[0]},{getCog[1]},{getCog[2]}")
#     if i % 1 == 0 and i > 1:
#         id = id + 1
#     time.sleep(0.5)
#     print(f"times {i}")