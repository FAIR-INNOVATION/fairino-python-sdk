def robot_init():
    # 初始化连接状态标志位
    logger.info("开始重连机器人")
    if Config.ROBOT.get("robot1", {}).get("ip", None):
        global robot1
        try:
            robot1.CloseRPC()
            logger.info("机器人1断开连接")
            time.sleep(0.5)
            robot1 = Robot.RPC(Config.ROBOT['robot1']['ip'])
            global_speed = Config.ROBOT['robot1']["global_speed"]
            err = robot1.SetSpeed(global_speed)
            if err != 0:
                Config.robot_try_status = 2
                alone_up_write_plc_signal(robot_1_no_reset_address, 1, 0)
            ret = UpEnable(robot1)
            if not ret:
                Config.robot_try_status = 2
                alone_up_write_plc_signal(robot_1_no_reset_address, 1, 0)
            logger.info(f"机器人1上使能 {ret}")
            logger.info(f"机器人1初始化完成")
        except Exception as e:
            Config.robot_try_status = 2
            alone_up_write_plc_signal(robot_1_no_reset_address, 1, 0)
            logger.error(e)
    if Config.ROBOT.get("robot2", {}).get("ip", None):
        global robot2
        try:
            robot2.CloseRPC()
            logger.info("机器人2断开连接")
            time.sleep(0.5)
            robot2 = Robot.RPC(Config.ROBOT['robot2']['ip'])
            global_speed = Config.ROBOT['robot2']["global_speed"]
            err = robot2.SetSpeed(global_speed)
            if err != 0:
                Config.robot_try_status = 2
                alone_up_write_plc_signal(robot_2_no_reset_address, 1, 0)
            ret = UpEnable(robot2)
            if not ret:
                Config.robot_try_status = 2
                alone_up_write_plc_signal(robot_2_no_reset_address, 1, 0)
            logger.info(f"机器人2上使能 {ret}")
            logger.info(f"机器人2初始化完成")
        except Exception as e:
            Config.robot_try_status = 2
            alone_up_write_plc_signal(robot_2_no_reset_address, 1, 0)
            logger.error(e)
    if Config.ROBOT.get("robot3", {}).get("ip", None):
        global robot3
        try:
            robot3.CloseRPC()
            logger.info("机器人3断开连接")
            time.sleep(0.5)
            robot3 = Robot.RPC(Config.ROBOT['robot3']['ip'])
            global_speed = Config.ROBOT['robot3']["global_speed"]
            err = robot3.SetSpeed(global_speed)
            if err != 0:
                Config.robot_try_status = 2
                alone_up_write_plc_signal(robot_3_no_reset_address, 1, 0)
            ret = UpEnable(robot3)
            if not ret:
                Config.robot_try_status = 2
                alone_up_write_plc_signal(robot_3_no_reset_address, 1, 0)
            logger.info(f"机器人3上使能 {ret}")
            logger.info(f"机器人3初始化完成")
        except Exception as e:
            Config.robot_try_status = 2
            alone_up_write_plc_signal(robot_3_no_reset_address, 1, 0)
            logger.error(e)