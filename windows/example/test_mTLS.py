from fairino import Robot
import time

robot = Robot.RPC(
    '192.168.58.2',
    tls_enable=True,
    tls_cert_path=r"E:\certs"
)

def test(self):
    """控制机器人程序暂停、恢复、停止"""
    robot.Mode(state=0)
    robot.ProgramRun()
    time.sleep(1)

    for i in range(0,100):
        error = robot.PauseMotion()
        print("PauseMotion return ", error)
        time.sleep(2)
        error = robot.ResumeMotion()
        print("ResumeMotion return ", error)
        time.sleep(2)
    error = robot.StopMotion()
    print("StopMotion return ", error)

test(robot)
