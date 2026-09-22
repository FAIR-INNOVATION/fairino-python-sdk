from fairino import Robot
import time


def main():
    # Establish connection with the robot controller (TLS enabled)
    robot = Robot.RPC(
        '192.168.58.2',
        tls_enable=True,
        tls_cert_path=r"E:\certs"
    )
    time.sleep(0.5)  # Wait for connection and data reception

    # Set simple UDP command reply callback
    def callback(src_type, count, cmd_id, data_len, content):
        print("Callback: cmd_id={} count={} data_len={} content={}".format(cmd_id, count, data_len, content))
        return 0

    robot.SetUDPCmdRpyCallback(callback)

    # Program name to load
    program_name = "mtls.lua"
    state = 0
    line = 0

    mode0 = "/f/bIII52III236III7IIIMode(0)III/b/f"
    mode1 = "/f/bIII52III236III7IIIMode(1)III/b/f"

    print(f"[SEND] {mode0}")
    robot.SendTCPFrame(mode0)
    time.sleep(1)

    print(f"[SEND] {mode1}")
    robot.SendTCPFrame(mode1)
    time.sleep(1)

    print(f"[SEND] {mode0}")
    robot.SendTCPFrame(mode0)

    # Set auto mode and load program
    robot.Mode(0)
    robot.LoadDefaultProgConfig(0, program_name)
    robot.ProgramLoad(program_name)
    robot.SetSpeedInstant(50)
    robot.ProgramRun()
    time.sleep(5)

    # Pause and query program state
    robot.PauseMotion()
    rtn, state = robot.GetProgramState()
    print(f"program state:{state}\n")

    rtn, line = robot.GetCurrentLine()
    print(f"current line:{line}\n")

    rtn, loaded_name = robot.GetLoadedProgram()
    print(f"program name:{loaded_name}\n")

    # Resume and pause again
    robot.ResumeMotion()
    time.sleep(2)
    robot.PauseMotion()
    time.sleep(2)

    # Close connection
    robot.CloseRPC()
    time.sleep(1)


if __name__ == "__main__":
    main()