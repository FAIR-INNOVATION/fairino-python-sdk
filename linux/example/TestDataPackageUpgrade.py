from fairino import Robot
import time


def test_data_package_upgrade(robot):
    """Data package upgrade test with password"""
    print("\n========== Data Package Upgrade Test ==========")

    time.sleep(0.2)  # 200ms

    # Data package upgrade with encryption enabled and password
    rtn = robot.DataPackageUpgrade("D://zUP/fr_user_data.tar.gz", True, "12345678")
    print(f"robot DataPackageUpgrade rtn is {rtn}")

    # Poll upgrade progress until 100%
    upgradeProgress = 0
    while upgradeProgress < 100:
        rtn, upgradeProgress = robot.GetDataPackageUpgradeProgress()
        print(f"robot data package upgrade progress : {upgradeProgress}")
        time.sleep(0.2)  # 200ms


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    # Run data package upgrade test
    test_data_package_upgrade(robot)

    # Close connection
    robot.CloseRPC()


if __name__ == "__main__":
    main()