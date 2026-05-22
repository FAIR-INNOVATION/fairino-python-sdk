"""
CNDE 快速测试脚本
用于快速验证 CNDE 连接和基本数据接收
"""
import sys
sys.path.insert(0, r'f:\PythonSDK\SDKV2.0.5\SDKV2.2.5')
from fairino import Robot
from fairino.Robot import RobotState
import time
import ctypes


def print_pkg_data(pkg, prefix="  "):
    """打印RobotStatePkg全部数据 - 使用反射遍历所有字段"""
    from fairino.Robot import RobotStatePkg
    import ctypes

    def print_struct_array(arr, name, pre):
        """展开打印结构体数组"""
        print(f"{pre}{name}: (长度: {len(arr)})")
        for i, item in enumerate(arr):
            if hasattr(item, '_fields_'):
                fields_str = []
                for fname, ftype in item._fields_:
                    fv = getattr(item, fname)
                    if isinstance(fv, float):
                        fields_str.append(f"{fname}={fv:.2f}")
                    else:
                        fields_str.append(f"{fname}={fv}")
                print(f"{pre}  [{i}] {', '.join(fields_str)}")
            else:
                print(f"{pre}  [{i}] {item}")

    print(f"{prefix}===== RobotStatePkg 完整数据 =====")

    # 获取所有字段定义
    if hasattr(pkg, '_fields_'):
        fields = pkg._fields_
    elif hasattr(RobotStatePkg, '_fields_'):
        fields = RobotStatePkg._fields_
    else:
        print(f"{prefix}错误: 无法获取字段定义")
        return

    # 遍历所有字段并打印
    for field_name, field_type in fields:
        try:
            value = getattr(pkg, field_name)

            # 处理数组类型
            if isinstance(value, ctypes.Array):
                if len(value) == 0:
                    print(f"{prefix}{field_name}: [] (空数组)")
                    continue
                # 判断数组元素类型
                first_elem = value[0]
                if isinstance(first_elem, (int, float)):
                    # 数值数组，格式化输出
                    if isinstance(first_elem, int):
                        array_str = ', '.join([str(x) for x in value])
                    else:
                        # float数组，保留3位小数
                        array_str = ', '.join([f'{x:.3f}' for x in value])
                    print(f"{prefix}{field_name}: [{array_str}] (长度: {len(value)})")
                elif hasattr(first_elem, '_fields_'):
                    # 结构体数组，展开显示
                    print_struct_array(value, field_name, prefix)
                else:
                    # 其他类型数组
                    print(f"{prefix}{field_name}: [{', '.join([str(x) for x in value])}] (长度: {len(value)})")

            # 处理 ctypes 基本类型 (c_int, c_double等)
            elif hasattr(value, 'value'):
                actual_value = value.value
                if isinstance(actual_value, float):
                    print(f"{prefix}{field_name}: {actual_value:.6f}")
                else:
                    print(f"{prefix}{field_name}: {actual_value} (0x{actual_value:X})")

            # 处理普通数值
            elif isinstance(value, (int, float)):
                if isinstance(value, float):
                    print(f"{prefix}{field_name}: {value:.6f}")
                else:
                    print(f"{prefix}{field_name}: {value}")

            # 其他类型直接打印
            else:
                print(f"{prefix}{field_name}: {value}")

        except Exception as e:
            print(f"{prefix}{field_name}: [访问错误: {e}]")

    print(f"{prefix}===== 共 {len(fields)} 个字段 =====")


def main():
    print("=" * 50)
    print("CNDE 快速测试")
    print("=" * 50)
    
    # 连接机器人（自动连接CNDE）
    print("\n1. 连接机器人...")
    robot = Robot.RPC('192.168.58.2')
    print("   ✓ RPC连接成功（CNDE自动连接）")
    
    # 获取CNDE配置
    print("\n2. 获取CNDE配置...")
    config = robot.CNDEGetConfig()
    if config:
        states, period = config
        print(f"   ✓ 默认配置: {len(states)} 个状态")
        print(f"   ✓ 数据周期: {period}ms")
        print(f"   前70个状态: {[s.name for s in states[:70]]}")
    
    # 接收数据验证 - 100次循环，每次打印完整数据
    print("\n3. 接收状态数据（100次完整数据打印）...")
    
    for i in range(100):
        pkg = robot.robot_state_pkg
        print(f"\n{'='*60}")
        print(f"第 {i+1}/10 帧数据 (frame_cnt={pkg.frame_cnt})")
        print(f"{'='*60}")
        print_pkg_data(pkg)
        time.sleep(0.2)
    
    print(f"\n   ✓ 共接收 10 帧数据")

    print("\n" + "=" * 50)
    print("测试完成！")
    print("=" * 50)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
