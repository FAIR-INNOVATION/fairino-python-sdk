# -*- coding: utf-8 -*-
"""
最小化 UDP 明文测试脚本（非加密）
固定发送: /f/bIII52III236III7IIIMode(0)III/b/f
"""

import socket

ROBOT_IP = "192.168.58.2"
ROBOT_PORT = 20007
LOCAL_BIND_PORT = 20008

FRAME = "/f/bIII52III236III7IIIMode(0)III/b/f"


def main():
    print("=" * 60)
    print("UDP 明文测试")
    print(f"目标: {ROBOT_IP}:{ROBOT_PORT}")
    print(f"本地源端口: {LOCAL_BIND_PORT}")
    print(f"发送帧: {FRAME}")
    print("=" * 60)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', LOCAL_BIND_PORT))
    except OSError as e:
        print(f"[FAIL] 绑定本地端口 {LOCAL_BIND_PORT} 失败: {e}")
        return
    sock.settimeout(2.0)
    print(f"[OK] UDP socket 已绑定: {sock.getsockname()}")

    try:
        sent = sock.sendto(FRAME.encode('utf-8'), (ROBOT_IP, ROBOT_PORT))
        print(f"[OK] sendto 返回 {sent} 字节")
    except Exception as e:
        print(f"[FAIL] sendto 异常: {e}")
        sock.close()
        return

    print("\n[等待] 最多等 2 秒收包...")
    try:
        data, addr = sock.recvfrom(4096)
        print(f"[OK] 收到 {len(data)} 字节，来自 {addr}")
        try:
            text = data.decode('utf-8')
        except UnicodeDecodeError:
            text = data.hex()
        print(f"     内容: {text}")
    except socket.timeout:
        print("[TIMEOUT] 2 秒内没收到任何回复")
        print("         → 包已发出但机器人未回")
    except Exception as e:
        print(f"[FAIL] recvfrom 异常: {e}")

    sock.close()
    print("\n[DONE] 测试结束")


if __name__ == "__main__":
    main()