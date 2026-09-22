# -*- coding: utf-8 -*-
"""
DTLS 连通性完整测试脚本（方案C：阻塞socket + 短超时驱动重传）
"""

import os
import time
import socket
import errno

from mbedtls.tls import (
    ClientContext,
    DTLSConfiguration,
    TrustStore,
    TLSVersion,
    WantReadError,
    WantWriteError,
)
from mbedtls import x509
from mbedtls.pk import RSA, ECC


# ==================== 配置区 ====================
ROBOT_IP        = "192.168.58.2"
ROBOT_DTLS_PORT = 20007
LOCAL_BIND_PORT = 20008
CERT_DIR        = r"E:\certs"

CA_CERT     = os.path.join(CERT_DIR, "ca.crt")
CLIENT_CERT = os.path.join(CERT_DIR, "client.crt")
CLIENT_KEY  = os.path.join(CERT_DIR, "client.key")

HANDSHAKE_TIMEOUT = 30.0        # 握手总超时
SOCK_TIMEOUT      = 0.2         # 单次 socket 读超时（关键参数）
RECV_TIMEOUT      = 3.0
TEST_MESSAGE      = "/f/bIII52III236III7IIIMode(0)III/b/f"


# ==================== 构建 DTLS 上下文 ====================
def build_dtls_context():
    if not os.path.exists(CA_CERT):
        raise FileNotFoundError(f"CA 证书不存在: {CA_CERT}")
    trust_store = TrustStore()
    trust_store.add(x509.CRT.from_file(CA_CERT))
    print(f"[DTLS] 已加载 CA: {CA_CERT}")

    if not os.path.exists(CLIENT_CERT):
        raise FileNotFoundError(f"客户端证书不存在: {CLIENT_CERT}")
    if not os.path.exists(CLIENT_KEY):
        raise FileNotFoundError(f"客户端私钥不存在: {CLIENT_KEY}")
    client_crt = x509.CRT.from_file(CLIENT_CERT)

    try:
        private_key = RSA.from_file(CLIENT_KEY)
        print("[DTLS] 私钥类型: RSA")
    except Exception:
        private_key = ECC.from_file(CLIENT_KEY)
        print("[DTLS] 私钥类型: ECC")

    cert_chain = ([client_crt], private_key)
    print(f"[DTLS] 已加载客户端证书: {CLIENT_CERT}")

    try:
        config = DTLSConfiguration(
            trust_store=trust_store,
            certificate_chain=cert_chain,
            validate_certificates=True,
            minimum_version=TLSVersion.TLS1_2,
            maximum_version=TLSVersion.TLS1_2,
        )
    except (ImportError, TypeError):
        config = DTLSConfiguration(
            trust_store=trust_store,
            certificate_chain=cert_chain,
            validate_certificates=True,
        )
    return ClientContext(config)


# ==================== 执行握手（方案C） ====================
def do_dtls_handshake(dtls_sock, udp_sock):
    """
    握手：底层 socket 保持阻塞，但设置短超时（SOCK_TIMEOUT），
    这样 recvfrom 会定期超时返回，mbedtls 每次重入检查重传定时器，
    从而真正驱动 DTLS 握手重传。
    """
    print("[..] 发起 DTLS 握手 ...")

    # 关键：阻塞 socket + 短超时，让底层 recvfrom 自己超时
    udp_sock.settimeout(SOCK_TIMEOUT)

    try:
        dtls_sock.connect((ROBOT_IP, ROBOT_DTLS_PORT))
    except Exception as e:
        print(f"[FAIL] connect 异常: {e}")
        return False

    start = time.time()
    last_print = start

    while time.time() - start < HANDSHAKE_TIMEOUT:
        try:
            dtls_sock.do_handshake()
            print(f"[OK] DTLS 握手成功，用时 {time.time() - start:.2f}s")
            # 握手完成后恢复正常超时
            udp_sock.settimeout(1.0)
            return True
        except (WantReadError, WantWriteError):
            # 正常非阻塞语义，直接循环让 mbedtls 检查定时器
            time.sleep(0.01)
            continue
        except (socket.timeout, OSError) as e:
            # 底层 recvfrom 超时 或 WSAEWOULDBLOCK，都视为"暂时没数据"
            winerr = getattr(e, "winerror", None)
            if winerr == 10035 or e.errno in (errno.EWOULDBLOCK, errno.EAGAIN):
                time.sleep(0.01)
                continue
            if isinstance(e, socket.timeout):
                # 每次超时都重新进循环，mbedtls 会检查重传定时器
                # 每 5 秒打印一次状态，方便观察
                if time.time() - last_print > 5.0:
                    print(f"[..] 握手进行中，已耗时 {time.time() - start:.1f}s")
                    last_print = time.time()
                continue
            print(f"[FAIL] DTLS 握手异常: {e}")
            return False
        except Exception as e:
            print(f"[FAIL] DTLS 握手异常: {e}")
            return False

    print(f"[FAIL] DTLS 握手超时（{HANDSHAKE_TIMEOUT}s）")
    return False


# ==================== 发送数据 ====================
def dtls_send(dtls_sock, payload: bytes):
    start = time.time()
    while time.time() - start < 3.0:
        try:
            return dtls_sock.send(payload)
        except (WantReadError, WantWriteError):
            time.sleep(0.02)
            continue
        except OSError as e:
            winerr = getattr(e, "winerror", None)
            if winerr == 10035 or e.errno in (errno.EWOULDBLOCK, errno.EAGAIN):
                time.sleep(0.02)
                continue
            raise
    raise TimeoutError("DTLS 发送超时")


# ==================== 接收数据 ====================
def dtls_recv(dtls_sock, timeout: float):
    start = time.time()
    while time.time() - start < timeout:
        try:
            data = dtls_sock.recv(65535)
            if data:
                return data
            time.sleep(0.02)
        except WantReadError:
            time.sleep(0.02)
            continue
        except WantWriteError:
            time.sleep(0.02)
            continue
        except (socket.timeout, OSError) as e:
            winerr = getattr(e, "winerror", None)
            if winerr == 10035 or e.errno in (errno.EWOULDBLOCK, errno.EAGAIN) \
                    or isinstance(e, socket.timeout):
                time.sleep(0.02)
                continue
            print(f"[WARN] 接收异常: {e}")
            return None
        except Exception as e:
            print(f"[WARN] 接收异常: {e}")
            return None
    return None


# ==================== 主流程 ====================
def main():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        udp_sock.bind(("0.0.0.0", LOCAL_BIND_PORT))
    except OSError as e:
        print(f"[ERROR] 绑定本地端口 {LOCAL_BIND_PORT} 失败: {e}")
        return
    print(f"[OK] UDP socket 已绑定本地端口 {LOCAL_BIND_PORT}")

    try:
        ctx = build_dtls_context()
        dtls_sock = ctx.wrap_socket(udp_sock, server_hostname=None)
        print("[OK] DTLS 上下文创建成功")
    except Exception as e:
        print(f"[ERROR] DTLS 上下文创建失败: {e}")
        udp_sock.close()
        return

    if not do_dtls_handshake(dtls_sock, udp_sock):
        print("[FAIL] DTLS 握手未完成，退出")
        try:
            dtls_sock.close()
        except Exception:
            pass
        return

    # 发送测试数据
    print(f"[..] 发送: {TEST_MESSAGE!r}")
    try:
        sent = dtls_send(dtls_sock, TEST_MESSAGE.encode("utf-8"))
        print(f"[OK] 发送成功，{sent} 字节")
    except Exception as e:
        print(f"[FAIL] 发送失败: {e}")
        try:
            dtls_sock.close()
        except Exception:
            pass
        return

    # 接收
    print(f"[..] 等待 {RECV_TIMEOUT} 秒接收回包 ...")
    data = dtls_recv(dtls_sock, RECV_TIMEOUT)
    if data:
        try:
            text = data.decode("utf-8", errors="replace")
        except Exception:
            text = data.hex()
        print(f"[OK] 收到回包 {len(data)} 字节:")
        print(f"     {text[:300]}")
    else:
        print("[INFO] 未收到回包")

    # 再发一次 SDK 帧格式
    print("\n[..] 尝试发送 SDK 帧格式 ...")
    frame = "/f/bIII52III236III7IIIMode(0)III/b/f"
    try:
        sent = dtls_send(dtls_sock, frame.encode("utf-8"))
        print(f"[OK] 帧发送成功，{sent} 字节")
    except Exception as e:
        print(f"[FAIL] 帧发送失败: {e}")

    print(f"[..] 等待 {RECV_TIMEOUT} 秒接收回包 ...")
    data = dtls_recv(dtls_sock, RECV_TIMEOUT)
    if data:
        try:
            text = data.decode("utf-8", errors="replace")
        except Exception:
            text = data.hex()
        print(f"[OK] 收到回包 {len(data)} 字节:")
        print(f"     {text[:300]}")
    else:
        print("[INFO] 未收到回包")

    try:
        dtls_sock.close()
    except Exception:
        pass
    print("\n[DONE] 测试结束")


if __name__ == "__main__":
    main()