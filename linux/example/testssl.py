import ssl
import socket

HOST = '192.168.58.2'
PORT = 8080

ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
ctx.load_verify_locations(r"E:\certs\ca.crt")
ctx.load_cert_chain(r"E:\certs\client.crt", r"E:\certs\client.key")
ctx.verify_mode = ssl.CERT_REQUIRED
ctx.check_hostname = False    # 如果是IP直连建议关掉

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)

try:
    ssock = ctx.wrap_socket(sock)
    print("SSL上下文已准备")
    ssock.connect((HOST, PORT))
    print(f"mTLS连接成功，协议: {ssock.version()}, 加密套件: {ssock.cipher()}")
    ssock.sendall(b"test")
    print("已发送数据")
    ssock.close()
except ssl.SSLError as e:
    print(f"SSL错误: {e}")
except socket.timeout:
    print("连接超时")
except ConnectionRefusedError:
    print("连接被拒绝")
except Exception as e:
    print(f"错误: {type(e).__name__}: {e}")