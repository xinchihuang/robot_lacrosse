import signal
import sys

def signal_handler(signum, frame):
    print("接收到信号，程序将退出...")
    sys.exit(0)

# 设置信号处理函数
signal.signal(signal.SIGINT, signal_handler)

print("程序正在运行，请尝试按 Ctrl+C 退出...")
while True:
    pass  # 无限循环，等待信号
