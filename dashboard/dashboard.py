import serial
import socket
import time


class TelegrafSocket:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self.address = (self.hostname, self.port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, string: str):
        self.socket.sendto(string.encode(), self.address)


class Formatter:
    def __init__(self, tags):
        self.tags = tags

    def format(self, data: str) -> str:
        data = data.strip().split(",")
        if len(data) != len(tags):
            print(f"{len(data) != len(tags)}")
            exit(1)
        influx_string = ""
        for key, val in zip(self.tags, data):
            influx_string += f"{key}={val},"
        influx_string = influx_string[:-1]
        return f"robot {influx_string} {time.time_ns()}"


class Monitor:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_monitor = serial.Serial(port, baudrate, timeout=1)

    def get_line(self):
        line = self.serial_monitor.readline().decode("utf-8").strip()
        if line:
            return line


if __name__ == "__main__":
    tags = ["encoder_A", "encoder_B"]
    monitor = Monitor("/dev/cu.usbserial-0001", 115200)
    socket = TelegrafSocket("127.0.0.1", 4000)
    formatter = Formatter(tags)
    while True:
        line = monitor.get_line()
        if not line:
            continue
        if not line.startswith("*"):
            continue
        # Otherwise, send to telegraf/grafana
        socket.send(formatter.format(line[1:]))
