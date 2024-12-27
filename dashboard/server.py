from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import serial
import asyncio
from contextlib import asynccontextmanager
import time
import socket


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

    def write(self, data):
        self.serial_monitor.write(data)


class Command:
    NOP = 0
    CALIBRATE_IMU = 1
    ARM = 2
    ABORT = 3
    FORWARD = 4
    BACKWARD = 5
    TURN_CCW = 6
    TURN_CW = 7
    ACTIVATE_SERVO = 8

    def get_command_from_string(command: str):
        return int(command)


class CommandQuery(BaseModel):
    command: int


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start serial polling on server startup."""
    asyncio.create_task(read_serial())
    yield


app = FastAPI(lifespan=lifespan)

origins = [
    "http://localhost:3000",
]

incoming_data = []


async def read_serial():
    """Continuously poll serial data in the background."""
    global ser, incoming_data
    while True:
        line = ser.get_line()
        if line and line.startswith("x"):
            socket.send(formatter.format(line[1:]))
        await asyncio.sleep(0.01)  # Prevent blocking


app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

led_state = False
tags = ["encoder_A", "encoder_B", "pitch"]
ser = Monitor("/dev/cu.usbserial-3", 115200)
socket = TelegrafSocket("127.0.0.1", 4000)
formatter = Formatter(tags)
print("opening serial port...")


@app.get("/")
async def root():
    return {"message": "Hello World"}


@app.post("/send_command/")
async def send_command(command: CommandQuery):
    global ser
    command_number = Command.get_command_from_string(command.command)
    message = f"{str(command_number)}x"
    print(message)
    ser.write(str.encode(message))
    return {"message": "yippee"}


@app.get("/serial_data/")
async def get_serial_data():
    """Retrieve accumulated serial data."""
    global incoming_data
    return {"data": incoming_data}
