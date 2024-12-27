from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import serial

app = FastAPI()

origins = [
    "http://localhost:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

led_state = False
ser = serial.Serial("/dev/cu.usbserial-3", 115200)
print("opening serial port...")


@app.get("/")
async def root():
    return {"message": "Hello World"}


@app.get("/test")
async def test():
    global led_state
    global ser
    print("hello, world")
    led_state = not led_state
    message = f"{str(int(led_state))}x"
    ser.write(str.encode(message))
    return {"message": str(led_state)}
