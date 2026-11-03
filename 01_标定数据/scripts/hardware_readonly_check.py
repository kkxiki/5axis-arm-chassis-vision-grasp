import socket

import minimalmodbus
import pyrealsense2 as rs
import serial


def check_arm():
    with socket.create_connection(("192.168.30.11", 5868), timeout=3):
        print("ARM_TCP_OK 192.168.30.11:5868")


def check_gripper():
    dev = minimalmodbus.Instrument("/dev/ttyUSB0", 1)
    dev.serial.baudrate = 115200
    dev.serial.bytesize = 8
    dev.serial.parity = serial.PARITY_NONE
    dev.serial.stopbits = 1
    dev.serial.timeout = 0.5
    dev.mode = minimalmodbus.MODE_RTU
    dev.clear_buffers_before_each_transaction = True
    position = dev.read_registers(0x060D, 2, functioncode=3)
    alarm = dev.read_register(0x0612, functioncode=3)
    ready = dev.read_register(0x0604, functioncode=3)
    raw_position = (position[0] << 16) | position[1]
    print(f"GRIPPER_RTU_OK position={raw_position} alarm=0x{alarm:04x} ready={ready}")
    dev.serial.close()


def check_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    try:
        frames = pipeline.wait_for_frames(5000)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        device = profile.get_device()
        serial_number = device.get_info(rs.camera_info.serial_number)
        print(f"REALSENSE_FRAME_OK serial={serial_number} color={color.get_width()}x{color.get_height()} depth={depth.get_width()}x{depth.get_height()}")
    finally:
        pipeline.stop()


for name, check in (("arm", check_arm), ("gripper", check_gripper), ("camera", check_camera)):
    try:
        check()
    except Exception as exc:
        print(f"{name.upper()}_CHECK_FAILED {type(exc).__name__}: {exc}")
