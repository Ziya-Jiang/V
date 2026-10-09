import serial
import struct
import time

class edb_force_sensor:
    def __init__(self, com_id, baud_rate=9600):
        # 增加超时为5秒
        self.serial_port = serial.Serial(com_id, baud_rate, timeout=5)

    def close(self):
        self.serial_port.close()

    def _crc16(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def _build_command(self, motor_id):
        header = [motor_id, 0x03, 0x00, 0x00, 0x00, 0x0D]
        crc = self._crc16(header)
        header += [crc & 0xFF, crc >> 8]
        return bytes(header)

    def _send_command(self, motor_id):
        tx_buffer = self._build_command(motor_id)
        #print(tx_buffer)
        self.serial_port.write(tx_buffer)
        return self._read_response()

    def _read_response(self):
        # 尝试读取31个字节，打印接收到的字节长度和内容用于调试
        response = self.serial_port.read(31)
        # print(f"Received {len(response)} bytes: {response}")
        if len(response) != 31:
            raise Exception("Incomplete response")

        # 根据协议解析数据，这里解析的是 3 到 6 字节的数据
        print(response[3:7].hex())
        f = struct.unpack(">f", response[3:7])[0] * -1  # 假设 force 是第3到第6字节的数据
        return f

    def read_force(self, motor_id):
        return self._send_command(motor_id)

# 示例用法
if __name__ == "__main__":
    force_sensor = edb_force_sensor("/dev/ttyUSB0", 9600)

    try:
        # 获取电机状态
        for i in range(100):
            f = force_sensor.read_force(1)
            print('Force:', f)
            time.sleep(0.05)

    finally:
        force_sensor.close()
