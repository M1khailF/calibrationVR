import time
import serial

class SerialLamp:
    def __init__(self, port, baud):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS
        )
        self.ser.isOpen()
        time.sleep(1)
        print("Open serial")

    def sendSerial(self, message):
        self.ser.write(message.encode())
        print(f"Sending: {message}")
        time.sleep(0.1)

    def onRed(self):
        self.ser.write(b'0001')
        time.sleep(0.1)

    def onYellow(self):
        self.ser.write(b'0010')
        time.sleep(0.1)

    def onGreen(self):
        self.ser.write(b'0100')
        time.sleep(0.1)

    def onBlue(self):
        self.ser.write(b'1000')
        time.sleep(0.1)

    def serialRead(self):
        # try:
        line = self.ser.readline().decode().strip()
        if line:
            print("Received:", line)
        # except RuntimeError as err:
        #     self.ser.close()


if __name__ == "__main__":
    lamp = SerialLamp('/dev/ttyACM0', 115200)

    while True:
        lamp.serialRead()
        lamp.onRed()