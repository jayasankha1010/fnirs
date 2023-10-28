import serial
class Reader:
    all = []

    def __init__(self, port, byteSize, timeout=2, baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self.byteSize = byteSize
        self.timeout = timeout
        Reader.all.append(self)

    def createSerialPort(self, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE):
        serialPort = serial.Serial(self.port, self.baudrate, self.byteSize, parity, stopbits, self.timeout)
        return serialPort
    def readPort(self,serialPort):
        serialString = serialPort.readline()
        data = serialString.decode('Ascii')
        print(data)
        return data
    def closePort(self,serialPort):
        serialPort.close()


