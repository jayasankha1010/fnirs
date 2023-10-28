from read import Reader
from write import Writer
port = 'COM5'
byteSize = 8
reader1 = Reader(port, byteSize)
writer1 = Writer('readings')
port = reader1.createSerialPort()
fp = writer1.openFile()
serialString = ""
while True:
    if(port.in_waiting > 0):
        data = reader1.readPort(port)
        serialString = port.readline()
        # Print the contents of the serial data
        decodeString = serialString.decode('Ascii')
        print(decodeString)
        writer1.writetoFile(fp, decodeString)
writer1.writetoPort(port)
writer1.closeFile(fp)
reader1.closePort(port)
