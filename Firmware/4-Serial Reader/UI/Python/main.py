from quopri import decodestring 
from read import Reader
from write import Writer
from timeit import default_timer as timer

port = 'COM5'
byteSize = 8
reader1 = Reader(port, byteSize)
writer1 = Writer('S07_AM.txt')
port = reader1.createSerialPort()
fp = writer1.openFile()
serialString = ""
start = timer()
totalTime = 0
while (totalTime<=60):
    if(port.in_waiting > 0):
        data = reader1.readPort(port)
        serialString = port.readline()
        t_point = timer()
        totalTime = t_point - start
        # Print the contents of the serial data
        timeString = str(totalTime)
        decodeString = serialString.decode('Ascii')
        # writeString = timeString + " : "+decodeString
        print(decodeString)
        # print("One Done")
        writer1.writetoFile(fp, decodeString)
writer1.writetoPort(port)
writer1.closeFile(fp)
reader1.closePort(port)
