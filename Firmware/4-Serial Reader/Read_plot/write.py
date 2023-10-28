import serial
class Writer:
    all = []
    def __init__(self,filePath):
        self.filePath = filePath
        Writer.all.append(self)
    def openFile(self):
        fp = open(self.filePath, 'r+')
        print("Opened the file")
        return fp
    def writetoFile(self,fp,line):
        fp.write(line)
    def closeFile(self,fp):
        fp.close()
    def writetoPort(self,port):
        port.write(b"Writing finished \r\n")