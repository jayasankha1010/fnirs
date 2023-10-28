import numpy as np
import matplotlib.pyplot as plt
import os
from random import randint
from scipy import signal
import matplotlib.pyplot as plt

#print("1")

b, a = signal.butter(4, 100, 'low', analog=True)
w, h = signal.freqs(b, a)
# plt.semilogx(w, 20 * np.log10(abs(h)))
# plt.title('Butterworth filter frequency response')
# plt.xlabel('Frequency [radians / second]')
# plt.ylabel('Amplitude [dB]')
# plt.margins(0, 0.1)
# plt.grid(which='both', axis='both')
# plt.axvline(100, color='green') # cutoff frequency
# plt.show()
# plt.close()

#print("2")

filePath = "data"
fileNames = [f for f in os.listdir(filePath) if f.endswith(".txt")]
#print(fileNames)

#print("3")

selFile = fileNames[0]
fileName = os.path.join(filePath,selFile)
yNum = 0
HbONum = 0
HbRNum = 0
HbRVal = []
HbOVal = []
fp = open(fileName,"r+")
for i in fp.readlines():
  lineVal = i.strip()
  if (lineVal.isnumeric()):
    yVal = int(lineVal)
    if (yNum%2==0):
      HbONum+=1
      HbOVal.append(yVal)
    else:
      HbRNum+=1
      HbRVal.append(yVal)
    yNum+=1
    
#print("4")    
    
    
HbRx = np.linspace(0,HbRNum,HbRNum)
HbOx = np.linspace(0,HbONum,HbONum)
#print(HbOx.shape)
HbOVal = np.array(HbOVal)
#print(HbOVal.shape)
HbRVal = np.array(HbRVal)


def plot(HbOx,HbOVal,HbRx,HbRVal,L,R):
  fig = plt.figure()
  fig.set_size_inches(12, 5)
  plt.title('fNIRS Raw Data')
  plt.xlim(L,R)
  plt.ylim(250,850)
  plt.plot(HbOx,HbOVal,label="Oxy-Heamoglobin Intensities")
  plt.plot(HbRx,HbRVal,label="DeOxy-Heamoglobin Intensities")
  plt.legend(loc='upper center', numpoints=1, bbox_to_anchor=(0.5, -0.05), ncol=1, fancybox=True, shadow=True)
  plt.show()

def plotSpecial(HbOx,HbOVal,HbRx,HbRVal,L,R):
  fig = plt.figure()
  fig.set_size_inches(12, 5)
  plt.title('fNIRS Raw Data')
  # plt.xlim(L,R)
  plt.ylim(250,850)
  #plt.axis([500, 12000, 400, 700])
  plt.plot(HbOx,HbOVal,label="Oxy-Heamoglobin Intensities")
  plt.plot(HbRx,HbRVal,label="DeOxy-Heamoglobin Intensities")
  plt.legend(loc='upper center', numpoints=1, bbox_to_anchor=(0.5, -0.05), ncol=1, fancybox=True, shadow=True)
  plt.show()
  
# for arithmatic task

peakNum = randint(2,4)
#print("Number of peaks: ",peakNum)
limit = 350
specialHbOVal = np.copy(HbOVal)
specialHbRVal = np.copy(HbRVal)
for peakInt in range(1,peakNum+1):
  Left =  randint(limit,min(limit+200,min(HbRNum-1,HbONum-1)))
  peak = Left + randint(75,100)
  Right = peak+ randint(75,100)
  #print("Peak Number : "+str(peakInt),[Left,peak,Right])
  
  if (peakInt%2==randint(0,1)):
    for i in range(Left,Right):
      specialHbOVal[min(i,HbONum-1)] += int(-(randint(75,100)/100.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
    for i in range(Left,Right):
      specialHbRVal[min(i+randint(10,20),HbRNum-1)] += int(+(randint(75,100)/100.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))

  else:
    for i in range(Left,Right):
      specialHbRVal[min(i,HbRNum-1)] += int(-(randint(75,100)/100.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
    for i in range(Left,Right):
      specialHbRVal[min(i+randint(10,20),HbRNum-1)] += int(+(randint(75,100)/100.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
  limit=Right
  
  
# for valsalva manuever task

# peakNum = randint(2,4)
# #print("Number of peaks: ",peakNum)
# limit = 100
# specialHbOVal = np.copy(HbOVal)
# specialHbRVal = np.copy(HbRVal)
# for peakInt in range(1,peakNum+1):
  # #print(peakInt)
  # Left =  randint(limit,min(limit+200,min(HbRNum-1,HbONum-1)))
  # peak = Left + randint(50,100)
  # Right = peak+ randint(50,100)
  # #print("Peak Number : "+str(peakInt),[Left,peak,Right])
  
  # if (peakInt%2==randint(0,1)):
    # for i in range(Left,Right):
      # specialHbOVal[min(i,HbONum-1)] += int(+(randint(70,100)/1000.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
    # for i in range(Left,Right):
      # specialHbRVal[min(i+randint(20,35),HbRNum-1)] += int(+(randint(60,100)/1000.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))

  # else:
    # for i in range(Left,Right):
      # specialHbRVal[min(i,HbRNum-1)] += int(+(randint(75,100)/1000.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
    # for i in range(Left,Right):
      # specialHbOVal[min(i+randint(20,35),HbONum-1)] += int(+(randint(65,100)/1000.0)*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
  # limit=Right
  
  
  # if (peakInt%2==randint(0,1)):
  #   for i in range(Left,Right):
  #     specialHbOVal[i] += int(-0.1*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
  #   for i in range(Left,Right):
  #     specialHbRVal[i+randint(20,35)] += int(+0.1*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))

  # else:
  #   for i in range(Left,Right):
  #     specialHbRVal[i] += int(-0.1*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))
  #   for i in range(Left,Right):
  #     specialHbOVal[i+randint(20,35)] += int(+0.1*((i-Left)*(i-Right)/(Left+Right)+randint(-10,10)))


# noise needed
for i in range(0,int(min(HbONum,HbRNum))):
  specialHbRVal[i] -= randint(0,10)
  specialHbOVal[i] += randint(0,10)
  #print(i)

trial = 5
if (trial==5):
  plotSpecial(HbOx,specialHbOVal,HbRx,specialHbRVal,0,12500)
else:
  plot(HbOx,HbOVal,HbRx,HbRVal,200,600)


#adding a low pass filter
sos = signal.butter(20, 5, 'lp', fs=80, output='sos')

fig = plt.figure()
fig.set_size_inches(12, 5)

filteredO = signal.sosfilt(sos, specialHbOVal)
plt.plot(HbOx, filteredO,label="Oxy-Heamoglobin Intensities")

filteredR = signal.sosfilt(sos, specialHbRVal)
plt.plot(HbRx, filteredR,label="DeOxy-Heamoglobin Intensities")


# plt.set_title('After 15 Hz high-pass filter')
# plt.axis([500, 12000, 450, 650])
# plt.set_xlabel('Time [seconds]')

plt.ylim(250,850)
plt.title('Low pass filtering the intensities')
plt.legend(loc='upper center', numpoints=1, bbox_to_anchor=(0.5, -0.05), ncol=1, fancybox=True, shadow=True)
plt.tight_layout()
plt.show()
plt.close()