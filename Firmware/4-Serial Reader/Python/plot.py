import matplotlib.pyplot as plt  
import numpy as np  
import os
path = "fNIRS data"
fileNames = [f for f in os.listdir(path) if f.endswith('.txt')]
for i in fileNames:
    filename = os.path.join(path,i)
    print(filename)
    fp = open(filename,'r+')
    Num = 0
    HbONum = 0
    HbRNum = 0
    yNum = 0
    HbOVal = []
    HbRVal = []
    for line in fp.readlines():
        fileVal = line.strip()
        if (fileVal.isnumeric()):
            y_val = int(fileVal)
            if(yNum%2==0):
                HbONum += 1
                HbOVal.append(y_val)
            else:
                HbRNum+=1
                HbRVal.append(y_val)
            yNum += 1
    HbOx = np.linspace(0,HbONum,HbONum)
    HbRx = np.linspace(0,HbRNum,HbRNum)
    figName = i[:6]+".png"
    fig = plt.figure()
    fig.set_size_inches(12, 10)
    plt.ylim(350,650)
    plt.plot(HbOx,HbOVal,label="Oxy-Heamoglobin Intensities")
    plt.plot(HbRx,HbRVal,label="DeOxy-Heamoglobin Intensities")
    plt.legend(loc='upper center', numpoints=1, bbox_to_anchor=(
        0.5, -0.05), ncol=1, fancybox=True, shadow=True)
    plt.show(block=False)
    plt.pause(3)
    # fig.savefig(figName, bbox_inches='tight')
    fig.savefig(figName)
    plt.close()
