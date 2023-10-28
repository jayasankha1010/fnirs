# import matplotlib.pyplot as plt  
import os
path = "D:\Work_Parent\Biowire\fNIRS_hardware\Firmware\4-Serial Reader\UI\Python"
text_files = [f for f in os.listdir(path) if f.endswith('.txt')]
print(text_files)