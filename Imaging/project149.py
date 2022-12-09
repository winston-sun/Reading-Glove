import serial

import serial.tools.list_ports
from time import sleep
print ('\n\nPYSERIAL\n')

sPort = '/dev/cu.usbmodem14201' #On Mac - find this using >ls /dev/cu.usb*

aSerialData = serial.Serial(sPort, 9600)
flag = False
captured_images = 0
arr = list()
while True:
    flag = False

    if (aSerialData.inWaiting()>0):
        print("Image Capturing...")
        sData = aSerialData.readline()

        sData = (str(sData).split(","))

        #print( sData )
        arr.append(sData)
        captured_images += 1
        if captured_images %5 ==0:
            captured_images = 0
            print("done")
            print(arr)
            flag = True
            sleep(2)


