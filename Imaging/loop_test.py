import numpy as np
import cv2
import pytesseract
import matplotlib.pyplot as plt
import os
from network import Bluetooth
import time

bt = Bluetooth()
bt.start_scan(-1)

adv_name = 'Heart Rate' # Change this to whatever our advertiser name is 

while True:
  adv = bt.get_adv()
  if adv and bt.resolve_adv_data(adv.data, Bluetooth.ADV_NAME_CMPL) == adv_name:
      try:
          conn = bt.connect(adv.mac)
          services = conn.services()
          for service in services:
              time.sleep(0.050)
              if type(service.uuid()) == bytes:
                  print('Reading chars from service = {}'.format(service.uuid()))
              else:
                  print('Reading chars from service = %x' % service.uuid())
              chars = service.characteristics()
              for char in chars: # Need to know how this works before I start looping
                  if (char.properties() & Bluetooth.PROP_READ):
                      print('char {} value = {}'.format(char.uuid(), char.read()))
          conn.disconnect()
          break
      except:
          pass
  else:
      time.sleep(0.050)

