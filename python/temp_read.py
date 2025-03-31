'''
Description: 
Author: zxy
Date: 2025-03-31 15:29:51
LastEditors: zxy
LastEditTime: 2025-03-31 15:29:51'
'''

import pcammls
from pcammls import * 
import cv2
import numpy
import sys
import os


def main():
    cl = PercipioSDK()

    dev_list = cl.ListDevice()
    for idx in range(len(dev_list)):
      dev = dev_list[idx]
      print ('{} -- {} \t {}'.format(idx,dev.id,dev.iface.id))
    if  len(dev_list)==0:
      print ('no device')
      return
    if len(dev_list) == 1:
        selected_idx = 0 
    else:
        selected_idx  = int(input('select a device:'))
    if selected_idx < 0 or selected_idx >= len(dev_list):
        return

    sn = dev_list[selected_idx].id
    
    handle = cl.Open(sn)
    if not cl.isValidHandle(handle):
      err = cl.TYGetLastErrorCodedescription()
      print('no device found : ', end='')
      print(err)
      return

    temp_mode = cl.DeviceGetParameter(handle, TY_COMPONENT_DEVICE, TY_ENUM_TEMPERATURE_ID)
    if temp_mode.isEmpty():
        print('Temperature is not support!')
    else :
        list = temp_mode.eList()
        for idx in range(len(list)):
            mode = list[idx]
            temp = cl.DeviceControlReadTemperature(handle, cl.Value(mode))
            print('{}: {} TEMP: {}'.format(idx, cl.Description(mode), temp))

    cl.Close(handle)
    pass

if __name__=='__main__':
    main()
