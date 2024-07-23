'''
Description: 
Author: zxy
Date: 2023-07-14 19:12:19
LastEditors: zxy
LastEditTime: 2023-07-18 12:07:14
'''
import pcammls
from pcammls import * 
import cv2
import numpy
import sys
import os

class PythonPercipioDeviceEvent(pcammls.DeviceEvent):
    Offline = False

    def __init__(self):
        pcammls.DeviceEvent.__init__(self)

    def run(self, handle, eventID):
        if eventID==TY_EVENT_DEVICE_OFFLINE:
          print('=== Event Callback: Device Offline!')
          self.Offline = True
        return 0

    def IsOffline(self):
        return self.Offline

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
      
    event = PythonPercipioDeviceEvent()
    cl.DeviceRegiststerCallBackEvent(event)

    cl.DeviceControlLaserPowerAutoControlEnable(handle, False)
    cl.DeviceControlLaserPowerConfig(handle, 80)

    err = cl.DeviceLoadDefaultParameters(handle)
    if err:
      print('Load default parameters fail: ', end='')
      print(cl.TYGetLastErrorCodedescription())
    else:
       print('Load default parameters successful')

    err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_IR_LEFT | PERCIPIO_STREAM_IR_RIGHT)
    if err:
       print('device stream enable err:{}'.format(err))
       return
    
    img_ir = image_data()
    cl.DeviceStreamOn(handle)

    while True:
      if event.IsOffline():
        break
        
      image_list = cl.DeviceStreamRead(handle, 2000)
      for i in range(len(image_list)):
        frame = image_list[i]
        if frame.streamID == PERCIPIO_STREAM_IR_LEFT:
          cl.DeviceStreamIRRender(frame, img_ir)
          arr = img_ir.as_nparray()
          cv2.imshow('leftir',arr)
        if frame.streamID == PERCIPIO_STREAM_IR_RIGHT:
          cl.DeviceStreamIRRender(frame, img_ir)
          arr = img_ir.as_nparray()
          cv2.imshow('right ir',arr)

      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

