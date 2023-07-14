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
    handle = cl.Open()
    if not cl.isValidHandle(handle):
      print('no device found')
      return
      
    event = PythonPercipioDeviceEvent()
    cl.DeviceRegiststerCallBackEvent(event)

    cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_IR_LEFT | PERCIPIO_STREAM_IR_RIGHT)

    cl.DeviceControlLaserPowerAutoControlEnable(handle, False)
    cl.DeviceControlLaserPowerConfig(handle, 80)

    img_ir = image_data()
    cl.DeviceStreamOn(handle)

    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamRead(handle, 2000)
      for i in range(len(image_list)):
        frame = image_list[i]
        cl.DeviceStreamImageDecode(frame, img_ir)
        arr = img_ir.as_nparray()
        if frame.streamID == PERCIPIO_STREAM_IR_LEFT:
          cv2.imshow('left it',arr)
        if frame.streamID == PERCIPIO_STREAM_IR_RIGHT:
          cv2.imshow('right ir',arr)
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

