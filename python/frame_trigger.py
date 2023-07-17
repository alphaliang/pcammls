'''
Description: 
Author: zxy
Date: 2023-07-13 15:38:51
LastEditors: zxy
LastEditTime: 2023-07-17 09:21:34
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
    handle = cl.Open()
    if not cl.isValidHandle(handle):
      print('no device found')
      return

    event = PythonPercipioDeviceEvent()
    cl.DeviceRegiststerCallBackEvent(event)

    cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH)

    depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH)
    print ('depth image format list:')
    for idx in range(len(depth_fmt_list)):
        fmt = depth_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
    cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0])

    cl.DeviceControlTriggerModeEnable(handle, 1)

    depth_render = image_data()
    cl.DeviceStreamOn(handle)

    while True:
      if event.IsOffline():
        break
      
      
      cl.DeviceControlTriggerModeSendTriggerSignal(handle)
      image_list = cl.DeviceStreamRead(handle, 20000)
      for i in range(len(image_list)):
        frame = image_list[i]
        arr = frame.as_nparray()
        if frame.streamID == PERCIPIO_STREAM_DEPTH:
          cl.DeviceStreamDepthRender(frame, depth_render)
          mat_depth_render = depth_render.as_nparray()
          cv2.imshow('depth',mat_depth_render)
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

