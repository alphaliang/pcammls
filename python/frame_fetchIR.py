import pcammls
from pcammls import * 
import cv2
import numpy
import sys
import os

def decode_rgb(pixelFormat,image):
    if pixelFormat == TY_PIXEL_FORMAT_YUYV:
        return cv2.cvtColor(image,cv2.COLOR_YUV2BGR_YUYV)
    if pixelFormat == TY_PIXEL_FORMAT_YVYU: 
        return cv2.cvtColor(image,cv2.COLOR_YUV2BGR_YVYU)
    if pixelFormat == TY_PIXEL_FORMAT_BAYER8GB:
        return cv2.cvtColor(image,cv2.COLOR_BayerGB2BGR)
    if pixelFormat == TY_PIXEL_FORMAT_BAYER8BG:
        return cv2.cvtColor(image,cv2.COLOR_BayerBG2BGR)
    if pixelFormat == TY_PIXEL_FORMAT_BAYER8GR:
        return cv2.cvtColor(image,cv2.COLOR_BayerGR2BGR)
    if pixelFormat == TY_PIXEL_FORMAT_BAYER8RG:
        return cv2.cvtColor(image,cv2.COLOR_BayerRG2BGR)
    if pixelFormat == TY_PIXEL_FORMAT_JPEG:
        return cv2.imdecode(image, cv2.IMREAD_COLOR)
    if pixelFormat == TY_PIXEL_FORMAT_MONO:
        return image
    return image

class PythonPercipioDeviceEvent(pcammls.DeviceEvent):
    Offline = False

    # Define Python class 'constructor'
    def __init__(self):
        # Call C++ base class constructor
        pcammls.DeviceEvent.__init__(self)

    # Override C++ method: virtual int handle(int a, int b) = 0;
    def run(self, handle, eventID):
        # Return the product
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

    cl.DeviceStreamOn(handle)

    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamRead(handle, 2000)
      for i in range(len(image_list)):
        frame = image_list[i]
        arr = frame.as_nparray()
        if frame.streamID == PERCIPIO_STREAM_IR_LEFT:
          leftIR = decode_rgb(frame.pixelFormat,arr)
          cv2.imshow('left it',leftIR)
        if frame.streamID == PERCIPIO_STREAM_IR_RIGHT:
          rightIR = decode_rgb(frame.pixelFormat,arr)
          cv2.imshow('right ir',rightIR)
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

