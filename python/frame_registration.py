'''
Description: 
Author: zxy
Date: 2023-07-14 09:48:00
LastEditors: zxy
LastEditTime: 2023-07-14 10:37:54
'''
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

    cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR | PERCIPIO_STREAM_DEPTH)

    color_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_COLOR)
    print ('color image format list:')
    for idx in range(len(color_fmt_list)):
        fmt = color_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
    cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_COLOR, color_fmt_list[0])

    depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH)
    print ('depth image format list:')
    for idx in range(len(depth_fmt_list)):
        fmt = depth_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
    cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0])

    scale_unit = cl.DeviceReadCalibDepthScaleUnit(handle)
    print ('depth image scale unit :{}'.format(scale_unit))

    depth_calib = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_DEPTH)
    color_calib = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_COLOR)

    cl.DeviceStreamOn(handle)
    registration_depth = image_data()
    
    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamRead(handle, 2000)
      for i in range(len(image_list)):
        frame = image_list[i]
        if frame.streamID == PERCIPIO_STREAM_DEPTH:
          arr_depth = frame
        if frame.streamID == PERCIPIO_STREAM_COLOR:
          arr_color = frame

      cl.DeviceStreamMapDepthImageToColorCoordinate(depth_calib.data(), arr_depth.width, arr_depth.height, scale_unit,  arr_depth,  color_calib.data(), arr_color.width, arr_color.height, registration_depth)
      arr = registration_depth.as_nparray()
      depthu8 = cv2.convertScaleAbs(arr, alpha=(255.0/4000.0))
      cv2.imshow('registration',depthu8)
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

