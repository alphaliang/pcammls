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

    scale_unit = cl.DeviceReadCalibDepthScaleUnit(handle)
    print ('depth image scale unit :{}'.format(scale_unit))

    cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH)

    depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH)
    print ('depth image format list:')
    for idx in range(len(depth_fmt_list)):
        fmt = depth_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
    cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0])

    depth_calib_data   = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_DEPTH)
    depth_calib_width  = depth_calib_data.Width()
    depth_calib_height = depth_calib_data.Height()
    depth_calib_intr   = depth_calib_data.Intrinsic()
    depth_calib_extr   = depth_calib_data.Extrinsic()
    depth_calib_dis    = depth_calib_data.Distortion()
    print('delth calib info:')
    print('\tcalib size       :[{}x{}]'.format(depth_calib_width, depth_calib_height))
    print('\tcalib intr       : {}'.format(depth_calib_intr))
    print('\tcalib extr       : {}'.format(depth_calib_extr))
    print('\tcalib distortion : {}'.format(depth_calib_dis))

    cl.DeviceStreamOn(handle)

    pointcloud_data_arr = pointcloud_data_list()
    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamFetch(handle, -1)
      for i in range(len(image_list)):
        frame = image_list[i]
        arr = frame.as_nparray()
        if frame.streamID == PERCIPIO_STREAM_DEPTH:
          cl.DeviceStreamMapDepthImageToPoint3D(frame, depth_calib_data, scale_unit, pointcloud_data_arr)
          sz = pointcloud_data_arr.size()
          print('get p3d size : {}'.format(sz))
          center = frame.width * frame.height / 2 + frame.width / 2
          p3d = pointcloud_data_arr.get_value(int(center))
          print('\tp3d data : {} {} {}'.format(p3d.getX(), p3d.getY(), p3d.getZ()))

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

