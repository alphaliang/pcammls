'''
Description: 
Author: zxy
Date: 2023-07-18 09:55:47
LastEditors: zxy
LastEditTime: 2023-12-28 15:49:28
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

    depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH)
    if len(depth_fmt_list) == 0:
      print ('device has no depth stream.')
      return
      
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

    err = cl.DeviceLoadDefaultParameters(handle)
    if err:
      print('Load default parameters fail: ', end='')
      print(cl.TYGetLastErrorCodedescription())
    else:
       print('Load default parameters successful')

    scale_unit = cl.DeviceReadCalibDepthScaleUnit(handle)
    print ('depth image scale unit :{}'.format(scale_unit))

    err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH)
    if err:
       print('device stream enable err:{}'.format(err))
       return
    
    cl.DeviceStreamOn(handle)

    pointcloud_data_arr = pointcloud_data_list()
    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamRead(handle, -1)
      
      for i in range(len(image_list)):
        frame = image_list[i]
        if frame.streamID == PERCIPIO_STREAM_DEPTH:
          cl.DeviceStreamMapDepthImageToPoint3D(frame, depth_calib_data, scale_unit, pointcloud_data_arr)
          sz = pointcloud_data_arr.size()
          print('get p3d size : {}'.format(sz))
          center = frame.width * frame.height / 2 + frame.width / 2

          #show p3d arr data
          p3d_nparray = pointcloud_data_arr.as_nparray()
          cv2.imshow('p3d',p3d_nparray)

          p3d = pointcloud_data_arr.get_value(int(center))
          print('\tp3d data : {} {} {}'.format(p3d.getX(), p3d.getY(), p3d.getZ()))
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

