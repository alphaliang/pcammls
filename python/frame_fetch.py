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

    color_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_COLOR)
    if len(color_fmt_list) != 0:
      print ('color image format list:')
      for idx in range(len(color_fmt_list)):
        fmt = color_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
      print('\tSelect {}'.format(fmt.getDesc()))
      cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_COLOR, color_fmt_list[0])

      color_enum_desc = TY_ENUM_ENTRY()
      cl.DeviceReadCurrentEnumData(handle, PERCIPIO_STREAM_COLOR, color_enum_desc)
      print('current color image mode  {}x{}'.format(cl.Width(color_enum_desc), cl.Height(color_enum_desc)))

      color_calib_data   = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_COLOR)
      color_calib_width  = color_calib_data.Width()
      color_calib_height = color_calib_data.Height()
      color_calib_intr   = color_calib_data.Intrinsic()
      color_calib_extr   = color_calib_data.Extrinsic()
      color_calib_dis    = color_calib_data.Distortion()
      print('color calib info:')
      print('\tcalib size       :[{}x{}]'.format(color_calib_width, color_calib_height))
      print('\tcalib intr       : {}'.format(color_calib_intr))
      print('\tcalib extr       : {}'.format(color_calib_extr))
      print('\tcalib distortion : {}'.format(color_calib_dis))

    depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH)
    if len(depth_fmt_list) != 0:
      print ('depth image format list:')
      for idx in range(len(depth_fmt_list)):
        fmt = depth_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
      print('\tSelect {}'.format(fmt.getDesc()))
      cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0])

      depth_enum_desc = TY_ENUM_ENTRY()
      cl.DeviceReadCurrentEnumData(handle, PERCIPIO_STREAM_DEPTH, depth_enum_desc)
      print('current depth image mode  {}x{}'.format(cl.Width(depth_enum_desc), cl.Height(depth_enum_desc)))

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

    err=cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR | PERCIPIO_STREAM_DEPTH)
    if err:
       print('device stream enable err:{}'.format(err))
       return

    rgb_image = image_data()
    depth_render = image_data()
    cl.DeviceStreamOn(handle)

    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamRead(handle, -1)
      for i in range(len(image_list)):
        frame = image_list[i]
        if frame.streamID == PERCIPIO_STREAM_DEPTH:
          cl.DeviceStreamDepthRender(frame, depth_render)
          arr = depth_render.as_nparray()
          cv2.imshow('depth',arr)
        if frame.streamID == PERCIPIO_STREAM_COLOR:
          cl.DeviceStreamImageDecode(frame, rgb_image)
          arr = rgb_image.as_nparray()
          cv2.imshow('color',arr)
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

