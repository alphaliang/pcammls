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

    err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR)
    if err:
       print('device stream enable err:{}'.format(err))
       return
    
    color_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_COLOR)
    if len(color_fmt_list) != 0:
      print ('color image format list:')
      for idx in range(len(color_fmt_list)):
        fmt = color_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
      print('\tSelect {}'.format(fmt.getDesc()))
      cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_COLOR, color_fmt_list[len(color_fmt_list) - 1])
    else:
      print ('device has no color stream.')
      cl.Close(handle)
      return

    #enable rgb image software isp
    cl.DeviceColorStreamIspEnable(handle, True)

    rgb_image = image_data()

    cl.DeviceStreamOn(handle)

    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamRead(handle, -1)
      for i in range(len(image_list)):
        frame = image_list[i]
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

