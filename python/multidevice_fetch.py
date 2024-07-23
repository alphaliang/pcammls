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

    #register offline event
    event = PythonPercipioDeviceEvent()
    cl.DeviceRegiststerCallBackEvent(event)

    #sn list init
    sn = [0] * len(dev_list)
    for idx in range(len(dev_list)):
      sn[idx] = dev_list[idx].id

    #open device
    handle = [0] * len(dev_list)
    for i in range(len(dev_list)):
      handle[i] = cl.Open(sn[i])
      if not cl.isValidHandle(handle[i]):
        err = cl.TYGetLastErrorCodedescription()
        print('no device found : ', end='')
        print(err)
        return

    #device stream config
    for i in range(len(dev_list)):
      depth_fmt_list = cl.DeviceStreamFormatDump(handle[i], PERCIPIO_STREAM_DEPTH)
      print ('depth image format list:')
      for idx in range(len(depth_fmt_list)):
        fmt = depth_fmt_list[idx]
        print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()))
      cl.DeviceStreamFormatConfig(handle[i], PERCIPIO_STREAM_DEPTH, depth_fmt_list[0])

      err = cl.DeviceLoadDefaultParameters(handle[i])
      if err:
        print('Load default parameters fail: ', end='')
        print(cl.TYGetLastErrorCodedescription())
      else:
        print('Load default parameters successful')
      
      err = cl.DeviceStreamEnable(handle[i], PERCIPIO_STREAM_DEPTH)
      if err:
        print('device stream enable err:{}'.format(err))
        return
      
      cl.DeviceStreamOn(handle[i])

    depth_render = [0] * len(dev_list)
    for i in range(len(dev_list)):
      depth_render[i] = image_data()

    while True:
      if event.IsOffline():
        break
      
      for m in range(len(dev_list)):
        image_list = cl.DeviceStreamRead(handle[m], -1)
        for i in range(len(image_list)):
          frame = image_list[i]
          if frame.streamID == PERCIPIO_STREAM_DEPTH:
            cl.DeviceStreamDepthRender(frame, depth_render[m])
            arr = depth_render[m].as_nparray()
            cv2.imshow(sn[m],arr)

      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    for i in range(len(dev_list)):
      cl.DeviceStreamOff(handle[i])    
    
    for i in range(len(dev_list)):
      cl.Close(handle[i])
    pass

if __name__=='__main__':
    main()

