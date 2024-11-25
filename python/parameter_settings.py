'''
Description: 
Author: zxy
Date: 2023-07-14 09:48:00
LastEditors: zxy
LastEditTime: 2024-11-25 11:36:57
'''
import pcammls
from pcammls import * 
import cv2
import numpy
import sys
import os


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
    
    err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR)
    if err:
       print('device stream enable err:{}'.format(err))
       return
    
    #bool:color aec
    aec = cl.DeviceGetParameter(handle, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE)
    if aec.isEmpty():
        print('aec is not support!')
    else :
        print('current aec status : {}'.format(aec.toBool()))

        #disable color aec
        aec = cl.DevParamFromBool(False)
        err = cl.DeviceSetParameter(handle, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE, aec)
        print('aec close result : ', end='')
        print(err)

    #int:color exposure time
    exp = cl.DeviceGetParameter(handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME)
    if exp.isEmpty():
        print('exposure time is not support!')
    else :
        print('current exposure time status : {}, range : {} - {}, inc : {}'.format(exp.toInt(), exp.mMin(), exp.mMax(), exp.mInc()))

        exposure_time  = int(input('Enter exposure time:'))
        exp = cl.DevParamFromInt(exposure_time)
        err = cl.DeviceSetParameter(handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, exp)
        print('set color exposure time result : ', end='')
        print(err)

    image_mode = cl.DeviceGetParameter(handle, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE)
    if image_mode.isEmpty():
        print('color image mode is not support!')
    else :
        list = image_mode.eList()
        for idx in range(len(list)):
            mode = list[idx]
            print('{}: {}x{} - {}'.format(idx, cl.Width(mode), cl.Height(mode), cl.Description(mode)))

        index = int(input('Enter image mode index:'))
        image_mode = cl.DevParamFromEnum(cl.Value(list[index]))
        err = cl.DeviceSetParameter(handle, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, image_mode)

    cl.DeviceStreamOn(handle)
    img_parsed_color        = image_data()
    while True:
        image_list = cl.DeviceStreamRead(handle, 2000)
        for i in range(len(image_list)):
            frame = image_list[i]
            if frame.streamID == PERCIPIO_STREAM_COLOR:
                img_color = frame
        
            cl.DeviceStreamImageDecode(img_color, img_parsed_color)
            mat_undistortion_color = img_parsed_color.as_nparray()
            cv2.imshow('rgb', mat_undistortion_color)

        k = cv2.waitKey(10)
        if k==ord('q'): 
          break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

