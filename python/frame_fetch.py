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

def main():
    cl = PercipioSDK()
    handle = cl.Open()
    if not cl.isValidHandle(handle):
      print('no device found')

    #cl.DeviceRegisterEventCallBack(handle, devEventCallback)

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

    color_calib_width  = cl.DeviceReadCalibWidth(handle, PERCIPIO_STREAM_COLOR)
    color_calib_height = cl.DeviceReadCalibHeight(handle, PERCIPIO_STREAM_COLOR)
    color_calib_intr   = cl.DeviceReadCalibIntrinsicArray(handle, PERCIPIO_STREAM_COLOR)
    color_calib_extr   = cl.DeviceReadCalibExtrinsicArray(handle, PERCIPIO_STREAM_COLOR)
    color_calib_dis    = cl.DeviceReadCalibDistortionArray(handle, PERCIPIO_STREAM_COLOR)
    print('color calib info:')
    print('\tcalib size       :[{}x{}]'.format(color_calib_width, color_calib_height))
    print('\tcalib intr       : {}'.format(color_calib_intr))
    print('\tcalib extr       : {}'.format(color_calib_extr))
    print('\tcalib distortion : {}'.format(color_calib_dis))

    depth_calib_width  = cl.DeviceReadCalibWidth(handle, PERCIPIO_STREAM_DEPTH)
    depth_calib_height = cl.DeviceReadCalibHeight(handle, PERCIPIO_STREAM_DEPTH)
    depth_calib_intr   = cl.DeviceReadCalibIntrinsicArray(handle, PERCIPIO_STREAM_DEPTH)
    depth_calib_extr   = cl.DeviceReadCalibExtrinsicArray(handle, PERCIPIO_STREAM_DEPTH)
    depth_calib_dis    = cl.DeviceReadCalibDistortionArray(handle, PERCIPIO_STREAM_DEPTH)
    print('delth calib info:')
    print('\tcalib size       :[{}x{}]'.format(depth_calib_width, depth_calib_height))
    print('\tcalib intr       : {}'.format(depth_calib_intr))
    print('\tcalib extr       : {}'.format(depth_calib_extr))
    print('\tcalib distortion : {}'.format(depth_calib_dis))

    cl.DeviceStreamOn(handle)

    while True:
      image_list = cl.DeviceStreamFetch(handle, 2000)
      for i in range(len(image_list)):
        frame = image_list[i]
        arr = frame.as_nparray()
        if frame.streamID == PERCIPIO_STREAM_DEPTH:
          depthu8 =  cv2.convertScaleAbs(arr, alpha=(255.0/4000.0))
          cv2.imshow('depth',depthu8)
        if frame.streamID == PERCIPIO_STREAM_COLOR:
          arr = decode_rgb(frame.pixelFormat,arr)
          cv2.imshow('color',arr)
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

if __name__=='__main__':
    main()

