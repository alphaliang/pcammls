from pcammls import * 
import cv2
import numpy
import sys
import os
import numpy as np
from ctypes import *

def select_device():
    ''' a simple way to get device like original sdk  '''
    argv = sys.argv
    sn = ''
    for idx in range(len(argv)):
        if argv[idx]=='-sn' and idx<len(argv)-1:
            sn = argv[idx+1]
            break
    dev_list = selectDevice(TY_INTERFACE_ALL,sn,'',10)
    print ('device found:')
    for idx in range(len(dev_list)):
        dev = dev_list[idx]
        print ('{} -- {} \t {}'.format(idx,dev.id,dev.iface.id))
    if  len(dev_list)==0:
        return None,None
    if len(dev_list) == 1 and sn!='':
        selected_idx = 0 
    else:
        selected_idx  = int(input('select a device:'))
    if selected_idx < 0 or selected_idx >= len(dev_list):
        return None,None
    dev = dev_list[selected_idx]
    return dev.iface.id, dev.id

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

def fetch_frame_loop(handle):
    comps = TYGetComponentIDs(handle)
    TYEnableComponents(handle,TY_COMPONENT_DEPTH_CAM & comps)
    TYEnableComponents(handle,TY_COMPONENT_RGB_CAM & comps)
    TYSetEnum(handle,TY_COMPONENT_DEPTH_CAM,TY_ENUM_IMAGE_MODE,TY_IMAGE_MODE_DEPTH16_640x480)
    sz = TYGetFrameBufferSize(handle)
    print ('buffer size:{}'.format(sz))
    if sz<0:
        print ('error size')
        return 
    buffs=[char_ARRAY(sz),char_ARRAY(sz)]
    TYEnqueueBuffer(handle,buffs[0],sz)
    TYEnqueueBuffer(handle,buffs[1],sz)
    
    depth_image_width  = TYGetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_WIDTH)
    depth_image_height = TYGetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_HEIGHT)
    color_image_width  = TYGetInt(handle, TY_COMPONENT_RGB_CAM, TY_INT_WIDTH)
    color_image_height = TYGetInt(handle, TY_COMPONENT_RGB_CAM, TY_INT_HEIGHT)
    print("color image size:                 {} {}".format(color_image_width, color_image_height))

    #After aligning to the color image coordinate
    #the resolution of the depth is the same as the color image
    p3d = TY_VECT_3F_ARRAY(color_image_width*color_image_height)

    dst_color_buffer = uint8_t_ARRAY(color_image_width * color_image_height * 3)
    dst_depth_buffer = uint16_t_ARRAY(color_image_width * color_image_height)

    depth_calib = TY_CAMERA_CALIB_INFO()
    color_calib = TY_CAMERA_CALIB_INFO()
    ret = TYGetStruct(handle, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, depth_calib, depth_calib.CSize())
    ret = TYGetStruct(handle, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, color_calib, color_calib.CSize())
    print("Depth cam calib data:")
    print("size:                 {} {}".format(depth_calib.intrinsicWidth, depth_calib.intrinsicHeight))
    print("intrinsic             {}".format(depth_calib.intrinsic.data))
    print("Color cam calib data:")
    print("size:                 {} {}".format(color_calib.intrinsicWidth, color_calib.intrinsicHeight))
    print("intrinsic             {}".format(color_calib.intrinsic.data))
    print("extrinsic             {}".format(color_calib.extrinsic.data))
    print("distortion            {}".format(color_calib.distortion.data))

    scale_unit = 1.0
    if TYHasFeature(handle,TY_COMPONENT_DEPTH_CAM,TY_FLOAT_SCALE_UNIT):
        scale_unit = TYGetFloat(handle, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT)
    print("Depth cam image scale uint:{}".format(scale_unit))
 
    print('start cap')
    TYStartCapture(handle)
    img_index =0 
    while True:
        frame = TY_FRAME_DATA()
        try:
            TYFetchFrame(handle,frame.this,2000)
            images = frame.image
            for img in images:
                if not img.buffer:
                    continue
                arr = img.as_nparray()
                if img.componentID == TY_COMPONENT_DEPTH_CAM:
                     arr_depth = arr
                if img.componentID == TY_COMPONENT_RGB_CAM:
                    arr_rgb = decode_rgb(img.pixelFormat,arr)
            
            sp = arr_rgb.shape
            src_color_buffer = uint8_t_ARRAY.from_nparray(arr_rgb)
            src_rgb = TYInitImageData(sp[0] * sp[1] * 3, src_color_buffer, sp[1], sp[0])
            src_rgb.pixelFormat = TY_PIXEL_FORMAT_RGB
            src_test_rgb = src_rgb.as_nparray()

            dst_rgb = TYInitImageData(sp[0] * sp[1] * 3, dst_color_buffer, sp[1], sp[0])
            dst_rgb.pixelFormat = TY_PIXEL_FORMAT_RGB

            dst_intr = TY_CAMERA_INTRINSIC()
            dst_intr.resize(color_calib.intrinsic, 1.0 * color_image_width / color_calib.intrinsicWidth, 1.0 * color_image_height / color_calib.intrinsicHeight)
            TYUndistortImage(color_calib, src_rgb, dst_intr, dst_rgb) 
            undistort_rgb = dst_rgb.as_nparray()
            #display color image(after do undistort)
            cv2.imshow('undistort_rgb',undistort_rgb)

            #Map depth image to color image coordinate
            src_depth_buffer = uint16_t_ARRAY.from_nparray(arr_depth)                     
            TYMapDepthImageToColorCoordinate(depth_calib, depth_image_width, depth_image_height, src_depth_buffer.cast(), color_calib, color_image_width, color_image_height, dst_depth_buffer.cast(), scale_unit)

            TYMapDepthImageToPoint3d(color_calib, color_image_width, color_image_height, dst_depth_buffer.cast(), p3d, 1.0)
            depth_center_y = color_image_height / 2
            depth_center_x = color_image_width / 2
            depth_center_offset=(color_image_height+1)*color_image_width/2
            print('Center color:{} '.format(undistort_rgb[depth_center_y][depth_center_x]))
            print('Center p3d value:{} | {} | {}'.format(p3d[int(depth_center_offset)].x, p3d[int(depth_center_offset)].y, p3d[int(depth_center_offset)].z))
            
            #display depth image(after aligning to the color image coordinate)
            dst_depth16 = TYInitImageData(color_image_width * color_image_height * 2, dst_depth_buffer, color_image_width, color_image_height)
            dst_depth16.pixelFormat = TY_PIXEL_FORMAT_DEPTH16
            dst_depth16_arr = dst_depth16.as_nparray()
            dst_depthu8 =  cv2.convertScaleAbs(dst_depth16_arr, alpha=(255.0/4000.0))
            cv2.imshow('registration_depth',dst_depthu8)

            k = cv2.waitKey(10)
            if k==ord('q'): 
                break

            uint8_t_ARRAY.Release(src_color_buffer)
            uint16_t_ARRAY.Release(src_depth_buffer)
            #uint8_t_ARRAY_ReleasePtr(src_color_buffer)
            #uint16_t_ARRAY_ReleasePtr(src_depth_buffer)

            TYEnqueueBuffer(handle,frame.userBuffer,frame.bufferSize)
            print('{} cap ok'.format(img_index))
            img_index+=1
        except Exception as err:
            print (err)
    TYStopCapture(handle)
    print('done')

def main():
    TYInitLib()
    iface_id,dev_sn = select_device()
    if not dev_sn:
        print ('no device')
        return 
    iface_handle = TYOpenInterface(iface_id)
    dev_handle = TYOpenDevice(iface_handle,dev_sn)
    fetch_frame_loop(dev_handle)
    TYCloseDevice(dev_handle)
    TYCloseInterface(iface_handle)
    TYDeinitLib()
    pass



if __name__=='__main__':
    main()

