from pcammls import * 
import cv2
import numpy
import sys
import os

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

def fetch_frame_loop(handle):
    comps = TYGetComponentIDs(handle)
    TYEnableComponents(handle,TY_COMPONENT_DEPTH_CAM & comps)
    #TYEnableComponents(handle,TY_COMPONENT_RGB_CAM_LEFT & comps)
    #TYEnableComponents(handle,TY_COMPONENT_IR_CAM_LEFT)
    #TYEnableComponents(handle,TY_COMPONENT_IR_CAM_RIGHT)
    TYSetEnum(handle,TY_COMPONENT_DEPTH_CAM,TY_ENUM_IMAGE_MODE,TY_IMAGE_MODE_DEPTH16_640x480)
    sz = TYGetFrameBufferSize(handle)
    print ('buffer size:{}'.format(sz))
    if sz<0:
        print ('error size')
        return 
    buffs=[char_ARRAY(sz),char_ARRAY(sz)]
    TYEnqueueBuffer(handle,buffs[0],sz)
    TYEnqueueBuffer(handle,buffs[1],sz)

    depth_calib = TY_CAMERA_CALIB_INFO()
    ret = TYGetStruct(handle, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, depth_calib, depth_calib.CSize());
    print("Depth cam calib data:")
    print("                 {} {}".format(depth_calib.intrinsicWidth, depth_calib.intrinsicHeight))
    print("                 {}".format(depth_calib.intrinsic.data))
    print("                 {}".format(depth_calib.extrinsic.data))
    print("                 {}".format(depth_calib.distortion.data))
    
    depth_image_width  = TYGetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_WIDTH)
    depth_image_height = TYGetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_HEIGHT)
    print("Depth cam image size:{} - {}".format(depth_image_width, depth_image_height))
    
    p3d = TY_VECT_3F_ARRAY(depth_image_width*depth_image_height)
    
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
                    print('Center depth value:{}'.format(arr[depth_image_height/2][depth_image_width/2]))
                    #Map depth image to point 3d
                    TYMapDepthImageToPoint3d(depth_calib, depth_image_width, depth_image_height, uint16_t_ARRAY_FromVoidPtr(img.buffer).cast(), p3d)
                    depth_center_offset=(depth_image_height+1)*depth_image_width/2
                    print('Center p3d value:{} | {} | {}'.format(p3d[depth_center_offset].x, p3d[depth_center_offset].y, p3d[depth_center_offset].z))
                    cv2.imshow('depth', arr)
            k = cv2.waitKey(10)
            if k==ord('q'): 
                break
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

