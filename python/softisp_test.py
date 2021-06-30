#------------
#some device output is bayer format rgb .
#we can use  softISP to convert image to rgb888 format for display.
#------------

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

def int_list_2_buffer(lst):
    sz = len(lst)
    int_buf = int32_t_ARRAY(sz)
    for k in range(sz):
        int_buf[k] = lst[k]
    arr_size = 4*sz
    int8_buf = uint8_t_ARRAY.FromVoidPtr(int_buf.VoidPtr(),arr_size)
    return int8_buf,arr_size

def float_list_2_buffer(lst):
    sz = len(lst)
    float_buf = float_ARRAY(sz)
    for k in range(sz):
        float_buf[k] = lst[k]
    arr_size = 4*sz
    int8_buf = uint8_t_ARRAY.FromVoidPtr(float_buf.VoidPtr(),arr_size)
    return int8_buf,arr_size

def init_softisp(isp_handle):
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_DEBUG_LOG, 00); 
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_BLACK_LEVEL, 11);
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_BLACK_LEVEL_GAIN, 1.);
    shading_buf = float_ARRAY(9)
    uint8_arr,arr_size = float_list_2_buffer( [ 0.30890417098999026, 10.63355541229248, -6.433426856994629,
                                                0.24413758516311646, 11.739893913269043, -8.148622512817383,
                                                0.1255662441253662, 11.88359546661377, -7.865192413330078 ])
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_SHADING, uint8_arr.cast(), arr_size)
    #shading center
    uint8_arr,arr_size = int_list_2_buffer([640 , 480])
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_SHADING_CENTER, uint8_arr.cast(), arr_size)
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_CCM_ENABLE, 0);#we are not using ccm by default
    #TODO: #TYISPSetFeature(isp_handle, TY_ISP_FEATURE_CAM_DEV_HANDLE, (uint8_t*)&dev_handle, sizeof(dev_handle))
    #set compoent
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_CAM_DEV_COMPONENT, TY_COMPONENT_RGB_CAM)
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_GAMMA, 1.);
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_AUTOBRIGHT, 1)#enable auto bright control
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_ENABLE_AUTO_EXPOSURE_GAIN, 0) #disable by default
    uint8_arr,arr_size = int_list_2_buffer([1280 , 960])
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_IMAGE_SIZE, uint8_arr.cast(), arr_size);
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_INPUT_RESAMPLE_SCALE, 1.);
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_ENABLE_AUTO_WHITEBALANCE, 1); #eanble auto white balance
    #gain range
    uint8_arr,arr_size = int_list_2_buffer([15 , 255])
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_AUTO_GAIN_RANGE, uint8_arr.cast(), arr_size);

def decode_rgb(isp_handle,image):
    pixelFormat = image.pixelFormat
    arr = image.as_nparray()
    if pixelFormat == TY_PIXEL_FORMAT_YUYV:
        return cv2.cvtColor(arr,cv2.COLOR_YUV2BGR_YUYV)
    elif pixelFormat == TY_PIXEL_FORMAT_YVYU: 
        return cv2.cvtColor(arr,cv2.COLOR_YUV2BGR_YVYU)
    elif pixelFormat == TY_PIXEL_FORMAT_JPEG:
        return cv2.imdecode(arr, CV_LOAD_IMAGE_COLOR)
    elif pixelFormat != TY_PIXEL_FORMAT_BAYER8GB:
        return arr
    # follow is bayer format case
    if not isp_handle:
        print('invalid isp handle')
        return cv2.cvtColor(arr,cv2.COLOR_BayerGB2BGR)
    print('w , h: {} {}'.format(image.width,image.height))
    img_out = TY_IMAGE_DATA() 
    cbuf = uint8_t_ARRAY(image.width*image.height*3)#be care this buffer will be recycle when exiting this method
    img_out.buffer =cbuf;
    img_out.size = image.width*image.height*3
    img_out.bufferSize = image.width*image.height*3
    img_out.pixelFormat = TY_PIXEL_FORMAT_BGR;
    TYISPProcessImage(isp_handle,image,img_out)
    arr = numpy.copy(img_out.as_nparray()) # need copy for buffer . otherwise you need keep cbuf handle 
    return arr
    

def fetch_frame_loop(handle):
    isp_handle = TYISPCreate()
    init_softisp(isp_handle)
    comps = TYGetComponentIDs(handle)
    TYEnableComponents(handle,TY_COMPONENT_RGB_CAM_LEFT & comps)
    sz = TYGetFrameBufferSize(handle)
    print ('buffer size:{}'.format(sz))
    if sz<0:
        print ('error size')
        return 
    buffs=[char_ARRAY(sz),char_ARRAY(sz)]
    TYEnqueueBuffer(handle,buffs[0],sz)
    TYEnqueueBuffer(handle,buffs[1],sz)

    depth_calib = TY_CAMERA_CALIB_INFO()

    hasTriggerParam = TYHasFeature(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM)
    if hasTriggerParam == True:
        trigger = TY_TRIGGER_PARAM()
        trigger.mode = TY_TRIGGER_MODE_OFF #TY_TRIGGER_MODE_SLAVE:
        print("Set trigger mode {}".format(trigger.mode))
        TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, trigger, trigger.CSize())
        if trigger.mode == TY_TRIGGER_MODE_SLAVE:
            #for network only
            hasResend = TYHasFeature(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND)
            if hasResend == True:
                print('=== Open resend')
                TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, True)
            else:
                print('=== Not support feature TY_BOOL_GVSP_RESEND')
                
    print('start cap')
    TYStartCapture(handle)
    img_index =0 
    while True:
        frame = TY_FRAME_DATA()
        #if use the software trigger mode, need to call this API:TYSendSoftTrigger to trigger the cam device
        #TYSendSoftTrigger(handle)
        try:
            TYFetchFrame(handle,frame.this,2000)
            images = frame.image
            for img in images:
                if not img.buffer:
                    continue
                if img.componentID == TY_COMPONENT_RGB_CAM:
                    arr = decode_rgb(isp_handle,img)
                    cv2.imshow('color',arr)
            k = cv2.waitKey(10)
            if k==ord('q'): 
                break
            TYEnqueueBuffer(handle,frame.userBuffer,frame.bufferSize)
            print('{} cap ok'.format(img_index))
            img_index+=1
        except Exception as err:
            print (err)
    TYStopCapture(handle)
    TYISPRelease(isp_handle)
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

