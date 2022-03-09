#------------
# some device output is bayer format rgb .
# we can use  softISP to convert image to rgb888 format for display.
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
    dev_list = selectDevice(TY_INTERFACE_ALL,sn,'',20)
    #TODO remove follow line
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

def isp_write_int_list_feature(handle , feat, lst):
    sz = len(lst)
    int_buf = int32_t_ARRAY(sz)
    for k in range(sz):
        int_buf[k] = lst[k]
    arr_size = 4*sz
    int8_buf = uint8_t_ARRAY.FromVoidPtr(int_buf.VoidPtr(),arr_size)
    TYISPSetFeature(handle, feat, int8_buf.cast(), arr_size) 
    uint8_t_ARRAY.ReleasePtr(int8_buf);

def isp_write_float_list_feature(handle , feat, lst):
    sz = len(lst)
    float_buf = float_ARRAY(sz)
    for k in range(sz):
        float_buf[k] = lst[k]
    arr_size = 4*sz
    int8_buf = uint8_t_ARRAY.FromVoidPtr(float_buf.VoidPtr(),arr_size)
    TYISPSetFeature(handle, feat, int8_buf.cast(), arr_size) 
    uint8_t_ARRAY.ReleasePtr(int8_buf);

def isp_read_int_list_feature(handle,feat,sz):
    arr_sz = sz*4
    cbuf = uint8_t_ARRAY(arr_sz)
    TYISPGetFeature(handle,feat,cbuf.cast(),arr_sz)
    int_buf = int32_t_ARRAY.FromVoidPtr(cbuf.VoidPtr(),sz)
    res = []
    for k in range(sz):
        res.append(int_buf[k])
    int32_t_ARRAY.ReleasePtr(int_buf);
    return res    

def show_isp_supported_feat(handle):
    cbuf=int32_t_ARRAY(1)
    TYISPGetFeatureInfoListSize(handle,cbuf.cast())
    sz = cbuf[0]
    if sz<=0:
        print('got 0 size')
        return 
    item_buf = TY_ISP_FEATURE_INFO_ARRAY(sz)
    TYISPGetFeatureInfoList(handle, item_buf.cast(),sz)
    for k in range(sz):
        item = item_buf[k]
        print('name:{} \t\ttype:{}'.format(item.name,item.value_type))

def init_softisp(isp_handle,dev_handle):
    #TYISPSetFeature(isp_handle, TY_ISP_FEATURE_DEBUG_LOG, 00); 
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_BLACK_LEVEL, 11);
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_BLACK_LEVEL_GAIN, 1.);
    shading =  [0]*9 #default no shading correction.. 
    isp_write_float_list_feature(isp_handle,TY_ISP_FEATURE_SHADING,shading)
    #shading center
    isp_write_int_list_feature(isp_handle , TY_ISP_FEATURE_SHADING_CENTER, [640 , 480])
    print('read center {}'.format(isp_read_int_list_feature(isp_handle , TY_ISP_FEATURE_SHADING_CENTER,2)))
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_CCM_ENABLE, 0);#we are not using ccm by default
    #set TY_ISP_FEATURE_CAM_DEV_HANDLE by helper function  TYISPSetFeature(isp_handle, TY_ISP_FEATURE_CAM_DEV_HANDLE, (uint8_t*)&dev_handle, sizeof(dev_handle));
    PY_TYISPSetDeviceHandle(isp_handle,dev_handle)
    #set compoent
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_CAM_DEV_COMPONENT, TY_COMPONENT_RGB_CAM)
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_GAMMA, 1.);
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_AUTOBRIGHT, 1)#enable auto bright control
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_ENABLE_AUTO_EXPOSURE_GAIN, 0) #disable by default
    isp_write_int_list_feature(isp_handle , TY_ISP_FEATURE_IMAGE_SIZE , [1280 , 960])
    #print('read iamge size {}'.format(isp_read_int_list_feature(isp_handle , TY_ISP_FEATURE_IMAGE_SIZE,2)))
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_INPUT_RESAMPLE_SCALE, 1);
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_ENABLE_AUTO_WHITEBALANCE, 1); #eanble auto white balance
    #gain range
    int_range = TY_INT_RANGE()
    TYGetIntRange(dev_handle, TY_COMPONENT_RGB_CAM, TY_INT_R_GAIN,  int_range.this );
    print('cam gain range {} {}'.format( int_range.min , int_range.max))
    isp_write_int_list_feature(isp_handle , TY_ISP_FEATURE_AUTO_GAIN_RANGE, [int_range.min , int_range.max])
    TYSetInt(dev_handle, TY_COMPONENT_RGB_CAM , TY_INT_R_GAIN,int_range.min);
    TYSetInt(dev_handle, TY_COMPONENT_RGB_CAM , TY_INT_G_GAIN,int_range.min);
    TYSetInt(dev_handle, TY_COMPONENT_RGB_CAM , TY_INT_B_GAIN,int_range.min);
    TYGetIntRange(dev_handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME,  int_range.this );
    print('cam exposure range {} {}'.format( int_range.min , int_range.max))
    int_range.max = int((int_range.max - int_range.min) * 0.7 + int_range.min) #limit range
    #exposure range (actual supported range should read from device)
    TYISPSetFeature(isp_handle, TY_ISP_FEATURE_ENABLE_AUTO_EXPOSURE_GAIN, 1) 
    isp_write_int_list_feature(isp_handle , TY_ISP_FEATURE_AUTO_EXPOSURE_RANGE, [ int_range.min, int_range.max])
    #TYSetInt(dev_handle, TY_COMPONENT_RGB_CAM_LEFT , TY_INT_EXPOSURE_TIME,int_range.max);


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
    img_out = TY_IMAGE_DATA() 
    buf_sz = image.width*image.height*3
    cbuf = uint8_t_ARRAY(buf_sz)#be care this buffer will be recycle when exiting this method
    img_out = TYInitImageData(buf_sz,cbuf.VoidPtr(),image.width,image.height) #create output image 
    TYISPProcessImage(isp_handle,image,img_out)
    arr = numpy.copy(img_out.as_nparray()) # need copy for buffer . otherwise you need keep cbuf handle 
    return arr
    

def fetch_frame_loop(handle):
    isp_handle = TYISPCreate()
    show_isp_supported_feat(isp_handle)
    init_softisp(isp_handle,handle)
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
            TYISPUpdateDevice(isp_handle);
            TYEnqueueBuffer(handle,frame.userBuffer,frame.bufferSize)
            if img_index%10 == 0:
                print('frame {} cap ok'.format(img_index))
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

