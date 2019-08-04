from pcammls import * 
import numpy
import sys
import os


def show_version():
    ver = TY_VERSION_INFO()
    TYLibVersion(ver.this)
    print('lib version :{}.{}.{}'.format(ver.major,ver.minor,ver.patch))

def show_interface():
    print ( '===interface '+ '='*100)
    TYUpdateInterfaceList()
    ifs_num = TYGetInterfaceNumber()
    info_buff =  TY_INTERFACE_INFO_ARRAY(ifs_num)
    filled_num = TYGetInterfaceList(info_buff.cast(),ifs_num)
    print (filled_num)
    for idx in range(filled_num):
        item =  info_buff[idx]
        print('{} {}'.format(item.name,item.id))

def show_devices():
    ''' using sdk api to find devices'''
    print ('===device'+'='*100)
    TYUpdateInterfaceList()
    ifs_num = TYGetInterfaceNumber()
    if ifs_num==0:
        print ('no interface')
        return
    info_buff =  TY_INTERFACE_INFO_ARRAY(ifs_num)
    filled_num = TYGetInterfaceList(info_buff.cast(),ifs_num)
    for idx in range(filled_num):
        item =  info_buff[idx]
        ifs_handle = TYOpenInterface(item.id)
        TYUpdateDeviceList(ifs_handle);
        dev_num = TYGetDeviceNumber(ifs_handle);
        if dev_num>0:
            dev_buff = TY_DEVICE_BASE_INFO_ARRAY(dev_num);
            dev_filled_num = TYGetDeviceList(ifs_handle,dev_buff.cast(),dev_num)
            for dev_idx in range(dev_filled_num):
                dev_info = dev_buff[dev_idx]
                if dev_info.iface.type == TY_INTERFACE_USB:
                    _info  = dev_info.get_usbinfo()
                    addr_desc = '{}\t{}'.format(_info.addr,_info.bus)
                elif dev_info.iface.type == TY_INTERFACE_ETHERNET:
                    _info  = dev_info.get_netinfo()
                    addr_desc = '{}\t{}'.format(_info.ip,_info.mac)
                else:   
                    addr_desc = ''
                print('{}\t{}\t{}'.format(dev_info.id,addr_desc,dev_info.iface.id ))

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

def show_features(dev_handle):
    print('features for this device ' + '='*50)
    componets = {} 
    componets[TY_COMPONENT_DEVICE      ] = 'TY_COMPONENT_DEVICE' 
    componets[TY_COMPONENT_DEPTH_CAM   ] =  'TY_COMPONENT_DEPTH_CAM' 
    componets[TY_COMPONENT_IR_CAM_LEFT ] =  'TY_COMPONENT_IR_CAM_LEFT'
    componets[TY_COMPONENT_RGB_CAM_LEFT] =  'TY_COMPONENT_RGB_CAM_LEFT'
    componets[TY_COMPONENT_LASER       ] =  'TY_COMPONENT_LASER'
    componets[TY_COMPONENT_IMU         ] =  'TY_COMPONENT_IMU'

    #try to show some feature
    features = {}
    features[TY_STRUCT_CAM_INTRINSIC       ] = 'TY_STRUCT_CAM_INTRINSIC'  
    features[TY_STRUCT_EXTRINSIC_TO_LEFT_IR] = 'TY_STRUCT_EXTRINSIC_TO_LEFT_IR'
    features[TY_STRUCT_CAM_DISTORTION      ] = 'TY_STRUCT_CAM_DISTORTION'
    features[TY_STRUCT_CAM_CALIB_DATA      ] = 'TY_STRUCT_CAM_CALIB_DATA'
    features[TY_INT_PERSISTENT_IP          ] = 'TY_INT_PERSISTENT_IP'
    features[TY_INT_PERSISTENT_SUBMASK     ] = 'TY_INT_PERSISTENT_SUBMASK'
    features[TY_INT_PERSISTENT_GATEWAY     ] = 'TY_INT_PERSISTENT_GATEWAY'
    features[TY_BOOL_GVSP_RESEND           ] = 'TY_BOOL_GVSP_RESEND'
    features[TY_INT_PACKET_DELAY           ] = 'TY_INT_PACKET_DELAY'
    features[TY_INT_ACCEPTABLE_PERCENT     ] = 'TY_INT_ACCEPTABLE_PERCENT'
    features[TY_INT_NTP_SERVER_IP          ] = 'TY_INT_NTP_SERVER_IP'
    features[TY_STRUCT_CAM_STATISTICS      ] = 'TY_STRUCT_CAM_STATISTICS'
    features[TY_INT_WIDTH_MAX              ] = 'TY_INT_WIDTH_MAX'
    features[TY_INT_HEIGHT_MAX             ] = 'TY_INT_HEIGHT_MAX'
    features[TY_INT_OFFSET_X               ] = 'TY_INT_OFFSET_X'
    features[TY_INT_OFFSET_Y               ] = 'TY_INT_OFFSET_Y'
    features[TY_INT_WIDTH                  ] = 'TY_INT_WIDTH'
    features[TY_INT_HEIGHT                 ] = 'TY_INT_HEIGHT'
    features[TY_ENUM_IMAGE_MODE            ] = 'TY_ENUM_IMAGE_MODE'

    ids = TYGetComponentIDs(dev_handle)
    for com in componets:
        if (com&ids)==0:
            print (componets[com] + ' -- UNSUPPORT')
            continue
        else:
            print (componets[com] + ' -- OK')
            for feat in features:
                if not TYHasFeature(dev_handle,com,feat):
                    continue
                feat_info = TY_FEATURE_INFO()
                TYGetFeatureInfo(dev_handle,com,feat,feat_info.this)
                print ('\t {} ({})'.format(features[feat],feat_info.accessMode))
                if feat == TY_STRUCT_CAM_INTRINSIC: #using TYGetStruct
                    intri = TY_CAMERA_INTRINSIC()
                    TYGetStruct(dev_handle,com,feat,intri,intri.CSize())
                    print ('\t \t intri: {}'.format(intri.data))
                elif TYFeatureType(feat)==TY_FEATURE_INT:#is int feature
                    val = TYGetInt(dev_handle,com,feat)
                    print ('\t \t value: {}'.format(val))
                elif TYFeatureType(feat)==TY_FEATURE_ENUM:#is enum feature
                    enum_count = TYGetEnumEntryCount(dev_handle,com,feat)
                    if enum_count>0:
                        enum_buff =  TY_ENUM_ENTRY_ARRAY(enum_count)
                        filled = TYGetEnumEntryInfo(dev_handle,com,feat,enum_buff.cast(),enum_count)
                        print ('\t \t enum list:')
                        for idx in range(filled):
                            item = enum_buff[idx]
                            print ('\t \t \t: {} - {} '.format(item.description,item.value))


def main():
    TYInitLib()
    show_version()
    show_interface()
    show_devices()
    iface_id,dev_sn = select_device()
    if not dev_sn:
        print ('no device')
        return 
    iface_handle = TYOpenInterface(iface_id)
    dev_handle = TYOpenDevice(iface_handle,dev_sn)
    show_features(dev_handle)
    TYCloseDevice(dev_handle)
    TYCloseInterface(iface_handle)
    TYDeinitLib()
    pass



if __name__=='__main__':
    main()

