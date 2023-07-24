#include <mutex>

#include "TYApi.h"
#include "TYCoordinateMapper.h"
#include "TYImageProc.h"
#include "../sample/common/Utils.hpp"

#include <vector>

typedef enum PERCIPIO_STREAM_LIST {
  PERCIPIO_STREAM_COLOR     = 0x00000001,
  PERCIPIO_STREAM_DEPTH     = 0x00000002,
  PERCIPIO_STREAM_IR_LEFT   = 0x00000004,
  PERCIPIO_STREAM_IR_RIGHT  = 0x00000008,
}PERCIPIO_STREAM_LIST;
typedef int PERCIPIO_STREAM_ID;


typedef struct PercipioCalibData
{
  public:
    PercipioCalibData() {};
    ~PercipioCalibData() {};
    PercipioCalibData(const TY_CAMERA_CALIB_INFO calib) {calib_data = calib;};

    int Width() const { return calib_data.intrinsicWidth; };
    int Height() const { return calib_data.intrinsicHeight; };
    const std::vector<float>   Intrinsic();
    const std::vector<float>   Extrinsic();
    const std::vector<float>   Distortion();

    const TY_CAMERA_CALIB_INFO& data() const { return calib_data; }

  private:
    TY_CAMERA_CALIB_INFO calib_data;
}PercipioCalibData;

const std::vector<float>   PercipioCalibData::Intrinsic() {
  float* ptr = calib_data.intrinsic.data;
  int cnt = sizeof(calib_data.intrinsic.data) / sizeof(float);
  return std::vector<float>(ptr, ptr + cnt);
}
const std::vector<float>   PercipioCalibData::Extrinsic() {
  float* ptr = calib_data.extrinsic.data;
  int cnt = sizeof(calib_data.extrinsic.data) / sizeof(float);
  return std::vector<float>(ptr, ptr + cnt);
}

const std::vector<float>   PercipioCalibData::Distortion() {
  float* ptr = calib_data.distortion.data;
  int cnt = sizeof(calib_data.distortion.data) / sizeof(float);
  return std::vector<float>(ptr, ptr + cnt);
}

typedef struct image_data {
  PERCIPIO_STREAM_ID streamID;

  int           timestamp;      ///< Timestamp in microseconds
  int           imageIndex;     ///< image index used in trigger mode
  int           status;         ///< Status of this buffer
  int           size;           ///< Buffer size
  void*         buffer;         ///< Pointer to data buffer
  int           width;          ///< Image width in pixels
  int           height;         ///< Image height in pixels
  int           pixelFormat;    ///< Pixel format, see TY_PIXEL_FORMAT_LIST

  image_data() {
    memset(this, 0, sizeof(image_data));
  }

  image_data(const TY_IMAGE_DATA& image) {
    timestamp = image.timestamp;
    imageIndex = image.imageIndex;
    status = image.status;
    size = image.size;
  
    width = image.width;
    height = image.height;
    pixelFormat =image.pixelFormat;

    switch(image.componentID) {
      case TY_COMPONENT_RGB_CAM:
        streamID = PERCIPIO_STREAM_COLOR;
        break;
      case TY_COMPONENT_DEPTH_CAM:
        streamID = PERCIPIO_STREAM_DEPTH;
        break;
      case TY_COMPONENT_IR_CAM_LEFT:
        streamID = PERCIPIO_STREAM_IR_LEFT;
        break;
      case TY_COMPONENT_IR_CAM_RIGHT:
        streamID = PERCIPIO_STREAM_IR_RIGHT;
        break;
      default:
        streamID = PERCIPIO_STREAM_COLOR;
        break;
    }

    if(size) {
      buffer = new char[size];
      memcpy(buffer, image.buffer, size);
    } else {
      buffer = NULL;
    }
  }

  image_data(const image_data & d) {
    streamID = d.streamID;
    timestamp = d.timestamp;
    imageIndex = d.imageIndex;
    status = d.status;
    size = d.size;
  
    width = d.width;
    height = d.height;
    pixelFormat =d.pixelFormat;

    if(size) {
      buffer = new char[size];
      memcpy(buffer, d.buffer, size);
    } else {
      buffer = NULL;
    }
  }

  bool resize(const int sz) {
    if(buffer) 
      delete []buffer;

    if(sz) 
      buffer = new char[sz];
    else
      buffer = NULL;

    size = sz;
    return true;
  }

  image_data& operator=(const image_data& d) {
    this->streamID = d.streamID;
    this->timestamp = d.timestamp;
    this->imageIndex = d.imageIndex;
    this->status = d.status;
    this->size = d.size;
  
    this->width = d.width;
    this->height = d.height;
    this->pixelFormat =d.pixelFormat;

    this->resize(d.size);

    if(d.size)
      memcpy(this->buffer, d.buffer, d.size);
    return *this;
  }

  ~image_data() {
    if(buffer) {
      delete []buffer;
      buffer = NULL;
    }
  }

  void* Ptr() {
    return buffer;
  }

}image_data;

#include "image_process.hpp"

typedef struct pointcloud_data {
  TY_VECT_3F p3d;

  pointcloud_data() {
    p3d.x = NAN;
    p3d.y = NAN;
    p3d.z = NAN;
  }

  pointcloud_data(const TY_VECT_3F& v) {
    p3d.x = v.x;
    p3d.y = v.y;
    p3d.z = v.z;
  }

  float getX() { return p3d.x; }
  float getY() { return p3d.y; }
  float getZ() { return p3d.z; }
}pointcloud_data;

typedef struct pointcloud_data_list {
  
  pointcloud_data_list() {
    memset(this, 0, sizeof(pointcloud_data_list));
  }

  pointcloud_data_list(const pointcloud_data_list& list) {
    cnt = list.cnt;
    p3d = new TY_VECT_3F[cnt];
    memcpy(&p3d[0], &list.p3d[0], cnt * sizeof(pointcloud_data));
  };

  ~pointcloud_data_list() {
    if(p3d) {
      delete []p3d;
      p3d = NULL;
    }
  }

  void resize(int size) {
    if(p3d) {
      delete []p3d;
    }

    p3d = new TY_VECT_3F[size];
    cnt = size;
  }

  int size() {
    return cnt;
  }

  pointcloud_data get_value(int idx) {
    if(idx < cnt)
      return pointcloud_data(p3d[idx]);
    else
      return pointcloud_data();
  }

  void* getPtr() {
    return p3d;
  }

  private:
    int cnt;
    TY_VECT_3F* p3d;
}pointcloud_data_list;

struct DeviceEvent {
  virtual int run(void* handle, TY_EVENT event_id) = 0;
  virtual ~DeviceEvent() {}
};

typedef DeviceEvent* DeviceEventHandle;

static DeviceEventHandle handler_ptr = NULL;
static int handler_execute(TY_DEV_HANDLE handle, TY_EVENT event_id) {
  // Make the call up to the target language when handler_ptr
  // is an instance of a target language director class
    if (handler_ptr != NULL)
        return handler_ptr->run(handle, event_id);
    else
        return -1;
}

void user_device_event_callback(TY_DEV_HANDLE handle, TY_EVENT_INFO evt) {
  handler_execute(handle, evt.eventId);
}

void percipio_device_callback(TY_EVENT_INFO *event_info, void *userdata) {
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        // Note: 
        //     Please set TY_BOOL_KEEP_ALIVE_ONOFF feature to false if you need to debug with breakpoint!
        TY_DEV_HANDLE handle = (TY_DEV_HANDLE)userdata;
        TY_EVENT_INFO _event = *event_info;
        user_device_event_callback(handle, _event);
    }
    else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) {
        LOGD("=== Event Callback: License Error!");
    }
}

class PercipioSDK
{
  public:
    PercipioSDK();
    ~PercipioSDK();

    std::vector<TY_DEVICE_BASE_INFO>& ListDevice();

    TY_DEV_HANDLE Open();
    TY_DEV_HANDLE Open(const char* sn);
    TY_DEV_HANDLE OpenDeviceByIP(const char* ip);
        bool isValidHandle(const TY_DEV_HANDLE handle);
    void Close(const TY_DEV_HANDLE handle);

    bool DeviceRegiststerCallBackEvent(DeviceEventHandle handler);

    /*stream control*/
    bool                                DeviceStreamEnable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    bool                                DeviceStreamDisable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    const std::vector<TY_ENUM_ENTRY>&   DeviceStreamFormatDump(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    
    bool                                DeviceStreamFormatConfig(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream, const TY_ENUM_ENTRY fmt);
        int  Width(const TY_ENUM_ENTRY fmt);
        int  Height(const TY_ENUM_ENTRY fmt);

    bool  DeviceStreamOn(const TY_DEV_HANDLE handle);
    const std::vector<image_data>& DeviceStreamRead(const TY_DEV_HANDLE handle, int timeout);
    bool DeviceStreamOff(const TY_DEV_HANDLE handle);


    /*read calib data*/
    PercipioCalibData&          DeviceReadCalibData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    const float                 DeviceReadCalibDepthScaleUnit(const TY_DEV_HANDLE handle);

    /*device control*/
    bool                        DeviceControlTriggerModeEnable(const TY_DEV_HANDLE handle, const int enable);
    bool                        DeviceControlTriggerModeSendTriggerSignal(const TY_DEV_HANDLE handle);
    bool                        DeviceControlLaserPowerAutoControlEnable(const TY_DEV_HANDLE handle, bool enable);
    bool                        DeviceControlLaserPowerConfig(const TY_DEV_HANDLE handle, int laser);

    /** stream control*/
    bool DeviceStreamDepthRender(const image_data& src, image_data& dst);
    bool DeviceStreamIRRender(const image_data& src, image_data& dst);
    bool DeviceStreamImageDecode(const image_data& src, image_data& dst);
    bool DeviceStreamMapDepthImageToPoint3D(const image_data& depth, const PercipioCalibData& calib_data, float scale, pointcloud_data_list& p3d);
    bool DeviceStreamDoUndistortion(const TY_CAMERA_CALIB_INFO& calib_data, const image_data& src, image_data& dst);
    bool DeviceStreamMapDepthImageToColorCoordinate(const TY_CAMERA_CALIB_INFO& depth_calib, const int depthW, const int depthH, const float scale, 
                                                    const image_data& srcDepth, 
                                                    const TY_CAMERA_CALIB_INFO& color_calib, const int targetW, const int targetH, 
                                                    image_data& dstDepth);

  private:
    std::mutex _mutex;


    typedef enum STREAM_FMT_LIST_IDX {
        STREMA_FMT_IDX_COLOR      = 0,
        STREMA_FMT_IDX_DEPTH      = 1,
        STREMA_FMT_IDX_IR_LEFT    = 2,
        STREMA_FMT_IDX_IR_RIGHT   = 3,
        STREMA_FMT_IDX_MAX        = 4,
      }STREAM_FMT_LIST_IDX;

    typedef struct device_info {
      TY_DEV_HANDLE       handle;
      std::string         devID;
      PERCIPIO_STREAM_ID  stream;
      float               depth_scale_unit;
      char*               frameBuffer[2];

      std::vector<TY_ENUM_ENTRY> fmt_list[STREMA_FMT_IDX_MAX];
      std::vector<image_data>    image_list;
      PercipioCalibData          calib_data_list[STREMA_FMT_IDX_MAX];

      device_info(const TY_DEV_HANDLE _handle, const char* id) {
        frameBuffer[0] = NULL;
        frameBuffer[1] = NULL;
        for(size_t i = 0; i < STREMA_FMT_IDX_MAX; i++) {
          fmt_list[i].clear();
          memset(&calib_data_list[i], 0, sizeof(PercipioCalibData));
        }
        depth_scale_unit = 1.f;
        handle = _handle;
        devID = std::string(id);
        image_list.clear();
      }

      ~device_info() {
      }

    }device_info;

    std::vector< TY_DEVICE_BASE_INFO>  iDevBaseInfoList;
    std::vector< TY_INTERFACE_HANDLE>  iFaceList;

    int isValidDevice(const char* sn);

    void AddInterface(const TY_INTERFACE_HANDLE iface);
    void AddDevice(const TY_DEV_HANDLE hanlde, const char* sn);

    std::vector<device_info>  DevList;

    int stream_idx(const PERCIPIO_STREAM_ID stream);
    
    int hasDevice(const TY_DEV_HANDLE handle);
    void ConfigDevice(const TY_DEV_HANDLE handle);
    void DumpDeviceInfo(const TY_DEV_HANDLE handle);
    bool FrameBufferAlloc(TY_DEV_HANDLE handle, unsigned int frameSize);
    void FrameBufferRelease(TY_DEV_HANDLE handle) ;
};

void PercipioSDK::AddInterface(const TY_INTERFACE_HANDLE iface) {
    for (size_t i = 0; i < iFaceList.size(); i++) {
        if (iFaceList[i] == iface)
            return;
    }
    iFaceList.push_back(iface);
}

void PercipioSDK::AddDevice(const TY_DEV_HANDLE hanlde, const char* sn) {
	DevList.push_back(device_info(hanlde, sn));
}

int PercipioSDK::stream_idx(const PERCIPIO_STREAM_ID stream) {

  int compIDX;
  switch(stream) {
    case PERCIPIO_STREAM_COLOR:
      compIDX = STREMA_FMT_IDX_COLOR;
      break;
    case PERCIPIO_STREAM_DEPTH:
      compIDX = STREMA_FMT_IDX_DEPTH;
      break;
    case PERCIPIO_STREAM_IR_LEFT:
      compIDX = STREMA_FMT_IDX_IR_LEFT;
      break;
    case PERCIPIO_STREAM_IR_RIGHT:
      compIDX = STREMA_FMT_IDX_IR_RIGHT;
      break;
    default:
      LOGE("stream mode not support : %d", stream);
      return STREMA_FMT_IDX_MAX;
  }
  return compIDX;
}

PercipioSDK::PercipioSDK() {
  TYInitLib();
}

PercipioSDK::~PercipioSDK() {
  for (size_t i = 0; i < iFaceList.size(); i++) {
      TYCloseInterface(iFaceList[i]);
  }
  TYDeinitLib();
}

std::vector<TY_DEVICE_BASE_INFO>& PercipioSDK::ListDevice()
{
  iDevBaseInfoList.clear();
  selectDevice(TY_INTERFACE_ALL, "", "", 10, iDevBaseInfoList);  
  return iDevBaseInfoList;
}

int PercipioSDK::isValidDevice(const char* sn) {
  size_t sz = iDevBaseInfoList.size();
  if(sz) {
    if(sn == NULL) {
      return 0;
    } 
    else {
      //search
      for(size_t i = 0; i < sz; i++) {
        if(0 == strcmp(iDevBaseInfoList[i].id, sn)) {
          return i;
        }
      }
    }
  }

  return -1;
}

TY_DEV_HANDLE PercipioSDK::Open(const char* sn) {
  std::unique_lock<std::mutex> lock(_mutex);
  std::string SN;
  if(sn != NULL)
    SN = std::string(sn);
  else
    SN = std::string("");

  TY_STATUS status = TY_STATUS_OK;
  TY_INTERFACE_HANDLE hIface = NULL;
  TY_DEV_HANDLE hDevice = NULL;

  TY_DEVICE_BASE_INFO selectedDev;
  int idx = isValidDevice(sn);
  if(idx < 0) {
    std::vector<TY_DEVICE_BASE_INFO> selected;
    status = selectDevice(TY_INTERFACE_ALL, SN, "", 10, selected);
    if(status != TY_STATUS_OK) {
      return 0;
    }

    if(!selected.size()) {
      return 0;
    }

    selectedDev = selected[0];
  } else {
    selectedDev = iDevBaseInfoList[idx];
  }
  
  
  status = TYOpenInterface(selectedDev.iface.id, &hIface);
  if(status != TY_STATUS_OK) {
    LOGE("TYOpenInterface failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return 0;
  }

  status = TYOpenDevice(hIface, selectedDev.id, &hDevice);
  if(status != TY_STATUS_OK) {
    LOGE("TYOpenDevice failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return 0;
  }

  AddInterface(hIface);
  AddDevice(hDevice, selectedDev.id);
  LOGD("Device %s is on!", selectedDev.id);

  TYRegisterEventCallback(hDevice, percipio_device_callback, hDevice);

  ConfigDevice(hDevice);
  
  DumpDeviceInfo(hDevice);

  return hDevice;
}

TY_DEV_HANDLE PercipioSDK::OpenDeviceByIP(const char* ip) {
  std::unique_lock<std::mutex> lock(_mutex);
  LOGD("Update interface list");
  ASSERT_OK( TYUpdateInterfaceList() );

  uint32_t n = 0;
  ASSERT_OK( TYGetInterfaceNumber(&n) );
  LOGD("Got %u interface list", n);
  if(n == 0){
    LOGE("interface number incorrect");
    return NULL;
  }

  TY_INTERFACE_HANDLE hIface = NULL;
  TY_DEV_HANDLE hDevice = NULL;

  std::vector<TY_INTERFACE_INFO> ifaces(n);
  ASSERT_OK( TYGetInterfaceList(&ifaces[0], n, &n) );
  ASSERT( n == ifaces.size() );
  for(uint32_t i = 0; i < n; i++){
    if(TYIsNetworkInterface(ifaces[i].type)){
      ASSERT_OK( TYOpenInterface(ifaces[i].id, &hIface) );
      if (TYOpenDeviceWithIP(hIface, ip, &hDevice) == TY_STATUS_OK) {

        TY_DEVICE_BASE_INFO device_base_info;
        TYGetDeviceInfo           (hDevice, &device_base_info);
        DevList.push_back(device_info(hDevice, device_base_info.id));
        LOGD("Device %s is on!", device_base_info.id);

        TYRegisterEventCallback(hDevice, percipio_device_callback, hDevice);

        ConfigDevice(hDevice);

        DumpDeviceInfo(hDevice);
      }
    }
  }

  return hDevice;
}

TY_DEV_HANDLE PercipioSDK::Open() {
  return Open(NULL);
}

int PercipioSDK::hasDevice(const TY_DEV_HANDLE handle) {
  for(size_t i = 0; i < DevList.size(); i++) {
    if(handle == DevList[i].handle) {
      return i;
    }
  }
  return -1;
}

bool PercipioSDK::isValidHandle(const TY_DEV_HANDLE handle) {
  if(handle && (hasDevice(handle) >= 0))
      return true;

  return false;
}

void PercipioSDK::ConfigDevice(const TY_DEV_HANDLE handle) {
  TY_TRIGGER_PARAM trigger;
  trigger.mode = TY_TRIGGER_MODE_OFF;
  
  TY_STATUS status = TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger));
  if(status != TY_STATUS_OK) {
    LOGE("TYSetStruct failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
  }

  bool hasResend = false;
  TYHasFeature(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, &hasResend);
  if (hasResend) {
      TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, false);
  } else {
    LOGW("=== Not support feature TY_BOOL_GVSP_RESEND");
  }

  TYSetBool(handle, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, true);
  TYSetInt(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, 100);

  return ;
}

void PercipioSDK::DumpDeviceInfo(const TY_DEV_HANDLE handle) {
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("DumpDeviceInfo failed: invalid handle %s:%d", __FILE__, __LINE__);
    return ;
  }

  //DUMP STREAM FMT LIST
  static TY_COMPONENT_ID compID[STREMA_FMT_IDX_MAX] = {
      TY_COMPONENT_RGB_CAM,
      TY_COMPONENT_DEPTH_CAM,
      TY_COMPONENT_IR_CAM_LEFT,
      TY_COMPONENT_IR_CAM_RIGHT
  };

  TY_STATUS status;
  for(size_t cnt = 0; cnt < STREMA_FMT_IDX_MAX; cnt++) {
    unsigned int n = 0;
    status = TYGetEnumEntryCount(handle, compID[cnt], TY_ENUM_IMAGE_MODE, &n);
    if(status != TY_STATUS_OK) {
      LOGE("TYGetEnumEntryCount failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
      DevList[idx].fmt_list[cnt].clear();
      continue;
    }

    if (n == 0){
      DevList[idx].fmt_list[cnt].clear();
      continue;
    }

    DevList[idx].fmt_list[cnt].resize(n);

    TY_ENUM_ENTRY* pEntry = &DevList[idx].fmt_list[cnt][0];
    status = TYGetEnumEntryInfo(handle, compID[cnt], TY_ENUM_IMAGE_MODE, pEntry, n, &n);
    if(status != TY_STATUS_OK) {
      LOGE("TYGetEnumEntryInfo failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
      DevList[idx].fmt_list[cnt].clear();
    }
  }

  //DUMP STREAM CALIB_DATA LIST
  for(size_t cnt = 0; cnt < STREMA_FMT_IDX_MAX; cnt++) {
    TY_CAMERA_CALIB_INFO calib_data;
    status = TYGetStruct(handle, compID[cnt], TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(TY_CAMERA_CALIB_INFO));
    if(status != TY_STATUS_OK) {
      LOGE("TYGetStruct failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
      memset(&DevList[idx].calib_data_list[cnt], 0, sizeof(TY_CAMERA_CALIB_INFO));
    }

    DevList[idx].calib_data_list[cnt] = PercipioCalibData(calib_data);
  }

  //DUMP DEPTH SCAL UNIT
  float scale_unit = 1.f;
  status = TYGetFloat(handle, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
  if(status != TY_STATUS_OK) {
    LOGE("TYGetFloat failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
  }
  DevList[idx].depth_scale_unit = scale_unit;

  //
  return;
}

bool PercipioSDK::DeviceRegiststerCallBackEvent(DeviceEventHandle handler) {
  handler_ptr = handler;
  handler = NULL;
  return true;
}

bool PercipioSDK::DeviceStreamEnable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {
  TY_STATUS status;
  TY_COMPONENT_ID allComps;
  status = TYGetComponentIDs(handle, &allComps);
  if(status != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }

  if(stream & PERCIPIO_STREAM_COLOR) {
    if(allComps & TY_COMPONENT_RGB_CAM) {
      status = TYEnableComponents(handle, TY_COMPONENT_RGB_CAM);
      if(status != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
        return false;
      }
    } else {
      LOGE("The device does not support color stream.\n");
      return false;
    }
  } else {
    TYDisableComponents(handle, TY_COMPONENT_RGB_CAM);
  }

  if(stream & PERCIPIO_STREAM_DEPTH) {
    if(allComps & TY_COMPONENT_DEPTH_CAM) {
      status = TYEnableComponents(handle, TY_COMPONENT_DEPTH_CAM);
      if(status != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
        return false;
      }
    } else {
      LOGE("The device does not support depth stream.\n");
      return false;
    }
  } else {
    TYDisableComponents(handle, TY_COMPONENT_DEPTH_CAM);
  }

  if(stream & PERCIPIO_STREAM_IR_LEFT) {
    if(allComps & TY_COMPONENT_IR_CAM_LEFT) {
      status = TYEnableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
      if(status != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
        return false;
      }
    } else {
      LOGE("The device does not support left ir stream.\n");
      return false;
    }
  } else {
    TYDisableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
  }

  if(stream & PERCIPIO_STREAM_IR_RIGHT) {
    if(allComps & TY_COMPONENT_IR_CAM_RIGHT) {
      status = TYEnableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
      if(status != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
        return false;
      }
    } else {
      LOGE("The device does not support right ir stream.\n");
      return false;
    }
  } else {
    TYDisableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
  }

  return true;
}

bool PercipioSDK::DeviceStreamDisable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {

  if(stream & PERCIPIO_STREAM_COLOR) {
    TYDisableComponents(handle, TY_COMPONENT_RGB_CAM);
  }

  if(stream & PERCIPIO_STREAM_DEPTH) {
    TYDisableComponents(handle, TY_COMPONENT_DEPTH_CAM);
  }

  if(stream & PERCIPIO_STREAM_IR_LEFT) {
    TYDisableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
  }

  if(stream & PERCIPIO_STREAM_IR_RIGHT) {
    TYDisableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
  }
  return true;
}

const std::vector<TY_ENUM_ENTRY>& PercipioSDK::DeviceStreamFormatDump(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {
  static std::vector<TY_ENUM_ENTRY>  invalid_enum;
  int compIDX = stream_idx(stream);
  if(compIDX == STREMA_FMT_IDX_MAX) {
      return invalid_enum;
  }

  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return invalid_enum;
  }

  return DevList[idx].fmt_list[compIDX];
}

int  PercipioSDK::Width(const TY_ENUM_ENTRY fmt) {
  return TYImageWidth(fmt.value);
}

int  PercipioSDK::Height(const TY_ENUM_ENTRY fmt) {
  return TYImageHeight(fmt.value);
}

bool PercipioSDK::DeviceStreamFormatConfig(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream, const TY_ENUM_ENTRY fmt) {

  TY_COMPONENT_ID compID;
  switch(stream) {
    case PERCIPIO_STREAM_COLOR:
      compID = TY_COMPONENT_RGB_CAM;
      break;
    case PERCIPIO_STREAM_DEPTH:
      compID = TY_COMPONENT_DEPTH_CAM;
      break;
    case PERCIPIO_STREAM_IR_LEFT:
      compID = TY_COMPONENT_IR_CAM_LEFT;
      break;
    case PERCIPIO_STREAM_IR_RIGHT:
      compID = TY_COMPONENT_IR_CAM_RIGHT;
      break;
    default:
      LOGE("stream mode not support : %d", stream);
      return false;
  }

  TY_STATUS status = TYSetEnum(handle, compID, TY_ENUM_IMAGE_MODE, fmt.value);
  if(status != TY_STATUS_OK) {
    LOGE("Stream fmt is not support!");
    return false;
  }

  return true;
}

bool PercipioSDK::FrameBufferAlloc(TY_DEV_HANDLE handle, unsigned int frameSize) {
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  if(DevList[idx].frameBuffer[0])
    free(DevList[idx].frameBuffer[0]);
  if(DevList[idx].frameBuffer[1])
    free(DevList[idx].frameBuffer[1]);
  DevList[idx].frameBuffer[0] = (void*)malloc(frameSize);
  DevList[idx].frameBuffer[1] = (void*)malloc(frameSize);

  ASSERT_OK(TYEnqueueBuffer(handle, DevList[idx].frameBuffer[0], frameSize));
  ASSERT_OK(TYEnqueueBuffer(handle, DevList[idx].frameBuffer[1], frameSize));

  return true;
}

void PercipioSDK::FrameBufferRelease(TY_DEV_HANDLE handle) {
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return ;
  }

  TYClearBufferQueue(handle);

  free(DevList[idx].frameBuffer[0]);
  free(DevList[idx].frameBuffer[1]);
  DevList[idx].frameBuffer[0] = NULL;
  DevList[idx].frameBuffer[1] = NULL;

  return ;
}

PercipioCalibData& PercipioSDK::DeviceReadCalibData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {

  static PercipioCalibData  invalid_calib_data;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return invalid_calib_data;
  }

  int compIDX = stream_idx(stream);
  if(compIDX == STREMA_FMT_IDX_MAX) {
      return invalid_calib_data;
  }

  return DevList[idx].calib_data_list[compIDX];
}

const float PercipioSDK::DeviceReadCalibDepthScaleUnit(const TY_DEV_HANDLE handle) {

  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return NAN;
  }

  return DevList[idx].depth_scale_unit;
}

bool PercipioSDK::DeviceStreamOn(const TY_DEV_HANDLE handle) {
  std::unique_lock<std::mutex> lock(_mutex);
  if(hasDevice(handle) < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  unsigned int frameSize;
  TY_STATUS status = TYGetFrameBufferSize(handle, &frameSize);
  if(status != TY_STATUS_OK) {
    LOGE("TYGetFrameBufferSize failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }

  if(!FrameBufferAlloc(handle, frameSize)) {
    LOGE("====FrameBufferAlloc fail!\n");
    return false;
  }

  status = TYStartCapture(handle);
  if(status != TY_STATUS_OK) {
    LOGE("TYStartCapture failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }

  LOGD("Stream ON!");
  return true;
}

const std::vector<image_data>& PercipioSDK::DeviceStreamRead(const TY_DEV_HANDLE handle, int timeout) {
  static std::vector<image_data> INVALID_FRAME(0);
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return INVALID_FRAME;
  }

  TY_FRAME_DATA frame;
  DevList[idx].image_list.clear();
  TY_STATUS status = TYFetchFrame(handle, &frame, timeout);
  if(status != TY_STATUS_OK) {
    LOGE("TYFetchFrame failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return INVALID_FRAME;
  }

  for (int i = 0; i < frame.validCount; i++) {
    if (frame.image[i].status != TY_STATUS_OK) continue;

    // get depth image
    if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
      DevList[idx].image_list.push_back(image_data(frame.image[i]));
    }
    
    // get left ir image
    if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT) {
      DevList[idx].image_list.push_back(image_data(frame.image[i]));
    }
    
    // get right ir image
    if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT) {
      DevList[idx].image_list.push_back(image_data(frame.image[i]));
    }

    // get BGR
    if (frame.image[i].componentID == TY_COMPONENT_RGB_CAM) {
      DevList[idx].image_list.push_back(image_data(frame.image[i]));
    }
  }

  status = TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
  if(status != TY_STATUS_OK) {
    LOGE("TYEnqueueBuffer failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
  }
  return DevList[idx].image_list;
}

bool PercipioSDK::DeviceStreamOff(const TY_DEV_HANDLE handle) {
  std::unique_lock<std::mutex> lock(_mutex);
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  TY_STATUS status = TYStopCapture(handle);
  if(status != TY_STATUS_OK) {
    LOGE("TYStopCapture failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }

  LOGD("Stream OFF!");
  FrameBufferRelease(handle);

  return true;
}

bool PercipioSDK::DeviceControlTriggerModeEnable(const TY_DEV_HANDLE handle, const int enable) {

  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  TY_TRIGGER_PARAM trigger;
  if(enable)
    trigger.mode = TY_TRIGGER_MODE_SLAVE;
  else
    trigger.mode = TY_TRIGGER_MODE_OFF;
  TY_STATUS status = TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger));
  if(status != TY_STATUS_OK) {
    LOGE("TYSetStruct failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }

  bool hasResend = false;
  TYHasFeature(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, &hasResend);
  if (hasResend) {
    LOGD("=== Open resend");
    if(enable)
      TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, true);
    else
      TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, false);
  } else {
    LOGW("=== Not support feature TY_BOOL_GVSP_RESEND");
  }

  return true;
}

bool PercipioSDK::DeviceControlTriggerModeSendTriggerSignal(const TY_DEV_HANDLE handle) {

  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  TY_STATUS status;
  while(true) {
    status = TYSendSoftTrigger(handle);
    if(status != TY_STATUS_BUSY)
      break;
  }

  if(status != TY_STATUS_OK) {
    LOGE("TYSendSoftTrigger failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }
  return true;
}

bool PercipioSDK::DeviceControlLaserPowerAutoControlEnable(const TY_DEV_HANDLE handle, bool enable) {
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  TY_STATUS status = TYSetBool(handle, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, enable);
  if(status != TY_STATUS_OK) {
    LOGE("TYSetBool failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }
  return true;
}

bool PercipioSDK::DeviceControlLaserPowerConfig(const TY_DEV_HANDLE handle, int laser) {

  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  TY_STATUS status = TYSetInt(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, laser);
  if(status != TY_STATUS_OK) {
    LOGE("TYSetInt failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }
  return true;
}

static bool parseCsiRaw10(const image_data& src, image_data& dst) {
  int width = src.width;
  int height = src.height;
  if(width & 0x3) {
    LOGE("Invalid ir stream size : %d x %d\n", src.width, src.height);
    return false;
  }

  dst.streamID     = src.streamID;
  dst.timestamp    = src.timestamp;
  dst.imageIndex   = src.imageIndex;
  dst.status       = src.status;
  dst.width        = src.width;
  dst.height       = src.height;
  dst.pixelFormat  = TY_PIXEL_FORMAT_MONO16;
  dst.resize(dst.width * dst.height * 2);

  uint16_t* src_ptr = (uint16_t*)src.buffer;
  uint16_t* dst_ptr = (uint16_t*)dst.buffer;
  int raw10_line_size = 5 * width / 4;
  for(size_t i = 0, j = 0; i < raw10_line_size * height; i+=5, j+=4)
  {
    //[A2 - A9] | [B2 - B9] | [C2 - C9] | [D2 - D9] | [A0A1-B0B1-C0C1-D0D1]
    dst_ptr[j + 0] = ((uint16_t)src_ptr[i + 0] << 2) | ((src_ptr[i + 4] & 0x3)  >> 0);
    dst_ptr[j + 1] = ((uint16_t)src_ptr[i + 1] << 2) | ((src_ptr[i + 4] & 0xc)  >> 2);
    dst_ptr[j + 2] = ((uint16_t)src_ptr[i + 2] << 2) | ((src_ptr[i + 4] & 0x30) >> 4);
    dst_ptr[j + 3] = ((uint16_t)src_ptr[i + 3] << 2) | ((src_ptr[i + 4] & 0xc0) >> 6);
  }
  return true;
}

static bool parseCsiRaw12(const image_data& src, image_data& dst) {
  int width = src.width;
  int height = src.height;
  if(width & 0x1) {
    LOGE("Invalid ir stream size : %d x %d\n", src.width, src.height);
    return false;
  }

  dst.streamID     = src.streamID;
  dst.timestamp    = src.timestamp;
  dst.imageIndex   = src.imageIndex;
  dst.status       = src.status;
  dst.width        = src.width;
  dst.height       = src.height;
  dst.pixelFormat  = TY_PIXEL_FORMAT_MONO16;
  dst.resize(dst.width * dst.height * 2);

  uint16_t* src_ptr = (uint16_t*)src.buffer;
  uint16_t* dst_ptr = (uint16_t*)dst.buffer;
  int raw12_line_size = 3 * width / 2;
  for(size_t i = 0, j = 0; i < raw12_line_size * height; i+=3, j+=2)
  {
    //[A4 - A11] | [B4 - B11] | [A0A1A2A3-B0B1B2B3]
    dst_ptr[j + 0] = ((uint16_t)src_ptr[i + 0] << 4) | ((src_ptr[i + 2] & 0x0f)  >> 0);
    dst_ptr[j + 1] = ((uint16_t)src_ptr[i + 1] << 4) | ((src_ptr[i + 2] & 0xf0)  >> 4);
  }
  return true;
}

bool PercipioSDK::DeviceStreamIRRender(const image_data& src, image_data& dst) {
    bool ret;
    image_data  temp, gray8;
    if (src.pixelFormat == TY_PIXEL_FORMAT_MONO16 || src.pixelFormat == TY_PIXEL_FORMAT_TOF_IR_MONO16) {
        ImgProc::cvtColor(src, ImgProc::IMGPROC_MONO162RGB888, dst);
    }
    else if (src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO10) {
        ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_MONO102RGB888, dst);
    }
    else if (src.pixelFormat == TY_PIXEL_FORMAT_MONO) {
        ImgProc::cvtColor(src, ImgProc::IMGPROC_MONO2RGB888, dst);
    }
    else if (src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO12) {
        ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_MONO122RGB888, dst);
    }
    else {
        LOGE("Invalid ir stream pixel format : %d\n", src.pixelFormat);
        return false;
    }

    return true;
}

static bool parseIrFrame(const image_data& src, image_data& dst) {
  if (src.pixelFormat == TY_PIXEL_FORMAT_MONO16 || src.pixelFormat==TY_PIXEL_FORMAT_TOF_IR_MONO16){
    dst = src;
    return true;
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO10) {
    //target: TY_PIXEL_FORMAT_MONO16
    return parseCsiRaw10(src, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_MONO) {
    //target: TY_PIXEL_FORMAT_MONO
    dst = src;
    return true;
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO12) {
    //target: TY_PIXEL_FORMAT_MONO16
    return parseCsiRaw12(src, dst);
  } 
  else {
    LOGE("Invalid ir stream pixel format : %d\n", src.pixelFormat);
	  return false;
  }
}

static bool parseColorFrame(const image_data& src, image_data& dst) {

  //TODO
  if (src.pixelFormat == TY_PIXEL_FORMAT_JPEG){
    ImgProc::cvtColor(src, ImgProc::IMGPROC_JPEG2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_YVYU) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_YVYU2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_YUYV) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_YUYV2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_RGB) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BGR2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_BGR){
    dst = src;
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_BAYER8GBRG) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8GB2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_BAYER8BGGR) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8BG2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_BAYER8GRBG) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8GR2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_BAYER8RGGB) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8RG2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_MONO){
    ImgProc::cvtColor(src, ImgProc::IMGPROC_MONO2RGB888, dst);
  } 
  
  else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10GBRG) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10GB2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10BGGR) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10BG2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10GRBG) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10GR2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10RGGB) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10RG2RGB888, dst);
  } 
  
  
  else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12GBRG) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12GB2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12BGGR) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12BG2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12GRBG) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12GR2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12RGGB) {
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12RG2RGB888, dst);
  }

  return true;
}

bool PercipioSDK::DeviceStreamDepthRender(const image_data& src, image_data& dst) {
  if(src.pixelFormat != TY_PIXEL_FORMAT_DEPTH16){
    LOGE("Invalid pixel format 0x:%x", src.pixelFormat);
    return false;
  }

  ImgProc::cvtColor(src, ImgProc::IMGPROC_DEPTH2RGB888, dst);
  return true;
}

bool PercipioSDK::DeviceStreamImageDecode(const image_data& src, image_data& dst) {

  if (src.streamID == PERCIPIO_STREAM_DEPTH) {
    if (src.pixelFormat == TY_PIXEL_FORMAT_XYZ48) {
      dst = src;
      return true;
    } else if(src.pixelFormat == TY_PIXEL_FORMAT_DEPTH16){
      dst = src;
      return true;
    } else {
      LOGE("Invalid depth stream pixel format : 0x%x\n", src.pixelFormat);
      return false;
    }
  }
  // get left ir image
  else if (src.streamID == PERCIPIO_STREAM_IR_LEFT) {
    return parseIrFrame(src, dst);
  }
  // get right ir image
  else if (src.streamID == PERCIPIO_STREAM_IR_RIGHT) {
    return parseIrFrame(src, dst);
  }
  // get BGR
  else if (src.streamID == PERCIPIO_STREAM_COLOR) {
    return parseColorFrame(src, dst);
  }
  else {
    LOGE("Invalid stream id : %d\n", src.streamID);
    return false;
  }
}

bool PercipioSDK::DeviceStreamMapDepthImageToPoint3D(const image_data& depth, const PercipioCalibData& calib_data, float scale, pointcloud_data_list& p3d) {
  if(depth.streamID != PERCIPIO_STREAM_DEPTH) {
    LOGE("Invalid stream data: %d.", depth.streamID);
      return false;
  }

  int size = depth.width * depth.height;
  p3d.resize(size);

  TYMapDepthImageToPoint3d(&calib_data.data(),
                                     depth.width, depth.height,
                                     (const uint16_t*)depth.buffer,
                                     (TY_VECT_3F*)p3d.getPtr(), 
                                     scale);
  return true;
}

bool PercipioSDK::DeviceStreamDoUndistortion(const TY_CAMERA_CALIB_INFO& calib_data, const image_data& src, image_data& dst) {

  bool is_support = false;
  static int fmt_support_list[] = {
    TY_PIXEL_FORMAT_MONO,
    TY_PIXEL_FORMAT_DEPTH16,
    TY_PIXEL_FORMAT_MONO16,
    TY_PIXEL_FORMAT_RGB,
    TY_PIXEL_FORMAT_BGR,
    TY_PIXEL_FORMAT_RGB48,
    TY_PIXEL_FORMAT_BGR48
  };
  for(size_t i = 0; i < sizeof(fmt_support_list) / sizeof(int); i++) {
    if(src.pixelFormat == fmt_support_list[i]) {
      is_support = true;
      break;
    }
  }

  if(!is_support) {
    LOGE("src stream pixel format is not support!");
    return false;
  }

  dst.resize(src.size);
  dst.streamID     = src.streamID;
  dst.timestamp    = src.timestamp;
  dst.imageIndex   = src.imageIndex;
  dst.status       = src.status;
  dst.width        = src.width;
  dst.height       = src.height;
  dst.pixelFormat  = src.pixelFormat;
  
  TY_IMAGE_DATA  src_image, dst_image;
  src_image.timestamp   = src.timestamp;
  src_image.imageIndex  = src.imageIndex;
  src_image.status      = src.status;
  src_image.size        = src.size;
  src_image.buffer      = src.buffer;
  src_image.width       = src.width;
  src_image.height      = src.height;
  src_image.pixelFormat = src.pixelFormat;

  dst_image.timestamp   = dst.timestamp;
  dst_image.imageIndex  = dst.imageIndex;
  dst_image.status      = dst.status;
  dst_image.size        = dst.size;
  dst_image.buffer      = dst.buffer;
  dst_image.width       = dst.width;
  dst_image.height      = dst.height;
  dst_image.pixelFormat = dst.pixelFormat;

  TY_STATUS status = TYUndistortImage(&calib_data, &src_image, NULL, &dst_image);
  if(status != TY_STATUS_OK) {
    LOGE("TYUndistortImage failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILE__, __LINE__);
    return false;
  }
  return true;
}

bool PercipioSDK::DeviceStreamMapDepthImageToColorCoordinate(const TY_CAMERA_CALIB_INFO& depth_calib, const int depthW, const int depthH, const float scale, 
                                                    const image_data& srcDepth, 
                                                    const TY_CAMERA_CALIB_INFO& color_calib, const int targetW, const int targetH, 
                                                    image_data& dstDepth)
{
  if(srcDepth.streamID != PERCIPIO_STREAM_DEPTH) {
    LOGE("Invalid stream data: %d.", srcDepth.streamID);
    return false;
  }

  if(srcDepth.size != 2*depthW*depthH) {
    LOGE("Invalid stream size: %d.", srcDepth.size);
    return false;
  }

  dstDepth.resize(targetW * targetH * 2);
  dstDepth.streamID     = PERCIPIO_STREAM_DEPTH;
  dstDepth.timestamp    = srcDepth.timestamp;
  dstDepth.imageIndex   = srcDepth.imageIndex;
  dstDepth.status       = srcDepth.status;
  dstDepth.width        = targetW;
  dstDepth.height       = targetH;
  dstDepth.pixelFormat  = srcDepth.pixelFormat;
  TYMapDepthImageToColorCoordinate(&depth_calib, depthW, depthH, (const uint16_t*)srcDepth.buffer,  &color_calib, targetW, targetH, (uint16_t*)dstDepth.buffer, scale);
  return true;
}

void PercipioSDK::Close(const TY_DEV_HANDLE handle)
{
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return ;
  }

  LOGD("Device %s is close!", DevList[idx].devID.c_str());
  DevList.erase(DevList.begin() + idx);
  
  TYCloseDevice(handle);
}