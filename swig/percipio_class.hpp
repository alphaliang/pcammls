#include <mutex>

#include "TYApi.h"
#include "TYCoordinateMapper.h"
#include "TYImageProc.h"
#include "Utils.hpp"
#include "huffman.h"
#include "../sample/common/BayerISP.hpp"

#include <vector>

#define __FILENAME__ (strrchr("/" __FILE__, '/') + 1)

#define TYLIB_API_ASSERT_OK(x)    do{ \
    int err = (x); \
    if(err != TY_STATUS_OK) { \
      LOGE("Assert failed: error %d(%s) at %s:%d", err, TYErrorString(err), __FILENAME__, __LINE__); \
      LOGE("    : " #x ); \
      abort(); \
    } \
  }while(0)

typedef enum PERCIPIO_STREAM_LIST {
  PERCIPIO_STREAM_COLOR     = 0x00000001,
  PERCIPIO_STREAM_DEPTH     = 0x00000002,
  PERCIPIO_STREAM_IR_LEFT   = 0x00000004,
  PERCIPIO_STREAM_IR_RIGHT  = 0x00000008,
}PERCIPIO_STREAM_LIST;
typedef int PERCIPIO_STREAM_ID;

typedef struct PercipioRectifyIntrData
{
  public:
    PercipioRectifyIntrData() {};
    ~PercipioRectifyIntrData() {};
    PercipioRectifyIntrData(const TY_CAMERA_INTRINSIC intr) {intr_data = intr;};

    const std::vector<float>   Data();
  private:
      TY_CAMERA_INTRINSIC intr_data;
}PercipioRectifyIntrData;

const std::vector<float>   PercipioRectifyIntrData::Data() {
  float* ptr = intr_data.data;
  int cnt = sizeof(intr_data.data) / sizeof(float);
  return std::vector<float>(ptr, ptr + cnt);
}

typedef struct PercipioRectifyRotaData
{
  public:
    PercipioRectifyRotaData() {};
    ~PercipioRectifyRotaData() {};
    PercipioRectifyRotaData(const TY_CAMERA_ROTATION intr) {roata_data = intr;};

    const std::vector<float>   Data();
  private:
    TY_CAMERA_ROTATION roata_data;
}PercipioRectifyRotaData;

const std::vector<float>   PercipioRectifyRotaData::Data() {
  float* ptr = roata_data.data;
  int cnt = sizeof(roata_data.data) / sizeof(float);
  return std::vector<float>(ptr, ptr + cnt);
}


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

  uint64_t      timestamp;      ///< Timestamp in microseconds
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

    if(sz) {
      buffer = new char[sz];
      memset(buffer, 0, size);
    } else
      buffer = NULL;

    size = sz;
    return true;
  }

  image_data& operator=(const image_data& d) {
    if(this != &d) {
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
    }
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
    _width = list._width;
    _height = list._height;
    p3d = new TY_VECT_3F[_width*_height];
    memcpy(&p3d[0], &list.p3d[0], _width*_height * sizeof(pointcloud_data));
  };

  ~pointcloud_data_list() {
    if(p3d) {
      delete []p3d;
      p3d = NULL;
    }
  }

  void resize(int w, int h) {
    if(p3d) {
      delete []p3d;
    }

    p3d = new TY_VECT_3F[w*h];
    _width = w;
    _height = h;
  }

  int size() {
    return _width * _height;
  }

  int width() {
    return _width;
  }

  int height() {
    return _height;
  }

  pointcloud_data get_value(int idx) {
    if(idx < (_width * _height))
      return pointcloud_data(p3d[idx]);
    else
      return pointcloud_data();
  }

  void* getPtr() {
    return p3d;
  }

  private:
    int _width;
    int _height;
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

typedef struct PercipioAecROI
{
    int32_t  _x;
    int32_t  _y;
    int32_t  _w;
    int32_t  _h;

    PercipioAecROI(int32_t x, int32_t y, int32_t w, int32_t h) {
      _x = x;
      _y = y;
      _w = w;
      _h = h;
    }
};

typedef struct DevParamByteArray
{
  unsigned char m_data[1024];
  int real_size;
};

typedef struct DevParamStruct
{
  int m_data[1024];
  int real_size;
};

typedef struct DevParamDataInt
{
  int value;
  TY_INT_RANGE  range;
};

typedef struct DevParamDataEnum
{
  int value;

  int entryCount; 
  TY_ENUM_ENTRY list[100];
};

typedef struct DevParamDataFloat
{
  float value;
  TY_FLOAT_RANGE range;
};

typedef struct DevParamDataBool
{
  bool value;
};

typedef union DevParamData
{
  DevParamDataBool    b_param;
  DevParamDataInt     m_param;
  DevParamDataEnum    u32_param;
  DevParamDataFloat   f_param;

  DevParamByteArray   byteArray;
  DevParamStruct      st_param;
};

#define DevParamTypeErrReminding( type )  \
    do {switch(type) { \
        case TY_FEATURE_BOOL:\
            LOGE("\t try use toBool()");\
            break;\
        case TY_FEATURE_INT:\
            LOGE("\t try use toInt()");\
            break;\
        case TY_FEATURE_ENUM:\
            LOGE("\t try use toEnum()");\
            break;\
        case TY_FEATURE_FLOAT:\
            LOGE("\t try use toFloat()");\
            break;\
        case TY_FEATURE_BYTEARRAY:\
            LOGE("\t try use toByteArray()");\
            break;\
        case TY_FEATURE_STRUCT:\
            LOGE("\t try use toArray()");\
            break;\
        default:\
            break;\
    }} while(0);

typedef struct DevParam
{
  TY_FEATURE_TYPE type;
  DevParamData data;

  bool isEmpty() {
    if(!type) return true;
    return false;
  }

  bool  toBool()  {
    if(type != TY_FEATURE_BOOL) {
      LOGE("Invalid device param data type.");
      DevParamTypeErrReminding(type);
      return false;
    }
    return data.b_param.value;
  }

  int   toInt()   {
    if(type != TY_FEATURE_INT) {
      LOGE("Invalid device param data type.");
      DevParamTypeErrReminding(type);
      return 0;
    }
    return data.m_param.value;
  }

  uint32_t toEnum() {
    if(type != TY_FEATURE_ENUM) {
      LOGE("Invalid device param data type.");
      DevParamTypeErrReminding(type);
      return 0;
    }
    return data.u32_param.value;
  }

  float toFloat() {
    if(type != TY_FEATURE_FLOAT) {
      LOGE("Invalid device param data type.");
      DevParamTypeErrReminding(type);
      return 0;
    }
    return data.f_param.value;
  }

  int mMin() {
    if(type != TY_FEATURE_INT) {
      LOGE("Invalid device param data type.");
      return 0;
    }
    return data.m_param.range.min;
  }
  int mMax() {
    if(type != TY_FEATURE_INT) {
      LOGE("Invalid device param data type.");
      return 0;
    }
    return data.m_param.range.max;
  }
  int mInc() {
    if(type != TY_FEATURE_INT) {
      LOGE("Invalid device param data type.");
      return 0;
    }
    return data.m_param.range.inc;
  }

  int fMin() {
    if(type != TY_FEATURE_FLOAT) {
      LOGE("Invalid device param data type.");
      return 0;
    }
    return data.f_param.range.min;
  }
  int fMax() {
    if(type != TY_FEATURE_FLOAT) {
      LOGE("Invalid device param data type.");
      return 0;
    }
    return data.f_param.range.max;
  }
  int fInc() {
    if(type != TY_FEATURE_FLOAT) {
      LOGE("Invalid device param data type.");
      return 0;
    }
    return data.f_param.range.inc;
  }

  std::vector<TY_ENUM_ENTRY> eList() {
    if(type != TY_FEATURE_ENUM) {
      LOGE("Invalid device param data type.");
      DevParamTypeErrReminding(type);
      return std::vector<TY_ENUM_ENTRY>();
    }
    
    if(0 == data.u32_param.entryCount) {
      return std::vector<TY_ENUM_ENTRY>();
    }

    return std::vector<TY_ENUM_ENTRY>(data.u32_param.list,  data.u32_param.list + data.u32_param.entryCount);
  }

  std::vector<unsigned char> toByteArray() {
    if(type != TY_FEATURE_BYTEARRAY) {
      LOGE("Invalid device param data type.");
      DevParamTypeErrReminding(type);
      return std::vector<unsigned char>();
    }
    return std::vector<unsigned char>(data.byteArray.m_data, data.byteArray.m_data + data.byteArray.real_size);
  }

  std::vector<int> toArray() {
    if(type != TY_FEATURE_STRUCT) {
      LOGE("Invalid device param data type.");
      DevParamTypeErrReminding(type);
      return std::vector<int>();
    }
    return std::vector<int>(data.st_param.m_data, data.st_param.m_data + data.st_param.real_size);
  }
};

class PercipioSDK
{
  public:
    PercipioSDK();
    ~PercipioSDK();

    std::vector<TY_DEVICE_BASE_INFO>& ListDevice();

    int IPv4StringToInt(char* ip);

    DevParam DevParamFromInt(int val) {
      DevParam param;
      param.type = TY_FEATURE_INT;
      param.data.m_param.value = val;
      return param;
    }

    DevParam DevParamFromEnum(uint32_t val) {
      DevParam param;
      param.type = TY_FEATURE_ENUM;
      param.data.u32_param.value = val;
      return param;
    }

    DevParam DevParamFromFloat(float val)
    {
      DevParam param;
      param.type = TY_FEATURE_FLOAT;
      param.data.f_param.value = val;
      return param;
    }

    DevParam DevParamFromBool(bool val)
    {
      DevParam param;
      param.type = TY_FEATURE_BOOL;
      param.data.b_param.value = val;
      return param;
    }

    DevParam DevParamFromByteArray(std::vector<unsigned char> val)
    {
      DevParam param;
      int max_size = sizeof(param.data.byteArray.m_data);
      if(val.size() > max_size) {
        LOGE("%s:%d Invalid ByteArray size.",__FILENAME__, __LINE__);
      } else {
        max_size = val.size();
      }
      param.type = TY_FEATURE_BYTEARRAY;
      param.data.byteArray.real_size = max_size;
      memcpy(param.data.byteArray.m_data, &val[0], max_size);
      return param;
    }

    DevParam DevParamFromPercipioAecROI(PercipioAecROI roi)
    {
      DevParam param;
      param.type = TY_FEATURE_STRUCT;
      param.data.st_param.real_size = 4;
      param.data.st_param.m_data[0] = roi._x;
      param.data.st_param.m_data[1] = roi._y;
      param.data.st_param.m_data[2] = roi._w;
      param.data.st_param.m_data[3] = roi._h;
      return param;
    }
    //st_param

    int TYGetLastErrorCode() { return m_last_error; };
    const char* TYGetLastErrorCodedescription() { return TYErrorString(m_last_error); };

    TY_DEV_HANDLE Open();
    TY_DEV_HANDLE Open(const char* sn);
    TY_DEV_HANDLE OpenDeviceByIP(const char* ip);
        bool isValidHandle(const TY_DEV_HANDLE handle);
    void Close(const TY_DEV_HANDLE handle);

    int DeviceWriteDefaultParametersFromJSFile(const TY_DEV_HANDLE handle, const char* file);
    int DeviceLoadDefaultParameters(const TY_DEV_HANDLE handle);
    int DeviceClearDefaultParameters(const TY_DEV_HANDLE handle);

    bool DeviceRegiststerCallBackEvent(DeviceEventHandle handler);

    int DeviceSetParameter(const TY_DEV_HANDLE handle, const int32_t comp, const TY_FEATURE_ID feat, DevParam param);
    DevParam DeviceGetParameter(const TY_DEV_HANDLE handle, const int32_t comp, const TY_FEATURE_ID feat);

    int DeviceSetParameter(const TY_DEV_HANDLE handle, const uint32_t comp, const TY_FEATURE_ID feat, DevParam param);
    DevParam DeviceGetParameter(const TY_DEV_HANDLE handle, const uint32_t comp, const TY_FEATURE_ID feat);

    int                           DeviceColorStreamIspEnable(const TY_DEV_HANDLE handle, bool enable);

    /*stream control*/
    bool                          DeviceHasStream(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    int                           DeviceStreamEnable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    int                           DeviceStreamDisable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    const std::vector<TY_ENUM_ENTRY>   DeviceStreamFormatDump(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    
    int                           DeviceStreamFormatConfig(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream, const TY_ENUM_ENTRY fmt);
        uint32_t Value(const TY_ENUM_ENTRY& fmt);
        int  Width(const TY_ENUM_ENTRY& fmt);
        int  Height(const TY_ENUM_ENTRY& fmt);
        const char* Description(const TY_ENUM_ENTRY& fmt);

    int  DeviceStreamOn(const TY_DEV_HANDLE handle);
    const std::vector<image_data> DeviceStreamRead(const TY_DEV_HANDLE handle, int timeout);
    int DeviceStreamOff(const TY_DEV_HANDLE handle);

    int                   DeviceReadCurrentEnumData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream, TY_ENUM_ENTRY& enum_desc);

    /*read calib data*/
    PercipioCalibData          DeviceReadCalibData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    PercipioRectifyIntrData    DeviceReadRectifiedIntrData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);
    PercipioRectifyRotaData    DeviceReadRectifiedRotationData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream);

    const float                 DeviceReadCalibDepthScaleUnit(const TY_DEV_HANDLE handle);


    /*device control*/
    float                 DeviceControlReadTemperature(const TY_DEV_HANDLE handle, TY_TEMPERATURE_ID ID);
    
    int                   DeviceControlTriggerModeEnable(const TY_DEV_HANDLE handle, const int enable);
    int                   DeviceControlTriggerModeSendTriggerSignal(const TY_DEV_HANDLE handle);
    int                   DeviceControlLaserPowerAutoControlEnable(const TY_DEV_HANDLE handle, bool enable);
    int                   DeviceControlLaserPowerConfig(const TY_DEV_HANDLE handle, int laser);

    /** stream control*/
    int DeviceStreamDepthRender(const image_data& src, image_data& dst);
    int DeviceStreamIRRender(const image_data& src, image_data& dst);
    int DeviceStreamImageDecode(const image_data& src, image_data& dst);
    int DeviceStreamMapDepthImageToPoint3D(const image_data& depth, const PercipioCalibData& calib_data, float scale, pointcloud_data_list& p3d);
    int DeviceStreamDoUndistortion(const PercipioCalibData& calib_data, const image_data& src, image_data& dst);
    int DeviceStreamMapDepthImageToColorCoordinate(const PercipioCalibData& depth_calib, const image_data& srcDepth, const float scale, 
                                                    const PercipioCalibData& color_calib, const int targetW, const int targetH, 
                                                    image_data& dstDepth);
    int DeviceStreamMapRGBImageToDepthCoordinate(const PercipioCalibData& depth_calib, const image_data& srcDepth, const float scale, 
                                                    const PercipioCalibData& color_calib, const image_data& srcColor, 
                                                    image_data& dstColor);

    //
    int DeviceStreamDepthSpeckleFilter(int max_spc_size,  int max_spc_diff, image_data& image);

  private:
    std::mutex _mutex;

    int m_last_error = TY_STATUS_OK;

    bool TyBayerColorConvert(const TY_DEV_HANDLE handle, const TY_IMAGE_DATA& src, image_data& dst);

    typedef enum STREAM_FMT_LIST_IDX {
        STREMA_FMT_IDX_COLOR      = 0,
        STREMA_FMT_IDX_DEPTH      = 1,
        STREMA_FMT_IDX_IR_LEFT    = 2,
        STREMA_FMT_IDX_IR_RIGHT   = 3,
        STREMA_FMT_IDX_MAX        = 4,
      }STREAM_FMT_LIST_IDX;

    typedef struct device_info {
      TY_DEV_HANDLE       handle;
      TY_ISP_HANDLE       isp;
      std::string         devID;
      PERCIPIO_STREAM_ID  stream;
      TY_COMPONENT_ID     Comps;
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
        isp    = NULL;
        devID = std::string(id);
        image_list.clear();

        TYGetComponentIDs(handle, &Comps);
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

int PercipioSDK::IPv4StringToInt(char* ip) {
  int32_t ip_i[4];
  uint8_t ip_b[4];
  int32_t ip32;
  sscanf(ip, "%d.%d.%d.%d", &ip_i[0], &ip_i[1], &ip_i[2], &ip_i[3]);
  ip_b[0] = ip_i[0];ip_b[1] = ip_i[1];ip_b[2] = ip_i[2];ip_b[3] = ip_i[3];
  ip32 = TYIPv4ToInt(ip_b);
  return ip32;
}

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
  TY_VERSION_INFO ver;
  TYLIB_API_ASSERT_OK(TYLibVersion(&ver));
  LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);
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
          return (int)i;
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

  m_last_error = TY_STATUS_OK;
  TY_INTERFACE_HANDLE hIface = NULL;
  TY_DEV_HANDLE hDevice = NULL;

  TY_DEVICE_BASE_INFO selectedDev;
  int idx = isValidDevice(sn);
  if(idx < 0) {
    std::vector<TY_DEVICE_BASE_INFO> selected;
    m_last_error = selectDevice(TY_INTERFACE_ALL, SN, "", 10, selected);
    if(m_last_error != TY_STATUS_OK) {
      return 0;
    }

    if(!selected.size()) {
      m_last_error = TY_STATUS_ERROR;
      return 0;
    }

    selectedDev = selected[0];
  } else {
    selectedDev = iDevBaseInfoList[idx];
  }
  
  
  m_last_error = TYOpenInterface(selectedDev.iface.id, &hIface);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYOpenInterface failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return 0;
  }

  m_last_error = TYOpenDevice(hIface, selectedDev.id, &hDevice);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYOpenDevice failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
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
  m_last_error = TY_STATUS_OK;

  LOGD("Update interface list");
  TYLIB_API_ASSERT_OK( TYUpdateInterfaceList() );

  uint32_t n = 0;
  TYLIB_API_ASSERT_OK( TYGetInterfaceNumber(&n) );
  LOGD("Got %u interface list", n);
  if(n == 0){
    LOGE("interface number incorrect");
    m_last_error = TY_STATUS_ERROR;
    return NULL;
  }

  TY_INTERFACE_HANDLE hIface = NULL;
  TY_DEV_HANDLE hDevice = NULL;

  LOGD("Update interface list");
  TYLIB_API_ASSERT_OK( TYUpdateInterfaceList() );

  TYLIB_API_ASSERT_OK( TYGetInterfaceNumber(&n) );
  LOGD("Got %u interface list", n);
  if(n == 0){
    LOGE("interface number incorrect");
    m_last_error = TY_STATUS_ERROR;
    return 0;
  }

  std::vector<TY_INTERFACE_INFO> ifaces(n);
  TYLIB_API_ASSERT_OK( TYGetInterfaceList(&ifaces[0], n, &n) );

  for(size_t i = 0; i < ifaces.size(); i++) {
    if( (ifaces[i].type == TY_INTERFACE_ETHERNET) || 
        (ifaces[i].type == TY_INTERFACE_IEEE80211)) {
      TYLIB_API_ASSERT_OK( TYOpenInterface(ifaces[i].id, &hIface) );
      m_last_error =  TYOpenDeviceWithIP(hIface, ip, &hDevice);
      if(m_last_error != TY_STATUS_OK)
        TYCloseInterface(hIface);
      else
        break;
    }
  }

  if(hDevice) {
    TY_DEVICE_BASE_INFO device_base_info;
    TYGetDeviceInfo           (hDevice, &device_base_info);
    DevList.push_back(device_info(hDevice, device_base_info.id));
    LOGD("Device %s is on!", device_base_info.id);

    TYRegisterEventCallback(hDevice, percipio_device_callback, hDevice);

    ConfigDevice(hDevice);

    DumpDeviceInfo(hDevice);
  }

  return hDevice;
}

TY_DEV_HANDLE PercipioSDK::Open() {
  return Open(NULL);
}

int PercipioSDK::hasDevice(const TY_DEV_HANDLE handle) {
  for(size_t i = 0; i < DevList.size(); i++) {
    if(handle == DevList[i].handle) {
      return (int)i;
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
  TY_COMPONENT_ID m_Comps = 0;
  TYGetComponentIDs(handle, &m_Comps);

  //device init
  if(m_Comps & TY_COMPONENT_DEVICE) {
    bool hasTriggerParam = false;
    TYHasFeature(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTriggerParam);
    if(hasTriggerParam) {
      TY_TRIGGER_PARAM trigger;
      trigger.mode = TY_TRIGGER_MODE_OFF;
    
      m_last_error = TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger));
      if(m_last_error != TY_STATUS_OK) {
        LOGE("TYSetStruct failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
      }
    } else {
      LOGW("=== Not support feature TY_STRUCT_TRIGGER_PARAM");
    }

    bool hasResend = false;
    TYHasFeature(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, &hasResend);
    if (hasResend) {
        TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, false);
    } else {
      LOGW("=== Not support feature TY_BOOL_GVSP_RESEND");
    }
  }

  //laser init
  if(m_Comps & TY_COMPONENT_LASER) {
    bool hasLaserAutoCtrl = false;
    TYHasFeature(handle, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, &hasLaserAutoCtrl);
    if(hasLaserAutoCtrl)
      TYSetBool(handle, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, true);

    bool hasLasePower = false;
    TYHasFeature(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, &hasLasePower);
    if(hasLasePower)
      TYSetInt(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, 100);
  }
  return ;
}

void PercipioSDK::DumpDeviceInfo(const TY_DEV_HANDLE handle) {
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("DumpDeviceInfo failed: invalid handle %s:%d", __FILENAME__, __LINE__);
    return ;
  }

  //DUMP STREAM FMT LIST
  static TY_COMPONENT_ID compID[STREMA_FMT_IDX_MAX] = {
      TY_COMPONENT_RGB_CAM,
      TY_COMPONENT_DEPTH_CAM,
      TY_COMPONENT_IR_CAM_LEFT,
      TY_COMPONENT_IR_CAM_RIGHT
  };

  TY_COMPONENT_ID IDs = 0;
  TYGetComponentIDs(handle, &IDs);

  for(size_t cnt = 0; cnt < STREMA_FMT_IDX_MAX; cnt++) {
    if(!(IDs & compID[cnt])) continue;

    bool has_image_mode = false;
    TYHasFeature(handle, compID[cnt], TY_ENUM_IMAGE_MODE, &has_image_mode);
    if(!has_image_mode) continue;

    unsigned int n = 0;
    m_last_error = TYGetEnumEntryCount(handle, compID[cnt], TY_ENUM_IMAGE_MODE, &n);
    if(m_last_error != TY_STATUS_OK) {
      DevList[idx].fmt_list[cnt].clear();
      continue;
    }

    if (n == 0){
      DevList[idx].fmt_list[cnt].clear();
      continue;
    }

    std::vector<TY_ENUM_ENTRY> temp;
    temp.resize(n);
    TY_ENUM_ENTRY* pEntry = &temp[0];
    m_last_error = TYGetEnumEntryInfo(handle, compID[cnt], TY_ENUM_IMAGE_MODE, pEntry, n, &n);
    if(m_last_error != TY_STATUS_OK) {
      DevList[idx].fmt_list[cnt].clear();
      continue;
    }

    for(size_t i = 0; i < n ; i++) {
        DevList[idx].fmt_list[cnt].push_back(temp[i]);
    }
  }

  //DUMP STREAM CALIB_DATA LIST
  for(size_t cnt = 0; cnt < STREMA_FMT_IDX_MAX; cnt++) {
    if(!(IDs & compID[cnt])) continue;

    TY_CAMERA_CALIB_INFO calib_data;
    m_last_error = TYGetStruct(handle, compID[cnt], TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(TY_CAMERA_CALIB_INFO));
    if(m_last_error != TY_STATUS_OK) {
      //LOGE("TYGetStruct failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
      memset(&DevList[idx].calib_data_list[cnt], 0, sizeof(TY_CAMERA_CALIB_INFO));
    }

    DevList[idx].calib_data_list[cnt] = PercipioCalibData(calib_data);
  }

  //DUMP DEPTH SCAL UNIT
  float scale_unit = 1.f;
  bool has_scale_unit = false;

  if(IDs & TY_COMPONENT_DEPTH_CAM) {
    TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &has_scale_unit);
    if(has_scale_unit) {
      m_last_error = TYGetFloat(handle, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
      if(m_last_error != TY_STATUS_OK) {
        LOGE("TYGetFloat failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
      }
    }
}
  DevList[idx].depth_scale_unit = scale_unit;

  //
  return;
}

bool PercipioSDK::DeviceRegiststerCallBackEvent(DeviceEventHandle handler) {
  if(handler) {
    handler_ptr = handler;
    handler = NULL;
    return true;
  } else {
    return false;
  }
}


enum EncodingType : uint32_t  
{
    HUFFMAN = 0,
};

#define MAX_STORAGE_SIZE    (10*1024*1024)
int PercipioSDK::DeviceWriteDefaultParametersFromJSFile(const TY_DEV_HANDLE handle, const char* file) {
    m_last_error = TY_STATUS_OK;

    int idx = hasDevice(handle);
    if(idx < 0) {
        LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_HANDLE;
        return TY_STATUS_INVALID_HANDLE;
    }

    TY_COMPONENT_ID IDs = 0;
    TYGetComponentIDs(handle, &IDs);
    if(! (IDs & TY_COMPONENT_STORAGE)) {
      m_last_error = TY_STATUS_INVALID_COMPONENT;
      LOGE("The device does not support JSON parameter Settings %s:%d", __FILENAME__, __LINE__);
      return TY_STATUS_INVALID_COMPONENT;
    }

    std::ifstream ifs(file);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    ifs.close();

    std::string err;
    std::string text(buffer.str());
    const auto json = Json::parse(text, err);
    if(json.is_null()) {
        m_last_error = TY_STATUS_NOT_IMPLEMENTED;
        return TY_STATUS_NOT_IMPLEMENTED;
    }

    std::string huffman_string;
    if(!TextHuffmanCompression(text, huffman_string)) {
        LOGE("Huffman compression error");
        m_last_error = TY_STATUS_ERROR;
        return TY_STATUS_ERROR;
    }

    const char* str = huffman_string.data();
    uint32_t crc = crc32_bitwise(str, huffman_string.length());

    uint32_t block_size;
    m_last_error = TYGetByteArraySize(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, &block_size);
    if(m_last_error != TY_STATUS_OK) {
        return m_last_error;
    }
    if(block_size < huffman_string.length() + 12) {
        LOGE("The configuration file is too large, the maximum size should not exceed 4000 bytes");
        m_last_error = TY_STATUS_ERROR;
        return TY_STATUS_ERROR;
    }
    
    uint8_t* blocks = new uint8_t[block_size] ();
    *(uint32_t*)blocks = crc;
    *(uint32_t*)(blocks + 4) = HUFFMAN;
    *(uint32_t*)(blocks + 8) = huffman_string.length();
    memcpy((char*)blocks + 12,  str, huffman_string.length());
    m_last_error = TYSetByteArray(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, blocks, block_size);
    delete []blocks;
    return m_last_error;
}

int PercipioSDK::DeviceLoadDefaultParameters(const TY_DEV_HANDLE handle) {
    m_last_error = TY_STATUS_OK;

    int idx = hasDevice(handle);
    if(idx < 0) {
        LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_HANDLE;
        return TY_STATUS_INVALID_HANDLE;
    }

    TY_COMPONENT_ID IDs = 0;
    TYGetComponentIDs(handle, &IDs);
    if(! (IDs & TY_COMPONENT_STORAGE)) {
      m_last_error = TY_STATUS_NO_DATA;
      LOGE("The device has no JSON parameter Settings %s:%d", __FILENAME__, __LINE__);
      return TY_STATUS_NO_DATA;
    }

    uint32_t block_size;
    uint8_t* blocks = new uint8_t[MAX_STORAGE_SIZE] ();
    m_last_error = TYGetByteArraySize(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, &block_size);
    if(m_last_error != TY_STATUS_OK) {
        delete []blocks;
        return m_last_error;
    }

    m_last_error = TYGetByteArray(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, blocks,  block_size);
    if(m_last_error != TY_STATUS_OK) {
        delete []blocks;
        return m_last_error;
    }
    
    uint32_t crc_data = *(uint32_t*)blocks;
    if(0 == crc_data || 0xffffffff == crc_data) {
        LOGW("The CRC check code is empty.");
        delete []blocks;
        m_last_error = TY_STATUS_NO_DATA;
        return TY_STATUS_NO_DATA;
    }

    uint32_t crc;
    uint8_t* js_code = blocks + 4;
    std::string js_string;
    crc = crc32_bitwise(js_code, strlen((const char*)js_code));
    if((crc != crc_data) || !isValidJsonString((const char*)js_code)) {
        EncodingType type     = *(EncodingType*)(blocks + 4);
        ASSERT(type == HUFFMAN);
        uint32_t huffman_size = *(uint32_t*)(blocks + 8);
        uint8_t* huffman_ptr  = (uint8_t*)(blocks + 12);
        if(huffman_size > (MAX_STORAGE_SIZE - 12)) {
            LOGE("Data length error.");
            delete []blocks;
            m_last_error = TY_STATUS_ERROR;
            return m_last_error;
        }
        
        crc = crc32_bitwise(huffman_ptr, huffman_size);
        if(crc_data != crc) {
            LOGE("The data in the storage area has a CRC check error.");
            delete []blocks;
            m_last_error = TY_STATUS_ERROR;
            return m_last_error;
        }

        std::string huffman_string(huffman_ptr, huffman_ptr + huffman_size);
        if(!TextHuffmanDecompression(huffman_string, js_string)) {
            LOGE("Huffman decoding error");
            delete []blocks;
            m_last_error = TY_STATUS_ERROR;
            return m_last_error;
        }
    } else {
        js_string = std::string((const char*)js_code);
    }

    if(!json_parse(handle, js_string.c_str())) {
      LOGE("Storage parameters load error.");
      delete []blocks;
      m_last_error = TY_STATUS_ERROR;
      return TY_STATUS_ERROR;
    }

    delete []blocks;
    return m_last_error;
}

int PercipioSDK::DeviceClearDefaultParameters(const TY_DEV_HANDLE handle)
{
    m_last_error = TY_STATUS_OK;
    int idx = hasDevice(handle);
    if(idx < 0) {
        LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_HANDLE;
        return TY_STATUS_INVALID_HANDLE;
    }

    TY_COMPONENT_ID IDs = 0;
    TYGetComponentIDs(handle, &IDs);
    if(! (IDs & TY_COMPONENT_STORAGE)) {
      LOGE("The device does not support JSON parameter Settings %s:%d", __FILENAME__, __LINE__);
      m_last_error = TY_STATUS_INVALID_COMPONENT;
      return TY_STATUS_INVALID_COMPONENT;
    }

    uint32_t block_size;
    m_last_error = TYGetByteArraySize(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, &block_size);
    if(m_last_error) {
        LOGE("Read storage size err %s:%d", __FILENAME__, __LINE__);
        return m_last_error;
    }

    uint8_t* blocks = new uint8_t[MAX_STORAGE_SIZE]();
    m_last_error = TYSetByteArray(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, blocks, block_size);
    if(m_last_error) {
        LOGE("Erase storage err %s:%d", __FILENAME__, __LINE__);
    }

    delete []blocks;
    return m_last_error;
}

int PercipioSDK::DeviceSetParameter(const TY_DEV_HANDLE handle, const int32_t comp, const TY_FEATURE_ID feat, DevParam value) {
    m_last_error = TY_STATUS_OK;
    int idx = hasDevice(handle);
    if(idx < 0) {
        LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_HANDLE;
        return TY_STATUS_INVALID_HANDLE;
    }

    bool has = false;
    TY_COMPONENT_ID  id = static_cast<TY_COMPONENT_ID>(comp);
    TY_COMPONENT_ID IDs = 0;
    TYGetComponentIDs(handle, &IDs);
    if(! (IDs & id)) {
      LOGE("Invalid component id %s:%d", __FILENAME__, __LINE__);
      m_last_error = TY_STATUS_INVALID_COMPONENT;
      return m_last_error;
    }

    m_last_error = TYHasFeature(handle, id, feat, &has);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("TYHasFeature failed %s:%d", __FILENAME__, __LINE__);
      return m_last_error;
    }

    if(!has) {
        LOGE("Invalid feature %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_FEATURE;
        return TY_STATUS_INVALID_FEATURE;
    }

    TY_FEATURE_TYPE type = TYFeatureType(feat);;
    if(type != value.type) {
      LOGE("Invalid parameter type %s:%d", __FILENAME__, __LINE__);
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return TY_STATUS_INVALID_PARAMETER;
    }

    switch (type)
    {
    case TY_FEATURE_INT:
        m_last_error = TYSetInt(handle, id, feat, static_cast<int32_t>(value.data.m_param.value));
        break;
    case  TY_FEATURE_ENUM:
        m_last_error = TYSetEnum(handle, id, feat, static_cast<uint32_t>(value.data.u32_param.value));
        break;
    case  TY_FEATURE_BOOL:
        m_last_error = TYSetBool(handle, id, feat, static_cast<bool>(value.data.b_param.value));
        break;
    case TY_FEATURE_FLOAT:
        m_last_error = TYSetFloat(handle, id, feat, static_cast<float>(value.data.f_param.value));
        break;
    case TY_FEATURE_BYTEARRAY:
        m_last_error = TYSetByteArray(handle, id, feat, value.data.byteArray.m_data, value.data.byteArray.real_size);
        break;
    case TY_FEATURE_STRUCT:
        if(feat == TY_STRUCT_AEC_ROI) {
            if(value.data.st_param.real_size != 4) {
                LOGE("Invalid feature data %s:%d", __FILENAME__, __LINE__);
                m_last_error = TY_STATUS_INVALID_PARAMETER;
                return TY_STATUS_INVALID_PARAMETER;
            }

            TY_AEC_ROI_PARAM roi;
            roi.x = value.data.st_param.m_data[0];
            roi.y = value.data.st_param.m_data[1];
            roi.w = value.data.st_param.m_data[2];
            roi.h = value.data.st_param.m_data[3];
            m_last_error = TYSetStruct(handle, id, feat, &roi, sizeof(roi));
        } else 
            m_last_error = TY_STATUS_INVALID_FEATURE;
        break;
    default:
        LOGE("Invalid feature type %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_PARAMETER;
        return TY_STATUS_INVALID_PARAMETER;
    }

    if(m_last_error != TY_STATUS_OK) {
        LOGE("Device set parameters failed: %s: %d", TYErrorString(m_last_error), __LINE__);
    }

    return m_last_error;
}

DevParam PercipioSDK::DeviceGetParameter(const TY_DEV_HANDLE handle, const int32_t comp, const TY_FEATURE_ID feat)
{
    m_last_error = TY_STATUS_OK;
    DevParam para;
    memset(&para, 0, sizeof(para));
    int idx = hasDevice(handle);
    if(idx < 0) {
        LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_HANDLE;
        return para;
    }

    bool has = false;
    TY_COMPONENT_ID  id = static_cast<TY_COMPONENT_ID>(comp);
    TY_COMPONENT_ID IDs = 0;
    TYGetComponentIDs(handle, &IDs);
    if(! (IDs & id)) {
      LOGE("Invalid component id %s:%d", __FILENAME__, __LINE__);
      m_last_error = TY_STATUS_INVALID_COMPONENT;
      return para;
    }

    m_last_error = TYHasFeature(handle, id, feat, &has);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("TYHasFeature failed %s:%d", __FILENAME__, __LINE__);
      return para;
    }

    if(!has) {
        LOGE("Invalid feature %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_FEATURE;
        return para;
    }

    uint32_t count = 0;
    TY_FEATURE_TYPE type = TYFeatureType(feat);
    switch (type)
    {
    case TY_FEATURE_INT:
        para.type = TY_FEATURE_INT;
        TYGetIntRange(handle, id, feat, &para.data.m_param.range);
        m_last_error = TYGetInt(handle, id, feat, &para.data.m_param.value);
        break;
    case  TY_FEATURE_ENUM:
        para.type = TY_FEATURE_ENUM;
        TYGetEnumEntryCount(handle, id, feat, &count);
        if(count > sizeof(para.data.u32_param.list) / sizeof(TY_ENUM_ENTRY))
          count = sizeof(para.data.u32_param.list);
        TYGetEnumEntryInfo(handle, id, feat, para.data.u32_param.list, count, (uint32_t*)(&para.data.u32_param.entryCount));
        m_last_error = TYGetEnum(handle, id, feat, (uint32_t*)&para.data.u32_param.value);
        break;
    case  TY_FEATURE_BOOL:
        para.type = TY_FEATURE_BOOL;
        m_last_error = TYGetBool(handle, id, feat, &para.data.b_param.value);
        break;
    case TY_FEATURE_FLOAT:
        para.type = TY_FEATURE_FLOAT;
        TYGetFloatRange(handle, id, feat, &para.data.f_param.range);
        m_last_error = TYGetFloat(handle, id, feat, &para.data.f_param.value);
        break;
    case TY_FEATURE_BYTEARRAY:
        para.type = TY_FEATURE_BYTEARRAY;
        TYGetByteArraySize(handle, id, feat, &count);
        if(count > sizeof(para.data.byteArray.m_data)) {
          LOGE("Dev byte array parameters legth exceeds the limit.");
          m_last_error = TY_STATUS_INVALID_FEATURE;
          return para;
        }
        para.data.byteArray.real_size = count;
        m_last_error = TYGetByteArray(handle, id, feat, para.data.byteArray.m_data, count);
        break;
    case TY_FEATURE_STRUCT:
        para.type = TY_FEATURE_STRUCT;
        if(feat == TY_STRUCT_AEC_ROI) {
            TY_AEC_ROI_PARAM roi;
            m_last_error = TYGetStruct(handle, id, feat, &roi, sizeof(roi));
            if(m_last_error == TY_STATUS_OK) {
              para.data.st_param.m_data[0] = roi.x;
              para.data.st_param.m_data[1] = roi.y;
              para.data.st_param.m_data[2] = roi.w;
              para.data.st_param.m_data[3] = roi.h;
              para.data.st_param.real_size = 4;
            }
        } else 
            m_last_error = TY_STATUS_INVALID_FEATURE;
        break;
    default:
        LOGE("Invalid feature type %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_FEATURE;
        return para;
    }

    if(m_last_error != TY_STATUS_OK) {
        LOGE("Device get parameter failed: %s: %d", TYErrorString(m_last_error), __LINE__);
    }

    return para;
}

int PercipioSDK::DeviceSetParameter(const TY_DEV_HANDLE handle, const uint32_t comp, const TY_FEATURE_ID feat, DevParam value) {
    m_last_error = TY_STATUS_OK;
    int idx = hasDevice(handle);
    if (idx < 0) {
        LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_HANDLE;
        return TY_STATUS_INVALID_HANDLE;
    }

    bool has = false;
    TY_COMPONENT_ID  id = static_cast<TY_COMPONENT_ID>(comp);
    TY_COMPONENT_ID IDs = 0;
    TYGetComponentIDs(handle, &IDs);
    if(! (IDs & id)) {
      LOGE("Invalid component id %s:%d", __FILENAME__, __LINE__);
      m_last_error = TY_STATUS_INVALID_COMPONENT;
      return TY_STATUS_INVALID_COMPONENT;
    }

    m_last_error = TYHasFeature(handle, id, feat, &has);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("TYHasFeature failed %s:%d", __FILENAME__, __LINE__);
      return m_last_error;
    }

    if (!has) {
        LOGE("Invalid feature %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_FEATURE;
        return TY_STATUS_INVALID_FEATURE;
    }

    TY_FEATURE_TYPE type = TYFeatureType(feat);
    if(type != value.type) {
      LOGE("Invalid parameter type %s:%d", __FILENAME__, __LINE__);
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return TY_STATUS_INVALID_PARAMETER;
    }

    switch (type)
    {
    case TY_FEATURE_INT:
        m_last_error = TYSetInt(handle, id, feat, static_cast<int32_t>(value.data.m_param.value));
        break;
    case  TY_FEATURE_ENUM:
        m_last_error = TYSetEnum(handle, id, feat, static_cast<uint32_t>(value.data.u32_param.value));
        break;
    case  TY_FEATURE_BOOL:
        m_last_error = TYSetBool(handle, id, feat, static_cast<bool>(value.data.b_param.value));
        break;
    case TY_FEATURE_FLOAT:
        m_last_error = TYSetFloat(handle, id, feat, static_cast<float>(value.data.f_param.value));
        break;
    case TY_FEATURE_BYTEARRAY:
        m_last_error = TYSetByteArray(handle, id, feat, value.data.byteArray.m_data, value.data.byteArray.real_size);
        break;
    case TY_FEATURE_STRUCT:
        if(feat == TY_STRUCT_AEC_ROI) {
            if(value.data.st_param.real_size != 4) {
                LOGE("Invalid feature data %s:%d", __FILENAME__, __LINE__);
                m_last_error = TY_STATUS_INVALID_PARAMETER;
                return TY_STATUS_INVALID_PARAMETER;
            }

            TY_AEC_ROI_PARAM roi;
            roi.x = value.data.st_param.m_data[0];
            roi.y = value.data.st_param.m_data[1];
            roi.w = value.data.st_param.m_data[2];
            roi.h = value.data.st_param.m_data[3];
            m_last_error = TYSetStruct(handle, id, feat, &roi, sizeof(roi));
        } else 
            m_last_error = TY_STATUS_INVALID_FEATURE;
        break;
    default:
        LOGE("Invalid feature type %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_PARAMETER;
        return TY_STATUS_INVALID_PARAMETER;
    }

    if (m_last_error != TY_STATUS_OK) {
        LOGE("Device set parameter failed: %s: %d", TYErrorString(m_last_error), __LINE__);
    }

    return m_last_error;
}

DevParam PercipioSDK::DeviceGetParameter(const TY_DEV_HANDLE handle, const uint32_t comp, const TY_FEATURE_ID feat)
{
    m_last_error = TY_STATUS_OK;
    DevParam para;
    memset(&para, 0, sizeof(para));
    int idx = hasDevice(handle);
    if (idx < 0) {
        LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_HANDLE;
        return para;
    }

    bool has = false;
    TY_COMPONENT_ID  id = static_cast<TY_COMPONENT_ID>(comp);
    TY_COMPONENT_ID IDs = 0;
    TYGetComponentIDs(handle, &IDs);
    if(! (IDs & id)) {
      LOGE("Invalid component id %s:%d", __FILENAME__, __LINE__);
      m_last_error = TY_STATUS_INVALID_COMPONENT;
      return para;
    }

    m_last_error = TYHasFeature(handle, id, feat, &has);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("TYHasFeature failed %s:%d", __FILENAME__, __LINE__);
      return para;
    }

    if (!has) {
        LOGE("Invalid feature %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_FEATURE;
        return para;
    }

    uint32_t count = 0;
    TY_FEATURE_TYPE type = TYFeatureType(feat);
    switch (type)
    {
    case TY_FEATURE_INT:
        para.type = TY_FEATURE_INT;
        TYGetIntRange(handle, id, feat, &para.data.m_param.range);
        m_last_error = TYGetInt(handle, id, feat, &para.data.m_param.value);
        break;
    case  TY_FEATURE_ENUM:
        para.type = TY_FEATURE_ENUM;
        TYGetEnumEntryCount(handle, id, feat, &count);
        if(count > sizeof(para.data.u32_param.list) / sizeof(TY_ENUM_ENTRY))
          count = sizeof(para.data.u32_param.list);
        TYGetEnumEntryInfo(handle, id, feat, para.data.u32_param.list, count, (uint32_t*)(&para.data.u32_param.entryCount));
        m_last_error = TYGetEnum(handle, id, feat, (uint32_t*)&para.data.u32_param.value);
        break;
    case  TY_FEATURE_BOOL:
        para.type = TY_FEATURE_BOOL;
        m_last_error = TYGetBool(handle, id, feat, &para.data.b_param.value);
        break;
    case TY_FEATURE_FLOAT:
        para.type = TY_FEATURE_FLOAT;
        TYGetFloatRange(handle, id, feat, &para.data.f_param.range);
        m_last_error = TYGetFloat(handle, id, feat, &para.data.f_param.value);
        break;
    case TY_FEATURE_BYTEARRAY:
        para.type = TY_FEATURE_BYTEARRAY;
        TYGetByteArraySize(handle, id, feat, &count);
        if(count > sizeof(para.data.byteArray.m_data)) {
          LOGE("Dev byte array parameters legth exceeds the limit.");
          m_last_error = TY_STATUS_INVALID_FEATURE;
          return para;
        }
        para.data.byteArray.real_size = count;
        m_last_error = TYGetByteArray(handle, id, feat, para.data.byteArray.m_data, count);
        break;
    case TY_FEATURE_STRUCT:
        para.type = TY_FEATURE_STRUCT;
        if(feat == TY_STRUCT_AEC_ROI) {
            TY_AEC_ROI_PARAM roi;
            m_last_error = TYGetStruct(handle, id, feat, &roi, sizeof(roi));
            if(m_last_error == TY_STATUS_OK) {
              para.data.st_param.m_data[0] = roi.x;
              para.data.st_param.m_data[1] = roi.y;
              para.data.st_param.m_data[2] = roi.w;
              para.data.st_param.m_data[3] = roi.h;
              para.data.st_param.real_size = 4;
            }
        } else 
            m_last_error = TY_STATUS_INVALID_FEATURE;
        break;
    default:
        LOGE("Invalid feature type %s:%d", __FILENAME__, __LINE__);
        m_last_error = TY_STATUS_INVALID_FEATURE;
        return para;
    }

    if(m_last_error != TY_STATUS_OK) {
        LOGE("Device get parameter failed: %s: %d", TYErrorString(m_last_error), __LINE__);
    }

    return para;
}

int PercipioSDK::DeviceColorStreamIspEnable(const TY_DEV_HANDLE handle, bool enable) {
  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Device handle check failed: invalid handle %s:%d", __FILENAME__, __LINE__);
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return m_last_error;
  }

  if(enable && DevList[idx].isp == NULL) {
    m_last_error = TYISPCreate(&DevList[idx].isp);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("TYISPCreate failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    }

    m_last_error = ColorIspInitSetting(DevList[idx].isp, handle);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("ColorIspInitSetting failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    }
  }

  if(!enable && DevList[idx].isp != NULL) {
    m_last_error = TYISPRelease(&DevList[idx].isp);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("TYISPRelease failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    }
    DevList[idx].isp = NULL;
  }

  return m_last_error;
}

bool PercipioSDK::DeviceHasStream(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream)
{
  TY_COMPONENT_ID allComps;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return false;
  }

  uint32_t m_compID;
  switch(stream) {
    case PERCIPIO_STREAM_COLOR:
      m_compID = TY_COMPONENT_RGB_CAM;
      break;
    case PERCIPIO_STREAM_DEPTH:
      m_compID = TY_COMPONENT_DEPTH_CAM;
      break;
    case PERCIPIO_STREAM_IR_LEFT:
      m_compID = TY_COMPONENT_IR_CAM_LEFT;
      break;
    case PERCIPIO_STREAM_IR_RIGHT:
      m_compID = TY_COMPONENT_IR_CAM_RIGHT;
      break;
    default:
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return false;
  }

  if(allComps & m_compID) {
    return true;
  }
  
  return false;
}

int PercipioSDK::DeviceStreamEnable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {
  TY_COMPONENT_ID allComps;
  m_last_error = TY_STATUS_OK;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(stream & PERCIPIO_STREAM_COLOR) {
    if(allComps & TY_COMPONENT_RGB_CAM) {
      m_last_error = TYEnableComponents(handle, TY_COMPONENT_RGB_CAM);
      if(m_last_error != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
        return m_last_error;
      }
    } else {
      LOGE("The device does not support color stream.\n");
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return m_last_error;
    }
  } else {
    if(allComps & TY_COMPONENT_RGB_CAM) 
      TYDisableComponents(handle, TY_COMPONENT_RGB_CAM);
  }

  if(stream & PERCIPIO_STREAM_DEPTH) {
    if(allComps & TY_COMPONENT_DEPTH_CAM) {
      m_last_error = TYEnableComponents(handle, TY_COMPONENT_DEPTH_CAM);
      if(m_last_error != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
        return m_last_error;
      }
    } else {
      LOGE("The device does not support depth stream.\n");
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return m_last_error;
    }
  } else {
    if(allComps & TY_COMPONENT_DEPTH_CAM)
      TYDisableComponents(handle, TY_COMPONENT_DEPTH_CAM);
  }

  if(stream & PERCIPIO_STREAM_IR_LEFT) {
    if(allComps & TY_COMPONENT_IR_CAM_LEFT) {
      m_last_error = TYEnableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
      if(m_last_error != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
        return m_last_error;
      }
    } else {
      LOGE("The device does not support left ir stream.\n");
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return m_last_error;
    }
  } else {
    if(allComps & TY_COMPONENT_IR_CAM_LEFT) 
      TYDisableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
  }

  if(stream & PERCIPIO_STREAM_IR_RIGHT) {
    if(allComps & TY_COMPONENT_IR_CAM_RIGHT) {
      m_last_error = TYEnableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
      if(m_last_error != TY_STATUS_OK) {
        LOGE("TYEnableComponents failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
        return m_last_error;
      }
    } else {
      LOGE("The device does not support right ir stream.\n");
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return m_last_error;
    }
  } else {
    if(allComps & TY_COMPONENT_IR_CAM_RIGHT) 
      TYDisableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
  }

  return m_last_error;
}

int PercipioSDK::DeviceStreamDisable(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {
  TY_COMPONENT_ID allComps;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(stream & PERCIPIO_STREAM_COLOR) {
    if(allComps & TY_COMPONENT_RGB_CAM) {
      m_last_error = TYDisableComponents(handle, TY_COMPONENT_RGB_CAM);
      if(m_last_error != TY_STATUS_OK) {
        return m_last_error;
      }
    } else {
      LOGE("The device does not support color stream.\n");
    }
  }

  if(stream & PERCIPIO_STREAM_DEPTH) {
    if(allComps & TY_COMPONENT_DEPTH_CAM) {
      m_last_error = TYDisableComponents(handle, TY_COMPONENT_DEPTH_CAM);
      if(m_last_error != TY_STATUS_OK) {
        return m_last_error;
      }
    } else {
      LOGE("The device does not support depth stream.\n");
    }
  }

  if(stream & PERCIPIO_STREAM_IR_LEFT) {
    if(allComps & PERCIPIO_STREAM_IR_LEFT) {
      m_last_error = TYDisableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
      if(m_last_error != TY_STATUS_OK) {
        return m_last_error;
      }
    } else {
      LOGE("The device does not support left ir stream.\n");
    }
  }

  if(stream & PERCIPIO_STREAM_IR_RIGHT) {
    if(allComps & TY_COMPONENT_IR_CAM_RIGHT) {
      m_last_error = TYDisableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
      if(m_last_error != TY_STATUS_OK) {
        return m_last_error;
      }
    } else {
      LOGE("The device does not support right ir stream.\n");
    }
  }
  return TY_STATUS_OK;
}

const std::vector<TY_ENUM_ENTRY> PercipioSDK::DeviceStreamFormatDump(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {
  m_last_error = TY_STATUS_OK;
  int compIDX = stream_idx(stream);
  if(compIDX == STREMA_FMT_IDX_MAX) {
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return std::vector<TY_ENUM_ENTRY>();
  }

  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return std::vector<TY_ENUM_ENTRY>();;
  }

  return DevList[idx].fmt_list[compIDX];
}

uint32_t PercipioSDK::Value(const TY_ENUM_ENTRY& fmt) {
  return fmt.value;
}

int  PercipioSDK::Width(const TY_ENUM_ENTRY& fmt) {
  return TYImageWidth(fmt.value);
}

int  PercipioSDK::Height(const TY_ENUM_ENTRY& fmt) {
  return TYImageHeight(fmt.value);
}

const char* PercipioSDK::Description(const TY_ENUM_ENTRY& fmt) 
{
  return fmt.description;
}

int PercipioSDK::DeviceStreamFormatConfig(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream, const TY_ENUM_ENTRY fmt) {

  TY_COMPONENT_ID compID;
  m_last_error = TY_STATUS_OK;
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
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return TY_STATUS_INVALID_PARAMETER;
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(!(allComps & compID)) {
    LOGE("stream mode not support : %d", stream);
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return m_last_error;
  }

  m_last_error = TYSetEnum(handle, compID, TY_ENUM_IMAGE_MODE, fmt.value);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("Stream fmt is not support!");
    return m_last_error;
  }

  return m_last_error;
}

int PercipioSDK::DeviceReadCurrentEnumData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream, TY_ENUM_ENTRY& enum_desc)
{
  m_last_error = TY_STATUS_OK;
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
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return TY_STATUS_INVALID_PARAMETER;
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(!(allComps & compID)) {
    LOGE("stream mode not support : %d", stream);
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return m_last_error;
  }

  bool has_image_mode = false;
  TYHasFeature(handle, compID, TY_ENUM_IMAGE_MODE, &has_image_mode);
  if(!has_image_mode) {
    m_last_error = TY_STATUS_INVALID_FEATURE;
    LOGE("Stream fmt is not support!");
    return m_last_error;
  }

  uint32_t value;
  m_last_error = TYGetEnum(handle, compID, TY_ENUM_IMAGE_MODE, &value);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("Stream fmt is not support!");
    return m_last_error;
  }

  std::vector<TY_ENUM_ENTRY> enum_data_list = DeviceStreamFormatDump(handle, stream);
  for(size_t i = 0; i < enum_data_list.size(); i++) {
    if(value == enum_data_list[i].value) {
      enum_desc = enum_data_list[i];
      return TY_STATUS_OK;
    }
  }

  m_last_error = TY_STATUS_INVALID_PARAMETER;
  return TY_STATUS_INVALID_PARAMETER;
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
  DevList[idx].frameBuffer[0] = (char*)malloc(frameSize);
  DevList[idx].frameBuffer[1] = (char*)malloc(frameSize);

  TYLIB_API_ASSERT_OK(TYEnqueueBuffer(handle, DevList[idx].frameBuffer[0], frameSize));
  TYLIB_API_ASSERT_OK(TYEnqueueBuffer(handle, DevList[idx].frameBuffer[1], frameSize));
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

PercipioCalibData PercipioSDK::DeviceReadCalibData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream) {
  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return PercipioCalibData();
  }

  int compIDX = stream_idx(stream);
  if(compIDX == STREMA_FMT_IDX_MAX) {
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return PercipioCalibData();
  }

  return DevList[idx].calib_data_list[compIDX];
}

PercipioRectifyIntrData PercipioSDK::DeviceReadRectifiedIntrData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream)
{
  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return PercipioRectifyIntrData();
  }

  int compIDX = stream_idx(stream);
  TY_COMPONENT_ID comp;
  switch(compIDX) {
    case STREMA_FMT_IDX_COLOR:
      comp = TY_COMPONENT_RGB_CAM;
      break;
    case STREMA_FMT_IDX_DEPTH:
    comp = TY_COMPONENT_DEPTH_CAM;
      break;
    case STREMA_FMT_IDX_IR_LEFT:
    comp = TY_COMPONENT_IR_CAM_LEFT;
      break;
    case STREMA_FMT_IDX_IR_RIGHT:
    comp = TY_COMPONENT_IR_CAM_RIGHT;
      break;
    case STREMA_FMT_IDX_MAX: {
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return PercipioRectifyIntrData();
    }
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return PercipioRectifyIntrData();
  }

  if(!(allComps & comp)) {
    LOGE("stream mode not support : %d", stream);
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return PercipioRectifyIntrData();;
  }

  bool hasRectifyIntr = false;
  TYHasFeature(handle, comp, TY_STRUCT_CAM_RECTIFIED_INTRI, &hasRectifyIntr);
  if(!hasRectifyIntr) {
    LOGE("Current stream mode not support rectified intri: %d", stream);
    return PercipioRectifyIntrData();
  }

  TY_CAMERA_INTRINSIC intr;
  m_last_error = TYGetStruct(handle, comp, TY_STRUCT_CAM_RECTIFIED_INTRI, &intr, sizeof(intr));
  if(m_last_error) {
    return PercipioRectifyIntrData();
  }

  return PercipioRectifyIntrData(intr);
}

PercipioRectifyRotaData PercipioSDK::DeviceReadRectifiedRotationData(const TY_DEV_HANDLE handle, const PERCIPIO_STREAM_ID stream)
{
  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return PercipioRectifyRotaData();
  }

  int compIDX = stream_idx(stream);
  TY_COMPONENT_ID comp;
  switch(compIDX) {
    case STREMA_FMT_IDX_COLOR:
      comp = TY_COMPONENT_RGB_CAM;
      break;
    case STREMA_FMT_IDX_DEPTH:
    comp = TY_COMPONENT_DEPTH_CAM;
      break;
    case STREMA_FMT_IDX_IR_LEFT:
    comp = TY_COMPONENT_IR_CAM_LEFT;
      break;
    case STREMA_FMT_IDX_IR_RIGHT:
    comp = TY_COMPONENT_IR_CAM_RIGHT;
      break;
    case STREMA_FMT_IDX_MAX: {
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return PercipioRectifyRotaData();
    }
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return PercipioRectifyRotaData();
  }

  if(!(allComps & comp)) {
    LOGE("stream mode not support : %d", stream);
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return PercipioRectifyRotaData();;
  }

  bool hasRectifyRotation = false;
  TYHasFeature(handle, comp, TY_STRUCT_CAM_RECTIFIED_ROTATION, &hasRectifyRotation);
  if(!hasRectifyRotation) {
    LOGE("Current stream mode not support rectified rotation: %d", stream);
    return PercipioRectifyRotaData();
  }

  TY_CAMERA_ROTATION rotation;
  m_last_error = TYGetStruct(handle, comp, TY_STRUCT_CAM_RECTIFIED_ROTATION, &rotation, sizeof(rotation));
  if(m_last_error) {
    return PercipioRectifyRotaData();
  }

  return PercipioRectifyRotaData(rotation);
}

const float PercipioSDK::DeviceReadCalibDepthScaleUnit(const TY_DEV_HANDLE handle) {

  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return NAN;
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return 1.f;
  }

  if(!(allComps & TY_COMPONENT_DEPTH_CAM)) {
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return 1.f;
  }

  float scale_unit = 1.f;
  bool has_scale_unit = false;
  TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &has_scale_unit);
  if(has_scale_unit) {
    m_last_error = TYGetFloat(handle, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
    if(m_last_error != TY_STATUS_OK) {
      LOGE("TYGetFloat failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    }
  }

  DevList[idx].depth_scale_unit = scale_unit;
  return scale_unit;
}

int PercipioSDK::DeviceStreamOn(const TY_DEV_HANDLE handle) {
  std::unique_lock<std::mutex> lock(_mutex);
  m_last_error = TY_STATUS_OK;
  if(hasDevice(handle) < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return TY_STATUS_INVALID_HANDLE;
  }

  unsigned int frameSize;
  m_last_error = TYGetFrameBufferSize(handle, &frameSize);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetFrameBufferSize failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(!FrameBufferAlloc(handle, frameSize)) {
    LOGE("====FrameBufferAlloc fail!\n");
    return m_last_error;
  }

  m_last_error = TYStartCapture(handle);
  if(m_last_error != TY_STATUS_OK) {
    return m_last_error;
  }

  LOGD("Stream ON!");
  return m_last_error;
}

//only support raw8 / raw10 / raw12
static bool IsBayerColor(int32_t         pixelFormat) {
  switch(pixelFormat) {
    case TY_PIXEL_FORMAT_BAYER8GRBG:
    case TY_PIXEL_FORMAT_BAYER8RGGB:
    case TY_PIXEL_FORMAT_BAYER8GBRG:
    case TY_PIXEL_FORMAT_BAYER8BGGR:
    case TY_PIXEL_FORMAT_CSI_BAYER10GRBG:
    case TY_PIXEL_FORMAT_CSI_BAYER10RGGB:
    case TY_PIXEL_FORMAT_CSI_BAYER10GBRG:
    case TY_PIXEL_FORMAT_CSI_BAYER10BGGR:
    case TY_PIXEL_FORMAT_CSI_BAYER12GRBG:
    case TY_PIXEL_FORMAT_CSI_BAYER12RGGB:
    case TY_PIXEL_FORMAT_CSI_BAYER12GBRG:
    case TY_PIXEL_FORMAT_CSI_BAYER12BGGR:
      return true;
    default:
      return false;
  }
  return false;
}

bool PercipioSDK::TyBayerColorConvert(const TY_DEV_HANDLE handle, const TY_IMAGE_DATA& src, image_data& dst)
{
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return false;
  }

  if(!DevList[idx].isp) {
    LOGE("Isp handle is empty!");
    return false;
  }

  if ((src.pixelFormat == TY_PIXEL_FORMAT_BAYER8GBRG) ||
      (src.pixelFormat == TY_PIXEL_FORMAT_BAYER8BGGR) ||
      (src.pixelFormat == TY_PIXEL_FORMAT_BAYER8GRBG) ||
      (src.pixelFormat == TY_PIXEL_FORMAT_BAYER8RGGB)) {
    int src_sz = src.width*src.height;
    int dst_sz = src.width*src.height * 3;
    dst.resize(dst_sz);
    TY_IMAGE_DATA out_buff = TYInitImageData(dst_sz, dst.buffer, src.width, src.height);
    out_buff.pixelFormat = TY_PIXEL_FORMAT_BGR;
    int res = TYISPProcessImage(DevList[idx].isp, &src, &out_buff);
    if (res != TY_STATUS_OK){
      LOGE("TYISPProcessImage failed: error %d(%s) at %s:%d", res, TYErrorString(res), __FILENAME__, __LINE__);
      return false;
    }
  }
  else if( (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10GBRG) ||
           (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10BGGR) ||
           (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10GRBG) ||
           (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10RGGB)) {
    std::vector<uint16_t> bayer10(src.width * src.height);
    std::vector<uint8_t> bayer8(src.width * src.height);
    ImgProc::parseCsiRaw10((uint8_t*)src.buffer, &bayer10[0], src.width, src.height);
    for(size_t idx = 0; idx < bayer10.size(); idx++)
      bayer8[idx] = bayer10[idx] >> 8;
    
    int src_sz = src.width*src.height;
    int dst_sz = src.width*src.height * 3;
    dst.resize(dst_sz);
    TY_IMAGE_DATA in_buff = TYInitImageData(src_sz, &bayer8[0], src.width, src.height);
    TY_IMAGE_DATA out_buff = TYInitImageData(dst_sz, dst.buffer, src.width, src.height);
    in_buff.pixelFormat = TY_PIXEL_8BIT  | ((0xf << 24) & src.pixelFormat);
    out_buff.pixelFormat = TY_PIXEL_FORMAT_BGR;
    int res = TYISPProcessImage(DevList[idx].isp, &in_buff, &out_buff);
    if (res != TY_STATUS_OK){
      LOGE("TYISPProcessImage failed: error %d(%s) at %s:%d", res, TYErrorString(res), __FILENAME__, __LINE__);
      return false;
    }
  } 
  else if( (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12GBRG) ||
           (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12BGGR) ||
           (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12GRBG) ||
           (src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12RGGB) ) {
    std::vector<uint16_t> bayer12(src.width * src.height);
    std::vector<uint8_t> bayer8(src.width * src.height);
    ImgProc::parseCsiRaw12((uint8_t*)src.buffer, &bayer12[0], src.width, src.height);
    for(size_t idx = 0; idx < bayer12.size(); idx++)
      bayer8[idx] = bayer12[idx] >> 8;
    
    int src_sz = src.width*src.height;
    int dst_sz = src.width*src.height * 3;
    dst.resize(dst_sz);
    TY_IMAGE_DATA in_buff = TYInitImageData(src_sz, &bayer8[0], src.width, src.height);
    TY_IMAGE_DATA out_buff = TYInitImageData(dst_sz, dst.buffer, src.width, src.height);
    in_buff.pixelFormat = TY_PIXEL_8BIT  | ((0xf << 24) & src.pixelFormat);
    out_buff.pixelFormat = TY_PIXEL_FORMAT_BGR;
    int res = TYISPProcessImage(DevList[idx].isp, &in_buff, &out_buff);
    if (res != TY_STATUS_OK){
      LOGE("TYISPProcessImage failed: error %d(%s) at %s:%d", res, TYErrorString(res), __FILENAME__, __LINE__);
      return false;
    }
  }
  else
  {
    return false;
  }

  dst.pixelFormat = TY_PIXEL_FORMAT_BGR;
  dst.streamID     = PERCIPIO_STREAM_COLOR;
  dst.timestamp    = src.timestamp;
  dst.imageIndex   = src.imageIndex;
  dst.status       = src.status;
  dst.width        = src.width;
  dst.height       = src.height;

  return true;
}

const std::vector<image_data> PercipioSDK::DeviceStreamRead(const TY_DEV_HANDLE handle, int timeout) {
  std::unique_lock<std::mutex> lock(_mutex);
  int idx = hasDevice(handle);
  if (idx < 0) {
      LOGE("Invalid device handle!");
      m_last_error = TY_STATUS_INVALID_HANDLE;
      return std::vector<image_data>();
  }

  DevList[idx].image_list.clear();

  TY_FRAME_DATA frame;
  m_last_error = TYFetchFrame(handle, &frame, timeout);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYFetchFrame failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return DevList[idx].image_list;
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
      if(DevList[idx].isp && IsBayerColor(frame.image[i].pixelFormat)) {
        image_data dst;
        TyBayerColorConvert(handle, frame.image[i], dst);
        DevList[idx].image_list.push_back(dst);
      } else
        DevList[idx].image_list.push_back(image_data(frame.image[i]));
    }
  }

  m_last_error = TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYEnqueueBuffer failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
  }

  return DevList[idx].image_list;
}

int PercipioSDK::DeviceStreamOff(const TY_DEV_HANDLE handle) {
  std::unique_lock<std::mutex> lock(_mutex);
  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return TY_STATUS_INVALID_HANDLE;
  }

  m_last_error = TYStopCapture(handle);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYStopCapture failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  LOGD("Stream OFF!");
  FrameBufferRelease(handle);

  return m_last_error;
}

float PercipioSDK::DeviceControlReadTemperature(const TY_DEV_HANDLE handle, TY_TEMPERATURE_ID ID)
{
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return NAN;
  }

  m_last_error = TYSetEnum(handle, TY_COMPONENT_DEVICE, TY_ENUM_TEMPERATURE_ID, ID);
  if(m_last_error) {
    LOGE("%s", TYErrorString(m_last_error));
    return m_last_error;
  }
  
  TY_TEMP_DATA  temp;
  m_last_error = TYGetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TEMPERATURE, &temp, sizeof(temp));
  if(m_last_error) {
    LOGE("%s", TYErrorString(m_last_error));
    return m_last_error;
  }

  float f_temp = 0.f;
  sscanf(temp.temp, "%f", &f_temp);
  return f_temp;
}

int PercipioSDK::DeviceControlTriggerModeEnable(const TY_DEV_HANDLE handle, const int enable) {

  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return TY_STATUS_INVALID_HANDLE;
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(!(allComps & TY_COMPONENT_DEVICE)) {
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return m_last_error;
  }

  bool hasTriggerParam = false;
  TYHasFeature(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTriggerParam);
  if(!hasTriggerParam) {
    LOGE("=== Not support feature TY_STRUCT_TRIGGER_PARAM");
    m_last_error = TY_STATUS_INVALID_FEATURE;
    return m_last_error;
  }

  TY_TRIGGER_PARAM trigger;
  if(enable)
    trigger.mode = TY_TRIGGER_MODE_SLAVE;
  else
    trigger.mode = TY_TRIGGER_MODE_OFF;
  m_last_error = TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger));
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYSetStruct failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
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

  return TY_STATUS_OK;
}

int PercipioSDK::DeviceControlTriggerModeSendTriggerSignal(const TY_DEV_HANDLE handle) {

  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return TY_STATUS_INVALID_HANDLE;
  }

  TY_STATUS status;
  while(true) {
    status = TYSendSoftTrigger(handle);
    if(status != TY_STATUS_BUSY)
      break;
  }

  if(status != TY_STATUS_OK) {
    LOGE("TYSendSoftTrigger failed: error %d(%s) at %s:%d", status, TYErrorString(status), __FILENAME__, __LINE__);
    m_last_error = status;
    return status;
  }
  return status;
}

int PercipioSDK::DeviceControlLaserPowerAutoControlEnable(const TY_DEV_HANDLE handle, bool enable) {

  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return TY_STATUS_INVALID_HANDLE;
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(!(allComps & TY_COMPONENT_LASER)) {
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return m_last_error;
  }

  bool hasLaserAuto = false;
  TYHasFeature(handle, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, &hasLaserAuto);
  if(!hasLaserAuto) {
    LOGE("=== Not support feature TY_BOOL_LASER_AUTO_CTRL");
    m_last_error = TY_STATUS_INVALID_FEATURE;
    return m_last_error;
  }

  m_last_error = TYSetBool(handle, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, enable);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYSetBool failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
  }
  return m_last_error;
}

int PercipioSDK::DeviceControlLaserPowerConfig(const TY_DEV_HANDLE handle, int laser) {

  m_last_error = TY_STATUS_OK;
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    m_last_error = TY_STATUS_INVALID_HANDLE;
    return TY_STATUS_INVALID_HANDLE;
  }

  TY_COMPONENT_ID allComps = 0;
  m_last_error = TYGetComponentIDs(handle, &allComps);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYGetComponentIDs failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
    return m_last_error;
  }

  if(!(allComps & TY_COMPONENT_LASER)) {
    m_last_error = TY_STATUS_INVALID_COMPONENT;
    return m_last_error;
  }

  bool hasLaserCtrl = false;
  TYHasFeature(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, &hasLaserCtrl);
  if(!hasLaserCtrl) {
    LOGE("=== Not support feature TY_INT_LASER_POWER");
    m_last_error = TY_STATUS_INVALID_FEATURE;
    return m_last_error;
  }

  m_last_error = TYSetInt(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, laser);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYSetInt failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
  }
  return m_last_error;
}

static int parseIRCsiRaw10(const image_data& src, image_data& dst) {
  int width = src.width;
  int height = src.height;
  if(width & 0x3) {
    LOGE("Invalid ir stream size : %d x %d\n", src.width, src.height);
    return TY_STATUS_INVALID_PARAMETER;
  }

  dst.streamID     = src.streamID;
  dst.timestamp    = src.timestamp;
  dst.imageIndex   = src.imageIndex;
  dst.status       = src.status;
  dst.width        = src.width;
  dst.height       = src.height;
  dst.pixelFormat  = TY_PIXEL_FORMAT_MONO16;
  dst.resize(dst.width * dst.height * 2);

  uint8_t* src_ptr = (uint8_t*)src.buffer;
  uint16_t* dst_ptr = (uint16_t*)dst.buffer;
  int raw10_line_size = 5 * width / 4;
  for(size_t i = 0, j = 0; i < raw10_line_size * height; i+=5, j+=4)
  {
    //[A2 - A9] | [B2 - B9] | [C2 - C9] | [D2 - D9] | [A0A1-B0B1-C0C1-D0D1]
    dst_ptr[j + 0] = (uint16_t)(src_ptr[i + 0] << 2) | ((src_ptr[i + 4] & 0x3)  >> 0);
    dst_ptr[j + 1] = (uint16_t)(src_ptr[i + 1] << 2) | ((src_ptr[i + 4] & 0xc)  >> 2);
    dst_ptr[j + 2] = (uint16_t)(src_ptr[i + 2] << 2) | ((src_ptr[i + 4] & 0x30) >> 4);
    dst_ptr[j + 3] = (uint16_t)(src_ptr[i + 3] << 2) | ((src_ptr[i + 4] & 0xc0) >> 6);
  }
  return TY_STATUS_OK;
}

static int parseIRCsiRaw12(const image_data& src, image_data& dst) {
  int width = src.width;
  int height = src.height;
  if(width & 0x1) {
    LOGE("Invalid ir stream size : %d x %d\n", src.width, src.height);
    return TY_STATUS_INVALID_PARAMETER;
  }

  dst.streamID     = src.streamID;
  dst.timestamp    = src.timestamp;
  dst.imageIndex   = src.imageIndex;
  dst.status       = src.status;
  dst.width        = src.width;
  dst.height       = src.height;
  dst.pixelFormat  = TY_PIXEL_FORMAT_MONO16;
  dst.resize(dst.width * dst.height * 2);

  uint8_t* src_ptr = (uint8_t*)src.buffer;
  uint16_t* dst_ptr = (uint16_t*)dst.buffer;
  int raw12_line_size = 3 * width / 2;
  for(size_t i = 0, j = 0; i < raw12_line_size * height; i+=3, j+=2)
  {
    //[A4 - A11] | [B4 - B11] | [A0A1A2A3-B0B1B2B3]
    dst_ptr[j + 0] = (uint16_t)(src_ptr[i + 0] << 4) | ((src_ptr[i + 2] & 0x0f)  >> 0);
    dst_ptr[j + 1] = (uint16_t)(src_ptr[i + 1] << 4) | ((src_ptr[i + 2] & 0xf0)  >> 4);
  }
  return TY_STATUS_OK;
}

int PercipioSDK::DeviceStreamIRRender(const image_data& src, image_data& dst) {
    m_last_error = TY_STATUS_OK;
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
        m_last_error = TY_STATUS_INVALID_PARAMETER;
        return TY_STATUS_INVALID_PARAMETER;
    }

    return TY_STATUS_OK;
}


#define YUV422_FRAME_CHECK(size, width, height) do {\
  if(size != 2* width * height) {\
    LOGE("Invalid image data size: %d", size); \
    return TY_STATUS_INVALID_PARAMETER; \
  }\
} while(0)

#define BMP_FRAME_CHECK(size, width, height) do {\
  if(size != 3* width * height) {\
    LOGE("Invalid image data size: %d", size); \
    return TY_STATUS_INVALID_PARAMETER; \
  }\
} while(0)

#define RAW8_FRAME_CHECK(size, width, height) do {\
  if(size != width * height) {\
    LOGE("Invalid image data size: %d", size); \
    return TY_STATUS_INVALID_PARAMETER; \
  }\
} while(0)

#define RAW16_FRAME_CHECK(size, width, height) do {\
  if(size != 2 * width * height) {\
    LOGE("Invalid image data size: %d", size); \
    return TY_STATUS_INVALID_PARAMETER; \
  }\
} while(0)


#define XYZ48_FRAME_CHECK(size, width, height) do {\
  if(size != 6 * width * height) {\
    LOGE("Invalid image data size: %d", size); \
    return TY_STATUS_INVALID_PARAMETER; \
  }\
} while(0)

#define CSI_RAW10_FRAME_CHECK(size, width, height) do {\
  if(size != (5 * width * height / 4)) {\
    LOGE("Invalid image data size: %d", size); \
    return TY_STATUS_INVALID_PARAMETER; \
  }\
} while(0)

#define CSI_RAW12_FRAME_CHECK(size, width, height) do {\
  if(size != (3 * width * height / 2)) {\
    LOGE("Invalid image data size: %d", size); \
    return TY_STATUS_INVALID_PARAMETER; \
  }\
} while(0)



static int parseIrFrame(const image_data& src, image_data& dst) {
  if (src.pixelFormat == TY_PIXEL_FORMAT_MONO16 || src.pixelFormat==TY_PIXEL_FORMAT_TOF_IR_MONO16){
    RAW16_FRAME_CHECK(src.size, src.width, src.height);
    dst = src;
    return TY_STATUS_OK;
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO10) {
    //target: TY_PIXEL_FORMAT_MONO16
    CSI_RAW10_FRAME_CHECK(src.size, src.width, src.height);
    return parseIRCsiRaw10(src, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_MONO) {
    //target: TY_PIXEL_FORMAT_MONO
    RAW8_FRAME_CHECK(src.size, src.width, src.height);
    dst = src;
    return TY_STATUS_OK;
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO12) {
    //target: TY_PIXEL_FORMAT_MONO16
    CSI_RAW12_FRAME_CHECK(src.size, src.width, src.height);
    return parseIRCsiRaw12(src, dst);
  } 
  else {
    LOGE("Invalid ir stream pixel format : %d\n", src.pixelFormat);
	  return TY_STATUS_INVALID_PARAMETER;
  }
}

static int parseColorFrame(const image_data& src, image_data& dst) {

  //TODO
  if (src.pixelFormat == TY_PIXEL_FORMAT_JPEG){
    ImgProc::cvtColor(src, ImgProc::IMGPROC_JPEG2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_YVYU) {
    YUV422_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_YVYU2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_YUYV) {
    YUV422_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_YUYV2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_RGB) {
    BMP_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BGR2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_BGR) {
    BMP_FRAME_CHECK(src.size, src.width, src.height);
    dst = src;
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_BAYER8GBRG) {
    RAW8_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8GB2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_BAYER8BGGR) {
    RAW8_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8BG2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_BAYER8GRBG) {
    RAW8_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8GR2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_BAYER8RGGB) {
    RAW8_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_BAYER8RG2RGB888, dst);
  } else if (src.pixelFormat == TY_PIXEL_FORMAT_MONO){
    RAW8_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_MONO2RGB888, dst);
  } 
  
  else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO10) {
    CSI_RAW10_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_MONO102RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10GBRG) {
    CSI_RAW10_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10GB2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10BGGR) {
    CSI_RAW10_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10BG2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10GRBG) {
    CSI_RAW10_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10GR2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER10RGGB) {
    CSI_RAW10_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER10RG2RGB888, dst);
  } 
  
  else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_MONO12) {
    CSI_RAW12_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_MONO122RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12GBRG) {
    CSI_RAW12_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12GB2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12BGGR) {
    CSI_RAW12_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12BG2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12GRBG) {
    CSI_RAW12_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12GR2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_CSI_BAYER12RGGB) {
    CSI_RAW12_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_CSI_BAYER12RG2RGB888, dst);
  } else {

  }

  return TY_STATUS_OK;
}

int PercipioSDK::DeviceStreamDepthRender(const image_data& src, image_data& dst) {
  m_last_error = TY_STATUS_OK;
  if(src.pixelFormat != TY_PIXEL_FORMAT_DEPTH16 && src.pixelFormat != TY_PIXEL_FORMAT_XYZ48){
    LOGE("Invalid pixel format 0x:%x", src.pixelFormat);
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
  }

  if(src.pixelFormat == TY_PIXEL_FORMAT_DEPTH16) {
    RAW16_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_DEPTH2RGB888, dst);
  } else if(src.pixelFormat == TY_PIXEL_FORMAT_XYZ48) {
    XYZ48_FRAME_CHECK(src.size, src.width, src.height);
    ImgProc::cvtColor(src, ImgProc::IMGPROC_XYZ482RGB888, dst);
  }
  return TY_STATUS_OK;
}

int PercipioSDK::DeviceStreamDepthSpeckleFilter(int max_spc_size,  int max_spc_diff, image_data& image) {
  m_last_error = TY_STATUS_OK;
  if(image.pixelFormat != TY_PIXEL_FORMAT_DEPTH16){
    LOGE("Invalid pixel format 0x:%x", image.pixelFormat);
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
  }

  TY_IMAGE_DATA depthImage;
  depthImage.timestamp = image.timestamp;
  depthImage.imageIndex = image.imageIndex;
  depthImage.status = image.status;
  depthImage.size = image.size;
  depthImage.buffer = image.buffer;
  
  depthImage.width = image.width;
  depthImage.height = image.height;
  depthImage.pixelFormat =image.pixelFormat;

  DepthSpeckleFilterParameters para = {max_spc_size, max_spc_diff};
  TYDepthSpeckleFilter(&depthImage,  &para);
  return TY_STATUS_OK;
}

int PercipioSDK::DeviceStreamImageDecode(const image_data& src, image_data& dst) {
  m_last_error = TY_STATUS_OK;
  if (src.streamID == PERCIPIO_STREAM_DEPTH) {
    if (src.pixelFormat == TY_PIXEL_FORMAT_XYZ48) {
      dst = src;
      return TY_STATUS_OK;
    } else if(src.pixelFormat == TY_PIXEL_FORMAT_DEPTH16){
      dst = src;
      return TY_STATUS_OK;
    } else {
      LOGE("Invalid depth stream pixel format : 0x%x\n", src.pixelFormat);
      m_last_error = TY_STATUS_INVALID_PARAMETER;
      return TY_STATUS_INVALID_PARAMETER;
    }
  }
  // get left ir image
  else if (src.streamID == PERCIPIO_STREAM_IR_LEFT) {
    m_last_error = parseIrFrame(src, dst);
    return m_last_error;
  }
  // get right ir image
  else if (src.streamID == PERCIPIO_STREAM_IR_RIGHT) {
    m_last_error = parseIrFrame(src, dst);
    return m_last_error;
  }
  // get BGR
  else if (src.streamID == PERCIPIO_STREAM_COLOR) {
    m_last_error = parseColorFrame(src, dst);
    return m_last_error;
  }
  else {
    LOGE("Invalid stream id : %d\n", src.streamID);
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
  }
}

int PercipioSDK::DeviceStreamMapDepthImageToPoint3D(const image_data& depth, const PercipioCalibData& calib_data, float scale, pointcloud_data_list& p3d) {
  m_last_error = TY_STATUS_OK;
  if(depth.streamID != PERCIPIO_STREAM_DEPTH) {
    LOGE("Invalid stream data: %d.", depth.streamID);
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
  }

  int size = depth.width * depth.height;
  p3d.resize(depth.width, depth.height);

  if(depth.pixelFormat == TY_PIXEL_FORMAT_DEPTH16) {
    TYMapDepthImageToPoint3d(&calib_data.data(),
                                     depth.width, depth.height,
                                     (const uint16_t*)depth.buffer,
                                     (TY_VECT_3F*)p3d.getPtr(), 
                                     scale);
  } else if(depth.pixelFormat == TY_PIXEL_FORMAT_XYZ48) {
    TY_VECT_3F* v3p = (TY_VECT_3F*)p3d.getPtr();
    int16_t* src =  (int16_t*)depth.buffer;
    for (int pix = 0; pix < size; pix++) {
      v3p[pix].x = (float)(*(src + 3*pix + 0) * scale + 0.5f);
      v3p[pix].y = (float)(*(src + 3*pix + 1) * scale + 0.5f);
      v3p[pix].z = (float)(*(src + 3*pix + 2) * scale + 0.5f);
    }
  }
  return TY_STATUS_OK;
}

int PercipioSDK::DeviceStreamDoUndistortion(const PercipioCalibData& calib_data, const image_data& src, image_data& dst) {

  m_last_error = TY_STATUS_OK;
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
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
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

  m_last_error = TYUndistortImage(&calib_data.data(), &src_image, NULL, &dst_image);
  if(m_last_error != TY_STATUS_OK) {
    LOGE("TYUndistortImage failed: error %d(%s) at %s:%d", m_last_error, TYErrorString(m_last_error), __FILENAME__, __LINE__);
  }
  return m_last_error;
}

int PercipioSDK::DeviceStreamMapDepthImageToColorCoordinate(const PercipioCalibData& depth_calib, const image_data& srcDepth, const float scale, 
                                                    const PercipioCalibData& color_calib, const int targetW, const int targetH, 
                                                    image_data& dstDepth)
{
  m_last_error = TY_STATUS_OK;
  if(srcDepth.streamID != PERCIPIO_STREAM_DEPTH) {
    LOGE("Invalid stream data: %d.", srcDepth.streamID);
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
  }

  if(srcDepth.pixelFormat != TY_PIXEL_FORMAT_DEPTH16) {
    LOGE("Invalid depth stream pixel format!");
    return TY_STATUS_INVALID_PARAMETER;
  }

  if(srcDepth.size != 2*srcDepth.width*srcDepth.height) {
    LOGE("Invalid stream size: %d.", srcDepth.size);
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
  }

  std::vector<uint16_t> map_data;
  int outW = srcDepth.width;
  int outH = static_cast<int>(1.0 * srcDepth.width * targetH / targetW + 0.5);
  map_data.resize(outW * outH);
  TYMapDepthImageToColorCoordinate(&depth_calib.data(), srcDepth.width, srcDepth.height, (const uint16_t*)srcDepth.buffer,  
      &color_calib.data(), outW, outH, (uint16_t*)&map_data[0], scale);

  dstDepth.resize(targetW * targetH * 2);
  dstDepth.streamID     = PERCIPIO_STREAM_DEPTH;
  dstDepth.timestamp    = srcDepth.timestamp;
  dstDepth.imageIndex   = srcDepth.imageIndex;
  dstDepth.status       = srcDepth.status;
  dstDepth.width        = targetW;
  dstDepth.height       = targetH;
  dstDepth.pixelFormat  = srcDepth.pixelFormat;
  const double scale_x = static_cast<double>(outW) / dstDepth.width;
  const double scale_y = static_cast<double>(outH) / dstDepth.height;

  uint16_t* dst = static_cast<uint16_t*>(dstDepth.buffer);
  for (int y = 0; y < dstDepth.height; y++) {
    for (int x = 0; x < dstDepth.width; x++) {
      const int src_x = static_cast<int>(std::round(x * scale_x));
      const int src_y = static_cast<int>(std::round(y * scale_y));

      const int safe_x = std::min(std::max(src_x, 0), outW - 1);
      const int safe_y = std::min(std::max(src_y, 0), outH - 1);

      dst[y * dstDepth.width + x] = map_data[safe_y * outW + safe_x];
    }
  }
  return TY_STATUS_OK;
}

int PercipioSDK::DeviceStreamMapRGBImageToDepthCoordinate(const PercipioCalibData& depth_calib, const image_data& srcDepth, const float scale, 
                                                    const PercipioCalibData& color_calib, const image_data& srcColor, 
                                                    image_data& dstColor)
{
  m_last_error = TY_STATUS_OK;
  if(srcDepth.streamID != PERCIPIO_STREAM_DEPTH) {
    LOGE("Invalid stream data: %d.", srcDepth.streamID);
    m_last_error = TY_STATUS_INVALID_PARAMETER;
    return TY_STATUS_INVALID_PARAMETER;
  }

  if(srcDepth.pixelFormat != TY_PIXEL_FORMAT_DEPTH16) {
    LOGE("Invalid depth stream pixel format!");
    return TY_STATUS_INVALID_PARAMETER;
  }

  dstColor.resize(srcDepth.width * srcDepth.height * 3);
  dstColor.streamID     = PERCIPIO_STREAM_COLOR;
  dstColor.timestamp    = srcColor.timestamp;
  dstColor.imageIndex   = srcColor.imageIndex;
  dstColor.pixelFormat  = srcColor.pixelFormat;
  dstColor.status       = srcColor.status;
  dstColor.width        = srcDepth.width;
  dstColor.height       = srcDepth.height;
  switch(srcColor.pixelFormat) {
    case TY_PIXEL_FORMAT_MONO:
    {
      TYMapMono8ImageToDepthCoordinate(
                  &depth_calib.data(), 
                  srcDepth.width, srcDepth.height, (const uint16_t*)srcDepth.buffer,
                  &color_calib.data(),
                  srcColor.width, srcColor.height, (const uint8_t*)srcColor.buffer,
                  (uint8_t*)dstColor.buffer,
                  scale);
      break;
    }
    case TY_PIXEL_FORMAT_RGB:
    case TY_PIXEL_FORMAT_BGR:
      TYMapRGBImageToDepthCoordinate(
                  &depth_calib.data(), 
                  srcDepth.width, srcDepth.height, (const uint16_t*)srcDepth.buffer,
                  &color_calib.data(),
                  srcColor.width, srcColor.height, (const uint8_t*)srcColor.buffer,
                  (uint8_t*)dstColor.buffer,
                  scale);
      break;
    case TY_PIXEL_FORMAT_RGB48:
    case TY_PIXEL_FORMAT_BGR48:
      TYMapRGB48ImageToDepthCoordinate(
                  &depth_calib.data(), 
                  srcDepth.width, srcDepth.height, (const uint16_t*)srcDepth.buffer,
                  &color_calib.data(),
                  srcColor.width, srcColor.height, (const uint16_t*)srcColor.buffer,
                  (uint16_t*)dstColor.buffer,
                  scale);
      break;
    case TY_PIXEL_FORMAT_MONO16:
      TYMapMono16ImageToDepthCoordinate(
                  &depth_calib.data(), 
                  srcDepth.width, srcDepth.height, (const uint16_t*)srcDepth.buffer,
                  &color_calib.data(),
                  srcColor.width, srcColor.height, (const uint16_t*)srcColor.buffer,
                  (uint16_t*)dstColor.buffer,
                  scale);
      break;
  }
  return TY_STATUS_OK;
}

void PercipioSDK::Close(const TY_DEV_HANDLE handle)
{
  int idx = hasDevice(handle);
  if(idx < 0) {
    LOGE("Invalid device handle!");
    return ;
  }

  if(DevList[idx].isp) {
    TYISPRelease(&DevList[idx].isp);
    DevList[idx].isp = NULL;
  }

  LOGD("Device %s is close!", DevList[idx].devID.c_str());
  DevList.erase(DevList.begin() + idx);
  
  TYCloseDevice(handle);
}
