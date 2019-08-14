%module(directors="1", "threads" = 1) pcammls

%{
#include "TYApi.h"
#include "TYImageProc.h"
#include "TyIsp.h"
#include "../sample/common/Utils.hpp"
%}


%include <stdint.i>
%include <typemaps.i>
%include "std_string.i"
%include "carrays.i"
%feature("autodoc", "2");


#define TY_STATIC_LIB 1

#ifdef SWIGPYTHON                      
%include "py_extend.i"
#endif

#ifdef SWIGCSHARP
%include "csharp_extend.i"
#endif

%define %C_ARRAY_BUFFER_DEF(element_type)
%array_class(element_type, element_type##_ARRAY);
%extend element_type##_ARRAY{
    static element_type##_ARRAY * FromVoidPtr(void* t) {
        return (element_type##_ARRAY*)(t);
    }
}
%enddef

%C_ARRAY_BUFFER_DEF(float);
%C_ARRAY_BUFFER_DEF(uint8_t);
%C_ARRAY_BUFFER_DEF(uint16_t);
%C_ARRAY_BUFFER_DEF(uint32_t);
%C_ARRAY_BUFFER_DEF(int32_t);
%C_ARRAY_BUFFER_DEF(char);
%C_ARRAY_BUFFER_DEF(TY_VECT_3F);
%C_ARRAY_BUFFER_DEF(TY_INTERFACE_INFO);
%C_ARRAY_BUFFER_DEF(TY_DEVICE_BASE_INFO);
%C_ARRAY_BUFFER_DEF(TY_FEATURE_INFO);
%C_ARRAY_BUFFER_DEF(TY_ENUM_ENTRY);
%C_ARRAY_BUFFER_DEF(TY_IMAGE_DATA);
%C_ARRAY_BUFFER_DEF(TY_FRAME_DATA);


%extend class TY_DEVICE_BASE_INFO{//deal with nested union which not suported by SWIG
    TY_DEVICE_NET_INFO get_netinfo() const {
        return self->netInfo;
    }

    TY_DEVICE_USB_INFO get_usbinfo() const {
        return self->usbInfo;
    }
}

%define %STRUCT_PTR_EXTEND(name)
%extend  struct name {
    /*!c struct size */
    size_t CSize() {
        return sizeof(name);
    }
}
%enddef


%STRUCT_PTR_EXTEND(TY_FEATURE_INFO)
%STRUCT_PTR_EXTEND(TY_INT_RANGE)
%STRUCT_PTR_EXTEND(TY_FLOAT_RANGE)
%STRUCT_PTR_EXTEND(TY_VECT_3F)
%STRUCT_PTR_EXTEND(TY_CAMERA_INTRINSIC)
%STRUCT_PTR_EXTEND(TY_CAMERA_EXTRINSIC)
%STRUCT_PTR_EXTEND(TY_CAMERA_DISTORTION)
%STRUCT_PTR_EXTEND(TY_CAMERA_CALIB_INFO)
%STRUCT_PTR_EXTEND(TY_TRIGGER_PARAM)
%STRUCT_PTR_EXTEND(TY_TRIGGER_PARAM_EX)
%STRUCT_PTR_EXTEND(TY_CAMERA_STATISTICS)
%STRUCT_PTR_EXTEND(TY_IMAGE_DATA)
%STRUCT_PTR_EXTEND(TY_FRAME_DATA)

%apply uint32_t* OUTPUT {uint32_t* pNumIfaces}
%apply uint32_t* OUTPUT {uint32_t* filledCount}
%apply uint32_t* OUTPUT {uint32_t* filledDeviceCount}
%apply uint32_t* OUTPUT {uint32_t* filledEntryCount}
%apply uint32_t* OUTPUT {uint32_t *deviceNumber }
%apply uint32_t* OUTPUT {int32_t* componentIDs}
%apply int32_t* OUTPUT {int32_t* value}
%apply uint32_t* OUTPUT {uint32_t *bufferSize}
%apply uint32_t* OUTPUT {uint32_t* entryCount}
%apply uint32_t* OUTPUT {uint32_t* pSize}
%apply bool* OUTPUT {bool*value}
%apply float* OUTPUT {float* value}


//event callback
%feature("director") EventCallback;
%inline %{

	class EventCallback{
	public:
		virtual ~EventCallback() {}
		virtual void run(TY_EVENT_INFO* info) {};

		TY_STATUS set_to(TY_DEV_HANDLE handle) {
			return TYRegisterEventCallback(handle, __ty_event_callback, this);
		}

	private:

		static void __ty_event_callback (TY_EVENT_INFO* info, void* userdata) {
			EventCallback* ptr = (EventCallback*)(userdata);
			if (ptr == nullptr) {
				return;
			}
			ptr->run(info);
		}

	};

%}


%include "TYApi.h"
%include "TYImageProc.h"
%include "TyIsp.h"
%include "../sample/common/Utils.hpp"



