//https://github.com/Softwariness/SwigUsageExample
//https://github.com/swig/swig/tree/master/Examples/csharp/callback


//helper function to create a python list 
%{

template<typename T>
PyObject* _CreatePyList(const T* data, size_t num,swig_type_info* ptype_info) {
    PyObject* out_list = PyList_New(0);
    for (size_t idx = 0; idx < num; idx++) {
        T* rr = new T(data[idx]);
        PyObject* temp = SWIG_NewPointerObj(SWIG_as_voidptr(rr), ptype_info, SWIG_POINTER_OWN |  0 );
        PyList_Append(out_list, temp);
        Py_DECREF(temp);
    }
	return out_list;
}

%}


%typemap(check) uint32_t _POSITIVE_VALUE{
    if ($1 <= 0) {
        SWIG_exception_fail(SWIG_ValueError, "Expection: positive count value");
   }
}
%apply uint32_t _POSITIVE_VALUE { uint32_t entryCount, uint32_t bufferCount }

//for carray & buffer///////////////////////////////////////////////

%typemap(check) int index{
	if ($1 < 0) {
		SWIG_exception_fail(SWIG_IndexError, "Expection: negative index");
   }
}

%typemap(out) float [ANY] {
	int i;
	$result = PyList_New($1_dim0);
	for (i = 0; i < $1_dim0; i++) {
		PyObject* o = SWIG_From_double((double)$1[i]);
		PyList_SetItem($result, i, o);
	}
}

%typemap(out) TY_IMAGE_DATA [ANY] {
	$result = _CreatePyList($1, $1_dim0,$1_descriptor);
}

%define %CARRAY_ITEM_ASSIGN(type_name)
%rename(__getitem__) type_name##_ARRAY::getitem;
%rename(__setitem__) type_name##_ARRAY::setitem;
%enddef


%CARRAY_ITEM_ASSIGN(float);
%CARRAY_ITEM_ASSIGN(uint8_t);
%CARRAY_ITEM_ASSIGN(uint16_t);
%CARRAY_ITEM_ASSIGN(uint32_t);
%CARRAY_ITEM_ASSIGN(int32_t);
%CARRAY_ITEM_ASSIGN(char);
%CARRAY_ITEM_ASSIGN(TY_VECT_3F);
%CARRAY_ITEM_ASSIGN(TY_PIXEL_DESC);
%CARRAY_ITEM_ASSIGN(TY_INTERFACE_INFO);
%CARRAY_ITEM_ASSIGN(TY_DEVICE_BASE_INFO);
%CARRAY_ITEM_ASSIGN(TY_FEATURE_INFO);
%CARRAY_ITEM_ASSIGN(TY_ENUM_ENTRY);
%CARRAY_ITEM_ASSIGN(TY_FRAME_DATA);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_INTRINSIC);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_EXTRINSIC);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_DISTORTION);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_CALIB_INFO);

//PVOID HANDLE /////////////////////////////////////////////////////////////////////////////
%{
	typedef void* _HANDLE;
%}


%typemap(in,numinputs=0) _HANDLE * OUTPUT (_HANDLE temp_if_handle) {
    temp_if_handle = nullptr;
    $1 = &temp_if_handle;
}

%typemap(argout) _HANDLE* OUTPUT{
    PyObject* temp = SWIG_NewPointerObj(SWIG_as_voidptr(*$1), SWIGTYPE_p_void, 0 | 0);
    $result = SWIG_Python_AppendOutput($result, temp);
}

// TY_INTERFACE_HANDLE ,TY_DEV_HANDLE
%apply _HANDLE* OUTPUT{TY_INTERFACE_HANDLE *, TY_DEV_HANDLE * , TY_ISP_HANDLE*}

// Utils.h selectDevice  //////////////////////////////////////////////////
%{
	#include <vector>
%}

%typemap(in, numinputs = 0) std::vector<TY_DEVICE_BASE_INFO>& out (std::vector<TY_DEVICE_BASE_INFO> buff) {
    $1 = &buff;
}

%typemap(argout) std::vector<TY_DEVICE_BASE_INFO>& out{//return a list of  TY_DEVICE_BASE_INFO
    std::vector< TY_DEVICE_BASE_INFO > &vec =*$1; 
	PyObject* out_list = _CreatePyList(&vec[0], vec.size(),SWIGTYPE_p_TY_DEVICE_BASE_INFO);
    $result = SWIG_Python_AppendOutput($result, out_list);
}

//return code to exception//////////////////////////////////////////////////////////

%typemap(out) TY_STATUS {
	if ($1!=TY_STATUS_OK){
		char msg_buff[1024];
		sprintf(msg_buff,"method $symname result got error code :%d",$1);
		SWIG_exception_fail(SWIG_ArgError(SWIG_RuntimeError), msg_buff); 
	}
	$result = SWIG_Py_Void();	
}

//numpy////////////////////////////////////////////////

#ifdef WITH_NUMPY

%{
#define SWIG_FILE_WITH_INIT
%}
%include "numpy/numpy.i"

%init %{
  import_array();
%}


%define MAKE_NPARRAY_CONVERT(data_type)
%apply (data_type** ARGOUTVIEW_ARRAY2, int* DIM1, int* DIM2) {
	(data_type **NP_ARRAY_PTR , int *ROW,int *COL)
}
%apply (data_type** ARGOUTVIEW_ARRAY3, int* DIM1, int* DIM2,int* DIM3) {
	(data_type **NP_ARRAY_PTR , int *ROW,int *COL,int *PSZ)
}
%extend struct TY_IMAGE_DATA {
	void __as_nparray_##data_type##_ch1(data_type** NP_ARRAY_PTR, int* ROW, int* COL) {
		*NP_ARRAY_PTR = (data_type*)self->buffer;
		*ROW = self->height;
		*COL = self->width;
	}
	void __as_nparray_##data_type##_ch2(data_type** NP_ARRAY_PTR, int* ROW, int* COL,int*PSZ) {
		*NP_ARRAY_PTR = (data_type*)self->buffer;
		*ROW = self->height;
		*COL = self->width;
		*PSZ = 2;
	}
	void __as_nparray_##data_type##_ch3(data_type** NP_ARRAY_PTR, int* ROW, int* COL,int*PSZ) {
		*NP_ARRAY_PTR = (data_type*)self->buffer;
		*ROW = self->height;
		*COL = self->width;
		*PSZ = 3;
	}
}
%enddef // %define MAKE_NPARRAY_CONVERT(data_type)

MAKE_NPARRAY_CONVERT(float)
MAKE_NPARRAY_CONVERT(uint16_t)
MAKE_NPARRAY_CONVERT(uint8_t)

%extend TY_IMAGE_DATA {
%pythoncode %{

__U8C1 = [TY_PIXEL_FORMAT_MONO , TY_PIXEL_FORMAT_BAYER8GB , TY_PIXEL_FORMAT_MJPG, TY_PIXEL_FORMAT_JPEG]
__U8C2 = [TY_PIXEL_FORMAT_YVYU,TY_PIXEL_FORMAT_YUYV]
__U8C3 = [TY_PIXEL_FORMAT_RGB,TY_PIXEL_FORMAT_BGR]
__U16C1 = [TY_PIXEL_FORMAT_DEPTH16]

def as_nparray(self):
	'''
	convert image to numpy array
	*** YOU SHOULD COPY DATA TO YOUR OWN ARRAY BEFORE invoking Enqueuebuffer  ***
	'''
	if self.buffer==None or self.width<=0 or self.height<=0:
		return None
	pformat = self.pixelFormat
	if pformat in self.__U8C1:
		return self.__as_nparray_uint8_t_ch1()
	elif pformat  in self.__U8C2:
		return self.__as_nparray_uint8_t_ch2()
	elif pformat  in self.__U8C3:
		return self.__as_nparray_uint8_t_ch3()
	elif pformat  in self.__U16C1:
		return self.__as_nparray_uint16_t_ch1()
	else:
		raise Exception('not supported format {}'.format(pformat))
	return None	
%}
}//endof  %extend TY_IMAGE_DATA 

#endif




