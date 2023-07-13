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


%typemap(out) image_data [ANY] {
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
%CARRAY_ITEM_ASSIGN(TY_EVENT_INFO);

%CARRAY_ITEM_ASSIGN(image_data);


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


//http://swig.org/Doc4.0/Typemaps.html#Typemaps_pattern_matching
//change TYISPRelease  calling conventions  to pass handle input
%rename(TYISPRelease) TYISPRelease_org;
%inline %{
    void TYISPRelease_org(TY_ISP_HANDLE handle){
        TYISPRelease(&handle);
    }
%}


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

%include "std_vector.i"

%template(ty_enum_vector) std::vector<TY_ENUM_ENTRY>;
%template(ty_image_data_vector) std::vector<image_data>;
%template(ty_float_vector) std::vector<float>;



//%constant void cb(TY_EVENT_INFO *event_info, void *userdata);
//%constant void func(TY_EVENT_INFO *event_info, void *userdata);
//int binary_op(int a, int b, int (*op)(int, int));
//bool PercipioSDK::DeviceRegisterEventCallBack(const TY_DEV_HANDLE , const TY_EVENT_CALLBACK );

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

//extend for carray to convert numpy array to carray
%define %NUMPY_TO_C_ARRAY(element_type)
    //input 
    %apply ( element_type* IN_ARRAY1, int DIM1) { (element_type* NP_ARRAY_PTR, int ROW)}
    %apply ( element_type* IN_ARRAY2, int DIM1, int DIM2 ) { (element_type* NP_ARRAY_PTR, int ROW,  int COL )}
    %apply ( element_type* IN_ARRAY3, int DIM1, int DIM2 , int DIM3) { (element_type* NP_ARRAY_PTR, int ROW,  int COL , int PSZ)}
    //output 

    %apply (element_type** ARGOUTVIEW_ARRAY1, int* DIM1) { (element_type**NP_ARRAY_PTR , int *ROW) }
    %apply (element_type** ARGOUTVIEW_ARRAY2, int* DIM1, int* DIM2) { (element_type**NP_ARRAY_PTR , int* ROW,int* COL) }
    %apply (element_type** ARGOUTVIEW_ARRAY3, int* DIM1, int* DIM2,int* DIM3) { (element_type**NP_ARRAY_PTR , int *ROW,int *COL,int *PSZ) }
    //extend convert method
    %extend element_type##_ARRAY{
        static element_type##_ARRAY* from_nparray1d(element_type* NP_ARRAY_PTR, int ROW) {
            int sz= ROW;
            element_type* out = new element_type[sz]();
            for(int idx=0;idx<sz;idx++){
                out[idx] = NP_ARRAY_PTR[idx];
            }
            return (element_type##_ARRAY*)out;
        }

        static element_type##_ARRAY* from_nparray2d(element_type* NP_ARRAY_PTR, int ROW,  int COL) {
            int sz= ROW*COL;
            element_type* out = new element_type[sz]();
            for(int idx=0;idx<sz;idx++){
                out[idx] = NP_ARRAY_PTR[idx];
            }
            return (element_type##_ARRAY*)out;
        }

        static element_type##_ARRAY* from_nparray3d(element_type* NP_ARRAY_PTR, int ROW,  int COL, int PSZ) {
            int sz= ROW*COL*PSZ;
            element_type* out = new element_type[sz]();
            for(int idx=0;idx<sz;idx++){
                out[idx] = NP_ARRAY_PTR[idx];
            }
            return (element_type##_ARRAY*)out;
        }

        static void Release(element_type##_ARRAY* ptr) {
            delete(ptr);
        }

        static void ptr_as_nparray1d (element_type** NP_ARRAY_PTR, int* ROW,void * ptr, int row) {
            *NP_ARRAY_PTR = (element_type*)ptr;
            *ROW = row;
        }

        static void ptr_as_nparray2d (element_type** NP_ARRAY_PTR, int* ROW,  int* COL ,
                                      void *ptr,int row, int col) {
            *NP_ARRAY_PTR = (element_type*)ptr;
            *ROW = row;
            *COL = col;
        }
    
        static void ptr_as_nparray3d (element_type** NP_ARRAY_PTR, int* ROW,  int* COL,  int *PSZ,
                                   void * ptr, int row, int col,int psz) {
            *NP_ARRAY_PTR = (element_type*)ptr;
            *ROW = row;
            *COL = col;
            *PSZ = psz;
        }

        void as_nparray1d (element_type** NP_ARRAY_PTR, int* ROW, int row) {
            *NP_ARRAY_PTR = self;
            *ROW = row;
        }

        void as_nparray2d (element_type** NP_ARRAY_PTR, int* ROW,  int* COL,
                                   int row, int col) {
            *NP_ARRAY_PTR = self;
            *ROW = row;
            *COL = col;
        }
    
        void as_nparray3d (element_type** NP_ARRAY_PTR, int* ROW,  int* COL,  int *PSZ,
                                   int row, int col,int psz) {
            *NP_ARRAY_PTR = self;
            *ROW = row;
            *COL = col;
            *PSZ = psz;
        }


%pythoncode %{
        
            @staticmethod
            def from_nparray(np_array):
                dim = np_array.ndim
                if dim==3:
                    return element_type##_ARRAY.from_nparray3d(np_array)
                elif dim==2: 
                    return element_type##_ARRAY.from_nparray2d(np_array)
                elif dim==1: 
                    return element_type##_ARRAY.from_nparray1d(np_array)
                else:
                    raise Exception('not support format')

            @staticmethod
            def release(ptr):
                element_type##_ARRAY.Release(ptr)

        %}// %pythoncode %{
    } //extend array


%enddef

%NUMPY_TO_C_ARRAY(float)
%NUMPY_TO_C_ARRAY(uint8_t)
%NUMPY_TO_C_ARRAY(uint16_t)
%NUMPY_TO_C_ARRAY(uint32_t)
%NUMPY_TO_C_ARRAY(int8_t)
%NUMPY_TO_C_ARRAY(int16_t)
%NUMPY_TO_C_ARRAY(int32_t)

//TY_VECT_3F_ARRAY////////////
%apply ( double* IN_ARRAY2, int DIM1, int DIM2 ) { (double* NP_ARRAY_PTR, int ROW,  int COL )}
%extend TY_VECT_3F_ARRAY{
%pythoncode %{
    def as_nparray(self,arr_size):
        return float_ARRAY.ptr_as_nparray2d(self.VoidPtr(),arr_size,3)
%}

static TY_VECT_3F_ARRAY* from_nparray(double* NP_ARRAY_PTR, int ROW,  int COL) {
    if (COL!=3){
        printf( "from_nparray:invalid numpy format , column should be 3");
        return NULL;
    }
    TY_VECT_3F* out = new TY_VECT_3F[ROW]();
    int src_idx =0 ;
    for(int idx=0; idx<ROW; idx++,src_idx+=3){
        out[idx].x = NP_ARRAY_PTR[src_idx  ];
        out[idx].y = NP_ARRAY_PTR[src_idx+1];
        out[idx].z = NP_ARRAY_PTR[src_idx+2];
    }
    return (TY_VECT_3F_ARRAY*)out;
}

}// %extend TY_VECT_3F_ARRAY{


%extend TY_IMAGE_DATA {
%pythoncode %{
__U8CX = [TY_PIXEL_FORMAT_MJPG, TY_PIXEL_FORMAT_JPEG]
__U8C1 = [TY_PIXEL_FORMAT_MONO , TY_PIXEL_FORMAT_BAYER8GB , TY_PIXEL_FORMAT_BAYER8BG]
__U8C2 = [TY_PIXEL_FORMAT_YVYU,TY_PIXEL_FORMAT_YUYV]
__U8C3 = [TY_PIXEL_FORMAT_RGB,TY_PIXEL_FORMAT_BGR]
__U16C1 = [TY_PIXEL_FORMAT_DEPTH16 , TY_PIXEL_FORMAT_MONO16]

def as_nparray(self):
    '''
    convert image to numpy array
    *** YOU SHOULD COPY DATA TO YOUR OWN ARRAY BEFORE invoking Enqueuebuffer  ***
    '''
    if self.buffer==None or self.width<=0 or self.height<=0:
        return None
    pformat = self.pixelFormat
    if pformat in self.__U8CX:
        sz = self.size
        return uint8_t_ARRAY.ptr_as_nparray1d(self.buffer,self.size)
    if pformat in self.__U8C1:
        sz = self.height*self.width
        return uint8_t_ARRAY.ptr_as_nparray2d(self.buffer,self.height,self.width)
    elif pformat  in self.__U8C2:
        return uint8_t_ARRAY.ptr_as_nparray3d(self.buffer,self.height,self.width,2)
    elif pformat  in self.__U8C3:
        sz = self.height*self.width*3
        return uint8_t_ARRAY.ptr_as_nparray3d(self.buffer,self.height,self.width,3)
    elif pformat  in self.__U16C1:
        sz = self.height*self.width
        return uint16_t_ARRAY.ptr_as_nparray3d(self.buffer,self.height,self.width,1)
    else:
        raise Exception('not supported format {}'.format(pformat))
    return None    
%}
}//endof  %extend TY_IMAGE_DATA 


%extend image_data {
%pythoncode %{
__U8CX = [TY_PIXEL_FORMAT_MJPG, TY_PIXEL_FORMAT_JPEG]
__U8C1 = [TY_PIXEL_FORMAT_MONO , TY_PIXEL_FORMAT_BAYER8GB , TY_PIXEL_FORMAT_BAYER8BG]
__U8C2 = [TY_PIXEL_FORMAT_YVYU,TY_PIXEL_FORMAT_YUYV]
__U8C3 = [TY_PIXEL_FORMAT_RGB,TY_PIXEL_FORMAT_BGR]
__U16C1 = [TY_PIXEL_FORMAT_DEPTH16 , TY_PIXEL_FORMAT_MONO16]

def as_nparray(self):
    '''
    convert image to numpy array
    *** YOU SHOULD COPY DATA TO YOUR OWN ARRAY BEFORE invoking Enqueuebuffer  ***
    '''
    if self.buffer==None or self.width<=0 or self.height<=0:
        return None
    pformat = self.pixelFormat
    if pformat in self.__U8CX:
        sz = self.size
        return uint8_t_ARRAY.ptr_as_nparray1d(self.buffer,self.size)
    if pformat in self.__U8C1:
        sz = self.height*self.width
        return uint8_t_ARRAY.ptr_as_nparray2d(self.buffer,self.height,self.width)
    elif pformat  in self.__U8C2:
        return uint8_t_ARRAY.ptr_as_nparray3d(self.buffer,self.height,self.width,2)
    elif pformat  in self.__U8C3:
        sz = self.height*self.width*3
        return uint8_t_ARRAY.ptr_as_nparray3d(self.buffer,self.height,self.width,3)
    elif pformat  in self.__U16C1:
        sz = self.height*self.width
        return uint16_t_ARRAY.ptr_as_nparray3d(self.buffer,self.height,self.width,1)
    else:
        raise Exception('not supported format {}'.format(pformat))
    return None    
%}
}//endof  %extend image_data 
#endif //WITH_NUMPY


//ISP////////////////////////////////////////////////


%C_ARRAY_BUFFER_DEF(TY_ISP_FEATURE_INFO);
%CARRAY_ITEM_ASSIGN(TY_ISP_FEATURE_INFO);


//isp helper functions
%inline %{
    TY_STATUS PY_TYISPSetDeviceHandle(TY_ISP_HANDLE isp_handle, TY_DEV_HANDLE dev_handle){
        return TYISPSetFeature(isp_handle, TY_ISP_FEATURE_CAM_DEV_HANDLE, (uint8_t*)&dev_handle, sizeof(dev_handle));
    }
%}


