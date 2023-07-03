
//ENUM //////////////////////////////////////////////////////////
%include <enumsimple.swg>
//because TY sdk api header file contains some bit operation , 
//we have to using enumsimple to define enum as const int

%csconst(1);


%define ENUM_UINT_TYPE_DEFINE(type_name)
%typemap(ctype) enum type_name "unsigned int"
%typemap(imtype) enum type_name "uint"
%typemap(csvarout)  enum type_name %{
    get {
      uint ret = $imcall;
      return ret;
    } 
%}
%typemap(cstype) enum type_name "uint"
%typemap(csout)  enum type_name {
      uint ret = $imcall;
      return ret;
}
%enddef


%define ENUM_INT_TYPE_DEFINE(type_name)
%typemap(ctype) enum type_name "signed int"
%typemap(imtype) enum type_name "int"
%typemap(csvarout)  enum type_name %{
    get {
      int ret = $imcall;
      return ret;
    } 
%}
%typemap(cstype) enum type_name "int"
%typemap(csout)  enum type_name {
      int ret = $imcall;
      return ret;
}
%enddef

ENUM_UINT_TYPE_DEFINE(TY_FEATURE_TYPE_LIST)
ENUM_UINT_TYPE_DEFINE(TY_DEVICE_COMPONENT_LIST)
ENUM_UINT_TYPE_DEFINE(TY_FEATURE_ID_LIST)
ENUM_UINT_TYPE_DEFINE(TY_ISP_FEATURE_ID)
ENUM_UINT_TYPE_DEFINE(TY_ISP_FEATURE_INFO)
ENUM_INT_TYPE_DEFINE(TY_PIXEL_BITS_LIST)
ENUM_INT_TYPE_DEFINE(TY_IMAGE_MODE_LIST)
ENUM_INT_TYPE_DEFINE(TY_PIXEL_FORMAT_LIST)

//--------------------------------------------------

%define %UINT_TYPE(type_name ,var_name )
%typemap(cstype) type_name var_name  "uint"
%typemap(imtype) type_name var_name  "uint"
%typemap(ctype) type_name var_name "unsigned int"
%typemap(csvarout)   type_name  var_name %{
    get {
      uint ret = $imcall;
      return ret;
    } 
%}
%typemap(csout)  type_name  var_name{
      uint ret = $imcall;
      return ret;
}
%enddef

%UINT_TYPE(TY_FEATURE_TYPE,);
%UINT_TYPE(TY_FEATURE_ID,);
%UINT_TYPE(TY_COMPONENT_ID,);
%UINT_TYPE(TY_IMAGE_MODE,);
%UINT_TYPE(TY_PIXEL_FORMAT,);
%UINT_TYPE(int,componentIDs);


//%apply int32_t {uint32_t componentID} 
//%apply int32_t {uint32_t featureID} 

//C ARRAY DEFINE //////////////////////////////////////////////////////////


%define %CARRAY_ITEM_ASSIGN(type_name,cs_typename)
%typemap(cscode) type_name##_ARRAY  %{
	public cs_typename this [int idx]{
		get{
			return getitem(idx);
		}
		set{
			setitem(idx,value);
		}
	}

	public global::System.IntPtr VoidPtr2(){
		return (global::System.IntPtr)swigCPtr;
	}

	static public type_name##_ARRAY FromIntPtr(global::System.IntPtr t) {
		type_name##_ARRAY var = new type_name##_ARRAY(t, false);
		return var ;
	}
%}

%enddef


%CARRAY_ITEM_ASSIGN(float, float);
%CARRAY_ITEM_ASSIGN(uint8_t, byte);
%CARRAY_ITEM_ASSIGN(uint16_t, ushort);
%CARRAY_ITEM_ASSIGN(uint32_t, uint);
%CARRAY_ITEM_ASSIGN(int32_t, int);
%CARRAY_ITEM_ASSIGN(char, char);
%CARRAY_ITEM_ASSIGN(TY_VECT_3F, TY_VECT_3F);
%CARRAY_ITEM_ASSIGN(TY_PIXEL_DESC, TY_PIXEL_DESC);
%CARRAY_ITEM_ASSIGN(TY_INTERFACE_INFO, TY_INTERFACE_INFO);
%CARRAY_ITEM_ASSIGN(TY_DEVICE_BASE_INFO, TY_DEVICE_BASE_INFO);
%CARRAY_ITEM_ASSIGN(TY_FEATURE_INFO, TY_FEATURE_INFO);
%CARRAY_ITEM_ASSIGN(TY_ENUM_ENTRY, TY_ENUM_ENTRY);
%CARRAY_ITEM_ASSIGN(TY_FRAME_DATA, TY_FRAME_DATA);
%CARRAY_ITEM_ASSIGN(TY_IMAGE_DATA, TY_IMAGE_DATA);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_INTRINSIC, TY_CAMERA_INTRINSIC);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_EXTRINSIC, TY_CAMERA_EXTRINSIC);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_DISTORTION, TY_CAMERA_DISTORTION);
%CARRAY_ITEM_ASSIGN(TY_CAMERA_CALIB_INFO, TY_CAMERA_CALIB_INFO);
//ARRAY BUFFER ARGOUT ////////////////////////////////////////

%include <std_vector.i>
namespace std
{
  %template(DeviceInfoVector) vector<TY_DEVICE_BASE_INFO>;
  %template(EnumEntryVector) vector<TY_ENUM_ENTRY>;
}
%CARRAY_ITEM_ASSIGN(DeviceInfoVector, TY_DEVICE_BASE_INFO);


%define %ARRAY_BUFFER_INPUT(typename ,varname)
%typemap(cstype) typename *varname %{typename##_ARRAY %}
%typemap(csin) typename *varname %{ typename.getCPtr($csinput.cast())  %}
%enddef

%ARRAY_BUFFER_INPUT(TY_INTERFACE_INFO,pIfaceInfos);
%ARRAY_BUFFER_INPUT(TY_DEVICE_BASE_INFO,deviceInfos);
%ARRAY_BUFFER_INPUT(TY_ENUM_ENTRY,entries);
%ARRAY_BUFFER_INPUT(TY_IMAGE_DATA,depthImages);

// ARRAYS  ////////////////////////////////////////////////////////////

%define %CARRAY_ACCESS(typename)
%typemap(cstype) typename[ANY]  %{ typename##_ARRAY %}
%typemap(csin)   typename[ANY]  "$csclassname.getCPtr(value.cast())"
%typemap(csvarin) typename[ANY] %{
	set{
		$imcall;
	}
%}
%typemap(csvarout) typename[ANY]  %{
	get{
	 global::System.IntPtr cPtr =	$imcall; $excode
	 $csclassname ptr = new $csclassname(cPtr,false);
	 return typename##_ARRAY.frompointer(ptr);
	}
%}

%enddef 

%CARRAY_ACCESS(float)
%CARRAY_ACCESS(int32_t)
%CARRAY_ACCESS(TY_IMAGE_DATA)


//HANDLE//////////////////////////////////////////////////////////

%typemap(ctype) void** INPUT "void**"
%typemap(imtype) void** INPUT "ref global::System.IntPtr"
%typemap(cstype) void** INPUT "ref global::System.IntPtr"
%typemap(csin) void** INPUT %{ ref $csinput %}

%typemap(ctype) void* INPUT "void*"
%typemap(imtype) void* INPUT " global::System.IntPtr"
%typemap(cstype) void* INPUT " global::System.IntPtr"
%typemap(csin) void* INPUT %{  $csinput %}

// TY_INTERFACE_HANDLE ,TY_DEV_HANDLE
%apply void** INPUT{TY_INTERFACE_HANDLE *, TY_DEV_HANDLE * , TY_ISP_HANDLE* , TY_FW_ERRORCODE*}
%apply void* INPUT{TY_INTERFACE_HANDLE , TY_DEV_HANDLE  , TY_ISP_HANDLE , TY_FW_ERRORCODE}


//STRUCT //////////////////////////////////////////////////

%define %CS_STRUCT_EXTEND(tname)
%typemap(cscode) tname %{
  public System.IntPtr getCPtr() {
    return tname.getCPtr(this).Handle;
  }
%}
%enddef


%CS_STRUCT_EXTEND(TY_INTERFACE_INFO);
%CS_STRUCT_EXTEND(TY_DEVICE_BASE_INFO);
%CS_STRUCT_EXTEND(TY_IMAGE_DATA);
%CS_STRUCT_EXTEND(TY_INT_RANGE);
%CS_STRUCT_EXTEND(TY_VECT_3F);
%CS_STRUCT_EXTEND(TY_PIXEL_DESC);
%CS_STRUCT_EXTEND(TY_TRIGGER_PARAM);
%CS_STRUCT_EXTEND(TY_TRIGGER_PARAM_EX);
%CS_STRUCT_EXTEND(TY_CAMERA_STATISTICS);
%CS_STRUCT_EXTEND(TY_CAMERA_INTRINSIC);
%CS_STRUCT_EXTEND(TY_CAMERA_EXTRINSIC);
%CS_STRUCT_EXTEND(TY_CAMERA_DISTORTION);
%CS_STRUCT_EXTEND(TY_CAMERA_CALIB_INFO);


%typemap(cstype) int32_t* outFwErrorcode "System.IntPtr"
%typemap(imtype) int32_t* outFwErrorcode "System.IntPtr"
%typemap(ctype) void* outFwErrorcode "void *"
%typemap(csin) void* outFwErrorcode %{  $csinput %}
%typemap(cstype) void* pStruct "System.IntPtr"
%typemap(imtype) void* pStruct "System.IntPtr"
%typemap(ctype) void* pStruct "void *"
%typemap(csin) void* pStruct %{  $csinput %}


//return code to exception  //////////////////////////////////////////////////

%typemap(cstype) TY_STATUS "int"
%typemap(csout) TY_STATUS {
	int ret;
	ret = $imcall;$excode
//	if (ret!=pcammls.TY_STATUS_OK){
//		var ex = new System.ComponentModel.Win32Exception(ret,string.Format("Failed with return code:{0}",ret));
//		throw ex;
//	}
    return ret;
}

%typemap(cstype) TY_STATUS errorID "int"

