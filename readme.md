Percipio depth CAMera  MultiLanguage Sdk
=======================================
图漾深度相机多语言接口封装库

[![Build status](https://ci.appveyor.com/api/projects/status/yibibephnf1wwu6r/branch/master?svg=true)](https://ci.appveyor.com/project/alphaliang/pcammls/branch/master)
[![License: Unlicense](https://img.shields.io/badge/license-Unlicense-blue.svg)](http://unlicense.org/)

目前支持情况：

platform |   language      | status   
-------- | --------------- | --------
windows |python          | ok
windows |C# .net fx4.0   | ok
liunx   |python          | ok
linux   |C# mono         | 未测试
linux   | java           | TODO

采用SWIG对原厂sdk的C语言库封装，生成各语言对应接口
接口操作尽量保持与原API一致。

已测试环境：
1. win10 + python3.6 + vs2019 + swig4.0.0
2. win10 + c# .net4.0 + vs2019 + swig4.0.0
3. ubuntu16 + python2.7 + swig4.0.0
 
目前版本对应原Camport3 v3.3.1版本
## 预编译版本下载
预编译版本包含库： 
windows python3 +  c# .net4.0
linux x64 python2
https://github.com/alphaliang/pcammls/releases

## 手工编译

依赖库&工具：

http://www.swig.org/  swig

https://cmake.org/   cmake3.10+

https://github.com/percipioxyz/camport3  camport3


需先配置 cmake 变量 CAMPORT_DIR 指向 camport3 sdk所在位置

配置CAMPORT_ARCH（或留空）以设置对应lib进行跨平台编译

## PYTHON使用方法
代码片段：
```python

def main():
    TYInitLib()
    iface_id,dev_sn = select_device()
    iface_handle = TYOpenInterface(iface_id)
    dev_handle = TYOpenDevice(iface_handle,dev_sn)
    fetch_frame_loop(dev_handle)
    TYCloseDevice(dev_handle)
    TYCloseInterface(iface_handle)
    TYDeinitLib()

def fetch_frame_loop(handle):
    TYEnableComponents(handle,TY_COMPONENT_DEPTH_CAM)
    sz = TYGetFrameBufferSize(handle)
    buffs=[char_ARRAY(sz),char_ARRAY(sz)]
    TYEnqueueBuffer(handle,buffs[0],sz)
    TYEnqueueBuffer(handle,buffs[1],sz)
    TYStartCapture(handle)
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
                    depthu8 =  cv2.convertScaleAbs(arr, alpha=(255.0/4000.0))
                    cv2.imshow('depth',depthu8)
            cv2.waitKey(10)
            TYEnqueueBuffer(handle,frame.userBuffer,frame.bufferSize)
        except Exception as err:
            print (err)
    TYStopCapture(handle)
```
其他常见调用参考 test.py 和 fetch_frame.py 文件

### python与原sdk区别 
* 原返回状态值封装为python异常
* TYGetXX 类型api改用返回值获取值
* 图像可输出为numpy array

## C#使用方法
代码片段：
```CSharp
static void Main(string[] args)
{
    try    {
        SDK.TYInitLib();
        TY_VERSION_INFO info = new TY_VERSION_INFO();
        SDK.TYLibVersion(info);
        Console.WriteLine(string.Format("LIB VERSION :{0} {1} {2}", info.major, info.minor, info.patch));
        var dev_info = SimpleDeviceSelect();
        IntPtr dev_handle = new IntPtr();
        IntPtr iface_handle = new IntPtr();
        SDK.TYOpenInterface(dev_info.iface.id, ref iface_handle);
        SDK.TYOpenDevice(iface_handle, dev_info.id, ref dev_handle);
        FetchFrameLoop(dev_handle);
        SDK.TYCloseDevice(dev_handle);
        SDK.TYCloseInterface(iface_handle);
    }
    catch (System.ComponentModel.Win32Exception ex)    {
        Console.WriteLine(ex.Message);
    }
    finally    {
        SDK.TYDeinitLib();
    }
}
```

其他常见调用参考test.cs 和frame_fetch.cs文件

### csharp版本与原sdk区别
 * 原返回状态值封装为异常
 * 原指针操作转换为 c# 输出参数 



