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

采用SWIG对原厂sdk的C语言库封装，生成各语言对应接口
接口操作尽量保持与原API一致。
另新增PercipioSDK 类封装接口，简化设备操作。

已测试环境：
1. win10 + python3.6 + vs2019 + swig4.0.0
2. win10 + c# .net4.0 + vs2019 + swig4.0.0
3. ubuntu16 + python2.7 + swig4.0.0
 
目前版本对应原Camport3 v3.6.33版本
## 预编译版本下载
预编译版本包含库： 
- windows：python3 +  c# .net4.0
- linux：x64 python2

https://github.com/alphaliang/pcammls/releases

## 手工编译

依赖库&工具：

http://www.swig.org/  swig4.0.0

https://cmake.org/   cmake3.15+


配置CAMPORT\_ARCH（留空为平台默认值）以设置对应lib进行跨平台编译。
ARCH名称参考camport3库/lib/路径中对应名称（无lib前缀）

## PYTHON使用方法
代码片段：
```python

class PythonPercipioDeviceEvent(pcammls.DeviceEvent):
    Offline = False

    def __init__(self):
        pcammls.DeviceEvent.__init__(self)

    def run(self, handle, eventID):
        if eventID==TY_EVENT_DEVICE_OFFLINE:
          print('=== Event Callback: Device Offline!')
          self.Offline = True
        return 0

    def IsOffline(self):
        return self.Offline

def main():
    cl = PercipioSDK()
    dev_list = cl.ListDevice()
    for idx in range(len(dev_list)):
      dev = dev_list[idx]
      print ('{} -- {} \t {}'.format(idx,dev.id,dev.iface.id))
    if  len(dev_list)==0:
      print ('no device')
      return
    if len(dev_list) == 1:
        selected_idx = 0 
    else:
        selected_idx  = int(input('select a device:'))
    if selected_idx < 0 or selected_idx >= len(dev_list):
        return

    sn = dev_list[selected_idx].id

    handle = cl.Open(sn)
    if not cl.isValidHandle(handle):
      print('no device found')
      return
      
    event = PythonPercipioDeviceEvent()
    cl.DeviceRegiststerCallBackEvent(event)

    cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH)
    depth_render = image_data()
    cl.DeviceStreamOn(handle)

    while True:
      if event.IsOffline():
        break
      image_list = cl.DeviceStreamRead(handle, -1)
      for i in range(len(image_list)):
        frame = image_list[i]
        if frame.streamID == PERCIPIO_STREAM_DEPTH:
          cl.DeviceStreamDepthRender(frame, depth_render)
          arr = depth_render.as_nparray()
          cv2.imshow('depth',arr)
      k = cv2.waitKey(10)
      if k==ord('q'): 
        break

    cl.DeviceStreamOff(handle)    
    cl.Close(handle)
    pass

```
其他常见调用参考 python 路径下示例文件。
扩展功能或者修改调用可修改swig路径下py\_extend.i文件

### python与原sdk区别 
* 原返回状态值封装为python异常
* TYGetXX 类型api改用返回值获取值
* 图像buffer可输出为numpy array , numpy array也可转回c buffer

## C#使用方法
代码片段：
```CSharp
static void Main(string[] args)
        {
            CSharpPercipioDeviceEvent _event;
            PercipioSDK cl;
            DeviceInfoVector dev_list;
            System.IntPtr handle;

            cl = new PercipioSDK();
            dev_list = cl.ListDevice();
            int sz = dev_list.Count();
            if (sz == 0) {
                Console.WriteLine(string.Format("no device found."));
                return ;
            }

            Console.WriteLine(string.Format("found follow devices:"));
            for (int idx = 0; idx < sz; idx++) {
                var item = dev_list[idx];
                Console.WriteLine("{0} -- {1} {2}", idx, item.id, item.modelName);
            }
            Console.WriteLine("select one:");
            int select = int.Parse(Console.ReadLine());

            handle = cl.Open(dev_list[select].id);
            if (!cl.isValidHandle(handle)) {
                Console.WriteLine(string.Format("can not open device!"));
                return;
            }

            _event = new CSharpPercipioDeviceEvent();
            cl.DeviceRegiststerCallBackEvent(_event);
            cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR | PERCIPIO_STREAM_DEPTH);

            cl.DeviceStreamOn(handle);

            while (true) {
                if (_event.isOffLine())
                    break;
                FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                if(frames.Count() > 0)
                    Console.WriteLine(string.Format("fetch frames ok!"));
            }

            cl.DeviceStreamOff(handle);
            cl.Close(handle);
        }
```

其他常见调用参考csharp 路径下文件.
扩展功能或者修改调用可修改swig路径下csharp\_extend.i文件

编译安装详细指南参考官网在线文档:
https://percipiodc.readthedocs.io/en/latest/getstarted/sdk-compile.html#pythoncsharp



