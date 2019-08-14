Percipio depth CAMera  MultiLanguage Sdk
=======================================
ͼ�������������Խӿڷ�װ��
Ŀǰ֧�������

platform |   language      | status   
-------- | --------------- | --------
windows |python          | ok
windows |C# .net fx4.0   | ok
liunx   |python          | ok
linux   |C# mono         | δ����
linux   | java           | TODO

����SWIG��ԭ��sdk��C���Կ��װ�����ɸ����Զ�Ӧ�ӿ�
�ӿڲ�������������ԭAPIһ�¡�

�Ѳ��Ի�����
1. win10 + python3.6 + vs2019
2. win10 + c# .net4.0 + vs2019
3. ubuntu16 + python2.7
 
Ŀǰ�汾��ӦԭCamport3 v3.3.1�汾
## Ԥ����汾����
Ԥ����汾�����⣺ 
windows python3 +  c# .net4.0
linux x64 python2
https://github.com/alphaliang/pcammls/releases

## �ֹ�����
������&���ߣ� 
http://www.swig.org/  swig
https://cmake.org/   cmake3.10+
https://github.com/percipioxyz/camport3  camport3

�������� cmake ���� CAMPORT_DIR ָ�� camport3 sdk����λ��
����CAMPORT_ARCH�������գ������ö�Ӧlib���п�ƽ̨����

## PYTHONʹ�÷���
����Ƭ�Σ�
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
�����������òο� test.py �� fetch_frame.py �ļ�

### python��ԭsdk���� 
* ԭ����״ֵ̬��װΪpython�쳣
* TYGetXX ����api���÷���ֵ��ȡֵ
* ͼ������Ϊnumpy array

## C#ʹ�÷���
����Ƭ�Σ�
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

�����������òο�test.cs ��frame_fetch.cs�ļ�

### csharp�汾��ԭsdk����
 * ԭ����״ֵ̬��װΪ�쳣
 * ԭָ�����ת��Ϊ c# ������� 



