using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using pcammls;
using SDK = pcammls.pcammls;
namespace pcammls_fetch_point3d
{

    class Program
    {
        static uint8_t_ARRAY[] buffer = new uint8_t_ARRAY[2];

        static TY_CAMERA_CALIB_INFO calib_inf = new TY_CAMERA_CALIB_INFO();

        static TY_DEVICE_BASE_INFO SimpleDeviceSelect()
        {
            DeviceInfoVector devs = new DeviceInfoVector();
            SDK.selectDevice(SDK.TY_INTERFACE_ALL, "", "", 10, devs);
            int sz = devs.Count();
            if (sz == 0)
            {
                return null;
            }
            Console.WriteLine("found follow devices:");
            for (int idx = 0; idx < sz; idx++)
            {
                var item = devs[idx];
                Console.WriteLine("{0} -- {1} {2}", idx, item.id, item.modelName);
            }
            Console.WriteLine("select one:");
            int selected_idx = int.Parse(Console.ReadLine());
            return devs[selected_idx];
        }

        static void FetchFrameLoop(IntPtr handle)
        {
            uint cal_size = calib_inf.CSize();
            SDK.TYGetStruct(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_STRUCT_CAM_CALIB_DATA, calib_inf.getCPtr(), cal_size);
            Console.WriteLine(string.Format("Depth calib inf width:{0} height:{1}", calib_inf.intrinsicWidth, calib_inf.intrinsicHeight));
            Console.WriteLine(string.Format("Depth intrinsic:{0} {1} {2} {3} {4} {5} {6} {7} {8}",
                calib_inf.intrinsic.data[0], calib_inf.intrinsic.data[1], calib_inf.intrinsic.data[2],
                calib_inf.intrinsic.data[3], calib_inf.intrinsic.data[4], calib_inf.intrinsic.data[5],
                calib_inf.intrinsic.data[6], calib_inf.intrinsic.data[7], calib_inf.intrinsic.data[8]));

            SDK.TYEnableComponents(handle, SDK.TY_COMPONENT_DEPTH_CAM);

            //set depth cam resolution
            SDK.TYSetEnum(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_ENUM_IMAGE_MODE, (int)(SDK.TY_RESOLUTION_MODE_640x480 | SDK.TY_PIXEL_FORMAT_DEPTH16));

            uint buff_sz;
            SDK.TYGetFrameBufferSize(handle, out buff_sz);

            buffer[0] = new uint8_t_ARRAY((int)buff_sz);
            buffer[1] = new uint8_t_ARRAY((int)buff_sz);

            SDK.TYEnqueueBuffer(handle, buffer[0].VoidPtr(), buff_sz);
            SDK.TYEnqueueBuffer(handle, buffer[1].VoidPtr(), buff_sz);

            TY_PIXEL_DESC_ARRAY pixArray = new TY_PIXEL_DESC_ARRAY(640*480);
            TY_VECT_3F_ARRAY p3dArray = new TY_VECT_3F_ARRAY(640 * 480);
            TY_PIXEL_DESC temp = new TY_PIXEL_DESC();

            //trigger mode
            TY_TRIGGER_PARAM param = new TY_TRIGGER_PARAM();
            param.mode = SDK.TY_TRIGGER_MODE_OFF;
            SDK.TYSetStruct(handle, SDK.TY_COMPONENT_DEVICE, SDK.TY_STRUCT_TRIGGER_PARAM, param.getCPtr(), param.CSize());

            SDK.TYStartCapture(handle);
            int img_index = 0;

            while (true)
            {
                TY_FRAME_DATA frame = new TY_FRAME_DATA();
                try
                {
                    //send soft trigger sig
                    //SDK.TYSendSoftTrigger(handle);
                    SDK.TYFetchFrame(handle, frame, 20000);
                    Console.WriteLine(string.Format("capture {0} ", img_index));

                    var images = frame.image;
                    for (int idx = 0; idx < frame.validCount; idx++)
                    {
                        var img = images[idx];
                        if (img.componentID == SDK.TY_COMPONENT_DEPTH_CAM)
                        {
                            var pixel_arr = uint16_t_ARRAY.FromVoidPtr(img.buffer,img.width*img.height);
                            float f_depth_unit = 1.0f;
                            SDK.TYMapDepthImageToPoint3d(calib_inf, img.width, img.height, pixel_arr.cast(), p3dArray.cast(), f_depth_unit);
                            uint16_t_ARRAY.ReleasePtr(pixel_arr);

                            IntPtr ptP3D = p3dArray.VoidPtr2();

                            int offset = img.width * img.height / 2 + img.width / 2;
                            float p3d_fx = p3dArray.getitem(offset).x;
                            float p3d_fy = p3dArray.getitem(offset).y;
                            float p3d_fz = p3dArray.getitem(offset).z;
                            Console.WriteLine(string.Format("Point Cloud Center Data:(x:{0} y:{1} z:{2})", p3d_fx, p3d_fy, p3d_fz));
                        }
                    }

                    SDK.TYEnqueueBuffer(handle, frame.userBuffer, (uint)frame.bufferSize);
                    img_index++;
                }
                catch (System.ComponentModel.Win32Exception ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            }
        }

        static void Main(string[] args)
        {
            Console.WriteLine("test start\n");
            try
            {
                SDK.TYInitLib();
                TY_VERSION_INFO info = new TY_VERSION_INFO();
                SDK.TYLibVersion(info);
                Console.WriteLine(string.Format("LIB VERSION :{0} {1} {2}", info.major, info.minor, info.patch));
                var dev_info = SimpleDeviceSelect();
                if (dev_info == null)
                {
                    return;
                }
                IntPtr dev_handle = new IntPtr();
                IntPtr iface_handle = new IntPtr();
                SDK.TYOpenInterface(dev_info.iface.id, ref iface_handle);
                
                IntPtr errCode = IntPtr.Zero;
                SDK.TYOpenDevice(iface_handle, dev_info.id, ref dev_handle, ref errCode);

                FetchFrameLoop(dev_handle);
                SDK.TYCloseDevice(dev_handle, false);
                SDK.TYCloseInterface(iface_handle);
            }
            catch (System.ComponentModel.Win32Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
            finally
            {
                SDK.TYDeinitLib();
            }
            Console.WriteLine("done");
            Console.ReadKey();
        }
    }
}
