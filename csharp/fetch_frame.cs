using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using pcammls;
using SDK = pcammls.pcammls;
using pcammls_isp;
using SDK_ISP = pcammls_isp.pcammls_isp_api;
namespace pcammls_fetch_frame
{

    class Program
    {
        static uint8_t_ARRAY[] buffer = new uint8_t_ARRAY[2];
        static uint8_t_ARRAY color_data;

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
            IntPtr color_isp_handle = new IntPtr();

            uint cal_size = calib_inf.CSize();
            SDK.TYGetStruct(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_STRUCT_CAM_CALIB_DATA, calib_inf.getCPtr(), cal_size);
            Console.WriteLine(string.Format("Depth calib inf width:{0} height:{1}", calib_inf.intrinsicWidth, calib_inf.intrinsicHeight));
            Console.WriteLine(string.Format("Depth intrinsic:{0} {1} {2} {3} {4} {5} {6} {7} {8}",
                calib_inf.intrinsic.data[0], calib_inf.intrinsic.data[1], calib_inf.intrinsic.data[2],
                calib_inf.intrinsic.data[3], calib_inf.intrinsic.data[4], calib_inf.intrinsic.data[5],
                calib_inf.intrinsic.data[6], calib_inf.intrinsic.data[7], calib_inf.intrinsic.data[8]));

            
            SDK.TYEnableComponents(handle,SDK.TY_COMPONENT_DEPTH_CAM);
            SDK.TYEnableComponents(handle, SDK.TY_COMPONENT_RGB_CAM);
            //set depth cam resolution
            SDK.TYSetEnum(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_ENUM_IMAGE_MODE, (int)(SDK.TY_RESOLUTION_MODE_640x480 | SDK.TY_PIXEL_FORMAT_DEPTH16));

            SDK.TYISPCreate(ref color_isp_handle);
            SDK.ColorIspInitSetting(color_isp_handle, handle);
            //SDK_ISP.ColorIspInitSetting(color_isp_handle, handle);

            uint buff_sz;
            SDK.TYGetFrameBufferSize(handle, out buff_sz);

            int width, height;
            SDK.TYGetInt(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_INT_WIDTH, out width);
            SDK.TYGetInt(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_INT_HEIGHT, out height);
            Console.WriteLine(string.Format("RGB Image size:{0} {1}", width, height));

            int color_size = width * height * 3;
            
            buffer[0] = new uint8_t_ARRAY((int)buff_sz);
            buffer[1] = new uint8_t_ARRAY((int)buff_sz);

            color_data = new uint8_t_ARRAY(color_size);
            SDK.TYEnqueueBuffer(handle, buffer[0].VoidPtr(), buff_sz);
            SDK.TYEnqueueBuffer(handle, buffer[1].VoidPtr(), buff_sz);

            float f_depth_unit = 1.0f;
            SDK.TYGetFloat(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_FLOAT_SCALE_UNIT, out f_depth_unit);
            Console.WriteLine(string.Format("##########f_depth_unit =  {0}", f_depth_unit));
                         
            SDK.TYStartCapture(handle);
            int img_index = 0;

            TY_PIXEL_DESC pix = new TY_PIXEL_DESC();
            TY_VECT_3F p3d = new TY_VECT_3F();
            
            while (true)
            {
                TY_FRAME_DATA frame = new TY_FRAME_DATA();
                try
                {
                    SDK.TYFetchFrame(handle, frame, -1);
                    Console.WriteLine(string.Format("capture {0} ", img_index));
                    
                    var images = frame.image;
                    for(int idx = 0; idx < frame.validCount; idx++)
                    {
                        var img = images[idx];
                        if(img.componentID == SDK.TY_COMPONENT_DEPTH_CAM)
                        {
                            var pixel_arr = uint16_t_ARRAY.FromVoidPtr(img.buffer, img.size / 2);
                            IntPtr pt = pixel_arr.VoidPtr2();

                            int offset = img.width * img.height / 2 + img.width / 2;
                            ushort distance =  pixel_arr[offset];

                            pix.x = (short)(img.width / 2);
                            pix.y = (short)(img.height / 2);
                            pix.depth = distance;

                            SDK.TYMapDepthToPoint3d(calib_inf, (uint)img.width, (uint)img.height, pix, 1, p3d, f_depth_unit);

                            float f_distance = distance * f_depth_unit;
                            Console.WriteLine(string.Format("Depth Image Center Pixel Distance:{0}", f_distance));
                            Console.WriteLine(string.Format("Point Cloud Center Data:(x:{0} y:{1} z:{2})", p3d.x, p3d.y, p3d.z));
                            
                            uint16_t_ARRAY.ReleasePtr(pixel_arr);
                        }
                        else if (img.componentID == SDK.TY_COMPONENT_RGB_CAM)
                        {
                            var pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer, img.size);
                            if (img.pixelFormat == SDK.TY_PIXEL_FORMAT_YVYU)
                            {
                                SDK_ISP.ConvertYVYU2RGB(pixel_arr, color_data, img.width, img.height);

                                int offset = 3 * (img.width * img.height / 2 + img.width / 2);
                                byte b = color_data[offset];
                                byte g = color_data[offset + 1];
                                byte r = color_data[offset + 2];
                                Console.WriteLine(string.Format("Color Image Center Pixel value(YVYU):{0} {1} {2}", r, g, b));
                            }
                            else if (img.pixelFormat == SDK.TY_PIXEL_FORMAT_YUYV)
                            {
                                SDK_ISP.ConvertYUYV2RGB(pixel_arr, color_data, img.width, img.height);

                                int offset = 3 * (img.width * img.height / 2 + img.width / 2);
                                byte b = color_data[offset];
                                byte g = color_data[offset + 1];
                                byte r = color_data[offset + 2];
                                Console.WriteLine(string.Format("Color Image Center Pixel value(YUYV):{0} {1} {2}", r, g, b));
                            }
                            else if ((img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8GB) ||
                                    (img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8BG) ||
                                    (img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8GR) ||
                                    (img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8RG))
                            {
                                SWIGTYPE_p_void pointer = (SWIGTYPE_p_void)color_data.VoidPtr();

                                TY_IMAGE_DATA out_buff = SDK.TYInitImageData((uint)color_size, pointer, (uint)(img.width), (uint)(img.height));
                                out_buff.pixelFormat = (int)SDK.TY_PIXEL_FORMAT_BGR;

                                int ret = SDK.TYISPProcessImage(color_isp_handle, img, out_buff);
                                SDK.TYISPUpdateDevice(color_isp_handle);

                                var color_pixel_arr = uint8_t_ARRAY.FromVoidPtr(out_buff.buffer, img.size * 3);

                                int offset = 3 * (img.width * img.height / 2 + img.width / 2);
                                byte b = color_pixel_arr[offset];
                                byte g = color_pixel_arr[offset + 1];
                                byte r = color_pixel_arr[offset + 2];
                                Console.WriteLine(string.Format("Color Image Center Pixel value(Bayer):{0} {1} {2}", r, g, b));

                                uint8_t_ARRAY.ReleasePtr(color_pixel_arr);
                            }
                            else {
                                Console.WriteLine(string.Format("Color Image Type:{0}", img.pixelFormat));
                            }

                            uint8_t_ARRAY.ReleasePtr(pixel_arr);
                        }
                    }
                    SDK.TYEnqueueBuffer(handle,frame.userBuffer, (uint)frame.bufferSize);
                    img_index++;
                }
                catch(System.ComponentModel.Win32Exception ex)
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

                IntPtr errCode = IntPtr.Zero;//0;// new IntPtr();
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
