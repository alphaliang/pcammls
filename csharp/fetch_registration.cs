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
        static uint8_t_ARRAY undistort_color_data;
        static uint16_t_ARRAY registration_depth_data;

        static TY_CAMERA_CALIB_INFO depth_calib = new TY_CAMERA_CALIB_INFO();
        static TY_CAMERA_CALIB_INFO color_calib = new TY_CAMERA_CALIB_INFO();

        static TY_IMAGE_DATA src = new TY_IMAGE_DATA();
        static TY_IMAGE_DATA dst = new TY_IMAGE_DATA();

        
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
            uint cal_size1 = depth_calib.CSize();
            SDK.TYGetStruct(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_STRUCT_CAM_CALIB_DATA, depth_calib.getCPtr(), cal_size1);

            uint cal_size2 = color_calib.CSize();
            SDK.TYGetStruct(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_STRUCT_CAM_CALIB_DATA, color_calib.getCPtr(), cal_size2);
            
            IntPtr color_isp_handle = new IntPtr();

            SDK.TYEnableComponents(handle,SDK.TY_COMPONENT_DEPTH_CAM);
            SDK.TYEnableComponents(handle, SDK.TY_COMPONENT_RGB_CAM);

            SDK.TYISPCreate(ref color_isp_handle);
            SDK.ColorIspInitSetting(color_isp_handle, handle);

            uint buff_sz;
            SDK.TYGetFrameBufferSize(handle, out buff_sz);

            int depth_width, depth_height;
            SDK.TYGetInt(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_INT_WIDTH, out depth_width);
            SDK.TYGetInt(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_INT_HEIGHT, out depth_height);
            Console.WriteLine(string.Format("Depth Image size:{0} {1}", depth_width, depth_height));

            int color_width, color_height;
            SDK.TYGetInt(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_INT_WIDTH, out color_width);
            SDK.TYGetInt(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_INT_HEIGHT, out color_height);
            Console.WriteLine(string.Format("RGB Image size:{0} {1}", color_width, color_height));

            int color_size = color_width * color_height * 3;
            
            buffer[0] = new uint8_t_ARRAY((int)buff_sz);
            buffer[1] = new uint8_t_ARRAY((int)buff_sz);
            
            color_data = new uint8_t_ARRAY(color_size);
            undistort_color_data = new uint8_t_ARRAY(color_size);
            registration_depth_data = new uint16_t_ARRAY(color_size);
            
            src.width = color_width;
            src.height = color_height;
            src.size = 3 * color_width * color_height;
            src.pixelFormat = (int)SDK.TY_PIXEL_FORMAT_RGB;
            src.buffer = color_data.VoidPtr();

            dst.width = color_width;
            dst.height = color_height;
            dst.size = 3 * color_width * color_height;
            dst.pixelFormat = (int)SDK.TY_PIXEL_FORMAT_RGB;
            dst.buffer = undistort_color_data.VoidPtr();
            
            SDK.TYEnqueueBuffer(handle, buffer[0].VoidPtr(), buff_sz);
            SDK.TYEnqueueBuffer(handle, buffer[1].VoidPtr(), buff_sz);

            float f_depth_unit = 1.0f;
            SDK.TYGetFloat(handle, SDK.TY_COMPONENT_DEPTH_CAM, SDK.TY_FLOAT_SCALE_UNIT, out f_depth_unit);
            Console.WriteLine(string.Format("##########f_depth_unit =  {0}", f_depth_unit));

            SDK.TYStartCapture(handle);
            int img_index = 0;

            while (true)
            {
                TY_FRAME_DATA frame = new TY_FRAME_DATA();
                try
                {
                    SDK.TYFetchFrame(handle, frame, -1);
                    Console.WriteLine(string.Format("capture {0} ", img_index));

                    bool depth_enable = false;
                    bool color_enable = false;

                    uint16_t_ARRAY depth_pixel_arr = null;
                    uint8_t_ARRAY pixel_arr = null;
                    uint8_t_ARRAY color_pixel_arr = null;
                    var images = frame.image;
                    for(int idx = 0; idx < frame.validCount; idx++)
                    {
                        var img = images[idx];
                        if(img.componentID == SDK.TY_COMPONENT_DEPTH_CAM)
                        {
                            depth_pixel_arr = uint16_t_ARRAY.FromVoidPtr(img.buffer, img.width*img.height);
                            depth_enable = true;
                        }
                        else if (img.componentID == SDK.TY_COMPONENT_RGB_CAM)
                        {
                            if(img.pixelFormat == SDK.TY_PIXEL_FORMAT_YVYU)
                            {
                                pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer,img.size);

                                SDK_ISP.ConvertYVYU2RGB(pixel_arr, color_data, img.width, img.height);
                                color_enable = true;
                            }
                            else if (img.pixelFormat == SDK.TY_PIXEL_FORMAT_YUYV)
                            {
                                pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer,img.size);

                                SDK_ISP.ConvertYUYV2RGB(pixel_arr, color_data, img.width, img.height);
                                color_enable = true;
                            }
                            else if ((img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8GB) ||
                                    (img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8BG) ||
                                    (img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8GR) ||
                                    (img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8RG))
                            {
                                pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer,img.size);

                                SWIGTYPE_p_void pointer = (SWIGTYPE_p_void)color_data.VoidPtr();
                                TY_IMAGE_DATA out_buff = SDK.TYInitImageData((uint)color_size, pointer, (uint)(img.width), (uint)(img.height));
                                out_buff.pixelFormat = (int)SDK.TY_PIXEL_FORMAT_BGR;

                                SDK.TYISPProcessImage(color_isp_handle, img, out_buff);
                                SDK.TYISPUpdateDevice(color_isp_handle);

                                //get rgb image data
                                color_pixel_arr = uint8_t_ARRAY.FromVoidPtr(out_buff.buffer, img.width*img.height*3);
                                color_enable = true;
                            }
                            else {
                                Console.WriteLine(string.Format("Color Image Type:{0}", img.pixelFormat));
                            }
                        }
                    }

                    if ((depth_enable) && (color_enable))
                    {
                        TY_CAMERA_INTRINSIC color_intrinsic = new TY_CAMERA_INTRINSIC();
                        color_intrinsic.resize(color_calib.intrinsic, 1.0f * color_width / color_calib.intrinsicWidth, 1.0f * color_height / color_calib.intrinsicHeight);
                        SDK.TYUndistortImage(color_calib, src, color_intrinsic, dst);
                        int offset = color_width * color_height / 2 + color_width / 2;
                        byte b = undistort_color_data[3 * offset];
                        byte g = undistort_color_data[3 * offset + 1];
                        byte r = undistort_color_data[3 * offset + 2];

                        SDK.TYMapDepthImageToColorCoordinate(depth_calib, (uint)depth_width, (uint)depth_height, depth_pixel_arr.cast(), color_calib,
                            (uint)color_width, (uint)color_height, registration_depth_data.cast(), f_depth_unit);

                        //The depth image after registration needs to be median filtered to fill the invalid hole.
                        //TODO...

                        ushort distance = registration_depth_data[offset];

                        Console.WriteLine(string.Format("The rgbd value of the center position of the image :R.{0} G.{1} B.{2} D.{3}", r, g, b, distance));
                    }

                    if(depth_pixel_arr != null)
                        uint16_t_ARRAY.ReleasePtr(depth_pixel_arr);

                    if (pixel_arr != null)
                        uint8_t_ARRAY.ReleasePtr(pixel_arr);

                    if(color_pixel_arr != null)
                        uint8_t_ARRAY.ReleasePtr(color_pixel_arr);

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

                IntPtr errCode = IntPtr.Zero;
                var status = SDK.TYOpenDevice(iface_handle, dev_info.id, ref dev_handle,  ref errCode);
                if (status != SDK.TY_STATUS_OK)
                {
                    Console.WriteLine(string.Format(".TYOpenDevice ret :{0}", status));
                    return;
                }
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
