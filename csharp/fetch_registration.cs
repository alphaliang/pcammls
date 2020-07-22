using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using pcammls;
using SDK = pcammls.pcammls;

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

        static double[,] YUV2RGB_CONVERT_MATRIX = new double[3, 3] { { 1, 0, 1.4022 }, { 1, -0.3456, -0.7145 }, { 1, 1.771, 0 } };
        static void ConvertYUYV2RGB(uint8_t_ARRAY yuvFrame, uint8_t_ARRAY rgbFrame, int width, int height)
        {
            int temp = 0;
            int widthstep = width * 2;
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x += 2)
                {
                    byte y1 = yuvFrame[y * widthstep + x * 2];
                    byte u = yuvFrame[y * widthstep + x * 2 + 1];
                    byte y2 = yuvFrame[y * widthstep + x * 2 + 2];
                    byte v = yuvFrame[y * widthstep + x * 2 + 3];
                    // R
                    temp = (int)(y1 + (v - 128) * YUV2RGB_CONVERT_MATRIX[0, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 2] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // G
                    temp = (int)(y1 + (u - 128) * YUV2RGB_CONVERT_MATRIX[1, 1] + (v - 128) * YUV2RGB_CONVERT_MATRIX[1, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 1] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // B
                    temp = (int)(y1 + (u - 128) * YUV2RGB_CONVERT_MATRIX[2, 1]);
                    rgbFrame[y * width * 3 + 3 * x] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));

                    // R
                    temp = (int)(y2 + (v - 128) * YUV2RGB_CONVERT_MATRIX[0, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 5] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // G
                    temp = (int)(y2 + (u - 128) * YUV2RGB_CONVERT_MATRIX[1, 1] + (v - 128) * YUV2RGB_CONVERT_MATRIX[1, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 4] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // B
                    temp = (int)(y2 + (u - 128) * YUV2RGB_CONVERT_MATRIX[2, 1]);
                    rgbFrame[y * width * 3 + 3 * x + 3] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                }
            }
        }

        static void ConvertYVYU2RGB(uint8_t_ARRAY yuvFrame, uint8_t_ARRAY rgbFrame, int width, int height)
        {
            int temp = 0;
            int widthstep = width * 2;
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x += 2)
                {
                    byte y1 = yuvFrame[y * widthstep + x * 2];
                    byte v  = yuvFrame[y * widthstep + x * 2 + 1];
                    byte y2 = yuvFrame[y * widthstep + x * 2 + 2];
                    byte u  = yuvFrame[y * widthstep + x * 2 + 3];
                    // R
                    temp = (int)(y1 + (v - 128) * YUV2RGB_CONVERT_MATRIX[0, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 2] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // G
                    temp = (int)(y1 + (u - 128) * YUV2RGB_CONVERT_MATRIX[1, 1] + (v - 128) * YUV2RGB_CONVERT_MATRIX[1, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 1] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // B
                    temp = (int)(y1 + (u - 128) * YUV2RGB_CONVERT_MATRIX[2, 1]);
                    rgbFrame[y * width *3 + 3 * x] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));

                    // R
                    temp = (int)(y2 + (v - 128) * YUV2RGB_CONVERT_MATRIX[0, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 5] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // G
                    temp = (int)(y2 + (u - 128) * YUV2RGB_CONVERT_MATRIX[1, 1] + (v - 128) * YUV2RGB_CONVERT_MATRIX[1, 2]);
                    rgbFrame[y * width * 3 + 3 * x + 4] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                    // B
                    temp = (int)(y2 + (u - 128) * YUV2RGB_CONVERT_MATRIX[2, 1]);
                    rgbFrame[y * width * 3 + 3 * x + 3] = (byte)(temp < 0 ? 0 : (temp > 255 ? 255 : temp));
                }
            }
        }

        static int __TYCompareFirmwareVersion(TY_DEVICE_BASE_INFO info, int major, int minor)
        {
            if (info.firmwareVersion.major < major)
                return -1;

            if ((info.firmwareVersion.major == major) &&
                (info.firmwareVersion.minor < minor))
                return -1;

            if ((info.firmwareVersion.major == major) &&
                (info.firmwareVersion.minor == minor))
                return 0;

            return 1;
        }
        static int __TYDetectOldVer21ColorCam(IntPtr handle)
        {
            TY_DEVICE_BASE_INFO info = new TY_DEVICE_BASE_INFO();
            int ret = SDK.TYGetDeviceInfo(handle, info);
            if (ret < 0) return -1;//ERROR

            if (info.iface.type == SDK.TY_INTERFACE_USB)
                return 1;

            ret = __TYCompareFirmwareVersion(info, 2, 2);
            if (((info.iface.type == SDK.TY_INTERFACE_ETHERNET) ||
                (info.iface.type == SDK.TY_INTERFACE_RAW)) &&
                (ret < 0))
                return 1;

            return 0;
        }

        static SWIGTYPE_p_unsigned_char FloatArrayToSWIGTYPE(float[] array, int len)
        {
            int size = len * sizeof(float);

            IntPtr structPtr = Marshal.AllocHGlobal(size);
            Marshal.Copy(array, 0, structPtr, len);

            byte[] bytes = new byte[size];
            Marshal.Copy(structPtr, bytes, 0, size);
            Marshal.FreeHGlobal(structPtr);

            uint8_t_ARRAY shading = new uint8_t_ARRAY(size);
            for (int i = 0; i < size; i++) shading[i] = bytes[i];
            SWIGTYPE_p_unsigned_char pointer = shading.cast();

            return pointer;
        }

        static SWIGTYPE_p_unsigned_char IntArrayToSWIGTYPE(int[] array, int len)
        {
            int size = len * sizeof(int);

            IntPtr structPtr = Marshal.AllocHGlobal(size);
            Marshal.Copy(array, 0, structPtr, len);

            byte[] bytes = new byte[size];
            Marshal.Copy(structPtr, bytes, 0, size);
            Marshal.FreeHGlobal(structPtr);

            uint8_t_ARRAY _arr = new uint8_t_ARRAY(size);
            for (int i = 0; i < size; i++) _arr[i] = bytes[i];
            SWIGTYPE_p_unsigned_char pointer = _arr.cast();
            return pointer;
        }

        static SWIGTYPE_p_unsigned_char IntPtrToSWIGTYPE(IntPtr p)
        {
            int size = sizeof(int);
            int[] m_handle = new int[1];
            m_handle[0] = (int)p;

            IntPtr structPtr = Marshal.AllocHGlobal(size);
            Marshal.Copy(m_handle, 0, structPtr, 1);

            byte[] bytes = new byte[size];
            Marshal.Copy(structPtr, bytes, 0, size);
            Marshal.FreeHGlobal(structPtr);

            uint8_t_ARRAY _handle = new uint8_t_ARRAY(size);
            for (int i = 0; i < size; i++) _handle[i] = bytes[i];
            SWIGTYPE_p_unsigned_char pointer = _handle.cast();
            return pointer;
        }

        static void ColorIspInitSetting(IntPtr isp_handle, IntPtr handle)
        {
            int is_v21_color_device = __TYDetectOldVer21ColorCam(handle);
            if (is_v21_color_device < 0) {
                Console.WriteLine("__TYDetectOldVer21ColorCam error.");
                return;
            }

            if (is_v21_color_device > 0)
            {
                SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_BLACK_LEVEL, 11);
                SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_BLACK_LEVEL_GAIN, 256.0f / (256 - 11));//float
            }
            else {
                SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_BLACK_LEVEL, 0);
                SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_BLACK_LEVEL_GAIN, 1.0f);//float

                bool b;
                SDK.TYHasFeature(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_INT_ANALOG_GAIN, out b);
                if (b) {
                    SDK.TYSetInt(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_INT_ANALOG_GAIN, 1);
                }
            }
            
            SWIGTYPE_p_unsigned_char pointer;

            float []fShading = new float[9];
            fShading[0] = 0.30890417098999026f;  fShading[1] = 10.63355541229248f; fShading[2] = -6.433426856994629f;
            fShading[3] = 0.24413758516311646f;  fShading[4] = 11.739893913269043f;fShading[5] =  -8.148622512817383f;
            fShading[6] = 0.1255662441253662f;   fShading[7] = 11.88359546661377f; fShading[8] = -7.865192413330078f;
            pointer = FloatArrayToSWIGTYPE(fShading, 9);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_SHADING, pointer, 9 * sizeof(float));

            int[] m_shading_center = new int[2];
            m_shading_center[0] = 640;
            m_shading_center[1] = 480;
            pointer = IntArrayToSWIGTYPE(m_shading_center, 2);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_SHADING_CENTER, pointer, 2 * sizeof(int));
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_CCM_ENABLE, 0);

            pointer = IntPtrToSWIGTYPE(handle);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_CAM_DEV_HANDLE, pointer, sizeof(int));

            Int32 t = (int)SDK.TY_COMPONENT_RGB_CAM;
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_CAM_DEV_COMPONENT, t);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_GAMMA, 1.0f);       //float
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_AUTOBRIGHT, 1);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_ENABLE_AUTO_EXPOSURE_GAIN, 0);

            int current_image_width = 1280;
            SDK.TYGetInt(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_INT_WIDTH, out current_image_width);

            int []image_size = new int[2];
            image_size[0] = 1280;
            image_size[1] = 960;// image size for current parameters
            pointer = IntArrayToSWIGTYPE(image_size, 2);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_IMAGE_SIZE, pointer, 2 * sizeof(int));
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_INPUT_RESAMPLE_SCALE, image_size[0] / current_image_width);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_ENABLE_AUTO_WHITEBALANCE, 1); //eanble auto white balance

            uint comp_all;
            SDK.TYGetComponentIDs(handle, out comp_all);
            if (0 != (comp_all & (int)SDK.TY_COMPONENT_STORAGE))
                return;

            bool has_isp_block = false;
            SDK.TYHasFeature(handle, SDK.TY_COMPONENT_STORAGE, SDK.TY_BYTEARRAY_ISP_BLOCK, out has_isp_block);
            if (!has_isp_block)
                return;

            UInt32 sz = 0;
            SDK.TYGetByteArraySize(handle, SDK.TY_COMPONENT_STORAGE, SDK.TY_BYTEARRAY_ISP_BLOCK, out sz);
            if (sz <= 0)
                return;

            uint8_t_ARRAY buff = new uint8_t_ARRAY((int)sz);
            SDK.TYGetByteArray(handle, SDK.TY_COMPONENT_STORAGE, SDK.TY_BYTEARRAY_ISP_BLOCK, buff.cast(), sz);
            SDK.TYISPLoadConfig(isp_handle, buff.cast(), sz);
        }
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
            ColorIspInitSetting(color_isp_handle, handle);

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
                    var images = frame.image;
                    for(int idx = 0; idx < frame.validCount; idx++)
                    {
                        var img = images[idx];
                        if(img.componentID == SDK.TY_COMPONENT_DEPTH_CAM)
                        {
                            depth_pixel_arr = uint16_t_ARRAY.FromVoidPtr(img.buffer);
                            depth_enable = true;
                        }
                        else if (img.componentID == SDK.TY_COMPONENT_RGB_CAM)
                        {
                            if(img.pixelFormat == SDK.TY_PIXEL_FORMAT_YVYU)
                            {
                                var pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer);

                                ConvertYVYU2RGB(pixel_arr, color_data, img.width, img.height);
                                color_enable = true;
                            }
                            else if (img.pixelFormat == SDK.TY_PIXEL_FORMAT_YUYV)
                            {
                                var pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer);

                                ConvertYUYV2RGB(pixel_arr, color_data, img.width, img.height);
                                color_enable = true;
                            }
                            else if (img.pixelFormat == SDK.TY_PIXEL_FORMAT_BAYER8GB)
                            {
                                var pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer);

                                SWIGTYPE_p_void pointer = (SWIGTYPE_p_void)color_data.VoidPtr();
                                TY_IMAGE_DATA out_buff = SDK.TYInitImageData((uint)color_size, pointer, (uint)(img.width), (uint)(img.height));
                                out_buff.pixelFormat = (int)SDK.TY_PIXEL_FORMAT_BGR;

                                SDK.TYISPProcessImage(color_isp_handle, img, out_buff);
                                SDK.TYISPUpdateDevice(color_isp_handle);

                                var color_pixel_arr = uint8_t_ARRAY.FromVoidPtr(out_buff.buffer);
                                color_enable = true;
                            }
                            else {
                                Console.WriteLine(string.Format("Color Image Type:{0}", img.pixelFormat));
                            }
                        }
                    }

                    if ((depth_enable) && (color_enable))
                    {
                        SDK.TYUndistortImage(color_calib, src, color_calib.intrinsic, dst);
                        int offset = color_width * color_height / 2 + color_width / 2;
                        byte b = undistort_color_data[3 * offset];
                        byte g = undistort_color_data[3 * offset + 1];
                        byte r = undistort_color_data[3 * offset + 2];
                        
                        SDK.TYMapDepthImageToColorCoordinate(depth_calib, (uint)depth_width, (uint)depth_height, depth_pixel_arr.cast(), color_calib,
                            (uint)color_width, (uint)color_height, registration_depth_data.cast());
                        ushort distance = registration_depth_data[offset];

                        Console.WriteLine(string.Format("The rgbd value of the center position of the image :R.{0} G.{1} B.{2} D.{3}", r, g, b, distance));
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
                SDK.TYOpenDevice(iface_handle, dev_info.id, ref dev_handle);

                FetchFrameLoop(dev_handle);
                SDK.TYCloseDevice(dev_handle);
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
