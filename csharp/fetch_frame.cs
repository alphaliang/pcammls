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

        static TY_CAMERA_CALIB_INFO calib_inf = new TY_CAMERA_CALIB_INFO();
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
            bool version = Environment.Is64BitProcess ? true : false;
            if(version)
                m_handle[0] = (int)p.ToInt64();
            else
                m_handle[0] = (int)p.ToInt32();

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
            ColorIspInitSetting(color_isp_handle, handle);

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

            SDK.TYStartCapture(handle);
            int img_index = 0;

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
                            var pixel_arr = uint16_t_ARRAY.FromVoidPtr(img.buffer);
                            int offset = img.width * img.height / 2 + img.width / 2;
                            ushort distance =  pixel_arr[offset];

                            TY_PIXEL_DESC pix = new TY_PIXEL_DESC();
                            TY_VECT_3F p3d = new TY_VECT_3F();

                            pix.x = (short)(img.width / 2);
                            pix.y = (short)(img.height / 2);
                            pix.depth = distance;

                            SDK.TYMapDepthToPoint3d(calib_inf, (uint)img.width, (uint)img.height, pix, 1, p3d);
                            Console.WriteLine(string.Format("Depth Image Center Pixel Distance:{0}", distance));
                            Console.WriteLine(string.Format("Point Cloud Center Data:(x:{0} y:{1} z:{2})", p3d.x, p3d.y, p3d.z));
                        }
                        else if (img.componentID == SDK.TY_COMPONENT_RGB_CAM)
                        {
                            if(img.pixelFormat == SDK.TY_PIXEL_FORMAT_YVYU)
                            {
                                var pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer);

                                ConvertYVYU2RGB(pixel_arr, color_data, img.width, img.height);

                                int offset = 3 * (img.width * img.height / 2 + img.width / 2);
                                byte b = color_data[offset];
                                byte g = color_data[offset + 1];
                                byte r = color_data[offset + 2];
                                Console.WriteLine(string.Format("Color Image Center Pixel value(YVYU):{0} {1} {2}", r, g, b));
                            }
                            else if (img.pixelFormat == SDK.TY_PIXEL_FORMAT_YUYV)
                            {
                                var pixel_arr = uint8_t_ARRAY.FromVoidPtr(img.buffer);

                                ConvertYUYV2RGB(pixel_arr, color_data, img.width, img.height);

                                int offset = 3 * (img.width * img.height / 2 + img.width / 2);
                                byte b = color_data[offset];
                                byte g = color_data[offset + 1];
                                byte r = color_data[offset + 2];
                                Console.WriteLine(string.Format("Color Image Center Pixel value(YUYV):{0} {1} {2}", r, g, b));
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

                                int offset = 3 * (img.width * img.height / 2 + img.width / 2);
                                byte b = color_pixel_arr[offset];
                                byte g = color_pixel_arr[offset + 1];
                                byte r = color_pixel_arr[offset + 2];
                                Console.WriteLine(string.Format("Color Image Center Pixel value(Bayer):{0} {1} {2}", r, g, b));
                            }
                            else {
                                Console.WriteLine(string.Format("Color Image Type:{0}", img.pixelFormat));
                            }
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
