using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using pcammls;
using SDK = pcammls.pcammls;



namespace pcammls_isp
{
    class pcammls_isp_api{
        static double[,] YUV2RGB_CONVERT_MATRIX = new double[3, 3] { { 1, 0, 1.4022 }, { 1, -0.3456, -0.7145 }, { 1, 1.771, 0 } };
        public static void ConvertYUYV2RGB(uint8_t_ARRAY yuvFrame, uint8_t_ARRAY rgbFrame, int width, int height)
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

        public static void ConvertYVYU2RGB(uint8_t_ARRAY yuvFrame, uint8_t_ARRAY rgbFrame, int width, int height)
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
        
        static void __TYParseSizeFromImageMode(int mode, int[] image_size)
        {
            const int mask = ((0x01 << 12) - 1);
            int height = mode & mask;
            int width = (mode >> 12) & mask;
            image_size[0] = width;
            image_size[1] = height;
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
            if(Environment.Is64BitProcess)
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
        public static void ColorIspInitSetting(IntPtr isp_handle, IntPtr handle)
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

            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_BAYER_PATTERN, SDK.TY_ISP_BAYER_AUTO);

            SWIGTYPE_p_unsigned_char pointer;

            float []fShading = new float[9];
            fShading[0] = 0.30890417098999026f;  fShading[1] = 10.63355541229248f; fShading[2] = -6.433426856994629f;
            fShading[3] = 0.24413758516311646f;  fShading[4] = 11.739893913269043f;fShading[5] =  -8.148622512817383f;
            fShading[6] = 0.1255662441253662f;   fShading[7] = 11.88359546661377f; fShading[8] = -7.865192413330078f;
            pointer = FloatArrayToSWIGTYPE(fShading, 9);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_SHADING, pointer, 9 * sizeof(float));
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_CCM_ENABLE, 0);//we are not using ccm by default
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_CAM_DEV_HANDLE, (int)handle);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_CAM_DEV_COMPONENT, SDK.TY_COMPONENT_RGB_CAM);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_GAMMA, 1.0f);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_AUTOBRIGHT, 1);//enable auto bright control
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_ENABLE_AUTO_EXPOSURE_GAIN, 0);//disable ae by default

            int[] default_image_size = new int[2];// = { 1280, 960 };// image size 
            default_image_size[0] = 1280;
            default_image_size[1] = 960;
            int[] current_image_size = new int[2];// = { 1280, 960 };// image size for current parameters
            current_image_size[0] = 1280;
            current_image_size[1] = 960;

            int img_mode;
            int res = SDK.TYGetEnum(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_ENUM_IMAGE_MODE, out img_mode);
            if (res == SDK.TY_STATUS_OK)
            {
                __TYParseSizeFromImageMode(img_mode, current_image_size);
            }

            TY_ENUM_ENTRY_ARRAY mode_entry = new TY_ENUM_ENTRY_ARRAY(10);
            uint num;
            res = SDK.TYGetEnumEntryInfo(handle, SDK.TY_COMPONENT_RGB_CAM, SDK.TY_ENUM_IMAGE_MODE, mode_entry, 10, out num);
            if (res == SDK.TY_STATUS_OK)
            {
                __TYParseSizeFromImageMode(mode_entry[0].value, default_image_size);
            }

            pointer = IntArrayToSWIGTYPE(default_image_size, 2);
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_IMAGE_SIZE, pointer, 2 * sizeof(int));//the orignal raw image size
            SDK.TYISPSetFeature(isp_handle, SDK.TY_ISP_FEATURE_INPUT_RESAMPLE_SCALE, default_image_size[0] / current_image_size[0]);//resampled input 

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
    }
};