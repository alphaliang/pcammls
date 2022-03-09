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
    }
};