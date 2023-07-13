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
using OpenCvSharp;
using static OpenCvSharp.Stitcher;

namespace pcammls_fetch_frame
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("test start\n");
            PercipioSDK cl = new PercipioSDK();
            System.IntPtr handle = cl.Open();
            if (!cl.isValidHandle(handle))
            {
                Console.WriteLine(string.Format("can not open device!"));
                return;
            }
            
            cl.DeviceStreamEnable(handle, SDK.PERCIPIO_STREAM_COLOR | SDK.PERCIPIO_STREAM_DEPTH);
            
            EnumEntryVector color_fmt_list = cl.DeviceStreamFormatDump(handle, SDK.PERCIPIO_STREAM_COLOR);
            Console.WriteLine(string.Format("color image format list:"));
            for (int i = 0; i < color_fmt_list.Count(); i++) { 
                TY_ENUM_ENTRY fmt = color_fmt_list[i];
                Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
            }
            cl.DeviceStreamFormatConfig(handle, SDK.PERCIPIO_STREAM_COLOR, color_fmt_list[0]);

            EnumEntryVector depth_fmt_list = cl.DeviceStreamFormatDump(handle, SDK.PERCIPIO_STREAM_DEPTH);
            Console.WriteLine(string.Format("depth image format list:"));
            for (int i = 0; i < depth_fmt_list.Count(); i++)
            {
                TY_ENUM_ENTRY fmt = depth_fmt_list[i];
                Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
            }
            cl.DeviceStreamFormatConfig(handle, SDK.PERCIPIO_STREAM_DEPTH, depth_fmt_list[0]);
            
            cl.DeviceStreamOn(handle);
            
            while(true) {
                FrameVector frames = cl.DeviceStreamFetch(handle, 2000);
                for (int i = 0; i < frames.Count(); i++)
                {
                    image_data image = frames[i];
                    if (image.streamID == SDK.PERCIPIO_STREAM_DEPTH) 
                    {
                        IntPtr pt = image.buffer.getCPtr();
                        Mat depthimg = new Mat(image.height, image.width, MatType.CV_16U, pt);
                        OpenCvSharp.Cv2.ImShow("depth", depthimg * 15);
                    }
                }
                
                int key = Cv2.WaitKey(1);
                if (key == 'q')
                    break;
            }
            
            cl.DeviceStreamOff(handle);
            
            cl.Close(handle);
        }
    }
}
