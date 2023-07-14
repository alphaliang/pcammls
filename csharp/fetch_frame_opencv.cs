using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using pcammls;
using static pcammls.pcammls;
using pcammls_isp;
using SDK_ISP = pcammls_isp.pcammls_isp_api;
using OpenCvSharp;
using static OpenCvSharp.Stitcher;

namespace pcammls_fetch_frame
{
    class CSharpPercipioDeviceEvent : DeviceEvent
    {
        bool Offline = false;
        public override int run(SWIGTYPE_p_void handle, int event_id)
        {
            IntPtr dev = handle.getCPtr();
            if(event_id == TY_EVENT_DEVICE_OFFLINE)
            {
                Offline = true;
                Console.WriteLine(string.Format("=== Event Callback: Device Offline"));
            }
            return 0;
        }

        public bool isOffLine()
        {
            return Offline;
        }
    }
    class Program
    {
        static void dump_calib_data(CalibDataVector data)
        { 
            for(int i = 0; i < data.Count(); i++)
                Console.Write(string.Format("{0}\t", data[i]));
            Console.WriteLine("\n");
        }
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

            CSharpPercipioDeviceEvent _event = new CSharpPercipioDeviceEvent();

            cl.DeviceRegiststerCallBackEvent(_event);

            cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR | PERCIPIO_STREAM_DEPTH);
            
            EnumEntryVector color_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_COLOR);
            Console.WriteLine(string.Format("color image format list:"));
            for (int i = 0; i < color_fmt_list.Count(); i++) { 
                TY_ENUM_ENTRY fmt = color_fmt_list[i];
                Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
            }
            cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_COLOR, color_fmt_list[0]);

            EnumEntryVector depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH);
            Console.WriteLine(string.Format("depth image format list:"));
            for (int i = 0; i < depth_fmt_list.Count(); i++)
            {
                TY_ENUM_ENTRY fmt = depth_fmt_list[i];
                Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
            }
            cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0]);

            PercipioCalibData color_calib_data = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_COLOR);
            int color_calib_width = color_calib_data.Width();
            int color_calib_height = color_calib_data.Height();
            Console.WriteLine(string.Format("color image calib size: {0}x{1}", color_calib_width, color_calib_height));
            CalibDataVector color_intr = color_calib_data.Intrinsic();
            Console.WriteLine(string.Format("color image calib intrinsic:"));
            dump_calib_data(color_intr);
            CalibDataVector color_extr = color_calib_data.Extrinsic();
            Console.WriteLine(string.Format("color image calib extrinsic:"));
            dump_calib_data(color_extr);
            CalibDataVector color_dist = color_calib_data.Distortion();
            Console.WriteLine(string.Format("color image calib distortion:"));
            dump_calib_data(color_dist);

            PercipioCalibData depth_calib_data = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_DEPTH);
            int depth_calib_width = depth_calib_data.Width();
            int depth_calib_height = depth_calib_data.Height();
            Console.WriteLine(string.Format("depth image calib size: {0}x{1}", depth_calib_width, depth_calib_height));
            CalibDataVector depth_intr = depth_calib_data.Intrinsic();
            Console.WriteLine(string.Format("depth image calib intrinsic:"));
            dump_calib_data(depth_intr);
            CalibDataVector depth_extr = depth_calib_data.Extrinsic();
            CalibDataVector depth_dist = depth_calib_data.Distortion();

            cl.DeviceStreamOn(handle);
            
            while(true) {
                if (_event.isOffLine())
                    break;

                FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                for (int i = 0; i < frames.Count(); i++)
                {
                    image_data image = frames[i];
                    if (image.streamID == PERCIPIO_STREAM_DEPTH) 
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
