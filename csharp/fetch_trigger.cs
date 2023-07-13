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

            float depth_scale_unit = cl.DeviceReadCalibDepthScaleUnit(handle);
            Console.WriteLine(string.Format("depth image scale unit:{0}", depth_scale_unit));

            cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH);
            
            EnumEntryVector depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH);
            Console.WriteLine(string.Format("depth image format list:"));
            for (int i = 0; i < depth_fmt_list.Count(); i++)
            {
                TY_ENUM_ENTRY fmt = depth_fmt_list[i];
                Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
            }
            cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0]);

            cl.DeviceControlTriggerModeEnable(handle, 1);

            cl.DeviceStreamOn(handle);
            
            while(true) {
                if (_event.isOffLine())
                    break;
                cl.DeviceControlTriggerModeSendTriggerSignal(handle);
                FrameVector frames = cl.DeviceStreamFetch(handle, 2000);
                for (int i = 0; i < frames.Count(); i++)
                {
                    image_data image = frames[i];
                    if (image.streamID == PERCIPIO_STREAM_DEPTH) 
                    {
                        unsafe
                        {
                            IntPtr pt = image.buffer.getCPtr();
                            ushort* ptr = (ushort*)pt.ToPointer();
                            int width = image.width;
                            int height = image.height;
                            int dep = ptr[width * height / 2 + width / 2];
                            float distance = depth_scale_unit * dep;
                            Console.WriteLine(string.Format("\tdistance :{0}", distance));
                        }
                    }
                }
            }
            
            cl.DeviceStreamOff(handle);
            
            cl.Close(handle);
        }
    }
}
