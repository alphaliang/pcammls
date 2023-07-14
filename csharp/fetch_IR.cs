using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using pcammls;
using static pcammls.pcammls;
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

            cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_IR_LEFT | PERCIPIO_STREAM_IR_RIGHT);
            
            cl.DeviceStreamOn(handle);
            
            while(true) {
                if (_event.isOffLine())
                    break;

                FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                Console.WriteLine(string.Format("DeviceStreamRead frames:{0}!", frames.Count()));
                for (int i = 0; i < frames.Count(); i++)
                {
                    image_data image = frames[i];
                    if (image.streamID == PERCIPIO_STREAM_IR_LEFT) 
                    {
                       //Console.WriteLine("IR LEFT ");
                    }
                    else if (image.streamID == PERCIPIO_STREAM_IR_RIGHT)
                    {
                       //Console.WriteLine("IR RIGHT ");
                    }
                }
            }
            
            cl.DeviceStreamOff(handle);
            
            cl.Close(handle);
        }
    }
}
