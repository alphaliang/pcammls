using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows.Forms;

using pcammls;
using static pcammls.pcammls;

namespace demo
{
    class CSharpPercipioDeviceEvent : DeviceEvent
    {
        bool Offline = false;
        public override int run(SWIGTYPE_p_void handle, int event_id)
        {
            IntPtr dev = handle.getCPtr();
            if (event_id == TY_EVENT_DEVICE_OFFLINE)
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

        public void Reset()
        {
            Offline = false;
        }
    }

	class Program
	{
        static CSharpPercipioDeviceEvent _event;
		static void Main(string[] args)
        {
            Console.WriteLine("test start\n");

            PercipioSDK cl = new PercipioSDK();
            _event = new CSharpPercipioDeviceEvent();

            DeviceInfoVector dev_list = cl.ListDevice();
            int sz = dev_list.Count();
            if (sz == 0) {
                Console.WriteLine(string.Format("no device found."));
                return;
            }
			
			Console.WriteLine(string.Format("found follow devices:"));
            for (int idx = 0; idx < sz; idx++) {
                var item = dev_list[idx];
                Console.WriteLine("{0} -- {1} {2}", idx, item.id, item.modelName);
            }
            Console.WriteLine("select one:");
            int select = int.Parse(Console.ReadLine());

            string serialNumber = dev_list[select].id;

            System.IntPtr handle;

            while (true)
            {
                while (true)
                {
                    handle = cl.Open(serialNumber);
                    if (!cl.isValidHandle(handle))
                    {
                        Console.WriteLine(string.Format("can not open device!"));
                        Thread.Sleep(1000);
                        Console.WriteLine(string.Format("Reconnect try again!"));
                    }
                    else
                    {
                        break;
                    }
                }

                cl.DeviceRegiststerCallBackEvent(_event);

                int err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH);
                if (err != TY_STATUS_OK)
                {
                    Console.WriteLine(string.Format("device stream enable err:{0}", err));
                    return;
                }


                cl.DeviceStreamOn(handle);

                while (true)
                {
                    if (_event.isOffLine())
                    {
                        _event.Reset();
                        break;
                    }
                    FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                    for (int i = 0; i < frames.Count(); i++)
                    {
                        image_data image = frames[i];
                        if (image.streamID == PERCIPIO_STREAM_DEPTH)
                        {
                            Console.WriteLine(string.Format("color image info : {0} x {1}", image.width, image.height));
                        }
                    }
                }

                cl.DeviceStreamOff(handle);
                cl.Close(handle);
            }

        }
    }
	
}