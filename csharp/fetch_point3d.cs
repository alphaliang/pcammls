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
    }
    class Program
    {
        static void Main(string[] args)
        {
            int err = 0;
            Console.WriteLine("test start\n");
            PercipioSDK cl = new PercipioSDK();

            DeviceInfoVector dev_list = cl.ListDevice();
            int sz = dev_list.Count();
            if (sz == 0)
            {
                Console.WriteLine(string.Format("no device found."));
                return ;
            }

            Console.WriteLine(string.Format("found follow devices:"));
            for (int idx = 0; idx < sz; idx++)
            {
                var item = dev_list[idx];
                Console.WriteLine("{0} -- {1} {2}", idx, item.id, item.modelName);
            }
            Console.WriteLine("select one:");
            int select = int.Parse(Console.ReadLine());

            IntPtr handle = cl.Open(dev_list[select].id);
            if (!cl.isValidHandle(handle))
            {
                Console.WriteLine(string.Format("can not open device!"));
                return;
            }

            CSharpPercipioDeviceEvent _event = new CSharpPercipioDeviceEvent();

            cl.DeviceRegiststerCallBackEvent(_event);

            EnumEntryVector depth_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_DEPTH);
            Console.WriteLine(string.Format("depth image format list:"));
            for (int i = 0; i < depth_fmt_list.Count(); i++)
            {
                TY_ENUM_ENTRY fmt = depth_fmt_list[i];
                Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
            }
            cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0]);

            err = cl.DeviceLoadDefaultParameters(handle);
            if (err != TY_STATUS_OK)
                Console.WriteLine(string.Format("Load default parameters fail: {0}!", err));
            else
                Console.WriteLine(string.Format("Load default parameters successful!"));

            float f_depth_scale = cl.DeviceReadCalibDepthScaleUnit(handle);
            PercipioCalibData depth_calib_data = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_DEPTH);

            err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH);
            if(err != TY_STATUS_OK) {
                Console.WriteLine(string.Format("enable stream err!"));
                return;
            }

            pointcloud_data_list p3d_list = new pointcloud_data_list();
            cl.DeviceStreamOn(handle);

            while (true)
            {
                if (_event.isOffLine())
                    break;

                FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                for (int i = 0; i < frames.Count(); i++)
                {
                    image_data image = frames[i];
                    if (image.streamID == PERCIPIO_STREAM_DEPTH)
                    {
                        if (cl.DeviceStreamMapDepthImageToPoint3D(image, depth_calib_data, f_depth_scale, p3d_list) == TY_STATUS_OK)
                        {
                            int cnt = p3d_list.size();
                            int center = image.width * image.height / 2 + image.width / 2;
                            pointcloud_data _p3d = p3d_list.get_value(center);
                            Console.WriteLine(string.Format("\tpoint cloud center value: {0} {1} {2}", _p3d.getX(), _p3d.getY(), _p3d.getZ()));
                        }

                    }
                }

            }

            cl.DeviceStreamOff(handle);

            cl.Close(handle);
        }
    }
}
