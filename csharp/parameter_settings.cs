using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Threading;
using System.Windows.Forms;

using pcammls;
using static pcammls.pcammls;

namespace demo
{

	class Program
	{
		static void Main(string[] args)
        {
            Console.WriteLine("test start\n");

            PercipioSDK cl = new PercipioSDK();

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

            System.IntPtr handle = cl.Open(dev_list[select].id);
            if (!cl.isValidHandle(handle))
            {
                Console.WriteLine(string.Format("can not open device!"));
                return;
            }

            int err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR);
            if (err != TY_STATUS_OK)
            {
                Console.WriteLine(string.Format("device stream enable err:{0}", err));
                return;
            }
            //bool:color aec
            DevParam aec = cl.DeviceGetParameter(handle, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE);
            if (aec.isEmpty())
                Console.WriteLine(string.Format("aec is not support!"));
            else { 
                Console.WriteLine(string.Format("current aec status : {0}", aec.toBool()));

                //disable color aec
                aec = cl.DevParamFromBool(false);
                err = cl.DeviceSetParameter(handle, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE, aec);
                Console.WriteLine(string.Format("aec close result : {0}", err));
            }

            // int:color exposure time
            DevParam exp = cl.DeviceGetParameter(handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME);
            if (exp.isEmpty())
                Console.WriteLine(string.Format("exposure time is not support!"));
            else { 
                Console.WriteLine(string.Format("current exposure time status : {0}, range : {1} - {2}, inc : {3}",
                    exp.toInt(), exp.mMin(), exp.mMax(), exp.mInc()));

                Console.WriteLine("Enter exposure time:");
                int exposure_time = int.Parse(Console.ReadLine());
                exp = cl.DevParamFromInt(exposure_time);
                err = cl.DeviceSetParameter(handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, exp);
                Console.WriteLine(string.Format("set color exposure time result : {0}", err));
            }

            //enum: color image mode
            DevParam image_mode = cl.DeviceGetParameter(handle, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE);
            if (image_mode.isEmpty())
                Console.WriteLine(string.Format("color image mode is not support!"));
            else {
                EnumEntryVector list = image_mode.eList();
                for (int idx = 0; idx < list.Count(); idx++) {
                    var mode = list[idx];
                    Console.WriteLine(string.Format("{0}: {1}x{2} - {3}", idx, cl.Width(mode), cl.Height(mode), cl.Description(mode)));
                }

                Console.WriteLine(string.Format("Enter image mode index:"));
                int index = int.Parse(Console.ReadLine());
                image_mode = cl.DevParamFromEnum(cl.Value(list[index]));
                err = cl.DeviceSetParameter(handle, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, image_mode);
                Console.WriteLine(string.Format("set color image mode result : {0}", err));
            }
            cl.DeviceStreamOn(handle);
            image_data img_parsed_color = new image_data();
             
            while (true) {
                FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                for (int i = 0; i < frames.Count(); i++) {
                    image_data image = frames[i];
                    if (image.streamID == PERCIPIO_STREAM_COLOR)
                    {
                        cl.DeviceStreamImageDecode(image, img_parsed_color);
                        Console.WriteLine(string.Format("color image info : {0} x {1}", img_parsed_color.width, img_parsed_color.height));
                    }
                }
            }

            cl.DeviceStreamOff(handle);
            cl.Close(handle);

        }
    }
	
}