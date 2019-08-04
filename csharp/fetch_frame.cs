using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using pcammls;
using SDK = pcammls.pcammls;

namespace pcammls_fetch_frame
{

    class Program
    {
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
            SDK.TYEnableComponents(handle,SDK.TY_COMPONENT_DEPTH_CAM);
            List<uint8_t_ARRAY> buffers = new List<uint8_t_ARRAY>();
            uint buff_sz;
            SDK.TYGetFrameBufferSize(handle, out buff_sz);
            buffers.Add(new uint8_t_ARRAY((int)buff_sz));
            buffers.Add(new uint8_t_ARRAY((int)buff_sz));
            SDK.TYEnqueueBuffer(handle, buffers[0].VoidPtr(), buff_sz);
            SDK.TYEnqueueBuffer(handle, buffers[1].VoidPtr(), buff_sz);
            SDK.TYStartCapture(handle);
            int img_index = 0;
            while (true)
            {
                TY_FRAME_DATA frame = new TY_FRAME_DATA();
                try
                {
                    SDK.TYFetchFrame(handle, frame, 5000);
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
                            Console.WriteLine(string.Format("Image Center Pixel Distance:{0}", distance));
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
