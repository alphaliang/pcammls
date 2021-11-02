using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using pcammls;
using SDK = pcammls.pcammls;

namespace pcammls_test
{

    class Program
    {

        static void ShowDeviceList()
        {
            SDK.TYUpdateInterfaceList();
            uint num = 0;
            SDK.TYGetInterfaceNumber(out num);
            if (num == 0)
            {
                return;
            }
            TY_INTERFACE_INFO_ARRAY arr = new TY_INTERFACE_INFO_ARRAY((int)num);
            uint filled;
            SDK.TYGetInterfaceList(arr, num, out filled);
            for (int idx = 0; idx < filled; idx++)
            {
                TY_INTERFACE_INFO ifaceinfo = arr[idx];
                Console.WriteLine(string.Format("Interface: {0} {1}", ifaceinfo.id, ifaceinfo.type));

                IntPtr iface_handle = new IntPtr();
                SDK.TYOpenInterface(ifaceinfo.id, ref iface_handle);
                SDK.TYUpdateDeviceList(iface_handle);
                uint dev_num,dev_arr_sz;
                SDK.TYGetDeviceNumber(iface_handle, out dev_num);
                if (dev_num == 0)
                {
                    continue;
                }
                TY_DEVICE_BASE_INFO_ARRAY dev_arr = new TY_DEVICE_BASE_INFO_ARRAY((int)dev_num);
                SDK.TYGetDeviceList(iface_handle, dev_arr, dev_num, out dev_arr_sz);
                for(int didx = 0; didx < dev_arr_sz; didx++)
                {
                    var dev_info = dev_arr[didx];
                    Console.WriteLine(string.Format("\t{0} {1}",dev_info.id,dev_info.modelName));
                }
            }
        }


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

        static void DisplayArray(float_ARRAY arr , int sz)
        {
            var sb = new StringBuilder();
            sb.Append("\t\t[");
            for (int idx = 0; idx < sz; idx++)
            {
                sb.AppendFormat("{0},  ", arr[idx]);
            }
            sb.Append("]");
            Console.WriteLine(sb.ToString());
        }

        static void ShowDeviceFeatures(IntPtr handle)
        {
            uint compIDs;
            SDK.TYGetComponentIDs(handle,out compIDs);
            Dictionary<uint,string> comp_list = new Dictionary<uint, string>{
                { SDK.TY_COMPONENT_DEPTH_CAM ,"TY_COMPONENT_DEPTH_CAM"},
                { SDK.TY_COMPONENT_RGB_CAM ,"TY_COMPONENT_RGB_CAM"},
                { SDK.TY_COMPONENT_IR_CAM_LEFT ,"TY_COMPONENT_IR_CAM_LEFT"}
            };
            Dictionary<uint, string> feature_list = new Dictionary<uint, string> {
                {SDK.TY_INT_WIDTH,"TY_INT_WIDTH" },
                {SDK.TY_INT_HEIGHT,"TY_INT_HEIGHT" },
                {SDK.TY_ENUM_IMAGE_MODE,"TY_ENUM_IMAGE_MODE" },
                {SDK.TY_STRUCT_CAM_INTRINSIC,"TY_STRUCT_CAM_INTRINSIC" },
                {SDK.TY_STRUCT_CAM_DISTORTION,"TY_STRUCT_CAM_DISTORTION" },
                {SDK.TY_STRUCT_EXTRINSIC_TO_DEPTH,"TY_STRUCT_EXTRINSIC_TO_DEPTH" },
            };
            foreach (var comp in comp_list)
            {
                if ((comp.Key & compIDs) == 0)
                {
                    continue;
                }
                Console.WriteLine(string.Format("== {0}", comp.Value));
                foreach (var feat in feature_list)
                {
                    bool has_feat;
                    SDK.TYHasFeature(handle, comp.Key, feat.Key, out has_feat);
                    if (!has_feat)
                    {
                        continue;
                    }
                    uint feat_type = SDK.TYFeatureType(feat.Key);
                    if (feat_type == SDK.TY_FEATURE_INT)
                    {
                        int val;
                        SDK.TYGetInt(handle, comp.Key, feat.Key, out val);
                        Console.WriteLine(string.Format("\t{0}: {1}", feat.Value, val));
                    }
                    if (feat_type == SDK.TY_FEATURE_ENUM)
                    {
                        Console.WriteLine(string.Format("\t{0}:", feat.Value));
                        uint entryCount,filledCount;
                        SDK.TYGetEnumEntryCount(handle, comp.Key, feat.Key, out entryCount);
                        TY_ENUM_ENTRY_ARRAY arr = new TY_ENUM_ENTRY_ARRAY((int)entryCount);
                        SDK.TYGetEnumEntryInfo(handle, comp.Key, feat.Key, arr, entryCount, out filledCount);
                        for(int idx = 0; idx < filledCount; idx++)
                        {
                            var entry = arr[idx];
                            Console.WriteLine(string.Format("\t\t {0} : {1}", entry.description, entry.value));
                        }
                    }
                    if (feat_type == SDK.TY_FEATURE_STRUCT)
                    {
                        Console.WriteLine(string.Format("\t{0}:", feat.Value));
                        if (SDK.TY_STRUCT_CAM_INTRINSIC == feat.Key)
                        {
                            TY_CAMERA_INTRINSIC intri = new TY_CAMERA_INTRINSIC();
                            SDK.TYGetStruct(handle, comp.Key, feat.Key, intri.getCPtr(), intri.CSize());
                            var arr = intri.data;
                            DisplayArray(arr, 9);
                        }
                        if (SDK.TY_STRUCT_EXTRINSIC_TO_DEPTH == feat.Key)
                        {
                            TY_CAMERA_EXTRINSIC intri = new TY_CAMERA_EXTRINSIC();
                            SDK.TYGetStruct(handle, comp.Key, feat.Key, intri.getCPtr(), intri.CSize());
                            var arr = intri.data;
                            DisplayArray(arr, 16);
                        }
                        if (SDK.TY_STRUCT_CAM_DISTORTION == feat.Key)
                        {
                            TY_CAMERA_DISTORTION intri = new TY_CAMERA_DISTORTION();
                            SDK.TYGetStruct(handle, comp.Key, feat.Key, intri.getCPtr(), intri.CSize());
                            var arr = intri.data;
                            DisplayArray(arr, 9);
                        }
                    }
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
                ShowDeviceList();
                var dev_info = SimpleDeviceSelect();
                if (dev_info != null)
                {
                    IntPtr dev_handle = new IntPtr();
                    IntPtr iface_handle = new IntPtr();

                    SDK.TYOpenInterface(dev_info.iface.id, ref iface_handle);

                    IntPtr errCode = IntPtr.Zero;
                    SDK.TYOpenDevice(iface_handle, dev_info.id, ref dev_handle, ref errCode);
                    ShowDeviceFeatures(dev_handle);
                    SDK.TYCloseDevice(dev_handle, false);
                    SDK.TYCloseInterface(iface_handle);
                }
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
