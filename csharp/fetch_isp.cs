using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
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
    }

    public partial class Form1 : Form
    {
        public Form1()
        {
            if (!DeviceInit()) {

                Console.WriteLine("Press Enter to exit");
                Console.Read();
                Environment.Exit(0);
            }
            InitializeComponent();

            textBox1.Text = "RGB";
            textBox2.Text = "EMPTY";

            CaptureCamera();
        }

        private Bitmap image;
        private Thread camera;
        private PercipioSDK cl;
        private DeviceInfoVector dev_list;
        private System.IntPtr handle;
        CSharpPercipioDeviceEvent _event;

        private bool isCameraRunning = false;

        private void dump_calib_data(CalibDataVector data, int col, int row)
        {
            for (int i = 0; i < row; i++) {
                for (int j = 0; j < col; j++) {
                    int idx = i * col + j;
                    if (idx < data.Count())
                        Console.Write("\t{0}\t", data[idx]);
                    else {
                        Console.Write("\n");
                        return;
                    }
                }
                Console.Write("\n");
            }
        }

        private bool DeviceInit()
        {
            Console.WriteLine("test start\n");
            cl = new PercipioSDK();

            dev_list = cl.ListDevice();
            int sz = dev_list.Count();
            if (sz == 0) {
                Console.WriteLine(string.Format("no device found."));
                return false;
            }

            Console.WriteLine(string.Format("found follow devices:"));
            for (int idx = 0; idx < sz; idx++) {
                var item = dev_list[idx];
                Console.WriteLine("{0} -- {1} {2}", idx, item.id, item.modelName);
            }
            Console.WriteLine("select one:");
            int select = int.Parse(Console.ReadLine());

            handle = cl.Open(dev_list[select].id);
            if (!cl.isValidHandle(handle))
            {
                Console.WriteLine(string.Format("can not open device!"));
                return false;
            }

            _event = new CSharpPercipioDeviceEvent();

            cl.DeviceRegiststerCallBackEvent(_event);

            cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_COLOR);

            EnumEntryVector color_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_COLOR);
			if(color_fmt_list.Count() != 0)
			{
				Console.WriteLine(string.Format("color image format list:"));
				for (int i = 0; i < color_fmt_list.Count(); i++)
				{
					TY_ENUM_ENTRY fmt = color_fmt_list[i];
					Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
				}
				cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_COLOR, color_fmt_list[color_fmt_list.Count() - 1]);

				//enable rgb image software isp 
				cl.DeviceColorStreamIspEnable(handle, true);
				return true;
			}
			return false;
        }

        private void CaptureCamera()
        {
            camera = new Thread(new ThreadStart(CaptureCameraCallback));
            camera.Start();
        }

        private IntPtr ArrToPtr(byte[] array)
        {
            return System.Runtime.InteropServices.Marshal.UnsafeAddrOfPinnedArrayElement(array, 0);
        }
        private void CaptureCameraCallback()
        {
            cl.DeviceStreamOn(handle);
            isCameraRunning = true;

            while (isCameraRunning)
            {
                if (_event.isOffLine())
                    break;

                FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                for (int i = 0; i < frames.Count(); i++)
                {
                    if (frames[i].streamID == PERCIPIO_STREAM_COLOR)
                    {
                        image_data bgr = new image_data();
                        cl.DeviceStreamImageDecode(frames[i], bgr);
                        IntPtr pt = bgr.buffer.getCPtr();
                        Bitmap bmp_color = new Bitmap(bgr.width, bgr.height, bgr.width * 3, PixelFormat.Format24bppRgb, pt);
                        pictureBox1.Image = (Image)(new Bitmap(bmp_color, new Size(640, 480))).Clone();
                    }
                }

                Application.DoEvents();
                Thread.Sleep(10);
            }

            cl.DeviceStreamOff(handle);

            cl.Close(handle);
        }

        private void button1_Click(object sender, EventArgs e)
        {
            isCameraRunning = false;
            camera.Join();

            Application.Exit();
        }
    }
}
