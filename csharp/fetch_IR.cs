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

            textBox1.Text = "Left IR";
            textBox2.Text = "Right IR";

            CaptureCamera();
        }

        private Bitmap image;
        private Thread camera;
        private PercipioSDK cl;
        private System.IntPtr handle;
        CSharpPercipioDeviceEvent _event;

        private bool isCameraRunning = false;

        private bool DeviceInit()
        {
            Console.WriteLine("test start\n");
            cl = new PercipioSDK();
            handle = cl.Open();
            if (!cl.isValidHandle(handle))
            {
                Console.WriteLine(string.Format("can not open device!"));
                return false;
            }

            _event = new CSharpPercipioDeviceEvent();

            cl.DeviceRegiststerCallBackEvent(_event);

            EnumEntryVector ir_fmt_list = cl.DeviceStreamFormatDump(handle, PERCIPIO_STREAM_IR_LEFT);
            Console.WriteLine(string.Format("ir image format list:"));
            for (int i = 0; i < ir_fmt_list.Count(); i++)
            {
                TY_ENUM_ENTRY fmt = ir_fmt_list[i];
                Console.WriteLine(string.Format("\t{0} -size[{1}x{2}]\t-\t desc:{3}", i, cl.Width(fmt), cl.Height(fmt), fmt.getDesc()));
            }
            cl.DeviceStreamFormatConfig(handle, PERCIPIO_STREAM_COLOR, ir_fmt_list[0]);

            cl.DeviceControlLaserPowerAutoControlEnable(handle, false);
            cl.DeviceControlLaserPowerConfig(handle, 80);


            cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_IR_LEFT | PERCIPIO_STREAM_IR_RIGHT);
            
			return true;
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

        private ColorPalette GreyColorPalette = null;
        private ColorPalette Grey16ColorPalette = null;

        private void CaptureCameraCallback()
        {
            cl.DeviceStreamOn(handle);
            isCameraRunning = true;

            while (isCameraRunning)
            {
                if (_event.isOffLine())
                    break;

                FrameVector frames = cl.DeviceStreamRead(handle, 2000);
                Console.WriteLine(string.Format("DeviceStreamRead frames:{0}!", frames.Count()));
                for (int i = 0; i < frames.Count(); i++)
                {
                    if (frames[i].streamID == PERCIPIO_STREAM_IR_LEFT)
                    {
                        image_data leftIR = new image_data();
                        cl.DeviceStreamIRRender(frames[i], leftIR);
                        IntPtr pt = leftIR.buffer.getCPtr();
						
						Bitmap leftIR_BMP = new Bitmap(leftIR.width, leftIR.height, 3*leftIR.width, PixelFormat.Format24bppRgb, pt);
                        pictureBox1.Image = (Image)(new Bitmap(leftIR_BMP, new Size(640, 480))).Clone();
                    }
                    else if (frames[i].streamID == PERCIPIO_STREAM_IR_RIGHT)
                    {
                        image_data rightIR = new image_data();
                        cl.DeviceStreamIRRender(frames[i], rightIR);
                        IntPtr pt = rightIR.buffer.getCPtr();
                        Bitmap rightIR_BMP = new Bitmap(rightIR.width, rightIR.height, 3*rightIR.width, PixelFormat.Format24bppRgb, pt);
                        pictureBox2.Image = (Image)(new Bitmap(rightIR_BMP, new Size(640, 480))).Clone();
                    }
                }

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
