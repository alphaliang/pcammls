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

            textBox1.Text = "depth2color";
            textBox2.Text = "RGB";

            CaptureCamera();
        }

        private Bitmap image;
        private Thread camera;
        private PercipioSDK cl;
        private DeviceInfoVector dev_list;
        private System.IntPtr handle;
        CSharpPercipioDeviceEvent _event;

        private bool isCameraRunning = false;

        private float scale_unit = 1.0f;

        private PercipioCalibData depth_calib;
        private PercipioCalibData color_calib;

        private image_data depth = new image_data();
        private image_data color = new image_data();

        private image_data registration_depth = new image_data();
        private image_data undsitortion_color = new image_data();

        private bool DeviceInit()
        {
            int err = 0;
            Console.WriteLine("test start\n");
            cl = new PercipioSDK();

            dev_list = cl.ListDevice();
            int sz = dev_list.Count();
            if (sz == 0)
            {
                Console.WriteLine(string.Format("no device found."));
                return false;
            }

            Console.WriteLine(string.Format("found follow devices:"));
            for (int idx = 0; idx < sz; idx++)
            {
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

            err = cl.DeviceLoadDefaultParameters(handle);
            if (err != TY_STATUS_OK)
                Console.WriteLine(string.Format("Load default parameters fail: {0}!", err));
            else
                Console.WriteLine(string.Format("Load default parameters successful!"));

            scale_unit = cl.DeviceReadCalibDepthScaleUnit(handle);
            depth_calib = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_DEPTH);
            color_calib = cl.DeviceReadCalibData(handle, PERCIPIO_STREAM_COLOR);

            err = cl.DeviceStreamEnable(handle, PERCIPIO_STREAM_DEPTH | PERCIPIO_STREAM_COLOR);
            if(err != TY_STATUS_OK) {
                Console.WriteLine(string.Format("enable stream err!"));
                return false;
            }
            
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
        private Image resizeImage(Image image, Size size)
        {
            return (Image)(new Bitmap(image, size));
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
                if (frames.Count() == 2)
                {
                    for (int i = 0; i < frames.Count(); i++)
                    {
                        image_data image = frames[i];
                        if (image.streamID == PERCIPIO_STREAM_DEPTH)
                        {
                            depth = image;
                        }
                        else if (image.streamID == PERCIPIO_STREAM_COLOR)
                        {
                            cl.DeviceStreamImageDecode(image, color);
                        }
                    }

                    cl.DeviceStreamDoUndistortion(color_calib, color, undsitortion_color);
                    cl.DeviceStreamMapDepthImageToColorCoordinate(depth_calib,
                            depth,
                            scale_unit,
                            color_calib,
                            undsitortion_color.width,
                            undsitortion_color.height,
                            registration_depth);

                    image_data depth_render = new image_data();
                    cl.DeviceStreamDepthRender(registration_depth, depth_render);
                    IntPtr pt1 = depth_render.buffer.getCPtr();
                    Bitmap bmp_depth = new Bitmap(depth_render.width, depth_render.height, depth_render.width * 3, PixelFormat.Format24bppRgb, pt1);
                    pictureBox1.Image =(Image)resizeImage(bmp_depth, new Size(640, 480)).Clone();

                    IntPtr pt2 = undsitortion_color.buffer.getCPtr();
                    Bitmap bmp_color = new Bitmap(undsitortion_color.width, undsitortion_color.height, 3 * undsitortion_color.width, PixelFormat.Format24bppRgb, pt2);
                    pictureBox2.Image = (Image)resizeImage(bmp_color, new Size(640, 480)).Clone();

                    Thread.Sleep(10);
                }
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
