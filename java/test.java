import com.percipio.pcammls.*;

public class test{
	public static void main(String[] args){
		System.out.println("hello");
		System.loadLibrary("pcammls");
        pcammls.TYInitLib();
        TY_VERSION_INFO ver = new TY_VERSION_INFO();
        pcammls.TYLibVersion(ver);
        System.out.printf("%d.%d.%d\n",ver.getMajor(),ver.getMinor(),ver.getPatch());
        pcammls.TYDeinitLib();
	}
}
