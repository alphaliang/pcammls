import com.percipio.pcammls.*;

public class test{

	public static void main(String[] args){
		System.loadLibrary("pcammls");
		Integer kk = 2;
		ttt(kk);
        System.out.printf("test %d\n",kk);
        pcammls.TYInitLib();
        TY_VERSION_INFO ver = new TY_VERSION_INFO();
        pcammls.TYLibVersion(ver);
        System.out.printf("sdk version is %d.%d.%d\n",ver.getMajor(),ver.getMinor(),ver.getPatch());
		pcammls.TYUpdateInterfaceList();
		long [] val_out = {0};
		System.out.println("call get interface num");
		pcammls.TYGetInterfaceNumber(val_out);
		System.out.printf("value is %d\n",val_out[0]);
		if (val_out[0]==0){
			System.out.println("no device interface");
			return;
		}
		TY_INTERFACE_INFO_ARRAY arr=new TY_INTERFACE_INFO_ARRAY((int)val_out[0]);
		System.out.println("create arr ok");
		long [] filled_num_out = {0};
		pcammls.TYGetInterfaceList(arr,val_out[0],filled_num_out);
		System.out.println("call list ok");
		for (int idx=0;idx<filled_num_out[0];idx++){
			System.out.printf("%d -- %s\n",idx,arr.getitem(idx).getId());
		}
        pcammls.TYDeinitLib();
	}
}
