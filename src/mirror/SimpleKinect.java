package mirror;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;
import org.openkinect.freenect.LedStatus;
import org.openkinect.freenect.DepthHandler;
import org.openkinect.freenect.FrameMode;

public class SimpleKinect {

	public static void main(String[] args) {
		//Create the context
		Context ctx = Freenect.createContext();
		//How many Kinects?
		int num = ctx.numDevices();
		//get device 0
		Device dev = ctx.openDevice(0);
		
		dev.close();
		ctx.shutdown();
		

	}

}
