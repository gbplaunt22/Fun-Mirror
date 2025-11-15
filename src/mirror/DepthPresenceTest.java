package mirror;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Freenect;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.LedStatus;
import org.openkinect.freenect.TiltStatus;

import java.nio.ByteBuffer;
import java.nio.ShortBuffer;

public class DepthPresenceTest {

	public static void main(String[] args) {
		
	//SETUP////////////////////////////////////////
		//helper!
		KinectHelpers helper = new KinectHelpers();
		//create the context
		Context ctx = Freenect.createContext();
		//Is context created? (prints to console)
		helper.getContextStatusVoid(ctx);
		//Is a device found? (prints to console)
		int devicesFound = ctx.numDevices();
		helper.kinectFound(devicesFound);
		//get device 0
		Device dev = ctx.openDevice(0);
		System.out.println("Opened device 0");
		
		//Set LED to blink while program is running
		dev.setLed(LedStatus.BLINK_RED_YELLOW);
		
		//Useful values
		dev.refreshTiltState();
		double angle = dev.getTiltAngle();
		TiltStatus tiltStatus = dev.getTiltStatus();
		double [] accel = dev.getAccel(); // [ax, ay, az]
		
		//Lets see em!
		System.out.println("Tilt angle: " + angle + " degrees" );
		System.out.println("Tilt status: " + tiltStatus);
		System.out.println("Accel (x,y,z): " + 
				accel[0] + ", " + accel[1] + ", " + accel[2]);
		
		//Testing Tilt
		System.out.println("Tilting by 15 degrees, hold your horses!");
		
		
		
		
		dev.setLed(LedStatus.OFF);
		dev.close();
		System.exit(0);
	////////////////////////////////////////////////
		
	}

}
