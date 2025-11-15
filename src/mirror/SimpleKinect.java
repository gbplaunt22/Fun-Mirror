package mirror;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;
import org.openkinect.freenect.LedStatus;
import org.openkinect.freenect.TiltStatus;


public class SimpleKinect {

	public static void main(String[] args) {
		//Create the context
		Context ctx = Freenect.createContext();
		
		if (ctx==null) {
			System.err.println("Could not create freenect context!");
			return;
		}
		
		
		//How many Kinects?
		int num = ctx.numDevices();
		System.out.println("Kinect devices found" + num);
		
		if(num == 0) {
			System.out.println("No devices found");
			return;
		}
		
		
		
		//get device 0
		Device dev = ctx.openDevice(0);
		System.out.println("Opened device 0");
		
		//Hello world! #Blink LED
		dev.setLed(LedStatus.BLINK_RED_YELLOW);
		System.out.println("Blinking LED until end of program...");
		System.out.println("Waiting 3 seconds just for the F#&% of it");
		sleep(3000);

		//Tilt + accelerometer test
		dev.refreshTiltState();
		double angle = dev.getTiltAngle();
		TiltStatus status = dev.getTiltStatus();
		double [] accel = dev.getAccel();       // [ax, ay, az]
		 
		System.out.println("Tilt angle: " + angle + " degrees" );
		System.out.println("Tilt status: " + status);
		System.out.println("Accel (x,y,z): " + 
				accel[0] + ", " + accel[1] + ", " + accel[2]);
		
		//LETS TEST TILT!!!!!
		System.out.println("Tilting by 5 degrees, hold your horses!");
		dev.setTiltAngle(angle + 5);
		sleep(2000); 
		dev.refreshTiltState();
		System.out.println("New Tilt angle: " + dev.getTiltAngle() + " degrees" );

		
		//We are not calling ctx.shutdown() as it seems to upset liusb
		//turn off LED
		dev.setLed(LedStatus.OFF);
		dev.close();
		System.out.println("Done. LED off, exiting");
		System.exit(0);
		

	}
	//Method for sleeeeeepyyyyy timeeeeeeee (ms)
	private static void sleep(int ms) {
		try {
			Thread.sleep(ms);
		} catch (InterruptedException ignored) {}
	}
}
