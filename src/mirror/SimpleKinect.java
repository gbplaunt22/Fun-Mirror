package mirror;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;
import org.openkinect.freenect.LedStatus;


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
		System.out.println("Blinking LED for 3 seconds...");
		
		try {
			Thread.sleep(3000);
			//This catch block is so our sleeping thread is not interrupted.
		} catch (InterruptedException ignored) {}
		
		//turn off LED
		dev.setLed(LedStatus.OFF);
		dev.close();
		
		//We are not calling ctx.shutdown() as it seems to upset liusb
		System.out.println("Done. LED off, exiting");
		System.exit(0);
	}
}
