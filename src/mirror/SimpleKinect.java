package mirror;

import org.openkinect.freenect.Context;
import org.openkinect.freenect.Device;
import org.openkinect.freenect.Freenect;
import org.openkinect.freenect.LedStatus;
import org.openkinect.freenect.TiltStatus;

public class SimpleKinect extends KinectHelpers{

	public static void main(String[] args) {
		
		KinectHelpers helper = new KinectHelpers();
		
		Context ctx = Freenect.createContext();
		helper.getContextStatusVoid(ctx);
		
		
		//Hello world! #Blink LED
		dev.setLed(LedStatus.BLINK_RED_YELLOW);
		System.out.println("Blinking LED until end of program...");
		System.out.println("Waiting 3 seconds just for the heck of it");
		sleep(3000);


		 
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
		
		dev.close();
		System.out.println("Done. LED off, exiting");
		System.exit(0);

	}
	
	
	
}
