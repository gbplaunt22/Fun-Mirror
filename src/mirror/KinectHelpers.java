package mirror;

import org.openkinect.freenect.*;

public class KinectHelpers {
	
	//Prints to console if context was created or not
	public void getContextStatusVoid(Context ctx) {
		if(ctx==null) {
			System.err.println(""
					+ "Could not create freenect context!");
		} else {
			System.out.println(""
					+ "freenect context is created!");
		}
	}
	
	//Returns true if context was created
	public boolean getContextStatusBool(Context ctx) {
		if(ctx==null) {
			return false;
		} else {
			return true;
		}
	}
	
	//Returns true if a Kinect device was found
	//by setting ctx.numDevices() to an integer.
	public boolean kinectFound(int x) {
		if(x>0) {
			System.out.println(""
					+ "Kinnect devices found: " + x);
			return true;
		} else {
			System.out.println("No devices found :( ");
			return false;
		}
	}
	
	//Method for sleeeeeepyyyyy timeeeeeeee (ms)
	public void sleep(int ms) {
		try {
			Thread.sleep(ms);
		} catch (InterruptedException ignored) {}
	}
	
	//A helper method to keep tilting until motor stops moving
	public void waitUntilStopped(Device dev, double targetDeg) {
		//tolerance, aka how much error is allowed in degrees
		double TOL = 1.0;
		long start = System.currentTimeMillis();
		
		while(true) {
			dev.refreshTiltState();
			TiltStatus status = dev.getTiltStatus();
			double angle = dev.getTiltAngle();
			
			//Close enough & motor not moving!
			if (status == TiltStatus.STOPPED &&
					Math.abs(angle - targetDeg) < TOL) {
				System.out.println("Tilt done at " 
				+ angle + " degrees.");
				break;
			}
			
			// safety timeout (7 seconds later)
			if (System.currentTimeMillis() - start > 15_000) {
				System.out.println("Timeout waiting for tilt, " +
				"current angle: " + angle);
				break;
			}
			//break between polls
			sleep(50);
			dev.setTiltAngle(targetDeg + 5);
			sleep(2000);//keep trying to set it
		}
	}
	
	//sets the kinnect tilt angle to 0 degrees,
	//returning true when its leveled
	//This is my cutest method :)
	public boolean level(Device dev, double deg) {
		dev.setTiltAngle(deg);
		waitUntilStopped(dev, deg);
		return true;
	}
	
	//A helper method to test tilting to a given angle
	public void testTiltHelper(Device dev, double testAngle) {
		KinectHelpers helper = new KinectHelpers();
		
		//Try to level to 0 degrees
		System.out.println("Tilting to 0 degrees");
		helper.level(dev, 0);
		
		//try to level to TestAngle!
		System.out.println("Tilting to " + testAngle + " degrees.");
		helper.level(dev, testAngle);
		
		//now level back to 0....
		System.out.println("Tilting back to 0 degrees");
		helper.level(dev, 0);
	}
	
	
	
}
