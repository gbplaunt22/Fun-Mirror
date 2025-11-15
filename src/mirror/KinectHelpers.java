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
	public static void sleep(int ms) {
		try {
			Thread.sleep(ms);
		} catch (InterruptedException ignored) {}
	}
	
	
	
}
