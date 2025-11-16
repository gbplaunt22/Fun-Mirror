package mirror;

import org.openkinect.freenect.*;
import javax.swing.*;
	
public class Main {
	public static void main(String[] args) {
		SwingUtilities.invokeLater(() -> {
			
			 JFrame frame = new JFrame("Head Tracker Visualization");
			 frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			 //uses headtracker visualizer panel
			 HeadTrackViz panel = new HeadTrackViz();
			 frame.setContentPane(panel);
			 
			 frame.setSize(800, 600);
			 frame.setLocationRelativeTo(null);
			 frame.setVisible(true);
			
			
			//Kinect setup
			KinectHelpers helper = new KinectHelpers();
			Context ctx = Freenect.createContext();
			Device dev = ctx.openDevice(0);
			
			dev.setLed(LedStatus.BLINK_RED_YELLOW);
		
			
			//Headtracker visualizer
			HeadTrackViz hv = new HeadTrackViz();
			HeadTrackViz.main(args);
		});
	}
}
