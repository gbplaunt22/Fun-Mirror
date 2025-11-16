package mirror;

//import org.openkinect.freenect.*;
import javax.swing.*;
	
public class Main {
	public static void main(String[] args) {
		SwingUtilities.invokeLater(() -> {
			
			 JFrame frame = new JFrame("Head Tracker Visualization");
			 frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			 //uses headtracker visualizer panel
			// frame.setContentPane(panel);
			 
			 frame.setSize(800, 600);
			 frame.setLocationRelativeTo(null);
			 frame.setVisible(true);
			
			
			//Kinect setup
			//Context ctx = Freenect.createContext();
			//Device dev = ctx.openDevice(0);
			
			//dev.setLed(LedStatus.BLINK_RED_YELLOW);
		
		});
	}
}
