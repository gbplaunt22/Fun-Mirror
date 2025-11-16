package mirror;

import javax.swing.*;
import java.awt.*;
import java.io.IOException;

public class HeadTrackViz extends JPanel {

	private final HeadTracker headTracker;

	public HeadTrackViz() throws IOException {
		// Use the full path to your C executable on the Pi
		headTracker = new HeadTracker("/home/gavin/kinect-headtrack/headtrack");

		// Repaint ~30 times per second
		Timer timer = new Timer(33, e -> repaint());
		timer.start();
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);

		int w = getWidth();
		int h = getHeight();
		int cx = w / 2;
		int cy = h / 2;

		double hx = headTracker.getHeadX();
		double hy = headTracker.getHeadY();
		double hz = headTracker.getHeadZ();

		// Simple scale: pixels per meter in x/y
		double scale = 400.0; // tweak if dot moves too little/too much

		int px = (int) Math.round(cx + hx * scale);
		int py = (int) Math.round(cy - hy * scale); // minus so up is up

		// Draw axes
		g.setColor(Color.LIGHT_GRAY);
		g.drawLine(cx, 0, cx, h);
		g.drawLine(0, cy, w, cy);

		// Draw head dot
		g.setColor(Color.RED);
		int r = 20;
		g.fillOval(px - r / 2, py - r / 2, r, r);

		// Draw text info
		g.setColor(Color.BLACK);
		g.drawString(String.format("HEAD x=%.3f  y=%.3f  z=%.3f", hx, hy, hz), 10, 20);
		g.drawString("Move left/right/up/down in front of Kinect.", 10, 40);
	}

	public static void main(String[] args) {
		SwingUtilities.invokeLater(() -> {
			try {
				JFrame frame = new JFrame("Head Tracker Visualization");
				frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
				frame.setContentPane(new HeadTrackViz());
				frame.setSize(800, 600);
				frame.setLocationRelativeTo(null);
				frame.setVisible(true);
			} catch (IOException e) {
				e.printStackTrace();
				JOptionPane.showMessageDialog(null, "Failed to start headtrack process:\n" + e.getMessage(), "Error",
						JOptionPane.ERROR_MESSAGE);
			}
		});
	}
}
