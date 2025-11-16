package mirror;

import javax.swing.*;
import java.awt.*;
import java.io.IOException;

public class HeadTrackViz extends JPanel {

	private HeadTracker headTracker;
	private boolean trackerOk = false;

	public HeadTrackViz() {
		setBackground(Color.WHITE);

		try {
			headTracker = new HeadTracker("/home/gavin/kinect-headtrack/headtrack");
			trackerOk = true;
		} catch (IOException e) {
			e.printStackTrace();
			trackerOk = false;
		}

		// repaint ~30 fps
		Timer timer = new Timer(33, e -> repaint());
		timer.setRepeats(true);
		timer.start();
	}

	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);

		int w = getWidth();
		int h = getHeight();
		int cx = w / 2;
		int cy = h / 2;

		// Draw a border so we know the panel is actually painting
		g.setColor(Color.GRAY);
		g.drawRect(0, 0, w - 1, h - 1);

		if (!trackerOk || headTracker == null) {
			g.setColor(Color.RED);
			g.drawString("HeadTracker failed to start", 20, 20);
			return;
		}

		double hx = headTracker.getHeadX();
		double hy = headTracker.getHeadY();
		double hz = headTracker.getHeadZ();

		// Big scaling so movement is obvious
		double scale = 500.0;

		int px = (int) Math.round(cx + hx * scale);
		int py = (int) Math.round(cy - hy * scale); // minus so up is up

		// Clamp to window bounds
		if (px < 0)
			px = 0;
		if (px >= w)
			px = w - 1;
		if (py < 0)
			py = 0;
		if (py >= h)
			py = h - 1;

		// Crosshairs
		g.setColor(Color.LIGHT_GRAY);
		g.drawLine(cx, 0, cx, h);
		g.drawLine(0, cy, w, cy);

		// GIANT red dot
		g.setColor(Color.RED);
		int r = 40;
		g.fillOval(px - r / 2, py - r / 2, r, r);

		// Text readout
		g.setColor(Color.BLACK);
		g.drawString(String.format("HEAD x=%.3f y=%.3f z=%.3f", hx, hy, hz), 20, 20);
		g.drawString("Move around in front of Kinect; dot should follow.", 20, 40);
	}

	public static void main(String[] args) {
		SwingUtilities.invokeLater(() -> {
			JFrame frame = new JFrame("Head Tracker Visualization");
			frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

			HeadTrackViz panel = new HeadTrackViz();
			frame.setContentPane(panel);
			frame.setSize(800, 600);
			frame.setLocationRelativeTo(null);
			frame.setVisible(true);
		});
	}
}
