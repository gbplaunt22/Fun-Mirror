package mirror;

import javax.swing.*;
import java.awt.*;
import java.io.IOException;

public class HeadTrackViz extends JPanel {

	private HeadTracker headTracker;
	private boolean trackerOk = false;
	private int paintCount = 0;

	public HeadTrackViz() {
		setBackground(Color.WHITE);

		try {
			headTracker = new HeadTracker("/home/gavin/Fun-Mirror/native/headtrack");
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

		// Fill background explicitly
		g.setColor(Color.WHITE);
		g.fillRect(0, 0, w, h);

		// Draw border so we KNOW this is painting
		g.setColor(Color.GRAY);
		g.drawRect(0, 0, w - 1, h - 1);

		// Draw a BLUE square dead center that should ALWAYS be visible
		g.setColor(Color.BLUE);
		int s = 20;
		g.fillRect(cx - s / 2, cy - s / 2, s, s);

		if (!trackerOk || headTracker == null) {
			g.setColor(Color.RED);
			g.drawString("HeadTracker failed to start", 20, 20);
			return;
		}

		double hx = headTracker.getHeadX();
		double hy = headTracker.getHeadY();
		double hz = headTracker.getHeadZ();

		// Big scaling for obvious motion
		double scale = 500.0;

		int px = (int) Math.round(cx + hx * scale);
		int py = (int) Math.round(cy - hy * scale); // minus so up is up

		// Clamp into window bounds
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

		// GIANT RED DOT at head position
		g.setColor(Color.RED);
		int r = 40;
		g.fillOval(px - r / 2, py - r / 2, r, r);

		// Text readout
		g.setColor(Color.BLACK);
		g.drawString(String.format("HEAD x=%.3f y=%.3f z=%.3f", hx, hy, hz), 20, 20);
		g.drawString(String.format("px=%d py=%d", px, py), 20, 40);

		// Optional: log a few paint cycles so we see coordinates in the console
		paintCount++;
		if (paintCount % 30 == 0) {
			System.out.printf("Paint: hx=%.3f hy=%.3f hz=%.3f -> px=%d py=%d%n", hx, hy, hz, px, py);
		}
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
