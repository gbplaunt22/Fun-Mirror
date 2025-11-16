package mirror;

import javax.swing.*;
import java.awt.*;

public class SwingTest extends JPanel {

	private int t = 0;

	public SwingTest() {
		setBackground(Color.WHITE);

		// repaint ~30fps
		Timer timer = new Timer(33, e -> {
			t++;
			repaint();
		});
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

		// Fill background
		g.setColor(Color.WHITE);
		g.fillRect(0, 0, w, h);

		// Border
		g.setColor(Color.GRAY);
		g.drawRect(0, 0, w - 1, h - 1);

		// Crosshair
		g.setColor(Color.LIGHT_GRAY);
		g.drawLine(cx, 0, cx, h);
		g.drawLine(0, cy, w, cy);

		// Simple horizontal motion: dot moves left-right
		int radius = 40;
		int amplitude = w / 3;
		int px = (int) (cx + Math.sin(t / 20.0) * amplitude);
		int py = cy;

		g.setColor(Color.RED);
		g.fillOval(px - radius / 2, py - radius / 2, radius, radius);

		// Text
		g.setColor(Color.BLACK);
		g.drawString("SwingTest: moving dot demo", 20, 20);
	}

	public static void main(String[] args) {
		SwingUtilities.invokeLater(() -> {
			JFrame frame = new JFrame("SwingTest");
			frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			SwingTest panel = new SwingTest();
			frame.setContentPane(panel);
			frame.setSize(800, 600);
			frame.setLocationRelativeTo(null);
			frame.setVisible(true);
		});
	}
}
