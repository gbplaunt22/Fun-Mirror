package mirror;

import javax.swing.*;
import java.awt.*;
import java.io.IOException;

public class HeadTrackViz extends JPanel {

    private HeadTracker headTracker;
    private boolean trackerOk = false;
    private int paintCount = 0;

    // smoothed values (for EMA)
    private boolean haveSmooth = false;
    private double smoothX = 0.0;
    private double smoothY = 0.0;
    private double smoothZ = 1.5;
    private static final double ALPHA = 0.25; // 0..1, higher = snappier

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

        // RAW values from C, already in [-1, 1]
        double rawX = headTracker.getHeadX();
        double rawY = headTracker.getHeadY();
        double rawZ = headTracker.getHeadZ();

        // Update smoothed values with EMA
        if (!haveSmooth) {
            smoothX = rawX;
            smoothY = rawY;
            smoothZ = rawZ;
            haveSmooth = true;
        } else {
            smoothX = ALPHA * rawX + (1.0 - ALPHA) * smoothX;
            smoothY = ALPHA * rawY + (1.0 - ALPHA) * smoothY;
            smoothZ = ALPHA * rawZ + (1.0 - ALPHA) * smoothZ;
        }

        double scale = Math.min(w, h) * 0.4;  // 40% of half-size => margin

        // Map RAW to pixels
        int rawPx = (int) Math.round(cx + rawX * scale);
        int rawPy = (int) Math.round(cy - rawY * scale); // -rawY so up is up

        // Map SMOOTH to pixels
        int smoothPx = (int) Math.round(cx + smoothX * scale);
        int smoothPy = (int) Math.round(cy - smoothY * scale);

        // Clamp into window bounds
        rawPx    = Math.max(0, Math.min(rawPx,    w - 1));
        rawPy    = Math.max(0, Math.min(rawPy,    h - 1));
        smoothPx = Math.max(0, Math.min(smoothPx, w - 1));
        smoothPy = Math.max(0, Math.min(smoothPy, h - 1));

        // Crosshairs
        g.setColor(Color.LIGHT_GRAY);
        g.drawLine(cx, 0, cx, h);
        g.drawLine(0, cy, w, cy);

        // RAW dot – small GREEN
        g.setColor(Color.GREEN.darker());
        int rRaw = 20;
        g.fillOval(rawPx - rRaw / 2, rawPy - rRaw / 2, rRaw, rRaw);

        // SMOOTH dot – big RED
        g.setColor(Color.RED);
        int rSmooth = 40;
        g.fillOval(smoothPx - rSmooth / 2, smoothPy - rSmooth / 2, rSmooth, rSmooth);

        // Text readout
        g.setColor(Color.BLACK);
        g.drawString(String.format("RAW    x=%.3f y=%.3f z=%.3f", rawX, rawY, rawZ), 20, 20);
        g.drawString(String.format("SMOOTH x=%.3f y=%.3f z=%.3f", smoothX, smoothY, smoothZ), 20, 40);
        g.drawString(String.format("raw px=%d py=%d  smooth px=%d py=%d",
                rawPx, rawPy, smoothPx, smoothPy), 20, 60);

        // Console debug every ~1s
        paintCount++;
        if (paintCount % 30 == 0) {
            System.out.printf(
                    "Paint: RAW(%.3f,%.3f,%.3f) -> SMOOTH(%.3f,%.3f,%.3f) -> raw(%d,%d) smooth(%d,%d)%n",
                    rawX, rawY, rawZ,
                    smoothX, smoothY, smoothZ,
                    rawPx, rawPy, smoothPx, smoothPy
            );
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Head Tracker RAW vs SMOOTH");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            HeadTrackViz panel = new HeadTrackViz();
            frame.setContentPane(panel);
            frame.setSize(800, 600);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}
