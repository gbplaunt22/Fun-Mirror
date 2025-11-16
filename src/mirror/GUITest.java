package mirror;

import javax.swing.*;
import java.awt.*;

public class GUITest {
    public static void main(String[] args) {
        System.out.println("DISPLAY=" + System.getenv("DISPLAY"));

        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("GUITest");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

            JPanel panel = new JPanel() {
                @Override
                protected void paintComponent(Graphics g) {
                    super.paintComponent(g);
                    int w = getWidth();
                    int h = getHeight();
                    System.out.println("paint w=" + w + " h=" + h);

                    g.setColor(Color.WHITE);
                    g.fillRect(0, 0, w, h);

                    g.setColor(Color.BLUE);
                    g.fillRect(w/2 - 50, h/2 - 50, 100, 100);

                    g.setColor(Color.BLACK);
                    g.drawString("HELLO FROM SWING", 20, 20);
                }
            };

            frame.setContentPane(panel);
            frame.setSize(600, 400);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}

