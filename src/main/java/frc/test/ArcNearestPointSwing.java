package frc.test;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Path2D;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

public class ArcNearestPointSwing extends JPanel {
    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    private Point clickPoint = null;

    // ---------- Drawing ----------

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;

        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        // Full circle
        g2.setColor(Color.DARK_GRAY);
        g2.drawOval(
                CENTER.x - (int) RADIUS,
                CENTER.y - (int) RADIUS,
                (int) (2 * RADIUS),
                (int) (2 * RADIUS));

        // Arc
        g2.setColor(Color.GREEN);
        Path2D arc = new Path2D.Double();
        boolean first = true;

        int steps = 200;
        for (int i = 0; i <= steps; i++) {
            double t = THETA_MIN + (THETA_MAX - THETA_MIN) * i / steps;
            Point p = pointOnCircle(CENTER, RADIUS, t);
            if (first) {
                arc.moveTo(p.x, p.y);
                first = false;
            } else {
                arc.lineTo(p.x, p.y);
            }
        }
        g2.draw(arc);

        // Center
        g2.setColor(Color.WHITE);
        g2.fillOval(CENTER.x - 3, CENTER.y - 3, 6, 6);

        if (clickPoint != null) {
            // Click point
            g2.setColor(Color.BLUE);
            g2.fillOval(clickPoint.x - 4, clickPoint.y - 4, 8, 8);

            Point closest = nearestPointOnCircleArc(
                    clickPoint, CENTER, RADIUS, THETA_MIN, THETA_MAX);

            if (closest != null) {
                // Closest point
                g2.setColor(Color.RED);
                g2.fillOval(closest.x - 5, closest.y - 5, 10, 10);

                // Connecting line
                g2.setColor(Color.YELLOW);
                g2.drawLine(clickPoint.x, clickPoint.y,
                        closest.x, closest.y);
            }
        }
    }

    // ---------- Setup ----------

    public ArcNearestPointSwing() {
        setPreferredSize(new Dimension(WIDTH, HEIGHT));
        setBackground(Color.BLACK);

        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                clickPoint = e.getPoint();
                repaint();
            }
        });
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Nearest Point on Arc (Swing)");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setContentPane(new ArcNearestPointSwing());
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}
