package inkly.integration;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Arc;

public class ArcNearestPointSwing extends JPanel {
    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    private Point clickPoint = null;

    private Arc arc = new Arc(new Translation2d(400, 300), 200, Rotation2d.kZero, Rotation2d.kCW_90deg);

    // ---------- Drawing ----------

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;

        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);

        // use the arc object instead
        g2.setColor(Color.DARK_GRAY);
        g2.drawOval(
                (int) (arc.getCenter().getX() - arc.getRadius()),
                (int) (arc.getCenter().getY() - arc.getRadius()),
                (int) (2 * arc.getRadius()),
                (int) (2 * arc.getRadius()));

        // Arc
        g2.setColor(Color.GREEN);
        Path2D arcPath = new Path2D.Double();

        int steps = 200;
        for (int i = 0; i <= steps; i++) {
            Rotation2d t = arc.getStart().interpolate(arc.getEnd(), (double) i / steps);
            Translation2d p = new Translation2d(arc.getRadius(), t).plus(arc.getCenter());
            if (i == 0) {
                arcPath.moveTo(p.getX(), p.getY());
            } else {
                arcPath.lineTo(p.getX(), p.getY());
            }
        }
        g2.draw(arcPath);

        // Center
        g2.setColor(Color.WHITE);
        g2.fillOval((int) arc.getCenter().getX() - 3, (int) arc.getCenter().getY() - 3, 6, 6);

        if (clickPoint == null) {
            return;
        }

        // Click point
        g2.setColor(Color.BLUE);
        g2.fillOval(clickPoint.x - 4, clickPoint.y - 4, 8, 8);

        Translation2d closest = arc.nearestPointOnArc(new Translation2d(clickPoint.x, clickPoint.y));

        // Closest point
        g2.setColor(Color.RED);
        g2.fillOval((int) closest.getX() - 4, (int) closest.getY() - 4, 8, 8);
        // Connecting line
        g2.setColor(Color.YELLOW);
        g2.drawLine(clickPoint.x, clickPoint.y,
                (int) closest.getX(), (int) closest.getY());
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
