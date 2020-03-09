package frc.robot.commands.auto.trajectory.visualization;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.auto.trajectory.Trajectories;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.File;
import java.io.IOException;

/**
 * Note: this file can be a bit finicky to run
 * What generally works is to use WPILib VSCode with the Java extension and select run option next to the main function
 * However sometimes you may receive an error about the WPILib HAL JNI
 * If so, then first run the robot simulation, close it, and rerun the main function
 */

public class TrajectoryVisualizer extends JFrame implements KeyListener {

    private Trajectory trajectory;

    public static void main(String[] args) {
        new TrajectoryVisualizer();
    }

    public TrajectoryVisualizer() {
        Trajectories.setUpTrajectories();
        trajectory = Trajectories.getTrajectory("Top-Trench-2");

        createWindow();
    }

    private void createWindow() {
        DrawPanel drawPanel = new DrawPanel();
        setSize(1134, 600);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setContentPane(drawPanel);
        setFocusable(true);
        setFocusTraversalKeysEnabled(false);
        addKeyListener(this);
        setTitle("Trajectory Visualizer - FRC Team 1257");
        setVisible(true);
    }

    public void keyPressed(KeyEvent e) {

    }

    public void keyReleased(KeyEvent e) {

    }

    public void keyTyped(KeyEvent e) {

    }

    class DrawPanel extends JPanel {

        Image fieldImage;
        int startX = 96, startY = 25;
        int endX = 1040, endY = 514;

        int dx = endX - startX;
        int dy = endY - startY;

        double dx_m = 15.98295;
        double dy_m = 8.21055;

        double x_convPixelToMeter = dx / dx_m;
        double y_convPixelToMeter = dy / dy_m;

        public DrawPanel() {
            File file = new File("src/main/java/frc/robot/commands/auto/trajectory/visualization/2020-Field.png");
            System.out.println(file.getAbsolutePath());
            try {
                fieldImage = ImageIO.read(file);
            }
            catch(IOException e) {
                e.printStackTrace();
            }
        }

        @Override
        public void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D)g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                                 RenderingHints.VALUE_ANTIALIAS_ON);
            setBackground(Color.white);
            g2.drawImage(fieldImage, 0, 0, null);

            double totalTime = trajectory.getTotalTimeSeconds();
            for (double t = 0; t <= totalTime; t += 0.05) {
                g2.setColor(new Color((int) Math.round(t / totalTime * 255), 0, 0));
                Trajectory.State state = trajectory.sample(t);
                Translation2d translation = state.poseMeters.getTranslation();
                g2.fillOval(cx(translation.getX()), cy(translation.getY()), 10, 10);
            }

            g2.setColor(Color.GREEN);
            Translation2d init = trajectory.sample(0).poseMeters.getTranslation();
            g2.fillOval(cx(init.getX()), cy(init.getY()), 10, 10);
            

            g2.setColor(Color.MAGENTA);
            Translation2d end = trajectory.sample(totalTime).poseMeters.getTranslation();
            g2.fillOval(cx(end.getX()), cy(end.getY()), 10, 10);
        }

        private int cx(double x) {
            return (int) Math.round(startX + x * x_convPixelToMeter);
        }

        private int cy(double y) {
            return startY - (int) Math.round(y * y_convPixelToMeter);
        }
    }
}
