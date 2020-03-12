package frc.robot.commands.auto.segmented.visualization;

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

public class SegmentVisualizer extends JFrame implements KeyListener {

    private static enum ActionType {
        DISTANCE,
        ANGLE
    }

    private static class Action {
        ActionType type;
        double value;

        private Action(ActionType type, double value) {
            this.type = type;
            this.value = value;
        }

        static Action parse(String string) {
            char typeCharacter = string.charAt(0);

            ActionType type = null;
            double value;
            switch(typeCharacter) {
                case 'D':
                    type = ActionType.DISTANCE;
                    break;
                case 'A':
                    type = ActionType.ANGLE;
                    break;
            }

            value = Double.parseDouble(string.substring(2, string.length() - 1));

            return new Action(type, value);
        }
    }

    // TODO Fix these >_<
    String topPath = "T,D(-2.603),D(5.652)";
    String middlePath = "M,A(36.6),D(-2.85),A(-36.6),D(-1.0),D(5.652)";
    String bottomPath = "B,A(54.9),D(-3.97),A(-54.9),D(-1.0),D(5.652)";

    double robotStartX, robotStartY, robotStartA;

    Action[] actions;
    public static void main(String[] args) {
        new SegmentVisualizer();
    }

    public SegmentVisualizer() {
        parsePath(middlePath);
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
        setTitle("Segmented Trajectory Visualizer - FRC Team 1257");
        setVisible(true);
    }

    public void parsePath(String path) {
        String[] tokens = path.split(",");
        actions = new Action[tokens.length - 1];
        for(int i = 1; i < tokens.length; i++) {
            actions[i - 1] = Action.parse(tokens[i]);
        }

        String start = tokens[0];
        Trajectories.setUpTrajectories();
        String trajectory = "";
        switch(start) {
            case "T":
                trajectory = "Top-Power";
                break;
            case "M":
                trajectory = "Mid-Power";
                break;
            case "B":
                trajectory = "Bot-Power";
                break;
        }

        Trajectory t = Trajectories.getTrajectory(trajectory);
        robotStartX = t.getInitialPose().getTranslation().getX();
        robotStartY = t.getInitialPose().getTranslation().getY();
        robotStartA = t.getInitialPose().getRotation().getRadians();
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
            
            double lastX = robotStartX;
            double lastY = robotStartY;
            double lastA = robotStartA;

            int count = 0;
            for(Action action : actions) {
                switch(action.type) {
                    case DISTANCE:
                        double newX = lastX + Math.cos(lastA) * action.value;
                        double newY = lastY + Math.sin(lastA) * action.value;

                        g2.setColor(new Color((int) Math.round((double) count * 255 / actions.length), 255, 255));
                        g2.drawLine(cx(lastX), cy(lastY), cx(newX), cy(newY));

                        lastX = newX;
                        lastY = newY;
                    break;
                    case ANGLE:
                        lastA += action.value;
                    break;
                }
                count++;
            }
        }

        private int cx(double x) {
            return (int) Math.round(startX + x * x_convPixelToMeter);
        }

        private int cy(double y) {
            return startY - (int) Math.round(y * y_convPixelToMeter);
        }
    }
}
