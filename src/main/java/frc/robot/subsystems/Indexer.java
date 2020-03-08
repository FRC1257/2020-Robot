package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import static frc.robot.Constants.ElectricalLayout;
import static frc.robot.Constants.NEO_550_CURRENT_LIMIT;

/**
 * Subsystem to handle the conveyor belt and the stop mechanism
 * 
 * - Utilizes two NEO 550 motors attached to the conveyor
 * 
 * - Utilizes one NEO 550 motor attached to the stop mechanism
 * 
 * - Uses PID to move the Indexer to specific setpoints
 */

public class Indexer extends SnailSubsystem {

    private final CANSparkMax conveyorMotorFrontBottom;
    private final CANSparkMax conveyorMotorFrontTop;
    private final CANSparkMax conveyorMotorBack;
    private final CANSparkMax stopMotor;

    private double topSpeed;

    private DigitalInput bottomFrontBreakbeam;
    private DigitalInput bottomBackBreakbeam;
    private ColorSensorV3 colorSensor;
    private MedianFilter filter;
    private double lastFilteredDist;
    private boolean override;

    private Notifier looper;

    /**
     * NEUTRAL - The position of each of the power cells is maintained
     * 
     * PID - Using PID control to go to and maintain a specific position
     * 
     * SHOOTING - The power cells are put into the shooter
     * 
     * RAISING - The power cells are raised higher into the indexer
     * 
     * LOWERING - THe power cells are lowered from the indexer to the intake
     */
    
    public enum State {
        NEUTRAL,
        SHOOTING, // manual
        RAISING,  // manual
        LOWERING, // manual
        BACK,
        CELL_RAISING,   // cell was just detected and the indexer should now be rising up
        CELL_RETURNING, // cell has just left breakbeam and is being put back in
        CELL_NUDGING    // cell has reentered breakbeam and is being nudged up
    }
    State state = State.NEUTRAL;

    public Indexer() {
        conveyorMotorFrontBottom = new CANSparkMax(ElectricalLayout.INDEXER_CONVEYOR_FRONT_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorFrontBottom.restoreFactoryDefaults();
        conveyorMotorFrontBottom.setIdleMode(IdleMode.kBrake);
        conveyorMotorFrontBottom.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorMotorFrontBottom.setInverted(true);

        conveyorMotorFrontTop = new CANSparkMax(ElectricalLayout.INDEXER_CONVEYOR_FRONT_TOP_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorFrontTop.restoreFactoryDefaults();
        conveyorMotorFrontTop.setIdleMode(IdleMode.kBrake);
        conveyorMotorFrontTop.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorMotorFrontTop.setInverted(true);

        conveyorMotorBack = new CANSparkMax(ElectricalLayout.INDEXER_CONVEYOR_BACK_MOTOR_ID, MotorType.kBrushless);
        conveyorMotorBack.restoreFactoryDefaults();
        conveyorMotorBack.setIdleMode(IdleMode.kBrake);
        conveyorMotorBack.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
        conveyorMotorBack.follow(conveyorMotorFrontTop, true);

        stopMotor = new CANSparkMax(ElectricalLayout.INDEXER_STOP_MOTOR_ID, MotorType.kBrushless);
        stopMotor.restoreFactoryDefaults();
        stopMotor.setIdleMode(IdleMode.kBrake);
        stopMotor.setSmartCurrentLimit(NEO_550_CURRENT_LIMIT);

        bottomFrontBreakbeam = new DigitalInput(ElectricalLayout.INDEXER_BOTTOM_BREAKBEAM_FRONT_ID);
        bottomBackBreakbeam = new DigitalInput(ElectricalLayout.INDEXER_BOTTOM_BREAKBEAM_BACK_ID);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        filter = new MedianFilter(Constants.Indexer.INDEXER_TOP_SENSOR_NUM_MED);

        looper = new Notifier(this::updateNotifier);
        looper.startPeriodic(Constants.Indexer.INDEXER_LOOPER_PERIOD);

        reset();
    }

    public void reset() {
        state = State.NEUTRAL;
        lastFilteredDist = 0;
        topSpeed = 0;
    }

    /**
     * Update motor outputs according to the current state
     * This method is called within a notifier to achieve a much faster refresh rate and in a separate thread
     */
    private void updateNotifier() {
        updateDistance();

        switch(state) {
            case NEUTRAL:
                conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_NEUTRAL_SPEED);
                stopMotor.set(Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED);

                // automatically index once it sees a ball
                // if (ballAtBot()) {
                //     state = State.CELL_RAISING;
                // }
                break;
            case SHOOTING:
                conveyorMotorFrontTop.set(-Constants.Indexer.INDEXER_CONVEYOR_SHOOT_SPEED);
                conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_SHOOT_SPEED);
                stopMotor.set(Constants.Indexer.INDEXER_STOP_SHOOT_SPEED);
                break;
            case RAISING:
                if (canMove() || override) {
                    conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_RAISE_SPEED);
                }
                stopMotor.set(Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED);
                break;
            case LOWERING:
                conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_LOWER_SPEED);
                stopMotor.set(Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED);
                break;
            case BACK:
                conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_NEUTRAL_SPEED);
                stopMotor.set(Constants.Indexer.INDEXER_STOP_BACK_SPEED);
                break;

            // automatic indexing
            case CELL_RAISING:
                if (!ballAtBot()) {
                    state = State.CELL_RETURNING;
                }
                else {
                    conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_RAISE_SPEED);
                    stopMotor.set(Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED);
                }
                if(ballAtTop()) {
                    state = State.NEUTRAL;
                }
                break;
            case CELL_RETURNING:
                if (ballAtBot()) {
                    state = State.CELL_NUDGING;
                }
                else {
                    conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_RETURN_SPEED);
                    stopMotor.set(Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED);
                }
                if(ballAtTop()) {
                    state = State.NEUTRAL;
                }
                break;
            case CELL_NUDGING:
                if (!ballAtBot()) {
                    state = State.NEUTRAL;
                }
                else {
                    conveyorMotorFrontBottom.set(Constants.Indexer.INDEXER_CONVEYOR_NUDGE_SPEED);
                }
                if(ballAtTop()) {
                    state = State.NEUTRAL;
                }
                break;
        }

        if(state != State.SHOOTING) {
            if(topSpeed > 0 && ballAtTop()) topSpeed = 0;
            conveyorMotorFrontTop.set(topSpeed);
        }
    }

    @Override
    public void periodic() {

    }

    public void setTopSpeed(double topSpeed) {
        this.topSpeed = topSpeed;
    }

    /**
     * Puts relevant values to Smart Dashboard
     */
    @Override
    public void outputValues() {
        SmartDashboard.putBooleanArray("Indexer Booleans (Front BB, Back BB, Move)", new boolean[] {
            bottomFrontBreakbeam.get(),
            bottomBackBreakbeam.get(),
            canMove()
        });
        
        SmartDashboard.putNumberArray("Indexer Currents (Front Top, Front Bottom, Back, Stop)", new double[] {
            conveyorMotorFrontBottom.getOutputCurrent(),
            conveyorMotorFrontTop.getOutputCurrent(),
            conveyorMotorBack.getOutputCurrent(),
            stopMotor.getOutputCurrent()
        });

        if(SmartDashboard.getBoolean("Testing", false)) {
            SmartDashboard.putString("Indexer State", state.name());
            SmartDashboard.putNumber("Indexer Color Sensor", lastFilteredDist);
        }
    }

    /**
     * Puts values that can be changed into Smart Dashboard
     */
    @Override
    public void setConstantTuning() {
        SmartDashboard.putNumber("Indexer Stop Shoot Speed", Constants.Indexer.INDEXER_STOP_SHOOT_SPEED);
        SmartDashboard.putNumber("Indexer Stop Neutral Speed", Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED);

        SmartDashboard.putNumber("Indexer Conveyor Raise Speed", Constants.Indexer.INDEXER_CONVEYOR_RAISE_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Return Speed", Constants.Indexer.INDEXER_CONVEYOR_RETURN_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Nudge Speed", Constants.Indexer.INDEXER_CONVEYOR_NUDGE_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Lower Speed", Constants.Indexer.INDEXER_CONVEYOR_LOWER_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Shoot Speed", Constants.Indexer.INDEXER_CONVEYOR_SHOOT_SPEED);
        SmartDashboard.putNumber("Indexer Conveyor Neutral Speed", Constants.Indexer.INDEXER_CONVEYOR_NEUTRAL_SPEED);
    }

    /**
     * Gets values that can be changed
     */
    @Override
    public void getConstantTuning() {

        Constants.Indexer.INDEXER_STOP_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Stop Shoot Speed", Constants.Indexer.INDEXER_STOP_SHOOT_SPEED);
        Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Stop Neutral Speed", Constants.Indexer.INDEXER_STOP_NEUTRAL_SPEED);

        Constants.Indexer.INDEXER_CONVEYOR_SHOOT_SPEED = SmartDashboard.getNumber("Indexer Conveyor Shoot Speed", Constants.Indexer.INDEXER_CONVEYOR_SHOOT_SPEED);
        Constants.Indexer.INDEXER_CONVEYOR_RETURN_SPEED = SmartDashboard.getNumber("Indexer Conveyor Return Speed", Constants.Indexer.INDEXER_CONVEYOR_RETURN_SPEED);
        Constants.Indexer.INDEXER_CONVEYOR_NUDGE_SPEED = SmartDashboard.getNumber("Indexer Conveyor Nudge Speed", Constants.Indexer.INDEXER_CONVEYOR_NUDGE_SPEED);
        Constants.Indexer.INDEXER_CONVEYOR_RAISE_SPEED = SmartDashboard.getNumber("Indexer Conveyor Raise Speed", Constants.Indexer.INDEXER_CONVEYOR_RAISE_SPEED);
        Constants.Indexer.INDEXER_CONVEYOR_LOWER_SPEED = SmartDashboard.getNumber("Indexer Conveyor Lower Speed", Constants.Indexer.INDEXER_CONVEYOR_LOWER_SPEED);
        Constants.Indexer.INDEXER_CONVEYOR_NEUTRAL_SPEED = SmartDashboard.getNumber("Indexer Conveyor Neutral Speed", Constants.Indexer.INDEXER_CONVEYOR_NEUTRAL_SPEED);
    }
    
    /**
    * Changes state to neutral if not automatically indexing at the moment
    */
    public void neutral() {
        if(state != State.CELL_RAISING && state != State.CELL_RETURNING && state != State.CELL_NUDGING) {
            state = State.NEUTRAL;
        }
    }

    /**
    * Changes state to shooting
    */
    public void shoot() {
        state = State.SHOOTING;
    }

    /**
    * Changes state to lowering
    */
    public void lower() {
        state = State.LOWERING;
    }

    /**
    * Changes state to raising
    */
    public void raise() {
        state = State.RAISING;
    }

    /**
    * Changes state to back
    */
    public void back() {
        state = State.BACK;
    }

    /**
     * Updates the filter for the color sensor distance
     */
    private void updateDistance() {
        lastFilteredDist = filter.calculate(colorSensor.getProximity());
    }

    /**
     * Returns whether or not the color sensor detects a ball at the top
     */
    public boolean ballAtTop() {
        return lastFilteredDist > Constants.Indexer.INDEXER_TOP_BALL_PROXIMITY;
    }

    /**
     * Returns whether or not the breakbeam sensors detect a ball at the bottom
     */
    public boolean ballAtBot() {
        return !bottomBackBreakbeam.get();// || !bottomFrontBreakbeam.get();
    }

    /**
     * Returns whether or not the indexer is free to move
     */
    public boolean canMove() {
        return ballAtBot() && !ballAtTop();
    }

    /**
     * Sets up the override for the sensors
     */
    public void setOverride(boolean override) {
        this.override = override;
    }

    /**
    * Returns the state
    */
    public State getState() {
        return state;
    }
}
