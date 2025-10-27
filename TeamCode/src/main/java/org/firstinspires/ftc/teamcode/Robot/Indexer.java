package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.signum;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Indexer {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    CRServo indexer = null;
    DcMotor indexMotor = null;
    Servo flipper = null;
    TouchSensor limitSwitch = null;

    final double CPR = 8192; // REV Through Bore Encoder
    private double indexMotorPos = 0;
    private double indexerMaxPower = 1.0;

    ElapsedTime stateTimer = new ElapsedTime();
    private boolean switchActivated = false;

    public enum positions {
        // Assign a double value to each enum constant
        ZERO(0.0),
        ONE(1.0),
        TWO(2.0);

        // Member variable to hold the value
        public final double target;

        // Enum constructor
        positions(double target) {
            this.target = target;
        }
    }

    public enum State {
        IDLE,
        HOMING,
        LAUNCHING,
        CYCLING,
        STOPPED
    }
    public State currentState = State.IDLE;

    public positions targetIndexPos = positions.ZERO;
    private double flipperPos = 0;
    private boolean homed = false;

    private boolean moving = false;

    public static double p = 0.5, i = 0.1, d = 0.99;
    PIDController angleController;

    /**
     * Initializes the Indexer's hardware components and software utilities.
     * This method should be called once during the OpMode's initialization phase. It retrieves
     * device references from the hardware map, sets motor directions and modes, and
     * initializes the PID controller.
     *
     * @param hwMap The hardware map from the OpMode, used to get device references.
     * @param telem       The telemetry object from the OpMode for displaying debug output.
     */
    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        // Initialize in the stopped state
        currentState = State.STOPPED;

        // Set up the PID controller for the indexer angle
        angleController = new PIDController(p, i, d);
        angleController.setTolerance(0.1); // the amount the controller uses to determine that it has reached the set point.

        try {
            indexer = hardwareMap.crservo.get("indexer");
        } catch (Exception e) {
            telemetry.addData("Continuous Servo \"indexer\" not found", 0);
        }
        try {
            indexMotor = hardwareMap.dcMotor.get("indexerpos");
            indexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            indexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Get the current position of the indexer (should be 0 since we just reset the encoder)
            indexMotorPos = 3 * indexMotor.getCurrentPosition()/CPR;
            // Call this target position ZERO
            targetIndexPos = positions.ZERO;

        } catch (Exception e) {
            telemetry.addData("Motor \"indexerpos\" not found", 0);
        }
        try{
            flipper = hardwareMap.servo.get("flipper");
        } catch (Exception e) {
            telemetry.addData("Servo \"flipper\" not found", 0);
        }
        try{
            limitSwitch = hardwareMap.touchSensor.get("magnet");
        } catch (Exception e) {
            telemetry.addData("Touch Sensor \"magnet\" not found", 0);
        }
    }

    /**
     * Initiates the homing sequence for the indexer.
     * This action is only performed if the system is currently in an IDLE or STOPPED state.
     * It transitions the state machine to HOMING and resets a timer to prevent the sequence
     * from running indefinitely. The core logic is handled by the state machine in the update() method.
     */
    public void homeAndReset(){
        if ( currentState == State.IDLE || currentState == State.STOPPED ) {
            currentState = State.HOMING;
            stateTimer.reset();
        }
    }

    /**
     * Commands the indexer to move to the next logical position in its cycle (e.g., ZERO to ONE).
     * This action is only performed if the system has been homed and is currently in an
     * IDLE or STOPPED state.
     *
     * @param power The maximum power to be used for the movement, clipped between -1.0 and 1.0.
     */
    public void nextThird(double power){
        if ( homed && ( currentState == State.IDLE || currentState == State.STOPPED ) ) {
            cycle(1);
            indexerMaxPower = power;
        }
    }

    /**
     * Commands the indexer to move to the previous logical position in its cycle (e.g., ONE to ZERO).
     * This action is only performed if the system has been homed and is currently in an
     * IDLE or STOPPED state.
     *
     * @param power The maximum power to be used for the movement, clipped between -1.0 and 1.0.
     */
    public void prevThird( double power)
    {
        if ( homed && ( currentState == State.IDLE || currentState == State.STOPPED ) ) {
            cycle(-1);
            indexerMaxPower = power;
        }
    }

    /**
     * A private helper method that calculates the next target position for the indexer
     * based on its current position and a given direction. It then transitions the
     * state machine into the CYCLING state to begin the movement.
     *
     * @param dir The direction to cycle. A positive value moves to the next position
     *            (e.g., ZERO -> ONE), while a negative value moves to the previous one
     *            (e.g., ONE -> ZERO).
     */
    private void cycle(double dir){
        //shifts the target angle by a third

        boolean positive = (dir > 0);
        switch (targetIndexPos)
        {
            case ZERO:
                if (positive) {
                    targetIndexPos = positions.ONE;
                }
                else {
                    targetIndexPos = positions.TWO;
                }
                break;
            case ONE:
                if (positive) {
                    targetIndexPos = positions.TWO;
                }
                else {
                    targetIndexPos = positions.ZERO;
                }
                break;
            case TWO:
                if (positive) {
                    targetIndexPos = positions.ZERO;
                }
                else {
                    targetIndexPos = positions.ONE;
                }
                break;
            default:
                break;
        }
        stateTimer.reset();
        currentState = State.CYCLING;
    }

    /**
     * Creates a Roadrunner Action to command the indexer to cycle its position.* This is a non-blocking action that initiates the cycling process.
     *
     * @param dir   The direction to cycle (e.g., 1 for next, -1 for previous).
     * @param power The power to use for the cycling action.  This is sent to the servo.
     * @return An Action that can be run in a Roadrunner trajectory sequence.
     */
    public Action cycleAction(double dir, double power){
        indexerMaxPower = power;
        return new Action() {
            private boolean firstTime = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if ( firstTime ) {
                    cycle(dir); // update the target to cycle to the next position, set the state to CYCLING,  and reset the timer
                    firstTime = false;
                }
                return currentState == State.CYCLING;
            }
        };
    }

    /**
     * Immediately stops all indexer motor movement by setting the state machine to STOPPED.
     * In the update loop, the STOPPED state will set motor power to zero.
     */
    public void stop(){
        currentState = State.STOPPED;
    }

    /**     * Creates a Roadrunner Action to stop all indexer movement.
     * This is a non-blocking action that transitions the state machine to STOPPED.
     *
     * @return An Action that stops all indexer movement.
     */
    public Action stopAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stop();
                return false;
            }
        };
    }

    /**
     * Initiates the automated launch sequence.
     * This action is only performed if the system has been homed and is in an IDLE or STOPPED state.
     * It transitions the state machine to LAUNCHING, which will handle the timed flipping
     * and unflinching of the flipper servo.
     */
    public void launch()
    {
        if ( homed && ( currentState == State.IDLE || currentState == State.STOPPED ) ) {
            stateTimer.reset();
            currentState = State.LAUNCHING;
        }
    }

    public Action launchAction(){
        return new Action() {
            private boolean firstTime = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if ( firstTime ) {
                    launch();  // sets state to LAUNCHING and resets the timer
                    firstTime = false;
                }
                return currentState == State.LAUNCHING;  // when the is false, the LAUNCHING state is complete
            }
        };
    }


    /**
     * Manually moves the flipper servo to the "up" position (0.65).
     * This action is only performed if the system has been homed and is currently in an
     * IDLE or STOPPED state, ensuring it doesn't interrupt another process.
     */
    public void flip(){
        if ( homed && ( currentState == State.IDLE || currentState == State.STOPPED ) ) {
            flipper.setPosition(0.65);
        }
    }

    /**
     * Manually moves the flipper servo to the "down" or resting position (0.25).
     * This action is only performed if the system has been homed and is currently in an
     * IDLE or STOPPED state, ensuring it doesn't interrupt another process.
     */
    public void unflip(){
        if ( homed && ( currentState == State.IDLE || currentState == State.STOPPED ) ) {
            flipper.setPosition(0.25);
        }
    }

    /**
     * Gets the last known position of the flipper servo. This value is updated
     * during each cycle of the `update()` method.
     *
     * @return The last read position of the flipper servo as a double.
     */
    public double getFlipperPos(){
        return flipperPos;
    }

    /**
     * Checks if the limit switch is currently activated (pressed). The state of the switch
     * is read and this value is updated during each cycle of the `update()` method.
     *
     * @return True if the switch is activated, false otherwise.
     */
    public boolean isSwitchActivated() {
        return switchActivated;
    }

    /**
     * Creates a Roadrunner Action to move the flipper to the up position.
     * This is a non-blocking action that calls the flip() method.
     *
     * @return An Action that commands the flipper to move up.
     */
    private Action flipperUpAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flip();
                return false;
            }
        };
    }

    /**
     * Creates a Roadrunner Action to move the flipper to the down position.
     * This is a non-blocking action that calls the unflip() method.
     *
     * @return An Action that commands the flipper to move down.
     */
    private Action flipperDownAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                unflip();
                return false;
            }
        };
    }

    /**
     * The main "heartbeat" method for the Indexer, designed to be called repeatedly in a loop.
     * It reads all sensor values (motor encoder, limit switch), manages the state machine logic
     * (IDLE, HOMING, CYCLING, etc.), calculates PID outputs for position control, and sends
     * power commands to motors and servos based on the current state.
     */
    public void update(){
        //current ticks / ticks per rotation = rotations since reset
        //multiply by 3 to get get nearest targetPos

        // Calculate the fraction of a rotation from the current position of the motor encoder.
        double fractionalRotation = indexMotor.getCurrentPosition() / CPR;
        // Convert the fractional rotation to a position of the indexer by multiplying by 3.  The
        // Math.floor part of the calculation is a way to extract just the fraction of a rotation
        // and leave off the integer part if the indexer happens to have rotated more than a full
        // rotation.
        indexMotorPos = 3 * (fractionalRotation - Math.floor(fractionalRotation));

        // Once everything is settled, we should move the setPID to init() and not update the
        // values in this method every time
        angleController.setPID(p, i, d);

        telemetry.addData("target Angle: ", targetIndexPos.target);
        telemetry.addData("indexer Angle: ", indexMotorPos);

        // Update the angleController PID with the current position and the target position.  Keep this
        // current so that we can always be ready to control the indexer.
        double indexerPower = angleController.calculate(indexMotorPos, targetIndexPos.target);
        telemetry.addData("Index Servo power: ", indexerPower);

        // Update the limit switch value
        switchActivated = limitSwitch.getValue() > 0;
        telemetry.addData("Magnet: ", switchActivated);

        // Update the flipper position
        flipperPos = flipper.getPosition();

        // Run the main state machine for the indexer
        switch (currentState)
        {
            case IDLE:
                // Do normal things to hold position
                // Not intended for this case to do the heavy lifting of moving to a position, which
                // is why moving = false down below
                indexerPower = Range.clip( indexerPower, -indexerMaxPower, indexerMaxPower );
                indexer.setPower(indexerPower);
                moving = false;
                currentState = State.IDLE;
                break;
            case HOMING:
                if( switchActivated || stateTimer.seconds() > 20 ){
                    indexer.setPower(0);

                    // Reset the encoder
                    indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    indexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    homed = switchActivated; // if we didn't succesfully hit the swtich, but we timed out
                                             // let's note that by making homed = false

                    // Reset the PID controller so that any built up error is cleared.
                    angleController.reset();

                    // Call this target position ZERO.  This is necessary so that we can
                    // cycle properly between positions.
                    targetIndexPos = positions.ZERO;
                    currentState = State.IDLE;
                    moving = false;
                }
                else {
                    moving = true;
                    indexer.setPower(0.05);
                    currentState = State.HOMING;
                }
                break;
            case CYCLING:
                indexerPower = Range.clip( indexerPower, -indexerMaxPower, indexerMaxPower );
                indexer.setPower(indexerPower);
                moving = !angleController.atSetPoint();
                if ( !moving || stateTimer.seconds() > 20) {
                    currentState = State.IDLE;
                }
                else {
                    currentState = State.CYCLING;
                }
                break;
            case LAUNCHING:
                stop();
                if ( stateTimer.seconds() < 0.5 )
                {
                    flip();
                    currentState = State.LAUNCHING;
                }
                else if (stateTimer.seconds() < 1.0 )
                {
                    unflip();
                    currentState = State.LAUNCHING;
                }
                else {
                    currentState = State.IDLE;
                }
                break;
            case STOPPED:
                stop();
                moving = false;
                currentState = State.STOPPED;
                break;
            default:
                break;
        }

    }

    /**
     * Creates a Roadrunner Action that executes the update() method.
     * This allows the indexer's state machine to run within a Roadrunner trajectory.
     *
     * @return An Action that runs the update loop and reports if the indexer is at its target.
     */
    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return !angleController.atSetPoint();
            }
        };
    }
}
