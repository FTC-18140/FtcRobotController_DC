package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The IndexerFacade is the high-level controller for the entire indexing and loading mechanism.
 * It coordinates the Turnstile, Flipper, and BallSensors to perform complex actions safely.
 */
public class IndexerFacade {

    // --- Sub-Components ---
    private Flipper flipper;
    private Turnstile turnstile;
    private BallSensor[] ballSensors = new BallSensor[3];
    private Telemetry telemetry;

    // --- Constants ---
    public static final double[] SLOT_ANGLES = {0, 120, 240}; // Angles for slots 0, 1, and 2

    // --- State Management ---
    public enum State { IDLE, HOMING, SELECTING_BALL, AWAITING_FLIP, FLIPPING }
    private State currentState = State.IDLE;

    /** The facade's internal model of what is in each slot. */
    public enum BallState { GREEN, PURPLE, VACANT }
    private BallState[] ballSlots = new BallState[3];
    private int currentTargetSlot = -1;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;

        // Initialize all sub-components
        flipper = new Flipper();
        flipper.init(hwMap, telem);

        turnstile = new Turnstile();
        turnstile.init(hwMap, telem);

        for (int i = 0; i < 3; i++) {
            ballSensors[i] = new BallSensor();
            // Assumes sensors are named "ball_sensor_0", "ball_sensor_1", etc. in config
            ballSensors[i].init(hwMap, telem, "ball_sensor_" + i);
            ballSlots[i] = BallState.VACANT;
        }

        // Begin the homing sequence immediately upon initialization
        currentState = State.HOMING;
        turnstile.home();
    }

    /**
     * Allows the TeleOp or Auto to declare the starting state of the balls.
     * Call this in your init() method after the facade.init().
     */
    public void setInitialBallStates(BallState[] initialStates) {
        if (initialStates.length == 3) {
            this.ballSlots = initialStates;
        }
    }

    // --- High-Level API (to be called from TeleOp/Auto) ---

    public void selectNextGreenBall() {
        // Refactored to have a single exit point
        if (currentState == State.IDLE || currentState == State.AWAITING_FLIP) {
            boolean ballFound = false;
            for (int i = 0; i < ballSlots.length && !ballFound; i++) {
                if (ballSlots[i] == BallState.GREEN) {
                    currentTargetSlot = i;
                    turnstile.seekToAngle(SLOT_ANGLES[currentTargetSlot]);
                    currentState = State.SELECTING_BALL;
                    ballFound = true; // Stop the loop after finding the first green ball
                }
            }
        }
    }

    public void flip() {
        // Logic for the flipping sequence will go here
    }

    /**
     * The main state machine for the facade. Call this once per loop.
     */
    public void update() {
        // 1. Update all sub-components to read their hardware and run their own state machines.
        flipper.update();
        turnstile.update();
        for (BallSensor sensor : ballSensors) {
            sensor.update();
        }

        // 2. Update the facade's internal model of the world based on sensor data.
        updateBallStates();

        // 3. Run the facade's high-level state machine.
        switch (currentState) {
            case HOMING:
                if (turnstile.isHomed()) {
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
                // Waiting for a command like selectNextGreenBall()
                break;

            case SELECTING_BALL:
                // Wait for the turnstile to reach the selected ball's slot.
                if (turnstile.isAtTarget()) {
                    currentState = State.AWAITING_FLIP;
                }
                break;

            case AWAITING_FLIP:
                // Ready to receive the flip() command.
                break;

            case FLIPPING:
                // Logic to manage the flip and retract sequence
                break;
        }

        // 4. Add Telemetry for debugging
        addTelemetry();
    }

    /**
     * Updates the internal 'ballSlots' array based on the latest readings from the BallSensors.
     */
    private void updateBallStates() {
        for (int i = 0; i < 3; i++) {
            BallSensor.BallColor detected = ballSensors[i].getDetectedColor();
            switch (detected) {
                case GREEN:
                    ballSlots[i] = BallState.GREEN;
                    break;
                case PURPLE:
                    ballSlots[i] = BallState.PURPLE;
                    break;
                case NONE:
                    ballSlots[i] = BallState.VACANT;
                    break;
            }
        }
    }

    private void addTelemetry() {
        telemetry.addData("Facade State", currentState.name());
        telemetry.addLine(String.format("Slots: [0]: %s, [1]: %s, [2]: %s",
                ballSlots[0], ballSlots[1], ballSlots[2]));
    }
}
