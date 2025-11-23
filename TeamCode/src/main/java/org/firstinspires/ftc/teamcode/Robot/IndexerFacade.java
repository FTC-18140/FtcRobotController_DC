package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private ElapsedTime flipTimer = new ElapsedTime();

    // --- Constants ---
    public static final double[] SLOT_ANGLES = {0, 120, 240}; // Angles for slots 0, 1, and 2
    private static final double FLIP_TIME_SECONDS = 0.3; // Time for the flipper to extend and retract

    // --- State Management ---
    public enum State { IDLE, HOMING, SELECTING_BALL, AWAITING_FLIP, FLIPPING, RETRACTING_FLIPPER }
    private State currentState = State.IDLE;

    /** The facade's internal model of what is in each slot. */
    public enum BallState { GREEN, PURPLE, VACANT }
    private BallState[] ballSlots = new BallState[3];
    private int currentTargetSlot = -1;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;

        flipper = new Flipper();
        flipper.init(hwMap, telem);

        turnstile = new Turnstile();
        turnstile.init(hwMap, telem);

        for (int i = 0; i < 3; i++) {
            ballSensors[i] = new BallSensor();
            ballSensors[i].init(hwMap, telem, "ball_sensor_" + i);
            ballSlots[i] = BallState.VACANT;
        }

        currentState = State.HOMING;
        turnstile.home();
    }

    public void setInitialBallStates(BallState[] initialStates) {
        if (initialStates.length == 3) {
            this.ballSlots = initialStates;
        }
    }

    // --- High-Level API & Compatibility Shims ---

    public void selectNextGreenBall() {
        if (currentState == State.IDLE || currentState == State.AWAITING_FLIP) {
            boolean ballFound = false;
            for (int i = 0; i < ballSlots.length && !ballFound; i++) {
                if (ballSlots[i] == BallState.GREEN) {
                    selectSlot(i);
                    ballFound = true;
                }
            }
        }
    }

    public void selectNextPurpleBall() {
        if (currentState == State.IDLE || currentState == State.AWAITING_FLIP) {
            boolean ballFound = false;
            for (int i = 0; i < ballSlots.length && !ballFound; i++) {
                if (ballSlots[i] == BallState.PURPLE) {
                    selectSlot(i);
                    ballFound = true;
                }
            }
        }
    }
    
    public void selectNextEmptySlot() {
        if (currentState == State.IDLE || currentState == State.AWAITING_FLIP) {
            int startSlot = (currentTargetSlot + 1) % 3;
            for (int i = 0; i < 3; i++) {
                int slotToCheck = (startSlot + i) % 3;
                if (ballSlots[slotToCheck] == BallState.VACANT) {
                    selectSlot(slotToCheck);
                    return;
                }
            }
        }
    }

    public void selectSlot(int slot) {
        if ((currentState == State.IDLE || currentState == State.AWAITING_FLIP) && slot >= 0 && slot < 3) {
            currentTargetSlot = slot;
            turnstile.seekToAngle(SLOT_ANGLES[currentTargetSlot]);
            currentState = State.SELECTING_BALL;
        }
    }

    public void flip() {
        if (currentState == State.AWAITING_FLIP && turnstile.isAtTarget()) {
            currentState = State.FLIPPING;
            flipper.extend();
            flipTimer.reset();
        }
    }

    // --- Compatibility Shims for TeleOp ---
    public void unflip() { /* The new state machine handles this automatically */ }
    public void adjustToThird() { selectSlot(2); }
    public void spin(double power) { turnstile.spin(power); }
    public void cycle(int direction) {
        if (direction > 0) selectNextGreenBall();
        else selectNextPurpleBall();
    }
    public State getState() { return currentState; }
    public void setState(State state) { this.currentState = state; }
    public BallState getBallState(int slot) {
        return (slot >= 0 && slot < 3) ? ballSlots[slot] : BallState.VACANT;
    }
    public int getCurrentTargetSlot() { return currentTargetSlot; }


    public void update() {
        flipper.update();
        turnstile.update();
        for (BallSensor sensor : ballSensors) {
            sensor.update();
        }
        updateBallStates();

        switch (currentState) {
            case HOMING:
                if (turnstile.isHomed()) {
                    currentState = State.IDLE;
                }
                break;
            case IDLE: // Waiting for a command
                break;
            case SELECTING_BALL:
                if (turnstile.isAtTarget()) {
                    currentState = State.AWAITING_FLIP;
                }
                break;
            case AWAITING_FLIP: // Ready to receive the flip() command.
                break;
            case FLIPPING:
                if (flipTimer.seconds() > FLIP_TIME_SECONDS) {
                    flipper.retract();
                    currentState = State.RETRACTING_FLIPPER;
                }
                break;
            case RETRACTING_FLIPPER:
                if (flipper.isRetracted()) {
                    currentState = State.IDLE;
                }
                break;
        }

        addTelemetry();
    }

    private void updateBallStates() {
        for (int i = 0; i < 3; i++) {
            BallSensor.BallColor detected = ballSensors[i].getDetectedColor();
            switch (detected) {
                case GREEN: ballSlots[i] = BallState.GREEN; break;
                case PURPLE: ballSlots[i] = BallState.PURPLE; break;
                case NONE: ballSlots[i] = BallState.VACANT; break;
            }
        }
    }

    private void addTelemetry() {
        telemetry.addData("Facade State", currentState.name());
        telemetry.addLine(String.format("Slots: [0]: %s, [1]: %s, [2]: %s",
                ballSlots[0], ballSlots[1], ballSlots[2]));
    }
    
    public boolean isDone() {
        return currentState == State.IDLE || currentState == State.AWAITING_FLIP;
    }
}
