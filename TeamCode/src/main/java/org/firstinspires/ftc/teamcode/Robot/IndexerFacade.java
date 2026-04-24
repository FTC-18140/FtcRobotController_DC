package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

/**
 * The IndexerFacade is the high-level controller for the entire indexing and loading mechanism.
 * It coordinates the Turnstile, Flipper, and BallSensors to perform complex actions safely.
 */
@Config
public class IndexerFacade {


    // --- Sub-Components ---
//    private Flipper flipper = null;
    private Turnstile turnstile = null;
    private BallSensor[] ballSensors = new BallSensor[6];
    private BeamBreaker beamBreak = new BeamBreaker();
    private Telemetry telemetry = null;
    private final ElapsedTime flipTimer = new ElapsedTime();

    // --- Constants ---
    public static final double[] SLOT_ANGLES = {240.0, 120.0, (double) 0}; // Angles for slots 0, 1, and 2

    public int beamBreakCounter = 0;

    public static boolean TELEM = true;
    private boolean updated = false;
    public static int TEST_TAG_ID = 21;

    // --- State Management ---
    public enum State {IDLE, HOMING, SELECTING_BALL, INTAKE, AWAITING_LAUNCH, LAUNCHING}

    private State currentState = State.IDLE;
    private boolean isIntaking = false;


    /**
     * The facade's internal model of what is in each slot.
     */
    public enum BallState {GREEN, PURPLE, VACANT, ALL}

    private BallState[] ballSlots = new BallState[3];
    private int currentTargetSlot = 2;
    private int ballNumber = 0;
    private BallState previousBallStateIntake = BallState.VACANT;


    // --- Auto-Sequence Management ---
    private List<BallState> shotSequence = null;
    private int sequence_id = -1;
    private boolean sequenceStarted = false;
    private int sequenceIndex = -1;
    private boolean[] slots_fired = new boolean[3];

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;

//        flipper = new Flipper();
//        flipper.init(hwMap, telem);

        turnstile = new Turnstile();
        turnstile.init(hwMap, telem);

        for (int i = 0; 6 > i; i++) {
            ballSensors[i] = new BallSensor();
            ballSensors[i].init(hwMap, telem, "color" + i, i);
        }
        for (int i = 0; 3 > i; i++) {
            ballSlots[i] = BallState.VACANT;
            slots_fired[i] = false;
        }

        beamBreak.init(hwMap, telem);

        updateBallSensors();
        updateBallStates();
        updateBallCount();

        currentState = State.IDLE;
        //turnstile.home();
    }

//    public void flipOverride(boolean up) {
//        if (up) {
//            flipper.extend();
//        } else {
//            flipper.retract();
//        }
//    }

    public void setInitialBallStates(BallState[] initialStates) {
        if (3 == initialStates.length) {
            ballSlots = initialStates;
        }
    }

    // --- High-Level API & Compatibility Shims ---

    /**
     * (Private Helper) Executes the next step in the planned shot sequence.
     * This method is the core of the autonomous firing logic. It finds the next required ball
     * from the sequence, locates it in one of the physical slots, and begins rotating the
     * turnstile to that slot. It critically modifies the internal `ballSlots` model to prevent
     * the same ball from being used twice to fulfill the sequence.
     */
    public boolean prepSequence() {
        if (null == shotSequence) planShotSequence(sequence_id);
        if (!shotSequence.contains(BallState.GREEN)) return true;

        int index = (2 - shotSequence.indexOf(BallState.GREEN)) % 3;
        int green_pos;
        for (int i = 0; 3 > i; i++) {
            if (BallState.GREEN == getBallState(i)) {
                green_pos = i;
                int angle = (currentTargetSlot + (index - green_pos));
                turnstile.seekToAngle(SLOT_ANGLES[Math.floorMod(angle, 3)]);
                setCurrentState(State.SELECTING_BALL);
                return true;
            }
        }
        return true;
    }


    public void launchAllInIndexer() {
        if (canLaunchAll()) {
            turnstile.launchSlots(3);
            setCurrentState(State.LAUNCHING);
            for (int i = 0; 3 > i; i++) {
                ballSlots[i] = BallState.VACANT;
            }
        }

    }

    private boolean canLaunchAll() {
        return State.IDLE == currentState || State.AWAITING_LAUNCH == currentState || State.LAUNCHING == currentState || State.SELECTING_BALL == currentState;
    }

    public Action runCurrentSequenceAction() {
        return new Action() {
            boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    for (int i = 0; 3 > i; i++) {
                        slots_fired[i] = false;
                    }
                    started = true;
                }

                return isInSequence();
            }
        };
    }

    public Action homeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                adjustToThird();
                return !turnstile.isHomed();
            }
        };
    }

    public void cancelSequence() {
        sequenceStarted = false;
        shotSequence = null;
        sequenceIndex = -1;
        for (int i = 0; 3 > i; i++) {
            slots_fired[i] = false;
        }
        if (State.IDLE != currentState) {
            currentState = State.IDLE;
        }
    }

    public void rotateBallStates(int iterations) {
        for (int i = 0; i < iterations; i++) {
            BallState temp = ballSlots[2];
            ballSlots[2] = ballSlots[1];
            ballSlots[1] = ballSlots[0];
            ballSlots[0] = temp;

            boolean slots = slots_fired[2];
            slots_fired[2] = slots_fired[1];
            slots_fired[1] = slots_fired[0];
            slots_fired[0] = slots;
        }
    }

    /**
     * Indexes a color i
     *
     * @param ballState the color we want to select
     * @return whether it found that color
     */
    public boolean selectNextSlot(BallState ballState) {
        // Refactored to have a single exit point
        boolean slotFound = false;
        if (State.IDLE == currentState || State.AWAITING_LAUNCH == currentState) {
            int startSlot = currentTargetSlot;

            for (int i = 3; 0 < i && !slotFound; i--) {
                updateBallSensors();
                updateBallStates();
                int slotToCheck = (startSlot + i) % 3;
                if (ballSlots[slotToCheck] == ballState || (BallState.ALL == ballState && BallState.VACANT != ballSlots[slotToCheck])) {

                    currentTargetSlot = slotToCheck;
                    turnstile.seekToAngle(SLOT_ANGLES[slotToCheck]);
                    beamBreakCounter = 0;
                    currentState = State.SELECTING_BALL;
                    slotFound = true;
                }
            }
        }
        return slotFound;
    }

    public boolean readyNextIntakeSlot(BallState ballState) {
        // Refactored to have a single exit point
        boolean slotFound = false;
        if (State.IDLE == currentState || State.AWAITING_LAUNCH == currentState || isIntaking) {
            int startSlot = 0;

            updateBallSensors();
            updateBallStates();
            for (int i = 0; i < 3 && !slotFound; i++) {

                int slotToCheck = (i) % 3;

                if (ballSlots[slotToCheck] == ballState) {

                    int slot = Math.floorMod(currentTargetSlot + (3 - slotToCheck), 3);
                    rotateBallStates(3 - slotToCheck);
                    currentTargetSlot = slot;
                    turnstile.seekToAngle(SLOT_ANGLES[slot]);
                    beamBreakCounter = 0;
                    currentState = State.SELECTING_BALL;
                    slotFound = true;
                }
            }
        }
        return slotFound;
    }


    /**
     * Commands the turnstile to rotate to a specific slot.
     * This is a foundational public method for the Indexer, used by both manual controls (like
     * cycling to the next slot) and by the autonomous shot planner. It will only execute if the
     * state machine is in a safe state (IDLE or AWAITING_FLIP) to prevent conflicting commands.
     *
     * @param slot The index of the target slot (0, 1, or 2).
     */
    public boolean selectSlot(int slot) {
        if (canSelectSlot(slot)) {
            rotateBallStates((slot - currentTargetSlot + 3) % 3);
            currentTargetSlot = slot;
            turnstile.seekToAngle(SLOT_ANGLES[currentTargetSlot]);
            beamBreakCounter = 0;
            currentState = State.SELECTING_BALL;
            return true;
        }
        return false;
    }

    private boolean canSelectSlot(int slot) {
        return (State.IDLE == currentState || State.AWAITING_LAUNCH == currentState || State.SELECTING_BALL == currentState || State.LAUNCHING == currentState || State.HOMING == currentState) && 0 <= slot && 3 > slot;
    }

    public boolean launch() {
        if (canLaunch()) {
            currentState = State.LAUNCHING;
            turnstile.launchSlots(1);
            ballSlots[2] = BallState.VACANT;
            rotateBallStates(1);
            return true;
        }
        return false;
    }

    private boolean canLaunch() {
        return State.AWAITING_LAUNCH == currentState || State.IDLE == currentState || State.SELECTING_BALL == currentState || State.LAUNCHING == currentState || State.HOMING == currentState;
    }

    /**
     * Plans and initiates an autonomous shot sequence based on a detected AprilTag ID.
     * This method creates the shot plan (e.g., [PURPLE, GREEN, PURPLE]) and then immediately
     * begins the process by rotating the first required ball into the firing position. The
     * actual flip/launch is handled automatically by the update() state machine.
     *
     * @param aprilTagId The ID of the AprilTag detected (21, 22, or 23).
     * @return
     */
    public String planShotSequence(int aprilTagId) {
        // Only start a new sequence if the facade is idle.
        if (State.IDLE == currentState || State.AWAITING_LAUNCH == currentState) {
            sequence_id = aprilTagId;

            switch (aprilTagId) {
                case 21: // Motif: G-P-P
                    shotSequence = Arrays.asList(BallState.GREEN, BallState.PURPLE, BallState.PURPLE);
                    break;
                case 22: // Motif: P-G-P
                    shotSequence = Arrays.asList(BallState.PURPLE, BallState.GREEN, BallState.PURPLE);
                    break;
                case 23: // Motif: P-P-G
                    shotSequence = Arrays.asList(BallState.PURPLE, BallState.PURPLE, BallState.GREEN);
                    break;
                default:
                    // Invalid ID, do nothing.
                    return null;
            }

            sequenceIndex = 0;

            return shotSequence.toString();
        } else return null;

    }


    // --- Compatibility Shims for TeleOp (Corrected) ---

    public void adjustToThird() {
        setCurrentState(State.HOMING);
        turnstile.home();
    } // Corrected: This is now a manual homing trigger.

    public void spin(double power) {
        turnstile.spin(power);
        currentState = State.IDLE;
    }

    public boolean cycle(int direction) {
        // Corrected: This now cycles to the next adjacent slot.
        int startSlot = (-1 == currentTargetSlot) ? 0 : currentTargetSlot;
        int nextSlot = (startSlot + direction + 3) % 3; // Handles positive/negative direction and wrap-around
        return selectSlot(nextSlot);
    }

    public boolean ballInIndexer() {
        return 3 < beamBreakCounter;
    }

    public boolean isAtTarget() {
        return turnstile.isAtTarget();
    }

    public boolean isNearSlot() {
        return turnstile.isOverSlot();
    }

    public State getCurrentState() {
        return currentState;
    }

    public void setCurrentState(State state) {
        currentState = state;
    }

    public void intake() {
        isIntaking = true;
        turnstile.intakeStart();
    }

    public void intakeStop() {
        isIntaking = false;
        turnstile.intakeStop();
    }

    public boolean inIntakeSlot() {
        return beamBreak.isBallDetectedInIndexer();
    }

    public BallState getBallState(int slot) {
        ballSensors[slot * 2].update();
        ballSensors[slot * 2 + 1].update();
        updated = true;
        updateBallStates();
        return (0 <= slot && 3 > slot) ? ballSlots[slot] : BallState.VACANT;
    }

    public BallState getLastBallState(int slot) {
        return (0 <= slot && 3 > slot) ? ballSlots[slot] : BallState.VACANT;
    }

    public boolean indexerIsFull() {
        return !(State.SELECTING_BALL == currentState || State.LAUNCHING == currentState) && 3 <= ballNumber;
    }

    public int getCurrentTargetSlot() {
        return currentTargetSlot;
    }

    public double getIndexerAngle() {
        return turnstile.getCurrentAngle();
    }

    public void updateBallSensors() {
        if (!updated) {
            if (TELEM) {
                telemetry.addData("updating color sensors: ", true);
            }
            for (int i = 0; 6 > i; i++) {
                ballSensors[i].update();
            }
            updated = true;
        }
    }

    public void updateBallCount() {
        ballNumber = 0;
        for (int i = 0; i < 3; i++) {
            if (i == 0) {
                if ((BallState.GREEN == ballSlots[i] || BallState.PURPLE == ballSlots[i]) && ballInIndexer()) {
                    ballNumber++;
                }
            } else if (BallState.GREEN == ballSlots[i] || BallState.PURPLE == ballSlots[i]) {
                ballNumber++;
            }
        }
        if (beamBreak.isBallDetectedInIntake()) {
            ballNumber++;
        }

    }

    public int getBallNumber() {
        return ballNumber;
    }

    public void update(boolean isAtRpm) {

//        flipper.update();
        turnstile.update();
        beamBreak.update();

        updated = false;

        if (isIntaking) {
            if (inIntakeSlot()) {
                beamBreakCounter++;
                beamBreakCounter = Math.min(beamBreakCounter, 5);
            } else {
                beamBreakCounter--;
                beamBreakCounter = Math.max(beamBreakCounter, 0);
            }

            if (turnstile.isOverSlot() && 3 <= beamBreakCounter && !indexerIsFull() && turnstile.isAtTargetTime()) {
                readyNextIntakeSlot(BallState.VACANT);
            }
        }


        switch (currentState) {
            case HOMING:
                if (turnstile.isHomed()) {
                    updateBallSensors();
                    beamBreakCounter = 0;
                    currentState = State.IDLE;
                }
                break;
            case IDLE: // Waiting for a command
                break;
            case SELECTING_BALL:
                if (turnstile.isAtTarget()) {
                    beamBreakCounter = 0;
                    updateBallSensors();
                    currentState = State.AWAITING_LAUNCH;

                }
                break;
            case AWAITING_LAUNCH: // In position, ready to receive a flip() command from an external source.
                // Do nothing. The system will wait here until flip() is called.
                if (!turnstile.isAtTarget()) {
                    currentState = State.SELECTING_BALL;
                }
                break;
            case LAUNCHING:
                if (turnstile.isAtTarget()) {
                    currentState = State.SELECTING_BALL;
                }
                break;
        }

        // Only update ball states from sensors if we are NOT in an active auto-sequence
        // This prevents a ball that has been logically "used" from being re-detected.
        updateBallStates();
        updateBallCount();
        previousBallStateIntake = ballSlots[0];

        addTelemetry();
    }

    public void updateBallStates() {
        for (int i = 0; 3 > i; i++) {
            // Get the detected colors from the sensor pairs (0,1), (2,3), (4,5)
            BallSensor sensorA = ballSensors[i * 2];
            BallSensor sensorB = ballSensors[i * 2 + 1];
            BallSensor.BallColor colorA = sensorA.getDetectedColor();
            BallSensor.BallColor colorB = sensorB.getDetectedColor();

//            BallSensor V3 = (sensorA.isV2 ? sensorB : sensorA);

            if (BallSensor.BallColor.PURPLE == colorA || BallSensor.BallColor.PURPLE == colorB) {
                // Priority 1: Either is Purple
                ballSlots[i] = BallState.PURPLE;
            } else if (BallSensor.BallColor.GREEN == colorA || BallSensor.BallColor.GREEN == colorB) {
                // Priority 2: Both must be Green
                ballSlots[i] = BallState.GREEN;
            } else {
                // Default: Both NONE or mixed Green/None
                ballSlots[i] = BallState.VACANT;
            }
            sensorA.addTelemetry();
            sensorB.addTelemetry();
        }
    }

    private void addTelemetry() {
        if (!TELEM) return;
        telemetry.addData("Indexer Facade State", currentState.name());
        telemetry.addData("Beam Break detection: ", ballInIndexer());
        telemetry.addLine(String.format("Slots: [0]: %s, [1]: %s, [2]: %s",
                ballSlots[0], ballSlots[1], ballSlots[2]));
        telemetry.addLine(String.format("Fired: [0]: %s, [1]: %s, [2]: %s",
                slots_fired[0], slots_fired[1], slots_fired[2]));
        telemetry.addData("Ball Number", ballNumber);
        telemetry.addData("Intake Previous", previousBallStateIntake);
        telemetry.addData("in Sequence: ", sequenceStarted);

        if (null != shotSequence) {
            telemetry.addData("Target Slot: ", currentTargetSlot);
            telemetry.addData("Sequence: ", shotSequence);
            telemetry.addData("Sequence Step", sequenceIndex + " / " + shotSequence.size());

        }
    }

    public boolean isInSequence() {
        return sequenceStarted || null != shotSequence;
    }

    public boolean isDone() {
        // The facade is "done" if it's idle or if it's ready for a manual flip.
        // During an auto-sequence, it is NOT done.
        return (null == shotSequence) && (State.IDLE == currentState || State.AWAITING_LAUNCH == currentState);
    }
}
