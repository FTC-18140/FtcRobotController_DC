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
    private Flipper flipper = null;
    private Turnstile turnstile = null;
    private BallSensor[] ballSensors = new BallSensor[6];
    private BeamBreaker beamBreak = new BeamBreaker();
    private Telemetry telemetry = null;
    private final ElapsedTime flipTimer = new ElapsedTime();

    // --- Constants ---
    public static final double[] SLOT_ANGLES = {120.0, 240.0, (double) 0}; // Angles for slots 0, 1, and 2
    public static final double FLIP_TIME_SECONDS = 0.16; // Time for the flipper to extend and retract

    private final ElapsedTime cycleTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public int beamBreakCounter = 0;

    public static boolean TELEM = false;
    private boolean updated = false;

    // --- State Management ---
    public enum State {IDLE, HOMING, SELECTING_BALL, INTAKE, AWAITING_FLIP, FLIPPING, FLIP_TO_CYCLE, RETRACTING_FLIPPER}

    private State currentState = State.IDLE;
    private boolean isIntaking = false;


    /**
     * The facade's internal model of what is in each slot.
     */
    public enum BallState {GREEN, PURPLE, VACANT, ALL}

    private BallState[] ballSlots = new BallState[3];
    private BallState[] positedBallStates = new BallState[3];
    private int currentTargetSlot = 2;
    private int ballNumber = 0;
    private BallState previousBallStateIntake = BallState.VACANT;


    // --- Auto-Sequence Management ---
    private List<BallState> shotSequence = null;
    private boolean sequenceStarted = false;
    private int sequenceIndex = -1;
    private boolean[] slots_fired = new boolean[3];

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;

        flipper = new Flipper();
        flipper.init(hwMap, telem);

        turnstile = new Turnstile();
        turnstile.init(hwMap, telem);

        for (int i = 0; 6 > i; i++) {
            ballSensors[i] = new BallSensor();
            ballSensors[i].init(hwMap, telem, "color" + i, i);
        }
        for (int i = 0; 3 > i; i++) {
            ballSlots[i] = BallState.VACANT;
        }
        for (int i = 0; 3 > i; i++) {
            slots_fired[i] = false;
        }

        beamBreak.init(hwMap, telem);

        cycleTimer.reset();
        updateBallSensors();
        updateBallStates();
        for (int i = 0; 3 > i; i++) {
            positedBallStates[i] = ballSlots[i];
            if (BallState.GREEN == ballSlots[i] || BallState.PURPLE == ballSlots[i]) {
                ballNumber++;
            }
        }

        currentState = State.IDLE;
        //turnstile.home();
    }

    public void flipOverride(boolean up) {
        if (up && turnstile.isAtTarget()) {
            flipper.extend();
        } else {
            flipper.retract();
        }
    }

    public void setInitialBallStates(BallState[] initialStates) {
        if (3 == initialStates.length) {
            this.ballSlots = initialStates;
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
    private boolean executeNextInSequence() {
        // Safety check: Do nothing if the sequence is not active.
        if (null == shotSequence || 0 > sequenceIndex || sequenceIndex >= shotSequence.size() || !flipper.isRetracted())
            return false;

        sequenceStarted = true;

        // Determine which color we need for this step of the sequence.
        BallState requiredColor = shotSequence.get(sequenceIndex);
        boolean ballFound = false;
        updateBallSensors();
        updateBallStates();

        // Search all physical slots for a ball that matches the required color.
        for (int i = 2; -1 < i && !ballFound; i--) {
            if (ballSlots[i] == requiredColor || (BallState.ALL == requiredColor && BallState.VACANT != ballSlots[i])) {
                // --- Critical Step ---
                // Mark this ball as "used" by changing its state in our software model to VACANT.
                // This prevents the system from re-selecting this same physical ball for a
                // later step in the sequence (e.g., if the sequence requires two PURPLE balls).
                ballSlots[i] = BallState.VACANT;
                slots_fired[i] = true;

                // Command the turnstile to rotate this slot into the firing position.
                ballFound = true;
                int slot = (currentTargetSlot + (2 - i)) % 3;
                telemetry.addData("selected Sequence Slot: ", slot);
                rotateBallStates(2 - i);
                currentTargetSlot = slot;
                turnstile.seekToAngle(SLOT_ANGLES[slot]);
                beamBreakCounter = 0;
                currentState = State.SELECTING_BALL;
            }
        }

        // If no ball of the required color could be found, something is wrong.
        // To prevent getting stuck, we cancel the entire autonomous sequence.
        if (ballFound) {
            return true;
        } else {
            return launchSafeBackup();
        }
    }

    private boolean launchSafeBackup() {
        boolean ballFound = false;
        updateBallSensors();
        updateBallStates();

        for (int i = 2; -1 < i && !ballFound; i--) {
            if (!slots_fired[i] && !usedLaterInSequence(ballSlots[i])) {

                ballSlots[i] = BallState.VACANT;
                slots_fired[i] = true;

                // Command the turnstile to rotate this slot into the firing position.
                ballFound = true;

                int slot = (currentTargetSlot + (2 - i)) % 3;
                telemetry.addData("selected Sequence Slot: ", slot);
                rotateBallStates(2 - i);
                currentTargetSlot = slot;
                turnstile.seekToAngle(SLOT_ANGLES[slot]);
                beamBreakCounter = 0;
                currentState = State.SELECTING_BALL;
            }
        }

        // If no ball of the required color could be found, something is wrong.
        // To prevent getting stuck, we cancel the entire autonomous sequence.
        if (ballFound) {
            return true;
        } else {
            return launchAnyBackup();
        }
    }

    private boolean launchAnyBackup() {
        boolean ballFound = false;
        updateBallSensors();
        updateBallStates();

        for (int i = 2; -1 < i && !ballFound; i--) {
            if (!slots_fired[i]) {

                ballSlots[i] = BallState.VACANT;
                slots_fired[i] = true;

                // Command the turnstile to rotate this slot into the firing position.
                ballFound = true;

                int slot = (currentTargetSlot + (2 - i)) % 3;
                telemetry.addData("selected Sequence Slot: ", slot);
                rotateBallStates(2 - i);
                currentTargetSlot = slot;
                turnstile.seekToAngle(SLOT_ANGLES[slot]);
                beamBreakCounter = 0;
                currentState = State.SELECTING_BALL;
            }
        }

        // If no ball of the required color could be found, something is wrong.
        // To prevent getting stuck, we cancel the entire autonomous sequence.
        if (ballFound) {
            return true;
        } else {
            beamBreakCounter = 0;
            currentState = State.SELECTING_BALL;
            return true;
        }
    }

    public boolean usedLaterInSequence(BallState ballState) {
        boolean returnValue = false;
        for (int i = sequenceIndex + 1; 3 > i; i++) {
            if (ballState == shotSequence.get(i)) {
                returnValue = true;
                break;
            }
        }
        return returnValue;
    }


    public void launchAllInIndexer() {
        if (State.IDLE != currentState && State.AWAITING_FLIP != currentState) return;

        shotSequence = Arrays.asList(BallState.ALL, BallState.ALL, BallState.ALL);

        sequenceIndex = 0;
        executeNextInSequence();
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

                return !executeNextInSequence() && isInSequence();
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

            BallState temp1 = positedBallStates[2];
            positedBallStates[2] = positedBallStates[1];
            positedBallStates[1] = positedBallStates[0];
            positedBallStates[0] = temp1;
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
        if ((State.IDLE == currentState || State.AWAITING_FLIP == currentState) && flipper.isRetracted()) {
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
        if ((State.IDLE == currentState || State.AWAITING_FLIP == currentState || isIntaking) && flipper.isRetracted()) {
            int startSlot = 0;

            updateBallSensors();
            updateBallStates();
            for (int i = 3; 0 < i && !slotFound; i--) {

                int slotToCheck = (startSlot + i) % 3;

                if (ballSlots[slotToCheck] == ballState) {

                    int slot = (currentTargetSlot + (3 - slotToCheck)) % 3;
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
        if (flipper.isRetracted() && (State.IDLE == currentState || State.AWAITING_FLIP == currentState || State.SELECTING_BALL == currentState || State.FLIP_TO_CYCLE == currentState || State.RETRACTING_FLIPPER == currentState) && 0 <= slot && 3 > slot) {
            rotateBallStates((slot - currentTargetSlot + 3) % 3);
            currentTargetSlot = slot;
            turnstile.seekToAngle(SLOT_ANGLES[currentTargetSlot]);
            beamBreakCounter = 0;
            currentState = State.SELECTING_BALL;
            return true;
        }
        return false;
    }

    public boolean flip() {
        if ((State.AWAITING_FLIP == currentState || State.IDLE == currentState) && turnstile.isAtTarget()) {
            currentState = State.FLIPPING;
            flipper.extend();
            flipTimer.reset();
            return true;
        }
        return false;
    }

    public void manualFlip() {

    }

    public boolean flipAndCycle() {
        boolean returnValue = false;
        if ((State.AWAITING_FLIP == currentState || State.IDLE == currentState) && turnstile.isAtTarget()) {
            currentState = State.FLIP_TO_CYCLE;
            flipper.extend();
            flipTimer.reset();
            returnValue = true;
        }
        return returnValue;
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
        if (State.IDLE != currentState && State.AWAITING_FLIP != currentState) return null;

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


    }


    // --- Compatibility Shims for TeleOp (Corrected) ---
    public void unflip() { /* The new state machine handles this automatically */ }

    public void adjustToThird() {
        setCurrentState(State.HOMING);
    } // Corrected: This is now a manual homing trigger.

    public void spin(double power) {
        turnstile.spin(power);
    }

    public boolean cycle(int direction) {
        // Corrected: This now cycles to the next adjacent slot.
        int startSlot = (-1 != currentTargetSlot) ? currentTargetSlot : 0;
        int nextSlot = (startSlot + direction + 3) % 3; // Handles positive/negative direction and wrap-around
        return selectSlot(nextSlot);
    }

    public boolean ballInIndexer() {
        return 3 < beamBreakCounter;
    }

    public boolean isBallInIntake() {
        return beamBreak.ballDetectedInIntake();
    }

    public boolean isFlipperDown() {
        return flipper.isRetracted();
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
    }

    public void intakeStop() {
        isIntaking = false;
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
        return !((BallState.VACANT == ballSlots[0] || BallState.VACANT == ballSlots[1] || BallState.VACANT == ballSlots[2]) || State.SELECTING_BALL == currentState) || 3 <= ballNumber;
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
        for (int i = 0; i < ballSlots.length; i++) {
            if (BallState.GREEN == ballSlots[i] || BallState.PURPLE == ballSlots[i] || (0 == i && ballInIndexer())) {
                ballNumber++;
            }
        }
        if (isBallInIntake()) ballNumber++;
    }

    public int getBallNumber() {
        return ballNumber;
    }

    public void updateIntakePosited() {
        positedBallStates[0] = ballSlots[0];
    }

    public void update(boolean isAtRpm) {

        flipper.update();
        turnstile.update();
        beamBreak.update();

        updated = false;

        if (beamBreak.ballDetectedInIndexer()) {
            beamBreakCounter++;
            beamBreakCounter = Math.min(beamBreakCounter, 5);
        } else {
            beamBreakCounter--;
            beamBreakCounter = Math.max(beamBreakCounter, 0);
        }

        if (isIntaking && turnstile.isOverSlot() && ballInIndexer()) {
//            if (previousBallStateIntake == BallState.VACANT && ballSlots[0] != BallState.VACANT) {
//                ballNumber++;
//            }

            cycleTimer.reset();
            readyNextIntakeSlot(BallState.VACANT);
        }


        switch (currentState) {
            case HOMING:
                turnstile.home();
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
                    updateIntakePosited();
                    currentState = State.AWAITING_FLIP;
                }
                break;
            case AWAITING_FLIP: // In position, ready to receive a flip() command from an external source.
                // Do nothing. The system will wait here until flip() is called.
                if (null != shotSequence && turnstile.isAtTarget() && sequenceStarted && isAtRpm) {
                    flip();
                    ballNumber--;
                    positedBallStates[2] = BallState.VACANT;
                    if (0 > ballNumber) ballNumber = 0;

                    sequenceStarted = false;
                }
                break;
            case FLIPPING:
                if (flipTimer.seconds() > FLIP_TIME_SECONDS) {
                    flipper.retract();
                    currentState = State.RETRACTING_FLIPPER;
                }
                break;
            case FLIP_TO_CYCLE:
                if (flipTimer.seconds() > FLIP_TIME_SECONDS) {
                    flipper.retract();
                    if (flipper.isRetracted()) {
                        if (cycle(1)) {
                            currentState = State.RETRACTING_FLIPPER;
                        }
                    }
                }
                break;
            case RETRACTING_FLIPPER:
                if (flipper.isRetracted()) {
                    // If we were in a sequence, advance to the next step.
                    if (null != shotSequence) {
                        sequenceIndex++;
                        if (sequenceIndex < shotSequence.size()) {
                            executeNextInSequence();
                        } else {
                            cancelSequence(); // Sequence complete
                        }
                    } else {
                        // Otherwise, just go back to idle.
                        updateBallSensors();
                        currentState = State.IDLE;
                    }
                }
                break;
        }

        // Only update ball states from sensors if we are NOT in an active auto-sequence
        // This prevents a ball that has been logically "used" from being re-detected.
        if (null == shotSequence) {
            updateBallStates();
        }
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
        telemetry.addLine(String.format("Slots: [0]: %s, [1]: %s, [2]: %s",
                positedBallStates[0], positedBallStates[1], positedBallStates[2]));
        telemetry.addLine(String.format("Fired: [0]: %s, [1]: %s, [2]: %s",
                slots_fired[0], slots_fired[1], slots_fired[2]));
        telemetry.addData("Ball Number", Integer.valueOf(ballNumber));
        telemetry.addData("Intake Previous", previousBallStateIntake);
        telemetry.addData("in Sequence: ", Boolean.valueOf(sequenceStarted));

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
        return (null == shotSequence) && (State.IDLE == currentState || State.AWAITING_FLIP == currentState);
    }
}
