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
    private Flipper flipper;
    private Turnstile turnstile;
    private BallSensor[] ballSensors = new BallSensor[6];
    private BeamBreaker beamBreak = new BeamBreaker();
    private Telemetry telemetry;
    private ElapsedTime flipTimer = new ElapsedTime();
    private IndexerModel ballModel = new IndexerModel();


    // --- Constants ---
    private static final int SLOT_COUNT = 3;
    public static final double[] SLOT_ANGLES = {120, 240, 0}; // Angles for slots 0, 1, and 2
    private static final double FLIP_TIME_SECONDS = 0.1; // Time for the flipper to extend and retract
    private static final double CYCLE_TIME_SECONDS = 0.5; // Time for the flipper to extend and retract

    public static boolean TELEM = false;
    private boolean updated = false;

    public void flipOverride( boolean up ) {
        if (up) {
            flipper.extend();
        }
        else {
            flipper.retract();
        }
    }


    // --- State Management ---
    public enum State { IDLE, HOMING, SELECTING_BALL, INTAKE, AWAITING_FLIP, FLIPPING, FLIP_TO_CYCLE, RETRACTING_FLIPPER }
    private State currentState = State.IDLE;
    private boolean intaking = false;

    /** The facade's internal model of what is in each slot. */
    public enum BallState { GREEN, PURPLE, VACANT, ALL, UNKNOWN }
    private int currentTargetSlot = 2;

    // --- Auto-Sequence Management ---
    private List<BallState> shotSequence = null;
    private boolean sequenceStarted = false;
    private int sequenceIndex = -1;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;

        flipper = new Flipper();
        flipper.init(hwMap, telem);

        turnstile = new Turnstile();
        turnstile.init(hwMap, telem);

        for (int i = 0; i < 6; i++) {
            ballSensors[i] = new BallSensor();
            ballSensors[i].init(hwMap, telem, "color" + i, i);
        }
        beamBreak.init(hwMap, telem);

        ballModel.reset();
        // Refresh sensors then rebuild model states before making planning decisions.
        updateBallSensors();
        updateBallStates();
        currentState = State.IDLE;
        //turnstile.home();
    }

    public void setInitialBallStates(BallState[] initialStates) {
        ballModel.setSlots(initialStates);
    }

    private int normalizeSlot(int slot) {
        return (slot % SLOT_COUNT + SLOT_COUNT) % SLOT_COUNT;
    }

    private boolean isValidSlot(int slot) {
        return slot >= 0 && slot < SLOT_COUNT;
    }

    private double angleForSlot(int slot) {
        return SLOT_ANGLES[normalizeSlot(slot)];
    }


    private boolean moveToSlot(int slot) {
        if (!isValidSlot(slot)) return false;

        // Compute minimal normalized CCW step count from current slot to requested slot.
        int stepsCCW = normalizeSlot(slot - currentTargetSlot);
        // Rotate the logical model in lockstep with the physical turnstile command.
        ballModel.rotateCCW(stepsCCW);
        currentTargetSlot = slot;
        turnstile.seekToAngle(angleForSlot(currentTargetSlot));
        currentState = State.SELECTING_BALL;
        return true;
    }

    private boolean isBallMatch(BallState targetState, BallState candidateState) {
        return ballModel.matches(targetState, candidateState);
    }

    private int mapRelativeSlotToPhysical(int relativeSlot) {
        return normalizeSlot(currentTargetSlot + (2 - relativeSlot));
    }

    // --- High-Level API & Compatibility Shims ---

    /**
     * (Private Helper) Executes the next step in the planned shot sequence.
     * This method is the core of the autonomous firing logic. It finds the next required ball
     * from the sequence, locates it in one of the physical slots, and begins rotating the
     * turnstile to that slot. It marks the selected slot as used in `IndexerModel` to prevent
     * the same ball from being used twice to fulfill the sequence.
     */
    private boolean executeNextInSequence() {
        // Safety check: Do nothing if the sequence is not active.
        if (shotSequence == null || sequenceIndex < 0 || sequenceIndex >= shotSequence.size()) return false;

        sequenceStarted = true;

        // Determine which color we need for this step of the sequence.
        BallState requiredColor = shotSequence.get(sequenceIndex);
        boolean ballFound = false;
        // Refresh sensors then rebuild model states before making planning decisions.
        updateBallSensors();
        updateBallStates();

        // Search all physical slots for a ball that matches the required color.
        for (int i = SLOT_COUNT - 1; i >= 0 && !ballFound; i--) {
            if (isBallMatch(requiredColor, ballModel.getSlot(i))) {
                // --- Critical Step ---
                // Mark this ball as "used" by changing its state in our software model to VACANT.
                // This prevents the system from re-selecting this same physical ball for a
                // later step in the sequence (e.g., if the sequence requires two PURPLE balls).
                // Consume this slot in the logical model so sequence planning does not reuse it.
                ballModel.markSlotUsed(i);

                // Command the turnstile to rotate this slot into the firing position.
                ballFound = true;
                // Convert model-relative slot choice into physical target slot for the turnstile.
                int slot = mapRelativeSlotToPhysical(i);
                telemetry.addData("selected Sequence Slot: ", slot);
                moveToSlot(slot);
            } 
        }

        // If no ball of the required color could be found, something is wrong.
        // To prevent getting stuck, we cancel the entire autonomous sequence.
        if (!ballFound) {
            return launchBackup();
        } else if (currentState == State.SELECTING_BALL){
            return true;
        }
        return false;
    }

    private boolean launchBackup() {
        boolean ballFound = false;
        // Refresh sensors then rebuild model states before making planning decisions.
        updateBallSensors();
        updateBallStates();

        for (int i = SLOT_COUNT - 1; i >= 0 && !ballFound; i--) {
            if (!ballModel.wasSlotFired(i) && !usedLaterInSequence(ballModel.getSlot(i))) {

                // Consume this slot in the logical model so sequence planning does not reuse it.
                ballModel.markSlotUsed(i);

                // Command the turnstile to rotate this slot into the firing position.
                ballFound = true;

                // Convert model-relative slot choice into physical target slot for the turnstile.
                int slot = mapRelativeSlotToPhysical(i);
                telemetry.addData("selected Sequence Slot: ", slot);
                moveToSlot(slot);
            }
        }

        // If no ball of the required color could be found, something is wrong.
        // To prevent getting stuck, we cancel the entire autonomous sequence.
        if (!ballFound) {
            currentState = State.SELECTING_BALL;
            return true;
        } else if (currentState == State.SELECTING_BALL){
            return true;
        }
        return false;
    }
    public boolean usedLaterInSequence(BallState ballState){
        for(int i = sequenceIndex + 1; i < shotSequence.size(); i++){
            if (ballState == shotSequence.get(i)){
                return true;
            }
        }
        return false;
    }



    public void launchAllInIndexer(){
        if (currentState != State.IDLE && currentState != State.AWAITING_FLIP) return;


        shotSequence = Arrays.asList(BallState.ALL, BallState.ALL, BallState.ALL);

        sequenceIndex = 0;
        executeNextInSequence();
    }
    public Action runCurrentSequenceAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ballModel.clearFiredFlags();

                return !executeNextInSequence() && inSequence();
            }
        };
    }

    public void cancelSequence() {
        sequenceStarted = false;
        shotSequence = null;
        sequenceIndex = -1;
        ballModel.clearFiredFlags();
        if (currentState != State.IDLE) {
            currentState = State.IDLE;
        }
    }

    /**
     * Indexes a color i
     * @param ballState the color we want to select
     * @return whether it found that color
     */
    public boolean selectNextSlot(BallState ballState) {
        // Refactored to have a single exit point
        boolean slotFound = false;
        if (currentState == State.IDLE || currentState == State.AWAITING_FLIP) {
            int startSlot = currentTargetSlot;

            for (int i = SLOT_COUNT; i > 0 && !slotFound; i--) {
                updateBallSensors();
                updateBallStates();
                // Sweep slots in priority order starting from the current target position.
                int slotToCheck = (startSlot + i) % SLOT_COUNT;
                if (isBallMatch(ballState, ballModel.getSlot(slotToCheck))) {
                    moveToSlot(slotToCheck);
                    slotFound = true;
                }
            }
        }
        return slotFound;
    }
    public boolean readyNextIntakeSlot(BallState ballState) {
        // Refactored to have a single exit point
        boolean slotFound = false;
        if (currentState == State.IDLE || currentState == State.AWAITING_FLIP || intaking) {
            int startSlot = 0;

            updateBallSensors();
            updateBallStates();
            for (int i = SLOT_COUNT; i > 0 && !slotFound; i--) {

                // Sweep slots in priority order starting from the current target position.
                int slotToCheck = (startSlot + i) % SLOT_COUNT;

                if (ballModel.getSlot(slotToCheck) == ballState) {

                    // Translate intake-relative vacancy index into turnstile target index.
                    int slot = normalizeSlot(currentTargetSlot + (SLOT_COUNT - slotToCheck));
                    moveToSlot(slot);
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
        return flipper.isRetracted()
                && (currentState == State.IDLE || currentState == State.AWAITING_FLIP || currentState == State.SELECTING_BALL || currentState == State.FLIP_TO_CYCLE || currentState == State.RETRACTING_FLIPPER)
                && moveToSlot(slot);
    }

    public boolean flip() {
        if ((currentState == State.AWAITING_FLIP || currentState == State.IDLE) && turnstile.isAtTarget()) {
            currentState = State.FLIPPING;
            flipper.extend();
            flipTimer.reset();
            return true;
        }
        return false;
    }

    public void manualFlip()
    {

    }

    public boolean flipAndCycle(){
        if ((currentState == State.AWAITING_FLIP || currentState == State.IDLE) && turnstile.isAtTarget()) {
            currentState = State.FLIP_TO_CYCLE;
            flipper.extend();
            flipTimer.reset();
            return true;
        }
        return false;
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
        if (currentState != State.IDLE && currentState != State.AWAITING_FLIP) return null;

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
        setState(State.HOMING);
    } // Corrected: This is now a manual homing trigger.
    public void spin(double power) { turnstile.spin(power); }
    public boolean cycle(int direction) {
        // Corrected: This now cycles to the next adjacent slot.
        int startSlot = (currentTargetSlot != -1) ? currentTargetSlot : 0;
        int nextSlot = (startSlot + direction + SLOT_COUNT) % SLOT_COUNT; // Handles positive/negative direction and wrap-around
        return selectSlot(nextSlot);
    }
    public boolean ballInIntake(){return beamBreak.ballDetected();}
    public boolean isAtTarget(){
        return turnstile.isAtTarget();
    }
    public State getState() { return currentState; }
    public void setState(State state) { this.currentState = state; }
    public void intake(){ intaking = true;}
    public void intakeStop(){ intaking = false;}
    public BallState getBallState(int slot) {
        // Force-refresh only the requested slot pair for a low-latency read.
        ballSensors[slot*2].update();
        ballSensors[slot*2+1].update();
        updated = true;
        updateBallStates();
        return ballModel.getSlot(slot);
    }
    public BallState getLastBallState(int slot) {
        return ballModel.getSlot(slot);
    }
    public boolean indexerIsFull(){
        return ballModel.isFull() && currentState != State.SELECTING_BALL;
    }
    public int getCurrentTargetSlot() { return currentTargetSlot; }
    public double getIndexerAngle(){
        return turnstile.getCurrentAngle();
    }
    public void updateBallSensors() {
        // Cache sensor reads per loop to avoid duplicate I2C transactions.
        if(!updated){
                if (TELEM) {
                    telemetry.addData("updating color sensors: ", true);
                }
                for (int i = 0; i < 6; i++) {
                    ballSensors[i].update();
                }
                updated = true;
        }
    }

    public void update(boolean isAtRpm) {
        flipper.update();
        turnstile.update();
        beamBreak.update();

        updated = false;

        if(intaking && turnstile.isOverSlot() && !indexerIsFull() && ballInIntake()){
            readyNextIntakeSlot(BallState.VACANT);
        }

        switch (currentState) {
        case HOMING:
            turnstile.home();
            if (turnstile.isHomed()) {
                updateBallSensors();
                currentState = State.IDLE;
            }
            break;
        case IDLE: // Waiting for a command
            break;
        case SELECTING_BALL:
            if (turnstile.isAtTarget()) {
                updateBallSensors();
                currentState = State.AWAITING_FLIP;
            }
            break;
        case AWAITING_FLIP: // In position, ready to receive a flip() command from an external source.
            // Do nothing. The system will wait here until flip() is called.
            if(shotSequence != null && turnstile.isAtTarget() && sequenceStarted && isAtRpm){
                flip();
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
                if(flipper.isRetracted()) {
                    if (cycle(1)) {
                        currentState = State.RETRACTING_FLIPPER;
                    }
                }
            }
            break;
        case RETRACTING_FLIPPER:
            if (flipper.isRetracted()) {
                // If we were in a sequence, advance to the next step.
                if (shotSequence != null) {
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
        if (shotSequence == null) {
            // In manual mode, let live sensors overwrite model state each cycle.
            updateBallStates();
        }

        addTelemetry();
    }

    public void updateBallStates() {
        ballModel.updateFromSensors(ballSensors);
    }

    private void addTelemetry() {
        if ( !TELEM ) return;
        addTelemetry("Indexer Facade State", currentState.name());
        addTelemetry("Beam Break detection: ", ballInIntake());
        // Snapshot model arrays once for consistent telemetry formatting.
        BallState[] slots = ballModel.copySlots();
        boolean[] fired = ballModel.copyFiredSlots();
        telemetry.addLine(String.format("Slots: [0]: %s, [1]: %s, [2]: %s",
                slots[0], slots[1], slots[2]));
        telemetry.addLine(String.format("Fired: [0]: %s, [1]: %s, [2]: %s",
                fired[0], fired[1], fired[2]));
        addTelemetry("in Sequence: ", sequenceStarted);
        if (shotSequence != null) {
            addTelemetry("Target Slot: ", getCurrentTargetSlot());
            addTelemetry("Sequence: ", shotSequence);
            addTelemetry("Sequence Step", sequenceIndex + " / " + shotSequence.size());
        }
    }

    /**
     * Generic method for Objects
     */
    public void addTelemetry(String name, Object value) {
        if (TELEM) {
            // [TAG   ] (6 chars) + Name (15 chars)
            // %-6.6s  -> Exactly 6 chars, Left Aligned
            // %-15.15s -> Exactly 10 chars, Left Aligned
            String tag = String.format("[%-6.6s]", this.getClass().getSimpleName().toUpperCase());

            telemetry.addData(tag + " " + name, value);
        }
    }

    /**
     * Overloaded method for Doubles (fixed precision + fixed label width)
     */
    public void addTelemetry(String name, double value) {
        if (TELEM) {
            String tag = String.format("[%-6.6s]", this.getClass().getSimpleName().toUpperCase());

            // %10.4f ensures the number itself doesn't jitter
            telemetry.addData(tag + " " + name, String.format("%10.4f", value));
        }
    }

    public boolean inSequence(){
        return sequenceStarted || shotSequence != null;
    }
    
    public boolean isDone() {
        // The facade is "done" if it's idle or if it's ready for a manual flip.
        // During an auto-sequence, it is NOT done.
        return (shotSequence == null) && (currentState == State.IDLE || currentState == State.AWAITING_FLIP);
    }
}
