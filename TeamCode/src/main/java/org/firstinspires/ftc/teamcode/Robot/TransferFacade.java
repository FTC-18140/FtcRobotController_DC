package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_TRANSFER_FACADE;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

@Config
public class TransferFacade
{
    private LiftingTurnstile turnstile = null;
    private BallSensor[] ballSensors = new BallSensor[2];
    private BeamBreakSystem beamBreak = new BeamBreakSystem();
    private Telemetry telemetry = null;

    private int currentTargetSlot = 0;
    private int pendingShift = 0;
    public static double IDLE_TOLERANCE = 15;
    private boolean enter = true;

    public enum State
    {UNKNOWN, HOMING, IDLE, MOVING, LAUNCHING}

    private State currentState = State.IDLE;

    public enum BallState
    {GREEN, PURPLE, VACANT, OCCUPIED, ALL, UNKNOWN}

    private BallState[] ballSlots = new BallState[3];

    ElapsedTime pauseTimer = new ElapsedTime();

    // Fields/Variables to support Auto
    private boolean override = false;
    private boolean isIntaking = false;
    private int ballCount = 0;

    /**
     * Updates the raw data from the ball sensors.
     */
    public void updateBallSensors()
    {
    }

    /**
     * Updates the internal states of the balls in each slot.
     */
    public void updateBallStates()
    {
    }

    /**
     * Selects the next appropriate slot based on the provided ball state.
     *
     * @param ballState The state of the ball to find a slot for.
     */
    public void selectNextSlot(IndexerFacade.BallState ballState)
    {
    }

    /**
     * Gets the index of the slot the turnstile is currently targeting.
     *
     * @return The current target slot index (0-2).
     */
    public int getCurrentTargetSlot()
    {
        return 1;
    }

    /**
     * Retrieves the state of a specific ball slot.
     *
     * @param slotToWatch The index of the slot to check.
     * @return The BallState of the specified slot.
     */
    public IndexerFacade.BallState getBallState(int slotToWatch)
    {
        return null;
    }

    /**
     * Spins the turnstile motor at a specified velocity.
     *
     * @param v The velocity to spin at.
     */
    public void spin(double v)
    {

    }

    /**
     * Initializes the TransferFacade and its sub-components.
     *
     * @param hwMap The hardware map from the OpMode.
     * @param telem The telemetry object for logging.
     */
    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;

        turnstile = new LiftingTurnstile();
        turnstile.init(hwMap, telem);

        for (int i = 0; i < 2; i++)
        {
            ballSensors[i] = new BallSensor();
            ballSensors[i].init(hwMap, telem, "color" + i, i);
        }
        for (int i = 0; i < 3; i++)
        {
            ballSlots[i] = BallState.UNKNOWN;
        }

        beamBreak.init(hwMap, telem);

        updateBallModel();
        changeState(State.IDLE);
    }


    /**
     * Initiates a single ball launch sequence.
     *
     * @return true if launch was initiated.
     */
    public boolean launch()
    {
        changeState(State.LAUNCHING);
        turnstile.launchSlots(1);
        pendingShift = 1;
        ballSlots[2] = BallState.VACANT;
        updateBallCount();
        return true;
    }

    /**
     * Launches all currently loaded balls if the system is ready or overridden.
     */
    public void launchAll()
    {
        if (canLaunchAll() || override)
        {
            turnstile.launchSlots(3);
            changeState(State.LAUNCHING);
            for (int i = 0; 3 > i; i++)
            {
                ballSlots[i] = BallState.VACANT;
            }
            pendingShift = 0;
            updateBallCount();
        }
    }

    /**
     * Signals that the intake process has stopped.
     */
    public void intakeStop()
    {
        isIntaking = false;
    }

    /**
     * Signals that the intake process has started.
     */
    public void intakeStart()
    {
        isIntaking = true;
    }

    /**
     * Checks if the indexer (turnstile) has reached its maximum ball capacity (3).
     *
     * @return true if full, false otherwise.
     */
    public boolean indexerIsFull()
    {
        return ballCount >= 3;
    }

    /**
     * Checks if there is a ball currently detected in the intake.
     *
     * @return true if a ball is in the intake, false otherwise.
     */
    public boolean ballInIntake()
    {
        return beamBreak.ballInIntake();
    }

    /**
     * Checks if the system is in a state that allows launching all balls.
     *
     * @return true if launching all is allowed.
     */
    private boolean canLaunchAll()
    {
        return currentState == State.IDLE ||
                currentState == State.MOVING;
    }

    /**
     * Selects and rotates to a specific slot index.
     *
     * @param slot The target slot index (0-2).
     * @return true if the selection was valid.
     */
    public boolean selectSlot(int slot)
    {
//        telemetry.addData("Initial CurrentTargetSlot", currentTargetSlot);
        int previousSlot = currentTargetSlot;
//        telemetry.addData("Go to Slot", slot);
        slot = Math.floorMod(slot, 3);

        telemetry.addData("selectSlot: Go to Slot (after floormod)", slot);

        // Store the shift so the state machine can use it upon arrival
//        pendingShift = Math.floorMod(slot - previousSlot, 3);
        pendingShift = previousSlot - slot;
        pendingShift = 0;  // fix this...  I am temporarily testing by taking pending shift out
                           // and rotating the ball model now.

        telemetry.addData("selectSlot: Pending shift for ball model", pendingShift);

        rotateBallStates(previousSlot-slot);
        currentTargetSlot = slot;
        turnstile.seekToAngle(slot * 120);
        changeState(State.MOVING);
        return true;
    }

    /**
     * Updates the internal ball position model after a physical rotation.
     *
     * @param shiftAmt The number of slots to shift the model.
     */
    private void rotateBallStates(int shiftAmt)
    {
        BallState[] newSlots = new BallState[3];
        telemetry.addData("shiftBallStates: Shift Amount", shiftAmt);
        int[] tempRotIndex = {0, 0, 0};
        int i = 0;
        try
        {
            for (i = 0; i < 3; i++)
            {
                // Shift indices based on rotation
                int rotatedIndex = Math.floorMod(i - shiftAmt, 3);
                tempRotIndex[i] = rotatedIndex;
                if (rotatedIndex == 3)
                {
                    rotatedIndex = 0;
                }
                if (rotatedIndex == -1)
                {
                    rotatedIndex = 2;
                }
                newSlots[i] = ballSlots[rotatedIndex];
            }
            telemetry.addData("shiftBallStates: Rotated Indices", Arrays.toString(tempRotIndex));
            ballSlots = newSlots;
        }
        catch (Exception e)
        {
            throw new RuntimeException("i: " + i + " Rotated Index: " + Arrays.toString(tempRotIndex), e);
        }
    }

    /**
     * Cycles the turnstile by a specified number of slots in a given direction.
     * This updates the target slot and initiates movement.
     *
     * @param direction The number of slots to move (e.g., 1 for forward, -1 for backward).
     *                  Commonly used with -1 to advance to the next empty slot for intaking.
     * @return true if the slot selection and movement were successfully initiated.
     */
    public boolean cycle(int direction)
    {
        int goToSlot = currentTargetSlot - direction;

        if (goToSlot == -1)
        {
            goToSlot = 2;
        }
        else if (goToSlot == 3)
        {
            goToSlot = 0;
        }
        return selectSlot(goToSlot);
    }

    /**
     * Initiates the homing sequence for the turnstile.
     */
    public void home()
    {
        changeState(State.HOMING);
        turnstile.home();
    }

    /**
     * Returns a Roadrunner Action for homing the turnstile.
     *
     * @return An Action that homes the turnstile and finishes when homed.
     */
    public Action homeAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                home();
                return !turnstile.isHomed();
            }
        };
    }

    /**
     * Updates the internal model of where balls are located based on sensors.
     */
    private void updateBallModel()
    {
        // If the indexer beam break is tripped, slot 0 is now occupied
        telemetry.addData("1 updateBallModel: Ball Model in updateBallModel", Arrays.toString(ballSlots));
        if (beamBreak.ballinIndexer())
        {
            if (ballSlots[0] == BallState.UNKNOWN ||
                    ballSlots[0] == BallState.VACANT ||
                    ballSlots[0] == BallState.ALL)
            {
                ballSlots[0] = BallState.OCCUPIED;
            }
            // if it is either PURPLE or GREEN, leave it alone.
        }

        for (int i = 0; i < 2; i++)
        {
            ballSensors[i].update();
        }

        updateBallColor();
        updateBallCount();
    }

    /**
     * Updates the detected color of the ball in the loading slot (slot 2).
     */
    private boolean updateBallColor()
    {
        BallSensor sensorA = ballSensors[0];
        BallSensor sensorB = ballSensors[1];
        BallSensor.BallColor colorA = sensorA.getDetectedColor();
        BallSensor.BallColor colorB = sensorB.getDetectedColor();

        if (colorA == BallSensor.BallColor.PURPLE || colorB == BallSensor.BallColor.PURPLE)
        {
            // Priority 1: Either is Purple
            ballSlots[2] = BallState.PURPLE;
        }
        else if (colorA == BallSensor.BallColor.GREEN || colorB == BallSensor.BallColor.GREEN)
        {
            // Priority 2: Either is Green
            ballSlots[2] = BallState.GREEN;
        }
        sensorA.addTelemetry();
        sensorB.addTelemetry();
        return sensorA.isBallPresent() && sensorB.isBallPresent();

    }

    /**
     * Calculates and updates the total number of balls currently in the system.
     */
    private void updateBallCount()
    {
        int count = 0;

        telemetry.addData("1 updateBallCount: HERE IS WHAT I AM COUNTING", Arrays.toString(ballSlots));
        // 1. Check the physical slots in the turnstile
        for (BallState slot : ballSlots)
        {
            if (slot == BallState.OCCUPIED || slot == BallState.PURPLE || slot == BallState.GREEN)
            {
                count++;
            }
        }
        // 2. Check the Intake "Entryway"
        // We only count this if it hasn't been "swallowed" into slot 0 yet
        // Necessary to know if our ball count in the robot is > 3
        if (beamBreak.ballInIntake())
        {
            count++;
            telemetry.addLine("2 updateBallCount: ADDING BALL FOR INTAKE.");
        }
        telemetry.addData("3 updateBallCount: FINAL BALL COUNT", count);
        this.ballCount = count;
    }

    /**
     * Gets the color of the ball currently in the launching position (slot 2).
     *
     * @return The BallState of the launch slot.
     */
    public BallState getLaunchColor()
    {
        return ballSlots[2];
    }

    /**
     * The main update loop for the TransferFacade.
     * Handles state transitions, component updates, and automatic indexing.
     *
     * @param isAtRpm Whether the shooter/launcher is at the target RPM.
     */
    public void update(boolean isAtRpm)
    {
        turnstile.update();
        beamBreak.update();

        switch (currentState)
        {
            case HOMING:
                if (enter)
                {
                    enter = false;
                }
                else if (turnstile.isHomed())
                {
                    updateBallModel();
                    changeState(State.IDLE);
                }
                break;
            case MOVING:
                telemetry.addData("1 MOVING: Ball In Indexer", beamBreak.ballinIndexer());
                telemetry.addData("2 MOVING: Pending Shift", pendingShift);
                telemetry.addData("3 MOVING: ball model", Arrays.toString(ballSlots));
                if (enter)
                {
                    enter = false;
                }
                else if (turnstile.isAtTarget())
                {
//                    // 1. Physically arrived? Now update the software model
//                    if (pendingShift != 0)
//                    {
//                        shiftBallModel(pendingShift);
//                        pendingShift = 0; // Reset so we don't shift twice
//                    }
//                    // 2. Refresh sensors and counts for the new position
//                    updateBallModel();
                    changeState(State.IDLE);

                }
                break;
            case IDLE:
                // In position, ready to receive a command from an external source.
                // Check if the indexer beam break is triggered AND
                // the system isn't full yet.
                if (enter)
                {
                    // Refresh sensors and counts for the new position
                    telemetry.addData("1 IDLE: Ball Model upon entering IDLE", Arrays.toString(ballSlots));
                    updateBallModel();
                    telemetry.addData("2 IDLE: Just updated Ball Count upon entering IDLE.  BALL CNT", ballCount);
                    telemetry.addData("3 IDLE: New Ball Model after processing enter in IDLE", Arrays.toString(ballSlots));

                    if (pauseTimer.milliseconds() > 1000)
                    {
                        enter = false;
                    }
                    enter = false;
                }
                else if (beamBreak.ballinIndexer() && !indexerIsFull())
                {
                    // I have detected a ball in the indexer and need to automatically shift to the
                    // next slot to make room.  Before I go, I will refresh the model to
                    // register the slot as OCCUPIED and update the ballCount.
                    telemetry.addData("4 IDLE: Ball Model because I think I have a new ball in IDLE", Arrays.toString(ballSlots));
                    updateBallModel();
                    telemetry.addData("5 IDLE: Just updated Ball Count because I thought I got a new ball in IDLE.  NEW BALL CNT", ballCount);
                    telemetry.addData("6 IDLE: New Ball Model after processing the potential new ball in IDLE", Arrays.toString(ballSlots));

                    // Rotate to the next slot
                    telemetry.addLine("7 IDLE: WOULD BE AUTO-CYCLING TO NEXT SLOT.");
                    telemetry.addData("8 IDLE: Ball Count", ballCount);
                    // I have just marked the intake slot as OCCUPIED
                    cycle(-1);
                }
                else if (turnstile.getAngleError() > IDLE_TOLERANCE)
                {
                    changeState(State.MOVING);
                }
                break;
            case LAUNCHING:
                if (enter)
                {
                    enter = false;
                }
                else if (turnstile.isAtTarget())
                {
                    rotateBallStates(1);
                    updateBallModel();
                    changeState(State.IDLE);
                }
                break;
            default:
                break;
        }

        if (DEBUG_TRANSFER_FACADE || SHOW_DEBUG_ALL)
        {
            telemetry.addData("Transfer Facade State", currentState.name());
            telemetry.addData("Turnstile Current Target Slot", currentTargetSlot);
            telemetry.addData("Turnstile at Target: ", turnstile.isAtTarget());
            telemetry.addData("Beam Break detection: ", beamBreak.ballinIndexer());
            telemetry.addData("Ball Count", ballCount);
            telemetry.addLine(String.format("Slots: [0]: %s, [1]: %s, [2]: %s",
                    ballSlots[0], ballSlots[1], ballSlots[2]));
        }
    }

    private void changeState(State nextState)
    {
        enter = true;
        currentState = nextState;
        pauseTimer.reset();
    }




    // Methods needed to keep Worlds Auto Working

    /**
     * Checks if the system is currently in override mode.
     *
     * @return true if overridden, false otherwise.
     */
    public boolean isOverridden()
    {
        return override;
    }

    /**
     * Prepares the sequence for operation.
     *
     * @return true if successfully prepared.
     */
    public boolean prepSequence()
    {
        return true;
    }

    /**
     * Plans a shot sequence based on the detected AprilTag ID.
     *
     * @param aprilTagId The ID of the AprilTag to target.
     * @return A string describing the planned sequence.
     */
    public String planShotSequence(int aprilTagId)
    {
        return "No Shot Sequence Planned";
    }

    /**
     * Prepares the next slot for intaking a ball of a specific state.
     *
     * @param ballState The state of the ball to be intaken.
     * @return true if ready for the next intake.
     */
    public boolean readyNextIntakeSlot(IndexerFacade.BallState ballState)
    {
        return true;
    }

    /**
     * Adjusts the system to the third slot by homing.
     */
    public void adjustToThird()
    {
        home();
    }

    /**
     * Cancels any ongoing sequence.
     */
    public void cancelSequence()
    {
    }

    /**
     * Checks if the system is currently executing a sequence.
     *
     * @return true if in a sequence, false otherwise.
     */
    public boolean isInSequence()
    {
        return false;
    }

    /**
     * Enables or disables the launching override.
     *
     * @param active true to enable override, false to disable.
     */
    public void overrideLaunching(boolean active)
    {
        override = active;
    }

    /**
     * Gets the current operational state of the facade.
     *
     * @return The current State.
     */
    public State getCurrentState()
    {
        return currentState;
    }
}