package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_TRANSFER_FACADE;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TransferFacade
{
    private LiftingTurnstile turnstile = null;
    private BallSensor[] ballSensors = new BallSensor[2];
    private BeamBreakSystem beamBreak = new BeamBreakSystem();
    private Telemetry telemetry = null;

    public static boolean TELEM = false;
    private int currentTargetSlot;
    private int pendingShift = 0;
    public static double IDLE_TOLERANCE = 15 ;

    public void updateBallSensors() {
    }

    public void updateBallStates() {
    }

    public void selectNextSlot(IndexerFacade.BallState ballState) {
    }

    public int getCurrentTargetSlot() {
        return 1;
    }

    public IndexerFacade.BallState getBallState(int slotToWatch) {
        return null;
    }

    public void spin(double v) {

    }

    public enum State
    {UNKOWNN, HOMING, IDLE, MOVING, LAUNCHING}

    private State currentState = State.IDLE;

    public enum BallState
    {GREEN, PURPLE, VACANT, OCCUPIED, ALL}

    private BallState[] ballSlots = new BallState[3];

    // Fields/Variables to support Auto
    private boolean override = false;
    private boolean isIntaking = false;
    private int ballCount = 0;


    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;

        turnstile = new LiftingTurnstile();
        turnstile.init(hwMap, telem);

        for (int i = 0; 2 > i; i++)
        {
            ballSensors[i] = new BallSensor();
            ballSensors[i].init(hwMap, telem, "color" + i, i);
        }
        for (int i = 0; 3 > i; i++)
        {
            ballSlots[i] = BallState.VACANT;
        }

        beamBreak.init(hwMap, telem);

        updateBallModel();

        currentState = State.IDLE;
    }

    // Methods needed to keep Worlds Auto Working
    public boolean isOverridden()
    {
        return override;
    }

    public boolean prepSequence()
    {
        return true;
    }

    public String planShotSequence(int aprilTagId)
    {
        return "No Shot Sequence Planned";
    }

    public boolean readyNextIntakeSlot(IndexerFacade.BallState ballState) { return true; }

    public void adjustToThird() { home(); }

    public void cancelSequence() { }
    public boolean isInSequence() { return false; }

    public void overrideLaunching(boolean active) {
        override = active;
    }
    public State getCurrentState() { return currentState; }

    public boolean launch()
    {
        currentState = State.LAUNCHING;
        turnstile.launchSlots(1);
        pendingShift = 1;
        ballSlots[2] = BallState.VACANT;
        updateBallCount();
        return true;
    }

    public void launchAllInIndexer()
    {
        if (canLaunchAll() || override)
        {
            turnstile.launchSlots(3);
            currentState = State.LAUNCHING;
            for (int i = 0; 3 > i; i++)
            {
                ballSlots[i] = BallState.VACANT;
            }
            pendingShift = 0;
            updateBallCount();
        }
    }

    public void intakeStop()
    {
        isIntaking = false;
    }

    public void intake()
    {
        isIntaking = true;
    }

    public boolean indexerIsFull()
    {
        return ballCount >= 3;
    }

    public boolean ballInIntake() { return beamBreak.isBallDetectedInIntake();}

    private boolean canLaunchAll()
    {
        return currentState == State.IDLE ||
                currentState == State.MOVING;
    }

    public boolean selectSlot(int slot)
    {
        int previousSlot = currentTargetSlot;
        slot = Math.floorMod(slot, 3);

        // Store the shift so the state machine can use it upon arrival
        pendingShift = Math.floorMod(slot - previousSlot, 3);

        currentTargetSlot = slot;
        turnstile.seekToAngle(slot * 120);
        currentState = State.MOVING;
        return true;
    }

    private void shiftBallModel(int shift)
    {
        BallState[] newSlots = new BallState[3];
        for (int i = 0; i < 3; i++)
        {
            // Shift indices based on rotation
            newSlots[Math.floorMod(i + shift, 3)] = ballSlots[i];
        }
        ballSlots = newSlots;
    }

    public boolean cycle(int direction)
    {
        int goToSlot = currentTargetSlot + direction;

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

    public void home()
    {
        currentState = State.HOMING;
        turnstile.home();
    }
    public Action homeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                home();
                return !turnstile.isHomed();
            }
        };
    }
    public void updateBallModel()
    {
        // If the indexer beam break is tripped, slot 0 is now occupied
        if (beamBreak.isBallDetectedInIndexer())
        {
            ballSlots[0] = BallState.OCCUPIED;
        }

        for (int i = 0; i < 2; i++)
        {
            ballSensors[i].update();
        }
        updateBallColor();
        updateBallCount();
    }

    private void updateBallColor()
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
    }

    private void updateBallCount()
    {
        int count = 0;

        // 1. Check the physical slots in the turnstile
        for (BallState slot : ballSlots)
        {
            if (slot != BallState.VACANT)
            {
                count++;
            }
        }

        // 2. Check the Intake "Entryway"
        // We only count this if it hasn't been "swallowed" into slot 0 yet
        if (beamBreak.isBallDetectedInIntake())
        {
            count++;
        }

        this.ballCount = count;
    }

    public BallState getLaunchColor() { return ballSlots[2]; }

    public void update(boolean isAtRpm)
    {
        turnstile.update();
        beamBreak.update();

        switch (currentState)
        {
            case HOMING:
                if (turnstile.isHomed())
                {
                    updateBallModel();
                    currentState = State.IDLE;
                }
                break;
            case MOVING:
                if (turnstile.isAtTarget())
                {
                    // 1. Physically arrived? Now update the software model
                    if (pendingShift != 0)
                    {
                        shiftBallModel(pendingShift);
                        pendingShift = 0; // Reset so we don't shift twice
                    }
                    // 2. Refresh sensors and counts for the new position
                    updateBallModel();
                    currentState = State.IDLE;
                }
                break;
            case IDLE: // In position, ready to receive a command from an external source.
                // Check if the indexer beam break is triggered AND
                // the system isn't full yet.
                if (beamBreak.isBallDetectedInIndexer() && !indexerIsFull())
                {
                    // Refresh the model to recognize the new ball
                    updateBallModel();

                    // Rotate to the next slot
                    cycle(-1);
                }
                else if (turnstile.getAngleError() > IDLE_TOLERANCE)
                {
                    currentState = State.MOVING;
                }
                break;
            case LAUNCHING:
                if (turnstile.isAtTarget())
                {
                    shiftBallModel(1);
                    updateBallModel();
                    currentState = State.IDLE;
                }
                break;
            default:
                break;
        }

        if (DEBUG_TRANSFER_FACADE || SHOW_DEBUG_ALL)
        {
            telemetry.addData("Transfer Facade State", currentState.name());
            telemetry.addData("Turnstile at Target: ", turnstile.isAtTarget());
            telemetry.addData("Beam Break detection: ", beamBreak.isBallDetectedInIndexer());
            telemetry.addLine(String.format("Slots: [0]: %s, [1]: %s, [2]: %s",
                    ballSlots[0], ballSlots[1], ballSlots[2]));
        }

    }
}
