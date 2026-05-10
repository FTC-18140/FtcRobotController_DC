package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TransferFacade
{
    private LiftingTurnstile turnstile = null;
    private BallSensor[] ballSensors = new BallSensor[2];
    private BeamBreakSystem beamBreak = new BeamBreakSystem();
    private Telemetry telemetry = null;

    public static boolean TELEM = false;
    private int currentTargetSlot;

    public enum State {UNKOWNN, HOMING, IDLE, MOVING, LAUNCHING}
    private State currentState = State.IDLE;

    public enum BallState {GREEN, PURPLE, VACANT, ALL}
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

    public boolean isOverridden() {
        return override;
    }

    public void launchAllInIndexer() {
        if (canLaunchAll() || override) {
            turnstile.launchSlots(3);
            currentState = State.LAUNCHING;
            for (int i = 0; 3 > i; i++) {
                ballSlots[i] = BallState.VACANT;
            }
        }

    }
    public void intakeStop() {
        isIntaking = false;
//        turnstile.intakeStop();
    }

    public void intake() {
        isIntaking = true;
//        turnstile.intakeStart();
    }
    public boolean indexerIsFull() {
        return ballCount >= 3;
    }

    private boolean canLaunchAll() {
        return  currentState == State.IDLE ||
                currentState == State.MOVING;
    }

    public boolean selectSlot(int slot)
    {
        slot = Math.floorMod(slot, 3);

        currentTargetSlot = slot;
        turnstile.seekToAngle(slot*120);  // slots are 120 degrees apart
        currentState = State.MOVING;
        return true;
    }

    public boolean cycle(int direction)
    {
        int goToSlot = currentTargetSlot + direction;

        if (goToSlot == -1)
        {
            goToSlot = 2;
        }
        else if ( goToSlot == 3 )
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

    public void updateBallModel()
    {
        for (int i = 0; 2 > i; i++)
        {
            ballSensors[i].update();
        }
        updateBallStates();
        updateBallCount();

    }
    private void updateBallStates()
    {
        BallSensor sensorA = ballSensors[0];
        BallSensor sensorB = ballSensors[1];
        BallSensor.BallColor colorA = sensorA.getDetectedColor();
        BallSensor.BallColor colorB = sensorB.getDetectedColor();

        if ( colorA == BallSensor.BallColor.PURPLE || colorB == BallSensor.BallColor.PURPLE )
        {
            // Priority 1: Either is Purple
            ballSlots[2] = BallState.PURPLE;
        }
        else if ( colorA == BallSensor.BallColor.GREEN || colorB == BallSensor.BallColor.GREEN )
        {
            // Priority 2: Either is Green
            ballSlots[2] = BallState.GREEN;
        }
        else
        {
            // Default: Both NONE or mixed Green/None
            ballSlots[2] = BallState.VACANT;
        }
        sensorA.addTelemetry();
        sensorB.addTelemetry();
    }

    private void updateBallCount()
    {
        ballCount = 0;
        if ( beamBreak.isBallDetectedInIndexer() )
        {
            ballCount++;
        }
        if ( ballSlots[1] != BallState.VACANT )
        {
            ballCount++;
        }
        if ( ballSlots[2] != BallState.VACANT )
        {
            ballCount++;
        }
        if ( beamBreak.isBallDetectedInIntake() )
        {
            ballCount++;
        }

    }

    public void update(boolean isAtRpm)
    {
        turnstile.update(ballCount);
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
                    updateBallModel();
                    currentState = State.IDLE;
                }
                break;
            case IDLE: // In position, ready to receive a command from an external source.
                if (!turnstile.isAtTarget())
                {
                    currentState = State.MOVING;
                }
                break;
            case LAUNCHING:
                if (turnstile.isAtTarget())
                {
                    currentState = State.MOVING;
                }
                break;
            default:
                break;
        }
    }
}
