package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BeamBreakSystem
{
    private Telemetry telemetry = null;
    private boolean inIntake = false;
    private boolean inIndexer = false;
    private DigitalChannel beamBreakIndexer = null;
    private DigitalChannel beamBreakIntake = null;
    private static boolean TELEM = false;

    private int intakeCount = 0;
    private int indexerCount = 0;

    public static int BREAK_COUNT = 5;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
            beamBreakIntake = hwMap.digitalChannel.get("beamBreak");
        } catch (RuntimeException e) {
            telemetry.addData("Error", "Could not find digital channel 'beamBreak'");
        }
        try {
            beamBreakIndexer = hwMap.digitalChannel.get("beamBreakIndexer");
        } catch (RuntimeException e) {
            telemetry.addData("Error", "Could not find digital channel 'beamBreakIndexer'");
        }
    }

    public void update() {
        if ( beamBreakIntake != null ) {
            boolean triggered = !beamBreakIntake.getState();
            if ( triggered )
            {
                intakeCount = Math.min(intakeCount + 1, BREAK_COUNT);
                inIntake = (intakeCount == BREAK_COUNT);
            }
            else
            {
                intakeCount = Math.max(intakeCount - 1, 0);
                inIntake = (intakeCount == 0);
            }
            if (TELEM)
            {
                telemetry.addData("Intake Beam Break sensor triggered: ", inIntake);
            }
        }
        if ( beamBreakIndexer != null ) {
            boolean triggered = !beamBreakIndexer.getState();
            if ( triggered )
            {
                indexerCount = Math.min(indexerCount + 1, BREAK_COUNT);
                inIndexer = (indexerCount == BREAK_COUNT);
            }
            else
            {
                indexerCount = Math.max(indexerCount - 1, 0);
                inIndexer = (indexerCount == 0);
            }
            if (TELEM) {
                telemetry.addData("Indexer Beam Break sensor triggered: ", inIndexer);
            }
        }
    }

    boolean isBallDetectedInIntake() {
        return inIntake;
    }

    boolean isBallDetectedInIndexer() {
        return inIndexer;
    }
}
