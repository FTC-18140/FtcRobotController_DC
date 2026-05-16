package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_BEAMBREAK;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class BeamBreaker {
    private Telemetry telemetry = null;
    private boolean inIntake = false;
    private boolean inIndexer = false;
    private DigitalChannel beamBreakIndexer = null;
    private DigitalChannel beamBreakIntake = null;

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
        if (null != beamBreakIntake) {
            inIntake = !beamBreakIntake.getState();
            if (DEBUG_BEAMBREAK || SHOW_DEBUG_ALL) {
                telemetry.addData("Intake Beam Break sensor triggered: ", inIntake);
            }
        }
        if (null != beamBreakIndexer) {
            inIndexer = !beamBreakIndexer.getState();
            if (DEBUG_BEAMBREAK || SHOW_DEBUG_ALL) {
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
