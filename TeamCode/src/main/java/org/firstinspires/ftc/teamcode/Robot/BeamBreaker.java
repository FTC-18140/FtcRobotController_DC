package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class BeamBreaker {
    private Telemetry telemetry;
    private boolean detected = false;
    DigitalChannel beamBreakIndexer = null;
    DigitalChannel beamBreakIntake = null;
    public static boolean TELEM = true;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
            beamBreakIndexer = hwMap.digitalChannel.get("beamBreak");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find digital channel 'beamBreak'");
        }
    }

    public void update() {
        if(beamBreakIndexer != null) {
            detected = !beamBreakIndexer.getState();
            if (TELEM) {
                telemetry.addData("Beam Break sensor triggered: ", detected);
            }
        }
    }

    public boolean ballDetected(){return detected;}
}
