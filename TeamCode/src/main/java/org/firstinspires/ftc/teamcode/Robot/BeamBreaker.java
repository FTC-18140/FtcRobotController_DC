package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class BeamBreaker {
    private Telemetry telemetry;
<<<<<<< HEAD
    private boolean inIntake = false;
    private boolean inIndexer = false;
=======
    private boolean detected = false;
>>>>>>> origin/River_Sandbox
    DigitalChannel beamBreakIndexer = null;
    DigitalChannel beamBreakIntake = null;
    public static boolean TELEM = true;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
<<<<<<< HEAD
            beamBreakIntake = hwMap.digitalChannel.get("beamBreak");
=======
            beamBreakIndexer = hwMap.digitalChannel.get("beamBreak");
>>>>>>> origin/River_Sandbox
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find digital channel 'beamBreak'");
        }

        try {
            beamBreakIndexer = hwMap.digitalChannel.get("beamBreakIndexer");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find digital channel 'beamBreakIndexer'");
        }
    }

    public void update() {
        if(beamBreakIndexer != null) {
<<<<<<< HEAD
            inIndexer = !beamBreakIndexer.getState();
=======
            detected = !beamBreakIndexer.getState();
>>>>>>> origin/River_Sandbox
            if (TELEM) {
                telemetry.addData("Indexer Beam Break sensor triggered: ", inIndexer);
            }
        }

        if(beamBreakIntake != null) {
            inIntake = !beamBreakIntake.getState();
            if (TELEM) {
                telemetry.addData("Intake Beam Break sensor triggered: ", inIntake);
            }
        }
    }

    public boolean ballDetectedInIndexer(){return inIndexer;}
    public boolean ballDetectedInIntake(){return inIntake;}
}
