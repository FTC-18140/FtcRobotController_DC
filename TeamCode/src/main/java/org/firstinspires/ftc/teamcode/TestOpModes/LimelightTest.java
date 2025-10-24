package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@TeleOp
public class LimelightTest extends OpMode {
    Limelight3A limelight = null;

    @Override
    public void init() {

        try
        {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            //limelight.pipelineSwitch(4);
            limelight.start();
        }
        catch (Exception e)
        {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            double distance = fiducial.getCameraPoseTargetSpace().getPosition().z;
            telemetry.addData("Fiducial: ", id);
            telemetry.addData("x: ",x);
            telemetry.addData("y: ",y);
            telemetry.addData("dist: ", distance);
        }

    }
}
