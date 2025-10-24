package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


public class Limelight {

    Limelight3A limelight = null;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    private static final double MINIMUM_TARGET_AREA = 10.0; // Example value, adjust as needed

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        hardwareMap = hwMap;
        try
        {
            limelight = hwMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(4);
            limelight.start();
        }
        catch (Exception e)
        {
            throw new RuntimeException(e);
        }
        this.telemetry = telemetry;
    }
    public void update(){
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
    /*
    public static double tx(int pipeline){
        limelight.pipelineSwitch(pipeline);
        LLResult result = limelight.getLatestResult();

        return result.getTx();
    }
    */
}