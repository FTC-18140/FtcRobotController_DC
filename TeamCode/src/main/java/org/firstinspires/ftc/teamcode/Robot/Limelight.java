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
    int id = 0; // The ID number of the fiducial
    double x = 0; // Where it is (left-right)
    double y = 0; // Where it is (up-down)
    double distance = 0;
    int index = 1;


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

    /**
     * Sets the limelight's pipeline to the input, sets the index variable to the input
     * @param pipeline
     */
    public void SetPipeline(int pipeline){
        index = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Updates the values associated with the apriltags the limelight sees
     */
    public void update(){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        limelight.pipelineSwitch(index);
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
            x = fiducial.getTargetXDegrees(); // Where it is (left-right)
            y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            distance = fiducial.getCameraPoseTargetSpace().getPosition().z;
//            telemetry.addData("Fiducial: ", id);
//            telemetry.addData("x: ", x);
//            telemetry.addData("y: ", y);
//            telemetry.addData("dist: ", distance);
        }
    }

    /**
     * Returns the last updated value of the apriltags degrees in the x coordinate
     * @return
     */
    public double xdegrees(){
        update();
        telemetry.addData("xdegrees: ", x);
        return x;
    }

    /**
     * Returns the id of the apriltag the limelight sees
     * @return the id number
     */
    public int id(){
        telemetry.addData("id: ",id);
        return id;
    }
    /**
     * changes the pipeline
     * reads whatever id is in front of the limelight
     *
     * @param pipeline the pipeline you want it to read
     * @return returns the id
     */
    public int limelightId(int pipeline){
        SetPipeline(pipeline);
        return id;
    }
    /*
    public static double tx(int pipeline){
        limelight.pipelineSwitch(pipeline);
        LLResult result = limelight.getLatestResult();

        return result.getTx();
    }
    */
}