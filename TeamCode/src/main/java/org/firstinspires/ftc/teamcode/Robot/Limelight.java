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
    int id = -1; // The ID number of the fiducial. Set to -1 to indicate no target.
    double x = 0; // Where it is (left-right)
    double y = 0; // Where it is (up-down)
    double distance = 0;
    int index = 1;


    private static final double MINIMUM_TARGET_AREA = 10.0; // Example value, adjust as needed
    private boolean validResults;

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
    public void setPipeline(int pipeline){
        index = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Updates the values associated with the apriltags the limelight sees
     */
    public void update() {
        validResults = false;
        id = -1; // Reset ID to -1 at the start of every loop.

        LLResult result = limelight.getLatestResult();
        if ( result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                // If we see any fiducials, take the first one in the list.
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                id = fiducial.getFiducialId(); // The ID number of the fiducial
                x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                distance = fiducial.getCameraPoseTargetSpace().getPosition().z;
                validResults = true;
            }
        }
    }

    /**
     * Returns the last updated value of the apriltags degrees in the x coordinate
     * @return
     */
    public double getX(){
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
        setPipeline(pipeline);
        return id;
    }

    public boolean hasTarget() {
        return validResults;
    }
    /*
    public static double tx(int pipeline){
        limelight.pipelineSwitch(pipeline);
        LLResult result = limelight.getLatestResult();

        return result.getTx();
    }
    */
}
