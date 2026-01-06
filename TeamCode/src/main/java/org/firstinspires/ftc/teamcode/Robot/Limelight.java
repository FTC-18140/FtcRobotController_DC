package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;

import java.util.List;


public class Limelight implements DataLoggable {

    public static final double INCHES_PER_METER = 39.3701;
    Limelight3A limelight = null;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    int id = -1; // The ID number of the fiducial. Set to -1 to indicate no target.
    double x = 0; // Where it is (left-right)
    double y = 0; // Where it is (up-down)
    double distance = -1;
    int index = 1;

    Pose2d visionPose;


    private static final double MINIMUM_TARGET_AREA = 10.0; // Example value, adjust as needed
    private boolean validResults = false;

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
        visionPose = new Pose2d(0,0,0);
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
        id = -1; // Reset ID to -1 at the start of every loop.
        distance = -1;
        visionPose = null;
        validResults = false;

        LLResult result = limelight.getLatestResult();
        if ( result != null && result.isValid()) {
            validResults = true;
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                // If we see any fiducials, take the first one in the list.
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                id = fiducial.getFiducialId(); // The ID number of the fiducial
                x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                y = fiducial .getTargetYDegrees(); // Where it is (up-down)
                distance = fiducial.getCameraPoseTargetSpace().getPosition().z * INCHES_PER_METER;
            }

            // Grab the MegaTag2 results to use for localization
            if (result.getBotposeTagCount() > 0) {
                Pose3D fullBotPose = result.getBotpose_MT2();

                double x_inches = fullBotPose.getPosition().x * INCHES_PER_METER;
                double y_inches = fullBotPose.getPosition().y * INCHES_PER_METER;
                double heading_radians = fullBotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                visionPose = new Pose2d(x_inches, y_inches, heading_radians);
            }

        }
        telemetry.addData("validResults", validResults);
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

    public double getDistanceToTarget() {
        return distance;
    }

    @Override
    public void logData(DataLogger logger) {
        logger.addField(this.x);
    }

    public Pose2d getMegaTagPose() {
        return visionPose;
    }

}
