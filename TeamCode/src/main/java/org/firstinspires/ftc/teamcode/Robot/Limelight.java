package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;

import java.util.List;

@Config
public class Limelight implements DataLoggable {

    public static final double INCHES_PER_METER = 39.37008;
    Limelight3A limelight = null;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    int id = -1; // The ID number of the fiducial. Set to -1 to indicate no target.
    double x = 0; // Where it is (left-right)
    double distance = -1;
    Vector2d visionPose;

    private double lastGoodX = 0;
    private double smoothedX = 0;
    private long lastUpdateTimeNs = 0;
    private long lastGoodTargetTimeNs = 0;
    private boolean firstReading = true;


    public static double alpha = 0.4; // 0.3–0.5 good

    // Milliseconds we will continue trusting a target after it disappears
    public static long TARGET_TRUST_WINDOW_MS = 150;

    public static double MINIMUM_TARGET_AREA = 10.0; // Example value, adjust as needed

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
            throw new RuntimeException("Limelight init failed", e);
        }
        this.telemetry = telemetry;
        visionPose = null;
    }

    /**
     * Sets the limelight's pipeline to the input, sets the index variable to the input
     * @param pipeline the pipeline you want it to read
     */
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Updates the values associated with the apriltags the limelight sees
     */
    public void update(double limelightAngle, Vector2d turretOffset) {
        // Always update orientation with latest robot heading (critical for MegaTag2 accuracy)
        limelight.updateRobotOrientation((limelightAngle + 360) % 360);

        // Reset per-loop state
        id = -1;
        distance = -1;
        visionPose = null;  // or new Vector2d(0,0) if you prefer non-null

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            // No new data → rely on trust window (handled in hasTarget())
            return;
        }

        long nowNs = System.nanoTime();
        lastUpdateTimeNs = nowNs;

        // Single tag expected due to pipeline ID filter
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            LLResultTypes.FiducialResult f = fiducials.get(0);  // Safe: pipeline filters to desired ID(s)

            double area = f.getTargetArea();
            if (area > MINIMUM_TARGET_AREA) {
                id = f.getFiducialId();  // For telemetry/logging (should match your pipeline)
                x = f.getTargetXDegrees();
                lastGoodX = x;
                if (firstReading) {
                    smoothedX = x;
                    firstReading = false;
                } else {
                    smoothedX = alpha * x + (1 - alpha) * smoothedX;
                }
                distance = f.getCameraPoseTargetSpace().getPosition().z * INCHES_PER_METER;
                lastGoodTargetTimeNs = nowNs;
            }
        }

        // MegaTag2 bot pose (field-relative, MT2-corrected)
        Pose3D botpose_mt2 = result.getBotpose_MT2();
        if (botpose_mt2 != null) {
            Vector2d botPoseInches = new Vector2d(
                    botpose_mt2.getPosition().x * INCHES_PER_METER,
                    botpose_mt2.getPosition().y * INCHES_PER_METER
            );
            // Apply turret offset (assumes it's in robot frame; rotate if turret angle affects it)
            // Don't accept a pose if it's off the field.
            if (Math.abs(botPoseInches.x) < 72 && Math.abs(botPoseInches.y) < 72) {
                visionPose = botPoseInches.plus(turretOffset);
            }
            // Keep your telemetry if you like
            telemetry.addData("Botpose", botPoseInches);
            telemetry.addData("Plus Offset", visionPose);
        }
    // Reset smoothing after extended loss (check regardless of fiducial presence)
        if (msSinceLastGoodTarget() > TARGET_TRUST_WINDOW_MS * 5) {
            firstReading = true;
        }
        // Optional extra telemetry (keep or remove based on your debugging needs)
        telemetry.addData("Target Fresh?", hasTarget());
        telemetry.addData("id: ", id);
    }

    /**
     * Returns the last updated value of the apriltags degrees in the x coordinate
     * @return x-value
     */
    public double getX(){
        return smoothedX;
    }

    /**
     * Returns the id of the apriltag the limelight sees
     * @return the id number
     */
    public int id(){
        telemetry.addData("id: ",id);
        return id;
    }

    public boolean hasTarget() {
        return targetIsFresh(TARGET_TRUST_WINDOW_MS);
    }

    @Override
    public void logData(DataLogger logger) {
        logger.addField(this.smoothedX);
        logger.addField(this.lastUpdateTimeNs);
    }

    public Vector2d getMegaTagPose() {
        return visionPose;
    }

    public double getDistance() {
        return distance;
    }
    public double getLastGoodX() {
        return lastGoodX;
    }

    // Returns the elapsed time since the last reliable AprilTag detection.
    public long msSinceLastGoodTarget() {
        if (lastGoodTargetTimeNs == 0) return Long.MAX_VALUE;
        return (System.nanoTime() - lastGoodTargetTimeNs) / 1_000_000;
    }

    // Determines whether a reliable target has been observed within the specified age window.
    public boolean targetIsFresh(long maxAgeMs) {
        return msSinceLastGoodTarget() < maxAgeMs;
    }
}
