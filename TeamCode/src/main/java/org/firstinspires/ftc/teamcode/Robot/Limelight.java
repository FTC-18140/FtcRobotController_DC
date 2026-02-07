package org.firstinspires.ftc.teamcode.Robot;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private boolean firstReading = true;

    private double latestDeltaFrameTime = 0;
    private ElapsedTime frameTimer = new ElapsedTime( ElapsedTime.Resolution.MILLISECONDS);

    public static boolean TELEM = true;

    public static double alpha = 0.4; // 0.3–0.5 good

    // Milliseconds we will continue trusting a target after it disappears
    public static long TARGET_TRUST_WINDOW_MS = 150;

    public static double MINIMUM_TARGET_AREA = 0.0; // Example value, adjust as needed

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
        frameTimer.reset();

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
                latestDeltaFrameTime = frameTimer.milliseconds();
                frameTimer.reset();
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
            addTelemetry("Botpose", botPoseInches);
            addTelemetry("Plus Offset", visionPose);
        }
    // Reset smoothing after extended loss (check regardless of fiducial presence)
        if (msSinceLastGoodTarget() > TARGET_TRUST_WINDOW_MS * 5) {
            firstReading = true;
        }
        addTelemetry("msSinceLastGoodTarget: ", msSinceLastGoodTarget());
        // Optional extra telemetry (keep or remove based on your debugging needs)
        addTelemetry("Target Fresh?", hasTarget());
        addTelemetry("id: ", id);
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
    public double msSinceLastGoodTarget() {
        if (latestDeltaFrameTime == 0)
        {
            return Long.MAX_VALUE;
        }
        return latestDeltaFrameTime;
    }

    // Determines whether a reliable target has been observed within the specified age window.
    public boolean targetIsFresh(double maxAgeMs) {
        boolean fresh = msSinceLastGoodTarget() < maxAgeMs;

        // Corrected logic: Alert when NOT fresh and we used to have a target
        if (!fresh && latestDeltaFrameTime != 0 && TELEM) {
            telemetry.addLine("[Limelight] -------------- LIMELIGHT LOST --------------");
        }
        return fresh;
    }

    /**
     * Generic method for Objects
     */
    public void addTelemetry(String name, Object value) {
        if (TELEM) {
            // [TAG   ] (6 chars) + Name (15 chars)
            // %-6.6s  -> Exactly 6 chars, Left Aligned
            // %-15.15s -> Exactly 10 chars, Left Aligned
            String tag = String.format("[%-6.6s]", this.getClass().getSimpleName().toUpperCase());
            String label = String.format("%-10.10s", name);

            telemetry.addData(tag + " " + label, value);
        }
    }

    /**
     * Overloaded method for Doubles (fixed precision + fixed label width)
     */
    public void addTelemetry(String name, double value) {
        if (TELEM) {
            String tag = String.format("[%-6.6s]", this.getClass().getSimpleName().toUpperCase());
            String label = String.format("%-10.10s", name);

            // %10.4f ensures the number itself doesn't jitter
            telemetry.addData(tag + " " + label, String.format("%10.4f", value));
        }
    }
}
