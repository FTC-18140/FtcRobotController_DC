package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Manages a single REV Color Sensor V3 for ball detection and color identification.
 */
@Config // Annotation to make this class tunable via FTC Dashboard
public class BallSensor {

    /** Enum to represent the detected color of a ball. */
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE // Used when no ball is detected
    }

    private NormalizedColorSensor colorSensor;
    private Telemetry telemetry;
    private String sensorName;

    // --- Tunable Constants via FTC Dashboard ---
    public static double PRESENCE_DISTANCE_CM = 4.0;
    public static double GREEN_BALL_MIN_G = 0.1;
    public static double GREEN_BALL_MAX_R = 0.1;
    public static double PURPLE_BALL_MIN_R = 0.12;
    public static double PURPLE_BALL_MIN_B = 0.12;
    public static float GAIN = 2.0f;

    // --- Cached Hardware Values ---
    private NormalizedRGBA colors;
    private double distanceCm;
    private BallColor detectedColor = BallColor.NONE;

    public void init(HardwareMap hwMap, Telemetry telem, String sensorName) {
        this.telemetry = telem;
        this.sensorName = sensorName;
        try {
            colorSensor = hwMap.get(NormalizedColorSensor.class, sensorName);
            colorSensor.setGain(GAIN);
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find BallSensor: " + sensorName);
        }
    }

    /** Reads sensor values and determines ball color. Call this once per loop. */
    public void update() {
        // Refactored to have a single exit point
        if (colorSensor != null) {
            // 1. Cache hardware reads
            colors = colorSensor.getNormalizedColors();
            if (colorSensor instanceof DistanceSensor) {
                distanceCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            }

            // Re-apply gain in the loop in case it's changed from the dashboard
            colorSensor.setGain(GAIN);

            // 2. Determine presence and color
            if (isBallPresentInternal()) {
                if (isGreen()) {
                    detectedColor = BallColor.GREEN;
                } else if (isPurple()) {
                    detectedColor = BallColor.PURPLE;
                } else {
                    detectedColor = BallColor.NONE; // Ball is present but color is not recognized
                }
            } else {
                detectedColor = BallColor.NONE;
            }

            // 3. Telemetry (optional, for tuning)
            addTelemetry(); // Good to have this on during tuning sessions
        }
    }

    /** Returns the color of the ball detected during the last update(). */
    public BallColor getDetectedColor() {
        return detectedColor;
    }

    /** Returns true if a ball was detected during the last update(). */
    public boolean isBallPresent() {
        return detectedColor != BallColor.NONE;
    }

    // --- Internal Helper Methods ---

    private boolean isBallPresentInternal() {
        return distanceCm < PRESENCE_DISTANCE_CM;
    }

    private boolean isGreen() {
        return colors.green > GREEN_BALL_MIN_G && colors.red < GREEN_BALL_MAX_R;
    }

    private boolean isPurple() {
        return colors.red > PURPLE_BALL_MIN_R && colors.blue > PURPLE_BALL_MIN_B;
    }

    /** Call this from update() to see live sensor values for tuning. */
    public void addTelemetry() {
        telemetry.addLine(String.format("--- Sensor: %s ---", sensorName));
        telemetry.addData("Detected", String.format("%s (Dist: %.2f cm)", detectedColor, distanceCm));
        telemetry.addData("R | G | B", String.format("%.3f | %.3f | %.3f", colors.red, colors.green, colors.blue));
        telemetry.addData("Tunable Gain", GAIN);
    }
}
