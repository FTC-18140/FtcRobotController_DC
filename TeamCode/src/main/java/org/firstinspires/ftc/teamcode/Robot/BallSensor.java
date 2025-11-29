package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Color;

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
    public static double PRESENCE_DISTANCE_CM = 5.0;

    public static double GREEN_BALL_MIN_G = 0.1;
    public static double GREEN_BALL_MAX_R = 0.1;
    public static double PURPLE_BALL_MIN_R = 0.12;
    public static double PURPLE_BALL_MIN_B = 0.12;

    public static int GREEN_HUE_MIN = 120;
    public static int GREEN_HUE_MAX = 185;
    public static int PURPLE_HUE_MIN = 205;
    public static int PURPLE_HUE_MAX = 245;

    public static float GAIN = 2.0f;
    float[] hsv = new float[3];
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
            updateColorsToHSV();
            if (colorSensor instanceof DistanceSensor) {
                distanceCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            }

            // Re-apply gain in the loop in case it's changed from the dashboard
            //colorSensor.setGain(GAIN);

            // 2. Determine presence and color
            if (isBallPresentInternal()) {
                if (isBallColorHSV(BallColor.GREEN)) {
                    detectedColor = BallColor.GREEN;
                } else if (isBallColorHSV(BallColor.PURPLE)) {
                    detectedColor = BallColor.PURPLE;
                } else {
                    detectedColor = BallColor.NONE; // Ball is present but color is not recognized
                }
            } else {
                detectedColor = BallColor.NONE;
            }

            // 3. Telemetry (optional, for tuning)
            addTelemetry(); // Good to have this on during tuning sessions
        } else {
            telemetry.addData(sensorName + " Not initialized", 0);
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

//    private boolean isGreenRGB() {
//        return colors.green > GREEN_BALL_MIN_G && colors.red < GREEN_BALL_MAX_R;
//    }
//
//    private boolean isPurpleRGB() {
//        return colors.red > PURPLE_BALL_MIN_R && colors.blue > PURPLE_BALL_MIN_B;
//    }

    private boolean isBallColorHSV(BallColor ballColor){
        boolean isBallColor = false;
        switch(ballColor){
            case GREEN:
                isBallColor = GREEN_HUE_MIN <= hsv[0] && hsv[0] <= GREEN_HUE_MAX;
                break;
            case PURPLE:
                isBallColor = PURPLE_HUE_MIN <= hsv[0] && hsv[0] <= PURPLE_HUE_MAX;
                break;
            case NONE:
                isBallColor = !isBallPresentInternal();
                break;
        }
        return isBallColor;

    }
    private float[] updateColorsToHSV(){
        Color.RGBToHSV(
                (int)(colors.red * 255),
                (int)(colors.green * 255),
                (int)(colors.blue * 255),
                hsv
        );
        return hsv;
    }

    /** Call this from update() to see live sensor values for tuning. */
    public void addTelemetry() {
        telemetry.addLine(String.format("--- Sensor: %s ---", sensorName));
        telemetry.addData("Detected", String.format("%s (Dist: %.2f cm)", detectedColor, distanceCm));
        //telemetry.addData("R | G | B", String.format("%.3f | %.3f | %.3f", colors.red, colors.green, colors.blue));
        telemetry.addData("H | S | V", String.format("%.3f | %.3f | %.3f", hsv[0], hsv[1],hsv[2]));
        telemetry.addData("Tunable Gain", GAIN);
    }
}
