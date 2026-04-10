package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.Size;

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

    private static final double HSV_CONST = 60.0;

    /**
     * Enum to represent the detected color of a ball.
     */
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE // Used when no ball is detected
    }

    private NormalizedColorSensor colorSensor = null;
    private Telemetry telemetry = null;
    private String sensorName = null;

    private static boolean TELEM = false;

    // --- Tunable Constants via FTC Dashboard ---

    private static final int GREEN_HUE_MIN = 110;
    private static final int GREEN_HUE_MAX = 160;
    private static final int PURPLE_HUE_MIN = 185;
    private static final int PURPLE_HUE_MAX = 245;


    private static double[] presenceDistances = {4.0, 4.0, 4.2, 6.7, 4.0, 4.0};
    private int id = 0;

    private static final float GAIN = 2.0f;
    private final float[] hsv = new float[3];
    // --- Cached Hardware Values ---
    private NormalizedRGBA colors = null;
    private double distanceCm = 0.0;
    private BallColor detectedColor = BallColor.NONE;

    public void init(HardwareMap hwMap, Telemetry telem, String sensorName, int id) {
        telemetry = telem;
        this.sensorName = sensorName;
        this.id = id;
        try {
            colorSensor = hwMap.get(NormalizedColorSensor.class, sensorName);
            colorSensor.setGain(GAIN);
        } catch (RuntimeException e) {
            telemetry.addData("Error", "Could not find BallSensor: " + sensorName);
        }
    }

    /**
     * Reads sensor values and determines ball color. Call this once per loop.
     */
    public void update() {
        // Refactored to have a single exit point
        if (null != colorSensor) {
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
                if (isBallColorHSV(BallColor.PURPLE)) {
                    detectedColor = BallColor.PURPLE;
                } else {
                    detectedColor = BallColor.GREEN;
                }
            } else {
                detectedColor = BallColor.NONE;
            }

            // 3. Telemetry (optional, for tuning)
            //addTelemetry(); // Good to have this on during tuning sessions
        } else {
            telemetry.addData(sensorName + " Not initialized", 0);
        }
        addTelemetry();
    }

    /**
     * Returns the color of the ball detected during the last update().
     */
    BallColor getDetectedColor() {
        return detectedColor;
    }

    /**
     * Returns true if a ball was detected during the last update().
     */
    public boolean isBallPresent() {
        return BallColor.NONE != detectedColor;
    }

    // --- Internal Helper Methods ---

    private boolean isBallPresentInternal() {
        return distanceCm < presenceDistances[id];
    }

    private boolean isBallColorHSV(BallColor ballColor) {
        boolean isBallColor = false;
        switch (ballColor) {
            case GREEN:
                isBallColor = (GREEN_HUE_MIN <= hsv[0]) && (GREEN_HUE_MAX >= hsv[0]);

                break;
            case PURPLE:
                isBallColor = (PURPLE_HUE_MIN <= hsv[0]) && (PURPLE_HUE_MAX >= hsv[0]);
                break;
            case NONE:
                isBallColor = !isBallPresentInternal();
                break;
        }
        return isBallColor;

    }

    private static void colorToHSV(double red, double green, double blue, @Size(3L) float[] hsv) {
        double greenBlueMax = Math.max(green, blue);
        double colorMax = Math.max(red, greenBlueMax);
        double greenBlueMin = Math.min(green, blue);
        double colorMin = Math.min(red, greenBlueMin);
        double delta = colorMax - colorMin;
        float hue = 0;
        float saturation;
        float value = (float) colorMax;
        if (0 == delta) {
            hue = 0;
        } else if (colorMax == red) {
            hue = (float) (HSV_CONST * ((((green - blue) / delta)) % 6.0));
        } else if (colorMax == green) {
            hue = (float) (HSV_CONST * ((((blue - red) / delta)) + 2.0));
        } else if (colorMax == blue) {
            hue = (float) (HSV_CONST * ((((red - green) / delta)) + 4.0));
        }
        saturation = 0 == colorMax ? 0 : (float) (delta / colorMax);
        hsv[0] = hue;
        hsv[1] = saturation;
        hsv[2] = value;
    }

    private float[] updateColorsToHSV() {
        colorToHSV(colors.red, colors.green, colors.blue, hsv);
        return hsv;
    }

    /**
     * Call this from update() to see live sensor values for tuning.
     */

    void addTelemetry() {
        if (!TELEM) return;
        String format = String.format("--- Sensor: %s ---", sensorName);
        telemetry.addLine(format);
//        telemetry.addData("Device Info", String.format("(Name: %1s, Version: %2s)", colorSensor.getDeviceName(), colorSensor.getVersion()));
        telemetry.addData("Detected", String.format("%s (Dist: %.2f cm, Hue: %.4f, Saturation: %.4f, Value: %.4f, Alpha: %.4f)", detectedColor, distanceCm, hsv[0], hsv[1], hsv[2], colors.alpha));
        //telemetry.addData("R | G | B", String.format("%.3f | %.3f | %.3f", colors.red, colors.green, colors.blue));
        //telemetry.addData("H | S | V", String.format("%.3f | %.3f | %.3f", hsv[0], hsv[1],hsv[2]));
        //telemetry.addData("Tunable Gain", GAIN);
    }
}
