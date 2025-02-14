package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * This class provides methods for interacting with the Limelight camera and processing its results.
 */
public class LimelightVision
{

    private Limelight3A limelight;
    private Telemetry telemetry;

    public static final double SPECIMEN_X = 36 / 39.37008;
    private static final double DEFAULT_TARGET_X = 0.0;
    private static final double MINIMUM_TARGET_AREA = 10.0; // Example value, adjust as needed

    /**
     * Initializes the Limelight camera and configures its pipeline.
     *
     * This method performs the following actions:
     * 1. Retrieves the Limelight 3A camera from the hardware map.
     * 2. Sets the Limelight's pipeline to pipeline number 1 (color pipeline).
     * 3. Starts the Limelight's processing.
     * 4. Stores the provided Telemetry object for later use.
     *
     * If any error occurs during the initialization process (e.g., Limelight not found),
     * a RuntimeException is thrown, indicating a critical failure in setup.
     *
     * @param hwMap The hardware map provided by the FTC SDK, containing the robot's hardware devices.
     * @param telemtry The telemetry object for sending data back to the Driver Station.
     * @throws RuntimeException If an error occurs during Limelight initialization.
     */
    public void init(HardwareMap hwMap, Telemetry telemtry)
    {
        try
        {
            limelight = hwMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(3); // color pipeline
            limelight.start();
        }
        catch (Exception e)
        {
            throw new RuntimeException(e);
        }
        this.telemetry = telemtry;
    }

    /**
     * Retrieves the target's X-coordinate (horizontal offset) from the Limelight camera's results.
     *
     * @return The target's X-coordinate in degrees, or DEFAULT_TARGET_X if no valid target is found.
     */
    public double updateHeading(double h){
        LLResult result = limelight.getLatestResult();

        // Early exit if no valid result is available.
        if (!isValidResult(result))
        {
            telemetry.addData("Limelight", "No valid result found.");
            return DEFAULT_TARGET_X;
        }

        // Log basic Limelight data.
        logLimelightData(result);

        // Find the best color result based on target area.
        Pose3D botpose = result.getBotpose();
        double offset = botpose.getOrientation().getYaw();
        telemetry.addData("limelight return position Angle: ", offset);

        return -90 - offset;
    }
    public double getTargetX()
    {
        LLResult result = limelight.getLatestResult();

        // Early exit if no valid result is available.
        if (!isValidResult(result))
        {
            telemetry.addData("Limelight", "No valid result found.");
            return DEFAULT_TARGET_X;
        }

        // Log basic Limelight data.
        logLimelightData(result);

        // Find the best color result based on target area.
        Pose3D botpose = result.getBotpose();
        double offset = botpose.getPosition().x;
        telemetry.addData("limelight return position X: ", offset);
        telemetry.addData("limelight return position X-inches: ", offset*39.37008);
        //LLResultTypes.ColorResult bestColorResult = findBestColorResult(result);

//        if (result != null)
//        {
            return SPECIMEN_X - offset;
//        }
//        else
//        {
//            telemetry.addData("Limelight", "No target found.");
//            return DEFAULT_TARGET_X;
//        }
    }

    public double getTargetXBlue()
    {
        LLResult result = limelight.getLatestResult();

        // Early exit if no valid result is available.
        if (!isValidResult(result))
        {
            telemetry.addData("Limelight", "No valid result found.");
            return DEFAULT_TARGET_X;
        }

        // Log basic Limelight data.
        logLimelightData(result);

        // Find the best color result based on target area.
        Pose3D botpose = result.getBotpose();
        double offset = botpose.getPosition().x;
        telemetry.addData("limelight return position X: ", offset);
        telemetry.addData("limelight return position X-inches: ", offset*39.37008);
        //LLResultTypes.ColorResult bestColorResult = findBestColorResult(result);

//        if (result != null)
//        {
        return -SPECIMEN_X - offset;
//        }
//        else
//        {
//            telemetry.addData("Limelight", "No target found.");
//            return DEFAULT_TARGET_X;
//        }
    }

    /**
     * Checks if the Limelight result is valid.
     *
     * @param result The Limelight result to check.
     * @return True if the result is valid, false otherwise.
     */
    private boolean isValidResult(LLResult result)
    {
        return result != null && result.isValid();
    }

    /**
     * Logs basic data from the Limelight result to telemetry.
     *
     * @param result The Limelight result to log.
     */
    private void logLimelightData(LLResult result)
    {
        Pose3D botpose = result.getBotpose(); // Consider using this if needed
        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
    }

    /**
     * Finds the best color result from the Limelight result based on target area.
     *
     * @param result The Limelight result containing color results.
     * @return The best color result, or null if no suitable result is found.
     */
    private LLResultTypes.ColorResult findBestColorResult(LLResult result)
    {
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

        if (colorResults == null || colorResults.isEmpty())
        {
            telemetry.addData("Limelight", "No color results found.");
            return null;
        }

        LLResultTypes.ColorResult bestResult = null;
        for (LLResultTypes.ColorResult cr : colorResults)
        {
            telemetry.addData("Color", "X: %.2f, Y: %.2f, Area: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees(), cr.getTargetArea());
            if (cr.getTargetArea() > MINIMUM_TARGET_AREA)
            {
                // If we find a result that meets the minimum area, we consider it the best.
                // You could add logic here to compare multiple results and choose the best one.
                bestResult = cr;
                break; // Exit the loop after finding the first suitable result.
            }
        }
        return bestResult;
    }
}