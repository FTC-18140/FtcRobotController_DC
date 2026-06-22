package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FlywheelController
{

    private Telemetry telemetry = null;
    private FilteredFlywheel lowerWheel = null;
    private FilteredFlywheel upperWheel = null;

    private double last_distance = 0;
    double angleToGoal = 0.0;
    PoseVelocity2d odoVelocity = null;
    public static double INERTIA_FACTOR = 0.009;
    public static double RPM_TOLERANCE = 40;
    public static double voltage_comp = 12.988;

//    public static double[] distances = {50, 70, 94, 167};
//    public static double[] rpms = {1750, 1750, 1850, 2400};

    public enum RunMode
    {
        OFF,
        STATIC,   // For chargeLow()
        DISTANCE  // For charge() / prepShot()
    }

    private RunMode currentMode = RunMode.OFF;
    private double lastCalculatedDistanceRpm = 0;

    static class DistRPM
    {
        double distance, rpm;

        DistRPM(double distance, double rpm)
        {
            super();
            this.distance = distance;
            this.rpm = rpm;
        }
    }

//    public static DistRPM distRPMs[] = new DistRPM[distances.length];
    public static DistRPM[] distRPMs =
            {
                    new DistRPM(50, 1750),
                    new DistRPM(70, 1750),
                    new DistRPM(94, 1850),
                    new DistRPM(167, 2400)
            };

    public static class LowerPID
    {
        public double P = 0.0012, I = 0.00, D = 0.000085;
        public double F_MAX = 0.55, F_MIN = 0.0, F_VEL = 0.0000, F_STATIC = 0.625;
        public double GEAR_RATIO = 2.0;
    }

    public static class UpperPID
    {
        public double P = 0.0012, I = 0.00, D = 0.000085;
        public double F_MAX = 0.55, F_MIN = 0.0, F_VEL = 0.0000, F_STATIC = 0.625;
        public double GEAR_RATIO = 32.0 / 15.0;
    }

    public static LowerPID LOWER_PID = new LowerPID();
    public static UpperPID UPPER_PID = new UpperPID();


    public static double FLYWHEEL_RATIO = 1.0;


    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;
//        initDistRPMs();
        lowerWheel = new FilteredFlywheel();
        upperWheel = new FilteredFlywheel();

        lowerWheel.init(hwMap, telem, "launcher2", "launcher2");
        upperWheel.init(hwMap, telem, "launcher", "turret");
//        upperWheel.setEncoderReversed();
//        lowerWheel.setEncoderReversed();

//        lowerWheel.setParameters(LOWER_PID.P, LOWER_PID.I, LOWER_PID.D, LOWER_PID.F_MIN, LOWER_PID.F_MAX, LOWER_PID.F_VEL, LOWER_PID.F_STATIC, LOWER_PID.GEAR_RATIO, FLYWHEEL_RATIO);
//        upperWheel.setParameters(UPPER_PID.P, UPPER_PID.I, UPPER_PID.D, UPPER_PID.F_MIN, UPPER_PID.F_MAX, UPPER_PID.F_VEL, UPPER_PID.F_STATIC, UPPER_PID.GEAR_RATIO, 1 / FLYWHEEL_RATIO);
    }

    public void update(PoseVelocity2d currentOdoVelocity, double distance)
    {
        update(currentOdoVelocity, 0.0, 13.0, distance);
    }

    public void update(PoseVelocity2d currentOdoVelocity, double fieldAngleToGoal, double voltage, double distance)
    {
//        initDistRPMs();

        // 1. ALWAYS calculate what the RPM SHOULD be (Continuous knowledge)
        lastCalculatedDistanceRpm = interpolateDistRPM(distance);

        double voltageFactor = voltage_comp / voltage + (voltage_comp - 13);

//        lowerWheel.setParameters(LOWER_PID.P, LOWER_PID.I, LOWER_PID.D, LOWER_PID.F_MIN, LOWER_PID.F_MAX * voltageFactor, LOWER_PID.F_VEL, LOWER_PID.F_STATIC * voltageFactor, LOWER_PID.GEAR_RATIO, FLYWHEEL_RATIO);
//        upperWheel.setParameters(UPPER_PID.P, UPPER_PID.I, UPPER_PID.D, UPPER_PID.F_MIN, UPPER_PID.F_MAX * voltageFactor, UPPER_PID.F_VEL, UPPER_PID.F_STATIC * voltageFactor, UPPER_PID.GEAR_RATIO, 1 / FLYWHEEL_RATIO);

        // 2. Decide what to actually tell the motors based on the Mode
        switch (currentMode)
        {
            case DISTANCE:
                // Actively track the moving distance
                setMotorTargets(lastCalculatedDistanceRpm);
                break;
            case STATIC:
                // Use your defined static idle speed (e.g., 1800)
                setMotorTargets(Flywheel.STATIC_RPM);
                break;
            case OFF:
                lowerWheel.stop();
                upperWheel.stop();
                break;
        }

        lowerWheel.update();
        upperWheel.update();

        angleToGoal = fieldAngleToGoal;
        odoVelocity = currentOdoVelocity;
    }

    private void setMotorTargets(double rpm)
    {
        lowerWheel.setTargetRpm(rpm);
        upperWheel.setTargetRpm(rpm);
    }

    public void setMode(RunMode mode)
    {
        this.currentMode = mode;
    }

    public double getCalculatedRpm()
    {
        return lastCalculatedDistanceRpm;
    }

//    private void initDistRPMs()
//    {
//        for (int i = 0; i < distances.length; i++)
//        {
//            distRPMs[i] = new DistRPM(distances[i], rpms[i]);
//        }
//    }


    public void stop()
    {
//        lowerWheel.stop();
//        upperWheel.stop();
        currentMode = RunMode.OFF;
    }

    public void setTargetRpm(double target)
    {
        lowerWheel.setTargetRpm(target);
        upperWheel.setTargetRpm(target);
    }

    public double getLowerFlywheelCurrentRPM()
    {
        return lowerWheel.getCurrentRpm();
    }

    public double getUpperFlywheelCurrentRPM()
    {
        return upperWheel.getCurrentRpm();
    }

    public double getLowerFlywheelTargetRPM()
    {
        return lowerWheel.getTargetRpm();
    }

    public double getUpperFlywheelTargetRPM()
    {
        return upperWheel.getTargetRpm();
    }

    public double getLowerFlywheelCurrentDraw()
    {
        return lowerWheel.getCurrentDraw();
    }

    public double getUpperFlywheelCurrentDraw()
    {
        return upperWheel.getCurrentDraw();
    }

    public double getTotalCurrentDraw()
    {
        return getLowerFlywheelCurrentDraw() + getUpperFlywheelCurrentDraw();
    }

    public boolean isAtTargetRpm()
    {
        boolean atRpm;
        double meanSqr = (Math.pow(lowerWheel.getError(), 2) + Math.pow(upperWheel.getError(), 2)) / 2;
        atRpm = RPM_TOLERANCE > Math.sqrt(meanSqr);

//        telemetry.addData("lower Flywheel error: ", lowerWheel.getError());
//        telemetry.addData("upper Flywheel error: ", upperWheel.getError());
//        telemetry.addData("Flywheel mean: ", Math.sqrt(meanSqr));

        return atRpm;
    }

    /**
     * function to interpolate the given data points using linear piecewise
     *
     * @param distance corresponds to the distance to use for interpolation.
     * @return rpm
     */
    public static double interpolateDistRPM(double distance)
    {
        if (distance <= distRPMs[0].distance)
            return distRPMs[0].rpm;

        if (distance >= distRPMs[distRPMs.length - 1].distance)
            return distRPMs[distRPMs.length - 1].rpm;

        for (int i = 0; i < distRPMs.length - 1; i++)
        {
            DistRPM p0 = distRPMs[i];
            DistRPM p1 = distRPMs[i + 1];

            if (distance >= p0.distance && distance <= p1.distance)
            {
                double t =
                        (distance - p0.distance) /
                                (p1.distance - p0.distance);

                return p0.rpm + t * (p1.rpm - p0.rpm);
            }
        }

        return distRPMs[distRPMs.length - 1].rpm;
    }



    public double calculateBallVelocity(double distance, double height, double angleDegrees)
    {
        last_distance = distance;

        double angleRad = Math.toRadians(angleDegrees);
        double g = 9.81;

        double numerator = distance * distance * g;
        double denominator = (distance * Math.sin(2.0 * angleRad)) - (2.0 * height * Math.pow(Math.cos(angleRad), 2.0));

        denominator = Math.max(denominator, 0.4);

//        telemetry.addData("Denominator: ", denominator);
        double ballVelocity = Math.sqrt(numerator / denominator);
        return ballVelocity - (INERTIA_FACTOR * odoVelocity.linearVel.dot(new Vector2d(Math.sin(angleToGoal), Math.cos(angleToGoal))));
    }

//    void setTargetRpmFromVelocity(double velocity)
//    {
//        double lowerWheelRpm = lowerWheel.calculateWheelRPM(velocity);
//        double upperWheelRpm = upperWheel.calculateWheelRPM(velocity);
//
//        lowerWheel.setTargetRpm(lowerWheelRpm);
//        upperWheel.setTargetRpm(upperWheelRpm);
//    }

    void setTargetRpmFromDistance(double distance)
    {
        double rpm = interpolateDistRPM(distance);
        lowerWheel.setTargetRpm(rpm);
        upperWheel.setTargetRpm(rpm);
    }

    public void DEBUG_upperFlywheel()
    {
        upperWheel.setTargetRpm(1000);
        telemetry.addLine("Upper RPM Set to 1000");
    }

    public void DEBUG_lowerFlywheel()
    {
        lowerWheel.setTargetRpm(1000);
        telemetry.addLine("Lower RPM set to 1000");
    }

}
