package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FlywheelController {

    private Telemetry telemetry = null;
    private Flywheel lowerWheel = null;
    private Flywheel upperWheel = null;

    private double last_distance = 0;
    double angleToGoal = 0.0;
    PoseVelocity2d odoVelocity = null;
    public static double INERTIA_FACTOR = 0.009;
    public static double RPM_TOLERANCE = 40;
    public static double voltage_comp = 12.987;

    public static double[] distances = {50, 70, 94, 167};
    public static double[] rpms = {1750, 1730, 1850, 2270};

    static class DistRPM {
        double x, y;

        DistRPM(double x, double y) {
            super();
            this.x = x;
            this.y = y;
        }
    }

    public static DistRPM distRPMs[] = new DistRPM[distances.length];

    ;

    public static class LowerPID {
        public double P = 0.0007, I = 0.00, D = 0.000021;
        public double F_MAX = 0.49, F_MIN = 0.0, F_VEL = 0.0000, F_STATIC = 0.62;
        public double GEAR_RATIO = 2.0;
    }

    public static class UpperPID {
        public double P = 0.0007, I = 0.00, D = 0.000019;
        public double F_MAX = 0.485, F_MIN = 0.0, F_VEL = 0.0000, F_STATIC = 0.62;
        public double GEAR_RATIO = 32.0 / 15.0;
    }

    public static LowerPID LOWER_PID = new LowerPID();
    public static UpperPID UPPER_PID = new UpperPID();


    public static double FLYWHEEL_RATIO = 1.0;


    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        initDistRPMs();
        lowerWheel = new Flywheel();
        upperWheel = new Flywheel();

        lowerWheel.init(hwMap, telem, "launcher2", "launcher2");
        upperWheel.init(hwMap, telem, "launcher", "turret");
        upperWheel.setEncoderReversed();
        lowerWheel.setEncoderReversed();

        lowerWheel.setParameters(LOWER_PID.P, LOWER_PID.I, LOWER_PID.D, LOWER_PID.F_MIN, LOWER_PID.F_MAX, LOWER_PID.F_VEL, LOWER_PID.F_STATIC, LOWER_PID.GEAR_RATIO, FLYWHEEL_RATIO);
        upperWheel.setParameters(UPPER_PID.P, UPPER_PID.I, UPPER_PID.D, UPPER_PID.F_MIN, UPPER_PID.F_MAX, UPPER_PID.F_VEL, UPPER_PID.F_STATIC, UPPER_PID.GEAR_RATIO, 1 / FLYWHEEL_RATIO);
    }

    public void update(PoseVelocity2d currentOdoVelocity, double fieldAngleToGoal, double voltage) {
        initDistRPMs();
        double voltage_factor = voltage_comp / voltage + (voltage_comp - 13);

        lowerWheel.setParameters(LOWER_PID.P, LOWER_PID.I, LOWER_PID.D, LOWER_PID.F_MIN, LOWER_PID.F_MAX * voltage_factor, LOWER_PID.F_VEL, LOWER_PID.F_STATIC * voltage_factor, LOWER_PID.GEAR_RATIO, FLYWHEEL_RATIO);
        upperWheel.setParameters(UPPER_PID.P, UPPER_PID.I, UPPER_PID.D, UPPER_PID.F_MIN, UPPER_PID.F_MAX * voltage_factor, UPPER_PID.F_VEL, UPPER_PID.F_STATIC * voltage_factor, UPPER_PID.GEAR_RATIO, 1 / FLYWHEEL_RATIO);

        lowerWheel.update();
        upperWheel.update();
        angleToGoal = fieldAngleToGoal;
        odoVelocity = currentOdoVelocity;
    }

    private void initDistRPMs() {
        for (int i = 0; i < distances.length; i++) {
            distRPMs[i] = new DistRPM(distances[i], rpms[i]);
        }
    }


    public void stop() {
        lowerWheel.stop();
        upperWheel.stop();
    }

    public void setTargetRpm(double target) {
        lowerWheel.setTargetRpm(target);
        upperWheel.setTargetRpm(target);
    }

    public double getLowerFlywheelCurrentRPM() {
        return lowerWheel.getCurrentRpm();
    }

    public double getUpperFlywheelCurrentRPM() {
        return upperWheel.getCurrentRpm();
    }

    public double getLowerFlywheelTargetRPM() {
        return lowerWheel.getTargetRpm();
    }

    public double getUpperFlywheelTargetRPM() {
        return upperWheel.getTargetRpm();
    }

    public double getLowerFlywheelCurrentDraw() {
        return lowerWheel.getCurrentDraw();
    }

    public double getUpperFlywheelCurrentDraw() {
        return upperWheel.getCurrentDraw();
    }

    public double getTotalCurrentDraw() {
        return getLowerFlywheelCurrentDraw() + getUpperFlywheelCurrentDraw();
    }

    public boolean isAtTargetRpm() {
        boolean atRpm;
        double meanSqr = (Math.pow(lowerWheel.getError(), 2) + Math.pow(upperWheel.getError(), 2)) / 2;
        atRpm = RPM_TOLERANCE > Math.sqrt(meanSqr);

        telemetry.addData("lower Flywheel error: ", lowerWheel.getError());
        telemetry.addData("upper Flywheel error: ", upperWheel.getError());
        telemetry.addData("Flywheel mean: ", Math.sqrt(meanSqr));

        return atRpm;


    }

    /**
     * function to interpolate the given data points using Lagrange's formula
     * https://www.geeksforgeeks.org/dsa/lagranges-interpolation/
     *
     * @param distRPMs
     * @param xi       corresponds to the new data point whose value is to be obtained
     * @return
     */
    public static double interpolateDistRPM(double xi) {
        double result = 0; // Initialize result
        int n = distRPMs.length; //represents the number of known data points
        for (int i = 0; i < n; i++) {
            // Compute individual terms of above formula
            double term = distRPMs[i].y;
            for (int j = 0; j < n; j++) {
                if (j != i)
                    term = term * (xi - distRPMs[j].x) / (distRPMs[i].x - distRPMs[j].x);
            }

            // Add current term to result
            result += term;
        }

        return result;
    }

    public double calculateBallVelocity(double distance, double height, double angleDegrees) {
        last_distance = distance;

        double angleRad = Math.toRadians(angleDegrees);
        double g = 9.81;

        double numerator = distance * distance * g;
        double denominator = (distance * Math.sin(2.0 * angleRad)) - (2.0 * height * Math.pow(Math.cos(angleRad), 2.0));

        denominator = Math.max(denominator, 0.4);

        telemetry.addData("Denominator: ", denominator);
        double ballVelocity = Math.sqrt(numerator / denominator);
        return ballVelocity - (INERTIA_FACTOR * odoVelocity.linearVel.dot(new Vector2d(Math.sin(angleToGoal), Math.cos(angleToGoal))));
    }

    void setTargetRpmFromVelocity(double velocity) {
        double lowerWheelRpm = lowerWheel.calculateWheelRPM(velocity);
        double upperWheelRpm = upperWheel.calculateWheelRPM(velocity);

        lowerWheel.setTargetRpm(lowerWheelRpm);
        upperWheel.setTargetRpm(upperWheelRpm);
    }

    void setTargetRpmFromDistance(double distance) {
        double rpm = interpolateDistRPM(distance);
        lowerWheel.setTargetRpm(rpm);
        upperWheel.setTargetRpm(rpm);
    }

    public void DEBUG_upperFlywheel() {
        upperWheel.setPower(0.5);
    }

    public void DEBUG_lowerFlywheel() {
        lowerWheel.setPower(0.5);
    }

}
