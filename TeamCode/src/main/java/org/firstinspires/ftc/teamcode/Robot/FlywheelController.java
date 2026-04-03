package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;

@Config
public class FlywheelController {

    private Telemetry telemetry = null;
    private Flywheel lowerWheel = null;
    private Flywheel upperWheel = null;

    private double last_distance = 0;

    public static class LowerPID {
        public double P = 0.00205, I = 0.05, D = 0.000001;
        public double F_MAX = 0.73, F_MIN = 0.56;
    }

    public static class UpperPID {
        public double P = 0.00225, I = 0.03, D = 0.000001;
        public double F_MAX = 0.56, F_MIN = 0.47;
    }

    public static LowerPID LOWER_PID = new LowerPID();
    public static UpperPID UPPER_PID = new UpperPID();


    public static double FLYWHEEL_RATIO = (double) 0.9;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        lowerWheel = new Flywheel();
        upperWheel = new Flywheel();

        lowerWheel.init(hwMap, telem, "launcher", "launcher");
        upperWheel.init(hwMap, telem, "launcher2", MecanumDrive.LEFT_BACK_MOTOR);
        upperWheel.setEncoderReversed();

        lowerWheel.setParameters(LOWER_PID.P, LOWER_PID.I, LOWER_PID.D, LOWER_PID.F_MIN, LOWER_PID.F_MAX, 1);
        upperWheel.setParameters(UPPER_PID.P, UPPER_PID.I, UPPER_PID.D, UPPER_PID.F_MIN, UPPER_PID.F_MAX, FLYWHEEL_RATIO);
    }

    public void update() {
        lowerWheel.setParameters(LOWER_PID.P, LOWER_PID.I, LOWER_PID.D, LOWER_PID.F_MIN, LOWER_PID.F_MAX, 1);
        upperWheel.setParameters(UPPER_PID.P, UPPER_PID.I, UPPER_PID.D, UPPER_PID.F_MIN, UPPER_PID.F_MAX, FLYWHEEL_RATIO);

        lowerWheel.update();
        upperWheel.update();
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
        double meanSqr = (Math.pow(lowerWheel.getError(), 2) + Math.pow(upperWheel.getError(), 2)) / 2;
        if (Math.sqrt(meanSqr) < 25) return true;

        telemetry.addData("lower Flywheel error: ", lowerWheel.getError());
        telemetry.addData("upper Flywheel error: ", upperWheel.getError());
        telemetry.addData("Flywheel mean: ", Math.sqrt(meanSqr));
        return false;
    }

    public double calculateBallVelocity(double distance, double height, double angleDegrees) {
        last_distance = distance;
        double angleRad = Math.toRadians(angleDegrees);
        double g = 9.81;

        double numerator = distance * distance * g;
        double denominator = (distance * Math.sin(2.0 * angleRad)) - (2.0 * height * Math.pow(Math.cos(angleRad), 2.0));

        denominator = Math.max(denominator, 0.4);

        telemetry.addData("Denominator: ", denominator);
        return Math.sqrt(numerator / denominator);
    }

    public void setTargetRpmFromVelocity(double velocity) {
        double lowerWheelRpm = lowerWheel.calculateWheelRPM(velocity);
        double upperWheelRpm = upperWheel.calculateWheelRPM(velocity);

        lowerWheel.setTargetRpm(lowerWheelRpm);
        upperWheel.setTargetRpm(upperWheelRpm);
    }

}
