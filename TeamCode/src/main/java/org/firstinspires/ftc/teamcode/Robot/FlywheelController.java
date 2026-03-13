package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FlywheelController {

    private Telemetry telemetry = null;
    private Flywheel lowerWheel = null;
    private Flywheel upperWheel = null;

    public static class LowerPID {
        public static double P = 0.0045, I = 0.006, D = 0.00011;
        public static double F_MAX = 0.62, F_MIN = 0.47;
    }

    public static class UpperPID {
        public static double P = 0.0045, I = 0.006, D = 0.00011;
        public static double F_MAX = 0.62, F_MIN = 0.47;
    }


    public static double FLYWHEEL_RATIO = (double) 1.0;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        lowerWheel = new Flywheel();
        upperWheel = new Flywheel();

        lowerWheel.init(hwMap, telem, "launcher");
        upperWheel.init(hwMap, telem, "launcher2");

        lowerWheel.setParameters(LowerPID.P, LowerPID.I, LowerPID.D, LowerPID.F_MIN, LowerPID.F_MAX);
        upperWheel.setParameters(UpperPID.P, UpperPID.I, UpperPID.D, UpperPID.F_MIN, UpperPID.F_MAX);
    }

    public void update() {
        lowerWheel.setParameters(LowerPID.P, LowerPID.I, LowerPID.D, LowerPID.F_MIN, LowerPID.F_MAX);
        upperWheel.setParameters(UpperPID.P, UpperPID.I, UpperPID.D, UpperPID.F_MIN, UpperPID.F_MAX);

        lowerWheel.update();
        upperWheel.update();
    }

    public void stop() {
        lowerWheel.stop();
        upperWheel.stop();
    }

    public void setTargetRpm(double target) {
        lowerWheel.setTargetRpm(target);
        upperWheel.setTargetRpm(target * FLYWHEEL_RATIO);
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
        if (lowerWheel.isAtTargetRpm() && upperWheel.isAtTargetRpm()) return true;

        telemetry.addData("lower Flywheel at target RPM: ", lowerWheel.isAtTargetRpm());
        telemetry.addData("upper Flywheel at target RPM: ", upperWheel.isAtTargetRpm());
        return false;
    }

    public double calculateBallVelocity(double distance, double height, double angleDegrees) {
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
        upperWheel.setTargetRpm(upperWheelRpm * FLYWHEEL_RATIO);
    }

}
