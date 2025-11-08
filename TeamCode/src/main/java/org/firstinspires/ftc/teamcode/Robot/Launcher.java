package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

import java.util.Objects;

@Config

public class Launcher {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotorEx launcher = null;
    DcMotorEx launcher2 = null;

    public static double p = 0.00145, i = 0.001, d = 0.000001, f = 0.0149;
    PIDController RPMController;

    public double turret_target_pos = 0;

    public double turret_current_pos = 0;
    public static double TURN_SPEED = 0.9;
    public static double MAX_SHOOTER_SPEED = 0.73;
    public static double MIN_SHOOTER_SPEED = 1.0;
    public static double SHOOTER_RADIUS = 45.239;

    public static double MAX_SHOOTER_RPM = 1050;
    public static double MIN_SHOOTER_RPM = 900;
    public static double  MAX_TURRET_POS = 1.5;
    public static double MIN_TURRET_POS = -1;

    public ElapsedTime timer;
    double previousTime = 0;

    public double rpm = 0;
    public MovingAverageFilter RPMFilter = new MovingAverageFilter(2);
    public double avgRpm = 0;
    public double power = 0;
    private double previousPos = 0;
    public double timeDifference = 0;
    static double targetX = 50;
    static double targetY = 50;

    public String color = "blue";

    CRServo turret = null;
    DcMotor turretEnc = null;
    public static double pTurret = 0.9, iTurret = 0.05, dTurret = 0.0000001;
    PIDController turretAimPID = new PIDController(pTurret, iTurret, dTurret);
    public static double INITIAL_TURRET_POS = .5;
    public static double TURRET_GEAR_RATIO = (double) 40/190;
    public static double TURRET_DEGREES_PER_SERVO_TURN = (1/TURRET_GEAR_RATIO)/360;
    public static double TURRET_DEGREES_PER_SERVO_COMMAND = .0048*(TURRET_DEGREES_PER_SERVO_TURN);


    Vector2d targetPos = new Vector2d(targetX, targetY);

    Vector2d targetPosRed = new Vector2d(targetX, -targetY);
    Vector2d targetDir = new Vector2d(0,1);
    public double trueAngle = 0;

    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        RPMController = new PIDController(p, i, d);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        try{
            launcher = hardwareMap.get(DcMotorEx.class,"launcher");
            //launcher.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher\" not found", 0);
        }
        try{
            launcher2 = hardwareMap.get(DcMotorEx.class,"launcher2");
            //launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher2\" not found", 0);
        }
        try{
            turret = hardwareMap.crservo.get("turret");
            turret.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Servo \"turret\" not found", 0);
        }
    }

    /**
     * Returns the correct goal distance to the goal based on alliance
     * @param robotPose the current position of the robot
     * @return the distance to the goal
     */
    public double goalDistance(Pose2d robotPose){
        if(Objects.equals(color, "red")){

            targetDir = targetPosRed.minus(robotPose.position);
        }else{

            targetDir = targetPos.minus(robotPose.position);
        }
        return Math.sqrt(Math.pow(targetDir.x, 2) + Math.pow(targetDir.y, 2));
    }

    /**
     * Updates several launcher variables, sets the power of the turret motor using Pid, adds some telemetry
     */
    public void update(){
        //rpm = launcher.getCurrentPosition() - previousPos;
        rpm = launcher.getVelocity();
        previousPos = launcher.getCurrentPosition();

        turret_target_pos = Range.clip(turret_target_pos, MIN_TURRET_POS, MAX_TURRET_POS);
        updateturret_current_pos();

        timeDifference = timer.milliseconds() - previousTime;
        previousTime = timer.milliseconds();

        avgRpm = RPMFilter.addValue(-rpm);

        previousTime = timer.milliseconds();

        //turret_target_pos = Range.clip(turret_target_pos, 0, 1);
        turretAimPID.setPID(pTurret, iTurret, dTurret);

        telemetry.addData("launcher vel: ", launcher.getVelocity());
        telemetry.addData("launcher2 vel: ", launcher2.getVelocity());

        telemetry.addData("time: ", timer.milliseconds());
        telemetry.addData("time difference: ", timeDifference);
    }

    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return true;
            }
        };
    }

    /**
     * Takes how many degrees off the aprilTag is (via the limelight) and changes the turret target position based on that
     * @param limelightxdegrees
     */
    public void lockOn(double limelightxdegrees){
        //double turretAngle = Range.scale(0, 0, 1.0, 0, 2*Math.PI);
        //trueAngle = robotPose.heading.toDouble()+turretAngle;
        //targetDir = targetPos.minus(robotPose.position);

        //double difference = targetDir.angleCast().toDouble() - trueAngle;
        updateturret_current_pos();

        double difference = limelightxdegrees * 150 *TURRET_DEGREES_PER_SERVO_COMMAND;
        difference = Range.clip(difference, -TURN_SPEED, TURN_SPEED);

        turret_target_pos = turret_current_pos + difference;
        turnToPosition(turret_target_pos);
        //turret_target_pos = Range.clip(turret_target_pos, 0, 1);
    }

    /**
     * Turns the turret to a particular position, adds some telem :)
     * @param angle The position you want the turret to rotate to; 1 is 90 degrees clockwise.
     */
    public void turnToPosition(double angle){
        turretAimPID.setPID(pTurret, iTurret, dTurret);

        angle = Range.clip(angle, MIN_TURRET_POS, MAX_TURRET_POS);

        double turret_pow = -turretAimPID.calculate(turret_current_pos, angle);

        turret.setPower(turret_pow);

        telemetry.addData("turret position: ", turret_current_pos);
        telemetry.addData("turret target: ", angle);
        telemetry.addData("turret power: ", turret_pow);
    }

    /**
     * Turns the turret using a power
     * @param dir The power, positive is clockwise
     */
    public void aim(double dir){
        updateturret_current_pos();
        if(dir > 0) {
            if(turret_current_pos > MIN_TURRET_POS){
                turret.setPower(dir);
            } else {
                turret.setPower(0);
            }
        }else if(dir < 0){
            if(turret_current_pos < MAX_TURRET_POS) {
                turret.setPower(dir);
            } else {
                turret.setPower(0);
            }
        }
    }

    /**
     * Updates the turret_current_position variable
     */
    public void updateturret_current_pos(){
        turret_current_pos = launcher2.getCurrentPosition() * TURRET_DEGREES_PER_SERVO_COMMAND;
    }

    /**
     * Sets the launcher motors power, adds some telemetry
     * @param robotPose the pose of the robot at present
     * @param distance the distance to the goal
     */
    public void shoot(Pose2d robotPose, double distance){
        power = calculatePower(goalDistance(robotPose), 89, Math.toRadians(60));

        double ff = f*Math.sin(Math.toRadians(avgRpm*(90.0/1100)));
        double toLaunchPow = Range.clip(RPMController.calculate(avgRpm, power), -0.1, 1) + ff;
        telemetry.addData("feedforward: ", ff);

        launcher.setPower(toLaunchPow);
        launcher2.setPower(toLaunchPow);
        telemetry.addData("power: ", toLaunchPow);

        telemetry.addData("target rpm: ", power);
        telemetry.addData("avgrpm: ", avgRpm);
    }

    /**
     * Calculates the rpm using some very complex untested math.
     * @param distance distance to the goal (in centimeters?)
     * @param height height of the goal in centimeters
     * @param hoodangle angle of the hood in cm
     * @return the rpm that the wheel should be spinning at
     */
    public double calculatePower(double distance, double height, double hoodangle){
        double g = -9.1;
        double coeff = .6;
        double sinA = Math.sin(hoodangle);
        double cosA = Math.cos(hoodangle);
        double zterm = height - (sinA * distance/(cosA - g));
        double sqrdxterm = .5 * g * Math.pow(distance, 2);
        double numerator = Math.sqrt(sqrdxterm/zterm)+g;
        double speed = numerator/cosA;
        double pwr = (SHOOTER_RADIUS * (1-coeff))/speed;
        pwr = Range.clip(pwr, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);
            return pwr;
    }

    /**
     * tells you whether the launcher is fast enough
     * @return A boolean that tells you whether the difference between power and avgRpm is less than an acceptable difference
     */
    public Action waitForCharge(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //double power = Range.clip(Range.scale(goalDistance(pose), 12, 130, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM), MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);
                if((power - avgRpm) < 0.275){
                    return false;
                }
                return true;
            }
        };
    }

    /**
     * Sets the launch power to maximum
     */
    public void launchMax(){
        launcher.setPower(MAX_SHOOTER_SPEED);
    }

    /**
     * Sets the launch power to minimum
     */
    public void launchMin(){
        launcher.setPower(MIN_SHOOTER_SPEED);
    }

    /**
     * Stops the launcher
     */
    public void stop(){
        launcher.setPower(0.0);
        launcher2.setPower(0.0);
    }
}
