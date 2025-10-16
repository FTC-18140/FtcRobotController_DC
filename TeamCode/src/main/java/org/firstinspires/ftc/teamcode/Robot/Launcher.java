package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class Launcher {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor launcher = null;

    public static double MAX_SHOOTER_SPEED = 0.73;
    public static double MIN_SHOOTER_SPEED = 0.6;

    public static double MAX_SHOOTER_RPM = 25;

    public double rpm = 0;
    private double previousPos = 0;
    static double targetX = 50;
    static double targetY = 50;
    CRServo turret = null;

    Vector2d targetPos = new Vector2d(targetX, targetY);
    Vector2d targetDir = new Vector2d(0,1);
    public double trueAngle = 0;
    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            launcher = hardwareMap.dcMotor.get("launcher");
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher\" not found", 0);
        }
        try{
            turret = hardwareMap.crservo.get("turret");
            turret.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Servo \"turret\" not found", 0);
        }
    }

    public double goalDistance(Pose2d robotPose){
        targetDir = targetPos.minus(robotPose.position);
        return Math.sqrt(Math.pow(targetDir.x, 2) + Math.pow(targetDir.y, 2));
    }
    public void update(){
        rpm = launcher.getCurrentPosition() - previousPos;
        previousPos = launcher.getCurrentPosition();
    }
    public void lockOn(Pose2d robotPose){
        double turretAngle = Range.scale(0, 0, 1.0, 0, 2*Math.PI);
        trueAngle = robotPose.heading.toDouble()+turretAngle;
        targetDir = targetPos.minus(robotPose.position);

        double difference = targetDir.angleCast().toDouble() - trueAngle;
        difference = Range.clip(difference, -1, 1);

    }

    public void shoot(Pose2d robotPose){
        double power = Range.clip(Range.scale(goalDistance(robotPose), 10, 120, MIN_SHOOTER_SPEED, MAX_SHOOTER_SPEED), MIN_SHOOTER_SPEED, MAX_SHOOTER_SPEED);

        power = Range.scale(power, MIN_SHOOTER_SPEED, MAX_SHOOTER_SPEED, 10, MAX_SHOOTER_RPM);


        launcher.setPower(Range.clip(power - rpm, 0, 1));
    }

    public void launchMax(){
        launcher.setPower(MAX_SHOOTER_SPEED);
    }

    public void launchMin(){
        launcher.setPower(MIN_SHOOTER_SPEED);
    }
    public void stop(){

        turret.setPower(0);
        launcher.setPower(0.0);
    }
}
