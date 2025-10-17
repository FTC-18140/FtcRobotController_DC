package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.MovingAverageFilter;

@Config

public class Launcher {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotorEx launcher = null;
    public static double TPR = 28;
    public static double MAX_SHOOTER_TPS = 25;
    public static double MIN_SHOOTER_TPS = 15;

    public double tps = 0;
    public MovingAverageFilter TPSFilter = new MovingAverageFilter(3);

    public double getAvgtps() {
        return avgtps;
    }

    public double avgtps = 0;
    public double targettps = 0;
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

        try {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher\" not found", 0);
        }
        try {
            turret = hardwareMap.crservo.get("turret");
            turret.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Servo \"turret\" not found", 0);
        }
    }
    // returns the distance to the goal in inches
    public double goalDistance(Pose2d robotPose){
        targetDir = targetPos.minus(robotPose.position);
        return Math.sqrt(Math.pow(targetDir.x, 2) + Math.pow(targetDir.y, 2));
    }
    public void lockOn(Pose2d robotPose){
        double turretAngle = Range.scale(0, 0, 1.0, 0, 2*Math.PI);
        trueAngle = robotPose.heading.toDouble()+turretAngle;
        targetDir = targetPos.minus(robotPose.position);

        double difference = targetDir.angleCast().toDouble() - trueAngle;
        difference = Range.clip(difference, -1, 1);

    }
    // sets ticks per second and average ticks per second
    public void update(){
        tps = launcher.getCurrentPosition() - previousPos;
        previousPos = launcher.getCurrentPosition();

        avgtps = TPSFilter.addValue(tps);
    }
//calculates target tps using goalDistance and returns it
    public void targetTps(){
        return ;
    }
    // sets velocity to target
    public void shoot(Pose2d robotPose){

        launcher.setVelocity(targettps);
    }

    public void launchMax(){
        launcher.setVelocity(MAX_SHOOTER_TPS);
    }

    public void launchMin(){
        launcher.setVelocity(MIN_SHOOTER_TPS);
    }

    public void stop(){

        turret.setPower(0);
        launcher.setPower(0.0);
    }
}
