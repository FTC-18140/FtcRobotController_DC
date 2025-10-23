package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Config

public class Launcher {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotorEx launcher = null;
    DcMotorEx launcher2 = null;

    public static double p = 0.001, i = 0, d = 0;
    PIDController RPMController;

    public static double MAX_SHOOTER_SPEED = 0.73;
    public static double MIN_SHOOTER_SPEED = 1.0;

    public static double MAX_SHOOTER_RPM = 2500;
    public static double MIN_SHOOTER_RPM = 1800;

    public ElapsedTime timer;
    double previousTime = 0;

    public double rpm = 0;
    public MovingAverageFilter RPMFilter = new MovingAverageFilter(1);
    public double avgRpm = 0;
    public double power = 0;
    private double previousPos = 0;
    public double timeDifference = 0;
    static double targetX = 50;
    static double targetY = 50;

    public String color = "blue";
    CRServo turret = null;


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
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher\" not found", 0);
        }
        try{
            launcher2 = hardwareMap.get(DcMotorEx.class,"launcher2");
            launcher2.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher2\" not found", 0);
        }
        try{
            turret = hardwareMap.crservo.get("turret");
            turret.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Servo \"turret\" not found", 0);
        }
    }

    public double goalDistance(Pose2d robotPose){
        if(Objects.equals(color, "red")){

            targetDir = targetPosRed.minus(robotPose.position);
        }else{

            targetDir = targetPos.minus(robotPose.position);
        }
        return Math.sqrt(Math.pow(targetDir.x, 2) + Math.pow(targetDir.y, 2));
    }
    public void update(){
        //rpm = launcher.getCurrentPosition() - previousPos;
        rpm = launcher.getVelocity();
        previousPos = launcher.getCurrentPosition();

        timeDifference = timer.milliseconds() - previousTime;
        previousTime = timer.milliseconds();

        avgRpm = RPMFilter.addValue(rpm);
        previousTime = timer.milliseconds();

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
    public void lockOn(Pose2d robotPose){
        double turretAngle = Range.scale(0, 0, 1.0, 0, 2*Math.PI);
        trueAngle = robotPose.heading.toDouble()+turretAngle;
        targetDir = targetPos.minus(robotPose.position);

        double difference = targetDir.angleCast().toDouble() - trueAngle;
        difference = Range.clip(difference, -1, 1);

    }

    public void shoot(Pose2d robotPose){
        power = Range.clip(Range.scale(goalDistance(robotPose), 12, 130, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM), MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);



        launcher.setPower(Range.scale(RPMController.calculate(avgRpm, power), -1, 1, -0.1, 1));
        launcher2.setPower(Range.scale(RPMController.calculate(avgRpm, power), -1, 1, -0.1, 1));
        telemetry.addData("power: ", power - avgRpm);

        telemetry.addData("target rpm: ", power);
        telemetry.addData("avgrpm: ", avgRpm);
    }

    public Action chargeAction(Pose2d pose, double duration){
        return new Action() {
            ElapsedTime counter = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    counter.reset();
                    started = true;
                }
                shoot(pose);

                telemetry.addData("time: ", counter.seconds());

                if (counter.seconds() > duration){
                    stop();
                    return false;
                }
                return true;
            }
        };
    }

    public Action waitForCharge(Pose2d pose){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double power = Range.clip(Range.scale(goalDistance(pose), 12, 130, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM), MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);
                if(power - avgRpm < 0.275){
                    return false;
                }
                return true;
            }
        };
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
        launcher2.setPower(0.0);
    }
}
