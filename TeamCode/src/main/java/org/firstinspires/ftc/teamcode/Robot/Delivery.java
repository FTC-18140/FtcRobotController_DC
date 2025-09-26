package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Delivery {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor launcher = null;

    static double targetX = 50;
    static double targetY = 50;
    Servo turret = null;

    Vector2d targetPos = new Vector2d(targetX, targetY);
    Vector2d targetDir = new Vector2d(0,1);
    public double trueAngle = 0;
    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            launcher = hardwareMap.dcMotor.get("launcher");
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher\" not found", 0);
        }
        try{
            turret = hardwareMap.servo.get("turret");
        } catch (Exception e) {
            telemetry.addData("DcMotor \"turret\" not found", 0);
        }
    }

    public void lockOn(Pose2d robotPose){
        double turretAngle = Range.scale(turret.getPosition(), 0, 1.0, 0, 2*Math.PI);
        trueAngle = robotPose.heading.toDouble()+turretAngle;
        targetDir = targetPos.minus(robotPose.position);

        double difference = targetDir.angleCast().toDouble() - trueAngle;
        difference = Range.clip(difference, -1, 1);

        

    }

    public void launch(){
        launcher.setPower(1.0);
    }
    public void stop(){
        launcher.setPower(0.0);
    }
}
