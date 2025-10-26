package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Indexer {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    CRServo indexer = null;
    DcMotor indexMotor = null;
    Servo flipper = null;
    TouchSensor limitSwitch = null;

    final double CPR = 8192;
    private double indexPos = 0;
    private double targetAngle = 0;

    public static double p = 0.5, i = 0.1, d = 0.99;
    PIDController angleController;

    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        angleController = new PIDController(p, i, d);
        try{
            indexer = hardwareMap.crservo.get("indexer");
        } catch (Exception e) {
            telemetry.addData("Continuous Servo \"indexer\" not found", 0);
        }
        try{
            indexMotor = hardwareMap.dcMotor.get("indexerpos");
            indexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            indexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            indexPos = 3 * indexMotor.getCurrentPosition()/CPR;
        } catch (Exception e) {
            telemetry.addData("Motor \"indexerpos\" not found", 0);
        }
        try{
            flipper = hardwareMap.servo.get("flipper");
        } catch (Exception e) {
            telemetry.addData("Servo \"flipper\" not found", 0);
        }
        try{
            limitSwitch = hardwareMap.touchSensor.get("magnet");
        } catch (Exception e) {
            telemetry.addData("Touch Sensor \"magnet\" not found", 0);
        }
    }

    public void intake(){
        indexer.setPower(0.5);
    }

    public boolean adjustToThird(){
        //rotates the indexer until it is over the switch

        if(limitSwitch.getValue() > 0){
            //resets the encoder
            indexer.setPower(0);
            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            indexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //sets target position to the closets corresponding third(0, 1, 2)
            targetAngle = 0;
            return true;
        }else{
            indexer.setPower(0.05);
        }
        telemetry.addData("indexer position: ", 3*indexMotor.getCurrentPosition()/CPR);
        return false;
    }

    public boolean update(){
        //current ticks / ticks per rotation = rotations
        //multiply by 3 to get get thirds

        telemetry.addData("Magnet: ", limitSwitch.getValue());
        indexPos = 3 * indexMotor.getCurrentPosition()/CPR;

        angleController.setPID(p, i, d);

        telemetry.addData("target Angle: ", targetAngle);
        telemetry.addData("indexer Angle: ", indexPos);
        telemetry.addData("Index power: ", angleController.calculate(indexPos, targetAngle));
        indexer.setPower(angleController.calculate(indexPos, targetAngle));

        return Math.abs(targetAngle - indexPos) < 0.1;
    }

    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return !(Math.abs(targetAngle - indexPos) < 0.1);
            }
        };
    }
    public void spin(double power){
        indexer.setPower(power);

        indexPos = 3 * indexMotor.getCurrentPosition()/CPR;
        //sets the target angle to the closest third
        targetAngle = Math.round(indexPos);

        telemetry.addData("> target Angle: ", targetAngle);
        telemetry.addData("> indexer Angle: ", indexPos);
    }
    public void cycle(double dir){
        //shifts the target angle by a third
        targetAngle += dir;
    }

    public Action cycleAction(double dir){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                cycle(dir);
                return false;
            }
        };
    }
    public void stop(){
        indexer.setPower(0);
    }
    public Action stopAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stop();
                return false;
            }
        };
    }
    public void flip(){
        flipper.setPosition(0.65);
    }
    public void unflip(){
        flipper.setPosition(0.25);
    }
    public double getFlipperPos(){
        return flipper.getPosition();
    }

    public Action flipperUpAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flip();
                return false;
            }
        };
    }
    public Action flipperDownAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                unflip();
                return false;
            }
        };
    }
}
