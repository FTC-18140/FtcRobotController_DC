package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Indexer {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    CRServo indexer = null;
    DcMotor indexMotor = null;
    Servo flipper = null;

    final double CPR = 8192;
    private double indexPos = 0;
    private double targetAngle = 0;

    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

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
    }

    public void intake(){
        indexer.setPower(0.5);
    }
    public boolean update(){
        indexPos = 3 * indexMotor.getCurrentPosition()/CPR;

        telemetry.addData("target Angle: ", targetAngle);
        telemetry.addData("indexer Angle: ", indexPos);
        indexer.setPower(Range.scale(targetAngle - (indexPos), -2, 2, -0.5, 0.5));

        return Math.abs(targetAngle - indexPos) < 0.1;
    }

    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !update();
            }
        };
    }
    public void spin(double power){
        indexer.setPower(power);

        indexPos = 3 * indexMotor.getCurrentPosition()/CPR;
        targetAngle = Math.round(indexPos);

        telemetry.addData("target Angle: ", targetAngle);
        telemetry.addData("indexer Angle: ", indexPos);
    }
    public void cycle(double dir){
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
    public void flip(){
        flipper.setPosition(0.65);
    }
    public void unflip(){
        flipper.setPosition(0.25);
    }
    public double getFlipperPos(){
        return flipper.getPosition();
    }

    public Action fliperAction(double pos){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flipper.setPosition(pos);

                return false;
            }
        };
    }
}
