package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Indexer {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    CRServo indexer = null;
    DcMotor indexMotor = null;
    Servo flipper = null;
    TouchSensor limitSwitch = null;
    ColorRangeSensor colorSensor = null;

    private static double index_error = 0.12;
    public enum IndexerState {
        ALIGNED,
        UNALIGNED,
        MANUAL,
        STOPPED,
        LOADING
    }
    private IndexerState state = IndexerState.ALIGNED;
    public static double LIMITER_OFFSET = 0.1;
    double offset = 0;
    final double CPR = 8192;
    private double indexPos = 0;
    private double targetAngle = 0;

    public static double p = 0.17, i = 0.0, d = 0.0;
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
        try{
            colorSensor = hardwareMap.get(ColorRangeSensor.class, "color");
        } catch (Exception e) {
            telemetry.addData("Color Range Sensor \"color\" not found", 0);
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
            offset = LIMITER_OFFSET;
            setState(IndexerState.ALIGNED);
            return true;
        }else{
            setState(IndexerState.MANUAL);
            indexer.setPower(0.05);
        }
        telemetry.addData("indexer position: ", 3*indexMotor.getCurrentPosition()/CPR);
        return false;
    }

    public void update(){
        //current ticks / ticks per rotation = rotations
        //multiply by 3 to get get thirds

        telemetry.addData("Magnet: ", limitSwitch.getValue());
        telemetry.addData("Color Sensor Color: ", colorSensor.argb());
        telemetry.addData("Color Sensor Distance: ", colorSensor.getDistance(DistanceUnit.MM));
        indexPos = 3 * indexMotor.getCurrentPosition()/CPR + offset;

        angleController.setPID(p, i, d);

        telemetry.addData("target Angle: ", targetAngle);
        telemetry.addData("indexer Angle: ", indexPos);
        telemetry.addData("Index power: ", angleController.calculate(indexPos, targetAngle));
        //indexer.setPower(angleController.calculate(indexPos, targetAngle));

        switch (state){
            case UNALIGNED:
                indexer.setPower(angleController.calculate(indexPos, targetAngle));
                if(Math.abs(targetAngle - indexPos) < index_error){
                    setState(IndexerState.ALIGNED);
                }
                break;
            case ALIGNED:
                indexer.setPower(angleController.calculate(indexPos, targetAngle));
                if(Math.abs(targetAngle - indexPos) > index_error){
                    setState(IndexerState.UNALIGNED);
                }
                break;
            case MANUAL:
                break;
            case LOADING:
                indexer.setPower(0);
                break;
            case STOPPED:
                indexer.setPower(0);
                break;

        }

        telemetry.addData("current State: ", state);

    }

    public void setState(IndexerState newstate){
        state = newstate;
    }

    public IndexerState getState(){return state;}

    public Action updateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                return state == IndexerState.ALIGNED;
            }
        };
    }
    public void spin(double power){
        state = IndexerState.MANUAL;
        indexer.setPower(power);

        indexPos = 3 * indexMotor.getCurrentPosition()/CPR + offset;
        //sets the target angle to the closest third
        targetAngle = Math.round(indexPos);

        telemetry.addData("> target Angle: ", targetAngle);
        telemetry.addData("> indexer Angle: ", indexPos);
    }
    public void cycle(double dir){
        //shifts the target angle by a third
        setState(IndexerState.UNALIGNED);
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
        setState(IndexerState.STOPPED);
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
        if(state != IndexerState.UNALIGNED) {
            setState(IndexerState.LOADING);
            flipper.setPosition(0.65);
        }
    }
    public void unflip(){
        if(getFlipperPos() >= 0.5) {
            flipper.setPosition(0.25);
            cycle(-1);
        }
    }
    public double getFlipperPos(){
        return flipper.getPosition();
    }

    public Action flipperUpAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flip();
                return getFlipperPos() < 0.5;
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
