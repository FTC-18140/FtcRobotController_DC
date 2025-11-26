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

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class Indexer {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    CRServo indexer = null;
    DcMotor indexMotor = null;
    Servo flipper = null;
    TouchSensor limitSwitch = null;
    //0, 1, and 2 numbered counterclockwise starting with the one closest to the intake
    ColorRangeSensor colorSensor0 = null;
    ColorRangeSensor colorSensor1 = null;
    ColorRangeSensor colorSensor2 = null;

    private static double index_error = 0.12;
    public enum IndexerState {
        ALIGNED,
        UNALIGNED,
        MANUAL,
        STOPPED,
        LOADING
    }
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE,
        INDET
    }
    static int[][] GREEN_THRESHOLDS = {{0,255},{0,255},{0,255},{0,255}}; // to be set
    static int[][] PURPLE_THRESHOLDS = {{0,255},{0,255},{0,255},{0,255}}; // to be set

    int delta;
    double lastIndexPos;
    List<BallColor> initialElements = Arrays.asList(BallColor.NONE,BallColor.NONE,BallColor.NONE);
    LinkedList<BallColor> inIndex = new LinkedList<>(initialElements);
    private IndexerState state = IndexerState.ALIGNED;
    public static double LIMITER_OFFSET = 0.1;
    double offset = 0;
    final double CPR = 8192;
    private double indexPos = 0;
    private double targetAngle = 0;

    public static double p = 0.147, i = 0.003, d = 0.0001;
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
            colorSensor0 = hardwareMap.get(ColorRangeSensor.class, "color0");
        } catch (Exception e) {
            telemetry.addData("Color Range Sensor \"colorSensor0\" not found", 0);
        }
        try{
            colorSensor1 = hardwareMap.get(ColorRangeSensor.class, "color1");
        } catch (Exception e) {
            telemetry.addData("Color Range Sensor \"colorSensor1\" not found", 0);
        }
        try{
            colorSensor2 = hardwareMap.get(ColorRangeSensor.class, "color2");
        } catch (Exception e) {
            telemetry.addData("Color Range Sensor \"colorSensor2\" not found", 0);
        }
    }

    public void update(){
        //current ticks / ticks per rotation = rotations
        //multiply by 3 to get get thirds

        telemetry.addData("target Angle: ", targetAngle);
        telemetry.addData("indexer Angle: ", indexPos);
        telemetry.addData("Index power: ", angleController.calculate(indexPos, targetAngle));
        telemetry.addData("Queue", inIndex);

        telemetry.addData("Magnet", limitSwitch.getValue());
        telemetry.addData("Color0 ARGB", argbOut(colorSensor0));
        telemetry.addData("Color0 Dist", colorSensor0.getDistance(DistanceUnit.MM));
        telemetry.addData("Color1 ARGB", argbOut(colorSensor1));

        telemetry.addData("Color1 Dist", colorSensor1.getDistance(DistanceUnit.MM));
        telemetry.addData("Color2 ARGB", argbOut(colorSensor2));
        telemetry.addData("Color2 Dist", colorSensor2.getDistance(DistanceUnit.MM));

        indexPos = 3 * indexMotor.getCurrentPosition()/CPR + offset;

        delta = (int) Math.round(indexPos) - (int) Math.round(lastIndexPos);
        cycleInIndex(delta);

        lastIndexPos = indexPos;

        angleController.setPID(p, i, d);


        //indexer.setPower(angleController.calculate(indexPos, targetAngle));

        switch (state){
            case UNALIGNED:
                indexer.setPower(angleController.calculate(indexPos, targetAngle));
                if(Math.abs(targetAngle - indexPos) < index_error){
                    setState(IndexerState.ALIGNED);
                }
                break;
            case ALIGNED:
                //update_inIndex();
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
    private boolean isWithinThresh(int[] values, int[][] thresholds){
        for(int i = 0; i < values.length; i++){
            if (values[i] < thresholds[i][0] || values[i] > thresholds[i][1]) {
                return false;
            }
        }
        return true;
    }
    public BallColor readBallColor(@NonNull ColorRangeSensor sensor){
        int[] aRGB = {sensor.alpha(), sensor.red(), sensor.green(),sensor.blue()};
        boolean gIsBall = isWithinThresh(aRGB, GREEN_THRESHOLDS);
        boolean pIsBall = isWithinThresh(aRGB, PURPLE_THRESHOLDS);

        if (pIsBall && gIsBall){
            return BallColor.INDET;
        } else if (pIsBall) {
            return BallColor.PURPLE;
        } else if (gIsBall) {
            return BallColor.GREEN;
        }
        return BallColor.NONE;
    }
    public int[] argbOut(ColorRangeSensor sensor){
        return new int[]{sensor.alpha(), sensor.red(), sensor.green(),sensor.blue()};
    }
    public void update_inIndex(){
        inIndex.set(0, readBallColor(colorSensor0));
        inIndex.set(1, readBallColor(colorSensor1));
        inIndex.set(2, readBallColor(colorSensor2));
    }
    public void setState(IndexerState newstate){
        state = newstate;
    }

    public IndexerState getState(){return state;}

//    public Action checkAlignmentAction(){
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                return state == IndexerState.ALIGNED;
//            }
//        };
//    }
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
        angleController.reset();
    }
    public void cycleInIndex(int turns){
        int shift = -turns % 3;
        BallColor holder;
        //rotates the list by turn amounts
        if (shift < 0){
            shift = shift + 3;
        }
        for (int i = 0; i < shift; i++ ){

                holder = inIndex.removeLast();
                inIndex.addFirst(holder);


        }
    }

    public void cycleTo(int pos){
        setState(IndexerState.UNALIGNED);
        targetAngle = pos;
        angleController.reset();
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
    public Action flipperDownAndCycleAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                unflip();
                return false;
            }
        };
    }
}
