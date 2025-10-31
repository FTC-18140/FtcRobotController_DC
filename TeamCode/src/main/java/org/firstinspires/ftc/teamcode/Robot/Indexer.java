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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    private static double index_error = 0.12;
    public enum IndexerState {
        ALIGNED,
        UNALIGNED,
        MANUAL,
        STOPPED,
        LOADING
    }
    private IndexerState state = IndexerState.ALIGNED;
    final double CPR = 8192;
    private double indexPos = 0;
    private double targetAngle = 0;

    double flipperPos = 0;

    public static double p = 0.7, i = 0.001, d = 0.001;
    PIDController angleController;

    ElapsedTime stateTimer = new ElapsedTime();

    double maxIndexerPower = 1.0;

    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        setState(IndexerState.UNALIGNED);

        angleController = new PIDController(p, i, d);
        angleController.setTolerance( 0.1 );

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

    public void adjustToThird(){
        //rotates the indexer until it is over the switch
        setState(IndexerState.MANUAL);
        maxIndexerPower = 0.05;
//        if(limitSwitch.getValue() > 0){
//            //resets the encoder
//            indexer.setPower(0);
//            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            indexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            //sets target position to the closets corresponding third(0, 1, 2)
//            targetAngle = 0;
//            setState(IndexerState.ALIGNED);
//            return true;
//        }else{
//            indexer.setPower(0.05);
//        }
//        telemetry.addData("indexer position: ", 3*indexMotor.getCurrentPosition()/CPR);
//        return false;
    }

    public void update(){
        //current ticks / ticks per rotation = rotations
        //multiply by 3 to get get thirds

        // Grab the latest values from the hardware connected to the indexer.  Get all the values
        // one time in this method to reduce the amount of calls to the hardware.
        boolean switchAligned = limitSwitch.isPressed();
        flipperPos = flipper.getPosition();
        telemetry.addData("Magnet: ", switchAligned);

        //Update he indexer position
        indexPos = 3 * indexMotor.getCurrentPosition()/CPR;

        //Update the PID controller
        angleController.setPID(p, i, d);
        double pidPower = angleController.calculate(indexPos, targetAngle);

        telemetry.addData("target Angle: ", targetAngle);
        telemetry.addData("indexer Angle: ", indexPos);
        telemetry.addData("Index power: ", pidPower);
        //indexer.setPower(angleController.calculate(indexPos, targetAngle));


        telemetry.addData("current State: ", state);

        // Run state machine for Indexer
        switch (state)
        {
            case ALIGNED:
                // I'm aligned, but I also need to use the PID to stay aligned.  Also make sure the
                // flipper is down.  Moving the flipper up happens in the LOADING state.
                indexer.setPower( pidPower );
                flipper.setPosition(0.25);
                // State transition conditions
                if ( angleController.atSetPoint())
                {
                    setState(IndexerState.ALIGNED);
                }
                else {
                    setState(IndexerState.UNALIGNED);
                }
                break;
            case UNALIGNED:
                // Move the indexer into position according to the PID controller, but the movement
                // is limited to the maxIndexPower.  Also make use the flipper is down.  Moving the
                // flipper happens in the LOADING state.
                indexer.setPower(Range.clip(pidPower, -maxIndexerPower, maxIndexerPower));
                flipper.setPosition(0.25);
                // State transition conditions
                if ( angleController.atSetPoint())
                {
                    maxIndexerPower = 1.0;
                    setState(IndexerState.ALIGNED);
                }
                else {
                    setState(IndexerState.UNALIGNED);
                }
                break;
            case MANUAL:
                // Move the indexer into position according to the maxIndexerPower supplied.  This
                // bypasses the PID control.  Also make sure the flipper is down.  Moving the flipper
                // happens in the LOADING state.
                indexer.setPower(maxIndexerPower);
                flipper.setPosition(0.25);
                // State transition conditions
                if( switchAligned ) {
                    //The switch is aligned, so reset the encoder
                    indexer.setPower(0);
                    indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    indexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //sets target position to 0 which is the "home" position.
                    targetAngle = 0;
                    angleController.reset();
                    //Reset the maxIndexerPower so that the full power is available to move the
                    // indexer around.
                    maxIndexerPower = 1.0;
                    setState(IndexerState.ALIGNED);
                }
                else {
                    setState(IndexerState.MANUAL);
                }
                break;
            case STOPPED:
                // Stop the indexer.  Also make sure the flipper is down.  Moving the flipper up
                // happens in the LOADING state.
                indexer.setPower(0);
                flipper.setPosition(0.25);
                // State transition conditions
                setState(IndexerState.STOPPED);
                // A command will be required to get us out of STOPPED.
                break;
            case LOADING:
                // Cycle the flipper up and then down according to the timer.
                if (stateTimer.seconds() < 0.5) {
                    flipper.setPosition(0.65);
                    setState(IndexerState.LOADING);
                }
                else if (stateTimer.seconds() < 1.0) {
                    flipper.setPosition(0.25);
                    setState(IndexerState.LOADING);
                }
                else
                {
                    setState(IndexerState.ALIGNED);
                }
                break;
            default:
                break;
        }

//        if(state != IndexerState.MANUAL && state != IndexerState.STOPPED){
//                indexer.setPower(angleController.calculate(indexPos, targetAngle));
//                if(Math.abs(targetAngle - indexPos) < index_error){
//                    setState(IndexerState.ALIGNED);
//                }
//        }

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
        maxIndexerPower = power;
        setState(IndexerState.UNALIGNED);
//        state = IndexerState.MANUAL;
//        indexer.setPower(power);
//
//        indexPos = 3 * indexMotor.getCurrentPosition()/CPR;
        // Set the target angle to the closest third
        targetAngle = Math.round(indexPos);
//
//        telemetry.addData("> target Angle: ", targetAngle);
//        telemetry.addData("> indexer Angle: ", indexPos);
    }
    public void cycle(double dir){
        //shifts the target angle by a third
        maxIndexerPower = 1.0;
        setState(IndexerState.UNALIGNED);
        targetAngle += dir;
    }

    public Action cycleAction(double dir){
        return new Action() {
            private boolean firstTime = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (firstTime) {
                    firstTime = false;
                    cycle(dir);
                }
                return state == IndexerState.ALIGNED;
            }
        };
    }
    public void stop(){
//        indexer.setPower(0);
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
        if(state == IndexerState.ALIGNED) {
            // Only move the flipper if I am ALIGNED.  If I am UNALIGNED, the indexer is moving, so don't flip.
            // If I am in MANUAL, the indexer is moving, so don't flip.  If I am STOPPED, the indexer
            // is not actively controlling its position, so don't flip.  If I am in LOADING, I'm in the
            // middle of an existing flip, so don't interrupt that.
            setState(IndexerState.LOADING);
            stateTimer.reset();
//            flipper.setPosition(0.65);
        }
    }
    public void unflip(){
//        if(flipperPos >= 0.5) {
//            flipper.setPosition(0.25);
            setState(IndexerState.ALIGNED);
//        }
    }
    public double getFlipperPos(){
        return flipperPos;
    }

    public Action flipperUpAction(){
        return new Action() {
            private boolean firstTime = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (firstTime) {
                    firstTime = false;
                    flip();
                }
                return state == IndexerState.ALIGNED;
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
