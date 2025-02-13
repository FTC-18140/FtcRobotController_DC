package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor liftLeft;
    DcMotor liftRight;
    public Servo leftServo;
    public Servo rightServo;
    public Servo podServo;

    public double offsetPos;

    public double podServoPos = 0;

    public final double LIFT_MAX = 790;
    public final double LIFT_SERVO_MAX = 0.35;
    public final double LIFT_SERVO_MAX_R = 0.33;
    public final double LIFT_SERVO_LIFT = 0.12;
    public final double LIFT_SERVO_LIFT_R = 0.12;

    public ElapsedTime lift_timer;

    public boolean hanging = false;

    public double lift_target = 0;
    public void init(HardwareMap hwMap, Telemetry telem, double startPos){
        lift_timer = new ElapsedTime();
        offsetPos = startPos;
        hardwareMap = hwMap;
        telemetry = telem;
        try{
            liftLeft = hardwareMap.dcMotor.get("liftL");
            liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setTargetPosition((int)lift_target);
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }catch (Exception e){
            telemetry.addData("left lift motor not found in configuration",0);
        }
        try{
            liftRight = hardwareMap.dcMotor.get("liftR");
            //liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setTargetPosition((int)lift_target);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }catch (Exception e){
            telemetry.addData("right lift motor not found in configuration",0);
        }
        try{
            leftServo = hardwareMap.servo.get("liftServoL");
            //leftServo.setDirection(Servo.Direction.REVERSE);
            leftServo.setPosition(LIFT_SERVO_MAX);
        }catch (Exception e){
            telemetry.addData("'liftServoL' not found in configuration", 0);
        }
        try{
            rightServo = hardwareMap.servo.get("liftServoR");
            rightServo.setDirection(Servo.Direction.REVERSE);
            rightServo.setPosition(LIFT_SERVO_MAX_R);
        }catch (Exception e){
            telemetry.addData("'liftServoR' not found in configuration", 0);
        }
        try{
            podServo = hardwareMap.servo.get("podServo");
            podServo.setPosition(0);
        }catch (Exception e){
            telemetry.addData("'liftServoR' not found in configuration", 0);
        }
    }
    public double getLiftPosR(){
        return liftRight.getCurrentPosition()+offsetPos;
    }
    public double getLiftPosL(){
        return liftLeft.getCurrentPosition()+offsetPos;
    }
    public double getLeftServoPos(){
        return leftServo.getPosition();
    }
    public double getRightServoPos(){
        return rightServo.getPosition();
    }
    public void moveLift(double power){
        if(power > 0){
            if(getLiftPosL() >= LIFT_MAX){
                liftLeft.setPower(0);
            }else{
                liftLeft.setPower(power);
            }
            if(getLiftPosR() >= LIFT_MAX){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        }else if(power < 0){
            if(getLiftPosL() <= 1){
                liftLeft.setPower(0);
            }else{
                liftLeft.setPower(power);
            }
            if(getLiftPosR() <= 1){
                liftRight.setPower(0);
            }else{
                liftRight.setPower(power);
            }
        } else{
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }

    }
    public void moveToTop(){
        leftServo.setPosition(-0.35);
        rightServo.setPosition(-0.35);
        podServoPos = 1;
        lift_target = LIFT_MAX;
        hanging = false;
    }
    public void moveToMin(){
        leftServo.setPosition(LIFT_SERVO_MAX);
        rightServo.setPosition(LIFT_SERVO_MAX_R);
        lift_target = 0;
    }
    public void moveToHanging(){
        leftServo.setPosition(LIFT_SERVO_LIFT);
        rightServo.setPosition(LIFT_SERVO_LIFT_R);
        lift_target = 0;
    }

    public void hang(){
        if(!hanging) {
            hanging = true;
            lift_timer.reset();
            lift_target = 0;
        }
    }
    public boolean liftUpTo(double position){
        if(Math.abs(position-getLiftPosR())<=1){
            moveLift(0);
            telemetry.addData("at the top", 0);
            return false;
        }else{
            moveLift(1);
            telemetry.addData("going up",0);
            return true;
        }

    }
    public boolean liftDownTo(double position){
        if(Math.abs(getLiftPosR()-position)<=1){
            moveLift(0);
            telemetry.addData("at the bottom", 0);
            return false;
        }else{
            moveLift(-1);
            telemetry.addData("going down", 0);
            return true;
        }

    }
    public Action liftToAction(double position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return liftUpTo(position);
            }
        };
    }
    public void update(){
        liftLeft.setPower(1);
        liftRight.setPower(1);
        if(lift_target < 50){
            //liftDownTo(lift_target);
            telemetry.addData("lift up", 0);
        }else if(lift_target > 500){
            //liftUpTo(lift_target);
            telemetry.addData("lift down", 0);
        }
        liftLeft.setTargetPosition((int)lift_target);
        liftRight.setTargetPosition((int)lift_target);
        if(hanging)
        {
            if(lift_timer.seconds() > 0.75)
            {
                leftServo.setPosition(LIFT_SERVO_LIFT);
                rightServo.setPosition(LIFT_SERVO_LIFT_R);
            }
        }
        podServo.setPosition(podServoPos);
    }
}
