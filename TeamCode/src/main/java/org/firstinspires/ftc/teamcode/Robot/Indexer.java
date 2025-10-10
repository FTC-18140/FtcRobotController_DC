package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Indexer {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    CRServo indexer = null;
    Servo flipper = null;
    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            indexer = hardwareMap.crservo.get("indexer");
        } catch (Exception e) {
            telemetry.addData("Continuous Servo \"indexer\" not found", 0);
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
    public void stop(){
        indexer.setPower(0);
    }
    public void flip(){
        flipper.setPosition(0.5);
    }
    public void unflip(){
        flipper.setPosition(0.25);
    }
    public double getFlipperPos(){
        return flipper.getPosition();
    }
}
