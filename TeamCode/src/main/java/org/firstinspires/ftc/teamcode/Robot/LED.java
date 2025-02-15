package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    Telemetry telemetry;
    Servo ledLeft = null;
    Servo ledRight = null;
    public ElapsedTime ledTimer = new ElapsedTime();
    public double red = 0.28;
    public double blue = 0.62;
    public double green = 0.5;
    public double yellow = 0.388;
    public double orange = 0.32;
    public double purple = 0.72;
    public double white = 1;
    public double theColor = white;
    public void init(HardwareMap hwMap, Telemetry telem){
        telemetry = telem;
        try{
            ledLeft = hwMap.servo.get("ledL");
            ledLeft.setPosition(red);
        }catch (Exception e){
            telemetry.addData("ledL not found in configuration", 0);
        }
        try{
            ledRight = hwMap.servo.get("ledR");
            ledRight.setPosition(red);
        }catch (Exception e){
            telemetry.addData("ledR not found in configuration", 0);
        }
    }
    public void setToColor(String color) {
        if(ledLeft != null && ledRight != null) {
            switch(color){
                case("red"):
                    theColor = red;
                    break;
                case("yellow"):
                    theColor = yellow;
                    break;
                case("blue"):
                    theColor = blue;
                    break;
                case("purple"):
                    theColor = purple;
                    break;
                case("green"):
                    theColor = green;
                    break;
                case("rainbow"):
                    theColor = Range.clip(0.22*Math.sin(ledTimer.seconds()/3)+0.5, 0.28, 0.72);
                    break;
                case("orange"):
//                    if(Math.sin(ledTimer.seconds()/2) > 0){
                        theColor = orange;
//                    }else{
//                        theColor = white;
//                    }
                    break;
                default:
                    theColor = white;
                    break;
            }
            ledLeft.setPosition(theColor);
            ledRight.setPosition(theColor);

        }
    }
}
