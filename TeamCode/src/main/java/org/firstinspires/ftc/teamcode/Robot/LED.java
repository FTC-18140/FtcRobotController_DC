package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    Telemetry telemetry;
    Servo led = null;

    public ElapsedTime ledTimer = new ElapsedTime();
    public double off = 0;
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
            led = hwMap.servo.get("led");
            led.setPosition(red);
        }catch (Exception e){
            telemetry.addData("led not found in configuration", 0);
        }
    }
    public void update(double tps, double target_tps) {
        double difference_tps =  tps - target_tps ;
        double acceptable_range_up = .2;
        double acceptable_range_down = -.2;
        if (difference_tps < acceptable_range_down) {
            setToColor("red");
        } else if (difference_tps > acceptable_range_up) {
            setToColor("blue");
        } else {
            setToColor("green");
        }
    }
    public void setToColor(String color) {
        if(led != null) {
            switch(color){
                case("off"):
                    theColor = off;
                    break;
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
            led.setPosition(theColor);

        }
    }
}
