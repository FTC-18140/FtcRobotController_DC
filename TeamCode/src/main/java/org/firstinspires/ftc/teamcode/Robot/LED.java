package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    Telemetry telemetry;
    Servo rpmLed = null;
    Servo launcherLed = null;

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
            rpmLed = hwMap.servo.get("led");
            rpmLed.setPosition(red);
        }catch (Exception e){
            telemetry.addData("led not found in configuration", 0);
        }
        try{
            launcherLed = hwMap.servo.get("led2");
            launcherLed.setPosition(red);
        }catch (Exception e){
            telemetry.addData("led2 not found in configuration", 0);
        }
    }


    public void update(double measured_rpm, double target_rpm, double runtime, IndexerFacade.BallState loaded_color, boolean isIndexerFull) {
        double difference_tps =  measured_rpm - target_rpm;
        double acceptable_range_up = 30;
        double acceptable_range_down = -30;
        if (difference_tps < acceptable_range_down) {
            setRPMLedToColor("red");
        } else if (difference_tps > acceptable_range_up) {
            setRPMLedToColor("blue");
        } else {
            setRPMLedToColor("green");
        }
        double alertTime_End = 10;
        if ((120 - runtime) < alertTime_End) {
            if (Math.ceil(runtime * 2) % 2 == 1){
                setRPMLedToColor("off");
            } else {
                setRPMLedToColor("orange");
            }

        }
        if (isIndexerFull) {
            if (Math.ceil(runtime * 2) % 2 == 1){
                setRPMLedToColor("white");
            } else {
                switch (loaded_color) {
                    case GREEN:
                        setLauncherLedToColor("green");
                        break;
                    case PURPLE:
                        setLauncherLedToColor("purple");
                        break;
                    default:
                        setLauncherLedToColor("off");
                }
            }

        }


    }
//    public void update(IndexerFacade.BallState ballcolor, double remainingSeconds) {
//        if (remainingSeconds < 10) {
//            if (Math.ceil(remainingSeconds * 2) % 2 == 1){
//                setToColor("off");
//            } else {
//                setToColor("orange");
//            }
//
//        }
//        else if (ballcolor == IndexerFacade.BallState.GREEN) {
//            setToColor("green");
//        } else if (ballcolor == IndexerFacade.BallState.PURPLE) {
//            setToColor("purple");
//        } else {
//            setToColor("blue");
//        }
//    }
    public void setRPMLedToColor(String color) {
        rpmLed.setPosition(getColor(color));
    }
    public void setLauncherLedToColor(String color){
        launcherLed.setPosition(getColor(color));
    }

    /**
     * sets the color of the leds based on an input string
     * @param color
     */
    public double getColor(String color) {
        if(rpmLed != null) {
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
                    theColor = Range.clip(0.22*Math.sin(ledTimer.seconds()*3)+0.5, 0.28, 0.72);
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
        }
        return theColor;
    }
}
