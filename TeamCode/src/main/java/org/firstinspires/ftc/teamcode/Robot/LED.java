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
    public double off = (double) 0;
    public double red = 0.28;
    public double blue = 0.62;
    public double green = 0.5;
    public double yellow = 0.388;
    public double orange = 0.32;
    public double purple = 0.72;
    public double white = 1.0;
    public double theColor = white;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        try {
            rpmLed = hwMap.servo.get("led");
            rpmLed.setPosition(red);
        } catch (RuntimeException e) {
            telemetry.addData("led not found in configuration", 0);
        }
        try {
            launcherLed = hwMap.servo.get("led2");
            launcherLed.setPosition(red);
        } catch (RuntimeException e) {
            telemetry.addData("led2 not found in configuration", 0);
        }
    }


    public void update(double measuredRpm, double targetRpm, double lowerBoundRpm, double upperBoundRpm, double runtime, IndexerFacade.BallState loaded_color, boolean isIndexerFull, IndexerFacade.State IndexerState) {
        double differenceTps = measuredRpm - targetRpm;

        if (differenceTps < -lowerBoundRpm) {
            setRPMLedToColor("red");
        } else if (differenceTps > upperBoundRpm) {
            setRPMLedToColor("blue");
        } else {
            setRPMLedToColor("green");
        }
        double alertTimeEnd = 10.0;
        if (5.0 > (120.0 - runtime)) {
            if (1.0 == Math.ceil(runtime * 2.0) % 2.0) {
                setRPMLedToColor("off");
            } else {
                setRPMLedToColor("red");
            }
        } else if ((120.0 - runtime) < alertTimeEnd) {
            if (1.0 == Math.ceil(runtime * 2.0) % 2.0) {
                setRPMLedToColor("off");
            } else {
                setRPMLedToColor("orange");
            }

        }
        if (isIndexerFull) {
            if (1.0 == Math.ceil(runtime * 2.0) % 2.0) {
                setLauncherLedToColor("white");
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

        if (IndexerFacade.State.HOMING == IndexerState) {
            setRPMLedToColor("blue");
            setLauncherLedToColor("blue");
        }
    }

    public void setRPMLedToColor(String color) {
        rpmLed.setPosition(getColor(color));
    }

    public void setLauncherLedToColor(String color) {
        double colorValue = getColor(color);
        launcherLed.setPosition(colorValue);
    }

    /**
     * sets the color of the leds based on an input string
     *
     * @param color
     */
    public double getColor(String color) {
        if (null != rpmLed) {
            switch (color) {
                case ("off"):
                    theColor = off;
                    break;
                case ("red"):
                    theColor = red;
                    break;
                case ("yellow"):
                    theColor = yellow;
                    break;
                case ("blue"):
                    theColor = blue;
                    break;
                case ("purple"):
                    theColor = purple;
                    break;
                case ("green"):
                    theColor = green;
                    break;
                case ("rainbow"):
                    theColor = Range.clip(0.22 * Math.sin(ledTimer.seconds() * 3.0) + 0.5, 0.28, 0.72);
                    break;
                case ("orange"):
                    theColor = orange;
                    break;
                default:
                    theColor = white;
                    break;
            }
        }
        return theColor;
    }
}
