package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    private Telemetry telemetry = null;
    private Servo rpmLed = null;
    private Servo launcherLed = null;

    private final ElapsedTime ledTimer = new ElapsedTime();

    enum Colors {
        OFF,
        RED,
        BLUE,
        GREEN,
        YELLOW,
        ORANGE,
        INDIGO,
        PURPLE,
        WHITE,
        RAINBOW
    }

    private double hue = 1.0;
    private double hueLauncherLed = 1.0;
    private double hueRpmLed = 1.0;
    private double lowerBoundRpm = 0.0;
    private double upperBoundRpm = 0.0;

    public void init(HardwareMap hwMap, Telemetry telem, double lowerBoundRpmIn, double upperBoundRpmIn) {
        telemetry = telem;

        try {
            rpmLed = hwMap.servo.get("led");
            setRPMLedToColor(Colors.RED);
        } catch (RuntimeException e) {
            telemetry.addData("led not found in configuration", 0);
        }
        try {
            launcherLed = hwMap.servo.get("led2");
            setLauncherLedToColor(Colors.RED);
        } catch (RuntimeException e) {
            telemetry.addData("led2 not found in configuration", 0);
        }
        lowerBoundRpm = lowerBoundRpmIn;
        upperBoundRpm = upperBoundRpmIn;
    }

    public void update(double measuredRpm, double targetRpm, double runtime, IndexerFacade.BallState loadedColor, boolean isIndexerFull, IndexerFacade.State indexerState) {
        double differenceTps = measuredRpm - targetRpm;

        if (differenceTps < -lowerBoundRpm) {
            setRPMLedToColor(Colors.RED);
        } else if (differenceTps > upperBoundRpm) {
            setRPMLedToColor(Colors.BLUE);
        } else {
            setRPMLedToColor(Colors.GREEN);
        }
        double alertTimeEnd = 10.0;
        if (5.0 > (120.0 - runtime)) {
            if (1.0 == (Math.ceil(runtime * 2.0) % 2.0)) {
                setRPMLedToColor(Colors.OFF);
            } else {
                setRPMLedToColor(Colors.RED);
            }
        } else if ((120.0 - runtime) < alertTimeEnd) {
            if (1.0 == Math.ceil(runtime * 2.0) % 2.0) {
                setRPMLedToColor(Colors.OFF);
            } else {
                setRPMLedToColor(Colors.ORANGE);
            }

        }
        if (isIndexerFull) {
            if (1.0 == Math.ceil(runtime * 2.0) % 2.0) {
                setLauncherLedToColor(Colors.WHITE);
            } else {
                switch (loadedColor) {
                    case GREEN:
                        setLauncherLedToColor(Colors.GREEN);
                        break;
                    case PURPLE:
                        setLauncherLedToColor(Colors.PURPLE);
                        break;
                    default:
                        setLauncherLedToColor(Colors.OFF);
                }
            }

        } else {
            switch (loadedColor) {
                case GREEN:
                    setLauncherLedToColor(Colors.GREEN);
                    break;
                case PURPLE:
                    setLauncherLedToColor(Colors.PURPLE);
                    break;
                default:
                    setLauncherLedToColor(Colors.OFF);
            }
        }
        setColorsIfHoming(indexerState);
        writeToLeds();
    }

    private void setColorsIfHoming(IndexerFacade.State indexerState) {
        if (IndexerFacade.State.HOMING == indexerState) {
            setRPMLedToColor(Colors.BLUE);
            setLauncherLedToColor(Colors.BLUE);
        }
    }

    private void setRPMLedToColor(Colors color) {
        hueRpmLed = getColor(color);
    }

    void setLauncherLedToColor(Colors color) {
        hueLauncherLed = getColor(color);
    }

    private void writeToLeds() {
        launcherLed.setPosition(hueLauncherLed);
        rpmLed.setPosition(hueRpmLed);
    }

    /**
     * sets the color of the leds based on an input string
     *
     * @param color
     */
    private double getColor(Colors color) {
        switch (color) {
            case OFF:
                hue = 0.0;
                break;
            case RED:
                hue = 0.28;
                break;
            case ORANGE:
                hue = 0.62;
                break;
            case YELLOW:
                hue = 0.388;
                break;
            case GREEN:
                hue = 0.5;
                break;
            case BLUE:
                hue = 0.611;
                break;
            case INDIGO:
                hue = 0.666;
                break;
            case PURPLE:
                hue = 0.722;
                break;
            case WHITE:
                hue = 1.0;
                break;
            case RAINBOW:
                double seconds = ledTimer.seconds();
                hue = Range.clip(0.22 * Math.sin(seconds * 3.0) + 0.5, 0.28, 0.72);
                break;

        }
        return hue;
    }
}
