package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED {
    private Telemetry telemetry = null;
    private Servo rpmLed = null;
    private Servo colorLed = null;
    private Servo intakeLed = null;

    private final ElapsedTime ledTimer = new ElapsedTime();

    public enum Colors {
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
    private double hueIntakeLed = 1.0;
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
            colorLed = hwMap.servo.get("led2");
            setLauncherLedToColor(Colors.RED);
        } catch (RuntimeException e) {
            telemetry.addData("led2 not found in configuration", 0);
        }
        try {
            intakeLed = hwMap.servo.get("led3");
            setIntakeLedToColor(Colors.RED);
        } catch (RuntimeException e) {
            telemetry.addData("led3 not found in configuration", 0);
        }
        lowerBoundRpm = lowerBoundRpmIn;
        upperBoundRpm = upperBoundRpmIn;
    }

    public void update(boolean inLaunchZone, double runtime, IndexerFacade.BallState loadedColor, boolean isIndexerFull, boolean isIntakeFull, IndexerFacade.State indexerState) {
        if (inLaunchZone) {
            setRPMLedToColor(Colors.YELLOW);
        } else {
            setRPMLedToColor(Colors.OFF);
        }
        double alertTimeEnd = 10.0;
        if (5.0 > (120.0 - runtime)) {
            if (1.0 == (Math.ceil(runtime * 2.0) % 2.0)) {
            } else {
//                setAllLedsToColor(Colors.RED);
            }
        } else if ((120.0 - runtime) < alertTimeEnd) {
            if (1.0 == Math.ceil(runtime * 2.0) % 2.0) {
            } else {
//                setAllLedsToColor(Colors.ORANGE);
            }

        }
        switch (loadedColor) {
            case GREEN:
                setLauncherLedToColor(Colors.GREEN);
                break;
            case PURPLE:
                setLauncherLedToColor(Colors.PURPLE);
                break;
            default:
                setLauncherLedToColor(Colors.OFF);
                break;
        }
        if (isIndexerFull && (1.0 == Math.ceil(runtime * 2.0) % 2.0)) {
            setLauncherLedToColor(Colors.WHITE);
        }

        if (isIntakeFull) {
            setIntakeLedToColor(Colors.RED);
        } else {
            setIntakeLedToColor(Colors.OFF);
        }
        setColorsIfHoming(indexerState);
//        if(runtime > 125) setAllLedsToColor(Colors.OFF);
        writeToLeds();
    }

    private void setColorsIfHoming(IndexerFacade.State indexerState) {
        if (IndexerFacade.State.HOMING == indexerState) {
            setAllLedsToColor(Colors.BLUE);
        }
    }

    private void setRPMLedToColor(Colors color) {
        hueRpmLed = getColor(color);
    }

    private void setIntakeLedToColor(Colors color) {
        hueIntakeLed = getColor(color);
    }

    void setLauncherLedToColor(Colors color) {
        hueLauncherLed = getColor(color);
    }

    private void setAllLedsToColor(Colors color) {
        setRPMLedToColor(color);
        setLauncherLedToColor(color);
        setIntakeLedToColor(color);
    }

    private void writeToLeds() {
        colorLed.setPosition(hueLauncherLed);
        rpmLed.setPosition(hueRpmLed);
        intakeLed.setPosition(hueIntakeLed);
    }

    /**
     * sets the color of the leds based on an input string
     *
     * @param color
     */
    public double getColor(Colors color) {
        switch (color) {
            case OFF:
                hue = 0.0;
                break;
            case RED:
                hue = 0.28;
                break;
            case ORANGE:
                hue = 0.333;
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
