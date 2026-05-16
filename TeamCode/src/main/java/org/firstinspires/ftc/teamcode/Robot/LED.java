package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LED
{
    private Telemetry telemetry = null;
    private Servo rpmLed = null;
    private Servo colorLed = null;
    private Servo intakeLed = null;

    private final ElapsedTime ledTimer = new ElapsedTime();

    public enum Colors
    {
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

    private Colors lastRpmColor = null;
    private Colors lastLauncherColor = null;
    private Colors lastIntakeColor = null;
    private boolean rpmDirty = true;
    private boolean launcherDirty = true;
    private boolean intakeDirty = true;

    public void init(HardwareMap hwMap, Telemetry telem, double lowerBoundRpmIn, double upperBoundRpmIn)
    {
        telemetry = telem;

        try
        {
            rpmLed = hwMap.servo.get("led");
            setRPMLedToColor(Colors.RED);
        }
        catch (RuntimeException e)
        {
            telemetry.addData("led not found in configuration", 0);
        }
        try
        {
            colorLed = hwMap.servo.get("led2");
            setLauncherLedToColor(Colors.RED);
        }
        catch (RuntimeException e)
        {
            telemetry.addData("led2 not found in configuration", 0);
        }
        try
        {
            intakeLed = hwMap.servo.get("led3");
            setIntakeLedToColor(Colors.RED);
        }
        catch (RuntimeException e)
        {
            telemetry.addData("led3 not found in configuration", 0);
        }
        lowerBoundRpm = lowerBoundRpmIn;
        upperBoundRpm = upperBoundRpmIn;
    }

    public void update(boolean inLaunchZone, boolean overridden, double runtime, TransferFacade.BallState loadedColor, boolean isIndexerFull, boolean isIntakeFull, TransferFacade.State indexerState)
    {
        if (inLaunchZone)
        {
            setRPMLedToColor(Colors.YELLOW);
        }
        else
        {
            setRPMLedToColor(Colors.OFF);
        }

        switch (loadedColor)
        {
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
        if (isIndexerFull && (1.0 == Math.ceil(runtime * 2.0) % 2.0))
        {
            setLauncherLedToColor(Colors.WHITE);
        }

        if (isIntakeFull)
        {
            setIntakeLedToColor(Colors.RED);
        }
        else if (overridden)
        {
            setIntakeLedToColor(Colors.WHITE);
        }
        else
        {
            setIntakeLedToColor(Colors.OFF);
        }
        setColorsIfHoming(indexerState);
        writeToLeds();
    }

    private void setColorsIfHoming(TransferFacade.State indexerState)
    {
        if (TransferFacade.State.HOMING == indexerState)
        {
            setAllLedsToColor(Colors.BLUE);
        }
    }

    private void setRPMLedToColor(Colors color)
    {
        if (color == Colors.RAINBOW || color != lastRpmColor)
        {
            hueRpmLed = getColor(color);
            lastRpmColor = color;
            rpmDirty = true;
        }
    }

    private void setIntakeLedToColor(Colors color)
    {
        if (color != lastIntakeColor)
        {
            hueIntakeLed = getColor(color);
            lastIntakeColor = color;
            intakeDirty = true; // Mark as needing an update
        }
    }

    void setLauncherLedToColor(Colors color)
    {
        if (color != lastLauncherColor)
        {
            hueLauncherLed = getColor(color);
            lastLauncherColor = color;
            launcherDirty = true; // Mark as needing an update
        }
    }

    private void setAllLedsToColor(Colors color)
    {
        setRPMLedToColor(color);
        setLauncherLedToColor(color);
        setIntakeLedToColor(color);
    }

    private void writeToLeds()
    {
        if (rpmDirty && rpmLed != null)
        {
            rpmLed.setPosition(hueRpmLed);
            rpmDirty = false;
        }

        if (launcherDirty && colorLed != null)
        {
            colorLed.setPosition(hueLauncherLed);
            launcherDirty = false;
        }

        if (intakeDirty && intakeLed != null)
        {
            intakeLed.setPosition(hueIntakeLed);
            intakeDirty = false;
        }
    }

    /**
     * sets the color of the leds based on an input string
     *
     * @param color
     */
    public double getColor(Colors color)
    {
        switch (color)
        {
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
