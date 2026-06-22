package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_GAMEPAD;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

//@Config
public class TBDGamepad
{
    private Telemetry telemetry;
    public Gamepad gamepad;

    public static double expoYValue = 2.5;
    public static double expoXValue = 2.5;
    public boolean[] buttons = new boolean[14];
    public boolean[] oldButtons = new boolean[14];
    public boolean[] changed = new boolean[14];
    private Colors currentLedColorEnum = Colors.OFF; // Track the last set color

    static final long RUMBLE_INTERVAL_MS = 500;
    private long lastRumbleTime = 0;

    public boolean leftOldTrigger = false;
    public boolean leftNewTrigger = false;
    public double triggerThreshold = 0.5;
    public boolean rightOldTrigger = false;
    public boolean rightNewTrigger = false;
    public boolean leftTriggerChanged = false;
    public boolean rightTriggerChanged = false;

    public enum Button
    {
        A(0), B(1), X(2), Y(3), LEFT_BUMPER(4), RIGHT_BUMPER(5), BACK(6),
        START(7), DPAD_UP(8), DPAD_DOWN(9), DPAD_LEFT(10), DPAD_RIGHT(11),
        LEFT_STICK_BUTTON(12), RIGHT_STICK_BUTTON(13), TRIANGLE(3), CROSS(0), SQUARE(2), CIRCLE(1);
        final int index;

        Button(int ind)
        {
            this.index = ind;
        }
    }

    /**
     * Represents the available LED colors for the gamepad.
     * Each enum constant directly embeds its RGB values (0.0 to 1.0).
     */
    public enum Colors
    {
        // Enum constants with embedded RGB values
        OFF(0.0, 0.0, 0.0),
        RED(1.0, 0.0, 0.0),
        ORANGE(1.0, 0.5, 0.0),
        YELLOW(1.0, 1.0, 0.0),
        YELLOWGREEN(0.5, 1.0, 0.0),
        GREEN(0.0, 1.0, 0.0),
        GREENBLUE(0.0, 1.0, 1.0),
        BLUE(0.0, 0.0, 1.0),
        INDIGO(0.5, 0.0, 1.0),
        PURPLE(1.0, 0.0, 1.0),
        WHITE(1.0, 1.0, 1.0);

        // Private fields to store the RGB components
        private final double r, g, b;

        // Enum constructor to initialize the RGB values for each constant
        Colors(double r, double g, double b)
        {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public enum Trigger
    {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    public enum Stick
    {
        LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y
    }

    public TBDGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
        Arrays.fill(buttons, false);
        Arrays.fill(oldButtons, false);
        Arrays.fill(changed, false);
    }

    public void init(Telemetry telem)
    {
        telemetry = telem;
    }

    /**
     * @param button the button object
     * @return the boolean value as to whether the button is active or not
     */
    public boolean getButton(Button button)
    {
        boolean buttonValue = false;
        switch (button)
        {
            case A:
                case CROSS:
                buttonValue = gamepad.a;
                break;
            case B:
            case CIRCLE:
                buttonValue = gamepad.b;
                break;
            case X:
                case SQUARE:
                buttonValue = gamepad.x;
                break;
            case Y:
            case TRIANGLE:
                buttonValue = gamepad.y;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper;
                break;
            case DPAD_UP:
                buttonValue = gamepad.dpad_up;
                break;
            case DPAD_DOWN:
                buttonValue = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                buttonValue = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonValue = gamepad.dpad_right;
                break;
            case BACK:
                buttonValue = gamepad.back;
                break;
            case START:
                buttonValue = gamepad.start;
                break;
            case LEFT_STICK_BUTTON:
                buttonValue = gamepad.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                buttonValue = gamepad.right_stick_button;
                break;
            default:
                break;
        }
        return buttonValue;
    }

    /**
     * @param trigger the trigger object
     * @return the value returned by the trigger in question
     */
    public double getTrigger(Trigger trigger)
    {
        double triggerValue = 0;
        switch (trigger)
        {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger;
                break;
            default:
                break;
        }
        return triggerValue;
    }

    public boolean getTriggerBoolean(Trigger trigger)
    {
        boolean triggerValue = false;
        switch (trigger)
        {
            case LEFT_TRIGGER:
                triggerValue = 0.1 < gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = 0.1 < gamepad.right_trigger;
                break;
            default:
                break;
        }
        return triggerValue;
    }

    /**
     * @return the y-value on the left analog stick
     */
    public double getLeftY()
    {
        return -gamepad.left_stick_y;
    }


    /**
     * @return the y-value on the right analog stick
     */
    public double getRightY()
    {
        return -gamepad.right_stick_y;
    }

    /**
     * @return the x-value on the left analog stick
     */
    public double getLeftX()
    {
        return gamepad.left_stick_x;
    }

    /**
     * @return the x-value on the right analog stick
     */
    public double getRightX()
    {
        return gamepad.right_stick_x;
    }

    public double getExpo(Stick stick)
    {
        switch (stick)
        {
            case LEFT_X:
                return Math.pow(getLeftX(), expoXValue);
            case LEFT_Y:
                return Math.pow(getLeftY(), expoYValue);
            case RIGHT_X:
                return Math.pow(getRightX(), expoXValue);
            case RIGHT_Y:
                return Math.pow(getRightY(), expoYValue);
            default:
                return 0;
        }
    }

    public boolean getButtonPressed(Button theButton)
    {
        return changed[theButton.index] && buttons[theButton.index];
    }

    public boolean getButtonReleased(Button theButton)
    {
        return changed[theButton.index] && !buttons[theButton.index];
    }

    public boolean getTriggerPressed(Trigger theTrigger)
    {
        if (Trigger.LEFT_TRIGGER == theTrigger)
        {
            return leftTriggerChanged && leftNewTrigger;
        }
        else
        {
            return rightTriggerChanged && rightNewTrigger;
        }

    }

    /**
     * Notifies the driver by rumbling the gamepad a specified number of times.
     * This method is rate-limited to prevent excessive calls, ensuring it
     * is called at most twice per second.
     *
     * @param numBlips The number of blips to rumble.
     */
    public void notifyDriver(int numBlips)
    {
        long currentTime = System.currentTimeMillis();

        // Check if enough time has passed since the last rumble.
        if (currentTime - lastRumbleTime >= RUMBLE_INTERVAL_MS)
        {
            gamepad.rumbleBlips(numBlips);
            lastRumbleTime = currentTime; // Update the last rumble time
        }
        // If not enough time has passed, the rumble call is skipped.
    }

    /**
     * Sets the LED color of the gamepad using a Colors enum.
     * This method tracks the desired enum color and delegates to setLedColorRGB
     * to handle the actual hardware call with redundancy checks.
     *
     * @param color The desired Colors enum value.
     */
    public void setLedColor(Colors color)
    {
        // Primary check: Only proceed if the *enum color* itself has changed from the last processed enum.
        if (this.currentLedColorEnum != color)
        {
            gamepad.setLedColor(color.r, color.g, color.b, Gamepad.LED_DURATION_CONTINUOUS);
            // Update the tracked enum color *only after* potentially setting the RGB.
            this.currentLedColorEnum = color;
        }
    }

    public void blipDriver()
    {
        notifyDriver(1);
    }

    public void update()
    {
        System.arraycopy(buttons, 0, oldButtons, 0, 14);

        buttons[Button.A.index] = gamepad.a;
        buttons[Button.B.index] = gamepad.b;
        buttons[Button.X.index] = gamepad.x;
        buttons[Button.Y.index] = gamepad.y;
        buttons[Button.LEFT_BUMPER.index] = gamepad.left_bumper;
        buttons[Button.RIGHT_BUMPER.index] = gamepad.right_bumper;
        buttons[Button.DPAD_UP.index] = gamepad.dpad_up;
        buttons[Button.DPAD_DOWN.index] = gamepad.dpad_down;
        buttons[Button.DPAD_LEFT.index] = gamepad.dpad_left;
        buttons[Button.DPAD_RIGHT.index] = gamepad.dpad_right;
        buttons[Button.BACK.index] = gamepad.back;
        buttons[Button.START.index] = gamepad.start;
        buttons[Button.LEFT_STICK_BUTTON.index] = gamepad.left_stick_button;
        buttons[Button.RIGHT_STICK_BUTTON.index] = gamepad.right_stick_button;
        leftOldTrigger = leftNewTrigger;
        leftNewTrigger = gamepad.left_trigger > triggerThreshold;
        leftTriggerChanged = leftOldTrigger != leftNewTrigger;
        rightOldTrigger = rightNewTrigger;
        rightNewTrigger = gamepad.right_trigger > triggerThreshold;
        rightTriggerChanged = rightOldTrigger != rightNewTrigger;

        for (int i = 0; 14 > i; i++)
        {
            changed[i] = oldButtons[i] != buttons[i];
        }

        if ( DEBUG_GAMEPAD )
        {
            telemetry.addData("dpad down:", gamepad.dpad_down );
        }
    }


}
