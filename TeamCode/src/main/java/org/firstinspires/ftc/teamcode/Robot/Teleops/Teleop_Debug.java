package org.firstinspires.ftc.teamcode.Robot.Teleops;

import static org.firstinspires.ftc.teamcode.Utilities.TBDGamepad.Button.CROSS;
import static org.firstinspires.ftc.teamcode.Utilities.TBDGamepad.Button.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.Utilities.TBDGamepad.Button.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Utilities.TBDGamepad.Button.TRIANGLE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2026;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp(group = Teleop_Debug.MATCH_TELEOP_GROUP)
public class Teleop_Debug extends OpMode
{

    public static final String MATCH_TELEOP_GROUP = "AAAMatchTeleops";
//    public TelemetryPacket p = new TelemetryPacket(true);
    // --- Mode States ---

    ThunderBot2026.Alliance_Color alliance = ThunderBot2026.Alliance_Color.BLUE;

    private TBDGamepad theGamepad1 = null;
    private TBDGamepad theGamepad2 = null;

    ThunderBot2026 robot = new ThunderBot2026();
    private double currentValue = 0.0;
    public static double increment = 50;

    @Override
    public void init()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d pose = new Pose2d(0, 0, 0);
        robot.init(hardwareMap, telemetry, pose);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        robot.runtime.reset();
        robot.setColor(alliance);
    }

    @Override
    public void loop()
    {
        robot.update();
        theGamepad1.update();
        theGamepad2.update();

        if (theGamepad2.getButtonPressed(DPAD_UP))
        {
            currentValue += increment;
        }
        if (theGamepad2.getButtonPressed(DPAD_DOWN))
        {
            currentValue -= increment;
        }

        if (theGamepad2.getButtonPressed(TRIANGLE))
        {
            robot.launcher.flywheelController.DEBUG_upperFlywheel(currentValue);
        }

        if (theGamepad2.getButtonPressed(CROSS))
        {
            robot.launcher.flywheelController.DEBUG_lowerFlywheel(currentValue);
        }

        robot.drive.localizer.update();

        telemetry.addData("Flywheel Upper: ", robot.launcher.getUpperFlywheelRpm());
        telemetry.addData("Flywheel Lower: ", robot.launcher.getLowerFlywheelRpm());


//        dashboard.sendTelemetryPacket(p);
        telemetry.update();
    }
}
