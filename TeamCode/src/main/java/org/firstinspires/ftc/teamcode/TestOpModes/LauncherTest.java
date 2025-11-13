package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp
public class LauncherTest extends OpMode {
    private TBDGamepad theGamepad1;
    ThunderBot2025 robot = new ThunderBot2025();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, new Pose2d(0,0, 0));

        theGamepad1 = new TBDGamepad(gamepad1);
    }

    @Override
    public void loop() {
        theGamepad1.update();
        robot.update();
        if(theGamepad1.getButton(TBDGamepad.Button.X)){
            robot.launcher.launchMax();
        } else if (theGamepad1.getButton(TBDGamepad.Button.B)) {
            robot.launcher.stop();
        }

        if(theGamepad1.getButtonPressed(TBDGamepad.Button.RIGHT_BUMPER)){
            robot.lockOn();
        }

        telemetry.addData("rpm: ", robot.launcher.avgRpm);
    }
}
