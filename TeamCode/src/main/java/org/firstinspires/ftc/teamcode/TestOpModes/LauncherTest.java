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
    ThunderBot2025 robot = null;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, new Pose2d(0,0, 0));

        theGamepad1 = new TBDGamepad(gamepad1);
    }

    @Override
    public void loop() {
        theGamepad1.update();
        if(theGamepad1.getButton(TBDGamepad.Button.X)){
        } else if (theGamepad1.getButton(TBDGamepad.Button.B)) {
        }

        if(theGamepad1.getButtonPressed(TBDGamepad.Button.RIGHT_BUMPER)){
            robot.lockOn();
        }
    }
}
