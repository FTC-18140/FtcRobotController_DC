package org.firstinspires.ftc.teamcode.Summer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Summer.Robot.ThunderBot2025_Summer;
import org.firstinspires.ftc.teamcode.TBDGamepad;

public class Teleop_Summer extends OpMode {

    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    ThunderBot2025_Summer robot = new ThunderBot2025_Summer();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();

        robot.fieldCentricDrive(forward, strafe, turn, 1.0);

    }
}
