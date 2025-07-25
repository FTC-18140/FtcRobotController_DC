package org.firstinspires.ftc.teamcode.Summer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Summer.Robot.ThunderBot2025_Summer;
import org.firstinspires.ftc.teamcode.TBDGamepad;

@TeleOp
@Config
public class Teleop_Summer extends OpMode {

    public TelemetryPacket p = new TelemetryPacket(true);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    ThunderBot2025_Summer robot = new ThunderBot2025_Summer();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();

        robot.fieldCentricDrive(forward, strafe, turn, 1.0, p);

        telemetry.addData("position X: ", robot.drive.pose.position.x);
        telemetry.addData("position Y: ", robot.drive.pose.position.y);
        telemetry.addData("heading: ", Math.toDegrees(robot.drive.pose.heading.toDouble()));
        telemetry.update();

        dashboard.sendTelemetryPacket(p);
    }
}
