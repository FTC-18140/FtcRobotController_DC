package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Indexer;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp
@Config
public class Teleop_BLUE extends OpMode {

    public TelemetryPacket p = new TelemetryPacket(true);
    private boolean barrel_spin = false;
    private boolean revolving = false;
    //public static boolean field_centric = true;

    String alliance = "blue";

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    ThunderBot2025 robot = new ThunderBot2025();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, new Pose2d(-12, 12, 0));
//        robot.launcher.color = alliance;
        robot.setColor(alliance);

        theGamepad1 = new TBDGamepad(gamepad1);
        theGamepad2 = new TBDGamepad(gamepad2);
        // Tell the driver that initialization is complete.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        theGamepad1.update();
        theGamepad2.update();
        robot.update();


        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();
        double speed = 0.7;

        // Speed Adjustment
        if(theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER)){
            speed = 0.3;
        } else if(theGamepad1.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)){
            speed = 1.0;
        }

        // Heading Reset
        if(theGamepad1.getButton(TBDGamepad.Button.Y)){
            robot.drive.localizer.setPose(new Pose2d(robot.drive.localizer.getPose().position, 0));
        }


        // Turret Aiming
        if(Math.abs(theGamepad2.getRightX()) > 0.05){
            robot.launcher.augmentedAim(-1.2 *theGamepad2.getRightX() + theGamepad1.getRightX() * speed);
        } else if(Math.abs(theGamepad1.getRightX()) > 0.05){
            robot.launcher.augmentedAim(theGamepad1.getRightX() * speed * 0.7 );
        } else {
            robot.launcher.aim();
        }

        // Chassis
        robot.drive(forward, strafe, turn * 0.7, speed, p);

        // Intake
        if(theGamepad2.getButton(TBDGamepad.Button.X)){
            robot.intake.intake();
        } else if (theGamepad2.getButton(TBDGamepad.Button.B)) {
            robot.intake.stop();
        } else if (theGamepad2.getButton(TBDGamepad.Button.A)) {
            robot.intake.spit();
        }

        // Flywheel Prep
        if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.LEFT_TRIGGER)){
            robot.launcher.prepShot();
        } else {
            robot.launcher.stop();
        }

        //Flipper / launch controls
        if(theGamepad2.getTriggerBoolean(TBDGamepad.Trigger.RIGHT_TRIGGER)){
            robot.indexer.flip();
        } else {
            robot.indexer.unflip();
        }

        // Indexer
        if(theGamepad2.getButton(TBDGamepad.Button.LEFT_STICK_BUTTON)){
            robot.indexer.adjustToThird();
        }else if(theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER)){
            robot.indexer.spin(-0.2);
        } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
            robot.indexer.spin(0.2);
        } else if(theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_LEFT)){
            robot.indexer.cycle(-1);
        } else if (theGamepad2.getButtonPressed(TBDGamepad.Button.DPAD_RIGHT)) {
            robot.indexer.cycle(1);
        } else if (robot.indexer.getState() == Indexer.IndexerState.MANUAL){
            robot.indexer.setState(Indexer.IndexerState.UNALIGNED);
        }

        telemetry.addData("position X: ", robot.drive.localizer.getPose().position.x);
        telemetry.addData("position Y: ", robot.drive.localizer.getPose().position.y);
        telemetry.addData("heading: ", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
//        telemetry.addData("rpm: ", robot.launcher.avgRpm);
//        telemetry.addData("goal distance: ", robot.launcher.goalDistance(robot.drive.localizer.getPose()));
//        telemetry.addData("target rpm: ", robot.launcher.targetRpm);

        dashboard.sendTelemetryPacket(p);
    }
}
