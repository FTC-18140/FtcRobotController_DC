package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp
@Config
public class Teleop_Summer extends OpMode {

    public TelemetryPacket p = new TelemetryPacket(true);
    //public static boolean field_centric = true;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private TBDGamepad theGamepad1;
    private TBDGamepad theGamepad2;

    ThunderBot2025 robot = new ThunderBot2025();

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
        robot.drive.updatePoseEstimate();
        double forward = theGamepad1.getLeftY();
        double strafe = theGamepad1.getLeftX();
        double turn = theGamepad1.getRightX();



//        if(robot.isFieldCentric())
//        {
//            robot.fieldCentricDrive(forward, strafe, turn, 0.7, p);
//        } else {
//            robot.robotCentricDrive(forward, strafe, turn, 0.7);
//        }

        robot.drive(forward, strafe, turn, 0.7, p);

        if(theGamepad2.getButton(TBDGamepad.Button.X)){
            robot.intake.intake();
        } else if (theGamepad2.getButton(TBDGamepad.Button.B)) {
            robot.intake.stop();
        }else if (theGamepad2.getButton(TBDGamepad.Button.A)) {
            robot.intake.spit();
        }


        if(theGamepad2.getTrigger(TBDGamepad.Trigger.LEFT_TRIGGER) >= 0.1){
            robot.launcher.shoot(robot.drive.localizer.getPose());
        } else if(theGamepad2.getTrigger(TBDGamepad.Trigger.LEFT_TRIGGER) > 0.01){
            robot.launcher.launchMin();
        } else {
            robot.launcher.stop();
        }

        //Flipper / launch controls
        if(theGamepad2.getTrigger(TBDGamepad.Trigger.RIGHT_TRIGGER) > 0.1){
            robot.indexer.flip();
        }else{
            //Prevents indexer from interfering with Flipper
            robot.indexer.unflip();

            if(theGamepad2.getButton(TBDGamepad.Button.LEFT_BUMPER)){
                robot.indexer.spin(-0.3);
            } else if (theGamepad2.getButton(TBDGamepad.Button.RIGHT_BUMPER)) {
                robot.indexer.spin(0.3);
            } else if(theGamepad2.getButton(TBDGamepad.Button.DPAD_LEFT)){
                robot.indexer.cycle(-1);
            } else if (theGamepad2.getButton(TBDGamepad.Button.DPAD_RIGHT)) {
                robot.indexer.cycle(1);
            }else{
                robot.indexer.stop();
            }
        }

        telemetry.addData("position X: ", robot.drive.localizer.getPose().position.x);
        telemetry.addData("position Y: ", robot.drive.localizer.getPose().position.y);
        telemetry.addData("heading: ", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("speed: ", robot.launcher.tps);
        telemetry.addData("flipper: ", robot.indexer.getFlipperPos());
        telemetry.addData("goal distance: ", robot.launcher.goalDistance(robot.drive.localizer.getPose()));


        dashboard.sendTelemetryPacket(p);
    }
}
