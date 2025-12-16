package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;
import org.firstinspires.ftc.teamcode.Utilities.TBDGamepad;

@TeleOp
public class IndexerTest extends OpMode {
    private TBDGamepad theGamepad1;
    ThunderBot2025 robot = new ThunderBot2025();
    private boolean firing = false;

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
            robot.indexer.selectSlot(0);
        } else if (theGamepad1.getButton(TBDGamepad.Button.A)) {
            robot.indexer.selectSlot(1);
        } else if (theGamepad1.getButton(TBDGamepad.Button.B)) {
            robot.indexer.selectSlot(2);
        }

        if(theGamepad1.getButtonPressed(TBDGamepad.Button.RIGHT_BUMPER)){
            robot.indexer.launchAllInIndexer();
        }

        if(theGamepad1.getButtonPressed(TBDGamepad.Button.Y)){
            firing = true;
        }
        if(firing){
            robot.charge();
            if(robot.launcher.isAtTargetRpm()) {
                firing = !robot.indexer.runCurrentSequence();
            }
        } else {
            robot.launcher.stop();
        }

//        telemetry.addData("rpm: ", robot.launcher.avgRpm);
    }
}
