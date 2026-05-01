package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class TEST_IndexerServo1 extends OpMode {
    CRServo indexer1 = null;
    @Override
    public void init() {
        indexer1 = hardwareMap.crservo.get("indexer");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            indexer1.setPower(1);
        } else if (gamepad1.b) {
            indexer1.setPower(-1);
        } else {
            indexer1.setPower(0);
        }
    }
}
