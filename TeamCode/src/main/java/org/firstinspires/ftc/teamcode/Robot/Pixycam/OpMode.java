package org.firstinspires.ftc.teamcode.Robot.Pixycam;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Teleop")

public class OpMode extends LinearOpMode {

    private Pixy pixy;

    @Override
    public void runOpMode() throws InterruptedException {

        pixy = hardwareMap.get(Pixy.class, "pixy");
        PixyBlock detectedBlock;

        pixy.turnOffLamps();
        pixy.turnOnLamps();

        waitForStart();
        while(opModeIsActive()){

            detectedBlock = pixy.getBlock();

            telemetry.addLine(detectedBlock.toString());

            if(detectedBlock.isValid()){
                // do something
            }
            telemetry.update();
        }
    }
}
