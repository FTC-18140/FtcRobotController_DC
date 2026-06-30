package org.firstinspires.ftc.teamcode.Robot.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2026;

@TeleOp(group = Teleop_Base.MATCH_TELEOP_GROUP)
public class Teleop_Red_Postseason extends Teleop_Base
{
    @Override
    protected ThunderBot2026.Alliance_Color getAlliance()
    {
        return ThunderBot2026.Alliance_Color.RED;
    }
}
