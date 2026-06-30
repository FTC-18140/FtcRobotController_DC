package org.firstinspires.ftc.teamcode.Robot.Teleops;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2026;


/**
 * Main TeleOp control program for the Blue Alliance during the Postseason.
 * <p>
 * This OpMode coordinates the {@link ThunderBot2026} hardware with dual-gamepad
 * controls, featuring field-centric driving, automated turret aiming,
 * and state-managed intake/indexing systems.
 * </p>
 *
 * @see ThunderBot2026
 */

@TeleOp(group = Teleop_Base.MATCH_TELEOP_GROUP)
public class Teleop_Blue_Postseason extends Teleop_Base
{
    @Override
    protected ThunderBot2026.Alliance_Color getAlliance()
    {
        return ThunderBot2026.Alliance_Color.BLUE;
    }
}
