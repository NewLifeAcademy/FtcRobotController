package org.firstinspires.ftc.teamcode.robots.opmode.robotbeta;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.MotorDirectionDebugger;
import org.firstinspires.ftc.teamcode.robots.BetaBot2024;

@TeleOp(group = "teamzackbot2023", name = "Robot B - MotorDirectionDebugger")
@Disabled
public class BetaMotorDirectionDebugger extends MotorDirectionDebugger {

    @Override
    public void runOpMode() throws InterruptedException {
        BetaBot2024 drive = new BetaBot2024(hardwareMap);
        super.opModeCode(drive);
    }
}
