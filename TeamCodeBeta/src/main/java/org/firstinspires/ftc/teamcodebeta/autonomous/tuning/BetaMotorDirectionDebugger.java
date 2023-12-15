package org.firstinspires.ftc.teamcodebeta.autonomous.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodebeta.drive.roadrunner.MotorDirectionDebugger;
import org.firstinspires.ftc.teamcodebeta.BetaBot2024;

@TeleOp(group = "teamzackbot2023", name = "Robot B - MotorDirectionDebugger")
@Disabled
public class BetaMotorDirectionDebugger extends MotorDirectionDebugger {

    @Override
    public void runOpMode() throws InterruptedException {
        BetaBot2024 drive = new BetaBot2024(hardwareMap);
        super.opModeCode(drive);
    }
}
