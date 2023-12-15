package org.firstinspires.ftc.teamcodebeta.autonomous.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodebeta.drive.roadrunner.LocalizationTest;
import org.firstinspires.ftc.teamcodebeta.BetaBot2024;

@TeleOp(group = "teamzackbot2023", name = "Robot B - LocalizationTest")
@Disabled
public class BetaLocalizationTest extends LocalizationTest {
    @Override
    public void runOpMode() throws InterruptedException {
        BetaBot2024 drive = new BetaBot2024(hardwareMap);
        opModeCode(drive, -1, -1, 1);
    }
}
