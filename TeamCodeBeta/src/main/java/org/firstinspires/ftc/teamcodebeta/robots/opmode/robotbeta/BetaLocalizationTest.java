package org.firstinspires.ftc.teamcodebeta.robots.opmode.robotbeta;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodebeta.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcodebeta.robots.BetaBot2024;

@TeleOp(group = "teamzackbot2023", name = "Robot B - LocalizationTest")
@Disabled
public class BetaLocalizationTest extends LocalizationTest {
    @Override
    public void runOpMode() throws InterruptedException {
        BetaBot2024 drive = new BetaBot2024(hardwareMap);
        opModeCode(drive, -1, -1, 1);
    }
}
