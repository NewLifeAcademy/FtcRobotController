package org.firstinspires.ftc.teamcodealpha.autonomous.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodealpha.drive.roadrunner.LocalizationTest;
import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;

@Disabled
@TeleOp(group = "teamgrantbot2023", name = "Robot A - LocalizationTest")
public class AlphaLocalizationTest extends LocalizationTest {

    @Override
    public void runOpMode() throws InterruptedException {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);
        opModeCode(drive, 1, 1, 1);
    }
}
