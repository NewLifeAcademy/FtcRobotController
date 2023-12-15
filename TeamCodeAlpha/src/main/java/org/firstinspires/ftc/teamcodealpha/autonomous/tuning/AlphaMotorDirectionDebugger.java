package org.firstinspires.ftc.teamcodealpha.autonomous.tuning;

import com.acmerobotics.dashboard.canvas.Alpha;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.drive.roadrunner.MotorDirectionDebugger;

@TeleOp(group = "Alpha", name = "Alpha - MotorDirectionDebugger")
public class AlphaMotorDirectionDebugger extends MotorDirectionDebugger {

    @Override
    public void runOpMode() throws InterruptedException {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);
        super.opModeCode(drive);
    }
}
