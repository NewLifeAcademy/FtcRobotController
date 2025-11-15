package org.firstinspires.ftc.teamcodewings;

import android.os.Build;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Check Android API Level")
public class CheckApiLevelOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Wait for start
        waitForStart();
        if (opModeIsActive()) {
            int apiLevel = Build.VERSION.SDK_INT;
            telemetry.addData("Android API Level", apiLevel);
            telemetry.addData("Android Version", Build.VERSION.RELEASE);
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
        }
    }
}
