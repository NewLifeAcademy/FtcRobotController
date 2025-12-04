package org.firstinspires.ftc.teamcodeiron.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodeiron.IronBot2025;

@Autonomous(name = "Auto - Mecanum Wheel Testing")
@Disabled
@Config
public class IronBotAutoMecanumWheelTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {

            // Run front left motor at half speed for 3 seconds
            telemetry.addLine("Running front left motor at half speed for 3 seconds");
            telemetry.update();
            robot.setMotorPowers(0.5, 0, 0, 0);
            robot.wait(3);
            robot.stopMotors();

            // Run back left motor at half speed for 3 seconds
            telemetry.addLine("Running back left motor at half speed for 3 seconds");
            telemetry.update();
            robot.setMotorPowers(0, 0.5, 0, 0);
            robot.wait(3);
            robot.stopMotors();

            // Run back right motor at half speed for 3 seconds
            telemetry.addLine("Running back right motor at half speed for 3 seconds");
            telemetry.update();
            robot.setMotorPowers(0, 0, 0.5, 0);
            robot.wait(3);
            robot.stopMotors();

            // Run front right motor at half speed for 3 seconds
            telemetry.addLine("Running front right motor at half speed for 3 seconds");
            telemetry.update();
            robot.setMotorPowers(0, 0, 0, 0.5);
            robot.wait(3);
            robot.stopMotors();

            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
