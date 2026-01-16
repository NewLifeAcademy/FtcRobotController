package org.firstinspires.ftc.teamcodeiron.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodeiron.IronBot2025;

@Autonomous(name = "Auto - Diagnostics", preselectTeleOp = "Iron2026Decode")
@Config
public class IronBotAutoDiagnostics extends LinearOpMode {
    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {

            // Run front left motor at half speed
            telemetry.addLine("Running front left motor at half speed");
            telemetry.update();
            robot.setMotorPowers(0.5, 0, 0, 0);
            robot.wait(2);
            robot.stopMotors();

            // Run back left motor at half speed
            telemetry.addLine("Running back left motor at half speed");
            telemetry.update();
            robot.setMotorPowers(0, 0.5, 0, 0);
            robot.wait(2);
            robot.stopMotors();

            // Run back right motor at half speed
            telemetry.addLine("Running back right motor at half speed");
            telemetry.update();
            robot.setMotorPowers(0, 0, 0.5, 0);
            robot.wait(2);
            robot.stopMotors();

            // Run front right motor at half speed
            telemetry.addLine("Running front right motor at half speed");
            telemetry.update();
            robot.setMotorPowers(0, 0, 0, 0.5);
            robot.wait(2);
            robot.stopMotors();

            // Test Intake
            telemetry.addLine("Testing intake mechanism");
            telemetry.update();
            robot.startIntake();
            robot.wait(2);
            robot.stopIntake();

            // Test Spinarizer
            telemetry.addLine("Testing spinarizer increment");
            telemetry.update();
            robot.incrementSpinarizer();
            robot.wait(1);
            robot.incrementSpinarizer();
            robot.wait(1);
            robot.incrementSpinarizer();

            // Test pusher servo
            telemetry.addLine("Testing pusher servo");
            telemetry.update();
            robot.triggerPusher();
            robot.wait(2);

            // Test flywheels
            telemetry.addLine("Testing flywheels for 3 seconds");
            telemetry.update();
            robot.startFlywheels(.5);
            robot.wait(3);
            robot.stopFlywheels();

            // Test artifact detection sensor
            telemetry.addLine("Testing artifact sensor");
            telemetry.update();
            robot.getSensorDistance(telemetry);

            // Testing camera image capture
            telemetry.addLine("Testing camera image capture");
            telemetry.update();
            robot.telemetryAprilTag(telemetry);

            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
