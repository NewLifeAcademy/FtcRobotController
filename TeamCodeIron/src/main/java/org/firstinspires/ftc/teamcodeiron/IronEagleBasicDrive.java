/*
Java based backup of our IronEagleStrafe2025 OpMode block code (10-31-2024)
*/

package org.firstinspires.ftc.teamcodeiron;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;

@TeleOp(name = "Basic Drive")
@Config
public class IronEagleBasicDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.initAprilTagDetection();

        waitForStart();

        // implementation from LocalizationTest.java
        while (opModeIsActive()) {
            robot.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            robot.updatePoseEstimate();

            while(gamepad1.a) {
                robot.telemetryAprilTag(telemetry);
                telemetry.update();
            }

            Pose2d pose = robot.localizer.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }   // end runOpMode()
}
