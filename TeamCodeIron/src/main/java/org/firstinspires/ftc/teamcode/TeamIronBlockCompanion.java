package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcodeiron.IronBot2025;

import java.io.File;

public class TeamIronBlockCompanion extends BlocksOpModeCompanion {
    static IronBot2025 robot = new IronBot2025(hardwareMap);

    @ExportToBlocks(
            comment = "Set the robot's drive motor powers",
            tooltip = "Set the robot's drive motor powers. Suggestion: Use -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x",
            parameterLabels = {"Drive Power", "Strafe Power", "Rotation Power"},
            parameterDefaultValues = {"0", "0", "0"}
    )
    public static void robotMotorPower(double drivePower, double strafePower, double rotationPower) {
        robot.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        drivePower,
                        strafePower
                ),
                rotationPower
        ));
    }

    @ExportToBlocks(
            comment = "Set the robot's initial pose (position and heading)",
            tooltip = "Set the robot's initial pose (position in inches and heading in degrees)",
            parameterLabels = {"X Position (inches)", "Y Position (inches)", "Heading (degrees)"},
            parameterDefaultValues = {"0", "0", "0"}
    )
    public static void setInitialRobotPose(double x, double y, double headingDegrees) {
        robot.localizer.setPose(new Pose2d(x, y, Math.toRadians(headingDegrees)));
    }
}
