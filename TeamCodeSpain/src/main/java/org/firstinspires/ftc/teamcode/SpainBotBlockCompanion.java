package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcodespain.SpainBot2025;

import java.io.File;

public class SpainBotBlockCompanion extends BlocksOpModeCompanion {
    static SpainBot2025 robot = new SpainBot2025(hardwareMap);

    @ExportToBlocks (
            comment = "Write the lift encoder value to a file on the device",
            tooltip = "Write the lift encoder value",
            parameterLabels = {"Lift Encoder Value"},
            parameterDefaultValues = {"0"}
    )
    public static void writeLiftEncoderToFile(int liftEncoderValue) {
        File file = new File("/sdcard/FIRST/SpainBotLiftEncoder.txt");

        ReadWriteFile.writeFile(file, String.valueOf(liftEncoderValue));
    }

    @ExportToBlocks (
            comment = "Read the lift encoder value from a file on the device",
            tooltip = "Read the lift encoder value",
            parameterLabels = {"Lift Encoder Value"},
            parameterDefaultValues = {"0"}
    )
    public static double readLiftEncoderFromFile() {
        File file = new File("/sdcard/FIRST/SpainBotLiftEncoder.txt");
        String liftEncoderValue = ReadWriteFile.readFile(file).trim();
        return Double.parseDouble(liftEncoderValue);
    }

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

    @ExportToBlocks(
            comment = "Move the robot using odometry to a specified position and orientation",
            tooltip = "Move the robot using odometry. forwardInches and strafeInches are relative to the robot's current position. rotationDegrees is absolute.",
            parameterLabels = {"Forward Inches", "Strafe Inches", "Rotation Degrees"},
            parameterDefaultValues = {"0", "0", "0"}
    )
    public static void robotOdometryMove(double forwardInches, double strafeInches, double rotationDegrees) {
        robot.odometryBasedMove(forwardInches, strafeInches, rotationDegrees);
    }
}
