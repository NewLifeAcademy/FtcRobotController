package org.firstinspires.ftc.teamcodespain;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Arrays;
import java.util.List;

/**
 * FTC 17240 Spain Bot 2025: GoBuilda Mecanum Chassis
 */
public class SpainBot2025 extends MecanumDrive {

    public SparkFunLEDStick ledStick;
    private HardwareMap hardwareMap;
    // AprilTag detection members
    private AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public SpainBot2025(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        // Save the hardware map reference
        this.hardwareMap = hardwareMap;

        // Configure the LED stick
        ledStick = hardwareMap.get(SparkFunLEDStick.class, "backLEDStick");
        ledStick.setBrightness(1);  // Set initial brightness (0-31)
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void odometryBasedMove(double forwardInches, double strafeInches, double rotationDegrees) {
        Action action = this.actionBuilder(this.localizer.getPose())
                .splineTo(new Vector2d(forwardInches, strafeInches), Math.toRadians(rotationDegrees))
                .build();

        Actions.runBlocking(new SequentialAction(action));
    }

    public void wait(int seconds) {
        try {
            Thread.sleep(seconds * 1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void initAprilTagDetection() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }
    public AprilTagDetection telemetryAprilTag(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return currentDetections.isEmpty() ? null : currentDetections.get(0);
    }   // end method telemetryAprilTag()
}
