package org.firstinspires.ftc.teamcodeiron;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * FTC 17240 Iron Eagles - REV Robotics Chassis
 */
public class IronBot2025 extends MecanumDrive {

    private HardwareMap hardwareMap;
    // AprilTag detection members
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private DcMotor flywheel;
    private CRServo intake;
    private CRServo midspace;
    private CRServo pusher;

    public IronBot2025(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        // Save the hardware map reference
        this.hardwareMap = hardwareMap;

        this.flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        this.intake = hardwareMap.get(CRServo.class, "intake");
        this.midspace = hardwareMap.get(CRServo.class, "midspace");
        this.pusher = hardwareMap.get(CRServo.class, "pusher");
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void wait(int seconds) {
        try {
            Thread.sleep(seconds * 1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void fireArtifacts(int fireDurationSeconds, int flywheelSpinupSeconds, double flywheelPower) {
        // Activate flywheel, intake, midspace, and pusher to fire artifacts
        flywheel.setPower(flywheelPower);

        // Wait for flywheel to spin up
        wait(flywheelSpinupSeconds);

        intake.setPower(-1.0);
        midspace.setPower(1.0);
        pusher.setPower(1.0);

        // Continue firing for the remaining duration
        wait(fireDurationSeconds - flywheelSpinupSeconds);

        // Stop all firing mechanisms
        flywheel.setPower(0);
        intake.setPower(0);
        midspace.setPower(0);
        pusher.setPower(0);
    }

    public void startIntake() {
        intake.setPower(-1.0);
        midspace.setPower(1.0);
    }

    public void stopIntake() {
        intake.setPower(0.0);
        midspace.setPower(0.0);
    }

    public void initAprilTagDetection() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }
    public void telemetryAprilTag(Telemetry telemetry) {

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

    }   // end method telemetryAprilTag()
}

