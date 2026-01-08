package org.firstinspires.ftc.teamcodewings;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * FTC 32014 Wings of Steel - GoBilda Chassis
 */
public class WingsBot2025 extends MecanumDrive {

    private HardwareMap hardwareMap;
    // AprilTag detection members
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private DcMotor spitterLeft; // motor0 (REV Robotics 20:1 HD Hex Motor) - expansion hub
    private DcMotor putter;
    private DcMotor ballPuter; // motor1 (REV Robotics 20:1 HD Hex Motor)- expansion hub
    private DcMotor intakeBeltSpiner; // motor3 (REV Robotics Core Hex Motor) - expansion hub

    private Servo ballPutterServo; // servo0 (Standard Servo) - expansion hub

    public WingsBot2025(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        // Save the hardware map reference
        this.hardwareMap = hardwareMap;

        this.spitterLeft = hardwareMap.get(DcMotor.class, "spitter left");
        this.putter = hardwareMap.get(DcMotor.class,"putter");
        this.ballPuter = hardwareMap.get(DcMotor.class, "ball puter");
        this.intakeBeltSpiner = hardwareMap.get(DcMotor.class, "intake belt spiner");
        this.ballPutterServo = hardwareMap.get(Servo.class, "ball putter servo");
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
        // Activate spitterLeft, intakeBeltSpiner, and ballPuter to fire artifacts
        spitterLeft.setPower(flywheelPower);
        wait(flywheelSpinupSeconds);
        ballPutterServo.setPosition(1);
        wait(1);
        intakeBeltSpiner.setPower(1.0);
        ballPuter.setPower(1.0);
        putter.setPower(-0.3);

        wait(fireDurationSeconds - flywheelSpinupSeconds);


        // Deactivate spitterLeft, intakeBeltSpiner, and ballPuter
        spitterLeft.setPower(0);
        intakeBeltSpiner.setPower(0);
        ballPuter.setPower(0);
        putter.setPower(0);

        ballPutterServo.setPosition(0);
    }

    public void startIntake() {
        // intakeBeltSpiner, and ballPuter to fire artifacts
        intakeBeltSpiner.setPower(1.0);
        ballPuter.setPower(1.0);
    }

    public void stopIntake() {
        // intakeBeltSpiner, and ballPuter to fire artifacts
        intakeBeltSpiner.setPower(0.0);
        ballPuter.setPower(0.0);
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
