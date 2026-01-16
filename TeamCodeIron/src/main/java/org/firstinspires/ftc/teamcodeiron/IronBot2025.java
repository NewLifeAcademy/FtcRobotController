package org.firstinspires.ftc.teamcodeiron;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * FTC 17240 Iron Eagles - REV Robotics Chassis
 */
public class IronBot2025 extends MecanumDrive {

    public static int SPINARIZER_INCREMENT = 96; // Encoder counts per spinarizer increment step
    public static double SPINARIZER_VELOCITY = 50.0; // Velocity for spinarizer motor

    private HardwareMap hardwareMap;
    // AprilTag detection members
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private DcMotor flywheelLeft;
    private DcMotor flywheelRight;
    private DcMotorEx spinarizer;
    private CRServo intake;
    private CRServo underlift;
    private Servo pusher;
    private DistanceSensor rangeSensor;

    public IronBot2025(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        // Save the hardware map reference
        this.hardwareMap = hardwareMap;

        // Initialize motors and servos
        this.flywheelLeft = hardwareMap.get(DcMotor.class, "leftlauncher");
        this.flywheelRight = hardwareMap.get(DcMotor.class, "rightlauncher");

        // reverse right flywheel direction
        this.flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        this.spinarizer = hardwareMap.get(DcMotorEx.class, "spinarizer");
        this.intake = hardwareMap.get(CRServo.class, "intake");
        this.underlift = hardwareMap.get(CRServo.class, "underlift");
        this.pusher = hardwareMap.get(Servo.class, "pusher");

        this.rangeSensor = hardwareMap.get(DistanceSensor.class, "sensor");
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

    public void firePreloadedArtifacts(Telemetry telemetry, double flywheelPower, int flywheelSpinupTimeSeconds, int waitBetweenShots) {
        telemetry.addLine("Firing first preloaded artifact");
        telemetry.update();
        fireArtifact(flywheelPower, flywheelSpinupTimeSeconds);
        startIntake();
        incrementSpinarizer();
        wait(waitBetweenShots); // wait 3 seconds between shots

        telemetry.addLine("Firing second preloaded artifact");
        telemetry.update();
        fireArtifact(flywheelPower, flywheelSpinupTimeSeconds);
        incrementSpinarizer();
        wait(waitBetweenShots);

        telemetry.addLine("Firing third preloaded artifact");
        telemetry.update();
        stopIntake();
        fireArtifact(flywheelPower, flywheelSpinupTimeSeconds);

        // stop autonomous and wait for finish
        telemetry.addLine("Testing complete. Stopping autonomous.");
        telemetry.update();
    }

    public void fireArtifact(double flywheelPower, int flywheelSpinupTimeSeconds) {
        // If both flywheels appear stopped, start them and wait for spin-up
        if (Math.abs(flywheelLeft.getPower()) < 1e-3 && Math.abs(flywheelRight.getPower()) < 1e-3) {
            startFlywheels(flywheelPower);
            wait(flywheelSpinupTimeSeconds);
        }
        // Trigger the pusher to fire one artifact
        triggerPusher();
    }

    public void startIntake() {
        intake.setPower(-1.0);
        underlift.setPower(1.0);
    }

    public void stopIntake() {
        intake.setPower(0.0);
        underlift.setPower(0.0);
    }

    public void incrementSpinarizer() {
        int targetPosition = spinarizer.getCurrentPosition() + SPINARIZER_INCREMENT;
        spinarizer.setTargetPosition(targetPosition);
        spinarizer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinarizer.setVelocity(SPINARIZER_VELOCITY);
        while (spinarizer.isBusy()) {
            // Wait until the motor reaches the target position
        }
    }

    public void triggerPusher() {
        pusher.setPosition(1.0);
        wait(2);
        pusher.setPosition(0.5);
        wait(1);
    }

    public void startFlywheels(double flywheelPower) {
        flywheelLeft.setPower(flywheelPower);
        flywheelRight.setPower(flywheelPower);
    }

    public void stopFlywheels() {
        flywheelLeft.setPower(0.0);
        flywheelRight.setPower(0.0);
    }

    public void getSensorDistance(Telemetry telemetry) {
        double distance = rangeSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Range Sensor Distance (cm): ", distance);
    }

    public void initAprilTagDetection() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }
    public void telemetryAprilTag(Telemetry telemetry) {

        if (aprilTagProcessor == null) {
            telemetry.addLine("Initialize apriltag processor.");
            this.initAprilTagDetection();
        }

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

