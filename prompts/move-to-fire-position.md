We have tested out robot at several points on the field, and captured the data in lines 116-143 in SpainBotMoveToFireSpotWithCamera. In SpainBotAutoWaypointTest we have written code that creates a SequentialAction that will move the robot to a specific position and heading on the field. In SpainBotMoveToFireSpotWithCamera at lines 96-102 we already have code that will look for an AprilTag when the y button is pressed. With this information, please update code in SpainBotMoveToFireSpotWithCamera to check if an AprilTag is detected, and if so, move the robot to the Target position (x:0, y:0, heading: 135).  I would expect the code will need to use the values from the detected AprilTag and calculate the distance that the robot would need to move based on the data that we captured in lines 116-143.


---
Result
Added an AprilTag-driven pose estimate that maps the recorded tag readings to field offsets, updates the localizer, and drives a RoadRunner action back to the (0,0,135°) fire spot, plus a helper in SpainBot2025 to access the latest detection.


TeamCodeSpain/src/main/java/org/firstinspires/ftc/teamcodespain/teleop/SpainBotMoveToFireSpotWithCamera.java
```
package org.firstinspires.ftc.teamcodespain.teleop;

import static org.firstinspires.ftc.teamcode.SpainBotBlockCompanion.robotMotorPower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcodespain.SpainBot2025;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Camera Firing Test")
@Config
public class SpainBotMoveToFireSpotWithCamera extends LinearOpMode {

    private static final Pose2d TARGET_POSE = new Pose2d(0, 0, Math.toRadians(135));
    private static final double TARGET_TAG_X = -6.4;
    private static final double TARGET_TAG_Y = 78.5;
    private static final double TARGET_TAG_BEARING = 4.7;
    private static final double DX_TO_FIELD_X = -1.342;
    private static final double DY_TO_FIELD_X = -0.955;
    private static final double DX_TO_FIELD_Y = 1.342;
    private static final double DY_TO_FIELD_Y = 0.955;
    private static final double POSITION_TOLERANCE_IN = 0.5;
    private static final double HEADING_TOLERANCE_DEG = 1.5;

    @Override
    public void runOpMode() {
        SpainBot2025 robot = new SpainBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.initAprilTagDetection();
        waitForStart();

        if (opModeIsActive()) {

            boolean yPressedLast = false;

            while (opModeIsActive()) {

                robot.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                if (gamepad1.dpad_down) {
                    robot.visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    robot.visionPortal.resumeStreaming();
                }

                boolean yPressed = gamepad1.y;

                if (yPressed && !yPressedLast) {
                    telemetry.addLine("Running AprilTag detection...");
                    robot.telemetryAprilTag(telemetry);

                    AprilTagDetection detection = robot.getPrimaryAprilTagDetection();
                    if (detection != null) {
                        TagPoseEstimate estimate = estimatePoseFromDetection(detection);
                        telemetry.addData("Field offset (in)", "X: %.1f, Y: %.1f",
                                estimate.offsetFromTarget.x, estimate.offsetFromTarget.y);
                        telemetry.addData("Heading delta (deg)", "%.1f", estimate.headingErrorDeg);
                        double distance = estimate.offsetFromTarget.norm();

                        if (distance > POSITION_TOLERANCE_IN || Math.abs(estimate.headingErrorDeg) > HEADING_TOLERANCE_DEG) {
                            moveRobotToTarget(robot, estimate.pose);
                            telemetry.addData("Move commanded", "ΔX: %.1f, ΔY: %.1f (%.1f in)",
                                    -estimate.offsetFromTarget.x, -estimate.offsetFromTarget.y, distance);
                        } else {
                            telemetry.addLine("Already within tolerance of target.");
                        }
                    } else {
                        telemetry.addLine("No AprilTag detected; cannot move to target.");
                    }

                    telemetry.addLine("Detection complete. Press Y to run again.");
                } else if (!yPressed) {
                    telemetry.addLine("Press Y to run AprilTag detection.");
                }

                telemetry.update();
                yPressedLast = yPressed;
                sleep(50);
            }
        }
    }

    private TagPoseEstimate estimatePoseFromDetection(AprilTagDetection detection) {
        double dx = detection.ftcPose.x - TARGET_TAG_X;
        double dy = detection.ftcPose.y - TARGET_TAG_Y;

        double offsetX = DX_TO_FIELD_X * dx + DY_TO_FIELD_X * dy;
        double offsetY = DX_TO_FIELD_Y * dx + DY_TO_FIELD_Y * dy;
        double bearingDelta = detection.ftcPose.bearing - TARGET_TAG_BEARING;

        Pose2d pose = new Pose2d(
                TARGET_POSE.x + offsetX,
                TARGET_POSE.y + offsetY,
                TARGET_POSE.heading - Math.toRadians(bearingDelta)
        );

        return new TagPoseEstimate(pose, new Vector2d(offsetX, offsetY), bearingDelta);
    }

    private void moveRobotToTarget(SpainBot2025 robot, Pose2d estimatedPose) {
        robot.localizer.setPose(estimatedPose);
        Action moveAction = robot.actionBuilder(estimatedPose)
                .splineToLinearHeading(TARGET_POSE, TARGET_POSE.heading)
                .build();
        Actions.runBlocking(new SequentialAction(moveAction));
        robot.stopMotors();
    }

    private static class TagPoseEstimate {
        final Pose2d pose;
        final Vector2d offsetFromTarget;
        final double headingErrorDeg;

        TagPoseEstimate(Pose2d pose, Vector2d offsetFromTarget, double headingErrorDeg) {
            this.pose = pose;
            this.offsetFromTarget = offsetFromTarget;
            this.headingErrorDeg = headingErrorDeg;
        }
    }
}
```

TeamCodeSpain/src/main/java/org/firstinspires/ftc/teamcodespain/SpainBot2025.java

```
    public void telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        ...
    }

    public AprilTagDetection getPrimaryAprilTagDetection() {
        if (aprilTagProcessor == null) {
            return null;
        }
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections.isEmpty() ? null : detections.get(0);
    }
```
