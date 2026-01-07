package org.firstinspires.ftc.teamcodewings.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodewings.WingsBot2025;

@Autonomous(name = "Auto - Start on Blue Goal", preselectTeleOp = "B-bot original code")
@Config
public class WingsStartOnBlueGoal extends LinearOpMode {

    public static int FIRE_TIME = 5;
    public static int FLYWHEEL_SPINUP_TIME = 2;
    public static double FLYWHEEL_POWER = 0.85;
    public static double START_POSE_X = -48;
    public static double START_POSE_Y = -48;
    public static double START_HEADING = 45;
    public static double WAYPOINT_FIRE_X = -16;
    public static double WAYPOINT_FIRE_Y = -16;
    public static double WAYPOINT_FIRE_HEADING = 315;
    public static double SPIKE_APPROACH_X = -9;
    public static double SPIKE_APPROACH_Y = -22;
    public static double SPIKE_APPROACH_HEADING = 270;
    public static double SPIKE_INTAKE_X = -11;
    public static double SPIKE_INTAKE_Y = -39;
    public static double SPIKE_INTAKE_HEADING = 270;
    public static double END_POSE_X = 17;
    public static double END_POSE_Y = -18;
    public static double END_HEADING = 270;

    @Override
    public void runOpMode() {
        WingsBot2025 robot = new WingsBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            // Starting Pose
            Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_HEADING));

            robot.localizer.setPose(startPose);

            // Move to near launch position
            Action action = robot.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d( WAYPOINT_FIRE_X , WAYPOINT_FIRE_Y, Math.toRadians( WAYPOINT_FIRE_HEADING ) ), Math.toRadians( WAYPOINT_FIRE_HEADING ))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // Fire three preloaded
            robot.fireArtifacts(FIRE_TIME, FLYWHEEL_SPINUP_TIME, FLYWHEEL_POWER);

            // Move to PPG spike approach position
            action = robot.actionBuilder(robot.localizer.getPose())
                    .splineToLinearHeading(new Pose2d( SPIKE_APPROACH_X , SPIKE_APPROACH_Y, Math.toRadians(SPIKE_APPROACH_HEADING) ), Math.toRadians( SPIKE_APPROACH_HEADING ))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // start the intake
            robot.startIntake();

            // intake PPG
            action = robot.actionBuilder(robot.localizer.getPose())
                    .lineToY(SPIKE_INTAKE_Y)
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // stop the intake
            robot.stopIntake();

            // Move to near launch position
            action = robot.actionBuilder(robot.localizer.getPose())
                    .splineToLinearHeading(new Pose2d( WAYPOINT_FIRE_X , WAYPOINT_FIRE_Y, Math.toRadians(WAYPOINT_FIRE_HEADING) ), Math.toRadians( WAYPOINT_FIRE_HEADING ))
                    .build();

            Actions.runBlocking(new SequentialAction(action));

            // Fire PPG
            robot.fireArtifacts(FIRE_TIME, FLYWHEEL_SPINUP_TIME, FLYWHEEL_POWER);

            // Move to PGP spike approach position
            action = robot.actionBuilder(robot.localizer.getPose())
                    .splineToLinearHeading(new Pose2d( END_POSE_X , END_POSE_Y, Math.toRadians(END_HEADING)), Math.toRadians( END_HEADING ))
                    .build();
            Actions.runBlocking(new SequentialAction(action));
            
            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
