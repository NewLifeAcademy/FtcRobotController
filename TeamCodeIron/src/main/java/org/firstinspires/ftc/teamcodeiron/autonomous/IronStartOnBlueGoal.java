package org.firstinspires.ftc.teamcodeiron.autonomous;

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
import org.firstinspires.ftc.teamcodeiron.IronBot2025;


@Autonomous(name = "Auto - Start on Blue Goal")
@Config
public class IronStartOnBlueGoal extends LinearOpMode {

    public static int FIRE_TIME = 3;

    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            // Starting Pose
            Pose2d startPose = new Pose2d(-48, -48, Math.toRadians(45));

            robot.localizer.setPose(startPose);

            // Build action sequence
            Action action = robot.actionBuilder(startPose)
                    // Fire three
                    .splineTo(new Vector2d( -12 , -12 ), Math.toRadians( 45 ))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // Fire three preloaded
            //robot.fireArtifacts(FIRE_TIME);

            action = robot.actionBuilder(robot.localizer.getPose())
                    .splineTo(new Vector2d( -11 , -30 ), Math.toRadians(270 ))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // start the intake
            //robot.startIntake();

            // intake PPG
            action = robot.actionBuilder(robot.localizer.getPose())
                    .lineToY(-39)
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // stop the intake
            //robot.stopIntake();

            // move to launch position
            action = robot.actionBuilder(robot.localizer.getPose())
                    .splineTo(new Vector2d( -12 , -12 ), Math.toRadians( 45 ))
                    .build();

            Actions.runBlocking(new SequentialAction(action));

            // Fire PPG
            //robot.fireArtifacts(FIRE_TIME);

            // move off of launch line
            action = robot.actionBuilder(robot.localizer.getPose())
                    .splineTo(new Vector2d( 13 , -30 ), Math.toRadians( 270 ))
                    .build();
            Actions.runBlocking(new SequentialAction(action));
            
            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
