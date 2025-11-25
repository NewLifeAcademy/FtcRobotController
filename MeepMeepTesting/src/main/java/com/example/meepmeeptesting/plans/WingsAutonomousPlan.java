package com.example.meepmeeptesting.plans;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.meepmeeptesting.util.BotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class WingsAutonomousPlan {
    public static void Run(MeepMeep meepMeep) {
        RoadRunnerBotEntity entity = BotBuilder.Build(meepMeep, new ColorSchemeRedDark());
        DriveShim robot = entity.getDrive();

        /* Wings of Steel Autonomous Drive Plan
        * To use:
        * 1. Create your Starting Pose
        * 2. Build your action sequence using waypoint functions:
        *   Simple Spline to a point:
        *       .splineTo(new Vector2d( x , y ), Math.toRadians( heading ))
        *
        *   Spline to a point with constant heading:
        *       .splineToConstantHeading(new Vector2d( x , y ), Math.toRadians( heading ))
        *
        *   Spline to a point with an entry tangent and linear heading:
        *       .setTangent(Math.toRadians( tangentAngle ))
        *       .splineToLinearHeading(new Pose2d( x , y , Math.toRadians( heading )), Math.toRadians( heading ))
        *
        *   Spline to a point with an entry tangent and spline heading:
        *       .setTangent(Math.toRadians( tangentAngle ))
        *       .splineToSplineHeading(new Pose2d( x , y , Math.toRadians( heading )), Math.toRadians( heading ))
        *
        *  3. Run the MeepMeep simulation to visualize the path.  Correct any errors before implementing on the robot.
        *  4. Copy the "Starting Pose" and "Build action sequence" code into the autonomous OpMode for the robot.
        *
        *  For examples of each of the waypoint functions, see AutoFirstWayPointTesting in the MeepMeepTesting project
         */

        // Starting Pose
        Pose2d startPose = new Pose2d(-44, 44, Math.toRadians(315));

        // Build action sequence
        Action action = robot.actionBuilder(startPose)

                // Fire three
                .splineTo(new Vector2d( -12 , 12 ), Math.toRadians( 315 ))
                .waitSeconds(3)
                .splineTo(new Vector2d( -11 , 30 ), Math.toRadians( 90 ))
                // intake PPG
                .lineToY(39)
                .waitSeconds(1)
                .splineTo(new Vector2d( -12 , 12 ), Math.toRadians( 315 ))
                .build();

        // End action sequence

        // Run the action sequence (MeepMeep only)
        entity.runAction(action);

        // Wings of Steel Autonomous Drive Plan -- End --
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(entity)
                .start();
    }
}
