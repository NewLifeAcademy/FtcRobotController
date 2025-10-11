package com.example.meepmeeptesting.plans;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.meepmeeptesting.util.BotBuilder;
import com.example.meepmeeptesting.util.Position;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoWayPointPlan {
    public static void Run(MeepMeep meepMeep) {
        RoadRunnerBotEntity bot = BotBuilder.Build(meepMeep, new ColorSchemeRedDark());

        Position firePosition = new Position(new Pose2d(-36, 32, Math.toRadians(135)), Math.toRadians(135));
        double fireTime = 1.5;

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(-60, 8, Math.toRadians(90)))
                // Move to fire position
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                // fire preloaded
                .waitSeconds(fireTime)
                // Move to spike 3 (PPG)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-11.5, 32, Math.toRadians(90)), Math.toRadians(90))
                // intake PPG
                .lineToY(44)
                // Move to fire position
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                // fire PPG
                .waitSeconds(fireTime)
                // Move to spike 2 (PGP)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(11.5, 32, Math.toRadians(90)), Math.toRadians(90))
                // intake PGP
                .lineToY(44)
                // Move to fire position
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                // fire PGP
                .waitSeconds(fireTime)
                // Move to spike 1 (GPP)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(34.5, 32, Math.toRadians(90)), Math.toRadians(90))
                // intake GPP
                .lineToY(44)
                // Move to fire position
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                // fire GPP
                .waitSeconds(fireTime)
                // Move to park
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-52, 8, Math.toRadians(0)), Math.toRadians(0))
                // end autonomous
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
