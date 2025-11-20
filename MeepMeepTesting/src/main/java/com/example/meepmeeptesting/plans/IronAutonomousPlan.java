package com.example.meepmeeptesting.plans;

import com.acmerobotics.roadrunner.Pose2d;
import com.example.meepmeeptesting.util.BotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class IronAutonomousPlan {
    public static void Run(MeepMeep meepMeep) {
        RoadRunnerBotEntity bot = BotBuilder.Build(meepMeep, new ColorSchemeBlueDark());

        // Iron Eagles Autonomous Drive Plan -- Start --
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(-60, -24, Math.toRadians(90)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-11.5, 0, Math.toRadians(90)), Math.toRadians(90))
                .build());

        // Iron Eagles Autonomous Drive Plan -- End --
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
