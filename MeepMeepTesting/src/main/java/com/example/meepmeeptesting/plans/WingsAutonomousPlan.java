package com.example.meepmeeptesting.plans;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.meepmeeptesting.util.BotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class WingsAutonomousPlan {
    public static void Run(MeepMeep meepMeep) {
        RoadRunnerBotEntity bot = BotBuilder.Build(meepMeep, new ColorSchemeRedDark());

        // Wings of Steel Autonomous Drive Plan -- Start --
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineTo(new Vector2d(24, 0), Math.toRadians(0))
                .build());

        // Wings of Steel Autonomous Drive Plan -- End --
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
