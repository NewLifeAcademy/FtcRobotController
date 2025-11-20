package com.example.meepmeeptesting;

import com.example.meepmeeptesting.plans.AutoFirstWayPointTesting;
import com.example.meepmeeptesting.plans.AutoWayPointPlan;
import com.example.meepmeeptesting.plans.IronAutonomousPlan;
import com.example.meepmeeptesting.plans.WingsAutonomousPlan;
import com.noahbres.meepmeep.MeepMeep;

public class MeepMeepTesting {
    public static void main(String[] args) {
//        MeepMeep meepMeep1 = new MeepMeep(800);
//        AutoFirstWayPointTesting.Run(meepMeep1);
//
//        MeepMeep meepMeep2 = new MeepMeep(800);
//        AutoWayPointPlan.Run(meepMeep2);
//
        MeepMeep meepMeepIron = new MeepMeep(800);
        IronAutonomousPlan.Run(meepMeepIron);

        MeepMeep meepMeepWings = new MeepMeep(800);
        WingsAutonomousPlan.Run(meepMeepWings);
    }
}