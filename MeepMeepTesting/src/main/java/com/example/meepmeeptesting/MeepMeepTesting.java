package com.example.meepmeeptesting;

import com.example.meepmeeptesting.plans.AutoFirstWayPointTesting;
import com.example.meepmeeptesting.plans.AutoWayPointPlan;
import com.noahbres.meepmeep.MeepMeep;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        AutoFirstWayPointTesting.Run(meepMeep);

        MeepMeep meepMeep2 = new MeepMeep(800);
        AutoWayPointPlan.Run(meepMeep2);
    }
}