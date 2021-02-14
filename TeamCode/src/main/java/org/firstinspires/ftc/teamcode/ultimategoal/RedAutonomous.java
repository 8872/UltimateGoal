package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAutonomous", group = "drive")
public class RedAutonomous extends UltimateGoalAutonomous {
    @Override
    protected int yMult() {
        return -1;
    }
}
