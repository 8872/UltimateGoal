package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutonomous", group = "drive")
public class BlueAutonomous extends UltimateGoalAutonomous {
    @Override
    protected int yMult() {
        return 1;
    }
}
