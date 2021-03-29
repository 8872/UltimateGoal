package org.firstinspires.ftc.teamcode.ultimategoal;

public class IntakePowerSetter extends UltimateGoalOpMode {

    private static double intakePower = 0;

    boolean lastAState = false;
    boolean lastBState = false;
    boolean lastXState = false;
    
    @Override
    public void loop() {
        if (!lastAState && gamepad1.a) {
            intakePower += 0.25;
        }
        if (!lastBState && gamepad1.b) {
            intakePower -= 0.25;
        }
        if (!lastXState && gamepad1.x) {
            intakePower = 0;
        }
        intake.setPower(intakePower);
        
        lastAState = gamepad1.a;
        lastBState = gamepad1.b;
    }

    @Override
    protected void composeTelemetry() {
        super.composeTelemetry();
        telemetry.addData("Intake", () -> intake.getPower());
    }
}
