package frc.robot.commands.arm;

import frc.robot.OI;
import frc.robot.subsystems.Arm;
import harkerrobolib.commands.IndefiniteCommand;

/**
 * Spins the BallIntake (Arm Rollers) using input from the Driver Gamepad's Triggers
 * 
 * @author Jatin Kohli
 * 
 * @since 6/17/19
 */
public class SpinBallIntakeManual extends IndefiniteCommand {

    private static final double SPARK_SPEED_MULTIPLIER = 0.15;

    public SpinBallIntakeManual() {
        requires(Arm.getInstance());
    }

    @Override
    protected void execute() {
        // double leftTrigger = OI.getInstance().getDriverGamepad().getLeftTrigger();
        // double rightTrigger = OI.getInstance().getDriverGamepad().getRightTrigger();
        // leftTrigger = MathUtil.mapJoystickOutput(-leftTrigger, OI.XBOX_TRIGGER_DEADBAND);
        // rightTrigger = MathUtil.mapJoystickOutput(rightTrigger, OI.XBOX_TRIGGER_DEADBAND);
        // double output = SPARK_SPEED_MULTIPLIER*(Math.abs(leftTrigger) > Math.abs(rightTrigger) ? leftTrigger : rightTrigger);

        boolean a = OI.getInstance().getDriverGamepad().getButtonAState();
        boolean x = OI.getInstance().getDriverGamepad().getButtonXState();
        double output = (a ? SPARK_SPEED_MULTIPLIER : (x ? -SPARK_SPEED_MULTIPLIER : 0));
        
        Arm.getInstance().getSpark().set(output);
    }
}