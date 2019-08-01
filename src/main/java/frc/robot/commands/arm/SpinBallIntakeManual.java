package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.subsystems.Arm;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Spins the BallIntake (Arm Rollers) using input from the Driver Gamepad's Triggers
 * 
 * @author Jatin Kohli
 * 
 * @since 6/17/19
 */
public class SpinBallIntakeManual extends IndefiniteCommand {

    private static final double SPEED_MULTIPLIER = 0.5;

    public SpinBallIntakeManual() {
        requires(Arm.getInstance());
    }

    @Override
    protected void execute() {
        double leftTrigger = MathUtil.mapJoystickOutput(-OI.getInstance().getDriverGamepad().getLeftTrigger(), OI.XBOX_TRIGGER_DEADBAND);
        double rightTrigger = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.XBOX_TRIGGER_DEADBAND);

        double output = Math.abs(leftTrigger) > rightTrigger ? leftTrigger : rightTrigger;

        Arm.getInstance().getRollers().set(ControlMode.PercentOutput, SPEED_MULTIPLIER * output);
    }
}