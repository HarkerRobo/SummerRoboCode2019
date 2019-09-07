package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.OI.DemoMode;
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
        double leftTrigger = ((OI.mode == DemoMode.NORMAL) ? OI.getInstance().getDriverGamepad().getLeftTrigger() : OI.getInstance().getOperatorGamepad().getLeftTrigger());
        double rightTrigger = ((OI.mode == DemoMode.NORMAL) ? OI.getInstance().getDriverGamepad().getRightTrigger() : OI.getInstance().getOperatorGamepad().getRightTrigger());
        leftTrigger = MathUtil.mapJoystickOutput(-leftTrigger, OI.XBOX_TRIGGER_DEADBAND);
        rightTrigger = MathUtil.mapJoystickOutput(rightTrigger, OI.XBOX_TRIGGER_DEADBAND);

        double output = Math.abs(leftTrigger) > Math.abs(rightTrigger) ? leftTrigger : rightTrigger;

        //Arm.getInstance().getRollers().set(ControlMode.PercentOutput, SPEED_MULTIPLIER * output);
        Arm.getInstance().getRollers().set(SPEED_MULTIPLIER * output);
    }
}