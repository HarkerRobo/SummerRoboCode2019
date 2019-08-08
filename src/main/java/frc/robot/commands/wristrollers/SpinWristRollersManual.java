package frc.robot.commands.wristrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import frc.robot.OI;
import frc.robot.OI.DemoMode;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristRollers;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Spins the Wrist Rollers using input from the Driver Gamepad's Triggers
 * 
 * @author Jatin Kohli
 * 
 * @since 6/17/19
 */
public class SpinWristRollersManual extends IndefiniteCommand {
    
    private static final double SPEED_MULTIPLIER = 0.5;

    public SpinWristRollersManual() {
        requires(WristRollers.getInstance());
    }

    @Override
    protected void execute() {
        double leftTrigger = ((OI.mode == DemoMode.NORMAL) ? OI.getInstance().getDriverGamepad().getLeftTrigger() : OI.getInstance().getOperatorGamepad().getLeftTrigger());
        double rightTrigger = ((OI.mode == DemoMode.NORMAL) ? OI.getInstance().getDriverGamepad().getRightTrigger() : OI.getInstance().getOperatorGamepad().getRightTrigger());
        leftTrigger = MathUtil.mapJoystickOutput(-leftTrigger, OI.XBOX_TRIGGER_DEADBAND);
        rightTrigger = MathUtil.mapJoystickOutput(rightTrigger, OI.XBOX_TRIGGER_DEADBAND);

        double output = Math.abs(leftTrigger) > Math.abs(rightTrigger) ? leftTrigger : rightTrigger;

        WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, output * SPEED_MULTIPLIER, DemandType.ArbitraryFeedForward, Wrist.CARGO_FF);
    }
}