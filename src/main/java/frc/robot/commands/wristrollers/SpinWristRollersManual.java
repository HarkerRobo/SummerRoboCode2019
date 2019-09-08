package frc.robot.commands.wristrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import frc.robot.OI;
import frc.robot.RobotMap;
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
        double leftTrigger = OI.getInstance().getDriverGamepad().getLeftTrigger();
        double rightTrigger = OI.getInstance().getDriverGamepad().getRightTrigger();
        leftTrigger = MathUtil.mapJoystickOutput(leftTrigger, OI.XBOX_TRIGGER_DEADBAND);
        rightTrigger = MathUtil.mapJoystickOutput(-rightTrigger, OI.XBOX_TRIGGER_DEADBAND);

        boolean a = OI.getInstance().getDriverGamepad().getButtonAState();
        boolean x = OI.getInstance().getDriverGamepad().getButtonXState();
        double output;

        if (RobotMap.SUMMER_BOT) {
            output = Math.abs(leftTrigger) > Math.abs(rightTrigger) ? leftTrigger : rightTrigger;
        } else {
            output = SPEED_MULTIPLIER * (a ? 1 : (x ? -1 : 0));
        }

        WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, WristRollers.CARGO_FF);
    }
}