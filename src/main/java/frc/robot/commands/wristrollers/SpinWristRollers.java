package frc.robot.commands.wristrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.subsystems.WristRollers;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SpinWristRollers extends IndefiniteCommand {
    
    private static final double SPEED_MULTIPLIER = 0.5;

    public SpinWristRollers() {
        requires(WristRollers.getInstance());
    }

    @Override
    protected void execute() {
        double leftTrigger = MathUtil.mapJoystickOutput(-OI.getInstance().getDriverGamepad().getLeftTrigger(), OI.XBOX_TRIGGER_DEADBAND);
        double rightTrigger = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightTrigger(), OI.XBOX_TRIGGER_DEADBAND);

        double output = Math.abs(leftTrigger) > rightTrigger ? leftTrigger : rightTrigger;

        WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, output * SPEED_MULTIPLIER);
    }
}