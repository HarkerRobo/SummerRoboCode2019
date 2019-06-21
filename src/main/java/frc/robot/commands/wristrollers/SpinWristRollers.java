package frc.robot.commands.wristrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.WristRollers;

public class SpinWristRollers extends Command {
    
    private static final double SPEED_MULTIPLIER = 0.5;

    public SpinWristRollers() {
        requires(WristRollers.getInstance());
    }

    @Override
    protected void execute() {
        double leftTrigger = -OI.getInstance().getDriverGamepad().getLeftTrigger();
        double rightTrigger = OI.getInstance().getDriverGamepad().getRightTrigger();

        double output = Math.abs(leftTrigger) > rightTrigger ? leftTrigger : rightTrigger;

        WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, output * SPEED_MULTIPLIER);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}