package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Arm;

public class SpinBallIntake extends Command {

    public SpinBallIntake() {
        requires(Arm.getInstance());
    }

    @Override
    protected void execute() {
        double leftTrigger = -OI.getInstance().getDriverGamepad().getLeftTrigger();
        double rightTrigger = OI.getInstance().getDriverGamepad().getRightTrigger();

        double output = Math.abs(leftTrigger) > rightTrigger ? leftTrigger : rightTrigger;

        Arm.getInstance().getRollers().set(ControlMode.PercentOutput, output);

        SmartDashboard.putNumber("Arm Roller", Arm.getInstance().getRollers().getMotorOutputPercent());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}