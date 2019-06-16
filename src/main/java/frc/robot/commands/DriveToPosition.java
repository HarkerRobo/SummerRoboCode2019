package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.PositionUnit;

/**
 * Moves the Drivetrain using Position PID
 */
public class DriveToPosition extends Command
{
    private double distance;

    public DriveToPosition(double feet) {
        requires(Drivetrain.getInstance());

        distance = Conversions.convertPosition(PositionUnit.FEET, feet, PositionUnit.ENCODER_UNITS);
    }

    protected void initialize() {
        Drivetrain.getInstance().setupPositionPID();
    }

    @Override
    protected void execute() {
        SmartDashboard.putNumber("PID Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
        SmartDashboard.putNumber("PID Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
        Drivetrain.getInstance().getLeftMaster().set(ControlMode.Position, -distance);
        Drivetrain.getInstance().getRightMaster().set(ControlMode.Position, distance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end() {
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}