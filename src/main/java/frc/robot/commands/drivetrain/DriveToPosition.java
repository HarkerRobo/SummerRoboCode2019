package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.PositionUnit;

/**
 * Moves the Drivetrain using Position PID
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/15/19
 */
public class DriveToPosition extends Command
{
    private static final int ALLOWABLE_ERROR = 100;

    private double distance;

    public DriveToPosition(double feet) {
        requires(Drivetrain.getInstance());

        distance = Conversions.convertPosition(PositionUnit.FEET, feet, PositionUnit.ENCODER_UNITS);
    }

    protected void initialize() {
        Drivetrain.getInstance().setupPositionPID();

        Drivetrain.getInstance().applyToMasters((talon) -> talon.set(ControlMode.Position, distance));
    }

    @Override
    public boolean isFinished() {
        return Drivetrain.getInstance().isClosedLoopErrorWithin(RobotMap.PRIMARY_PID_INDEX, ALLOWABLE_ERROR);
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