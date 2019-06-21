package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.MathUtil;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Moves the Drivetrain using Velocity PID
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/15/19
 */
public class DriveWithVelocity extends IndefiniteCommand
{
    private static final double SPEED_MULTIPLIER = 0.2;

    public DriveWithVelocity() {
        requires(Drivetrain.getInstance());
    }

    protected void initialize() {
        Drivetrain.getInstance().setupVelocityPID();
    }

    @Override
    protected void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND) * Drivetrain.MAX_FORWARD_VELOCITY * SPEED_MULTIPLIER;
        double turn = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND) * Drivetrain.MAX_TURN_VELOCITY * SPEED_MULTIPLIER;

        Drivetrain.getInstance().arcadeDriveVelocity(
                Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, speed, SpeedUnit.ENCODER_UNITS), 
                Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, turn, SpeedUnit.ENCODER_UNITS)
        );
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