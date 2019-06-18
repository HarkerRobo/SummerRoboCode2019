package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.MathUtil;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Moves the Drivetrain using Velocity PID
 */
public class DriveWithVelocity extends Command
{
    private static final double SPEED_MULTIPLIER = 0.2;

    private static final double MAX_FORWARD_VELOCITY = 14;
    private static final double MAX_TURN_VELOCITY = 8;

    public DriveWithVelocity() {
        requires(Drivetrain.getInstance());
    }

    protected void initialize() {
        Drivetrain.getInstance().setupVelocityPID();
    }

    @Override
    protected void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND) * MAX_FORWARD_VELOCITY * SPEED_MULTIPLIER;
        double turn = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND) * MAX_TURN_VELOCITY * SPEED_MULTIPLIER;

        speed = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, speed, SpeedUnit.ENCODER_UNITS);
        turn = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, turn, SpeedUnit.ENCODER_UNITS);

        SmartDashboard.putNumber("PID Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
        SmartDashboard.putNumber("PID Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
        Drivetrain.getInstance().getLeftMaster().set(ControlMode.Velocity, speed + turn);
        Drivetrain.getInstance().getRightMaster().set(ControlMode.Velocity, speed - turn);
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