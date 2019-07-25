package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
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
public class DriveWithVelocity extends IndefiniteCommand {
    private static final double SPEED_MULTIPLIER = 0.2;

    public DriveWithVelocity() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        Drivetrain.getInstance().applyToMasters((talon) -> talon.selectProfileSlot(Drivetrain.VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.configClosedloopRamp(Drivetrain.VELOCITY_RAMP_RATE));
        System.out.println("DriveWithVelocity Initialized");
    }

    @Override
    protected void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(),
                OI.XBOX_JOYSTICK_DEADBAND) * Drivetrain.MAX_FORWARD_VELOCITY * SPEED_MULTIPLIER;
        double turn = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(),
                OI.XBOX_JOYSTICK_DEADBAND) * Drivetrain.MAX_TURN_VELOCITY * SPEED_MULTIPLIER;

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Turn", turn);

        speed = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, speed, SpeedUnit.ENCODER_UNITS);
        turn = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, turn, SpeedUnit.ENCODER_UNITS);

        Drivetrain.getInstance().getLeftMaster().set(ControlMode.Velocity, speed + turn);
        Drivetrain.getInstance().getRightMaster().set(ControlMode.Velocity, speed - turn);
    }

    @Override
    protected void interrupted() {
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
    }
}