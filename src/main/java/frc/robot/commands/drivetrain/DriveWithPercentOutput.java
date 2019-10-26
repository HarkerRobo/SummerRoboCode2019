package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Moves the Drivetrain using the PercentOutput ControlMode
 * 
 * @author Finn Frankis
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/14/19
 */
public class DriveWithPercentOutput extends IndefiniteCommand {
    private static final double SPEED_MULTIPLIER = 0.2;

    public DriveWithPercentOutput() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        System.out.println("DriveWithPercentOutput Initialized");
    }

    @Override
    public void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND) * SPEED_MULTIPLIER;
        //double turn = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND) * SPEED_MULTIPLIER;
        double turn = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND) * SPEED_MULTIPLIER;

        //Drivetrain.getInstance().arcadeDrivePercentOutput(speed, turn);
        Drivetrain.getInstance().getLeftMaster().set(ControlMode.PercentOutput, speed - turn);
        Drivetrain.getInstance().getRightMaster().set(ControlMode.PercentOutput, speed + turn);
    }

    @Override
    protected void interrupted() {
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
    }
}