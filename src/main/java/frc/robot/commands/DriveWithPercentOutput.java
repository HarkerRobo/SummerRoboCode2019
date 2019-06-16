package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.MathUtil;

/**
 * Moves the Drivetrain using the PercentOutput ControlMode
 * 
 * @author Finn Frankis
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @since 6/14/19
 */
public class DriveWithPercentOutput extends Command {
    private static final double SPEED_MULTIPLIER = 0.2;

    public DriveWithPercentOutput() {
        requires(Drivetrain.getInstance());
    }

    @Override
    public void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND) * SPEED_MULTIPLIER;
        double turn = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.XBOX_JOYSTICK_DEADBAND) * SPEED_MULTIPLIER;
        Drivetrain.getInstance().getLeftMaster().set(ControlMode.PercentOutput, speed + turn);
        Drivetrain.getInstance().getRightFollower().set(ControlMode.PercentOutput, speed - turn);
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