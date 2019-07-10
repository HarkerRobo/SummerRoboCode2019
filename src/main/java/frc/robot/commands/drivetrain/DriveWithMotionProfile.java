package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * Follows the Specified Left and Right Trajectories by closed looping to each position and adding the necessary Arbitrary FeedForward
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/19/19
 */
public class DriveWithMotionProfile extends Command {
     
    public DriveWithMotionProfile() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        end();
    }
}