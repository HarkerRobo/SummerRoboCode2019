package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * Moves the Drivetrain using the PercentOutput ControlMode
 * 
 * @author Finn Frankis
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @since 6/14/19
 */
public class DriveWithPercentOutput extends Command {
    private double leftOutput;
    private double rightOutput;

    public DriveWithPercentOutput(double leftOutput, double rightOutput) {
        requires(Drivetrain.getInstance());

        this.leftOutput = leftOutput;
        this.rightOutput = rightOutput;
    }

    public void execute() {
        Drivetrain.getInstance().getLeftMaster().set(ControlMode.PercentOutput, leftOutput);
        Drivetrain.getInstance().getRightFollower().set(ControlMode.PercentOutput, rightOutput);
    }    
    
    public void end() {
        Drivetrain.getInstance().setBoth(ControlMode.DISABLED, 0);
    }

    public boolean isFinished() {
        return false;
    }
}