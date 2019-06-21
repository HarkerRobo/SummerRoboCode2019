package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.PositionUnit;

/**
 * Follows the Specified Left and Right Trajectories by closed looping to each position and adding the necessary Arbitrary FeedForward
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since June 19, 2019
 */
public class DriveWithMotionProfile extends IndefiniteCommand {

    private Notifier notifier;
    private double timeDur;
    private int index;

    public DriveWithMotionProfile(double[][] leftPath, double[][] rightPath, double timeDur) {
        requires(Drivetrain.getInstance());

        this.timeDur = timeDur;
        
        index = 0;
        notifier = new Notifier(() -> {
            if (index < leftPath.length) {
                double leftPosition = Conversions.convertPosition(PositionUnit.FEET, leftPath[index][0], PositionUnit.ENCODER_UNITS);
                double rightPosition = Conversions.convertPosition(PositionUnit.FEET, rightPath[index][0], PositionUnit.ENCODER_UNITS);

                double leftVelocity = leftPath[index][1];
                double rightVelocity = rightPath[index][1];

                double leftAcceleration = leftPath[index][2];
                double rightAcceleration = rightPath[index][2];

                double leftArbitraryFeedForward = Drivetrain.kS + Drivetrain.kF * leftVelocity + Drivetrain.kA * leftAcceleration;
                double rightArbitraryFeedForward = Drivetrain.kS + Drivetrain.kF * rightVelocity + Drivetrain.kA * rightAcceleration;

                Drivetrain.getInstance().getLeftMaster().set(ControlMode.Position, leftPosition, DemandType.ArbitraryFeedForward, leftArbitraryFeedForward);
                Drivetrain.getInstance().getRightMaster().set(ControlMode.Position, rightPosition, DemandType.ArbitraryFeedForward, rightArbitraryFeedForward);

                index++;
            }
        });
    }

    @Override
    protected void initialize() {
        Drivetrain.getInstance().setupMotionProfilePID();

        notifier.startPeriodic(timeDur);
    }

    public void execute() {
        SmartDashboard.putNumber("Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
        SmartDashboard.putNumber("Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
        
        SmartDashboard.putNumber("Left", Drivetrain.getInstance().getLeftMaster().getSelectedSensorPosition());
        SmartDashboard.putNumber("Right", Drivetrain.getInstance().getRightMaster().getSelectedSensorPosition());
    }

    @Override
    protected void end() {
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
        notifier.close();
    }

    @Override
    protected void interrupted() {
        end();
    }             
}