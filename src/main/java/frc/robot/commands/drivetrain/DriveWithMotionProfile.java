package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.PositionUnit;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Drives and tests the motion profile constants or smoething
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since June 19, 2019
 */
public class DriveWithMotionProfile extends Command {

    private static final int MIN_POINTS = 10; 

    private double[][] leftPath;
    private double[][] rightPath;

    private MotionProfileStatus status;

    private double timeDur;
    private int index;

    public DriveWithMotionProfile(double[][] leftPath, double[][] rightPath, double timeDur) {
        requires(Drivetrain.getInstance());

        this.leftPath = leftPath;
        this.rightPath = rightPath;

        this.timeDur = timeDur;
        index = 0;

        status = new MotionProfileStatus();
    }

    @Override
    protected void initialize() {
        System.out.println("DriveWithMotionProfile Initialized");
        Drivetrain.getInstance().setupMotionProfilePID();

        Notifier notifier = new Notifier(() -> {
            if (index < leftPath.length) {
                System.out.println(index);
                double leftPosition = leftPath[index][0];
                double rightPosition = rightPath[index][0];
                
                double leftVelocity = leftPath[index][1];
                double rightVelocity = rightPath[index][1];

                double leftAcceleration = leftPath[index][2];
                double rightAcceleration = rightPath[index][2];

                double leftArbitraryFeedForward = Drivetrain.kS + Drivetrain.kF * leftVelocity + Drivetrain.kA * leftAcceleration;
                double rightArbitraryFeedForward = Drivetrain.kS + Drivetrain.kF * rightVelocity + Drivetrain.kA * rightAcceleration;

                SmartDashboard.putNumber("Left Arb FF", leftArbitraryFeedForward);
                SmartDashboard.putNumber("Right Arb FF", rightArbitraryFeedForward);

                Drivetrain.getInstance().getLeftMaster().set(ControlMode.Position, 
                                            Conversions.convertPosition(PositionUnit.FEET, leftPosition, PositionUnit.ENCODER_UNITS), 
                                            DemandType.ArbitraryFeedForward, leftArbitraryFeedForward);
                Drivetrain.getInstance().getRightMaster().set(ControlMode.Position, 
                                            Conversions.convertPosition(PositionUnit.FEET, rightPosition, PositionUnit.ENCODER_UNITS), 
                                            DemandType.ArbitraryFeedForward, rightArbitraryFeedForward);

                index++;
            }
        });

        notifier.startPeriodic(timeDur);
    }

    public void execute() {
        SmartDashboard.putNumber("Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
        SmartDashboard.putNumber("Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
        
        SmartDashboard.putNumber("Left", Drivetrain.getInstance().getLeftMaster().getSelectedSensorPosition());
        SmartDashboard.putNumber("Right", Drivetrain.getInstance().getRightMaster().getSelectedSensorPosition());
    }

    public void end() {
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
    }

    @Override
    protected boolean isFinished() {
        //return Drivetrain.getInstance().getLeftMaster().isMotionProfileFinished() && Drivetrain.getInstance().getRightMaster().isMotionProfileFinished()
        return false;
    }               
}