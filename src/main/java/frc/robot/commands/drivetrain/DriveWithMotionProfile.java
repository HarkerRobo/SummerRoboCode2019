package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.PositionUnit;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Follows the Specified Left and Right Trajectories by closed looping to each position and adding the necessary Arbitrary FeedForward
 * 
 * @author Jatin Kohli
 * @author Angela Jia
 * 
 * @since 7/23/19
 */
public class DriveWithMotionProfile extends Command {

    private static final int MIN_BUFFERED_PTS = 4;
    private BufferedTrajectoryPointStream leftStream;
    private BufferedTrajectoryPointStream rightStream;
    private double leftLastSetpoint;
    private double rightLastSetpoint;
    private double timeTaken;

    /**
     * @param leftPath The 2D Array of doubles representing the left side path (p,v,a,h)
     * @param rightPath The 2D Array of doubles representing the right side path (p,v,a,h)
     * @param timeDur The time between each segment of the path in milliseconds
     */
    public DriveWithMotionProfile(double[][] leftPath, double[][] rightPath, int timeDur) {
        requires(Drivetrain.getInstance());
        leftStream = new BufferedTrajectoryPointStream();
        rightStream = new BufferedTrajectoryPointStream();
        setupTrajectoryStream(leftStream, leftPath, true);
        setupTrajectoryStream(rightStream, rightPath, false);
        Drivetrain.getInstance().applyToMasters((talon) -> talon.configMotionProfileTrajectoryPeriod(timeDur));
        timeTaken = leftPath.length * timeDur;
    }

    private void setupTrajectoryStream(BufferedTrajectoryPointStream stream, double [][] path, boolean isLeft) {
        for(int i = 0; i < path.length; i++) {
            TrajectoryPoint point = new TrajectoryPoint();
            point.position = Conversions.convert(PositionUnit.FEET, path[i][0], PositionUnit.ENCODER_UNITS);
            point.velocity = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, path[i][1], SpeedUnit.ENCODER_UNITS);
            point.arbFeedFwd = path[i][2]*(isLeft ? Drivetrain.leftkA : Drivetrain.rightkA) + Math.signum(point.velocity) * (isLeft ? Drivetrain.leftkS : Drivetrain.rightkS);
            if(i == 0) 
                point.zeroPos = true;
            else if(i == path.length-1) {
                point.isLastPoint = true;
                if (isLeft)
                    leftLastSetpoint = point.position;
                else
                    rightLastSetpoint = point.position;
            }
            point.profileSlotSelect0 = Drivetrain.MOTION_PROF_SLOT;
            point.timeDur = 0;
            stream.Write(point);
        }
    }
    
    @Override
    protected void initialize() {
        System.out.println("mp initialize");
        Drivetrain.getInstance().applyToMasters((talon) -> talon.selectProfileSlot(Drivetrain.MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX));
        Drivetrain.getInstance().getLeftMaster().startMotionProfile(leftStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        Drivetrain.getInstance().getRightMaster().startMotionProfile(rightStream, MIN_BUFFERED_PTS, ControlMode.MotionProfile);
        
    }

    @Override
    protected void execute() {
        SmartDashboard.putNumber("Left Velocity", Drivetrain.getInstance().getLeftMaster().getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Right Velocity", Drivetrain.getInstance().getRightMaster().getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
        SmartDashboard.putNumber("Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
    }

    @Override
    protected boolean isFinished() {
        return Drivetrain.getInstance().getLeftMaster().isMotionProfileFinished() && Drivetrain.getInstance().getRightMaster().isMotionProfileFinished();
    }

    @Override
    protected void end() {
        System.out.println("mp end");
    }

    @Override
    protected void interrupted() {
        System.out.println("mp interrupted");
        end();
    }
}