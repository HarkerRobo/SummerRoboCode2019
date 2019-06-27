package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.PositionUnit;

/**
 * Follows the Specified Left and Right Trajectories by closed looping to each position and adding the necessary Arbitrary FeedForward
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/19/19
 */
public class DriveWithMotionProfileOld extends Command {

    private Notifier notifier;
    private double timeDur;
    private int index;
    private int maxIndex;

    public DriveWithMotionProfileOld(double[][] leftPath, double[][] rightPath, double timeDur) {
        requires(Drivetrain.getInstance());

        //Drivetrain.getInstance().applyToMasters((talon) -> talon.selectProfileSlot(Drivetrain.MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX));
        
        Drivetrain.getInstance().getLeftMaster().selectProfileSlot(Drivetrain.MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX);
        Drivetrain.getInstance().getRightMaster().selectProfileSlot(Drivetrain.MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX);
        Drivetrain.getInstance().applyToMasters((talon) -> talon.configClosedloopRamp(Drivetrain.MOTION_PROF_RAMP_RATE));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.setSelectedSensorPosition(0));

        System.out.println(Drivetrain.getInstance().getLeftMaster().getSelectedSensorPosition());

        this.timeDur = timeDur;
        
        index = 0;
        maxIndex = leftPath.length;
        notifier = new Notifier(() -> {
            if (index < leftPath.length) {
                double leftPosition = Conversions.convertPosition(PositionUnit.FEET, leftPath[index][0], PositionUnit.ENCODER_UNITS);
                double rightPosition = Conversions.convertPosition(PositionUnit.FEET, rightPath[index][0], PositionUnit.ENCODER_UNITS);

                double leftVelocity = leftPath[index][1];
                double rightVelocity = rightPath[index][1];

                double leftAcceleration = leftPath[index][2];
                double rightAcceleration = rightPath[index][2];

                double leftArbitraryFeedForward = Drivetrain.leftkS + Drivetrain.leftkF * leftVelocity + Drivetrain.kA * leftAcceleration;
                double rightArbitraryFeedForward = Drivetrain.rightkS + Drivetrain.rightkF * rightVelocity + Drivetrain.kA * rightAcceleration;

                Drivetrain.getInstance().getLeftMaster().set(ControlMode.Position, leftPosition); //, DemandType.ArbitraryFeedForward, leftArbitraryFeedForward);
                Drivetrain.getInstance().getRightMaster().set(ControlMode.Position, rightPosition); //, DemandType.ArbitraryFeedForward, rightArbitraryFeedForward);

                //Drivetrain.getInstance().getLeftMaster().set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, leftArbitraryFeedForward);
                //Drivetrain.getInstance().getRightMaster().set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, rightArbitraryFeedForward);

                SmartDashboard.putNumber("Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
                SmartDashboard.putNumber("Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
        
                SmartDashboard.putNumber("Left Output", Drivetrain.getInstance().getLeftMaster().getMotorOutputPercent());
                SmartDashboard.putNumber("Right Output", Drivetrain.getInstance().getRightMaster().getMotorOutputPercent());

                index++;
                System.out.println(index);
            }
        });
    }

    @Override
    protected void initialize() {
        System.out.println("Motion Prof Init");

        notifier.startPeriodic(timeDur);
    }

    @Override
    protected boolean isFinished() {
        return index >= maxIndex - 1;
    }

    @Override
    protected void end() {
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
        notifier.close();
        System.out.println("End Motion Prof");
        index = 0;
    }

    @Override
    protected void interrupted() {
        end();
    }
}