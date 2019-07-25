// package frc.robot.commands.drivetrain;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;

// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.RobotMap;
// import frc.robot.subsystems.Drivetrain;
// import harkerrobolib.util.Conversions;
// import harkerrobolib.util.Conversions.PositionUnit;

// /**
//  * Follows the Specified Left and Right Trajectories by using Position Closed Loop on the Talons
//  * 
//  * @author Jatin Kohli
//  * @author Chirag Kaushik
//  * 
//  * @since 6/19/19
//  */
// public class DriveWithMotionProfileOld extends Command {

//     private Notifier notifier;
//     private double timeDur;
//     private int index;
//     private int maxIndex;
//     private double[][] leftPath;
//     private double[][] rightPath;

//     /**
//      * @param leftPath The 2D array representing the left path ()
//      * @param rightPath
//      * @param timeDur
//      */
//     public DriveWithMotionProfileOld(double[][] leftPath, double[][] rightPath, double timeDur) {
//         requires(Drivetrain.getInstance());
//         Drivetrain.getInstance().applyToMasters((talon) -> talon.selectProfileSlot(Drivetrain.MOTION_PROF_SLOT, RobotMap.PRIMARY_PID_INDEX));

//         Drivetrain.getInstance().applyToMasters((talon) -> talon.configClosedloopRamp(Drivetrain.MOTION_PROF_RAMP_RATE));
//         System.out.println(Drivetrain.getInstance().getLeftMaster().getSelectedSensorPosition());

//         this.timeDur = timeDur;
//         this.leftPath = leftPath;
//         this.rightPath = rightPath;
        
//         index = 0;
//         maxIndex = leftPath.length;
        
//     }

//     @Override
//     protected void initialize() {
//         System.out.println("before notifier");
//         Drivetrain.getInstance().applyToMasters((talon) -> talon.setSelectedSensorPosition(0));
//         notifier = new Notifier(() -> {
//             if (index < leftPath.length) {
//                 double leftPosition = Conversions.convertPosition(PositionUnit.FEET, leftPath[index][0], PositionUnit.ENCODER_UNITS);
//                 double rightPosition = Conversions.convertPosition(PositionUnit.FEET, rightPath[index][0], PositionUnit.ENCODER_UNITS);
//                 SmartDashboard.putNumber("leftPosition", leftPosition);
//                 SmartDashboard.putNumber("rightPosition", rightPosition);
//                 double leftVelocity = leftPath[index][1];
//                 double rightVelocity = rightPath[index][1];

//                 double leftAcceleration = leftPath[index][2];
//                 double rightAcceleration = rightPath[index][2];

//                 double leftArbitraryFeedForward = Drivetrain.leftkF * leftVelocity; // Drivetrain.leftkS * Math.signum(leftVelocity) + Drivetrain.kA * leftAcceleration;
//                 double rightArbitraryFeedForward = Drivetrain.rightkF * rightVelocity;  // Drivetrain.rightkS * Math.signum(rightVelocity) + Drivetrain.kA * rightAcceleration;

//                 Drivetrain.getInstance().getLeftMaster().set(ControlMode.PercentOutput, Drivetrain.leftkF * leftVelocity);
//                 Drivetrain.getInstance().getRightMaster().set(ControlMode.PercentOutput, Drivetrain.rightkF * rightVelocity);
//                 // Drivetrain.getInstance().getLeftMaster().set(ControlMode.Position, leftPosition, DemandType.ArbitraryFeedForward, leftArbitraryFeedForward); //, DemandType.ArbitraryFeedForward, leftArbitraryFeedForward);
//                 // Drivetrain.getInstance().getRightMaster().set(ControlMode.Position, rightPosition, DemandType.ArbitraryFeedForward, rightArbitraryFeedForward); //, DemandType.ArbitraryFeedForward, rightArbitraryFeedForward);

//                 //Drivetrain.getInstance().getLeftMaster().set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, leftArbitraryFeedForward);
//                 // // //Drivetrain.getInstance().getRightMaster().set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, rightArbitraryFeedForward);

//                 // SmartDashboard.putNumber("left velocity error", leftVelocity - Drivetrain.getInstance().getLeftMaster().getSelectedSensorVelocity());
//                 // SmartDashboard.putNumber("right velocity error", rightVelocity - Drivetrain.getInstance().getRightMaster().getSelectedSensorVelocity());

//                 // SmartDashboard.putNumber("Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
//                 // SmartDashboard.putNumber("Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
        
//                 SmartDashboard.putNumber("Left desired velocity", leftVelocity);
//                 SmartDashboard.putNumber("Right desired velocity", rightVelocity);

//                 SmartDashboard.putNumber("Left current velocity", Drivetrain.getInstance().getLeftMaster().getSelectedSensorVelocity());
//                 SmartDashboard.putNumber("Right current velocity", Drivetrain.getInstance().getRightMaster().getSelectedSensorVelocity());

//                 index++;
//                 System.out.println(index);
//             }
//             else {
//                 Drivetrain.getInstance().applyToMasters((talon) -> talon.set(ControlMode.Disabled, 0));
//             }
//         });
//         System.out.println("Motion Prof Init");

//         notifier.startPeriodic(timeDur);
//     }

//     @Override
//     protected boolean isFinished() {
//         return false;
//     }

//     @Override
//     protected void end() {
//         Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
//         notifier.close();
//         System.out.println("End Motion Prof");
//         index = 0;
//     }

//     @Override
//     protected void interrupted() {
//         end();
//     }
// }