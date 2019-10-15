// package frc.robot.commands;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;

// import edu.wpi.first.wpilibj.command.TimedCommand;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.RobotMap;
// import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.WristRollers;

// public class ClimbElevatorWrist extends TimedCommand {
//     private static final double TIMEOUT = 30.0;//3.0;
//     private static final double WRIST_ROLLERS_PERCENT_OUTPUT = -0.2;
//     private static final double DRIVETRAIN_PERCENT_OUTPUT = -0.15;
//     private static boolean lvl2;

//     public ClimbElevatorWrist(boolean level2){
//         super(TIMEOUT);
//         requires(Elevator.getInstance());
//         requires(Wrist.getInstance());
//         requires(Climber.getInstance());
//         requires(Drivetrain.getInstance());
//         requires(WristRollers.getInstance());
//         lvl2 = level2;
//     }

//     protected void initialize() {
//         Wrist.getInstance().getMaster().configMotionCruiseVelocity((int)(Wrist.CRUISE_VELOCITY*0.5));
//         Climber.getInstance().getMaster().setSelectedSensorPosition(0);


//         Elevator.getInstance().getMaster().configPeakCurrentLimit(Elevator.CLIMBING_CURRENT_PEAK);      
//         Wrist.getInstance().getMaster().configPeakCurrentLimit(Wrist.CLIMBING_CURRENT_PEAK);    

//         Elevator.getInstance().getMaster().enableCurrentLimit(true);
//         Wrist.getInstance().getMaster().enableCurrentLimit(true);
//     }

//     protected void execute() {
//         Elevator.getInstance().getMaster().set(ControlMode.PercentOutput, Elevator.CLIMB_FF);
//         Wrist.getInstance().getMaster().set(ControlMode.MotionMagic, Wrist.END_CLIMB, DemandType.ArbitraryFeedForward, Wrist.CLIMB_FF+Wrist.HORIZONTAL_FORWARD_GRAV_FF);
//         Climber.getInstance().getMaster().set(ControlMode.MotionMagic, lvl2 ? Climber.LVL_2_POS : Climber.LVL_3_POS, DemandType.ArbitraryFeedForward, Climber.CLIMBER_FF);
//         WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, WRIST_ROLLERS_PERCENT_OUTPUT);
//         Drivetrain.getInstance().getLeftMaster().set(ControlMode.PercentOutput, DRIVETRAIN_PERCENT_OUTPUT);
//         Drivetrain.getInstance().getRightMaster().set(ControlMode.PercentOutput, DRIVETRAIN_PERCENT_OUTPUT);

//         SmartDashboard.putNumber("Climber Output", Climber.getInstance().getMaster().getMotorOutputPercent());   
//     }

//     @Override
//     protected boolean isFinished() {
//         return Math.abs(Climber.getInstance().getMaster().getClosedLoopError()) < Climber.ALLOWABLE_ERROR && isTimedOut();
//     }

//     protected void end() {
//         if(isTimedOut())
//         {
//             interrupted();
//         }

//         Elevator.getInstance().getMaster().configPeakCurrentLimit(Elevator.CURRENT_PEAK);      
//         Wrist.getInstance().getMaster().configPeakCurrentLimit(Wrist.PEAK_CURRENT_LIMIT);   

//         Elevator.getInstance().getMaster().enableCurrentLimit(true);
//         Wrist.getInstance().getMaster().enableCurrentLimit(true);
//     }

//     protected void interrupted() {
//         Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0);
//         Wrist.getInstance().getMaster().set(ControlMode.Disabled,0);
//         Climber.getInstance().getMaster().set(ControlMode.Disabled, 0);

//         Elevator.getInstance().getMaster().configPeakCurrentLimit(Elevator.CURRENT_PEAK);      
//         Wrist.getInstance().getMaster().configPeakCurrentLimit(Wrist.PEAK_CURRENT_LIMIT);   

//         Elevator.getInstance().getMaster().enableCurrentLimit(true);
//         Wrist.getInstance().getMaster().enableCurrentLimit(true);
//     }
        
// }

// // pranav backs into hab, 

// // 1st button press: climber pos - setelevator and wrist hatch 2 wrist pos slightly below ,intake wheels on hab, elev in right place

// // 2nd button press: 
// // sequential
// //     elevator down ff only, wrist ff, climber going down ff 
// //     if climber in pos, outtake to roll forward 


