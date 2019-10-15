// package frc.robot.commands;

// import com.ctre.phoenix.motorcontrol.ControlMode;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.WristRollers;

// public class DriveIntoWall extends Command {

//     private static final double DT_OUTPUT = 0.5;
//     private static final double ROLLERS_OUTPUT = 1.0;

//     public DriveIntoWall() {
//         requires(Drivetrain.getInstance());
//         requires(WristRollers.getInstance());
//     }

//     @Override
//     protected void execute() {
//         Drivetrain.getInstance().applyToMasters((talon) -> talon.set(ControlMode.PercentOutput, DT_OUTPUT));
//         WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, ROLLERS_OUTPUT);
//     }

//     @Override
//     protected boolean isFinished() {
//         return false;
//     }
// }