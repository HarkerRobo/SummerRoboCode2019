/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchExtender;
import frc.robot.subsystems.HatchFlower;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristRollers;
import frc.robot.util.Limelight;
import harkerrobolib.commands.CancelCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @author Angela Jia
 * 
 * @since 6/14/19
 */
public class Robot extends TimedRobot {

    // Solenoid solenoid = new Solenoid(4);
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        //Initialize OI and button bindings
        OI.getInstance();
        
        //Initialize Subsystems
        Arm.getInstance();
        Wrist.getInstance();
        Elevator.getInstance();
        Drivetrain.getInstance();
        HatchFlower.getInstance();
        WristRollers.getInstance();
        HatchExtender.getInstance();
        
        Limelight.setLEDS(false);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Elevator Position", Elevator.getInstance().getMaster().getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist Position", Wrist.getInstance().getMaster().getSelectedSensorPosition());    
        SmartDashboard.putBoolean("Is scoring on cargo ship?", OI.getInstance().getCargoShipMode());
        SmartDashboard.putBoolean("Has hatch?", HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN);
        
        SmartDashboard.putString("Current Wrist Command", Wrist.getInstance().getCurrentCommandName());
    }

    /**
     * This autonomous (along with
     *  the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        HatchFlower.getInstance().getSolenoid().set(HatchFlower.OPEN);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
        if (Drivetrain.getInstance().getCurrentCommand() != null)
            new CancelCommand(Drivetrain.getInstance().getCurrentCommand()).start();
        if (Elevator.getInstance().getCurrentCommand() != null)
            new CancelCommand(Elevator.getInstance().getCurrentCommand()).start();
        if (Wrist.getInstance().getCurrentCommand() != null)
            new CancelCommand(Wrist.getInstance().getCurrentCommand()).start();

        Drivetrain.getInstance().applyToMasters((talon) -> talon.clearMotionProfileTrajectories());
        Elevator.getInstance().getMaster().clearMotionProfileTrajectories();
        Wrist.getInstance().getMaster().clearMotionProfileTrajectories();
        
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0);
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);

        Drivetrain.getInstance().applyToAll((talon) -> talon.setNeutralMode(NeutralMode.Brake));
    }

    @Override
    public void disabledPeriodic() {
        if (Drivetrain.getInstance().getCurrentCommand() != null)
            new CancelCommand(Drivetrain.getInstance().getCurrentCommand()).start();
        if (Elevator.getInstance().getCurrentCommand() != null)
            new CancelCommand(Elevator.getInstance().getCurrentCommand()).start();
        if (Wrist.getInstance().getCurrentCommand() != null)
            new CancelCommand(Wrist.getInstance().getCurrentCommand()).start();

        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0);
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}
