package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.wristrollers.SpinWristRollersManual;

/**
 * The Wrist Rollers Subsystem, which includes rollers on the wrist
 * 
 * @author Jatin Kohli
 * 
 * @since 6/14/19
 */
public class WristRollers extends Subsystem {
    
    private static WristRollers instance;
    
    private TalonSRX rollers;

    private WristRollers() {
        rollers = new TalonSRX(RobotMap.CAN_IDS.ROLLERS_TALON);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SpinWristRollersManual());
    }

    private void talonInit() {
        
    }

    public TalonSRX getRollers() {
        return rollers;
    }

    public static WristRollers getInstance() {
        if (instance == null)
            instance = new WristRollers();
        return instance;
    }
}