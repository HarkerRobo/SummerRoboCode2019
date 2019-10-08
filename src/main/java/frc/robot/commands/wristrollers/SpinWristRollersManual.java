package frc.robot.commands.wristrollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.OI.DemoMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristRollers;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Spins the Wrist Rollers using input from the Driver Gamepad's Triggers
 * 
 * @author Jatin Kohli
 * 
 * @since 6/17/19
 */
public class SpinWristRollersManual extends IndefiniteCommand {
    
    static {
        if(OI.mode == DemoMode.NORMAL) {
            OUTTAKE_SPEED_MULTIPLIER = 0.45;
        } else {
            OUTTAKE_SPEED_MULTIPLIER = 0.3;
        }
    }
    private static final double INTAKE_SPEED_MULTIPLIER = 0.7;
    private static final double OUTTAKE_SPEED_MULTIPLIER;

    public SpinWristRollersManual() {
        requires(WristRollers.getInstance());
        //requires(Arm.getInstance());
    }

    @Override
    protected void initialize() {
        // if(RobotMap.PRACTICE_BOT){
        //     Arm.getInstance().getSolenoid().set(Arm.OUT);
        // }
    }
    @Override
    protected void execute() {
        
        boolean a = OI.getInstance().getDriverGamepad().getButtonAState();
        boolean x = OI.getInstance().getDriverGamepad().getButtonXState();
        double output;
        output = (a ? 1 : (x ? -1 : 0));
        output *= Math.signum(output) == -1 ? INTAKE_SPEED_MULTIPLIER : OUTTAKE_SPEED_MULTIPLIER;
        
        if (Math.abs(output) > 0)
            WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, output);
        else
            WristRollers.getInstance().getRollers().set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, WristRollers.CARGO_FF);
        
        SmartDashboard.putNumber("WristRoller Speed", WristRollers.getInstance().getRollers().getMotorOutputPercent());
        SmartDashboard.putNumber("WristRoller current", WristRollers.getInstance().getRollers().getOutputCurrent());
        }
}