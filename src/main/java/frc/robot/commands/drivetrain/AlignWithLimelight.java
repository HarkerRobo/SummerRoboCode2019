package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.HSPIDController;
import frc.robot.util.Limelight;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.Conversions.SpeedUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Aligns the Robot with the target, using the Limelight's tX to turn towards the target
 * and its thor to get close enough to the target
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * 
 * @since 8/14/19
 */
public class AlignWithLimelight extends Command {
    
    private HSPIDController txController;
    private HSPIDController thorController;

    public AlignWithLimelight() {
        requires(Drivetrain.getInstance());
        
        txController = new HSPIDController(Drivetrain.TX_kP, Drivetrain.TX_kI, Drivetrain.TX_kD, 
                () -> Limelight.getTx(), PIDSourceType.kDisplacement);
        thorController = new HSPIDController(Drivetrain.THOR_kP, Drivetrain.THOR_kI, Drivetrain.THOR_kD, 
                () -> Limelight.getThor(), PIDSourceType.kDisplacement);

        txController.setSetpoint(Drivetrain.TX_SETPOINT);
        thorController.setSetpoint(Drivetrain.THOR_SETPOINT);
    }

    @Override
    protected void initialize() {
        txController.enable();
        thorController.enable();

        Drivetrain.getInstance().applyToMasters((talon) -> talon.selectProfileSlot(Drivetrain.VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.setNeutralMode(NeutralMode.Brake));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.configClosedloopRamp(Drivetrain.VELOCITY_RAMP_RATE));
    }

    @Override
    protected void execute() {
        double speed = -thorController.getOutput() * Drivetrain.MAX_FORWARD_VELOCITY;
        double turn = txController.getOutput() * Drivetrain.MAX_TURN_VELOCITY;

        speed = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, speed, SpeedUnit.ENCODER_UNITS);
        turn = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, turn, SpeedUnit.ENCODER_UNITS);
        
        Drivetrain.getInstance().getLeftMaster().set(ControlMode.Velocity, speed - turn, DemandType.ArbitraryFeedForward, Math.signum(speed - turn)*Drivetrain.leftkS);
        Drivetrain.getInstance().getRightMaster().set(ControlMode.Velocity, speed + turn, DemandType.ArbitraryFeedForward, Math.signum(speed + turn)*Drivetrain.rightkS);
    
        SmartDashboard.putNumber("thor", Limelight.getThor());
        SmartDashboard.putNumber("tx Error", txController.getError());
        SmartDashboard.putNumber("thor Error", thorController.getError());
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(txController.getError()) < Drivetrain.TX_ALLOWABLE_ERROR &&
                Math.abs(thorController.getError()) < Drivetrain.THOR_ALLOWABLE_ERROR;
    }

    @Override
    protected void end() {
        txController.reset();
        thorController.reset();
        Drivetrain.getInstance().applyToMasters((talon) -> talon.set(ControlMode.Disabled, 0));
    }

  @Override
  protected void interrupted() {
      end();
  }
}