package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

/**
 * Moves the Elevator to a desired position using Motion Magic
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * 
 * @since 6/30/19
 */
public class MoveElevatorMotionMagic extends TimedCommand {

    private int setpoint;
    private double kF, kP, kI, kD;
    private double prevVel;

    public MoveElevatorMotionMagic(int setpoint, double timeout) {
        super(timeout);
        this.setpoint = setpoint;
        prevVel = 0;
        requires(Elevator.getInstance());
    }
    
    @Override
    protected void initialize() {
        kF = SmartDashboard.getNumber("kF", Elevator.MOTION_MAGIC_KF);
        kP = SmartDashboard.getNumber("kP", Elevator.MOTION_MAGIC_KP);
        kI = SmartDashboard.getNumber("kI", Elevator.MOTION_MAGIC_KI);
        kD = SmartDashboard.getNumber("kD", Elevator.MOTION_MAGIC_KD);
        
        Elevator.getInstance().getMaster().config_kF(Elevator.MOTION_MAGIC_SLOT, kF);
        Elevator.getInstance().getMaster().config_kP(Elevator.MOTION_MAGIC_SLOT, kP);
        Elevator.getInstance().getMaster().config_kI(Elevator.MOTION_MAGIC_SLOT, kI);
        Elevator.getInstance().getMaster().config_kD(Elevator.MOTION_MAGIC_SLOT, kD);

        Elevator.getInstance().getMaster().selectProfileSlot(Elevator.MOTION_MAGIC_SLOT, RobotMap.PRIMARY_PID_INDEX);
    }

    @Override
    protected void execute() {
        double vel = Elevator.getInstance().getMaster().getActiveTrajectoryVelocity();
        double accelSign = Math.signum(vel - prevVel);
        Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, 
                                                DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF + 
                                                                                 Elevator.kS * Math.signum(Elevator.getInstance().getMaster().getClosedLoopError()) + 
                                                                                 Elevator.kA * accelSign);
        SmartDashboard.putNumber("Error", Elevator.getInstance().getMaster().getClosedLoopError());
        SmartDashboard.putNumber("Total Error", setpoint - Elevator.getInstance().getMaster().getSelectedSensorPosition());
        prevVel = vel;
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || Math.abs(Elevator.getInstance().getMaster().getSelectedSensorPosition() - setpoint) <= Elevator.ALLOWABLE_ERROR;
    }

    @Override
    protected void end() {
        Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, 
                                                DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
    }

    @Override
    protected void interrupted() {
        end();
    }
}