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

    public MoveElevatorMotionMagic(int setpoint, double timeout) {
        super(timeout);
        this.setpoint = setpoint;
        requires(Elevator.getInstance());
    }
    
    @Override
    protected void initialize() {
        Elevator.getInstance().getMaster().selectProfileSlot(Elevator.MOTION_MAGIC_SLOT, RobotMap.PRIMARY_PID_INDEX);
    }

    @Override
    protected void execute() {
        Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, 
                                                DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF + Elevator.kS * Math.signum(Elevator.getInstance().getMaster().getClosedLoopError()));

        SmartDashboard.putNumber("Error", Elevator.getInstance().getMaster().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX));
        SmartDashboard.putNumber("Output", Elevator.getInstance().getMaster().getMotorOutputPercent());
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || Math.abs(Elevator.getInstance().getMaster().getSelectedSensorPosition() - setpoint) <= Elevator.ALLOWABLE_ERROR;
    }

    @Override
    protected void end() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
    }

    @Override
    protected void interrupted() {
        end();
    }
}