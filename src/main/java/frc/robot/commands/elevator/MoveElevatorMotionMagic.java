package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.Timer;
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

    private static final double TIMEOUT = 3.0;
    private static final double INVALID_TIME = 0.06;
    private int setpoint;
    private double prevVel;
    private double startTime;
    private double allowableError;

    public MoveElevatorMotionMagic(int setpoint) {
        this(setpoint, Elevator.ALLOWABLE_ERROR);
    }

    public MoveElevatorMotionMagic(int setpoint, int allowableError) {
        super(TIMEOUT);
        requires(Elevator.getInstance());
        this.setpoint = setpoint;
        prevVel = 0;
        this.allowableError = allowableError;
    }
    
    @Override
    protected void initialize() {
        Elevator.getInstance().getMaster().selectProfileSlot(Elevator.MOTION_MAGIC_SLOT, RobotMap.PRIMARY_PID_INDEX);
        startTime = Timer.getFPGATimestamp();

        Elevator.getInstance().getMaster().configForwardSoftLimitEnable(true);
        Elevator.getInstance().getMaster().configReverseSoftLimitEnable(true);
    }

    @Override
    protected void execute() {
        double vel = Elevator.getInstance().getMaster().getActiveTrajectoryVelocity();
        double accelSign = Math.signum(vel - prevVel);
        Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, 
                                                DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF + 
                                                                                 Elevator.kS * Math.signum(Elevator.getInstance().getMaster().getClosedLoopError()) + 
                                                                                 Elevator.kA * accelSign);
        SmartDashboard.putNumber("el setpoint", setpoint);
        SmartDashboard.putNumber("el ks", Elevator.kS * Math.signum(Elevator.getInstance().getMaster().getClosedLoopError()));
        SmartDashboard.putNumber("el ka", Elevator.kA * accelSign);
        SmartDashboard.putNumber("el output", Elevator.getInstance().getMaster().getMotorOutputPercent());
        SmartDashboard.putNumber("el error", Elevator.getInstance().getMaster().getClosedLoopError());

        prevVel = vel;
    }

    @Override
    protected boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > INVALID_TIME &&
                (isTimedOut() || Math.abs(Elevator.getInstance().getMaster().getSelectedSensorPosition() - setpoint) <= allowableError);
    }

    @Override
    protected void end() {
        Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, 
                                                DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
    }

    @Override
    protected void interrupted() {
        Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, 
                                                DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
    }
}