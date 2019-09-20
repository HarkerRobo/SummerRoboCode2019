package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Wrist;

/**
 * Moves the Wrist Using the Motion Magic ControlMode
 * 
 * @author Jatin Kohli
 * @author Angela Jia
 * 
 * @since 7/13/19
 */
public class MoveWristMotionMagic extends TimedCommand {

    private static final double TIMEOUT = 3.0;
    private static final double INVALID_TIME = 0.06;
    private int setpoint;
    private double prevVel;
    private double startTime;
    private double allowableError;

    public MoveWristMotionMagic(int setpoint) {
        this(setpoint, Wrist.ALLOWABLE_ERROR);
    }

    public MoveWristMotionMagic(int setpoint, double allowableError) {
        super(TIMEOUT);
        requires(Wrist.getInstance());
        if (setpoint > Wrist.BACKMOST_POSITION || setpoint < Wrist.FRONTMOST_POSITION) {
            System.out.println("Wrist Motion Magic Setpoint is unsafe!");
            cancel();
        }
        this.setpoint = setpoint;
        this.allowableError = allowableError;
        prevVel = 0;
    }

    @Override
    protected void initialize() {
        Wrist.getInstance().getMaster().selectProfileSlot(Wrist.MOTION_MAGIC_SLOT, RobotMap.PRIMARY_PID_INDEX);
        Wrist.getInstance().getMaster().configClosedloopRamp(Wrist.RAMP_RATE);

        startTime = Timer.getFPGATimestamp();

        Wrist.getInstance().getMaster().configForwardSoftLimitEnable(true);
        Wrist.getInstance().getMaster().configReverseSoftLimitEnable(true);
    }

    @Override
    protected void execute() {
        double vel = Wrist.getInstance().getMaster().getActiveTrajectoryVelocity();
        double accelSign = Math.signum(vel - prevVel);
        double totalErrorSign = Math.signum(setpoint - Wrist.getInstance().getMaster().getSelectedSensorPosition());
        Wrist.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF() + Wrist.kS * totalErrorSign + Wrist.kA * accelSign * Wrist.MAX_ACCELERATION);
        prevVel = vel;

        SmartDashboard.putNumber("Wrist Error", Wrist.getInstance().getMaster().getClosedLoopError());
    }

    @Override
    protected boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime > INVALID_TIME) &&
                (isTimedOut() || Math.abs(setpoint - Wrist.getInstance().getMaster().getSelectedSensorPosition()) <= allowableError);
    }

    @Override
    protected void end() {
        Wrist.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF());
    }

    @Override
    protected void interrupted() {
        Wrist.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF());
    }
}