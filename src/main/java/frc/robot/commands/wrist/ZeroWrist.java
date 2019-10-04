package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Wrist;

/**
 * Moves the wrist to the front and sets the bottom to 0. 
 */
public class ZeroWrist extends TimedCommand {

    static {
        if (RobotMap.PRACTICE_BOT)
            SPEED = 0.3;
        else
            SPEED = -0.2;
    }

    private static final double SPEED;
    private static final double INVALID_TIME = 0.06;
    private static final double TIMEOUT = 5;
    private double VELOCITY_ERROR = -60;
    private boolean isSpike;
    private double startTime;

    public ZeroWrist() {
        super(TIMEOUT);
        requires(Wrist.getInstance());
        isSpike = false;
    }

    @Override
    protected void initialize() {
        Wrist.getInstance().getMaster().selectProfileSlot(Wrist.VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX);
        isSpike = false;
        startTime = Timer.getFPGATimestamp();
        Wrist.getInstance().getMaster().configForwardSoftLimitEnable(false);
        Wrist.getInstance().getMaster().configReverseSoftLimitEnable(false);
    }

    @Override
    protected void execute() {
        Wrist.getInstance().getMaster().set(ControlMode.Velocity, SPEED * Wrist.CRUISE_VELOCITY);
        SmartDashboard.putNumber("velocity error", Wrist.getInstance().getMaster().getClosedLoopError());
        isSpike = Wrist.getInstance().getMaster().getClosedLoopError() <= VELOCITY_ERROR && Timer.getFPGATimestamp() - startTime >= INVALID_TIME;
        SmartDashboard.putBoolean("isSpike", isSpike);
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || isSpike;
    }

    @Override
    protected void end() {
        Wrist.getInstance().getMaster().setSelectedSensorPosition(RobotMap.PRACTICE_BOT ? Wrist.BACKMOST_POSITION : Wrist.FRONTMOST_POSITION);
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }

    @Override
    protected void interrupted() {
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}