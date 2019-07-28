package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Wrist;

/**
 * Moves the wrist to the front and sets the bottom to 0. 
 */
public class ZeroWrist extends TimedCommand {

    private double SPEED = -0.2;
    private double VELOCITY_SPIKE = -73;
    private boolean isSpike;
    private double startTime;
    private static final double INVALID_TIME = 0.06;
    private static final double TIMEOUT = 1.0;

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
    }

    @Override
    protected void execute() {
        Wrist.getInstance().getMaster().set(ControlMode.Velocity, SPEED * Wrist.CRUISE_VELOCITY);
        SmartDashboard.putNumber("velocity error", Wrist.getInstance().getMaster().getClosedLoopError());
        isSpike = Wrist.getInstance().getMaster().getClosedLoopError() <= VELOCITY_SPIKE && Timer.getFPGATimestamp() - startTime >= INVALID_TIME;
        SmartDashboard.putBoolean("isSpike", isSpike);
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || isSpike;
    }

    @Override
    protected void end() {
        Wrist.getInstance().getMaster().setSelectedSensorPosition(Wrist.FRONTMOST_POSITION);
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }

    @Override
    protected void interrupted() {
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}