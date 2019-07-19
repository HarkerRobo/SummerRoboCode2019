package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

/**
 * Moves the Elevator to the bottom at a safe speed and then zeros the master's encoder
 * 
 * @author Jatin Kohli
 * @author Angela Jia
 * 
 * @since 7/6/19
 */
public class ZeroElevator extends TimedCommand {
    
    private static final double TIMEOUT = 3.0;
    private static final double DOWN_PERCENT_OUTPUT = -0.15;
    private static final double CURRENT_SPIKE_DIFF = 0.4;
    private static final double INVALID_TIME = 0.06;
    private double prevCurrent;
    private boolean isSpike;
    private double startTime;

    public ZeroElevator() {
        super(3.0);
        requires(Elevator.getInstance());
    }

    @Override
    protected void initialize() {
        Elevator.getInstance().getMaster().set(ControlMode.PercentOutput, DOWN_PERCENT_OUTPUT);
        Elevator.getInstance().getMaster().configForwardSoftLimitEnable(false);
        Elevator.getInstance().getMaster().configReverseSoftLimitEnable(false);
        prevCurrent = Elevator.getInstance().getMaster().getOutputCurrent();

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    protected void execute() {
        double current = Elevator.getInstance().getMaster().getOutputCurrent();
        isSpike = current - prevCurrent > CURRENT_SPIKE_DIFF && Timer.getFPGATimestamp() - startTime > INVALID_TIME;
        prevCurrent = current;
        SmartDashboard.putNumber("Current", current);
    }

    @Override
    protected boolean isFinished() {
        return isSpike || isTimedOut();
    }

    @Override
    protected void end() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
        Elevator.getInstance().getMaster().setSelectedSensorPosition(0);
        Elevator.getInstance().getMaster().configForwardSoftLimitEnable(true);
        Elevator.getInstance().getMaster().configReverseSoftLimitEnable(true);
    }

    @Override
    protected void interrupted() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
        Elevator.getInstance().getMaster().configForwardSoftLimitEnable(true);
        Elevator.getInstance().getMaster().configReverseSoftLimitEnable(true);
    }
}
