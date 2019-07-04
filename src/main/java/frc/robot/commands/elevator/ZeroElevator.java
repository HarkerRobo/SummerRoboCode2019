package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends TimedCommand {
    
    private static final double DOWN_PERCENT_OUTPUT = -0.1;
    private static final double TIMEOUT = 3;

    public ZeroElevator() {
        super(TIMEOUT);
        requires(Elevator.getInstance());
    }

    @Override
    protected void initialize() {
        Elevator.getInstance().getMaster().set(ControlMode.PercentOutput, DOWN_PERCENT_OUTPUT);
    }

    @Override
    protected void execute() {
        SmartDashboard.putNumber("Current", Elevator.getInstance().getMaster().getOutputCurrent());
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
        Elevator.getInstance().getMaster().setSelectedSensorPosition(0);
    }

    @Override
    protected void interrupted() {
        
    }
}