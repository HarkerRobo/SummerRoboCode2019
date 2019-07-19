package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Wrist;

/**
 * Moves the wrist to the front and sets the bottom to 0. 
 */
public class ZeroWrist extends TimedCommand {

    private double SPEED = -0.2;

    private static final double TIMEOUT = 1.0;

    public ZeroWrist() {
        super(TIMEOUT);
        requires(Wrist.getInstance());
    }

    @Override
    protected void execute() {
        Wrist.getInstance().getMaster().set(ControlMode.PercentOutput, SPEED);
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