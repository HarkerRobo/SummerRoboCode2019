package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Moves the Elevator using Percent Output mode manually with the right joystick
 * 
 * @author Jatin Kohli
 * 
 * @since 6/26/19
 */
public class MoveElevatorPercentOutput extends IndefiniteCommand {

    private static final double SPEED_MULTIPLIER = 0.4; //Remember to change CURRENT_SPIKE_MAGNITUDE in ZeroElevator

    public MoveElevatorPercentOutput() {
        requires(Elevator.getInstance());
    }

   @Override
    protected void execute() {
        double output = SPEED_MULTIPLIER * MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightY(), OI.XBOX_JOYSTICK_DEADBAND);
        
        SmartDashboard.putNumber("Current", Elevator.getInstance().getMaster().getOutputCurrent());
        Elevator.getInstance().getMaster().set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
    }

    @Override
    protected void interrupted() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}