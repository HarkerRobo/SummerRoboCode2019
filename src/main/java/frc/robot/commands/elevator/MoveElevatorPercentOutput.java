package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import frc.robot.OI;
import frc.robot.subsystems.Elevator;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class MoveElevatorPercentOutput extends IndefiniteCommand {

    public MoveElevatorPercentOutput() {
        requires(Elevator.getInstance());
    }

   @Override
    protected void execute() {
        double output = -MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightY(), OI.XBOX_JOYSTICK_DEADBAND);
        
        Elevator.getInstance().getMaster().set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF);
    }

    @Override
    protected void interrupted() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}