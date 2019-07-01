package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorMotionMagic extends Command {

    private int setpoint;

    public MoveElevatorMotionMagic(int setpoint) {
        this.setpoint = setpoint;
        requires(Elevator.getInstance());
    }

    @Override
    protected void execute() {
        Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint);
    }

    @Override
    protected void isFinished() {
        return Math.abs(Elevator.getInstance().getMaster().getSelectedSensorPosition() - setpoint) <= Elevator.ALLOWABLE_ERROR;
    }
}