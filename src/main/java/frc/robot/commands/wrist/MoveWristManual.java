package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.OI;
import frc.robot.subsystems.Wrist;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class MoveWristManual extends IndefiniteCommand {

    public MoveWristManual() {
        requires(Wrist.getInstance());
    }

    @Override
    protected void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightY(), OI.XBOX_JOYSTICK_DEADBAND);

        Wrist.getInstance().getMaster().set(ControlMode.PercentOutput, speed);
    }

    @Override
    protected void end() {
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}