package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Wrist;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Moves the Wrist Manually using the Driver's Right Joystick
 * 
 * @since 6/26/19
 */
public class MoveWristManual extends IndefiniteCommand {

    private double SPEED_MULTIPLIER = 0.3;

    public MoveWristManual() {
        requires(Wrist.getInstance());
    }

    @Override
    protected void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);
        Wrist.getInstance().getMaster().set(ControlMode.PercentOutput, SPEED_MULTIPLIER*speed, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF());
    }

    @Override
    protected void interrupted() {
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}