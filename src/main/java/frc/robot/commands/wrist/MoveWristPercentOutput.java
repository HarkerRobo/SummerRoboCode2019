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
public class MoveWristPercentOutput extends IndefiniteCommand {

    private static final double LAG_COMPENSATION = 0.7;
    private double SPEED_MULTIPLIER = 0.3;
    private double lastSetpoint;
    private boolean shouldHold;

    public MoveWristPercentOutput() {
        requires(Wrist.getInstance());
    }

    @Override
    protected void initialize() {
        lastSetpoint = Wrist.getInstance().getMaster().getSelectedSensorPosition();
        shouldHold = false;

        Wrist.getInstance().getMaster().configForwardSoftLimitEnable(false);
        Wrist.getInstance().getMaster().configReverseSoftLimitEnable(false);
    }

    @Override
    protected void execute() {
        double output = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.XBOX_JOYSTICK_DEADBAND);
        
        if (Math.abs(output) > 0) {
            Wrist.getInstance().getMaster().set(ControlMode.PercentOutput, SPEED_MULTIPLIER*output, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF() + Wrist.kS * Math.signum(output));
            lastSetpoint = Wrist.getInstance().getMaster().getSelectedSensorPosition() + (int)(Wrist.getInstance().getMaster().getSelectedSensorVelocity() * LAG_COMPENSATION);
            shouldHold = true;
        }
        else if (shouldHold) {
            if (lastSetpoint > Wrist.BACKMOST_POSITION)
                lastSetpoint = Wrist.HORIZONTAL_BACK;
            else if (lastSetpoint < Wrist.FRONTMOST_POSITION)
                lastSetpoint = Wrist.HORIZONTAL_FRONT;
            Wrist.getInstance().getMaster().set(ControlMode.MotionMagic, lastSetpoint, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF() + Math.signum(output) * Wrist.kS);
        }
   }

    @Override
    protected void interrupted() {
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF());
    }
}