package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Wrist;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Moves the Wrist Manually using the Driver's Right Joystick
 * 
 * @since 6/26/19
 */
public class MoveWristPercentOutput extends IndefiniteCommand {

    static {
        if (RobotMap.PRACTICE_BOT)
            LAG_COMPENSATION = 0.7;
        else
            LAG_COMPENSATION = 1;
    }

    private static final double LAG_COMPENSATION;
    private static final double SPEED_MULTIPLIER = 0.3;
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
        double driverRightX = OI.getInstance().getDriverGamepad().getRightX();
        double operatorRightX = OI.getInstance().getOperatorGamepad().getRightX();

        double joystickValue = Math.abs(driverRightX) > 0 ? driverRightX : operatorRightX;
        
        double output = MathUtil.mapJoystickOutput(joystickValue, OI.XBOX_JOYSTICK_DEADBAND);
        
        if (Math.abs(output) > 0) {
            Wrist.getInstance().getMaster().set(ControlMode.PercentOutput, SPEED_MULTIPLIER*output);//, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF() + Wrist.kS * Math.signum(output));
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