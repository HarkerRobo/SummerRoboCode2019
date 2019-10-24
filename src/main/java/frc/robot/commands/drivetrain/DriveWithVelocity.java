package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.OI.DemoMode;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.MathUtil;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Moves the Drivetrain using Velocity PID
 * 
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * 
 * @since 6/15/19
 */
public class DriveWithVelocity extends IndefiniteCommand {

    private static final double SPEED_MULTIPLIER;
    private static final double SLOW_MODE_MULTIPLIER = 0.2;
    
    static {
        if(OI.mode == DemoMode.SAFE) {
            SPEED_MULTIPLIER = 0.5;
        } else {
            SPEED_MULTIPLIER = 1;
        }
    }

    private boolean hasJoystickInput;

    public DriveWithVelocity() {
        requires(Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        Drivetrain.getInstance().applyToMasters((talon) -> talon.selectProfileSlot(Drivetrain.VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.configClosedloopRamp(Drivetrain.VELOCITY_RAMP_RATE));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.setNeutralMode(NeutralMode.Brake));

        hasJoystickInput = false;

        SmartDashboard.putString("Drivetrain Mode", "Manual");
    }

    @Override
    protected void execute() {
        double joystickY = OI.getInstance().getDriverGamepad().getLeftY();
        double joystickX = OI.getInstance().getDriverGamepad().getLeftX();
        
        double speed = MathUtil.mapJoystickOutput(joystickY, OI.XBOX_JOYSTICK_DEADBAND) * Drivetrain.MAX_FORWARD_VELOCITY * SPEED_MULTIPLIER;
        double turn = MathUtil.mapJoystickOutput(joystickX, OI.XBOX_JOYSTICK_DEADBAND);
        turn = turn * turn * Math.signum(turn);
        turn *= Drivetrain.MAX_TURN_VELOCITY * SPEED_MULTIPLIER;

        if(OI.getInstance().getDriverGamepad().getLeftTrigger() > OI.XBOX_TRIGGER_DEADBAND) {
            speed *= SLOW_MODE_MULTIPLIER;
            turn *= SLOW_MODE_MULTIPLIER;
        }

        speed = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, speed, SpeedUnit.ENCODER_UNITS);
        turn = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, turn, SpeedUnit.ENCODER_UNITS);

        if (speed != 0 || turn != 0)
            hasJoystickInput = true;
        if(hasJoystickInput) {
            Drivetrain.getInstance().getLeftMaster().set(ControlMode.Velocity, speed + turn, DemandType.ArbitraryFeedForward, Math.signum(speed + turn)*Drivetrain.leftkS);
            Drivetrain.getInstance().getRightMaster().set(ControlMode.Velocity, speed - turn, DemandType.ArbitraryFeedForward, Math.signum(speed - turn)*Drivetrain.rightkS);
        }

        SmartDashboard.putNumber("Left Error", Drivetrain.getInstance().getLeftMaster().getClosedLoopError());
        SmartDashboard.putNumber("Right Error", Drivetrain.getInstance().getRightMaster().getClosedLoopError());
        SmartDashboard.putNumber("left current", Drivetrain.getInstance().getLeftMaster().getOutputCurrent());
        SmartDashboard.putNumber("right current", Drivetrain.getInstance().getRightMaster().getOutputCurrent());
    }

    @Override
    protected void interrupted() {
        Drivetrain.getInstance().setBoth(ControlMode.Disabled, 0);
    }
}