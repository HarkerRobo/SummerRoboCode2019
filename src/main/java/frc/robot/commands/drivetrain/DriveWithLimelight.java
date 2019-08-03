package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.HSPIDController;
import frc.robot.util.Limelight;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.MathUtil;
import harkerrobolib.util.Conversions.SpeedUnit;

/**
 * Aligns the Robot with a Field Element while allowing the driver to move forward and backward
 * 
 * @author Jatin Kohli
 * 
 * @since 8/3/19
 */
public class DriveWithLimelight extends IndefiniteCommand {
    private static final double SPEED_MULTIPLIER = 0.2;
    private HSPIDController controller;

    public DriveWithLimelight() {
        requires(Drivetrain.getInstance());
        controller = new HSPIDController(Drivetrain.LIMELIGHT_kP, Drivetrain.LIMELIGHT_kI, Drivetrain.LIMELIGHT_kD, 
                () -> Limelight.getTx(), PIDSourceType.kDisplacement);
    }

    @Override
    protected void initialize() {
        Drivetrain.getInstance().applyToMasters((talon) -> talon.selectProfileSlot(Drivetrain.VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.configClosedloopRamp(Drivetrain.VELOCITY_RAMP_RATE));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.setNeutralMode(NeutralMode.Brake));
        controller.enable();
        
        SmartDashboard.putString("Drivetrain Mode", "Limelight");
    }

    @Override
    protected void execute() {
        double speed = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.XBOX_JOYSTICK_DEADBAND) * Drivetrain.MAX_FORWARD_VELOCITY * SPEED_MULTIPLIER;
        speed = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, speed, SpeedUnit.ENCODER_UNITS);

        double turn = controller.getOutput() * Drivetrain.MAX_TURN_VELOCITY;
        turn = Conversions.convertSpeed(SpeedUnit.FEET_PER_SECOND, turn, SpeedUnit.ENCODER_UNITS);

        Drivetrain.getInstance().getLeftMaster().set(ControlMode.Velocity, speed - turn, DemandType.ArbitraryFeedForward, Drivetrain.leftkS * Math.abs(speed - turn));
        Drivetrain.getInstance().getRightMaster().set(ControlMode.Velocity, speed + turn, DemandType.ArbitraryFeedForward, Drivetrain.rightkS * Math.abs(speed + turn));
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected void end() {
        controller.reset();
        Drivetrain.getInstance().applyToMasters((talon) -> talon.set(ControlMode.Disabled, 0));
        Drivetrain.getInstance().applyToMasters((talon) -> talon.setNeutralMode(NeutralMode.Brake));
    }
}