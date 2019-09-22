package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.OI.DemoMode;
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

    private static final double SPEED_MULTIPLIER = 0.3;
    private static final double LAG_COMPENSATION = 0.5;
    private int lastSetpoint;
    private boolean shouldHold;

    public MoveElevatorPercentOutput() {
        requires(Elevator.getInstance());
    }
    
    @Override
    protected void initialize() {
        lastSetpoint = Elevator.getInstance().getMaster().getSelectedSensorPosition();
        shouldHold = false;

        Elevator.getInstance().getMaster().configForwardSoftLimitEnable(false);
        Elevator.getInstance().getMaster().configReverseSoftLimitEnable(false);
    }

    @Override
    protected void execute() {
        double driverRightY = OI.getInstance().getDriverGamepad().getRightY();
        double operatorRightY = OI.getInstance().getOperatorGamepad().getRightY();

        double joystickValue = Math.abs(driverRightY) > 0 ? driverRightY : operatorRightY;
        
        double output = SPEED_MULTIPLIER * MathUtil.mapJoystickOutput(joystickValue, OI.XBOX_JOYSTICK_DEADBAND);
        
        if (Math.abs(output) > 0) {
            Elevator.getInstance().getMaster().set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF + Math.signum(output) * Elevator.kS);
            lastSetpoint = Elevator.getInstance().getMaster().getSelectedSensorPosition() + (int)(Elevator.getInstance().getMaster().getSelectedSensorVelocity() * LAG_COMPENSATION);
            shouldHold = true;
        }
        else if (shouldHold) {
            if (lastSetpoint > Elevator.UPPER_SOFT_LIMIT)
                lastSetpoint = Elevator.SAFE_UPPER_LIMIT;
            else if (lastSetpoint < 0)
                lastSetpoint = 0;
            Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, lastSetpoint, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF + Elevator.kS);
        }

        SmartDashboard.putNumber("Elevator Error", Elevator.getInstance().getMaster().getClosedLoopError());
        SmartDashboard.putString("Elevator Control Mode", Elevator.getInstance().getMaster().getControlMode().toString());
    }

    @Override
    protected void interrupted() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}