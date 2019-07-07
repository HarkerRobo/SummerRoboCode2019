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

    private static final double SPEED_MULTIPLIER = 0.3;
    private static final double LAG_COMPENSATION = 0.5;
    private int lastSetpoint;

    public MoveElevatorPercentOutput() {
        requires(Elevator.getInstance());
    }
    
    @Override
    protected void initialize() {
        lastSetpoint = Elevator.getInstance().getMaster().getSelectedSensorPosition();
    }

    @Override
    protected void execute() {
        double output = SPEED_MULTIPLIER * MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightY(), OI.XBOX_JOYSTICK_DEADBAND);
        SmartDashboard.putNumber("El Position", Elevator.getInstance().getMaster().getSelectedSensorPosition());

        if (Math.abs(output) > 0) {
            Elevator.getInstance().getMaster().set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF + Elevator.kS);
            lastSetpoint = Elevator.getInstance().getMaster().getSelectedSensorPosition() + (int)(Elevator.getInstance().getMaster().getSelectedSensorVelocity() * LAG_COMPENSATION);
        }
        else {
            if (lastSetpoint > Elevator.UPPER_SOFT_LIMIT)
                lastSetpoint = Elevator.SAFE_UPPER_LIMIT;
            // else if (lastSetpoint < Elevator.LOWER_SOFT_LIMIT)
            //     lastSetpoint = Elevator.SAFE_LOWER_LIMIT;
            Elevator.getInstance().getMaster().set(ControlMode.MotionMagic, lastSetpoint, DemandType.ArbitraryFeedForward, Elevator.GRAVITY_FF + Elevator.kS);
        }
    }

    @Override
    protected void interrupted() {
        Elevator.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}