package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.OI.DemoMode;
import frc.robot.subsystems.Arm;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

/**
 * Spins the BallIntake (Arm Rollers) using input from the Driver Gamepad's Triggers
 * 
 * @author Jatin Kohli
 * 
 * @since 6/17/19
 */
public class SpinBallIntakeManual extends IndefiniteCommand {

    private static final double VICTOR_SPEED_MULTIPLIER = 0.6;
    private static final double SPARK_SPEED_MULTIPLIER = 0.25;

    public SpinBallIntakeManual() {
        requires(Arm.getInstance());
    }

    @Override
    protected void execute() {
        double leftTrigger = OI.getInstance().getDriverGamepad().getLeftTrigger();
        double rightTrigger = OI.getInstance().getDriverGamepad().getRightTrigger();
        leftTrigger = MathUtil.mapJoystickOutput(-leftTrigger, OI.XBOX_TRIGGER_DEADBAND);
        rightTrigger = MathUtil.mapJoystickOutput(rightTrigger, OI.XBOX_TRIGGER_DEADBAND);

        boolean a = OI.getInstance().getDriverGamepad().getButtonAState();
        boolean x = OI.getInstance().getDriverGamepad().getButtonXState();
        double output;

        if (OI.mode == DemoMode.SAFE) {
            output = Math.abs(leftTrigger) > Math.abs(rightTrigger) ? leftTrigger : rightTrigger;
        } else {
            output = (a ? 1 : (x ? -1 : 0));
        }
        SmartDashboard.putBoolean("A", OI.getInstance().getDriverGamepad().getButtonAState());
        // Arm.getInstance().getRollers().set(ControlMode.PercentOutput, VICTOR_SPEED_MULTIPLIER * output);
        Arm.getInstance().getRollers().set(SPARK_SPEED_MULTIPLIER * output);
    }
}