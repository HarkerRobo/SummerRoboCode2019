package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SpinBallIntake;

public class Arm extends Subsystem{

    private static Arm instance;

    private DoubleSolenoid solenoid;
    private VictorSPX rollers;

    public Arm() {
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.ARM_FORWARD_CHANNEL, RobotMap.CAN_IDS.ARM_REVERSE_CHANNEL);
        rollers = new VictorSPX(RobotMap.CAN_IDS.BALL_INTAKE_MASTER_VICTOR);
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public VictorSPX getRollers() {
        return rollers;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SpinBallIntake());
    }

    public static Arm getInstance()
    {
        if (instance == null)
            instance = new Arm();
        return instance;
    }
}