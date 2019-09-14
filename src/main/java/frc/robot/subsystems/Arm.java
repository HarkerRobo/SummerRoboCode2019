package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.arm.SpinBallIntakeManual;

/**
 * The Arm subsystem, which includes both the Rollers and the Solenoid
 * 
 * @author Jatin Kohli
 * 
 * @since 6/14/19
 */
public class Arm extends Subsystem{

    private static Arm instance;

    private DoubleSolenoid solenoid;
    private VictorSPX victor; //For Practice Bot
    private CANSparkMax spark; //For Comp Bot

    public static final DoubleSolenoid.Value IN = Value.kReverse;
    public static final DoubleSolenoid.Value OUT = Value.kForward;
    
    public Arm() {
        solenoid = new DoubleSolenoid(RobotMap.CAN_IDS.ARM_FORWARD_CHANNEL, RobotMap.CAN_IDS.ARM_REVERSE_CHANNEL);
        victor = new VictorSPX(RobotMap.CAN_IDS.BALL_INTAKE_VICTOR);
        spark = new CANSparkMax(RobotMap.CAN_IDS.BALL_INTAKE_SPARK, MotorType.kBrushless);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SpinBallIntakeManual());
    }

    public DoubleSolenoid getSolenoid() {
        return solenoid;
    }

    public VictorSPX getVictor() {
        return victor;
    }

    public CANSparkMax getSpark() {
        return spark;
    }

    public static Arm getInstance()
    {
        if (instance == null)
            instance = new Arm();
        return instance;
    }
}