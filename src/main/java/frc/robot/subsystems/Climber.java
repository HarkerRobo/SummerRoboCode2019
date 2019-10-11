package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.climber.ExtendClimbers;

/**
 * This represents the climber on our robot.
 *
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Jatin Kohli
 * @author Angela Jia
 * @author Arnav Gupta
 */
public class Climber extends Subsystem {

    private static Climber climber;

    private TalonSRX climberTalon;
    private VictorSPX climberVictor;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;


    public static final int LOWEST_POS = 0;
    public static final int HAB_DISTANCE = 0;
    public static final boolean MASTER_INVERTED = false;
    public static final boolean VICTOR_INVERTED = false;
    private static final boolean SENSOR_PHASE = true;

    public static final double PUSHED_DOWN_POSITION = 0.0; 

    public Climber(){
        climberTalon = new TalonSRX(RobotMap.CAN_IDS.CLIMBER_MASTER);
        climberVictor = new VictorSPX(RobotMap.CAN_IDS.CLIMBER_FOLLOWER);
        talonInit();
    }    

    @Override
    protected void initDefaultCommand() {
       //setDefaultCommand(new ExtendClimbers()); 
    }

    public static Climber getInstance() {
        if(climber == null){
            climber = new Climber();
        }
        return climber;
    }
    
    public void talonInit(){
        climberTalon.configFactoryDefault();
        climberVictor.configFactoryDefault();
        followMasters();

        climberTalon.setInverted(MASTER_INVERTED);
        climberVictor.setInverted(VICTOR_INVERTED);

        climberTalon.setNeutralMode(NeutralMode.Brake);
        climberTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        climberTalon.setSensorPhase(SENSOR_PHASE);
        climberTalon.setSelectedSensorPosition(0);
        
        climberTalon.config_kP(RobotMap.PRIMARY_PID_INDEX, kP);
        climberTalon.config_kI(RobotMap.PRIMARY_PID_INDEX, kI);
        climberTalon.config_kD(RobotMap.PRIMARY_PID_INDEX, kD);
        climberTalon.config_kF(RobotMap.PRIMARY_PID_INDEX, kF);
    }
    
    public void followMasters(){
        climberVictor.follow(climberTalon);
    }

    public TalonSRX getMaster() {
        return climberTalon;
    }

    public VictorSPX getFollower() {
        return climberVictor;
    }

}