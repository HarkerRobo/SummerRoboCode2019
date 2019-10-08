package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.climber.ExtendClimbers;

public class Climber extends Subsystem {

    private static Climber instance;
    private TalonSRX climberTalon;
    private VictorSPX climberVictor;
    public static final int LOWEST_POS = 0;
    public static final int HAB_DISTANCE = 0;
    public Climber(){
        climberTalon = new TalonSRX(RobotMap.CAN_IDS.CLIMBER_MASTER);
        climberVictor = new VictorSPX(RobotMap.CAN_IDS.CLIMBER_FOLLOWER);
    }    
    @Override
    protected void initDefaultCommand() {
       setDefaultCommand(new ExtendClimbers());
    }
    public static Climber getInstance() {
        if(instance==null){
            instance = new Climber();
        }
        return instance;
    }
    public void talonInit(){

    }
    public void followMasters(){
        climberVictor.follow(climberTalon);
    }

}