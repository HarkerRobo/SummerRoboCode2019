package frc.robot.subsystems;

import harkerrobolib.subsystems.HSDrivetrain;
import harkerrobolib.wrappers.HSTalon;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.RobotMap;

/**
 * Represents the drivetrain on the robot.
 * 
 * @author Finn Frankis
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * @since 6/14/19
 */
public class Drivetrain extends HSDrivetrain {
   private static Drivetrain instance;

   private Drivetrain() {
      super(new HSTalon(RobotMap.CAN_IDS.DT_LEFT_MASTER), new HSTalon(RobotMap.CAN_IDS.DT_RIGHT_MASTER),
            new VictorSPX(RobotMap.CAN_IDS.DT_RIGHT_FOLLOWER), new VictorSPX(RobotMap.CAN_IDS.DT_LEFT_FOLLOWER));
   }

   public void initDefaultCommand() {
      // setDefaultCommand();
   }
   
   public static Drivetrain getInstance() {
       if (instance == null) {
          instance = new Drivetrain();
       }
       return instance;
   }
}