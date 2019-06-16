package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.RobotMap;
import frc.robot.commands.DriveWithPercentOutput;
import frc.robot.commands.DriveWithVelocity;
import harkerrobolib.subsystems.HSDrivetrain;
import harkerrobolib.wrappers.HSTalon;

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

   public static final int WHEEL_DIAMETER = 4;

   private static final boolean LEFT_MASTER_INVERTED = true;
   private static final boolean LEFT_VICTOR_INVERTED = true;
   private static final boolean RIGHT_MASTER_INVERTED = false;
   private static final boolean RIGHT_VICTOR_INVERTED = false;

   private static final int VELOCITY_SLOT = 0;
   private static final double VELOCITY_LEFT_kF = 0.23;
   private static final double VELOCITY_LEFT_kP = 0.75;
   private static final double VELOCITY_LEFT_kI = 0;
   private static final double VELOCITY_LEFT_kD = 0;
   private static final double VELOCITY_RIGHT_kF = 0.275;
   private static final double VELOCITY_RIGHT_kP = 0.75;
   private static final double VELOCITY_RIGHT_kI = 0;
   private static final double VELOCITY_RIGHT_kD = 0;
   private static final double VELOCITY_RAMP_RATE = 0.2;

   private static final int POSITION_SLOT = 1;
   private static final double POSITION_LEFT_kP = 0.1;
   private static final double POSITION_LEFT_kI = 0;
   private static final double POSITION_LEFT_kD = 0;
   private static final double POSITION_RIGHT_kP = 0.1;
   private static final double POSITION_RIGHT_kI = 0;
   private static final double POSITION_RIGHT_kD = 0;
   private static final double POSITION_RAMP_RATE = 0.2;

   private Drivetrain() {
      super(new HSTalon(RobotMap.CAN_IDS.DT_LEFT_MASTER), new HSTalon(RobotMap.CAN_IDS.DT_RIGHT_MASTER),
            new VictorSPX(RobotMap.CAN_IDS.DT_LEFT_FOLLOWER), new VictorSPX(RobotMap.CAN_IDS.DT_RIGHT_FOLLOWER));
   }

   public void talonInit() {
      resetMasters();
      invertTalons(LEFT_MASTER_INVERTED, RIGHT_MASTER_INVERTED, LEFT_VICTOR_INVERTED, RIGHT_VICTOR_INVERTED);
      setNeutralMode(NeutralMode.Brake);
      configBothFeedbackSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
   }

   public void initDefaultCommand() {
      setDefaultCommand(new DriveWithVelocity());
   }

   public void setupVelocityPID() {
      getLeftMaster().config_kF(VELOCITY_SLOT, VELOCITY_LEFT_kF);
      getLeftMaster().config_kP(VELOCITY_SLOT, VELOCITY_LEFT_kP);
      getLeftMaster().config_kI(VELOCITY_SLOT, VELOCITY_LEFT_kI);
      getLeftMaster().config_kD(VELOCITY_SLOT, VELOCITY_LEFT_kD);

      getRightMaster().config_kF(VELOCITY_SLOT, VELOCITY_RIGHT_kF);
      getRightMaster().config_kP(VELOCITY_SLOT, VELOCITY_RIGHT_kP);
      getRightMaster().config_kI(VELOCITY_SLOT, VELOCITY_RIGHT_kI);
      getRightMaster().config_kD(VELOCITY_SLOT, VELOCITY_RIGHT_kD);
      
      getLeftMaster().selectProfileSlot(VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX);
      getRightMaster().selectProfileSlot(VELOCITY_SLOT, RobotMap.PRIMARY_PID_INDEX);
   
      getLeftMaster().configClosedloopRamp(VELOCITY_RAMP_RATE);
      getRightMaster().configClosedloopRamp(VELOCITY_RAMP_RATE);
   }

   public void setupPositionPID() {
      getLeftMaster().config_kP(POSITION_SLOT, POSITION_LEFT_kP);
      getLeftMaster().config_kI(POSITION_SLOT, POSITION_LEFT_kI);
      getLeftMaster().config_kD(POSITION_SLOT, POSITION_LEFT_kD);

      getRightMaster().config_kP(POSITION_SLOT, POSITION_RIGHT_kP);
      getRightMaster().config_kI(POSITION_SLOT, POSITION_RIGHT_kI);
      getRightMaster().config_kD(POSITION_SLOT, POSITION_RIGHT_kD);
      
      getLeftMaster().selectProfileSlot(POSITION_SLOT, RobotMap.PRIMARY_PID_INDEX);
      getRightMaster().selectProfileSlot(POSITION_SLOT, RobotMap.PRIMARY_PID_INDEX);
   
      getLeftMaster().configClosedloopRamp(POSITION_RAMP_RATE);
      getRightMaster().configClosedloopRamp(POSITION_RAMP_RATE);
   }
   
   public static Drivetrain getInstance() {
       if (instance == null) {
          instance = new Drivetrain();
       }
       return instance;
   }
}