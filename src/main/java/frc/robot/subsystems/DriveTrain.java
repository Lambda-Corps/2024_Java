// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  // Members for functionality
  private TalonFX m_left_leader, m_right_leader;
  private VoltageOut m_left_volts, m_right_volts;
  AHRS m_gyro;
  DifferentialDriveOdometry m_Odometry;
  DifferentialDriveKinematics m_kinematics;
  Field2d m_Field2d;

  // Sim Stuff
  DifferentialDrivetrainSim m_driveSim;
  int m_gyroSim;
  TalonFXSimState m_leftSim, m_rightSim;
  SimDouble m_angle;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    m_left_leader = configureLeftSide();
    m_right_leader = configureRightSide();

    // Set status signals to be coherent
    m_left_leader.getPosition().setUpdateFrequency(100);
    m_right_leader.getPosition().setUpdateFrequency(100);

    m_left_volts = new VoltageOut(0);
    m_right_volts = new VoltageOut(0);
    
    m_gyro = new AHRS(SPI.Port.kMXP);

    if(Robot.isSimulation()){
      m_rightSim = m_right_leader.getSimState();

      m_gyroSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
      m_angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_gyroSim, "Yaw"));

      //self._system = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
      m_driveSim = new DifferentialDrivetrainSim(
					DCMotor.getFalcon500(2), // 2 Falcon 500s on each side of the drivetrain.
					Constants.DT_GEAR_RATIO, // Standard AndyMark Gearing reduction.
					2.1, // MOI of 2.1 kg m^2 (from CAD model).
					Constants.ROBOT_MASS, // Mass of the robot is 26.5 kg.
					Units.inchesToMeters(Constants.DT_WHEEL_RADIUS_INCHES), // Robot uses 3" radius (6" diameter) wheels.
					Constants.DT_TRACKWIDTH_METERS, // Distance between wheels is _ meters.

					/*
					 * The standard deviations for measurement noise:
					 * x and y: 0.001 m
					 * heading: 0.001 rad
					 * l and r velocity: 0.1 m/s
					 * l and r position: 0.005 m
					 */
					null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this
							// line to add measurement noise.
			);
    }

    // Setup the odometry
    m_Field2d = new Field2d();
    SmartDashboard.putData((m_Field2d));
		m_Odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);

    m_kinematics = new DifferentialDriveKinematics(Constants.DT_TRACKWIDTH_METERS);
    // Setup the AutoConfiguration
    AutoBuilder.configureRamsete(
      this::getPose, 
      this::resetPose, 
      this::getCurrentSpeeds, 
      this::driveChassisSpeeds, 
      new ReplanningConfig(), 
      this::shouldFlippath, 
      this
    );
  }

  private TalonFX configureLeftSide(){
    TalonFX talon = new TalonFX(1);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    if (Robot.isReal()){
      config.Feedback.SensorToMechanismRatio = Constants.DT_GEAR_RATIO;
    }
    StatusCode returnCode;
    do{
      returnCode = talon.getConfigurator().apply(config);
    } while (!returnCode.isOK());
    

    m_leftSim = talon.getSimState();
    m_leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
    return talon;
  }

  private TalonFX configureRightSide(){
    TalonFX talon = new TalonFX(2);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    if (Robot.isReal()){
      config.Feedback.SensorToMechanismRatio = Constants.DT_GEAR_RATIO;
    }


    StatusCode returnCode;
    do{
      returnCode = talon.getConfigurator().apply(config);
    } while (!returnCode.isOK());
    
    // Setup sim
    m_rightSim = talon.getSimState();
    m_rightSim.Orientation = ChassisReference.Clockwise_Positive;
    return talon;
  }

  public void drive_teleop(double forward, double turn){
    forward = MathUtil.applyDeadband(forward, .01);
		turn = MathUtil.applyDeadband(turn, .01);
    forward = MathUtil.clamp(forward, -1.0, 1.0);
    turn = MathUtil.clamp(turn, -1.0, 1.0);
    
    var speeds = DifferentialDrive.curvatureDriveIK(forward, turn, true);
    SmartDashboard.putNumber(("Left"), speeds.left);
    SmartDashboard.putNumber("Right", speeds.right);

    m_left_volts.Output = speeds.left * 12.0;
    m_right_volts.Output = speeds.right * 12.0;

    m_left_leader.setControl(m_left_volts);
    m_right_leader.setControl(m_right_volts);
  }

  public void drive_volts(double left, double right) {
    m_left_volts.Output = left;
    m_right_volts.Output = right;

    m_left_leader.setControl(m_left_volts);
    m_right_leader.setControl(m_right_volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(m_gyro.getRotation2d(),
      rotationsToMeters(m_left_leader.getPosition().getValue()),
      rotationsToMeters(m_right_leader.getPosition().getValue()) 
    );
		m_Field2d.setRobotPose(m_Odometry.getPoseMeters());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_driveSim.setInputs(m_leftSim.getMotorVoltage(), m_rightSim.getMotorVoltage());

    m_driveSim.update(Constants.ROBOT_PERIOD_MS);

    m_leftSim.setRawRotorPosition(metersToRotations(m_driveSim.getLeftPositionMeters()));
    m_leftSim.setRotorVelocity(metersToRotations(m_driveSim.getLeftVelocityMetersPerSecond()));
    m_rightSim.setRawRotorPosition(metersToRotations(m_driveSim.getRightPositionMeters()));
    m_rightSim.setRotorVelocity(metersToRotations(m_driveSim.getRightVelocityMetersPerSecond()));

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
		SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
		angle.set(-m_driveSim.getHeading().getDegrees());

  }

  private double rotationsToMeters(double rotations){
    return rotations * Constants.DT_WHEEL_CIRCUMFERENCE_METERS;
  }

  private double metersToRotations(double meters){
    //                             1
    // rot_per_meter = -----------------------------
    //                 pi * wheel_diameter (meters)
    double rots_per_meter = 1.0 / Constants.DT_WHEEL_CIRCUMFERENCE_METERS;
    return rots_per_meter * meters;
  }

  public boolean shouldFlippath(){
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    
    return false;
  }

  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose){
      m_left_leader.setPosition(metersToRotations(pose.getX()));
      m_right_leader.setPosition(metersToRotations(pose.getY()));
      m_Odometry.resetPosition(
        pose.getRotation(),
        m_left_leader.getPosition().getValue(), 
        m_right_leader.getPosition().getValue(), 
        pose);
    //m_Odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return m_kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(
        m_left_leader.getVelocity().getValue(),
        m_right_leader.getVelocity().getValue()
      )
    );
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds){
    DifferentialDriveWheelSpeeds diffSpeeds = m_kinematics.toWheelSpeeds(speeds);

    double voltage = RobotController.getBatteryVoltage();

    drive_volts(diffSpeeds.leftMetersPerSecond / voltage, diffSpeeds.rightMetersPerSecond / voltage);
  }

  public Command followRamseteCommand(){
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, 0);
    DifferentialDriveVoltageConstraint constraint = new DifferentialDriveVoltageConstraint(ff, m_kinematics, 10);
    TrajectoryConfig config = new TrajectoryConfig(3, 3)
      .setKinematics(m_kinematics)
      .addConstraint(constraint);

    Pose2d start = new Pose2d(1.34, 5.55, new Rotation2d(Math.PI));
    Pose2d end = new Pose2d(2.27, 5.55, new Rotation2d(Math.PI));

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(start, end), config);
    
    RamseteCommand cmd = new RamseteCommand(trajectory, this::getPose, new RamseteController(), m_kinematics, this::drive_volts, this);
    
    return Commands.runOnce(() -> this.resetPose(trajectory.getInitialPose()))
      .andThen(cmd)
      .andThen(Commands.runOnce(()-> this.drive_volts(0, 0)));
  }

  public Command followPathPlannerCommand(String name){
    PathPlannerPath path = PathPlannerPath.fromPathFile("SubRing2");

    return Commands.runOnce(()-> this.resetPose(path.getStartingDifferentialPose()))
      .andThen(new FollowPathRamsete(
        path, 
        this::getPose, 
        this::getCurrentSpeeds, 
        this::driveChassisSpeeds, 
        new ReplanningConfig(), 
        this::shouldFlippath, 
        this
      )).andThen(Commands.runOnce(()-> this.drive_volts(0, 0)));
  }
}
