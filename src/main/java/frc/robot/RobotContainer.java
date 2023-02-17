// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainSubsystem;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WheelSubsystem;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final RobotBase robot;

    private CANSparkMax backRightAngleMotor = new CANSparkMax(DrivetrainSubsystem.BACK_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backRightSpeedMotor = new CANSparkMax(DrivetrainSubsystem.BACK_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backRightEncoder = new AnalogEncoder(DrivetrainSubsystem.BACK_RIGHT_ENCODER);

    private CANSparkMax frontRightAngleMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontRightSpeedMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontRightEncoder = new AnalogEncoder(DrivetrainSubsystem.FRONT_RIGHT_ENCODER);

    private CANSparkMax backLeftAngleMotor = new CANSparkMax(DrivetrainSubsystem.BACK_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backLeftSpeedMotor = new CANSparkMax(DrivetrainSubsystem.BACK_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backLeftEncoder = new AnalogEncoder(DrivetrainSubsystem.BACK_LEFT_ENCODER);

    private CANSparkMax frontLeftAngleMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontLeftSpeedMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontLeftEncoder = new AnalogEncoder(DrivetrainSubsystem.FRONT_LEFT_ENCODER);


    private WheelSubsystem backRightWheel = new WheelSubsystem (
        backRightAngleMotor, backRightSpeedMotor, backRightEncoder,
        DrivetrainSubsystem.m_backRightLocation);
    public WheelSubsystem backLeftWheel = new WheelSubsystem (
      backLeftAngleMotor, backLeftSpeedMotor, backLeftEncoder,
      DrivetrainSubsystem.m_backLeftLocation);
    private WheelSubsystem frontRightWheel = new WheelSubsystem (
      frontRightAngleMotor, frontRightSpeedMotor, frontRightEncoder,
      DrivetrainSubsystem.m_frontRightLocation);
    private WheelSubsystem frontLeftWheel = new WheelSubsystem (
      frontLeftAngleMotor, frontLeftSpeedMotor, frontLeftEncoder,
      DrivetrainSubsystem.m_frontLeftLocation);
    

    Gyro gyro = new AHRS(SPI.Port.kMXP);

    private SwerveModulePosition[] positions = new SwerveModulePosition[] {
      frontLeftWheel.getSwerveModulePosition(),
      frontRightWheel.getSwerveModulePosition(),
      backLeftWheel.getSwerveModulePosition(),
      backRightWheel.getSwerveModulePosition()
    };

    private PWMTalonSRX intakeMotor = new PWMTalonSRX(IntakeSubsystemConstants.TALON_ID);
    Faults _faults = new Faults(); /* temp to fill with latest faults */
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeMotor);



    private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(DrivetrainSubsystem.swerveKinematics, gyro.getRotation2d(), positions);
    
    private SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(
        backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
        DrivetrainSubsystem.swerveKinematics);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(RobotBase robot) {
      this.robot = robot;

      // Configure the button bindings
      configureButtonBindings();
      configureObjects();
    }

    public void configureObjects() {
      // frontLeftAngleMotor.setInverted(true);
      // backRightAngleMotor.setInverted(true);

      frontLeftAngleMotor.setIdleMode(IdleMode.kBrake);
      frontRightAngleMotor.setIdleMode(IdleMode.kBrake);
      backLeftAngleMotor.setIdleMode(IdleMode.kBrake);
      backRightAngleMotor.setIdleMode(IdleMode.kBrake);

      frontLeftSpeedMotor.setIdleMode(IdleMode.kBrake);
      frontRightSpeedMotor.setIdleMode(IdleMode.kBrake);
      backLeftSpeedMotor.setIdleMode(IdleMode.kBrake);
      backRightSpeedMotor.setIdleMode(IdleMode.kBrake);

      // frontLeftAngleMotor.setInverted(true);
      // frontRightAngleMotor.setInverted(true);
      // backLeftAngleMotor.setInverted(true);
      // backRightAngleMotor.setInverted(false);

      frontRightSpeedMotor.setInverted(true);
      backRightSpeedMotor.setInverted(false);

      backRightEncoder.setPositionOffset(DrivetrainSubsystem.BACK_RIGHT_ENCODER_OFFSET);
      backLeftEncoder.setPositionOffset(DrivetrainSubsystem.BACK_LEFT_ENCODER_OFFSET);
      frontRightEncoder.setPositionOffset(DrivetrainSubsystem.FRONT_RIGHT_ENCODER_OFFSET);
      frontLeftEncoder.setPositionOffset(DrivetrainSubsystem.FRONT_LEFT_ENCODER_OFFSET);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      Joystick leftJoystick = new Joystick(0);
      Joystick rightJoystick = new Joystick(1);
      Joystick altJoystick = new Joystick(2);


      JoystickButton intakeInButton = new JoystickButton(altJoystick, 6);

      JoystickButton intakeOutButton = new JoystickButton(altJoystick, 5);


      swerveDrive.setDefaultCommand(new RunCommand(
          () -> {
            if (robot.isTeleopEnabled()) {
              swerveDrive.drive(
                  applyDeadband(-leftJoystick.getX() * DrivetrainSubsystem.MOVEMENT_SPEED, DrivetrainSubsystem.DRIFT_DEADBAND),
                  applyDeadband(leftJoystick.getY() * DrivetrainSubsystem.MOVEMENT_SPEED,DrivetrainSubsystem.DRIFT_DEADBAND),
                  applyDeadband(-rightJoystick.getX() * DrivetrainSubsystem.ROTATION_SPEED, DrivetrainSubsystem.ROTATION_DEADBAND));
            } else {
              swerveDrive.drive(0, 0, 0);
            }
          },
          swerveDrive));

      intakeInButton.whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeMotor(0.25), intakeSubsystem));
      intakeInButton.onFalse(new InstantCommand(() -> intakeSubsystem.setIntakeMotor(0)));

      intakeOutButton.whileTrue(new RunCommand(() -> intakeSubsystem.setIntakeMotor(-0.5), intakeSubsystem));
      intakeOutButton.onFalse(new InstantCommand(() -> intakeSubsystem.setIntakeMotor(0)));

      // fieldCentricButton.onTrue(new InstantCommand(
      //     () -> {
      //       System.out.println("FIELD CENTRIC TOGGLED");
      //       swerveDrive.toggleFieldCentric();
      //     }, swerveDrive));

      // resetEncoderButton.whileTrue(new RunCommand(
      //   () -> {
      //     System.out.println("RESET");
      //     swerveDrive.resetEncoders();
      // }, swerveDrive));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public double applyDeadband(double val, double deadband){
      if (Math.abs(val) < deadband) return 0;
      else return val;
    }

    public void coastDrive() {
      frontLeftAngleMotor.setIdleMode(IdleMode.kCoast);
      frontRightAngleMotor.setIdleMode(IdleMode.kCoast);
      backLeftAngleMotor.setIdleMode(IdleMode.kCoast);
      backRightAngleMotor.setIdleMode(IdleMode.kCoast);


      frontLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
      frontRightSpeedMotor.setIdleMode(IdleMode.kCoast);
      backLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
      backRightSpeedMotor.setIdleMode(IdleMode.kCoast);
    }
}
