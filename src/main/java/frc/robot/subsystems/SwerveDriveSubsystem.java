package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase{

    private WheelSubsystem backRight;
    private WheelSubsystem backLeft;
    private WheelSubsystem frontRight;
    private WheelSubsystem frontLeft;
    
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry backRightAngleEncoder = table.getEntry("backRightAngleEncoder");
    private final NetworkTableEntry backLeftAngleEncoder = table.getEntry("backLeftAngleEncoder");
    private final NetworkTableEntry frontRightAngleEncoder = table.getEntry("frontRightAngleEncoder");
    private final NetworkTableEntry frontLeftAngleEncoder = table.getEntry("frontLeftAngleEncoder");

    private final NetworkTableEntry backRightEncoderOutput = table.getEntry("backRightEncoderOutput");
    private final NetworkTableEntry backLeftEncoderOutput = table.getEntry("backLeftEncoderOutput");
    private final NetworkTableEntry frontRightEncoderOutput = table.getEntry("frontRightEncoderOutput");
    private final NetworkTableEntry frontLeftEncoderOutput = table.getEntry("frontLeftEncoderOutput");
    private final NetworkTableEntry isFieldCentric = table.getEntry("isFieldCentric");

    private final NetworkTable odometryTable = ntInstance.getTable("/common/Odometry");
    private final NetworkTableEntry odometryPose = odometryTable.getEntry("odometryPose");



    private SwerveDriveKinematics kinematics;
    
    public boolean fieldCentric = false;

    public SwerveDriveSubsystem (WheelSubsystem backRight, WheelSubsystem backLeft, WheelSubsystem frontRight, WheelSubsystem frontLeft, SwerveDriveKinematics kinematics) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;

        this.kinematics = kinematics;

        this.isFieldCentric.setBoolean(fieldCentric);
    }


    public void drive (double x, double y, double rot) {
        

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rot);
        // ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        frontLeft.drive(frontLeftState);
        frontRight.drive(frontRightState);
        backLeft.drive(backLeftState);
        backRight.drive(backRightState);

    }

    public void drive (SwerveModuleState[] moduleStates) {
        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        frontLeft.drive(frontLeftState);
        frontRight.drive(frontRightState);
        backLeft.drive(backLeftState);
        backRight.drive(backRightState);

    }
    

    public void resetEncoders() {
        frontLeft.getEncoder().reset();
        frontLeft.getAngleMotor().getEncoder().setPosition(0);

        frontRight.getEncoder().reset();
        frontRight.getAngleMotor().getEncoder().setPosition(0);

        backLeft.getEncoder().reset();
        backLeft.getAngleMotor().getEncoder().setPosition(0);

        backRight.getEncoder().reset();
        backRight.getAngleMotor().getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        

        // backLeftAngleEncoder.setDouble(backLeft.getDistanceDriven());
        // backRightAngleEncoder.setDouble(backRight.getDistanceDriven());
        // frontLeftAngleEncoder.setDouble(frontLeft.getDistanceDriven());
        // frontRightAngleEncoder.setDouble(frontRight.getDistanceDriven());

        backLeftAngleEncoder.setDouble(backLeft.getEncoderPosition());
        backRightAngleEncoder.setDouble(backRight.getEncoderPosition());
        frontLeftAngleEncoder.setDouble(frontLeft.getEncoderPosition());
        frontRightAngleEncoder.setDouble(frontRight.getEncoderPosition());

        // backLeftEncoderOutput.setDouble(backLeft.getAngleVoltage());
        // backRightEncoderOutput.setDouble(backRight.getAngleVoltage());
        // frontLeftEncoderOutput.setDouble(frontLeft.getAngleVoltage());
        // frontRightEncoderOutput.setDouble(frontRight.getAngleVoltage());
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
        isFieldCentric.setBoolean(fieldCentric);
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
          };
    }

}
