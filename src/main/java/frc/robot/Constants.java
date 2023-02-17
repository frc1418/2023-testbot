// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static class DrivetrainSubsystem {

        public static final double DRIFT_DEADBAND = 0.05;
        public static final double ROTATION_DEADBAND = 0.002;

        public static final double WIDTH = 22.5;
        public static final double LENGTH = 22.5;

        public final static int BACK_RIGHT_SPEED_ID = 4;
        public final static int BACK_RIGHT_ANGLE_ID = 3;
        public final static int BACK_RIGHT_ENCODER = 0;
        public final static double BACK_RIGHT_ENCODER_OFFSET = 0.241;

        public final static int BACK_LEFT_SPEED_ID = 2;
        public final static int BACK_LEFT_ANGLE_ID = 1;
        public final static int BACK_LEFT_ENCODER = 1;
        public final static double BACK_LEFT_ENCODER_OFFSET = 0.015;

        public final static int FRONT_RIGHT_SPEED_ID = 6;
        public final static int FRONT_RIGHT_ANGLE_ID = 5;
        public final static int FRONT_RIGHT_ENCODER = 2;
        public final static double FRONT_RIGHT_ENCODER_OFFSET = 0.734;

        public final static int FRONT_LEFT_SPEED_ID = 8;
        public final static int FRONT_LEFT_ANGLE_ID = 7;
        public final static int FRONT_LEFT_ENCODER = 3;
        public final static double FRONT_LEFT_ENCODER_OFFSET = 0.705;

        public final static Translation2d m_frontLeftLocation = new Translation2d(9.4, 9.4);
        public final static Translation2d m_frontRightLocation = new Translation2d(9.4, -9.4);
        public final static Translation2d m_backLeftLocation = new Translation2d(-9.4, 9.4);
        public final static Translation2d m_backRightLocation = new Translation2d(-9.4, -9.4);

        public final static double MOVEMENT_SPEED = 0.25;
        public final static double ROTATION_SPEED = 0.025;

        public final static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          m_frontLeftLocation,
          m_frontRightLocation,
          m_backLeftLocation,
          m_backRightLocation);

    }
    public final static class IntakeSubsystemConstants {
        public final static int TALON_ID = 0;
    }
}
