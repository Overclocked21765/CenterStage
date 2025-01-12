package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    private Constants(){}

    @Config
    public static class Drive{
        private Drive(){}
        public static double ROTATION_CONSTANT = 0.75;//how fast will the robot turn (as a percentage)
        public static final int MECANUM_MOTOR_NUMBER = 4; //unused, was intended for ftclib
        public static final int MECANUM_FRONT_LEFT_MOTOR = 0; //the following four variables are for arrays
        public static final int MECANUM_FRONT_RIGHT_MOTOR = 1;
        public static final int MECANUM_BACK_LEFT_MOTOR = 2;
        public static final int MECANUM_BACK_RIGHT_MOTOR = 3;
        public static double DRIVE_POWER_MODIFIER = 0.95; //how fast will the robot drive (as a percentage)
        public static double SLOW_DRIVE_MODIFIER = 0.3; //a requested slower speed
    }

    @Config
    public static class Lift{
        private Lift(){}
        public static double RESET_SPEED = -0.1;

        public static int GROUND_POSITION = 0;
        public static int MAX = 2000;
        public static int TICK_PER_REQ = 250;
    }

    @Config
    public static class Arm{
        private Arm(){}

        public static double INTAKE_POSITION_LEFT = 0.08;
        public static double INTAKE_POSITION_RIGHT = 0.08;
        public static double OUTTAKE_POSITION_LEFT = 1;
        public static double OUTTAKE_POSITION_RIGHT = 1;
        public static double OUTTAKE_ALT_POSITION_LEFT = 1;
        public static double OUTTAKE_ALT_POSITION_RIGHT = 1;

    }

    @Config
    public static class EndEffector {
        private EndEffector(){}
        public static double
            LEFT_OPEN = 0.55,
            LEFT_CLOSE = 0.8;

        public static double
            RIGHT_OPEN = 0.5,
            RIGHT_CLOSE = 0.25;

        public static double
            WRIST_INTAKE = 0.2,
            WRIST_STOW = 0.1,
            WRIST_DEPOSIT = 0.01,
            WRIST_ALT_DEPOSIT = 0.55,
            WRIST_PASSTRHOUGH = 0.43;
    }

    //XPS gang
}
