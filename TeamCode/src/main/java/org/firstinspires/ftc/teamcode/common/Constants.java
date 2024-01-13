package org.firstinspires.ftc.teamcode.common;

public class Constants {
    private Constants(){}

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

    public static class Lift{
        private Lift(){}
        public static double RESET_SPEED = -0.2;

        public static int GROUND_POSITION = 0;
    }

    public static class Arm{
        private Arm(){}

        public static double INTAKE_POSITION = 0;
        public static double OUTTAKE_POSITION = 1;
        public static double OUTTAKE_ALT_POSITION = 1;

    }

    public static class EndEffector {
        private EndEffector(){}
        public static double
            LEFT_OPEN = 0,
            LEFT_CLOSE = 1;

        public static double
            RIGHT_OPEN = 0,
            RIGHT_CLOSE = 1;

        public static double
            WRIST_INTAKE = 0.2,
            WRIST_STOW = 0.3,
            WRIST_ALT_DEPOSIT = 0.4;
    }

    //XPS gang
}
