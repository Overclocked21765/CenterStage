package org.firstinspires.ftc.teamcode.common;



public class Constants {
    public static class Drive{
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
        public static double RESET_SPEED = -0.2;

        public static int GROUND_POSITION = 0;
    }

    public static class PassThrough{
        public static final double SLIDE_SERVO_ZERO_POSITION = 0.73;
        public static final double SLIDE_SERVO_ROTATED_POSITION = 0.04; //should be 0.04 for testing its been changed
        public static final double SERVO_ROTATE_TIME = 1;
    }

    public static class Claw{
        public static final double CLAW_MIN = 0.69; //grab
        public static final double CLAW_MAX = 0.54; //release
        public static double TIME_FOR_RELEASE_CLAW = 0.3;
    }

    //Auto Constants
    public static class Auto{
        public static final double ONE_TILE_STRAFE = 1.1;
        public static final double ONE_TILE_FORWARD = 1.1;
        public static final double QUARTER_ROTATION = 1.05;
        public static final double STAY_STILL_AND_RELEASE = 2;
        public static final double ONE_SECOND = 1;
    }

    //XPS gang
}
