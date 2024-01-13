package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class RobotHardwareConfig {
    private RobotHardwareConfig(){}
    public static class Drive{
        public static final String FL_STRING = "FL";
        public static final String FR_STRING = "FR";
        public static final String BL_STRING = "BL";
        public static final String BR_STRING = "BR";

        public static final String IMU_STRING = "imu";

        public static final DcMotorSimple.Direction
            FL_DIRECTION = DcMotorSimple.Direction.REVERSE,
            FR_DIRECTION = DcMotorSimple.Direction.FORWARD,
            BL_DIRECTION = DcMotorSimple.Direction.REVERSE,
            BR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_FACING = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_FACING = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        public static DcMotor.RunMode DEFAULT_RUNMODE = DcMotor.RunMode.RUN_USING_ENCODER;
        public static DcMotor.ZeroPowerBehavior DEFAULT_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
    }

    public static class Lift{
        public static String
            LEFT_MOTOR_STRING = "Lift Left",
            RIGHT_MOTOR_STRING = "Lift Right";

        public static DcMotorSimple.Direction
            LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD,
            RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;



    }

    public static class Arm{
        public static final String LEFT_SERVO = "Arm Left";
        public static final String RIGHT_SERVO = "Arm Right";

        public static Servo.Direction ARM_LEFT_DIRECTION = Servo.Direction.FORWARD;
        public static Servo.Direction ARM_RIGHT_DIRECTION = Servo.Direction.REVERSE;
    }



    public static class EndEffector {
        public static final String CLAW_LEFT_STRING = "Claw Left";
        public static final String CLAW_RIGHT_STRING = "Claw Right";

        public static Servo.Direction CLAW_LEFT_DIRECTION = Servo.Direction.FORWARD;
        public static Servo.Direction CLAW_RIGHT_DIRECTION = Servo.Direction.REVERSE;

    }
}
