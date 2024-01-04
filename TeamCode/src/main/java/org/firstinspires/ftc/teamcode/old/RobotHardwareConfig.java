package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Deprecated
public final class RobotHardwareConfig {
    public static class Drive{
        public static final String FL_STRING = "Front_Left";
        public static final String FR_STRING = "Front_Right";
        public static final String BL_STRING = "Back_Left";
        public static final String BR_STRING = "Back_Right";

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
        public static final String LIFT_STRING = "Slide_Motor";

        public static DcMotorSimple.Direction LIFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static DcMotor.RunMode DEFAULT_RUNMODE = DcMotor.RunMode.RUN_TO_POSITION;
        public static DcMotor.ZeroPowerBehavior DEFAULT_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

    }



    public static class Claw{
        public static final String CLAW_STRING = "Claw_Servo";

        public static Servo.Direction CLAW_DIRECTION = Servo.Direction.FORWARD;

    }
}
