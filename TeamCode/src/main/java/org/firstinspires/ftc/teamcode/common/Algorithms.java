package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Constants.Drive.ROTATION_CONSTANT;

public class Algorithms {
    //remaps joystick from grid to circle
    public static double[] mapJoystick(double x, double y){
        double m_x = x * Math.sqrt(1 - y * y / 2);
        double m_y = y * Math.sqrt(1 - x * x / 2);
        double[] converted = {m_x, m_y};
        return converted;
    }
    //finds r (for r, theta aqquired from the joystick)
    static double inputMagnitude(double x, double y){
        double[] cords = mapJoystick(x, y);
        return (Math.hypot(cords[0], cords[1]));
    }

    //finds theta (for r, theta aquirred from the joystick)
    //essentailly a custom Math.atan2 implementation
    static double inputAngle(double x, double y){
        double[] cords = mapJoystick(x, y);
        if (cords[0] == 0){
            if (cords[1] > 0){
                return 0;
            } else if (cords[1] < 0){
                return 180;
            } else {
                return 0;
            }
        }
        double deg = (Math.toDegrees(Math.atan(cords[1] / cords[0])) - 90);
        if (x < 0){
            return deg + 180;
        }
        return deg;
    }

    //calculates power for motors (teleop)
    public static double[] returnMecanumValues(double rotation, double strafe, double forward, double heading, double scalePower){
        double angle = inputAngle(strafe, forward) + 45 - heading;
        double power = inputMagnitude(strafe, forward);
        double sin = Math.sin(Math.toRadians(angle));
        double cos = Math.cos(Math.toRadians(angle));
        double maxTrig = Math.max(Math.abs(sin), Math.abs(Math.cos(angle)));
        double xPower = power * cos;
        double yPower = power * sin;

        if (maxTrig != 0){
            xPower /= maxTrig;
            yPower /= maxTrig;
        }

        double frontLeft = xPower;
        double frontRight = yPower;
        double backLeft = yPower;
        double backRight = xPower;
        frontRight -= (rotation * ROTATION_CONSTANT);
        backLeft += (rotation * ROTATION_CONSTANT);
        frontLeft += (rotation * ROTATION_CONSTANT);
        backRight -= (rotation * ROTATION_CONSTANT);
        double maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));

        if (maxPower > 1){
            frontLeft /= maxPower;
            frontRight /= maxPower;
            backLeft /= maxPower;
            backRight /= maxPower;
        }

        frontLeft *= scalePower;
        frontRight *= scalePower;
        backLeft *= scalePower;
        backRight *= scalePower;
        double[] values = {frontLeft, frontRight, backLeft, backRight};
        return values;
    }

    //used for time based auto
    //applies power based off of angle and speed instead of strafe and forward
    public static double[] returnMecanumValuesAuto(double magnitude, double angle, double rotation, double heading, double scalePower){
        double xPower = magnitude * Math.cos(Math.toRadians(angle + 45 - heading));
        double yPower = magnitude * Math.sin(Math.toRadians(angle + 45 - heading));
        double frontLeft = xPower;
        double frontRight = yPower;
        double backLeft = yPower;
        double backRight = xPower;
        frontRight -= (rotation * ROTATION_CONSTANT);
        backLeft += (rotation * ROTATION_CONSTANT);
        frontLeft += (rotation * ROTATION_CONSTANT);
        backRight -= (rotation * ROTATION_CONSTANT);
        frontLeft *= scalePower;
        frontRight *= scalePower;
        backLeft *= scalePower;
        backRight *= scalePower;
        double[] values = {frontLeft, frontRight, backLeft, backRight};
        return values;
    }

    //vector calculations for coaxial swerve (r = direction, theta = power)
    public static Vector[] returnSwerve(Vector drive, double rotation, double heading){
        double offsetTheta = 45;
        double rotPower = 0.8;

        Vector correctedDrive = drive.rotate(heading);
        Vector FL = correctedDrive.add(new Vector(new Vector.VectorPolar(rotation * rotPower, offsetTheta)));
        Vector FR = correctedDrive.add(new Vector(new Vector.VectorPolar(rotation * rotPower, -offsetTheta)));
        Vector BL = correctedDrive.add(new Vector(new Vector.VectorPolar(rotation * rotPower, 180 - offsetTheta)));
        Vector BR = correctedDrive.add(new Vector(new Vector.VectorPolar(rotation * rotPower, 180 + offsetTheta)));

        return neutralize(FL, FR, BL, BR);

    }

    //coaxial swerve
    //rescales four vectors such that max r = 1
    //ensures motor never tries to set more power than possible
    public static Vector[] neutralize(Vector one, Vector two, Vector three, Vector four){
        double max = Math.max(Math.max(one.r, two.r),
                Math.max(three.r, four.r));
        return new Vector[] {new Vector(new Vector.VectorPolar(one.r / max, one.theta)),
                            new Vector(new Vector.VectorPolar(two.r / max, two.theta)),
                            new Vector(new Vector.VectorPolar(three.r / max, three.theta)),
                            new Vector(new Vector.VectorPolar(four.r / max, four.theta))};
    }
}
