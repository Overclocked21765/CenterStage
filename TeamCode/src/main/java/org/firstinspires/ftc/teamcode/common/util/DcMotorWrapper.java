package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorWrapper implements DcMotorEx {
    public DcMotorEx motor;
    private double lastAngleRate;
    private AngleUnit lastAngleUnit;
    private double[] lastPIDF = {0, 0, 0, 0};
    private double lastP;
    private double lastTolerance;
    private double lastCurrentAlert;
    private CurrentUnit lastCurrentAlertUnit;
    private int lastTargetPosition;
    private RunMode runmode;
    private Direction direction;
    private double lastPower;

    public DcMotorWrapper(String deviceName, HardwareMap hardwareMap, Direction direction, RunMode runmode){
        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        this.direction = direction;
        lastAngleRate = 0;
        lastAngleUnit = AngleUnit.DEGREES;
        lastP = 0;
        lastTolerance = 0;
        lastCurrentAlert = 0;
        lastCurrentAlertUnit = CurrentUnit.AMPS;
        lastTargetPosition = 0;
        lastPower = 0;
        this.runmode = runmode;


        this.motor.setMode(runmode);
        this.motor.setDirection(direction);
    }

    public DcMotorWrapper(String deviceName, HardwareMap hardwareMap){
        this(deviceName, hardwareMap, Direction.FORWARD, RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotorWrapper(String deviceName, HardwareMap hardwareMap, Direction direction){
        this(deviceName, hardwareMap, direction, RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        if (lastAngleUnit == AngleUnit.DEGREES){
            if (Math.toRadians(lastAngleRate) != angularRate){
                motor.setVelocity(angularRate);
                lastAngleRate = angularRate;
                lastAngleUnit = AngleUnit.RADIANS;
            }
        } else if (lastAngleRate != angularRate){
            motor.setVelocity(angularRate);
            lastAngleRate = angularRate;
            lastAngleUnit = AngleUnit.RADIANS;
        }

    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        double currentRate;
        if (this.lastAngleUnit == AngleUnit.DEGREES){
            currentRate = Math.toRadians(this.lastAngleRate);
        } else {
            currentRate = this.lastAngleRate;
        }

        double setRate = (unit == AngleUnit.DEGREES) ? Math.toRadians(angularRate) : angularRate;

        if (Double.compare(setRate, currentRate) != 0) {
            motor.setVelocity(angularRate, unit);
            lastAngleRate = angularRate;
            lastAngleUnit = unit;
        }



    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {}

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (!(lastPIDF[0] == p && lastPIDF[1] == i && lastPIDF[2] == d && lastPIDF[3] == f)){
            motor.setVelocityPIDFCoefficients(p, i, d, f);
            lastPIDF = new double[] {p, i, d, f};
        }
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        if (lastP != p){
            motor.setPositionPIDFCoefficients(p);
            lastP = p;
        }
    }

    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) {return null;}

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (lastTolerance != tolerance){
            motor.setTargetPositionTolerance(tolerance);
            lastTolerance = tolerance;
        }
    }

    @Override
    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        if (!(lastCurrentAlertUnit == unit && lastCurrentAlert == current)){
            motor.setCurrentAlert(current, unit);
            lastCurrentAlert = current;
            lastCurrentAlertUnit = unit;
        }
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Override public void setPowerFloat() {}

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        if (lastTargetPosition != position){
            motor.setTargetPosition(position);
            lastTargetPosition = position;
        }
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        if (mode != runmode){
            motor.setMode(mode);
            runmode = mode;
        }
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        if (this.direction != direction){
            setDirection(direction);
            this.direction = direction;
        }
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public void setPower(double power) {
        if (lastPower != power){
            motor.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}
