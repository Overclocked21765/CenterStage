package org.firstinspires.ftc.teamcode.based.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Represents an example of a user defined subsystem
 *
 * All subsystems are to be user declared
 */
public class ExampleSubsystem extends SubsystemBase {
    private DcMotorEx motor;

    public ExampleSubsystem(DcMotorEx motor){
        super();

        this.motor = motor;
    }

    public void move(){
        this.motor.setPower(1);
    }

    public void brake(){
        this.motor.setPower(0);
    }

    //Called every loop
    @Override
    public void periodic(){

    }
}
