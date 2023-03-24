package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@TeleOp

public class Single extends LinearOpMode {
    private DcMotor single;
            
    @Override
    public void runOpMode() {

        single = hardwareMap.get(DcMotor.class, "lift");
        waitForStart();
        while(opModeIsActive()){
            
             single.setPower(gamepad1.left_stick_y);
             
        }
    }
    
}