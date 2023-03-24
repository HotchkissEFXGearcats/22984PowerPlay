package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@Disabled
@TeleOp

public class MechaDriver extends LinearOpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotor lift0, lift1;
    private int liftZero, liftPos, max, buffer;

    @Override
    public void runOpMode() {
        
        motorSetup();
        
        max = 2800; // max lift position
        buffer = max - 300;
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addData("Lift Postion, Init: ", liftZero);
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            if (abs(gamepad2.right_stick_y) > 0.05) {
                liftPos = moveLift(0.5 * gamepad2.right_stick_y);
            } else {
                liftPos = moveLift(0.0);
            }
            
            telemetry.addData("Status", "Running");
            telemetry.addData("Gamepad Stick: ", "%.3f", gamepad2.right_stick_y);
            telemetry.addLine();
            telemetry.addData("Lift Postion: ", liftPos);
            telemetry.update();

        }  // end opmode while
        
    }  // end method runopmode
    
    
    
    
    
    void motorSetup() {
        
        lift0 = hardwareMap.get(DcMotor.class, "lift1");
        lift1 = hardwareMap.get(DcMotor.class, "lift2");
        
        lift0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift0.setDirection(DcMotor.Direction.REVERSE);
        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        liftZero = -lift0.getCurrentPosition();
        liftPos = liftZero;
        
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        //leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        //leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        //rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        //rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        
        
        //claw = hardwareMap.get(Servo.class, "claw");
        //flipperLeft = hardwareMap.get(Servo.class, "flipperLeft");
        //flipperRight = hardwareMap.get(Servo.class, "flipperRight");
        
        //flipperLeft.setDirection(Servo.Direction.REVERSE);
        //flipperRight.setDirection(Servo.Direction.FORWARD);

        //leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
    }  // end method motorSetup
    
    
    
    private int moveLift(double power) {
        lift0.setPower(power);
        lift1.setPower(power);
        return -lift0.getCurrentPosition();
    }  // end method moveLift
    
    
    /*
    
    
    public int limitMove(double power) {
    if (power < 0.0) {
        // move up
        if (-motor.getCurrentPosition() < max) {
            if (-motor.getCurrentPosition() < buffer) {
                moveLift(fast * power);
            } else {
                moveLift(slow * power);
            }
        } else {
            moveLift(0.0);
        }  // end if-else
    } else if (power > 0.0) {
        // move down
        if (-motor.getCurrentPosition() > zero) {
            if (-motor.getCurrentPosition() > threshold) {
                moveLift(fast * power);
            } else {
                moveLift(slow * power);
            }
        } else {
            moveLift(0.0);
        }  // end if-else
    } else {
        motor.setPower(0.0);
    }
    return leftLift.getCurrentPosition();
    }  // end method move
    
    */
    
    
    
    
}  // end class



