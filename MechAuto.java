package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class MechAuto extends LinearOpMode{
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor leftLift, rightLift;
    private DcMotor coreMotor;
    private Servo roller;
    BNO055IMU c_imu, e_imu;

    NormalizedColorSensor color;
    // todo: write your code here
    
    @Override
    public void runOpMode() {
        motorSetup();
        
        while(!isStarted()) {
            NormalizedRGBA colors = color.getNormalizedColors();
            telemetry.addData("Color.red", colors.red);
            telemetry.addData("Color.blue", colors.blue);
            telemetry.addData("Color.green", colors.green);
            telemetry.addData("Color.alpha", colors.alpha);
            telemetry.update();
        }
        
        waitForStart();
        
        goAngle(0.25, 270, 1000);
        long startTime = getTime();
        while(getColor().alpha <= 0.5 && getTime() - startTime < 15000) {
            opModeIsActive();
            telemetry.addData("Alpha", getColor().alpha);
            telemetry.update();
        }
        
        NormalizedRGBA colors = getColor();
        
        if (colors.alpha >= 0.4) {
            if (colors.blue >= colors.green && colors.blue >= colors.red) {
                //1
                goAngle(0.5, 270, 1000);
                goAngle(0.5, 180, 3000);
            } else if (colors.red >= colors.green && colors.red >= colors.blue) {
                //2 
                goAngle(0.5, 270, 2000);
            } else if (colors.green >= colors.red && colors.green >= colors.blue) {
                //3 
                goAngle(0.5, 270, 1000);
                goAngle(0.5, 0, 4000);
            } 
        }
        
        
        stopDrive();
        
        //goAngle(0.5, 0, 1000);
    }
    
    void goAngle(double power, double robotAngle, long time) {
        robotAngle += Math.PI/2;
        robotAngle *= Math.PI/180;
        
        double rightBackPower = (power * Math.sin(robotAngle - (Math.PI/4))) * power;
        double leftBackPower = (power * Math.sin(robotAngle + (Math.PI/4))) * power;
        double leftFrontPower = (power * Math.sin(robotAngle - (Math.PI/4))) * power;
        double rightFrontPower = (power * Math.sin(robotAngle + (Math.PI/4))) * power;
        
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        
        sleep(time);
        
    }
    
    void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    
    public static long getTime()
    {
        return (System.nanoTime()) / 1000000;
    }
    
    NormalizedRGBA getColor() {
        return color.getNormalizedColors();
    }
    
    void motorSetup() {
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftLift = hardwareMap.get(DcMotor.class, "lift1");
        rightLift = hardwareMap.get(DcMotor.class, "lift2");
        
        roller = hardwareMap.get(Servo.class, "roller");
        roller.setDirection(Servo.Direction.REVERSE);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLift.setDirection(DcMotor.Direction.REVERSE);
    }
}
