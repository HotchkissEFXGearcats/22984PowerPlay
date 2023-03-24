
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

//New imports of OG Code
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
//end here
@TeleOp

public class FinalMecanumTeleop extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private Gyroscope imu;
    private DcMotor lift;
    // private Servo roller;

    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor leftLift, rightLift;
    private DcMotor coreMotor;
    private Servo roller;
    BNO055IMU c_imu, e_imu;
    
    double offset, angle, rawAngle;
    private int corePos;
    private boolean flag = true;
    
    private int min, max, liftZero, bottomBuffer, topBuffer;
    private double slow, fast;
    
    boolean headless = false;
    double headlessWait = 0;
    
      @Override
    public void runOpMode() {
        motorSetup();
        gyroSetup();
        initializeAutoLift();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getAngle();
            //drivetrainPower();
            //drivetrainSimple();
            
            moveLift(-0.2 * gamepad2.right_stick_y);
            autoLift(1.0);
            intake();
            
            coreMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            
            //telemetry.addData("Angle", angle);
            //telemetry.addData("headless", headless);
            
            telemetry.addData("Lift Zero  : ", liftZero);
            telemetry.addLine();
            telemetry.addData("var max           : ", max);
            telemetry.addData("var topBuffer     : ", topBuffer);
            telemetry.addData("var bottomBuffer  : ", bottomBuffer);
            telemetry.addData("var min           : ", min);
            //telemetry.addLine();
            telemetry.addData("var slow          : ", "%.2f", slow);
            telemetry.addData("var fast          : ", "%.2f", fast);
            telemetry.addLine();
            telemetry.addData("Core Motor Position : ", coreMotor.getCurrentPosition());
            telemetry.addData("Core Motor Power    : ", coreMotor.getPower());
            telemetry.addLine();
            //telemetry.addData("Left Trigger:  ", "%.2f", -gamepad2.left_trigger);
            //telemetry.addData("Right Trigger: ", "%.2f", gamepad2.right_trigger);
            telemetry.addLine();
            
            //telemetry.update();
        }  // end while opmodeisactive
        
    }  // end method runopmode
    
    
    
    // >>>>>>>>>>>>>>> METHODS >>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    
    
    
    public void flipCore(double powerUp, double powerDown) {
        if ((coreMotor.getCurrentPosition() < -50) && leftLift.getCurrentPosition() > 1000) {  // was < 40
            // flip motor up
            while (coreMotor.getCurrentPosition() < -15) {  // was < 70
                coreMotor.setPower(powerUp);
            }  // end while
            coreMotor.setPower(0.0);
        } else if ((coreMotor.getCurrentPosition() > -25) && leftLift.getCurrentPosition() < 1300) {  // was < 75
            // flip motor down
            while (coreMotor.getCurrentPosition() > -20) {  // was > 60
                coreMotor.setPower(-powerDown);
            }  // end while
            coreMotor.setPower(0.0);
        }  // end if-else
        coreMotor.setPower(0.0);
    }  // end method flipCore
    
    // *********************
    
    
    private void initializeAutoLift() {
        this.fast = 0.7;
        this.slow = 0.1;
        this.max = 2630;
        this.min = 60;
        this.topBuffer = this.max - 350;
        this.bottomBuffer = this.min + 300;
        this.liftZero = leftLift.getCurrentPosition();
    }
    
    
     private void moveLift(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }
    
    
    public int autoLift(double power) {
        if (gamepad2.dpad_up) {
            // move up
            //flipCore(0.9,0);
            idle();
            while (leftLift.getCurrentPosition() < max) {
                telemetry.addData("leftLift Pos  : ", leftLift.getCurrentPosition());
                telemetry.addData("rightLift Pos : ", rightLift.getCurrentPosition());
                telemetry.addLine();
                telemetry.update();
                drivetrainPower();
                if (gamepad2.x) {
                    break;
                }
                if (leftLift.getCurrentPosition() < topBuffer) {
                    moveLift(fast * power);
                } else {
                    moveLift(slow * power);
                    flipCore(0.9, 0.0);   //put this back in
                }
            }  // end while
            moveLift(0.0);
            coreMotor.setPower(0.0);
        } else if (gamepad2.dpad_down) {
            // move down
            while (leftLift.getCurrentPosition() > min) {
                telemetry.addData("leftLift Pos  : ", leftLift.getCurrentPosition());
                telemetry.addData("rightLift Pos : ", rightLift.getCurrentPosition());
                telemetry.addLine();
                telemetry.update();
                drivetrainPower();
                if (gamepad2.x) {
                    break;
                }
                if (leftLift.getCurrentPosition() > bottomBuffer) {
                    moveLift(-fast * power);
                } else {
                    moveLift(-slow * power);
                    flipCore(0, 0.5);
                }
            }  // end while
            moveLift(0.0);
            coreMotor.setPower(0.0);
        } else {
            telemetry.addData("leftLift Pos  : ", leftLift.getCurrentPosition());
            telemetry.addData("rightLift Pos : ", rightLift.getCurrentPosition());
            telemetry.addLine();
            telemetry.update();
            drivetrainPower();
        }  // end if-else
        return leftLift.getCurrentPosition();
    }  // end method autoLift
    
    
    public void intake() {
        
        if (gamepad2.right_trigger > 0.1){
            roller.setPosition(0.75);
        }
        else if (gamepad2.left_trigger > 0.1){
            roller.setPosition(0.25);
        }
        else{
            roller.setPosition(0.5);
        }
    }
    
    
    
    // ***********
    
    void drivetrainPower(){
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        
        final double turnMod = 0.75;
        
        if (headless)
        {
            robotAngle += angle*(Math.PI/180);
        } 
        
        double yaw = -gamepad1.right_stick_x * turnMod;
        double power=0.4;
        power = power+((gamepad1.right_trigger)/2);
        
        if (power > 1){
            power = 1;
        }
        
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double rightBackPower = (r * Math.sin(robotAngle - (Math.PI/4)) - yaw) * power;
        double leftBackPower = (r * Math.sin(robotAngle + (Math.PI/4)) + yaw) * power;
        double leftFrontPower = (r * Math.sin(robotAngle - (Math.PI/4)) + yaw) * power;
        double rightFrontPower = (r * Math.sin(robotAngle + (Math.PI/4)) - yaw) * power;



        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        // Check if you need this telemetry, and then remove it if you do not. 
        //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        ///telemetry.addLine();
        ///telemetry.addData("Control Pad Left Stick: ", "left X (%.2f), left Y (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
        //telemetry.addData("Control Pad Sticks: ", "right Y (%.2f), right Y (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
        //telemetry.addLine();
       // telemetry.update();
    }
    
    
    void drivetrainSimple(){
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        
        final double turnMod = 0.75;
        
        double yaw = -gamepad1.right_stick_x * turnMod;
        double power=0.4;
        power = power+((gamepad1.right_trigger)/2);
        
        if (power > 1){
            power = 1;
        }
        
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double rightBackPower = (r * Math.sin(robotAngle - (Math.PI/4)) - yaw) * power;
        double leftBackPower = (r * Math.sin(robotAngle + (Math.PI/4)) + yaw) * power;
        double leftFrontPower = (r * Math.sin(robotAngle - (Math.PI/4)) + yaw) * power;
        double rightFrontPower = (r * Math.sin(robotAngle + (Math.PI/4)) - yaw) * power;



        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        // Check if you need this telemetry, and then remove it if you do not. 
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addLine();
        telemetry.addData("Control Pad Left Stick: ", "left X (%.2f), left Y (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Control Pad Sticks: ", "right Y (%.2f), right Y (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addLine();
        //telemetry.update();
    }
    
    
    
    public static long getTime()
    {
        return (System.nanoTime()) / 1000000;
    }
    
    public void getAngle() {
        Orientation c_angles = c_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        Orientation e_angles = e_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        //rawAngle = (c_angles.firstAngle + e_angles.firstAngle)/2;
        rawAngle = c_angles.firstAngle;
        angle = rawAngle + offset;
    }
    
    //set a new "0" heading for the robot
    public void offset() {
        offset = -rawAngle;
    }
    
    public void gyroSetup() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        c_imu = hardwareMap.get(BNO055IMU.class, "imu");
        e_imu = hardwareMap.get(BNO055IMU.class, "imu2");

        c_imu.initialize(parameters);
        e_imu.initialize(parameters);
    }
    
    
    
    
    
    void motorSetup() {
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
        
        coreMotor = hardwareMap.get(DcMotor.class, "coreMotor");
        coreMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreMotor.setDirection(DcMotor.Direction.FORWARD);
        coreMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        coreMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        corePos = coreMotor.getCurrentPosition();
    
    }  // end motorSetup
    
    
} // end class


