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
@Disabled
@TeleOp

public class mecanumTepeOpBroken extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private Gyroscope imu;
    private DcMotor lift;
    
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor leftLift, rightLift;
    private DcMotor coreMotor;
  
    BNO055IMU c_imu, e_imu;
    
    double offset, angle, rawAngle;
    private int corePos;
    private Servo claw, roller;


    
    boolean headless = false;
    double headlessWait = 0;
    
      @Override
    public void runOpMode() {
        motorSetup();
        gyroSetup();
        
        resetLiftMin();
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getAngle();
            servoPower();
            drivetrainPower();
            liftPower();
            flipCore();
            
            telemetry.addData("Angle", angle);
            telemetry.addData("headless", headless);
            
            if (gamepad2.y) {
                resetLiftMin();
            } else if (gamepad1.x) {
                if ((getTime()) - headlessWait >= 500) {
                    headless = !headless;
                    headlessWait = getTime();
                }
            } else if (gamepad1.y) {
                offset();
            }
            //resetEncoders();
            
            telemetry.addData("lift", leftLift.getCurrentPosition());
            telemetry.addData("lift2", rightLift.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Core Motor Position: ", coreMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Left Trigger:  ", "%.2f", -gamepad2.left_trigger);
            telemetry.addData("Right Trigger: ", "%.2f", gamepad2.right_trigger);
            telemetry.addLine();
            
            //telemetry.update();
        }  // end while opmodeisactive
    }  // end method runopmode
    
    //boolean clawOpen = false;
    long switchTime = getTime();
    //intake
    
    void servoPower(){
        /*
        if (gamepad2.x && getTime() - switchTime > 250) {
            switchTime = getTime();
            clawOpen = !clawOpen;
        }
        if (clawOpen){
           claw.setPosition(0.25); 
        } 
        else claw.setPosition(0);
        telemetry.addData("claw", claw.getPosition());
        */
        
        /*
        if (gamepad2.right_trigger > 0){
            roller.setDirection(Servo.Direction.FORWARD);
            roller.setPosition(0.75);
        }
        else if (gamepad2.right_bumper){
            roller.setDirection(Servo.Direction.REVERSE);
            roller.setPosition(0.75);
        }
        else{
            roller.setPosition(0);
        }
        */
        
        if ((gamepad2.right_trigger - gamepad2.left_trigger) > 0.0) {
            roller.setPosition(0.75);
        } else if ((gamepad2.right_trigger - gamepad2.left_trigger) < 0.0) {
            roller.setPosition(0.25);
        }  // end if roller
       
    }
    
    
    void liftSimplePower() {
        rightLift.setPower(-gamepad2.right_stick_y);
        leftLift.setPower(gamepad2.right_stick_y);
    }
    
    //the minimum position of the lift, for the left and right sides
    int leftLiftMinPosition = 0;
    int rightLiftMinPosition = 0;
    
    //The distance to the top of the lift from the bottom, in encoder units
    int liftLength = 2600;
    
    //the main function to move the lift
    //call inside the main loop, probably near the generalPower() that moves the robot. 
    boolean liftUp = false;
    boolean liftDown = false;
    double liftAutoPower = 0.75;
    
    void liftPower() {
        
        //to make code later down the line shorter
        double power = -gamepad2.left_stick_y;
        
        if (gamepad2.dpad_down) {
            power = -1;
        } else if (gamepad2.dpad_up) {
            power = 1;
        }
    
        //some variables to make later lines clearer
        //the extension is the total distance above the minimum position.
        //if it is 0, the lift is at the bottom. if it is at liftLength, it is at the top
        int leftLiftExtension = leftLift.getCurrentPosition() - leftLiftMinPosition;
        int rightLiftExtension = rightLift.getCurrentPosition() - rightLiftMinPosition;
    
        if (!gamepad2.right_bumper) {
            //make the lift move slowly if it's at the top of the range. 
            //will trigger if either lift motor is over.
            //they *should* be synced, but that's not a guarantee. Especially on Octo. 
            if (leftLiftExtension >= liftLength || rightLiftExtension >= liftLength) {
                //but only if the stick is going upwards
                //ex: if the power is negative (lift going down) that should still work fine
                if (power >= 0.1) {
                    power = 0.1;
                }
            }
    
        
            //if the lift is at the bottom of the range, also move slowly
            if (leftLiftExtension <= 150 || rightLiftExtension <= 150) {
                //again, only if the stick is going down. 
                //going up should still work just fine. 
                if (power <= -0.1) {
                    power = -0.1;
                }
             }
    
            //set the power to the lift motors
            leftLift.setPower(power);
            rightLift.setPower(power);
        } else {
            leftLift.setPower(-gamepad2.left_stick_y);
            rightLift.setPower(-gamepad2.left_stick_y);
        }
    }
    
    //In case the minimum position of the lift gets messed up in any way
    //set this to be triggered by one of the buttons on the gamepad
    //What this does is it resets where the robot thinks the bottom of the lift is.
    //To use, lower the lift all the way down and then call the function
    void resetLiftMin() {
        leftLiftMinPosition = leftLift.getCurrentPosition();
        rightLiftMinPosition = rightLift.getCurrentPosition();
    }
        void setLiftTarget(int target) {
        leftLift.setTargetPosition(target);
        rightLift.setTargetPosition(target);
    }
    
    void setLiftEncoder(boolean encoder) {
        if (encoder) {
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    
    void setLiftPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

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
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
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
    
    
    // Rotates the chain bar to opposing position
    //
    public void flipCore() {
        coreMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        corePos = coreMotor.getCurrentPosition();;
    }  // end method flipCore
    
    
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
    
    // todo: write your code here
}



///
