package org.firstinspires.ftc.teamcode.botSetup;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * This is the code for setting up my Centerstage robot which consists of
 *  a small mecanum chassis. This will be version of mecanum drive. The class
 *  will review other version using mathematical equations to achieve
 *  more elegant coding.
 *
 *  Hardware Setup
 *
 *  ---- Control Hub ---
 *  MOTORS
 *  Port 0 - backLeft
 *  Port 1 - frontLeft
 *  Port 2 - frontRight
 *  Port 3 - backRight
 *
 *  SERVOS
 *  Port 0 - TBD
 *
 *  SENSORS
 *  I2C Expansion
 *  Port 0 - rightDist (Distance Sensor)
 *  Port 1 - backDist
 *
 * --- Expansion Hub ----
 * Port 0 - Motor TBD
 * Port 3 - Motor TBD
 *
 * @author DNel2
 * @version 10/06/2023
 *
 */
public class RoboSetup {
    //Instance Variables for Mechanisms
    //////////////////////
    //      Motors      //
    //////////////////////
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
   // private DcMotor liftMotor;
   // private DcMotor intakeMotor;
    //////////////////////
    //      Servos      //
    //////////////////////
    private CRServo inTake;
    //private Servo carousel2;
    //private Servo flick;

    //////////////////////
    //      Sensors      //
    //////////////////////
    private DistanceSensor myDistanceSensorRight;
    private DistanceSensor myDistanceSensorBack;
    //private TouchSensor myLiftTouch;

    /////////////////////////////
    //     Counter Value       //
    ////////////////////////////
    private double ticksPerRotation;
    private double ticksPerRotationFL;
    private double ticksPerRotationBL;
    private double ticksPerRotationFR;
    private double ticksPerRotationBR;

    //Add the param for init with HardwareMap
    public void init(HardwareMap hwMap) {
        //setting up the motors
        //Left Back Wheel Motor
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All left will be reversed for Math version of Mec Drive
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Left Front Wheel Motor
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All left will be reversed for Math version of Mec Drive
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Right Back Wheel Motor
        backRight = hwMap.get(DcMotor.class, "backRight");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All Right will be reversed ... never
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Right Front Wheel Motor
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All Right will be reversed ... never
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Setting up the Sensors
        myDistanceSensorRight = hwMap.get(DistanceSensor.class, "distSensor");
        myDistanceSensorBack = hwMap.get(DistanceSensor.class, "distSensorBack");
        // myLiftTouch = hwMap.get(TouchSensor.class, "liftTouch");

        //Setting up the servos
        inTake = hwMap.get(CRServo.class, "inTake");
        inTake.resetDeviceConfigurationForOpMode();

    } //END OF Init

    //Methods

    ///////////////////////////////////////////////////
    //                                               //
    // Hardcoded Mecanum Methods - Not Best Practice //
    //                                               //
    ///////////////////////////////////////////////////
    /**
     * This method will move the robot forward
     * @param power - double value to set the power of the movement
     */
    public void setForward(double power){
        backLeft.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot backward
     * @param power - double value to set the power of the movement
     */
    public void setBackward(double power){
        backLeft.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot to slide to the left
     * @param power - double value to set the power of the movement
     */
    public void setSlideLeft(double power){
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot to slide to the Right
     * @param power - double value to set the power of the movement
     */
    public void setSlideRight(double power){
        backLeft.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(-power);
    }

    /**
     * This method will move the robot to turn left
     * @param power - double value to set the power of the movement
     */
    public void setTurnLeft(double power){
        backLeft.setPower(-power);
        frontLeft.setPower(-power);
        backRight.setPower(power);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot to turn left
     * @param power - double value to set the power of the movement
     */
    public void setTurnRight(double power){
        backLeft.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(-power);
        frontRight.setPower(-power);
    }

    /**
     * This method will move the robot diagonally forward to the left
     * @param power - double value to set the power of the movement
     */
    public void setDiagLtF(double power){
        backLeft.setPower(power);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(power);
    }

    /**
     * This method will move the robot diagonally backward to the left
     * @param power - double value to set the power of the movement
     */
    public void setDiagLtB(double power){
        backLeft.setPower(0);
        frontLeft.setPower(-power);
        backRight.setPower(-power);
        frontRight.setPower(0);
    }

    /**
     * This method will move the robot diagonally forward to the right
     * @param power - double value to set the power of the movement
     */
    public void setDiagRtF(double power){
        backLeft.setPower(0);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(0);
    }

    /**
     * This method will move the robot diagonally backward to the left
     * @param power - double value to set the power of the movement
     */
    public void setDiagRtB(double power){
        backLeft.setPower(-power);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(-power);
    }


    //////////////////////////////////////////////////////////
    //                                                      //
    //   CLEANER VERSION OF THE PROGRAM FOR MECANUM         //
    //                                                      //
    //////////////////////////////////////////////////////////

    /**
     * This method will set all the values for the motors
     *  to drive the mecanum chassis. This method is inspired
     *  by code from gmZero programming tutorial
     *
     *  The left stick will control linear movement:
     *   forward, back, slide left, slide right, and
     *   diagonal movement.
     *  The right stick will control the rotational movement:
     *   spin left, spin right.
     *
     * @param y - the left stick y (double) value
     * @param x - the left stick x (double) value
     * @param rx - the right stick x (double) value
     */
    public void setMecanumDrive(double y, double x, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x)+Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) /denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    //////////////////////////////////////////////
    //                                          //
    //      Sensor Methods                      //
    //                                          //
    /////////////////////////////////////////////

    /**
     * This method will return the sensor reading
     * of the distance sensor
     *
     * @return Double value for the distance read from the sensor
     *
     */
    public double getCurrentDistanceRight(DistanceUnit du){
        return myDistanceSensorRight.getDistance(du);
    }

    /**
     * This method will return the sensor reading
     * of the distance sensor that is located at the back
     * of the robot
     *
     * @return Double value for the distance read from the sensor
     *
     */
    public double getCurrentDistanceBack(DistanceUnit du){
        return myDistanceSensorBack.getDistance(du);
    }


    /**
     * This method will return the state of the
     *  touch sensor located at the top of the lift
     *  mechanism
     *
     * @return - boolean value true pressed false unpressed
     */
//    public boolean getLiftTouchSensorState(){
//        boolean isLiftPressed = false;
//        isLiftPressed = myLiftTouch.isPressed();
//        return isLiftPressed;
//    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationFR() {
        ticksPerRotationFR = frontRight.getMotorType().getTicksPerRev();
        return frontRight.getCurrentPosition() / ticksPerRotationFR;
    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationBR() {
        ticksPerRotationBR = backRight.getMotorType().getTicksPerRev();
        return backRight.getCurrentPosition() / ticksPerRotationBR;
    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationFL() {
        ticksPerRotationFL = frontLeft.getMotorType().getTicksPerRev();
        return frontLeft.getCurrentPosition() / ticksPerRotationFL;
    }

    /**
     * This method will return the motor encoder rotations
     *
     * FRONT RIGHT WHEEL
     *
     * @return - the number of rotations
     */
    public double getRotationBL() {
        ticksPerRotationBL = backRight.getMotorType().getTicksPerRev();
        return backLeft.getCurrentPosition() / ticksPerRotationBL;
    }


}//END OF CLASS
