package org.firstinspires.ftc.teamcode.myOpmodes;

/**
 * This is a basic Autonomous program for the
 *  Centerstage competition using a mecanum drive
 *  style robot. The robot will be starting in the
 *  Red side alliance in the Audience position.
 *
 * @author DNelson2
 * @version 11/15/2023
 *
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.botSetup.RoboSetup;

@Autonomous(name = "Auto Red Audience", group = "NelsonBot")
//@Disabled
public class BasicAutoAudienceRed extends LinearOpMode {
    //Add instance of the RoboSetup
    RoboSetup myMec = new RoboSetup();

    //Declare a timer
    private ElapsedTime runtime = new ElapsedTime();

    //Variable to keep track of distance from an object
    private double currentDist = 0.0;

    //Constants
    static final double FORWARD_SPEED = 0.2;
    static final double TURN_SPEED = 0.3;
    //goBilda Encoder count 537.7 PPR at the Output Shaft
    static final double GOBILDA_OUTPUT = 537.7;


    @Override
    public void runOpMode() {
        runtime.reset();//resets the timer
         /*
            Initialize the drive system variables.
            THe init() method of the hardware map does
            all of the work here.
         */
        myMec.init(hardwareMap);

        //Wait for the game to start (driver presses PLAY)
        waitForStart();

        //STEP 01.5 - Wait for other team to get out of the way
        //Thread.sleep(1500);
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("Status: ", "Waiting for Alliance Partner to move.");
            telemetry.update();
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();//resets the timer

        //STEP 02 - Move Forward
//        myMec.resetEncoders();
//        while(opModeIsActive() && (myMec.getRotationBR() < 0.5)){
//            telemetry.addData("Rotation: ", myMec.getRotationBR());
//            telemetry.addData("Status:", "Step 2 Execute");
//            telemetry.update();
//            myMec.setMecanumDrive(0,-0.2,0);
//
//        }

        while(opModeIsActive() && runtime.seconds() < 10){
            telemetry.addData("Rotation: ", myMec.getRotationBR());
            telemetry.addData("Status:", "Step 2 Execute");
            telemetry.update();
            myMec.setMecanumDrive(-0.2,0,0);

        }
        //Stop Robot
        myMec.setMecanumDrive(0,0,0);

        //STEP 03 - Drive Backwards until crosses the backstage line

    }//END OF runOpMode()
}
