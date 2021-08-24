package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;



@Autonomous(name="AutonomousBlue", group="Autonomous")
//@Disabled
public class AutonomousBlue extends LinearOpMode {

    private boolean turnFirst = true;
    private boolean turning = false;
    boolean turningRight = false;
    boolean TurnRight = false;
    boolean bBoolean = true;
    private int iringCount=0;


    Variables var;
    Motors motors;
    RobotHardwareMap robot;

    AutonomousMove DriveTrain = AutonomousMove.STOP;
    AutonomousShooter Shooter = AutonomousShooter.SHOOTERREST;

    private AutonomousMove curMoveState = AutonomousMove.STOP;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " --- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        var = new Initialize().Init(hardwareMap);
        motors = var.motors;

        robot = var.robot;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        waitForStart();

if (iringCount==0) {
    while (robot.colourF.alpha() < 2000) ;
    {
        robot.leftFront.setPower(1);
        robot.leftBack.setPower(1);
        robot.rightFront.setPower(1);
        robot.rightBack.setPower(1);

    }


}

        if (iringCount==1) {
            while (robot.colourF.alpha() < 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) ;
            {
                motors.driveStrafe(135 * Math.PI / 180, 0.4, true);

            }

            while (robot.colourF.alpha() < 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() < 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }





        }
        if (iringCount==4) {

            while (robot.colourF.alpha() < 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }while (robot.colourF.alpha() < 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }
            while (robot.colourF.alpha() < 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }





        }


//        ShooterState(AutonomousShooter.AUTOMATEDSHOOTERREST);

//
//    ShooterState(AutonomousShooter.AUTOMATEDSHOOTER);

//        while (opModeIsActive()) {
//            robot.shooterMotor.setPower(-1);
//
//        }


        while (robot.distanceR.getDistance(DistanceUnit.MM) >450)
        {
            motors.driveStrafe(135 * Math.PI / 180, 0.4, true);

        }
        motors.stop();
        telemetry.addData("Right",robot.distanceR.getDistance(DistanceUnit.MM));

sleep(3000);
        while (robot.distanceB.getDistance(DistanceUnit.MM) < 450)
        {
            robot.leftFront.setPower(0.1);
            robot.leftBack.setPower(0.1);
            robot.rightFront.setPower(0.1);
            robot.rightBack.setPower(0.1);
        }
        motors.stop();

        robot.shooterMotor.setPower(-1);
        robot.magazineLifter.setPosition(0);
        sleep(700);
        robot.Tap.setPosition(0);
        sleep(500);
        robot.Tap.setPosition(0.4);
        sleep(500);
        robot.Tap.setPosition(0);
        sleep(500);
        robot.Tap.setPosition(0.4);
        robot.magazineLifter.setPosition(0.4);
        sleep(700);
        robot.sucker.setPower(-1);
        sleep(2000);
        robot.sucker.setPower(0);
        robot.magazineLifter.setPosition(0);
        sleep(700);
        robot.Tap.setPosition(0);

        while (robot.distanceB.getDistance(DistanceUnit.MM) >=400)
        {
            robot.leftFront.setPower(-0.4);
            robot.leftBack.setPower(-0.4);
            robot.rightBack.setPower(-0.4);
            robot.rightBack.setPower(-0.4);
        }
 while (robot.colourF.alpha()<1000)
 {
     motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
 }

 while ((robot.distanceR.getDistance(DistanceUnit.MM)<=915))  //&& ringtargetIsNotVisible
 {

     motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
 }

 while (robot.distanceL.getDistance(DistanceUnit.MM) >=150)
 {
//strafe LEFT
     motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
 }


        if (iringCount==1) {
            while (robot.colourF.alpha() < 2000) ;
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }
        }
            //Strafe R tot Distance R = 950mm

            while (robot.distanceL.getDistance(DistanceUnit.MM) <= 900)
            {

                motors.driveStrafe(135 * Math.PI / 180, 0.4, true);
            }
//Kyk vir die wit lyn
            while (robot.colourF.alpha() < 2000) ;
            {

                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }
            // Vanaf die wit lyn, kyk vir die grys

            while (robot.colourF.alpha() >2000)
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }
        //Vanaf die grys, kyk weer vir die wit/blou lyn
            while (robot.colourF.alpha() <2000)
            {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);
            }



//        ShooterState(AutonomousShooter.TAPRIGHT);
//
//        ShooterState(AutonomousShooter.TAPDEFAULT);





//        while (robot.Tap.getPosition() != 0.4) {
//        ShooterState(AutonomousShooter.TAPDEFAULT);
//    }
//
//        while (robot.Tap.getPosition() != 0) {
//            ShooterState(AutonomousShooter.TAPRIGHT);
//        }


//        while (robot.Tap.getPosition() != 0.4) {
//            ShooterState(AutonomousShooter.TAPDEFAULT);
//        }
// ----Turning----//

//        MoveState(DriveTrain.TURNLEFT); // Turn to 90 left
//
//        MoveState(DriveTrain.TURNFRONT); // Turn to start position

// ----Shooter on ---- //

//        motors.Fire(10000);
//////
//        while (robot.shooterMotor.getCurrentPosition() < robot.shooterMotor.getTargetPosition()) {
//            robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.shooterMotor.setPower(-1);
////}

            //----Driving forward---- //

//            motors.stop();
//while  (bBoolean)
//        {
//            robot.leftFront.setPower(0.5);
//            robot.rightBack.setPower(0.5);
//            robot.rightFront.setPower(0.5);
//            robot.leftBack.setPower(0.5);
//        }
//


//
//        if ((robot.distanceL.getDistance(DistanceUnit.MM) < 300))
//            motors.stop();


// ----Strafe Right---- //
//while (bBoolean)
//        motors.driveStrafe(135 * Math.PI / 180, 0.4, true);

//        if ((robot.distanceL.getDistance(DistanceUnit.MM) < 300))
//            motors.stop();

//----Strafe Left-----
//while (bBoolean)
//        motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);


            telemetry.addData("IMU", var.getAngle());
            telemetry.update();




    }
    /**
     * Returns an estimation of the horizontal angle to the detected object.
     */
    void estimateAngleToObject(AngleUnit angleUnit) {

    }

    private void MoveState(AutonomousMove DriveTrain) {

        switch (DriveTrain) {

            case TURNLEFT:

                var.resetAngle();
                motors.pidRotate.reset();
                motors.pidRotate.enable();


                motors.rotate(90);

                while (!motors.pidRotate.onTarget())
                    motors.rotate(90);

                if (motors.pidRotate.onTarget())
                    telemetry.addData("IMU", var.getAngle());
                    sleep(2000);



                break;

            case TURNFRONT:

//                motors.pidRotate.reset();
//                motors.pidRotate.enable();


                motors.rotate(-90);

                while (!motors.pidRotate.onTarget())
                    motors.rotate(-90);

                if (motors.pidRotate.onTarget())
                    telemetry.addData("IMU", var.getAngle());
                    sleep(2000);
                break;

            case STOP:
                motors.stop();
        }

    }
        private void ShooterState(AutonomousShooter Shooter)
        {

            switch (Shooter) {
                case FIRE:
                    robot.shooterMotor.setPower(-1);
                break;

                case TAPDEFAULT:
                    robot.Tap.setPosition(0.4);
break;

                case TAPRIGHT:
                    robot.Tap.setPosition(0);
                    break;


                case AUTOMATEDSHOOTER:

//                    robot.Tap.setPosition(0.4);
break;

                case AUTOMATEDSHOOTERREST:
                    robot.shooterMotor.setPower(0);
                    robot.magazineLifter.setPosition(0);
                    robot.Tap.setPosition(0);
            }

        }








    }
