package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@Autonomous(name = "AutonomousBlue", group = "Autonomous")
//@Disabled
public class AutonomousBlue extends LinearOpMode {

//    private boolean turnFirst = true;
//    private boolean turning = false;
//    boolean turningRight = false;
//    boolean TurnRight = false;
//    boolean bBoolean = true;
    private final int FiringCount = 0;


    Variables var;
    Motors motors;
    RobotHardwareMap robot;

//    AutonomousMove DriveTrain = AutonomousMove.STOP;
//    AutonomousShooter Shooter = AutonomousShooter.SHOOTERREST;

//    private AutonomousMove curMoveState = AutonomousMove.STOP;

    //Vuforia
    VuforiaTrackable lastTrackable;

    private boolean targetVisible = false;

    @Override
    public void runOpMode() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;
        robot = var.robot;

        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable : var.allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                lastTrackable = trackable;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    var.lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = var.lastLocation.getTranslation();
            telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0), translation.get(1), translation.get(2));

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(var.lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }


        if (var.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = var.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            } else
                telemetry.addData("# Object Detected", 0);
        }


        waitForStart();

        if (FiringCount == 0) {
            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);
            }
        }

        if (FiringCount == 1) {
            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) {
                motors.driveStrafe(135 * Math.PI / 180, 0.4, true);

            }

            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }


        }
        if (FiringCount == 4) {

            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }
            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }
            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }

            while (robot.colourF.alpha() > 2000) {
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


        while (robot.distanceR.getDistance(DistanceUnit.MM) > 450) {
            motors.driveStrafe(135 * Math.PI / 180, 0.4, true);

        }
        motors.stop();
        telemetry.addData("Right", robot.distanceR.getDistance(DistanceUnit.MM));

        sleep(3000);
        while (robot.distanceB.getDistance(DistanceUnit.MM) < 450) {
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

        while (robot.distanceB.getDistance(DistanceUnit.MM) >= 400) {
            robot.leftFront.setPower(-0.4);
            robot.leftBack.setPower(-0.4);
            robot.rightBack.setPower(-0.4);
            robot.rightBack.setPower(-0.4);
        }
        while (robot.colourF.alpha() < 1000) {
            motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
        }

        while ((robot.distanceR.getDistance(DistanceUnit.MM) <= 915))  //&& ringtargetIsNotVisible
        {

            motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
        }

        while (robot.distanceL.getDistance(DistanceUnit.MM) >= 150) {
//strafe LEFT
            motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
        }


        if (FiringCount == 1) {
            while (robot.colourF.alpha() < 2000) {
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightFront.setPower(1);
                robot.rightBack.setPower(1);

            }
        }
        //Strafe R tot Distance R = 950mm

        while (robot.distanceL.getDistance(DistanceUnit.MM) <= 900) {

            motors.driveStrafe(135 * Math.PI / 180, 0.4, true);
        }
//Kyk vir die wit lyn
        while (robot.colourF.alpha() < 2000) {

            robot.leftFront.setPower(1);
            robot.leftBack.setPower(1);
            robot.rightFront.setPower(1);
            robot.rightBack.setPower(1);

        }
        // Vanaf die wit lyn, kyk vir die grys

        while (robot.colourF.alpha() > 2000) {
            robot.leftFront.setPower(1);
            robot.leftBack.setPower(1);
            robot.rightFront.setPower(1);
            robot.rightBack.setPower(1);

        }
        //Vanaf die grys, kyk weer vir die wit/blou lyn
        while (robot.colourF.alpha() < 2000) {
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

    private void ShooterState(AutonomousShooter Shooter) {

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
