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

    DriveColour driveColour = DriveColour.ONE;

//    AutonomousMove DriveTrain = AutonomousMove.STOP;
//    AutonomousShooter Shooter = AutonomousShooter.SHOOTERREST;

//    private AutonomousMove curMoveState = AutonomousMove.STOP;

    //Vuforia
    VuforiaTrackable lastTrackable;

    private boolean targetVisible = false;
    private String stack;

    @Override
    public void runOpMode() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;
        robot = var.robot;

        robot.wobbleF.setPosition(0);
        robot.wobbleB.setPosition(-0.3);


        stack = "";
        while (!opModeIsActive()) {
        if (var.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = var.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("Stack", recognition.getLabel());
                    stack = recognition.getLabel();
//                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                            recognition.getLeft(), recognition.getTop());
//                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                            recognition.getRight(), recognition.getBottom());
                }
            }
//                telemetry.addData("# Object Detected", 0);
        }

            telemetry.update();
        }

        motors.stop();
        telemetry.addData("Right", robot.distanceR.getDistance(DistanceUnit.MM));
        telemetry.addData("Imu", var.getAngle());

//        sleep(3000);
        MoveState(AutonomousMove.TURNPOWER);
        while (robot.distanceB.getDistance(DistanceUnit.MM) < 700 && opModeIsActive()) {
            robot.leftFront.setPower(0.2);
            robot.leftBack.setPower(0.2);
            robot.rightFront.setPower(0.2);
            robot.rightBack.setPower(0.2);
        }



        motors.stop();

        robot.shooterMotor.setPower(-0.9);
        robot.magazineLifter.setPosition(0);
        sleep(1000);
        robot.Tap.setPosition(0);
        sleep(500);
        robot.Tap.setPosition(0.4);
        sleep(500);
        robot.Tap.setPosition(0);
        sleep(500);
        robot.Tap.setPosition(0.4);
        robot.shooterMotor.setPower(0);

        while (robot.distanceL.getDistance(DistanceUnit.MM)>200 && opModeIsActive()) {
            motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
        }

//        MoveState(AutonomousMove.TURNFRONT);

        MoveState(AutonomousMove.MOVEBLUE);

        MoveState(AutonomousMove.MOVEBLACK);

        MoveState(AutonomousMove.MOVEBLUE);

        if (stack.isEmpty())
            driveColour = DriveColour.ZERO;
        else if (stack.equalsIgnoreCase("single"))
            driveColour = DriveColour.ONE;
        else if (stack.equalsIgnoreCase("quad"))
            driveColour = DriveColour.FOUR;

        switch (driveColour) {
            case ZERO:

                for (int i = 0; i < 5000; i++) {
                    robot.leftBack.setPower(0.4);
                    robot.leftFront.setPower(0.4);
                    robot.rightBack.setPower(0.4);
                    robot.rightFront.setPower(0.4);
                }
                motors.stop();
                placeWobble();

                break;

            case ONE:

                MoveState(AutonomousMove.MOVEBLACK);

                MoveState(AutonomousMove.MOVEBLUE);

                MoveState(AutonomousMove.TURNLEFT);

//                MoveState(AutonomousMove.REVERSEBLUE);

                placeWobble();
                break;
            case FOUR:
                MoveState(AutonomousMove.MOVEBLACK);

                MoveState(AutonomousMove.MOVEBLUE);

                MoveState(AutonomousMove.TURNBACK);

                placeWobble();
                break;
        }

//        while (robot.distanceB.getDistance(DistanceUnit.MM) >= 400) {
//            robot.leftFront.setPower(-0.4);
//            robot.leftBack.setPower(-0.4);
//            robot.rightBack.setPower(-0.4);
//            robot.rightBack.setPower(-0.4);
//        }
//        while (robot.colourF.alpha() < 1000) {
//            motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
//        }
//
//        while ((robot.distanceR.getDistance(DistanceUnit.MM) <= 915))  //&& ringtargetIsNotVisible
//        {
//
//            motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
//        }
//
//        while (robot.distanceL.getDistance(DistanceUnit.MM) >= 150) {
////strafe LEFT
//            motors.driveStrafe(-45 * Math.PI / 180, 0.2, true);
//        }
//
//
//        if (FiringCount == 1) {
//            while (robot.colourF.alpha() < 2000) {
//                robot.leftFront.setPower(1);
//                robot.leftBack.setPower(1);
//                robot.rightFront.setPower(1);
//                robot.rightBack.setPower(1);
//
//            }
//        }
//        //Strafe R tot Distance R = 950mm
//
//        while (robot.distanceL.getDistance(DistanceUnit.MM) <= 900) {
//
//            motors.driveStrafe(135 * Math.PI / 180, 0.4, true);
//        }
//Kyk vir die wit lyn
//        while (robot.colourF.alpha() < 2000) {
//
//            robot.leftFront.setPower(1);
//            robot.leftBack.setPower(1);
//            robot.rightFront.setPower(1);
//            robot.rightBack.setPower(1);
//
//        }
//        // Vanaf die wit lyn, kyk vir die grys
//
//        while (robot.colourF.alpha() > 2000) {
//            robot.leftFront.setPower(1);
//            robot.leftBack.setPower(1);
//            robot.rightFront.setPower(1);
//            robot.rightBack.setPower(1);
//
//        }
//        //Vanaf die grys, kyk weer vir die wit/blou lyn
//        while (robot.colourF.alpha() < 2000) {
//            robot.leftFront.setPower(1);
//            robot.leftBack.setPower(1);
//            robot.rightFront.setPower(1);
//            robot.rightBack.setPower(1);
//        }


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

    void placeWobble() {
        while (robot.wobbleMotor.getCurrentPosition() < 350 && opModeIsActive()) {
            robot.wobbleMotor.setPower(0.5);
        }
            robot.wobbleMotor.setPower(0);

        robot.wobbleF.setPosition(0.3);
        robot.wobbleB.setPosition(0.2);
        for (int i = 0; i < 10000; i++) {
            robot.leftBack.setPower(0.4);
            robot.leftFront.setPower(0.4);
            robot.rightBack.setPower(0.4);
            robot.rightFront.setPower(0.4);
        }
        motors.stop();
        while (robot.wobbleMotor.getCurrentPosition() > 250 && opModeIsActive()) {
            robot.wobbleMotor.setPower(-0.5);
        }
            robot.wobbleMotor.setPower(0);

    }

    /**
     * Returns an estimation of the horizontal angle to the detected object.
     */
    void estimateAngleToObject(AngleUnit angleUnit) {

    }

    private void MoveState(AutonomousMove DriveTrain) {

        switch (DriveTrain) {
            case MOVEBLACK:
                while (robot.colourF.alpha() >2000 && opModeIsActive()) {
                    robot.leftFront.setPower(0.2);
                    robot.leftBack.setPower(0.2);
                    robot.rightBack.setPower(0.2);
                    robot.rightFront.setPower(0.2);
                }
                motors.stop();


                break;

            case MOVEBLUE:

                while (robot.colourF.alpha() <2000 && opModeIsActive()) {
                    robot.leftFront.setPower(0.2);
                    robot.leftBack.setPower(0.2);
                    robot.rightBack.setPower(0.2);
                    robot.rightFront.setPower(0.2);
                }
                motors.stop();

                break;

            case REVERSEBLUE:

                while (robot.colourF.alpha() <2000 && opModeIsActive()) {
                    robot.leftFront.setPower(-0.4);
                    robot.leftBack.setPower(-0.4);
                    robot.rightBack.setPower(-0.4);
                    robot.rightFront.setPower(-0.4);
                }
                motors.stop();

                break;
            case TURNLEFT:

//                var.resetAngle();
                motors.pidRotate.reset();
                motors.pidRotate.enable();


                motors.rotate(90);

                while (!motors.pidRotate.onTarget() && opModeIsActive())
                    motors.rotate(90);

                if (motors.pidRotate.onTarget())
                    telemetry.addData("IMU", var.getAngle());


                break;

            case TURNFRONT:

                motors.pidRotate.reset();
                motors.pidRotate.enable();


                motors.rotate(1);

                while (!motors.pidRotate.onTarget() && opModeIsActive())
                    motors.rotate(1);

                if (motors.pidRotate.onTarget())
                    telemetry.addData("IMU", var.getAngle());
                break;

            case TURNPOWER:

//                var.resetAngle();
                motors.pidRotate.reset();
                motors.pidRotate.enable();


                motors.rotate(-4);

                while (!motors.pidRotate.onTarget() && opModeIsActive())
                    motors.rotate(-4);

                    telemetry.addData("IMU", var.getAngle());
                sleep(500);


                break;

            case TURNBACK:
                motors.pidRotate.reset();
                motors.pidRotate.enable();

                motors.rotate(180);

                while (!motors.pidRotate.onTarget() && opModeIsActive())
                    motors.rotate(180);

                if (motors.pidRotate.onTarget())
                    telemetry.addData("IMU", var.getAngle());
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
