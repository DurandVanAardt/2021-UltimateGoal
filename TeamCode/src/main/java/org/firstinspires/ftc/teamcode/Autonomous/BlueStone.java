package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.Resources.Stone;

import java.util.ArrayList;
import java.util.List;
//import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name="BlueStone", group ="Autonomous")

public class BlueStone extends LinearOpMode {

//    private ElapsedTime runtime = new ElapsedTime();
//
//    private static final double     LIFT_COUNTS_PER_MOTOR_REV = 7;
//    private static final double     LIFT_GEAR_REDUCTION = 20;
//    private static final double     LIFT_COUNTS_PER_MM = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION / 5);

    private static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 40;//3.7 ; //40    // This is < 1.0 if geared UP 40:1 reduce to 160 rpm
    private static final double     WHEEL_DIAMETER_MM   = 100 ;     // For figuring circumference
    private static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);
    //    static final double     DRIVE_SPEED             = 1;
//    static final double     TURN_SPEED              = 0.65;
//    static final double     STRAFE_SPEED            = 0.5;
    RobotHardwareMap robot = new RobotHardwareMap(hardwareMap);

    private Stone state = Stone.DRIVE_STONE;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
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
            "AYSD/P//////AAABmR1zVmwX0kdckb8df7pS1fRsEtj6VXa2Pxmx7PDDfAespnbdYju+FFSuSw6soY6xdrnsS263KCGQQhzpOl7rM8ljS5bqY24sIv1RblWBcEbkVQZ9+F2mAT+75KZBGlmAcss3ccaEj+xjEVftZuiW7CZ/DAgfNUIYdjdWJe8zSB22xS6YRa3nQ4JQ4PopaWI2D/kDF5KghljXKWvdqltKgyVGY3AKhIahUsXAx18hCJt9TExs60MSVQSNQH7tQC9OenKEysMc3481HB56Xaih8TTobKcexXeeZDCVQ6lDfxoaZaVDJir5K5ptLFNIACoDpbgxwgbFJ14JZiAAhOMVZ4pusmWcSSOAdido5x0ZlxT1";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
//    private OpenGLMatrix lastLocation = null;

    private float phoneXRotate    = 0;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    //    private boolean stoneGot;
    private boolean strafeStart;
    //    private int con;
    private int count = 0;
    private double distance;
    private double stone2Distance;
    private boolean liftUp;
    private double stoneDistanceR;
    private int position;
    private boolean third;

    @Override
    public void runOpMode() {
        robot.Claw.setPosition(0.1);

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BNO055IMU.Parameters parameters_1 = new BNO055IMU.Parameters();
//
//        parameters_1.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters_1.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters_1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters_1.loggingEnabled      = true;
//        parameters_1.loggingTag          = "IMU";
//        parameters_1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters_1);
        /*
         * Retrieve the camera we are to use.
         */
        /*
         * This is the webcam we are to use. As with other hardware devices such as motors and
         * servos, this device is identified using the robot configuration tool in the FTC application.
         */
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =VUFORIA_KEY;

        /*
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName =webcamName;

        //  Instantiate the Vuforia engine
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<>(targetsSkyStone);

        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0,0,stoneZ)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,-90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX,bridgeY,bridgeZ)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0,bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX,bridgeY,bridgeZ)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0,-bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX,-bridgeY,bridgeZ)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0,-bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX,-bridgeY,bridgeZ)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0,bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField,-halfField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField,-halfField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField,-quadField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField,quadField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField,halfField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField,halfField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField,quadField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,-90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField,-quadField,mmTargetHeight)
                .

                        multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,0,-90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        float phoneYRotate;
        if(CAMERA_CHOICE ==BACK)

        {
            phoneYRotate = -90;
        } else

        {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if(PHONE_IS_PORTRAIT)

        {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 20;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 11;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        float phoneZRotate = 90;
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /*  Let all the trackable listeners know where the phone is.  */
        for(
                VuforiaTrackable trackable : allTrackables)

        {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        waitForStart();
        boolean reset = true;
        boolean lf, rb, lb, rf;
        boolean result;
        boolean result1 = false;
        double pos;
        double pos2;
        boolean x = true;
        targetsSkyStone.activate();

//        state = Stone.STRAFESTONE1;
        while (opModeIsActive()){
            double v1 = 0.5;
            double v2 = 0.5;
            double v3 = 0.5;
            double v4 = 0.5;

            switch (state) {

                case DRIVE_STONE:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }
                    pos2 = (450) * COUNTS_PER_MM;
                    pos = (350) * COUNTS_PER_MM;

                    if (!result1) {


                        robot.leftFront.setPower(1);
                        robot.rightFront.setPower(1);
                        robot.leftBack.setPower(1);
                        robot.rightBack.setPower(1);

                        lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                        rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                        lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                        rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                        result1 = (lf && rf && lb && rb);

                        if (!result1)
                            break;
                    }
                    robot.leftFront.setPower(0.5);
                    robot.rightFront.setPower(0.5);
                    robot.leftBack.setPower(0.5);
                    robot.rightBack.setPower(0.5);

                    lf = (robot.leftFront.getCurrentPosition() > pos2 || robot.leftFront.getCurrentPosition() < -pos2);
                    rf = (robot.rightFront.getCurrentPosition() > pos2 || robot.rightFront.getCurrentPosition() < -pos2);
                    lb = (robot.leftBack.getCurrentPosition() > pos2 || robot.leftBack.getCurrentPosition() < -pos2);
                    rb = (robot.rightBack.getCurrentPosition() > pos2 || robot.rightBack.getCurrentPosition() < -pos2);

                    result = (lf && rf && lb && rb);
                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    // state = Stone./*GETSTONE*/STOP;
                    state = Stone.STRAFESTONE1;

                    reset = true;
                    break;


                case STRAFESTONE1:

                    if (!strafeStart) {
//                        resetIMU();
                        strafeStart = true;
                    }

                    if (getAngle() > 0) //turn anticlockwise
                    {
                        v1 += 0.09;
                        v2 += 0.09;
                        v3 += -0.09;
                        v4 += -0.09;
                    } else if (getAngle() < 0) {
                        v1 += -0.09;
                        v2 += -0.09;
                        v3 += 0.09;
                        v4 += 0.09;
                    }
//                    if (robot.distanceB.getDistance(DistanceUnit.MM) < wallDistance){
//                        v2 += -0.06;
//                        v3 += -0.06;
//                    }else if (robot.distanceB.getDistance(DistanceUnit.MM) < wallDistance){
//                        v1 += -0.06;
//                        v4 += -0.06;
//                    }
                    if (robot.leftFront.getPower() == 0 && robot.rightFront.getPower() == 0 && robot.leftBack.getPower() == 0 && robot.rightBack.getPower() == 0) {
                        robot.leftFront.setPower(v1 / 2);
                        robot.rightFront.setPower(-v2 /2 );
                        robot.leftBack.setPower(-v3);
                        robot.rightBack.setPower(v4);
                    }

                    robot.leftFront.setPower(v1);
                    robot.rightFront.setPower(-v2);
                    robot.leftBack.setPower(-v3);
                    robot.rightBack.setPower(v4);


//                    if (y != 50){
//                        z += robot.distanceR.getDistance(DistanceUnit.MM);
//                        y ++;
//                    } else {
//                        z /= 50;
//                    }
                    //    double z;
                    boolean stop = robot.distanceR.getDistance(DistanceUnit.MM) <= 203.2 * 6 - 120;

//                    if (y == 50)
//                        z = 0;

                    if (!stop)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.FINDSTONE;
                    strafeStart = false;
                    break;

                case STRAFESTONE2:

                    if (!strafeStart) {
                        resetIMU();
                        strafeStart = true;
                    }

                    if (getAngle() > 0) //turn anticlockwise
                    {
                        v1 += 0.09;
                        v2 += 0.09;
                        v3 += -0.09;
                        v4 += -0.09;
                    } else if (getAngle() < 0) {
                        v1 += -0.09;
                        v2 += -0.09;
                        v3 += 0.09;
                        v4 += 0.09;
                    }
                    if (robot.leftFront.getPower() == 0 && robot.rightFront.getPower() == 0 && robot.leftBack.getPower() == 0 && robot.rightBack.getPower() == 0) {
                        robot.leftFront.setPower(v1 / 2);
                        robot.rightFront.setPower(-v2 /2 );
                        robot.leftBack.setPower(-v3);
                        robot.rightBack.setPower(v4);
                    }

                    robot.leftFront.setPower(v1);
                    robot.rightFront.setPower(-v2);
                    robot.leftBack.setPower(-v3);
                    robot.rightBack.setPower(v4);

//                    if (y != 50){
//                        z += robot.distanceL.getDistance(DistanceUnit.MM);
//                        y ++;
//                    } else {
//                        z /= 50;
//                    }
                    stop = robot.distanceR.getDistance(DistanceUnit.MM) <= 203.2 * 5 - 130;

//                    if (y == 50)
//                        z = 0;

                    if (!stop)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.FINDSTONE;

                    break;

                case STRAFESTONE3:

                    if (!strafeStart) {
                        resetIMU();
//                        int y = 0;
                        strafeStart = true;
                    }

                    if (getAngle() > 0) //turn anticlockwise
                    {
                        v1 += 0.09;
                        v2 += 0.09;
                        v3 += -0.09;
                        v4 += -0.09;
                    } else if (getAngle() < 0) {
                        v1 += -0.09;
                        v2 += -0.09;
                        v3 += 0.09;
                        v4 += 0.09;
                    }
                    if (robot.leftFront.getPower() == 0 && robot.rightFront.getPower() == 0 && robot.leftBack.getPower() == 0 && robot.rightBack.getPower() == 0) {
                        robot.leftFront.setPower(v1 / 2);
                        robot.rightFront.setPower(-v2 /2 );
                        robot.leftBack.setPower(-v3);
                        robot.rightBack.setPower(v4);
                    }

                    robot.leftFront.setPower(v1);
                    robot.rightFront.setPower(-v2);
                    robot.leftBack.setPower(-v3);
                    robot.rightBack.setPower(v4);

//                    if (y != 50){
//                        z += robot.distanceL.getDistance(DistanceUnit.MM);
//                        y ++;
//                    } else {
//                        z /= 50;
//                    }
                    stop = robot.distanceR.getDistance(DistanceUnit.MM) <= 203.2 * 4 - 120;

//                    if (y == 50)
//                        z = 0;

                    if (!stop)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.FINDSTONE;

                    break;


                case FINDSTONE:
                    boolean stone = false;
                    count += 1;
                    int hi;
                    if (count != 3) {
                        for (hi = 0; hi <= 25250; hi++) {
                            for (VuforiaTrackable trackable : allTrackables) {
                                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                                    stone = trackable.getName().equals("Stone Target");
                                    break;
                                }

                            }
                            if (stone)
                                break;
                        }
                    }

                    if (stone || count == 3)
                    { distance = robot.distanceFM.getDistance(DistanceUnit.MM) - 10;
                        stone2Distance = robot.distanceR.getDistance(DistanceUnit.MM) - 203.2 * 3;
                        state = Stone.DRIVETOSTONE;
                        third = count == 3;
                        break;
                    }else {
                        if (count == 1){
                            state = Stone.STRAFESTONE2;
                            break;
                        }
                        if (count == 2){
                            state = Stone.STRAFESTONE3;
                            break;
                        }

                    }

                case DRIVETOSTONE:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;

                    }

                    pos = (distance-50) * COUNTS_PER_MM;

                        robot.leftFront.setPower(0.5);
                        robot.rightFront.setPower(0.5);
                        robot.leftBack.setPower(0.5);
                        robot.rightBack.setPower(0.5);

                        lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                        rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                        lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                        rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                        result1 = (lf && rf && lb && rb);

                        if (result1)
                            break;


                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.GETSTONE;

                    reset = true;
                    break;

                case GETSTONE:

                    robot.Claw.setPosition(0.8);

//                    if (con == 100)
//                        break;
                    this.sleep(750);
                    state = Stone.DRIVE_BACK1;
                    break;

                case DRIVE_BACK1:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }
                    pos = (100) * COUNTS_PER_MM;

                    robot.leftFront.setPower(-1);
                    robot.rightFront.setPower(-1);
                    robot.leftBack.setPower(-1);
                    robot.rightBack.setPower(-1);

                    lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                    rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                    lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                    rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                    result = (lf && rf && lb && rb);

                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.TURN901;
                    reset = true;
                    break;

                case TURN901:
                    if (x) {
                        robot.leftFront.setPower(-1);
                        robot.rightFront.setPower(1);
                        robot.leftBack.setPower(-1);
                        robot.rightBack.setPower(1);
                        resetIMU();
                        x = false;
                    }
                    if (getAngle() >= 65){
                        robot.leftFront.setPower(-0.5);
                        robot.rightFront.setPower(0.5);
                        robot.leftBack.setPower(-0.5);
                        robot.rightBack.setPower(0.5);
                    }
                    if (getAngle() >= 75){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        distance = 2100 - robot.distanceB.getDistance(DistanceUnit.MM);
                            state = Stone.DRIVE_BRIDGE;
                        x = true;
                    }
                    break;

                case DRIVE_BRIDGE:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }
                    pos = (distance) * COUNTS_PER_MM;

                    robot.leftFront.setPower(1);
                    robot.rightFront.setPower(1);
                    robot.leftBack.setPower(1);
                    robot.rightBack.setPower(1);

                    lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                    rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                    lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                    rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                    result = (lf && rf && lb && rb);

                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.DROPSTONE;
                    reset = true;
                    break;

                case DROPSTONE:
                    robot.Claw.setPosition(0.1);
                        state = Stone.DRIVEBACKSTONE2;
                        this.sleep(500);
                    break;

                case DRIVEBACKSTONE2:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }

                    if (third) {
                        pos = (distance - 600) * COUNTS_PER_MM;
                        position = 1;
                    }
                    else {
                        pos = (distance + 500) * COUNTS_PER_MM;
                        position = 2;
                    }

                    robot.leftFront.setPower(-1);
                    robot.rightFront.setPower(-1);
                    robot.leftBack.setPower(-1);
                    robot.rightBack.setPower(-1);

                    lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                    rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                    lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                    rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                    result = (lf && rf && lb && rb);

                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.TURN_BACK90;
                    reset = true;
                    break;

                case TURN_BACK90:

                    if (x) {
                        resetIMU();
                        robot.leftFront.setPower(1);
                        robot.rightFront.setPower(-1);
                        robot.leftBack.setPower(1);
                        robot.rightBack.setPower(-1);
                        x = false;
                    }
                    if (getAngle() <= 20){
                        robot.leftFront.setPower(0.5);
                        robot.rightFront.setPower(-0.5);
                        robot.leftBack.setPower(0.5);
                        robot.rightBack.setPower(-0.5);
                    }
                    if (getAngle() <= 10){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        state = Stone.STRAFE_STONE2;
                        x = true;
                    }
                    break;

                case STRAFE_STONE2:

                    if (!strafeStart) {
                        resetIMU();
                        strafeStart = true;
                    }
                    if (position == 2)
                        stop = robot.distanceR.getDistance(DistanceUnit.MM) <= stone2Distance;
                    else
                        stop = robot.distanceR.getDistance(DistanceUnit.MM) <= 203.2 * 6 - 50;

                    if (!stop) {
                        if (getAngle() > 0) //turn anticlockwise
                        {
                            v1 += 0.15;
                            v2 += 0.15;
                            v3 += -0.15;
                            v4 += -0.15;
                        } else if (getAngle() < 0) {
                            v1 += -0.15;
                            v2 += -0.15;
                            v3 += 0.15;
                            v4 += 0.15;
                        }
//                    if (robot.leftFront.getPower() == 0 && robot.rightFront.getPower() == 0 && robot.leftBack.getPower() == 0 && robot.rightBack.getPower() == 0) {
//                        robot.leftFront.setPower(v1 * 1);
//                        robot.rightFront.setPower(-v2 * 1);
//                        robot.leftBack.setPower(-v3 / 2);
//                        robot.rightBack.setPower(v4 / 2);
//                    }

                        robot.leftFront.setPower(v1);
                        robot.rightFront.setPower(-v2);
                        robot.leftBack.setPower(-v3);
                        robot.rightBack.setPower(v4);
                    }
                    if (position == 2)
                        stop = robot.distanceR.getDistance(DistanceUnit.MM) <= stone2Distance;
                    else
                        stop = robot.distanceR.getDistance(DistanceUnit.MM) <= 203.2 * 6 - 150;

                    if (!stop)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    distance = robot.distanceFM.getDistance(DistanceUnit.MM) - 15 ;
                    state = Stone.DRIVETOSTONE2;
                    break;

                case DRIVETOSTONE2:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }
                    pos2 = (distance) * COUNTS_PER_MM;
                    pos = (distance-50) * COUNTS_PER_MM;

                    if (!result1) {


                        robot.leftFront.setPower(0.5);
                        robot.rightFront.setPower(0.5);
                        robot.leftBack.setPower(0.5);
                        robot.rightBack.setPower(0.5);

                        lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                        rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                        lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                        rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                        result1 = (lf && rf && lb && rb);

                        if (!result1)
                            break;
                    }
                    robot.leftFront.setPower(0.2);
                    robot.rightFront.setPower(0.2);
                    robot.leftBack.setPower(0.2);
                    robot.rightBack.setPower(0.2);

                    lf = (robot.leftFront.getCurrentPosition() > pos2 || robot.leftFront.getCurrentPosition() < -pos2);
                    rf = (robot.rightFront.getCurrentPosition() > pos2 || robot.rightFront.getCurrentPosition() < -pos2);
                    lb = (robot.leftBack.getCurrentPosition() > pos2 || robot.leftBack.getCurrentPosition() < -pos2);
                    rb = (robot.rightBack.getCurrentPosition() > pos2 || robot.rightBack.getCurrentPosition() < -pos2);

                    result = (lf && rf && lb && rb);
                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    // state = Stone./*GETSTONE*/STOP;
                    state = Stone.GETSTONE2;

                    reset = true;
                    break;


                case GETSTONE2:
                    robot.Claw.setPosition(0.8);
                    this.sleep(750);
                    state = Stone.DRIVE_BACK2;
                    break;

                case DRIVE_BACK2:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }
                    pos = (100) * COUNTS_PER_MM;

                    robot.leftFront.setPower(-1);
                    robot.rightFront.setPower(-1);
                    robot.leftBack.setPower(-1);
                    robot.rightBack.setPower(-1);

                    lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                    rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                    lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                    rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                    result = (lf && rf && lb && rb);

                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.TURN903;
                    reset = true;
                    break;

                case TURN903:
                    resetIMU();

                    if (x) {
                        robot.leftFront.setPower(-1);
                        robot.rightFront.setPower(1);
                        robot.leftBack.setPower(-1);
                        robot.rightBack.setPower(1);
                        x = false;
                    }
                    if (getAngle() >= 65){
                        robot.leftFront.setPower(-0.5);
                        robot.rightFront.setPower(0.5);
                        robot.leftBack.setPower(-0.5);
                        robot.rightBack.setPower(0.5);
                    }
                    if (getAngle() >= 75){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        distance = 2000 - robot.distanceB.getDistance(DistanceUnit.MM);
                        state = Stone.DRIVE_BRIDGE2;
                        x = true;
                    }
                    break;

                case DRIVE_BRIDGE2:

                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }
                    pos = (distance) * COUNTS_PER_MM;

                    robot.leftFront.setPower(1);
                    robot.rightFront.setPower(1);
                    robot.leftBack.setPower(1);
                    robot.rightBack.setPower(1);

                    lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                    rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                    lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                    rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                    result = (lf && rf && lb && rb);

                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.DROPSTONE2;
                    reset = true;
                    break;

                case DROPSTONE2:
                    robot.Claw.setPosition(0.1);
                    state = Stone.DRIVEBRIDGE;
                    this.sleep(1000);
                    break;

                case DRIVEBRIDGE:


                    if (reset) {
                        ResetEncoders();
                        enableEncoders();
                        reset = false;
                    }

                    pos = 500 * COUNTS_PER_MM;

                    robot.leftFront.setPower(-1);
                    robot.rightFront.setPower(-1);
                    robot.leftBack.setPower(-1);
                    robot.rightBack.setPower(-1);

                    lf = (robot.leftFront.getCurrentPosition() > pos || robot.leftFront.getCurrentPosition() < -pos);
                    rf = (robot.rightFront.getCurrentPosition() > pos || robot.rightFront.getCurrentPosition() < -pos);
                    lb = (robot.leftBack.getCurrentPosition() > pos || robot.leftBack.getCurrentPosition() < -pos);
                    rb = (robot.rightBack.getCurrentPosition() > pos || robot.rightBack.getCurrentPosition() < -pos);

                    result = (lf && rf && lb && rb);

                    if (!result)
                        break;

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    state = Stone.STOP;
                    reset = true;
                    break;

                case STOP:
                    break;
            }
        }
    }

    private void resetIMU(){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angles.firstAngle = 0;
    }

    private void enableEncoders(){

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    private void ResetEncoders(){
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


//    private double checkDirection()
//    {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//
//        double angle = getAngle();
//
//        double correction;
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        double gain = .10;
//        correction = correction * gain;
//
//        return correction;
//    }

    public double getAngle()
    {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


}
