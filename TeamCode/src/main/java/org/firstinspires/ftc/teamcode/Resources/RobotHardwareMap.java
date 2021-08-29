package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class RobotHardwareMap {
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor wobbleMotor;
//    public DcMotor liftMotor;
    public DcMotor shooterMotor;
    public DcMotor sucker;
    public Servo wobbleB;
    public Servo wobbleF;


    public DcMotor shooterAngleMotor;
    public DistanceSensor distanceR;
    public DistanceSensor distanceL;
    public DistanceSensor distanceFL;
    public DistanceSensor distanceFR;
    public DistanceSensor distanceFM;
    public DistanceSensor distanceB;
    public DigitalChannel downLimit;
    public BNO055IMU imu;

    public Servo RBG;
    public Servo LBG;
    public Servo magazineLifter;
    public Servo Tap;
    public RevColorSensorV3 colourF;
public RevColorSensorV3 colourL;
    public RevColorSensorV3 colourR;

    public RobotHardwareMap(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap){

        wobbleF = hardwareMap.get(Servo.class, "wobbleF");
        wobbleB = hardwareMap.get(Servo.class, "wobbleB");
        Tap = hardwareMap.get(Servo.class, "Tap");
        magazineLifter = hardwareMap.get(Servo.class, "magazineLifter");
//
//        RBG = hardwareMap.get(Servo.class, "RBG");
//        LBG = hardwareMap.get(Servo.class, "LBG");
//        shooterAngleServo = hardwareMap.get(Servo.class, "shooterAngleServo");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        initializeIMU();

        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceB = hardwareMap.get(DistanceSensor.class, "distanceB");
//        distanceFL = hardwareMap.get(DistanceSensor.class, "distanceFR");
//        distanceFR = hardwareMap.get(DistanceSensor.class, "distanceFL");
//        distanceFM = hardwareMap.get(DistanceSensor.class, "distanceFM");

        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");

        shooterAngleMotor = hardwareMap.get(DcMotor.class, "shooterAngleMotor");
//        liftMotor = hardwareMap.get(DcMotor.class, "motorHeight");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        sucker = hardwareMap.get(DcMotor.class, "sucker");


//        downLimit = hardwareMap.get(DigitalChannel.class, "downLimit");
//        downLimit.setMode(DigitalChannel.Mode.INPUT);
//
//        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
//        sucker.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront.setDirection(DcMotor.Direction.REVERSE);
//       rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.REVERSE);

          colourF = hardwareMap.get(RevColorSensorV3.class, "colourF");

//        colourR = hardwareMap.get(RevColorSensorV3.class, "colourR");

//        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enableEncoders();
    }

    private void enableEncoders(){

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void initializeIMU(){
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


    }
}
