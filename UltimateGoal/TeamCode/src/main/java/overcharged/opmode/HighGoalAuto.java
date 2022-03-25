package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.OcServo;
import overcharged.components.RotationAxis;
import overcharged.components.RingDetector;
import overcharged.components.RingPosition;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.test.EasyOpenCVExample;

import static overcharged.config.RobotConstants.TAG_A;

/*
 * Overcharged Team #12599 Autonomous
 */

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "HighGoalAuto", group = "Game")
public class HighGoalAuto extends LinearOpMode {
    /// Overcharged Autonomous Robot class
    private Robot6WheelLinear robot;
    /// Overcharged Swerve Drive class
    private TankDriveLinear drive;
    /// OpenCV Ring detector and webcam
    RingDetector detector;
    OpenCvWebcam webcam;

    DcMotorEx flywheel_motor1;
    DcMotorEx flywheel_motor2;
    PIDFCoefficients flywheel_pid;
    //DcMotorEx wobbleArm;
    PIDFCoefficients wobble_pid;
    //DcMotorEx intake;
    DcMotorEx intakeL;
    DcMotorEx intakeR;
    PIDFCoefficients intake_pid;

    OcServo claw;
    float clawOpen = 63/255f;
    float clawClosed = 215/255f;

    OcServo slide;

    float slideUp = 97/255f;
    float slideDown = 171/255f;
    float slidePos1 = 142/255f;
    float slidePos2 = 120/255f;

    OcServo wobble;

    float wobbleInit = 100/255f;
    float wobbleUp = 160/255f;
    float wobbleDown = 255/255f;

    OcServo stopper;
    float stopperOpen = 150/255f;
    float stopperClosed = 14/255f;

    OcServo barL;
    OcServo barR;
    float barStartL = 28/255f;
    float barStartR = 225/255f;
    float barInit = 10/255f;
    float barLower = 190/255f;
    float barUpL = barStartL-barInit;
    float barUpR = barStartR+barInit;
    float barDownL = barStartL+barLower;
    float barDownR = barStartR-barLower;

    ///Constants
    public final float intake_power = 0.7f;
    final float power = .30f;
    final float movepower = .70f;
    final float intakePower = 1f;//0.7105f;
    final float wobblePower = -0.6f;
    final float slowPower = .20f;
    final float fastPower = 0.50f;
    final float turnPower = .30f;
    private boolean isLeft = true;
    private RingPosition ringPosition = RingPosition.A;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    /**
     * Autonomous opMode, entry point into the program
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // init
            robot = new Robot6WheelLinear(this);
            RobotLog.ii(TAG_A, "Robot6WheelLinear initialized");
            drive = robot.getTankDriveLinear();
            RobotLog.ii(TAG_A, "TankDriveLinear initialized");
            initCamera();
            slide = new OcServo(hardwareMap, "slideServo", slideDown);
            claw = new OcServo(hardwareMap, "clawServo", clawClosed);
            wobble = new OcServo(hardwareMap, "wobbleServo", wobbleInit);
            stopper = new OcServo(hardwareMap, "stopperServo", stopperClosed);
            barL = new OcServo(hardwareMap, "barServoL", barUpL);
            barR = new OcServo(hardwareMap, "barServoR", barUpR);
            intakeL = hardwareMap.get(DcMotorEx.class, "intakeL");
            intakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            intakeL.setDirection(DcMotorEx.Direction.FORWARD);

            intakeR = hardwareMap.get(DcMotorEx.class, "intakeR");
            intakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            intakeR.setDirection(DcMotorEx.Direction.REVERSE);

            intake_pid = new PIDFCoefficients(800,20,20,5);
            intakeL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intake_pid);
            intakeR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intake_pid);

            flywheel_pid = new PIDFCoefficients(1080, 65, 15, 15);

            flywheel_motor1 = hardwareMap.get(DcMotorEx.class, "fm1");
            flywheel_motor2 = hardwareMap.get(DcMotorEx.class, "fm2");

            flywheel_motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel_motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            flywheel_motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pid);
            flywheel_motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pid);

            flywheel_motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel_motor2.setDirection(DcMotorSimple.Direction.FORWARD);
            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * Camera Initialization Function
     */
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
    }

    /**
     * Set flywheel power
     */
    private void setFlywheelPower(float pwr){
        flywheel_motor1.setPower(pwr);
        flywheel_motor2.setPower(pwr);
    }

    /**
     * Main movement method
     * @param lp WaitLinear object used for pauses
     * @param ringPosition The detected position of the rings
     * @param isRight Where the robot starts (Right or left of the rings)
     * @param startTime Current time upon entering the function
     * @throws InterruptedException
     */
    private void move(WaitLinear lp, RingPosition ringPosition, boolean isRight, long startTime) throws InterruptedException {
        int distance; //Go to wobble goal square first time
        int turnAngle; //Turn towards wobble goal square first time
        int turnAngle2; //Turn towards stack
        int distance2; //Move towards stack
        int turnAngle3; //Turn towards wobble goal square second time

        if (ringPosition == RingPosition.A) {
            distance = -40;
            turnAngle = -41;
            turnAngle2 = 0;
            distance2 = 0;
            turnAngle3 = -99;//-100
        } else if (ringPosition == RingPosition.B) {
            distance = -37;
            turnAngle = -10;
            turnAngle2 = 28;
            distance2 = 45;
            turnAngle3 = 142;
        } else {
            distance = -60;
            turnAngle = -15;
            turnAngle2 = 12;
            distance2 = 66;
            turnAngle3 = 144;
        }

        setFlywheelPower(0.72f); //0.72
        //wobbleArm.setPower(-(wobblePower-0.2));
        //lp.waitMillis(350);
        //wobbleArm.setPower(0);
        wobble.setPosition(wobbleUp);

        drive.moveToEncoderInchUsingPID(-50, movepower-0.2f, 3000, true);
        drive.turnUsingPID(-9.4,power,RotationAxis.RIGHT);
        highGoal(lp);

        drive.turnUsingPID(turnAngle,power,RotationAxis.CENTER);
        if (ringPosition == RingPosition.C) {
            drive.moveToEncoderInchUsingPID(distance-3, movepower, 6000, true);
        }
        else{
            drive.moveToEncoderInchUsingPID(distance+13, movepower, 6000, true);
        }

        //wobbleArm.setPower(-wobblePower);
        //lp.waitMillis(1350);
        //wobbleArm.setPower(0);
        wobble.setPosition(wobbleDown);
        lp.waitMillis(800);
        claw.setPosition(clawOpen);
        lp.waitMillis(300);
        wobble.setPosition(wobbleUp);

        if (ringPosition == RingPosition.B || ringPosition == RingPosition.C) {
            setFlywheelPower(0.7f);//0.577
            if (ringPosition == RingPosition.B){
                drive.turnUsingPID(turnAngle2,power,RotationAxis.RIGHT);
            }
            else{
                drive.turnUsingPID(turnAngle2-7,power,RotationAxis.CENTER);
            }
            intakeR.setPower(intakePower);
            intakeL.setPower(intakePower);
            if (ringPosition == RingPosition.C) {
                barL.setPosition(barDownL);
                barR.setPosition(barDownR);
            }

            drive.moveToEncoderInchUsingPID(distance2-20, movepower-0.2f, 6000, true);

            if (ringPosition == RingPosition.C) {
                drive.turnUsingPID(19,power-0.2f,RotationAxis.CENTER);//12
            }
            else {
                drive.turnUsingPID(-0.7,power,RotationAxis.CENTER);
            }

            /*if (ringPosition == RingPosition.C) {
                barL.setPosition(barUpL);
                barR.setPosition(barUpR);
            }*/
            drive.moveToEncoderInchUsingPID(19, movepower, 3000, true);

            lp.waitMillis(350);
            drive.moveToEncoderInchUsingPID(-10, movepower, 3000, true);
            highGoal(lp);

            if (ringPosition == RingPosition.C){
                drive.moveToEncoderInchUsingPID(24, movepower-0.3f, 3000, true);
                //drive.moveToEncoderInchUsingPID(4, power, 3000, true);
                //lp.waitMillis(200);
                //intake.setPower(0f);

                drive.moveToEncoderInchUsingPID(-15, movepower, 3000, true);
                highGoal(lp);
            }
            /*if (ringPosition == RingPosition.C){
                drive.moveToEncoderInchUsingPID(19, power, 3000, true);
                lp.waitMillis(900);
                drive.moveToEncoderInchUsingPID(-20, movepower-0.2f, 3000, true);
                highGoal(lp);
            }*/
            setFlywheelPower(0f);
            intakeR.setPower(0);
            intakeL.setPower(0);

            wobble.setPosition(wobbleDown);
            if (ringPosition == RingPosition.C){
                drive.turnUsingPID(-154,power,RotationAxis.CENTER);
            }
            else {
                drive.turnUsingPID(-152,power,RotationAxis.CENTER);
            }
            if (ringPosition == RingPosition.B){
                drive.moveToEncoderInchUsingPID(-18, power+0.1f, 3000, true);
            }
            else{
                drive.moveToEncoderInchUsingPID(-18f, power+0.1f, 3000, true);
            }
            claw.setPosition(clawClosed);
            lp.waitMillis(800);
            wobble.setPosition(wobbleUp);
            //wobbleArm.setPower(wobblePower);
            //lp.waitMillis(700);
            //wobbleArm.setPower(0);
            //wobbleGoalGrab(lp);

            drive.turnUsingPID(turnAngle3,power,RotationAxis.CENTER);
            if (ringPosition == RingPosition.C) {
                drive.moveToEncoderInchUsingPID(distance-7, movepower, 6000, true);
            }
            else{
                drive.moveToEncoderInchUsingPID(distance-5, movepower, 6000, true);
            }
            wobbleGoalDrop(lp);

            if (ringPosition == RingPosition.C){
                drive.moveToEncoderInchUsingPID(-distance-36, movepower+0.1f, 3000, true);
            }
        }
        else {
            setFlywheelPower(0f);
            intakeR.setPower(0);
            intakeL.setPower(0);
            drive.turnUsingPID(turnAngle3,power,RotationAxis.CENTER);
            wobble.setPosition(wobbleDown);
            drive.moveToEncoderInchUsingPID(distance+9f, movepower, 3000, true);
            claw.setPosition(clawClosed);
            lp.waitMillis(800);
            wobble.setPosition(wobbleUp);
            //wobbleArm.setPower(wobblePower);
            //lp.waitMillis(700);
            //wobbleArm.setPower(0);

            drive.turnUsingPID(140,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(-20f, movepower, 3000, true);
            wobbleGoalDrop(lp);
            drive.moveToEncoderInchUsingPID(3, movepower, 3000, true);
            drive.turnUsingPID(-130,power+0.2,RotationAxis.RIGHT);
            drive.moveToEncoderInchUsingPID(24, movepower, 3000, true);

        }

    }

    /**
     * Shoot the power shots
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void powerShots(WaitLinear lp) throws InterruptedException {
        slide.setPosition(slidePos1);
        lp.waitMillis(500);
        drive.turnUsingPID(-3.125,power,RotationAxis.CENTER);
        slide.setPosition(slidePos2);
        lp.waitMillis(500);
        drive.turnUsingPID(-3.125,power,RotationAxis.CENTER);
        slide.setPosition(slideUp);
        lp.waitMillis(500);
        setFlywheelPower(0f);
        slide.setPosition(slideDown);
    }

    /**
     * Shoot the rings into the high goal
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void highGoal(WaitLinear lp) throws InterruptedException {
        //setFlywheelPower(0.75f);
        stopper.setPosition(stopperOpen);
        lp.waitMillis(800);
        slide.setPosition(slideUp);
        lp.waitMillis(1100);
        //setFlywheelPower(0f);
        slide.setPosition(slideDown);
        stopper.setPosition(stopperClosed);
    }

    /**
     * Grab the wobble goal
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    /*private void wobbleGoalGrab(WaitLinear lp) throws InterruptedException {
        claw.setPosition(clawOpen);
        wobbleArm.setPower(-wobblePower);
        lp.waitMillis(600);
        wobbleArm.setPower(0);
        claw.setPosition(clawClosed);
        wobbleArm.setPower(wobblePower);
        lp.waitMillis(700);
        wobbleArm.setPower(0);
    }*/

    /**
     * Drop the wobble goal
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void wobbleGoalDrop(WaitLinear lp) throws InterruptedException {
        wobble.setPosition(wobbleDown);
        //wobbleArm.setPower(-wobblePower);
        //wobbleArm.setPower(0);
        lp.waitMillis(800);
        claw.setPosition(clawOpen);
        lp.waitMillis(600);

        wobble.setPosition(wobbleUp);
        //wobbleArm.setPower(wobblePower);
        //lp.waitMillis(500);
        //wobbleArm.setPower(0);
        claw.setPosition(clawClosed);
    }

    /**
     * Autonomous run function
     * This is the main function that performs all the actions at once
     * @throws InterruptedException
     */
    public void run() throws InterruptedException {
        WaitLinear lp = new WaitLinear(this);
        long currentTime;

        this.detector = new RingDetector(isLeft);
        this.detector.useDefaults();
        this.detector.setRedRange(0.63, 115, 28);
        this.detector.setBlueRange(0.75, 33, 120);
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        long time1 = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();
        while (currentTime - time1 < RingDetector.DETECTION_WAIT_TIME) {
            ringPosition = detector.getRingPosition();
            currentTime = System.currentTimeMillis();
        }
        //webcam.pauseViewport();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && robot.gyroSensor.isCalibrating())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Waiting", "Autonomous");
        telemetry.update();

        ///Wait for Start to be pressed
        RobotLog.v(TAG_A, "Wait for Start to be pressed");

        drive.resetAngle();
        waitForStart();

        //The time when the autonomous run started i.e. the start/stop button was pressed
        long startTime = System.currentTimeMillis();

        //webcam.pauseViewport();
        detector.reset();
        //webcam.resumeViewport();
        long time2 = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();
        while (currentTime - time2 < RingDetector.DETECTION_WAIT_TIME) {
            ringPosition = detector.getRingPosition();
            currentTime = System.currentTimeMillis();
        }

        telemetry.addData("Ring Position=", ringPosition);
        telemetry.update();

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        //return immediately if stop has been pressed instead of play
        if (isStopRequested()) {
            return;
        }

        ///If the user presses stop, waitForStart() will return, only run if the start button is pressed (not stop).
        if (opModeIsActive()) {
            move(lp, ringPosition, isLeft, startTime);
        }

        drive.stop();
        RobotLog.ii(TAG_A, "Autonomous finished in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        //wait a bit longer than the 30 seconds just in case the start was accidentally pressed earlier
        lp.waitMillis(32000 - (int)(System.currentTimeMillis()-startTime));
        ///End
    }
}