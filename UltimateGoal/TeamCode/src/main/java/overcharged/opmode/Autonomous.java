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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "OnePathAuto", group = "Game")
public class Autonomous extends LinearOpMode {
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
    DcMotorEx wobbleArm;
    PIDFCoefficients wobble_pid;
    DcMotorEx intake;
    PIDFCoefficients intake_pid;

    OcServo claw;
    float clawOpen = 147/255f;
    float clawKindaopen = 238/255f;
    float clawClosed = 254/255f;

    OcServo slide;
    float slideUp = 63/255f;
    float slideDown = 166/255f;
    float slidePos1 = 140/255f;
    float slidePos2 = 118/255f;

    ///Constants
    public final float intake_power = 0.7f;
    final float power = .30f;
    final float movepower = .70f;
    final float intakePower = 0.75f;
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

            /*intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake_pid = new PIDFCoefficients(500,0,0,0);
            intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);*/

            wobbleArm = hardwareMap.get(DcMotorEx.class, "arm");
            wobbleArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            wobble_pid = new PIDFCoefficients(500,0,0,0);
            wobbleArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            flywheel_pid = new PIDFCoefficients(1030, 70, 15, 15);

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
            distance = -30;
            turnAngle = -41;
            turnAngle2 = 0;
            distance2 = 0;
            turnAngle3 = -90;
        } else if (ringPosition == RingPosition.B) {
            distance = -37;
            turnAngle = -12;
            turnAngle2 = 25;
            distance2 = 45;
            turnAngle3 = 150;
        } else {
            distance = -60;
            turnAngle = -14;
            turnAngle2 = 13;
            distance2 = 66;
            turnAngle3 = 125;
        }

        setFlywheelPower(0.55f);
        wobbleArm.setPower(-(wobblePower-0.2));
        lp.waitMillis(350);
        wobbleArm.setPower(0);
        drive.turnUsingPID(4.75,power,RotationAxis.RIGHT);

        drive.moveToEncoderInchUsingPID(-48, movepower-0.2f, 3000, true);
        powerShots(lp);

        drive.turnUsingPID(turnAngle,power,RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(distance, movepower, 6000, true);
        wobbleArm.setPower(-wobblePower);
        lp.waitMillis(1350);
        wobbleArm.setPower(0);
        claw.setPosition(clawOpen);
        //wobbleGoalDrop(lp);

        if (ringPosition == RingPosition.B || ringPosition == RingPosition.C) {
            setFlywheelPower(0.65f);
            if (ringPosition == RingPosition.B){
                drive.turnUsingPID(turnAngle2,power,RotationAxis.RIGHT);
            }
            else{
                drive.turnUsingPID(turnAngle2,power,RotationAxis.CENTER);
            }
            intake.setPower(intakePower);
            drive.moveToEncoderInchUsingPID(distance2, movepower-0.2f, 6000, true);

            if (ringPosition == RingPosition.C) {
                drive.turnUsingPID(2,power,RotationAxis.CENTER);
            }

            drive.moveToEncoderInchUsingPID(5, movepower+0.2f, 3000, true);
            drive.moveToEncoderInchUsingPID(4, power, 3000, true);
            lp.waitMillis(900);
            //intake.setPower(0f);
            if (ringPosition == RingPosition.B){
                drive.turnUsingPID(-3,power,RotationAxis.CENTER);
            }

            drive.moveToEncoderInchUsingPID(-13, movepower, 3000, true);
            highGoal(lp);
            if (ringPosition == RingPosition.C){
                drive.moveToEncoderInchUsingPID(22, power, 3000, true);
                lp.waitMillis(900);
                drive.moveToEncoderInchUsingPID(-22, movepower-0.2f, 3000, true);
                highGoal(lp);
            }
            setFlywheelPower(0f);
            intake.setPower(0f);

            if (ringPosition == RingPosition.C){
                drive.turnUsingPID(-137,power,RotationAxis.CENTER);
            }
            else {
                drive.turnUsingPID(-134,power,RotationAxis.CENTER);
            }
            if (ringPosition == RingPosition.B){
                drive.moveToEncoderInchUsingPID(-22, power+0.1f, 3000, true);
            }
            else{
                drive.moveToEncoderInchUsingPID(-20, power+0.1f, 3000, true);
            }
            claw.setPosition(clawClosed);
            lp.waitMillis(400);
            wobbleArm.setPower(wobblePower);
            lp.waitMillis(700);
            wobbleArm.setPower(0);
            //wobbleGoalGrab(lp);

            drive.turnUsingPID(turnAngle3,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(distance-4, movepower+0.1f, 6000, true);
            wobbleGoalDrop(lp);

            if (ringPosition == RingPosition.C){
                drive.moveToEncoderInchUsingPID(-distance-35, movepower+0.1f, 3000, true);
            }
        }
        else {
            drive.turnUsingPID(turnAngle3,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(distance-5, movepower, 3000, true);
            claw.setPosition(clawClosed);
            lp.waitMillis(400);
            wobbleArm.setPower(wobblePower);
            lp.waitMillis(700);
            wobbleArm.setPower(0);

            drive.turnUsingPID(138,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(-26.5f, movepower, 3000, true);
            wobbleGoalDrop(lp);
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
        slide.setPosition(slideUp);
        lp.waitMillis(1000);
        //setFlywheelPower(0f);
        slide.setPosition(slideDown);
    }

    /**
     * Grab the wobble goal
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void wobbleGoalGrab(WaitLinear lp) throws InterruptedException {
        claw.setPosition(clawOpen);
        wobbleArm.setPower(-wobblePower);
        lp.waitMillis(600);
        wobbleArm.setPower(0);

        claw.setPosition(clawClosed);
        wobbleArm.setPower(wobblePower);
        lp.waitMillis(700);
        wobbleArm.setPower(0);
    }

    /**
     * Drop the wobble goal
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void wobbleGoalDrop(WaitLinear lp) throws InterruptedException {
        wobbleArm.setPower(-wobblePower);
        lp.waitMillis(600);
        wobbleArm.setPower(0);
        claw.setPosition(clawOpen);

        wobbleArm.setPower(wobblePower);
        lp.waitMillis(500);
        wobbleArm.setPower(0);
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
        this.detector.setRedRange(0.74, 115, 28);
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