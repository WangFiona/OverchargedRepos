package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.sql.RowId;

import overcharged.components.OcServo;
import overcharged.components.RotationAxis;
import overcharged.components.RingDetector;
import overcharged.components.RingPosition;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.test.EasyOpenCVExample;

import static overcharged.config.RobotConstants.TAG_A;

/*
 * Overcharged Team #12599 Autonomous
 */

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "UltimateGoalAuto1", group = "Game")
public class OptionsAuto extends LinearOpMode {
    /// Overcharged Autonomous Robot class
    private Robot6WheelLinear robot;
    /// Overcharged Swerve Drive class
    private TankDriveLinear drive;
    /// OpenCV Ring detector and webcam
    RingDetector detector;
    OpenCvWebcam webcam;

    boolean powerShots = true;
    boolean highGoal = true;
    boolean wobbleGoal = true;
    boolean wobbleGoalNumber = true;

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
    float clawClosed = 251/255f;

    OcServo slide;
    float slideUp = 63/255f;
    float slideDown = 168/255f;
    float slidePos1 = 140/255f;
    float slidePos2 = 118/255f;

    ///Constants
    public final float intake_power = 0.7f;
    final float power = .30f;
    final float intakePower = 1f;
    final float wobblePower = 0.5f;
    final float slowPower = .20f;
    final float fastPower = 0.50f;
    final float turnPower = .30f;
    private boolean isRed = true;
    private boolean isRight = true;
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
            drive.resetAngle();
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

            flywheel_pid = new PIDFCoefficients(1000, 65, 10, 15);

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
         * @param isRed The alliance of the robot
         * @param isRight Where the robot starts (Right or left of the rings)
         * @param wobbleGoalNumber Number of wobble goals (1 or 2)
         * @param startTime Current time upon entering the function
         * @throws InterruptedException
         */
    private void move(WaitLinear lp, RingPosition ringPosition, boolean isRed, boolean isRight, boolean wobbleGoalNumber, long startTime) throws InterruptedException {
        int distance; //Go to wobble goal square first time
        int turnAngle; //Turn towards wobble goal square first time
        int turnAngle2; //Turn towards stack
        int distance2; //Move towards stack
        int turnAngle3; //Turn towards wobble goal square second time

        if (ringPosition == RingPosition.A) {
            distance = -45;
            turnAngle = -45;
            turnAngle2 = 0;
            distance2 = 0;
            turnAngle3 = -135;
        } else if (ringPosition == RingPosition.B) {
            distance = -40;
            turnAngle = -15;
            turnAngle2 = 18;
            distance2 = 45;
            turnAngle3 = -150;
        } else {
            distance = -70;
            turnAngle = -30;
            turnAngle2 = 10;
            distance2 = 62;
            turnAngle3 = -135;
        }

        drive.moveToEncoderInchUsingPID(-48, power, 3000, true);
        powerShots(lp, slidePos1);

        drive.turnUsingPID(-2,power,RotationAxis.CENTER);
        powerShots(lp, slidePos2);

        drive.turnUsingPID(-2,power,RotationAxis.CENTER);
        powerShots(lp, slideUp);
        slide.setPosition(slideDown);

        drive.turnUsingPID(turnAngle,power,RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(distance, power, 3000, true);
        wobbleGoalDrop(lp);

        if (ringPosition == RingPosition.B || ringPosition == RingPosition.C) {
            drive.turnUsingPID(turnAngle2,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(distance2, power, 3000, true);

            intake.setPower(intakePower);
            drive.moveToEncoderInchUsingPID(3, power, 3000, true);
            lp.waitMillis(500);
            intake.setPower(0f);

            if (ringPosition == RingPosition.C) {
                drive.turnUsingPID(10,power,RotationAxis.CENTER);
            }
            highGoal(lp);

            drive.turnUsingPID(-125,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(-10, power, 3000, true);
            wobbleGoalGrab(lp);

            drive.turnUsingPID(turnAngle3,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(distance, power, 3000, true);
            wobbleGoalDrop(lp);

            drive.moveToEncoderInchUsingPID(-(distance+30), power, 3000, true);
        }
        else {
            drive.turnUsingPID(turnAngle3,power,RotationAxis.CENTER);
            drive.moveToEncoderInchUsingPID(distance, power, 3000, true);
            wobbleGoalGrab(lp);

            drive.moveToEncoderInchUsingPID(-(distance+5), power, 3000, true);
            wobbleGoalDrop(lp);
        }

    }

    /**
     * Shoot the power shots
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void powerShots(WaitLinear lp, float position) throws InterruptedException {
        setFlywheelPower(0.5f);
        slide.setPosition(position);
        lp.waitMillis(1000);
        setFlywheelPower(0f);
    }

    /**
     * Shoot the rings into the high goal
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void highGoal(WaitLinear lp) throws InterruptedException {
        setFlywheelPower(0.5f);
        slide.setPosition(slideUp);
        lp.waitMillis(1000);
        setFlywheelPower(0f);
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
        lp.waitMillis(200);
        wobbleArm.setPower(0);

        claw.setPosition(clawClosed);
        wobbleArm.setPower(wobblePower);
        lp.waitMillis(200);
        wobbleArm.setPower(0);
    }

    /**
     * Drop the wobble goal
     * @param lp WaitLinear object used for pauses
     * @throws InterruptedException
     */
    private void wobbleGoalDrop(WaitLinear lp) throws InterruptedException {
        wobbleArm.setPower(-wobblePower);
        lp.waitMillis(200);
        wobbleArm.setPower(0);
        claw.setPosition(clawOpen);

        wobbleArm.setPower(wobblePower);
        lp.waitMillis(200);
        wobbleArm.setPower(0);
        claw.setPosition(clawClosed);
    }

    /**
     * Only park
     * @param lp WaitLinear object used for pauses
     * @param isRed The alliance of the robot
     * @param isRight Where the robot starts (Right or left of the rings)
     * @param startTime Current time upon entering the function
     * @throws InterruptedException
     */
    private void onlyDoParking(WaitLinear lp, boolean isRed, boolean isRight, long startTime) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(-53, power, 3000, true);
    }

    /**
     * Autonomous run function
     * This is the main function that performs all the actions at once
     * @throws InterruptedException
     */
    public void run() throws InterruptedException {
        int MAXIMUM_DELAY = 10000;

        WaitLinear lp = new WaitLinear(this);
        SelectLinear sl = new SelectLinear(this);

        long currentTime;

        ///Make the alliance selection (Red/Blue)
        isRed = sl.selectAlliance();
        ///Make the position selection for ring detection (Right/Left)
        isRight = sl.selectDetectorPosition();
        ///Do you want to only park (Yes/No)
        boolean onlyPark = sl.selectOnlyPark();
        ///Delay before starting
        int startDelaySecs = sl.adjust("Delay Start", 20);
        int startDelay = startDelaySecs * 1000;

        //For park only
        int startPosition = 1;
        int turnFarAngle = 0;
        int forwardFarDistance = 24;
        int distanceFromPosition = 0;
        String direction = "forward";

        if (!onlyPark) {
            powerShots = sl.selectPowerShots();
            if (!powerShots) {
               highGoal = sl.selectHighGoal();
            }
            wobbleGoal = sl.selectWobbleGoal();
            if (wobbleGoal) {
                wobbleGoalNumber = sl.selectWobbleGoalNumber();
            }
        }

        if (wobbleGoal) {
            this.detector = new RingDetector(isRight);
            this.detector.useDefaults();
            this.detector.setRedRange(0.15, 115, 28);
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
        }

        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isRight ? "Right" : "Left");
        telemetry.addData("Park Only", onlyPark ? "Yes" : "No");
        telemetry.addData("Delay Start (Seconds)", startDelaySecs);
        /*if (onlyPark) {
            telemetry.addData("Park Only", onlyPark ? "Yes" : "No");
            telemetry.addData("Starting Position", startPosition);
            if (parkNear) {
                telemetry.addData("Note", "Robot only moves " + direction + " " + Math.abs(distanceFromPosition) + " inches");
            } else {
                if (isRed) {
                    telemetry.addData("Note", "Robot moves forward " + forwardFarDistance + " inches, turns left and moves " + direction + " for " + Math.abs(distanceFromPosition) + " inches");
                } else {
                    telemetry.addData("Note", "Robot moves forward " + forwardFarDistance + " inches, turns right and moves " + direction + " for " + Math.abs(distanceFromPosition) + " inches");
                }
            }
        }*/
        if (!onlyPark) {
            telemetry.addData("Shoot Power Shots", powerShots ? "Yes" : "No");
            telemetry.addData("Shoot High Goal", highGoal ? "Yes" : "No");
            telemetry.addData("Move Wobble Goal", wobbleGoal ? "Yes" : "No");
            if (wobbleGoal) {
                telemetry.addData("Number of Wobble Goals", wobbleGoalNumber ? "1" : "2");
                telemetry.addData("Ring Position=", ringPosition);
            }
            //telemetry.addData("Delay Before Moving Foundation (Seconds)", foundationDelaySecs);
        }

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

        waitForStart();

        //The time when the autonomous run started i.e. the start/stop button was pressed
        long startTime = System.currentTimeMillis();

        if (wobbleGoal) {
            //webcam.pauseViewport();
            detector.reset();
            //webcam.resumeViewport();
            long time1 = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();
            while (currentTime - time1 < RingDetector.DETECTION_WAIT_TIME) {
                ringPosition = detector.getRingPosition();
                currentTime = System.currentTimeMillis();
            }

            telemetry.addData("Ring Position=", ringPosition);
            telemetry.update();
        }
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        //return immediately if stop has been pressed instead of play
        if (isStopRequested()) {
            return;
        }

        ///If the user presses stop, waitForStart() will return, only run if the start button is pressed (not stop).
        if (opModeIsActive()) {
            if (startDelay > 0) lp.waitMillis(startDelay);

            if (onlyPark) {
                onlyDoParking(lp, isRed, isRight, startTime);
            } else {
                move(lp, ringPosition, isRed, isRight, wobbleGoalNumber, startTime);
                /*if (wobbleGoal) {
                }
                if (powerShots) {
                    powerShots(lp, ringPosition, isRed, isRight, startTime);
                } else if (highGoal) {
                    highGoal(lp, ringPosition, isRed, isRight, startTime);
                }*/
            }
        }

        drive.stop();
        RobotLog.ii(TAG_A, "Autonomous finished in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        //wait a bit longer than the 30 seconds just in case the start was accidentally pressed earlier
        lp.waitMillis(32000 - (int)(System.currentTimeMillis()-startTime));
        ///End
    }

    /**
     * Detect if you actually collected the stone to avoid performing the action of placing the stone
     * @return true if the stone is in or if we fail to detect (to be safe)
     */
    private boolean isCollectionSuccessful()
    {
        try {
            if (robot.sensorDistance.getDistance(DistanceUnit.INCH) > 1) return false;
        } catch (Exception e) {
            RobotLog.ii(TAG_A, "Failed to detect if collection is successful" + e.getMessage());
        }
        return true;
    }
}