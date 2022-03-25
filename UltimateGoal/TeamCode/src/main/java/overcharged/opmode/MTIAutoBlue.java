package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.RingDetector;
import overcharged.components.RingPosition;
import overcharged.components.RotationAxis;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.test.EasyOpenCVExample;

import static overcharged.config.RobotConstants.TAG_A;

@Autonomous(name="Blue")
public class MTIAutoBlue extends LinearOpMode {

    private Robot6WheelLinear robot;
    private TankDriveLinear drive;
    private boolean isRed = true;
    RingDetector detector;
    OpenCvWebcam webcam;
    private RingPosition ringPosition = RingPosition.A;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    double powerFromStack = 0.51925;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new Robot6WheelLinear(this);
            drive = robot.getTankDriveLinear();
            WaitLinear lp = new WaitLinear(this);
            robot.doorDown();
            initCamera();

            while (!isStopRequested() && robot.gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro Status", "Calibrating");
                telemetry.update();
                sleep(50);
                idle();
            }
            telemetry.addData("Gyro Status", "Calibrated");
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();
            drive.resetAngle();
            if (opModeIsActive()) {
                driveToStack(lp);
            }

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


    public void driveToStack(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(4, 0.3f, 3000, true);
        drive.turnUsingPID(-70.2, 0.2f, RotationAxis.RIGHT);
        drive.stop();
        drive.moveToEncoderInchUsingPID(6.4f, 0.3f, 3000, true);
        drive.turnUsingPID(70.5f, 0.2f, RotationAxis.LEFT);
        drive.stop();

        long currentTime;
        this.detector = new RingDetector(!isRed);
        this.detector.useDefaults();
        this.detector.setRedRange(0.1, 115, 28);
        this.detector.setBlueRange(0.35, 33, 120);
        webcam.setPipeline(detector);

        try {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        } catch(Exception e) {
            try {
                this.detector = new RingDetector(!isRed);
                this.detector.useDefaults();
                this.detector.setRedRange(0.1, 115, 28);
                this.detector.setBlueRange(0.15, 33, 120);
                webcam.setPipeline(detector);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch (Exception f) {
                ringPosition = RingPosition.C;
            }
        }

        long time1 = System.currentTimeMillis();
        currentTime = System.currentTimeMillis();
        while (opModeIsActive() && currentTime - time1 < RingDetector.DETECTION_WAIT_TIME) {
            ringPosition = detector.getRingPosition();
            currentTime = System.currentTimeMillis();
        }

        //webcam.pauseViewport();
        detector.reset();


        telemetry.addData("Ring Position=", ringPosition);


        telemetry.addLine("Important Message");

        telemetry.update();

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        drive.stop();

        if(opModeIsActive()){
            if (ringPosition == RingPosition.A) {
                shoot(lp,powerFromStack-0.0129, 600);
                moveToA(lp);
            }
            else if (ringPosition == RingPosition.B) {
                drive.turnUsingPID(1.5, 0.3f, RotationAxis.LEFT);
                drive.stop();
                shoot(lp,powerFromStack-0.013, 600);
                collectStarterStackB(lp);
            }
            else {
                drive.moveToEncoderInchUsingPID(11, 1f, 3000, true);
                shoot(lp,powerFromStack-0.022, 600);
                drive.stop();
                collectStarterStackC(lp);
            }
        }
    }

    public void collectStarterStackB(WaitLinear lp) throws InterruptedException {
        robot.intakeOn();
        drive.moveToEncoderInchUsingPID(12, 0.2f, 3000, true);
        drive.moveToEncoderInchUsingPID(-4, 0.3f, 3000, true);
        drive.turnUsingPID(1, 0.3f, RotationAxis.LEFT);
        drive.stop();
        shoot(lp,powerFromStack-0.014, 900);

        moveToB(lp);
    }

    public void collectStarterStackC(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(-4, 0.4f, 3000, true);

        robot.intakeOn();
        drive.moveToEncoderInchUsingPID(5, 0.1f, 3000, true);
        drive.moveToEncoderInchUsingPID(-4, 0.3f, 3000, true);
        drive.turnUsingPID(1.5, 0.3f, RotationAxis.LEFT);
        drive.stop();
        lp.waitMillis(750);
        shoot(lp,powerFromStack-0.02, 900);

        robot.intakeOn();
        drive.moveToEncoderInchUsingPID(19, 0.13f, 3000, true);
        drive.moveToEncoderInchUsingPID(-5, 0.3f, 3000, true);
        robot.intakeOff();
        shoot(lp,powerFromStack-0.019, 600);

        //Add more intaking for extra rings

        moveToC(lp);
    }


    public void moveToC(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(48, 0.9f, 3000, true);
        drive.stop();

        drive.turnUsingPID(80, 0.6f, RotationAxis.LEFT);
        drive.stop();

        robot.autoWobbleUp();
        lp.waitMillis(500);

        drive.turnUsingPID(-85, -0.6f, RotationAxis.LEFT);
        drive.stop();

        drive.moveToEncoderInchUsingPID(-23, 0.9f, 3000, true);
    }

    public void moveToB(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(33, 0.9f, 3000, true);
        drive.turnUsingPID(20, 0.3f, RotationAxis.LEFT);
        drive.stop();
        robot.autoWobbleUp();
        lp.waitMillis(400);

        drive.turnUsingPID(-140, -0.3, RotationAxis.LEFT);
        drive.stop();
        drive.moveToEncoderInchUsingPID(-10, 0.7f, 3000, true);
    }

    public void moveToA(WaitLinear lp) throws InterruptedException {
        drive.moveToEncoderInchUsingPID(26, 0.9f, 3000, true);
        drive.turnUsingPID(86, 0.3, RotationAxis.LEFT);
        drive.stop();

        robot.autoWobbleUp();
        lp.waitMillis(500);

        drive.turnUsingPID(-86, -0.3, RotationAxis.LEFT);
        drive.stop();

        drive.moveToEncoderInchUsingPID(-20, 0.4f, 3000, true);
        lp.waitMillis(8000);
        drive.moveToEncoderInchUsingPID(35, 0.6f, 3000, true);
        drive.stop();
    }

    public void shoot(WaitLinear lp, double power, int waitBeforeShootMillis) throws InterruptedException{
        robot.flywheelOn(power);
        robot.intakeOff();
        robot.doorDown();
        lp.waitMillis(300);

        robot.trayUp();

        lp.waitMillis(700);
        robot.doorUp();
        lp.waitMillis(waitBeforeShootMillis);

        robot.slide1();
        lp.waitMillis(500);
        robot.slide2();
        lp.waitMillis(500);
        robot.slide3();

        lp.waitMillis(200);
        robot.trayDown();
        robot.doorUp();
        robot.slideDown();
        robot.flywheelOff();
    }
}
