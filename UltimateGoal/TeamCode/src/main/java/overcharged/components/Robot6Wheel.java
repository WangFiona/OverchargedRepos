package overcharged.components;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;
import overcharged.odometry.Localization;

import static overcharged.config.RobotConstants.TAG_R;

/**
 * Overcharged Team #12599
 * Robot definition for Six wheel robot
 */
public class Robot6Wheel {
    protected Telemetry telemetry;

    ///Drive components
    public final OcMotor driveLeftFront;
    public final OcMotor driveLeftBack;
    public final OcMotor driveRightFront;
    public final OcMotor driveRightBack;

    public Encoder left;
    public Encoder right;

    public DcMotor leftEncMotor;
    public DcMotor rightEncMotor;

    public final Drive drive;

    ///Robot Gyro sensor
    public OcGyro2 gyroSensor;
    //public final OcGyro gyroSensor1;

    //public NavxImu gyroSensor;
    public Intake intake;
    public Flywheel flywheel;
    public Slide slide;
    public Wobble wobble;
    public AutoWobble autoWobble;
    public Tray tray;

    /// Odometry
    public Localization odometryLocalization = null;

    public DistanceSensor sensorDistance;

    public final List<OcServo> servos = new ArrayList<>();
    public final List<OcSwitch> switchs = new ArrayList<>();

//    ///Led indicator components
//    private final OcLed ledYellow;
//    private final OcLed ledGreen;
//    private final OcLed ledWhite;
//    private final OcLed ledBlue;
//    private final OcLed ledRed;
//    public final List<OcLed> leds = new ArrayList<>();

    /**
     * initialize the robot
     * initialize all the hardware components used in our robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public Robot6Wheel(OpMode op,
                       boolean isAutonomous)
    {
        String missing = "";
        ///report the number of missing components
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        RobotLog.ii(RobotConstants.TAG_R, "Initializing motors");
        ///Initialize Motors
        OcMotor driveLeftFront = null;
        try {
            driveLeftFront = new OcMotor(hardwareMap,
                    "driveLF",
                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: driveLF " + e.getMessage());
            missing = missing + "driveLF";
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        OcMotor driveLeftBack = null;
        try {
            driveLeftBack = new OcMotor(hardwareMap,
                    "driveLB",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: driveLB " + e.getMessage());
            missing = missing + ", driveLB";
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        OcMotor driveRightFront = null;
        try {
            driveRightFront = new OcMotor(hardwareMap,
                    "driveRF",
                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: driveRF " + e.getMessage());
            missing = missing + ", driveRF";
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        OcMotor driveRightBack = null;
        try {
            driveRightBack = new OcMotor(hardwareMap,
                    "driveRB",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: driveRB " + e.getMessage());
            missing = missing + ", driveRB";
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        try {
            left = new Encoder("intakeL", DcMotorSimple.Direction.FORWARD, hardwareMap);
            right = new Encoder("intakeR", DcMotorSimple.Direction.REVERSE, hardwareMap);
        } catch(Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: encoders " + e.getMessage());
            numberMissing++;
        }

        // intake initialization
        try {
            intake = new Intake(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: intake " + e.getMessage());
            numberMissing++;
        }

        // flywheel initialization
        try {
            flywheel = new Flywheel(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: flywheel " + e.getMessage());
            numberMissing++;
        }

        DistanceSensor distanceSensor = null;

        OcGyro2 gyro2 = null;

        try {
            gyro2 = new OcBnoGyro2(hardwareMap, "limu", "rimu");
            while (isAutonomous && gyro2.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: gyro_sensor " + e.getMessage());
            missing = missing + ", Gyro";
            numberMissing++;
        }

        this.gyroSensor = gyro2;

        try {
            gyroSensor = new OcBnoGyro2(hardwareMap, "limu", "rimu");
            while (gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R, "missing: navx " + e.getMessage());
            numberMissing++;
        }

        this.drive = createDrive();

        try {
            slide = new Slide(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: slide " + e.getMessage());
            numberMissing++;
        }

        try {
            wobble = new Wobble(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: wobble " + e.getMessage());
            numberMissing++;
        }

        try {
            tray = new Tray(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: tray " + e.getMessage());
            numberMissing++;
        }

        try {
            autoWobble = new AutoWobble(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: autoWobble " + e.getMessage());
            numberMissing++;
        }

        RobotLog.ii(RobotConstants.TAG_R,  "Initializing done");
        telemetry.addData("Missing Devices", numberMissing);
        telemetry.addData("Missing", missing);
        telemetry.update();
    }

    public void intakeOn(){
        intake.on();
    }

    public void intakeOff(){
        intake.off();
    }

    public void intakeOut(){
        intake.out();
    }

    public void flywheelOn(){
        flywheel.on();
    }

    public void flywheelMid(){
        flywheel.mid();
    }

    public void flywheelOn(double power){ flywheel.on(power); }

    public void flywheelOff(){
        flywheel.off();
    }

    public void slideUp(){
        slide.setPosition(Slide.up);
    }

    public void slideDown(){
        slide.setPosition(Slide.down);
    }

    public void slide1(){
        slide.setPosition(Slide.pos1);
    }

    public void slide2(){
        slide.setPosition(Slide.pos2);
    }

    public void slide3(){
        slide.setPosition(Slide.up);
    }

    public void wobbleUp(){
        wobble.setArmPosition(Wobble.up);
    }

    public void wobbleDown(){
        wobble.setArmPosition(Wobble.down);
    }

    public void wobbleInit() { wobble.setArmPosition(Wobble.init); }

    public void wobbleOpen(){
        wobble.setClawPosition(Wobble.open);
    }

    public void wobbleClose(){
        wobble.setClawPosition(Wobble.closed);
    }

    public void trayUp(){
        tray.setTrayPosition(Tray.up);
    }

    public void trayDown(){
        tray.setTrayPosition(Tray.down);
    }

    public void doorUp(){
        tray.setDoorPosition(Tray.doorup);
    }

    public void doorDown(){
        tray.setDoorPosition(Tray.doordown);
    }

    public void autoWobbleUp(){ autoWobble.setArmUp(); }

    public void autoWobbleDown(){ autoWobble.setArmDown(); }

    /**
     * Robot and sensor shut down
     */
    public void close ()
    {
        if (this.drive != null) {
            this.drive.stop();
        } else {
            if (this.driveLeftFront != null) this.driveLeftFront.setPower(0f);
            if (this.driveLeftBack != null) this.driveLeftBack.setPower(0f);
            if (this.driveRightFront != null) this.driveRightFront.setPower(0f);
            if (this.driveRightBack != null) this.driveRightBack.setPower(0f);
        }

    }

    /**
     * subclass override this method
     * @return Drive
     */
    protected Drive createDrive () {
        return new TankDrive(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);
    }

    /**
     * initialize tank drive
     * @return TankDrive
     */
    public TankDrive getTankDrive () {
        return (TankDrive) this.drive;
    }

}