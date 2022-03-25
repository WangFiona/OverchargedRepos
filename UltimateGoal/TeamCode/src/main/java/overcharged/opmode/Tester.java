package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;
import java.util.List;

import overcharged.components.Button;
import overcharged.test.MotorTestInfo;
import overcharged.components.OcLed;
import overcharged.components.OcServo;
import overcharged.components.OcSwitch;
import overcharged.components.TurnType;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.test.ServoTestInfo;

import static overcharged.components.Button.BTN_FLYWHEEL;
import static overcharged.components.Button.BTN_POWERSHOT;
import static overcharged.components.Button.BTN_SLIDE_DOWN;
import static overcharged.components.Button.BTN_SLIDE_UP;
import static overcharged.components.Button.BTN_WOBBLE_UP;

/**
 * Overcharged Team #12599 Tester
 * This tester program has 16 separate tests.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tester", group = "Game")
public class Tester
        extends LinearOpMode {

    ///Overcharged Autonomous Robot class
    private Robot6WheelLinear robot;
    ///Overcharged Autonomous Tank Drive class
    private TankDriveLinear drive;

    DcMotorEx intake;
    OcServo wobble;
    DcMotorEx flywheel_motor1;
    DcMotorEx flywheel_motor2;

    float wobbleInit = 196/255f;
    float wobbleUp = 157/255f;
    float wobbleDown = 66/255f;

    OcServo slide;
    float slideUp = 97/255f;
    float slideDown = 173/255f;
    float slidePos1 = 147/255f;
    float slidePos2 = 127/255f;

    /**
     * Counter of servos in servo test
     */
    private int servoTestCounter = 0;
    /**
     * Counter for servos for servo calibrate test
     */
    private int servoCalibrateCounter = 0;
    public final float intake_power = 1f;
    public final float flywheel_power = 0.72f;

    private final static int MIN_SERVO_TICK = 1;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    public enum WobbleState {
        INIT,
        UP,
        DOWN
    }
    //private TeleOp.WobbleState wobbleState = TeleOp.WobbleState.INIT;

    public enum SlideState {
        UP,
        DOWN
    }
    public enum PowershotState {
        POS_1,
        POS_2,
        POS_3
    }
    PowershotState powershotState = PowershotState.POS_1;

    /**
     * Enumeration for the different tests
     */
    public enum ETest {
        NONE,
        SWITCH,
        ENCODER,
        MOTOR,
        DRIVE,
        SERVO_CALIBRATE,
        GYRO,
        INTAKE,
        WOBBLE_GOAL,
        SLIDE,
        FLYWHEEL,
        ;

        private static int numberTests = 0;

        /**
         * Get the test according to number
         *
         * @param ordinal the test number
         * @return the test according to the number given
         */
        public static ETest getTest(int ordinal) {
            for (ETest e : values()) {
                if (e.ordinal() == ordinal) {
                    return e;
                }
            }

            return NONE;
        }

        /**
         * Get the number of tests
         *
         * @return the number of tests
         */
        public static int getNumberTests() {
            if (numberTests == 0) {
                for (ETest ignored : values()) {
                    numberTests++;
                }
            }
            return numberTests;
        }
    }

    /**
     * Function for running tests
     */
    @Override
    public void runOpMode() {
        //Initialization
        robot = new Robot6WheelLinear(this);
        drive = robot.getTankDriveLinear();

        slide = new OcServo(hardwareMap, "slideServo", slideDown);
        wobble = new OcServo(hardwareMap, "wobbleServo", wobbleInit);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int testCounter = 0;
        ///Set current test to NONE
        ETest currentTest = ETest.NONE;

        waitForStart();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Choosing the desired test
            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                testCounter++;
                if (testCounter >= ETest.getNumberTests()) {
                    testCounter = 0;
                }
                currentTest = ETest.getTest(testCounter);
            } else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                testCounter--;
                if (testCounter < 0) {
                    testCounter = ETest.getNumberTests() - 1;
                }
                currentTest = ETest.getTest(testCounter);
            }

            telemetry.addData("Test", currentTest);
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Confirm", "Start");

            ///Loop tests
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                switch (currentTest) {
                    case ENCODER:
                        encoderTest();
                        break;
                    case MOTOR:
                        motorTest();
                        break;
                    case DRIVE:
                        driveTest();
                        break;
                    case SERVO_CALIBRATE:
                        servoCalibrate(robot.servos);
                        break;
                    case GYRO:
                        //gyroTest();
                        break;
                    case SWITCH:
                        switchTest();
                        break;
                    case INTAKE:
                        intakeTest();
                        break;
                    case WOBBLE_GOAL:
                        //wobbleTest();
                        break;
                    case SLIDE:
                        slideTest();
                        break;
                    case FLYWHEEL:
                        flywheelTest();
                    case NONE:
                    default:
                        break;
                }
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * Test for encoder values of all DC motors in the robot
     */
    private void encoderTest() {
        ///Set all motors to FLOAT behavior while unpowered
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Reset encoders on all motors
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                drive.resetPosition();
                idle();
            } else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Front",
                    "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                            "Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
            telemetry.addData("Back",
                    "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                            "Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
        ///Set drive train motors to BRAKE behavior while unpowered
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    /**
     * Test the motors
     */
    private void motorTest() {
        float[] powers = new float[]{0.25f, -0.25f};

        MotorTestInfo[] motorTestInfos = new MotorTestInfo[]{new MotorTestInfo(robot.driveLeftFront, "driveLeftFront"),
                new MotorTestInfo(robot.driveLeftBack, "driveLeftBack"),
                new MotorTestInfo(robot.driveRightFront, "driveRightFront"),
                new MotorTestInfo(robot.driveRightBack, "driveRightBack")
        };

        back:
        for (MotorTestInfo motorTestInfo : motorTestInfos) {
            for (float power : powers) {
                motorTestInfo.stop();

                long startTimestamp = System.currentTimeMillis();
                long timeStamp = startTimestamp;

                while (opModeIsActive() &&
                        timeStamp - startTimestamp < 2000) {
                    telemetry.addData("Motor", motorTestInfo.motorName);
                    telemetry.addData("Power", power);
                    telemetry.addData("CurrentPosition", motorTestInfo.motor.getCurrentPosition());

                    motorTestInfo.motor.setPower(power);

                    telemetry.addData("Stop", "Back");

                    if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                        break back;
                    }

                    telemetry.update();
                    idle();
                    timeStamp = System.currentTimeMillis();
                }
                motorTestInfo.stop();
            }
        }

        idle();
    }

    /**
     * Test the limit switches at the bottom and top of the slide system
     */
    private void switchTest() {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            for (OcSwitch s : robot.switchs) {
                telemetry.addData(s.toString(), Boolean.toString(s.isTouch()));
            }

            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
    }

    /**
     * Calibration of servos in robot
     *
     * @param servoCalibrateList servos to be tested
     */
    private void servoCalibrate(List<OcServo> servoCalibrateList) {
        int posJoy1 = (int) (servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            ///Choose a servo for calibration
            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoCalibrateCounter++;
                if (servoCalibrateCounter >= servoCalibrateList.size()) {
                    servoCalibrateCounter = 0;
                }
                posJoy1 = (int) (servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            } else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoCalibrateCounter--;
                if (servoCalibrateCounter < 0) {
                    servoCalibrateCounter = servoCalibrateList.size() - 1;
                }
                posJoy1 = (int) (servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            } else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            }

            ///Change servo position for calibration
            if (gamepad1.x && Button.BTN_MINUS.canPress4Short(timeStamp)) {
                posJoy1 -= MIN_SERVO_TICK;
            } else if (gamepad1.b && Button.BTN_PLUS.canPress4Short(timeStamp)) {
                posJoy1 += MIN_SERVO_TICK;
            } else if (gamepad1.y && Button.BTN_MAX.canPress(timeStamp)) {
                posJoy1 = 255;
            } else if (gamepad1.a && Button.BTN_MIN.canPress(timeStamp)) {
                if (servoCalibrateCounter == 5) {
                    posJoy1 = 100;
                } else {
                    posJoy1 = 0;
                }
            } else if (gamepad1.right_stick_button && Button.BTN_MID.canPress(timeStamp)) {
                posJoy1 = 128;
            }
            posJoy1 = Range.clip(posJoy1, 0, 255);
            servoCalibrateList.get(servoCalibrateCounter).setPosition(posJoy1 / 255f);

            telemetry.addData("Adjust", "+: B -: X Max: Y Min: A Mid: RStick");
            telemetry.addData("Position", Integer.toString(posJoy1));
            telemetry.addData("Servo", servoCalibrateList.get(servoCalibrateCounter));
            telemetry.addData("Select", "Next: RightTrigger Prev: LeftTrigger");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
    }

    /**
     * Tests each individual servo
     *
     * @param servoTestInfos the servos to be tested
     */
    private void servoTest(ServoTestInfo[] servoTestInfos) {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoTestCounter++;
                if (servoTestCounter >= servoTestInfos.length) {
                    servoTestCounter = 0;
                }
            } else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoTestCounter--;
                if (servoTestCounter < 0) {
                    servoTestCounter = servoTestInfos.length - 1;
                }
            } else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            } else if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                servoTest(servoTestInfos[servoTestCounter]);
            }

            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Servo", servoTestInfos[servoTestCounter].servo);
            telemetry.addData("Confirm", "Start");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
    }

    /**
     * Tests each individual servo
     *
     * @param servoTestInfo the servos to be tested
     */
    private void servoTest(ServoTestInfo servoTestInfo) {
        for (int i = 0; i <= servoTestInfo.positions.length; i++) {
            float currentPosition = servoTestInfo.servo.getPosition();
            float position;
            if (i < servoTestInfo.positions.length) {
                position = servoTestInfo.positions[i];
            } else {
                position = servoTestInfo.servo.getInitialPosition();
            }

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;
            long timeout = (long) (Math.abs(position - currentPosition) * 1000 * servoTestInfo.timeScale);

            while (opModeIsActive() &&
                    timeStamp - startTimestamp < timeout) {
                servoTestInfo.servo.setPosition(position);

                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    return;
                }

                telemetry.addData("Position", Float.toString(servoTestInfo.servo.getPosition()));
                telemetry.addData("Servo", servoTestInfo.servo);
                telemetry.addData("Stop", "Back");

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }
        }
    }

    /**
     * Test the drive train, motors, and servos
     */
    private void driveTest() {
        float[][] powers = new float[][]{
                {0.25f, 0.25f},
                {-0.25f, -0.25f},
                {0.25f, -0.25f},
                {-0.25f, 0.25f},
        };

        TurnType[] turnTypes = new TurnType[]{
                TurnType.TURN_REGULAR,
                TurnType.TURN_RIGHT_PIVOT,
                TurnType.TURN_LEFT_PIVOT,
        };

        back:
        for (TurnType turnType : turnTypes) {
            for (float[] power : powers) {
                drive.resetPosition();

                long startTimestamp = System.currentTimeMillis();
                long timeStamp = startTimestamp;

                while (opModeIsActive() &&
                        timeStamp - startTimestamp < 5000) {

                    if (turnType == TurnType.TANK) {
                        telemetry.addData("DriveType", "TANK");
                        drive.setPower(power[0], power[1]);
                    }
                    drive.setPower(power[0], power[1], turnType);

                    telemetry.addData("Front",
                            "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                                    " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
                    telemetry.addData("Back",
                            "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                                    " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
                    telemetry.addData("Stop", "Back");

                    if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                        break back;
                    }

                    telemetry.update();
                    idle();
                    timeStamp = System.currentTimeMillis();
                }

                drive.stop();
            }
        }

        drive.stop();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    /**
     * Test showing IMU headings
     */
    /*private void gyroTest() {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                robot.gyroSensor.resetHeading();
                idle();
            } else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Heading",
                    Float.toString(robot.gyroSensor.getHeading()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
        idle();
    }*/

    /**
     * Test of different modes for glyph intake system
     */
    private void intakeTest() {
        //intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        float pwr = intake_power;

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();
            ///Intake controls
            if (gamepad1.right_bumper && Button.BTN_COLLECT.canPress(timeStamp)) {
                intake.setPower(pwr);
            } else if (gamepad1.left_bumper && Button.BTN_REJECT.canPress(timeStamp)) {
                intake.setPower(-0.4 * pwr);
            } else if (gamepad1.a && Button.BTN_REJECT.canPress(timeStamp)) {
                intake.setPower(0);
            }
        }
    }

    /*private void wobbleTest() {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.y && BTN_WOBBLE_UP.canPress(timeStamp)) {
                if (wobbleState == TeleOp.WobbleState.INIT) {
                    wobble.setPosition(wobbleUp);
                    wobbleState = TeleOp.WobbleState.UP;
                } else if (wobbleState == TeleOp.WobbleState.UP) {
                    wobble.setPosition(wobbleDown);
                    wobbleState = TeleOp.WobbleState.DOWN;
                } else if (wobbleState == TeleOp.WobbleState.DOWN) {
                    wobble.setPosition(wobbleUp);
                    wobbleState = TeleOp.WobbleState.UP;
                }
            }
        }
    }*/

    private void slideTest() {
        while (opModeIsActive()) {
            long timestamp = System.currentTimeMillis();
            if (gamepad1.dpad_up && BTN_SLIDE_DOWN.canPress(timestamp)) {
                slide.setPosition(slideDown);
            } else if (gamepad1.dpad_down && BTN_SLIDE_UP.canPress(timestamp)) {
                slide.setPosition(slideUp);
            }

            if (gamepad1.a && BTN_POWERSHOT.canPress(timestamp)) {
                if (powershotState == PowershotState.POS_1) {
                    slide.setPosition(slidePos1);
                    powershotState = PowershotState.POS_2;
                } else if (powershotState == PowershotState.POS_2) {
                    slide.setPosition(slidePos2);
                    powershotState = PowershotState.POS_3;
                } else if (powershotState == PowershotState.POS_3) {
                    slide.setPosition(slideUp);
                    powershotState = PowershotState.POS_1;
                }
            }
        }
    }

    private void flywheelTest() {
        flywheel_motor1 = hardwareMap.get(DcMotorEx.class, "flywheel_motor_1");
        flywheel_motor2 = hardwareMap.get(DcMotorEx.class, "flywheel_motor_2");
        flywheel_motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel_motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        float pwr = flywheel_power;

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();
            ///Flywheel controls
            if (gamepad1.right_bumper && Button.BTN_COLLECT.canPress(timeStamp)) {
                flywheel_motor1.setPower(pwr);
                flywheel_motor2.setPower(pwr);
            } else if (gamepad1.left_bumper && Button.BTN_REJECT.canPress(timeStamp)) {
                flywheel_motor1.setPower(-0.4*pwr);
                flywheel_motor2.setPower(-0.4*pwr);
            } else if (gamepad1.a && Button.BTN_REJECT.canPress(timeStamp)) {
                flywheel_motor2.setPower(0);
                flywheel_motor1.setPower(0);
            }
        }

    }

}


//    /**
//     * Test for LEDs, blinks on and off for 5 LEDs
//     */
//    private void ledTest () {
//        final OcLed.ELedStatus[] ledStatuses = new OcLed.ELedStatus[] {
//                OcLed.ELedStatus.LED_ON,
//                OcLed.ELedStatus.LED_BLINK,
//
//                OcLed.ELedStatus.LED_OFF};
//        for (OcLed led: robot.leds) {
//
//            for (int i = 0; i < ledStatuses.length; i++) {
//                long startTimestamp = System.currentTimeMillis();
//                long timeStamp = startTimestamp;
//
//                while (opModeIsActive() &&
//                        timeStamp - startTimestamp < 2000) {
//
//                    if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
//                        return;
//                    }
//
//                    led.set(ledStatuses[i]);
//                    led.draw();
//                    telemetry.addData("LED", led + " " + led.toString() + " " + ledStatuses[i]);
//                    telemetry.addData("Stop", "Back");
//
//                    telemetry.update();
//                    idle();
//                    timeStamp = System.currentTimeMillis();
//                }
//            }
//        }
//    }
