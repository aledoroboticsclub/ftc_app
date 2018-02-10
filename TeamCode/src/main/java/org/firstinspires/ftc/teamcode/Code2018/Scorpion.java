package org.firstinspires.ftc.teamcode.Code2018;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Code2018.DogeCV.ClosableVuforiaLocalizer;

import java.util.Locale;


/**
 * Created by Aledo HS Robotics on 4/12/2017.
 */
public class Scorpion {

	public DcMotor frontLeft;
	public DcMotor frontRight;
	public DcMotor backLeft;
	public DcMotor backRight;
	DcMotor lift1;
	DcMotor lift2;
	DcMotor inputMR;
	DcMotor inputML;

	Servo trayServo;
	Servo jewelPusher;
	Servo relicGrabber;
	Servo extender;
	Servo intakeLockLeft;
	Servo intakeLockRight;
	public Servo phoneServo;

	//ColorSensor MRColor;
	BNO055IMU.Parameters gyroParameters;
	BNO055IMU rev1;
	BNO055IMU rev2;
	private static final int turningPrecisionRequirement =3;
	private static final int turnSpeedForEncoderRunToPosition =5;

	Telemetry telemetry;
	HardwareMap hardwareMap;

	public static final String TAG = "Vuforia VuMark";
	OpenGLMatrix lastLocation = null;
	public ClosableVuforiaLocalizer vuforia;
	int cameraMonitorViewId;
	VuforiaLocalizer.Parameters parameters;
	public VuforiaTrackables relicTrackables;
	public VuforiaTrackable relicTemplate;
	private static final int vuforiaTimeoutTime=10000;//in milliseconds

	public JewelDetector jewelDetector = null;

	ElapsedTime timer = new ElapsedTime();

	private static final double initialTrayPosition=.89;
	private static final double parallelTrayPosition =.64;
	private static final double placementTrayPosition=0.0;

	private static final double jewelPusherDownPosition=.7;
	private static final double jewelPusherUpPosition=.23;
	private static final double jewelPusherStartPosition=.15;

	private static final double relicGrabberGrabbedPosition=.57;
	private static final double relicGrabberReleasePosition=.35;

	private static final double phoneUpPosition=.41;
	private static final double phoneJewelPosition=.62;

	private static final int ticksPerInch=89;//TODO: test this value, this one is from last year
	private static final int encoderSafeZone=50;/*a motor must be within this many ticks of its
	target to be considered "on target"*/

	//in motor ticks
	private static final int liftPosition0=0;//loading position, and places on the ground
	private static final int liftPosition1=440;//places over 1 cubes
	private static final int liftPosition2=2360;//places over 2 cubes
	private static final int liftPosition3=3000;//places over 3 cubes
	private static final double liftPower=1;//max speed lift travels at
	private static final double liftSpeedMultiplier=80;//modifies the speed of manual control of lift


	public void initRobot(HardwareMap spareMap, Telemetry tempTelemetry) {
		getOpmodeVariables(spareMap, tempTelemetry);
		initHardware();

		lift1.setPower(liftPower);
		lift2.setPower(liftPower);

		setTrayToIntake();
		jewelPusher.setPosition(jewelPusherStartPosition);
		setGrabberToGrabbed();
		setPhoneToUpPosition();

		//MRColor.enableLed(true);
	}

	public void getOpmodeVariables(HardwareMap spareMap, Telemetry tempTelemetry) {
		hardwareMap=spareMap;
		telemetry = tempTelemetry;
	}
	public void initVuforia(){
		cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		parameters.vuforiaLicenseKey = "Ac8Q0nb/////AAAAGZRBQQCid0gNt3ydtz8W8gB8MrlkYn+Gu+jvldH+Igx9SypXvRwUWJw/71iF8xhpjKXBDv1UDD+EsjkvvC1+Zko/hF+lZG/TglT50MCsw6/q2MuSc+AUFDqT9lhEJcyroMMp20VPNwj/fUoUAxr5DV4+VUdwwYW/sCML6iL/x0rWEzUGxJf8qvKSrZcI/4X2fWsryCaprTXecsZCTudHQiElph2GCtMva4843D9sx+a6NB9zhPiyn6aaydEs5T4Ygc5o2nK1p6o8G82++XtlDYPkBuVBajLsO6z0Zvk980xIWmgyKjMNZlLofM7lLJdjt5Sh4a1imlIlsAWbQqPvs35MxJLmmrugrO7WXXveK4TY";

		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		this.vuforia = new ClosableVuforiaLocalizer(parameters);

		relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
		relicTemplate = relicTrackables.get(0);
		relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
		relicTrackables.activate();
	}
	public void deactivateVuforia(){
		relicTrackables.deactivate();
		vuforia.close();//ends vuforia
	}
	public void initJewelDetector(){
		jewelDetector = new JewelDetector();
		jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

		//Jewel Detector Settings
		jewelDetector.areaWeight = 0.02;
		jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
		//jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
		jewelDetector.debugContours = true;
		jewelDetector.maxDiffrence = 15;
		jewelDetector.ratioWeight = 15;
		jewelDetector.minArea = 700;
		jewelDetector.rotateMat=true;

		jewelDetector.enable();
	}
	public void initGyro() {
		gyroParameters = new BNO055IMU.Parameters();
		gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
		gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		gyroParameters.loggingEnabled      = true;
		gyroParameters.loggingTag          = "IMU";
		gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		rev1 = hardwareMap.get(BNO055IMU.class, "rev1");
		rev2 = hardwareMap.get(BNO055IMU.class, "rev2");
		rev1.initialize(gyroParameters);
		rev2.initialize(gyroParameters);


	}
	public void initHardware(){
		frontLeft = hardwareMap.dcMotor.get("front left wheel");
		frontRight = hardwareMap.dcMotor.get("front right wheel");
		backLeft = hardwareMap.dcMotor.get("back left wheel");
		backRight = hardwareMap.dcMotor.get("back right wheel");
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

		lift1=hardwareMap.dcMotor.get("lift1");
		lift2=hardwareMap.dcMotor.get("lift2");
		lift2.setDirection(DcMotorSimple.Direction.REVERSE);
		setLiftMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setLiftMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setLiftMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

		inputML=hardwareMap.dcMotor.get("inputML");
		inputMR=hardwareMap.dcMotor.get("inputMR");
		inputML.setDirection(DcMotorSimple.Direction.REVERSE);
		inputML.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		inputMR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		trayServo=hardwareMap.servo.get("trayServo");
		relicGrabber=hardwareMap.servo.get("relicServo");
		jewelPusher=hardwareMap.servo.get("jewelPusher");
		extender=hardwareMap.servo.get("extender");
		intakeLockLeft=hardwareMap.servo.get("intakeLockLeft");
		intakeLockRight=hardwareMap.servo.get("intakeLockRight");
		phoneServo=hardwareMap.servo.get("phoneServo");

		//MRColor=hardwareMap.colorSensor.get("Color");
	}

	String format(OpenGLMatrix transformationMatrix) {
		return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
	}
	public void displayJewelResults(){//assumes dogeCV is turned on
		telemetry.addData("Jewel Order: ", jewelDetector.getCurrentOrder().toString());
		telemetry.update();
	}
	String formatAngle(AngleUnit angleUnit, double angle) {
		return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
	}
	String formatDegrees(double degrees){
		return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
	}

	public void rightIntake (double power) {
		inputMR.setPower(power*.4);
	}
	public void leftIntake (double power) {
		inputML.setPower(power*-.7);
	}

	public void setTrayToIntake() {
		trayServo.setPosition(initialTrayPosition);
	}
	public void setTrayToPlace() {
		trayServo.setPosition(placementTrayPosition);
	}
	public void setTrayToParallel() {trayServo.setPosition(parallelTrayPosition);}

	public void extenderOut(){
		extender.setPosition(1);
	}
	public void extenderIn() {
		extender.setPosition(0);
	}
	public void extenderStill(){
		extender.setPosition(.5);
	}

	public void setGrabberToGrabbed(){relicGrabber.setPosition(relicGrabberGrabbedPosition);}
	public void setGrabberToRelease(){relicGrabber.setPosition(relicGrabberReleasePosition);}

	public void moveLiftDown(double power) {//power is a value 0 to 1
		lift1.setTargetPosition(lift1.getCurrentPosition()+((int) Math.round(power * liftSpeedMultiplier)));
		lift2.setTargetPosition(lift2.getCurrentPosition()+((int) Math.round(power * liftSpeedMultiplier)));
	}
	public void moveLiftUp(double power) {
		lift1.setTargetPosition(lift1.getCurrentPosition()-((int) Math.round(power * liftSpeedMultiplier)));
		lift2.setTargetPosition(lift2.getCurrentPosition()-((int) Math.round(power * liftSpeedMultiplier)));
	}

	public void setLiftToPosition0() {
		lift1.setTargetPosition(liftPosition0);
		lift2.setTargetPosition(liftPosition0);
		telemetry.addData("lift position: ", "0");
		telemetry.addData("ready to load\nready to place on ground", "");
	}
	public void setLiftToPosition1() {
		lift1.setTargetPosition(liftPosition1);
		lift2.setTargetPosition(liftPosition1);
		telemetry.addData("lift position: ", "1");
		telemetry.addData("ready to place over 1 cube", "");
	}
	public void setLiftToPosition2() {
		lift1.setTargetPosition(liftPosition2);
		lift2.setTargetPosition(liftPosition2);
		telemetry.addData("lift position: ", "2");
		telemetry.addData("ready to place over 2 cubes", "");
	}
	public void setLiftToPosition3() {
		lift1.setTargetPosition(liftPosition3);
		lift2.setTargetPosition(liftPosition3);
		telemetry.addData("lift position: ", "3");
		telemetry.addData("ready to place over 3 cubes", "");
	}

	public void setPhoneToUpPosition(){phoneServo.setPosition(phoneUpPosition);}
	public void setPhoneToJewelPosition(){phoneServo.setPosition(phoneJewelPosition);}

	public void setMotorEncoderForward(int distance) {
		frontLeft.setTargetPosition(distance);
		frontRight.setTargetPosition(distance);
		backLeft.setTargetPosition(distance);
		backRight.setTargetPosition(distance);
	}
	public void setMotorEncoderBackward(int distance) {
		frontLeft.setTargetPosition(-distance);
		frontRight.setTargetPosition(-distance);
		backLeft.setTargetPosition(-distance);
		backRight.setTargetPosition(-distance);
	}
	public void setMotorEncoderLeft(int distance) {
		frontLeft.setTargetPosition(-distance);
		frontRight.setTargetPosition(distance);
		backLeft.setTargetPosition(distance);
		backRight.setTargetPosition(-distance);
	}
	public void setMotorEncoderRight(int distance) {
		frontLeft.setTargetPosition(distance);
		frontRight.setTargetPosition(-distance);
		backLeft.setTargetPosition(-distance);
		backRight.setTargetPosition(distance);
	}
	public void setMotorEncoderClockwise(int distance) {
		frontLeft.setTargetPosition(distance);
		frontRight.setTargetPosition(-distance);
		backLeft.setTargetPosition(distance);
		backRight.setTargetPosition(-distance);
	}
	public void setMotorEncoderCounterwise(int distance) {
		frontLeft.setTargetPosition(-distance);
		frontRight.setTargetPosition(distance);
		backLeft.setTargetPosition(-distance);
		backRight.setTargetPosition(distance);
	}

	//TODO: implement acceleration and deceleration into this to prevent sliding
	public void driveForwardEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		setMotorEncoderForward(distance*ticksPerInch+frontLeft.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
			frontLDist>encoderSafeZone &&
			frontRDist>encoderSafeZone &&
			backLDist>encoderSafeZone &&
			backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveBackwardEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		setMotorEncoderBackward(distance*ticksPerInch+frontLeft.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
			frontLDist>encoderSafeZone &&
			frontRDist>encoderSafeZone &&
			backLDist>encoderSafeZone &&
			backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveLeftEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		setMotorEncoderLeft(distance*ticksPerInch+frontLeft.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
			frontLDist>encoderSafeZone &&
			frontRDist>encoderSafeZone &&
			backLDist>encoderSafeZone &&
			backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void driveRightEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		setMotorEncoderRight(distance*ticksPerInch+frontLeft.getCurrentPosition());
		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
			frontLDist>encoderSafeZone &&
			frontRDist>encoderSafeZone &&
			backLDist>encoderSafeZone &&
			backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void turnClockwiseEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		setMotorEncoderClockwise(distance*ticksPerInch+frontLeft.getCurrentPosition());

		setToForward(power);
		do{
			frontLDist=Math.abs(frontLeft.getTargetPosition()-frontLeft.getCurrentPosition());
			frontRDist=Math.abs(frontRight.getTargetPosition()-frontRight.getCurrentPosition());
			backLDist=Math.abs(backLeft.getTargetPosition()-backLeft.getCurrentPosition());
			backRDist=Math.abs(backRight.getTargetPosition()-backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ",frontLDist);
			telemetry.addData("frontRight distanceFrom: ",frontRDist);
			telemetry.addData("backLeft distanceFrom: ",backLDist);
			telemetry.addData("backRight distanceFrom: ",backRDist);
			telemetry.update();
		}while(
			frontLDist>encoderSafeZone &&
			frontRDist>encoderSafeZone &&
			backLDist>encoderSafeZone &&
			backRDist>encoderSafeZone
				);
		setToStill();
	}
	public void turnCounterwiseEncoder(double power, int distance) {
		setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
		int frontLDist, frontRDist, backLDist, backRDist;
		setMotorEncoderCounterwise(distance * ticksPerInch + frontLeft.getCurrentPosition());

		setToForward(power);
		do {
			frontLDist = Math.abs(frontLeft.getTargetPosition() - frontLeft.getCurrentPosition());
			frontRDist = Math.abs(frontRight.getTargetPosition() - frontRight.getCurrentPosition());
			backLDist = Math.abs(backLeft.getTargetPosition() - backLeft.getCurrentPosition());
			backRDist = Math.abs(backRight.getTargetPosition() - backRight.getCurrentPosition());

			telemetry.addData("frontLeft distanceFrom: ", frontLDist);
			telemetry.addData("frontRight distanceFrom: ", frontRDist);
			telemetry.addData("backLeft distanceFrom: ", backLDist);
			telemetry.addData("backRight distanceFrom: ", backRDist);
			telemetry.update();
		} while (
				frontLDist > encoderSafeZone &&
						frontRDist > encoderSafeZone &&
						backLDist > encoderSafeZone &&
						backRDist > encoderSafeZone
				);
		setToStill();
	}

	public void setDriveMotorMode(DcMotor.RunMode mode) {
		switch(mode) {
			case RUN_USING_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_USING_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);break;
			case RUN_WITHOUT_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);break;
			case STOP_AND_RESET_ENCODER:
				if(frontLeft.getMode()==DcMotor.RunMode.STOP_AND_RESET_ENCODER)
					break;
				frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);break;
			case RUN_TO_POSITION:
				if(frontLeft.getMode()==DcMotor.RunMode.RUN_TO_POSITION)
					break;
				frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);break;
		}
	}
	public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
		switch(behavior) {
			case BRAKE:
				if(frontLeft.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.BRAKE)
					break;
				frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);break;
			case FLOAT:
				if(frontLeft.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.FLOAT)
					break;
				frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);break;
		}
	}
	public void setDriveMotorPower(double power){
		frontLeft.setPower(power);
		frontRight.setPower(power);
		backLeft.setPower(power);
		backRight.setPower(power);
	}//for use with driveEncoder methods

	public void setLiftMotorMode(DcMotor.RunMode mode){
		switch(mode) {
			case RUN_USING_ENCODER:
				if(lift1.getMode()==DcMotor.RunMode.RUN_USING_ENCODER)
					break;
				lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);break;
			case RUN_WITHOUT_ENCODER:
				if(lift1.getMode()==DcMotor.RunMode.RUN_WITHOUT_ENCODER)
					break;
				lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);break;
			case STOP_AND_RESET_ENCODER:
				if(lift1.getMode()==DcMotor.RunMode.STOP_AND_RESET_ENCODER)
					break;
				lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);break;
			case RUN_TO_POSITION:
				if(lift1.getMode()==DcMotor.RunMode.RUN_TO_POSITION)
					break;
				lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);break;
		}
	}
	public void setLiftMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
		switch(behavior) {
			case BRAKE:
				if(lift1.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.BRAKE)
					break;
				lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;break;
			case FLOAT:
				if(lift1.getZeroPowerBehavior()==DcMotor.ZeroPowerBehavior.FLOAT)
					break;
				lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);break;
		}
	}

	public void waiter(int time) {
		timer.reset();
		while(timer.milliseconds()<time){}
	}

	//jewel pusher methods
	public void pushJewel(String teamColor){
		waiter(1000);
		if(teamColor.equals("Blue")){
			if(jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.BLUE_RED)){
				telemetry.addData("Sequence",jewelDetector.getCurrentOrder());
				telemetry.update();
				turnClockwiseEncoder(.1,5);
				jewelPusher.setPosition(jewelPusherUpPosition);
				waiter(2000);
				turnCounterwiseEncoder(.1,1);
			}
			else if(jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.RED_BLUE)){
				telemetry.addData("Sequence",jewelDetector.getCurrentOrder());
				telemetry.update();
				turnCounterwiseEncoder(.1,5);
				jewelPusher.setPosition(jewelPusherUpPosition);
				waiter(2000);
				turnClockwiseEncoder(.1,1);
			}
		}
		if(teamColor.equals("Red")){
			if(jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.RED_BLUE)){
				telemetry.addData("Sequence",jewelDetector.getCurrentOrder());
				telemetry.update();
				turnClockwiseEncoder(.1,5);
				jewelPusher.setPosition(jewelPusherUpPosition);
				waiter(2000);
				turnCounterwiseEncoder(.1,1);
			}
			else if(jewelDetector.getCurrentOrder().equals(JewelDetector.JewelOrder.BLUE_RED)){
				telemetry.addData("Sequence",jewelDetector.getCurrentOrder());
				telemetry.update();
				turnCounterwiseEncoder(.1,5);
				jewelPusher.setPosition(jewelPusherUpPosition);
				waiter(2000);
				turnClockwiseEncoder(.1,1);
			}
		}
	}
	public void setJewelPusherToDown (){
		jewelPusher.setPosition(jewelPusherDownPosition);
	}
	public void setJewelPusherToUp (){
		jewelPusher.setPosition(jewelPusherUpPosition);
	}

	//driveTime methods
	//TODO: consider implementing acceleration and deceleration into the driveTime methods, even though it may be more work than it is worth
	public void driveForwardTime(double power, int time) {
		setToForward(power);
		waiter(time);
		setToStill();
	}
	public void driveBackwardTime(double power, int time) {
		setToBackward(power);
		waiter(time);
		setToStill();
	}
	public void driveClockwiseTime(double power, int time) {
		setToClockwise(power);
		waiter(time);
		setToStill();
	}
	public void driveCounterwiseTime(double power, int time) {
		setToCounterwise(power);
		waiter(time);
		setToStill();
	}
	public void driveLeftTime(double power, int time) {
		setToLeft(power);
		waiter(time);
		setToStill();
	}
	public void driveRightTime(double power, int time) {
		setToRight(power);
		waiter(time);
		setToStill();
	}

	//setTo methods
	//TODO: there may be a more efficient way to write these methods
	public void setToForward(double power) {
		frontLeft.setPower(power);
		frontRight.setPower(power);
		backLeft.setPower(power);
		backRight.setPower(power);
	}
	public void setToBackward(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToCounterwise(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(1 * power);
	}
	public void setToClockwise(double power) {
		frontLeft.setPower(1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToRight(double power) {
		frontLeft.setPower(1 * power);
		frontRight.setPower(-1 * power);
		backLeft.setPower(-1 * power);
		backRight.setPower(1 * power);
	}
	public void setToLeft(double power) {
		frontLeft.setPower(-1 * power);
		frontRight.setPower(1 * power);
		backLeft.setPower(1 * power);
		backRight.setPower(-1 * power);
	}
	public void setToStill() {
		frontLeft.setPower(0);
		frontRight.setPower(0);
		backLeft.setPower(0);
		backRight.setPower(0);
	}

	public double getGyroAvgZ1(){
		Orientation orient1 = rev1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		double angle1 = Double.parseDouble(formatAngle(orient1.angleUnit, orient1.firstAngle));
		return angle1;
	}
	public double getGyroAvgZ2(){
		Orientation orient2 = rev2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		double angle2 = Double.parseDouble(formatAngle(orient2.angleUnit, orient2.firstAngle));
		return angle2;
	}

	//TODO: must change so that it can work in SET_TO_POSTITION mode, maybe one for both modes
	/*public void turnAbsoluteUsingEncoder(int target, double power) {//for RUN_USING_ENCODER mode
		double zAccumulated = getGyroAvgZ();  //Set variables to gyro readings

		while (Math.abs(zAccumulated - target)  > turningPrecisionRequirement){//while off target
			if (zAccumulated > target)//if gyro is positive, turn counterwise
				setToCounterwise(power);
			if (zAccumulated < target)//if gyro is negative, turn clockwise
				setToClockwise(power);
		}
		setToStill();
	}*/

	/*public void turnAbsoluteRunToPosition(int target, double power) {//for RUN_TO_POSITION mode
		double zAccumulated = getGyroAvgZ();  //Set variables to gyro readings

		while (Math.abs(zAccumulated - target)  > turningPrecisionRequirement){//Continue while the robot direction is further than three degrees from the target
			if (zAccumulated > target) {//if gyro is positive, turn counterwise
				frontLeft.setTargetPosition(frontLeft.getCurrentPosition()-turnSpeedForEncoderRunToPosition);
				frontRight.setTargetPosition(frontRight.getCurrentPosition()+turnSpeedForEncoderRunToPosition);
				backLeft.setTargetPosition(backLeft.getCurrentPosition()-turnSpeedForEncoderRunToPosition);
				backRight.setTargetPosition(backRight.getCurrentPosition()+turnSpeedForEncoderRunToPosition);
			}
			if (zAccumulated < target){//if gyro is negative, turn clockwise
				frontLeft.setTargetPosition(frontLeft.getCurrentPosition()+turnSpeedForEncoderRunToPosition);
				frontRight.setTargetPosition(frontRight.getCurrentPosition()-turnSpeedForEncoderRunToPosition);
				backLeft.setTargetPosition(backLeft.getCurrentPosition()+turnSpeedForEncoderRunToPosition);
				backRight.setTargetPosition(backRight.getCurrentPosition()-turnSpeedForEncoderRunToPosition);
			}
		setToStill();
	}*/

	public RelicRecoveryVuMark waitUntilVuMarkIsFound() { //returns a distance for the robot to travel
		RelicRecoveryVuMark returner=RelicRecoveryVuMark.UNKNOWN;
		ElapsedTime check=new ElapsedTime();

		RelicRecoveryVuMark vuMark=RelicRecoveryVuMark.from(relicTemplate);
		while(vuMark==RelicRecoveryVuMark.UNKNOWN){
			vuMark = RelicRecoveryVuMark.from(relicTemplate);
			if (vuMark != RelicRecoveryVuMark.UNKNOWN){
				telemetry.addData("VuMark", "%s visible", vuMark);
				return vuMark;
			}
			else
				telemetry.addData("VuMark", "not visible");
			if(check.milliseconds()>vuforiaTimeoutTime) //if it takes too long to find a vuMark quit the method
				break;
			telemetry.update();
		}
		return RelicRecoveryVuMark.UNKNOWN;
	}
}