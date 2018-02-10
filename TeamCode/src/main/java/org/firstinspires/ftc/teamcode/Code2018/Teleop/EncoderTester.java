package org.firstinspires.ftc.teamcode.Code2018.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;
//the purpose of this program is to test specific user controlled distances, and be able to see the
//corresponding encoder value necessary to reach that distance in RUN_TO_POSITION mode
//due to the simplistic nature of the program, it will only work on movements where all motors move
//by a common distance, i.e. turns in place, forward + backward, left + right
@TeleOp(name="EncoderTester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
     //Determines if the program shows up on Driver Station


public class EncoderTester extends OpMode
{
	Scorpion r=new Scorpion();
	ElapsedTime runtime=new ElapsedTime();
	private static final int encoderStepValue=10;
	public void init() {
		r.initRobot(hardwareMap,telemetry);
		telemetry.addData("init Complete","");

		telemetry.update();
	}

	public void initLoop(){}

	public void start(){
		runtime.reset();
	}

	int encoderVal=0;
	public void loop() {
		if(gamepad1.left_stick_y<0) {//going forward
			encoderVal+=encoderStepValue;
			r.setMotorEncoderForward(encoderVal);
		}
		else if(gamepad1.left_stick_y>0) {//going backward
			encoderVal+=encoderStepValue;
			r.setMotorEncoderBackward(encoderVal);
		}
		else if(gamepad1.left_stick_x>0) {//going left
			encoderVal+=encoderStepValue;
			r.setMotorEncoderLeft(encoderVal);
		}
		else if(gamepad1.left_stick_x<0) {//going right
			encoderVal+=encoderStepValue;
			r.setMotorEncoderRight(encoderVal);
		}
		else if(gamepad1.right_stick_x<0) {//counterwise forward
			encoderVal+=encoderStepValue;
			r.setMotorEncoderCounterwise(encoderVal);
		}
		else if(gamepad1.right_stick_x>0) {//clockwise forward
			encoderVal+=encoderStepValue;
			r.setMotorEncoderClockwise(encoderVal);
		}
	}
}
