package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;


@TeleOp(name="Latest Optimized Release", group="Releases")
public class Debug_OpmodeIterative extends OpMode {
	// Configurations

	// Declare members.
	private final ElapsedTime runtime = new ElapsedTime();
	private final DcMotor[] driveMotors = new DcMotor[4];
	private final DcMotor[] shooterMotors = new DcMotor[3];
	private DcMotor armMotor;
	private Servo shooterServo, armServo;
	private final double[] wheelSpeeds = new double[4];
	//double fb, lr, turn, extraAxis;

	@Override
	public void init() {
		driveMotors[0] = hardwareMap.get(DcMotor.class, "frontLeft");
		driveMotors[1] = hardwareMap.get(DcMotor.class, "frontRight");
		driveMotors[2] = hardwareMap.get(DcMotor.class, "backLeft");
		driveMotors[3] = hardwareMap.get(DcMotor.class, "backRight");
		shooterMotors[0] = hardwareMap.get(DcMotor.class, "intakeMotor");
		shooterMotors[1] = hardwareMap.get(DcMotor.class, "conveyorMotor");
		shooterMotors[2] = hardwareMap.get(DcMotor.class, "shooterMotor");
		armMotor = hardwareMap.get(DcMotor.class, "armMotor");
		shooterServo = hardwareMap.get(Servo.class, "shooterServo");
		armServo = hardwareMap.get(Servo.class, "armServo");
	}

	@Override
	public void init_loop() {}

	@Override
	public void start() {
		shooterServo.setPosition(0.5);
	}

	@Override
	public void loop() {
		gamepadHandler();
	}

	public void mecanumDrive_Cartesian(double y,double x,double rotation) {
		wheelSpeeds[0] = -(y+x+rotation);
		wheelSpeeds[1] = y-x-rotation;
		wheelSpeeds[2] = -(y-x+rotation);
		wheelSpeeds[3] = y+x-rotation;

		double top = Math.max(Math.abs(wheelSpeeds[0]), Math.max(Math.abs(wheelSpeeds[1]),
				Math.max(Math.abs(wheelSpeeds[2]), Math.abs(wheelSpeeds[3]))));

		// Divides wheelSpeeds proportionally
		if (Math.abs(top)>1.0) for (int i = 0; i < wheelSpeeds.length; i++)
			if (wheelSpeeds[i] > 1.0 || wheelSpeeds[i] < -1.0)
				wheelSpeeds[i] = wheelSpeeds[i] / top;

		// Send calculated power to motors
		for (int i = 0; i < driveMotors.length; i++) driveMotors[i].setPower(wheelSpeeds[i]);
	}

	private void gamepadHandler()  {
		//- This uses basic math to combine motions and is easier to drive straight.
		mecanumDrive_Cartesian(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

		boolean curr[] = {gamepad1.x, gamepad1.left_trigger>0.3, gamepad1.dpad_right};
		boolean prev[] = new boolean[curr.length];

		// If the previous X-button input is the same as the current input
		if (curr[0] && !prev[0]) {
			shooterMotors[0].setPower(0.8);
			shooterMotors[1].setPower(0.8);
		}
		prev[0] = curr[0];

		if (gamepad1.left_trigger>0.3 && !prev[1])
			for (int i = 0; shooterMotors[2].getPower()<0.9; i++) shooterMotors[2].setPower(0.04*i);
		if (gamepad1.dpad_right && !prev[2]) {
			while (armMotor.getCurrentPosition() < 200) armMotor.setPower(0.4);
			armMotor.setPower(0.0);
		}
		prev = curr;

		if (gamepad1.right_trigger>0.3 && shooterServo.getPosition()>0.2) shooterServo.setPosition(0.2);
		else if (gamepad1.right_trigger<=0.3 && shooterServo.getPosition()<0.5) shooterServo.setPosition(0.5);
		else if (shooterServo.getPosition()<0.2 || shooterServo.getPosition()>0.5) shooterServo.setPosition(0.5);

		if (gamepad1.start && gamepad1.back) {
			for (DcMotor motor : driveMotors) motor.setPower(0);
			for (DcMotor motor : shooterMotors) motor.setPower(0);
			armMotor.setPower(0);
			Log.i("Emergency Stop:", "Activated, Stopping Everything");
			stop();
		}
	}

	@Override
	public void stop() {
		// Slowly turn off the motors after pressing stop
		driveMotors[0].setPower(0.0);
		driveMotors[1].setPower(0.0);
		driveMotors[2].setPower(0.0);
		driveMotors[3].setPower(0.0);
	}
}