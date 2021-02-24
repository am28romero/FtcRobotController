package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;


@TeleOp(name="Main OpMode (debug)", group="Debug")
public class Debug_OpmodeIterative extends OpMode {
	// Configurations

	// Declare members.
	private final ElapsedTime runtime = new ElapsedTime();
	private final DcMotor[] driveMotors = new DcMotor[4];
	private final DcMotor[] motors = new DcMotor[4];
	private Servo shooterServo;
	private final double[] wheelSpeeds = new double[4];
	boolean toggleMotor;
	//double fb, lr, turn, extraAxis;


	@Override
	public void init() {
		driveMotors[0] = hardwareMap.get(DcMotor.class, "frontLeft");
		driveMotors[1] = hardwareMap.get(DcMotor.class, "frontRight");
		driveMotors[2] = hardwareMap.get(DcMotor.class, "backLeft");
		driveMotors[3] = hardwareMap.get(DcMotor.class, "backRight");
		motors[0] = hardwareMap.get(DcMotor.class, "intakeMotor");
		motors[1] = hardwareMap.get(DcMotor.class, "conveyorMotor");
		motors[2] = hardwareMap.get(DcMotor.class, "shooterMotor");
		shooterServo = hardwareMap.get(Servo.class, "shooterServo");
	}

	@Override
	public void init_loop() {}

	@Override
	public void start() {
		shooterServo.setPosition(0.5);
		for (DcMotor driveMotor:driveMotors)
			driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		for (DcMotor motor:motors)
			motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void loop() {
		//fb = -gamepad1.left_stick_y;
		//lr = gamepad1.left_stick_x;
		//turn  = gamepad1.right_stick_x;
		//extraAxis = -gamepad1.right_stick_y;

		//- This uses basic math to combine motions and is easier to drive straight.
		mecanumDrive_Cartesian(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

		if (gamepad1.x) {
			motors[0].setPower(0.8);
			motors[1].setPower(0.8);
			//toggleMotor = !toggleMotor;
		}
		if (gamepad1.b)
			for (DcMotor motor:motors) motor.setPower(0.0);
		if (gamepad1.left_trigger>0.3)
			for (int i=0; motors[2].getPower()<0.9; i++) motors[2].setPower(0.04*i);

		if (gamepad1.right_trigger>0.3 && shooterServo.getPosition()>0.2) shooterServo.setPosition(0.2);
		else if (gamepad1.right_trigger<=0.3 && shooterServo.getPosition()<0.5) shooterServo.setPosition(0.5);
		else if (shooterServo.getPosition()<0.2 || shooterServo.getPosition()>0.5) shooterServo.setPosition(0.5);

//		if (gamepad1.x) {
//			//if (!toggleMotor) {
//			motors[0].setPower(0.8);
//			motors[1].setPower(0.8);
//			motors[2].setPower(-0.9);
//			} else for (DcMotor motor:motors) motor.setPower(0.0);
//			//toggleMotor = !toggleMotor;
//		}
	}

	public void mecanumDrive_Cartesian(double y,double x,double rotation) {
		wheelSpeeds[0] = y+x+rotation;
		wheelSpeeds[1] = y-x-rotation;
		wheelSpeeds[2] = y-x+rotation;
		wheelSpeeds[3] = y+x-rotation;

		double top = Math.max(Math.abs(wheelSpeeds[0]),
				Math.max(Math.abs(wheelSpeeds[1]),
						Math.max(Math.abs(wheelSpeeds[2]), Math.abs(wheelSpeeds[3]))));

		// 1. Divides wheelSpeeds proportionally
		for (int i = 0; i < wheelSpeeds.length; i++)
			if (wheelSpeeds[i] > 1.0 || wheelSpeeds[i] < -1.0)
				wheelSpeeds[i] = Double.parseDouble(new DecimalFormat("0.000").format(wheelSpeeds[i] / top));

		// Send calculated power to motors
		driveMotors[0].setPower(-wheelSpeeds[0]);
		driveMotors[1].setPower(wheelSpeeds[1]);
		driveMotors[2].setPower(-wheelSpeeds[2]);
		driveMotors[3].setPower(wheelSpeeds[3]);
	}

	/*
	private void gamepadHandler()  {
		if (gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_y > 0.15) {
			// - This uses basic math to combine motions and is easier to drive straight.
			mecanumDrive_Cartesian(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
		}
		if (gamepad1.right_trigger > 0.3) {
			shooterServo.setPosition(0.5);
		}
		if (gamepad1.x) {
			if (!toggleMotor) {
				motors[0].setPower(0.8);
				motors[1].setPower(0.8);
				motors[2].setPower(-0.9);
			} else {
				motors[0].setPower(0);
				motors[1].setPower(0);
				motors[2].setPower(0);
			}

			toggleMotor = !toggleMotor;
		}
		/*
		if (gamepad1.start && gamepad1.back) {
			for (DcMotor motor : driveMotors) motor.setPower(0);
			for (DcMotor motor : motors) motor.setPower(0);
			Log.i("Emergency Stop:", "Activated, Stopping Everything");
			stop();
		}
	}*/

	@Override
	public void stop() {
		// Slowly turn off the motors after pressing stop
		driveMotors[0].setPower(0.0);
		driveMotors[1].setPower(0.0);
		driveMotors[2].setPower(0.0);
		driveMotors[3].setPower(0.0);
	}
}