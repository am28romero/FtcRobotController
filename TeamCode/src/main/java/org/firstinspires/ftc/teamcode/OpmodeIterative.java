/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

@TeleOp(name="Main OpMode", group="Releases")
public class OpmodeIterative extends OpMode {
	// Configurations
	//final int topHandler = 1;
	///final String decFormat = "0.000";
	//final String telemetryFloatFormat = "%.3f";
	//final double softStartThreshold = 0.05;
	final double[] target = {0.8,0.8,-0.9,0.2};

	// Declare members.
	private final ElapsedTime runtime = new ElapsedTime();
	final public DcMotor[] driveMotors = new DcMotor[4];
	final public DcMotor[] motors = new DcMotor[3];
	Servo shooterServo;
	final double[] motorSpeeds = new double[driveMotors.length+motors.length];
	//double fb,lr,turn,extraAxis;
	boolean[] toggle = new boolean[2];
	double[] stopwatch;

	// Code to run ONCE when the driver hits INIT
	@Override
	public void init() {
		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		//try {
		driveMotors[0] = hardwareMap.get(DcMotor.class, "frontLeft");
		driveMotors[1] = hardwareMap.get(DcMotor.class, "frontRight");
		driveMotors[2] = hardwareMap.get(DcMotor.class, "backLeft");
		driveMotors[3] = hardwareMap.get(DcMotor.class, "backRight");
		motors[0] = hardwareMap.get(DcMotor.class, "intakeMotor");
		motors[1] = hardwareMap.get(DcMotor.class, "conveyorMotor");
		motors[2] = hardwareMap.get(DcMotor.class, "shooterMotor");
		shooterServo = hardwareMap.get(Servo.class, "shooterServo");
		//} catch (Exception e) {
		//	telemetry.addData("Initialization Status", "One or more of the devices is not configured correctly.\n" + e.getCause());
		//	Log.w("Initialization Status", "One or more of the devices is not configured correctly.\n" + e.getCause());
		//}

		telemetry.addData("Status", "Initialized");
	}

	//Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

	@Override
	public void init_loop() {}

	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() {
		runtime.reset();
	}

	@Override
	public void loop() {
		gamepadHandler();
		telemetry.addData("Run Time", "%.3f", runtime.milliseconds()/1000);
	}

	private void gamepadHandler()  {
		if (runtime.milliseconds()>0) {
			if (gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_y > 0.15) {
				// - This uses basic math to combine motions and is easier to drive straight.
				Utils.mecanumDrive_Cartesian(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
			}
			if (gamepad1.right_trigger > 0.3)
				shooterServo.setPosition(target[3]);
			//					for (int i =0; i<(target[3]/5);i++)
			//						shooterServo.setPosition(i*5);
			if (gamepad1.left_trigger > 0.5 && runtime.milliseconds() - stopwatch[0] < 500) {
				if (!toggle[0]) {
					motors[0].setPower(target[0]);
					motors[1].setPower(target[1]);
					motors[2].setPower(target[2]);
				} else {
					motors[0].setPower(0);
					motors[1].setPower(0);
					motors[2].setPower(0);
				}

				toggle[0] = !toggle[0];
			}
			if (gamepad1.start && gamepad1.back) {
				for (DcMotor motor : driveMotors) motor.setPower(0);
				for (DcMotor motor : motors) motor.setPower(0);
				Log.i("Emergency Stop:", "Activated, Stopping Everything");
				stop();
			}
		}
	}

	public static class Utils extends OpmodeIterative {
		private static final Utils utils = new Utils();
		public static void display(String group) { utils.m_display(group); }
		public static void mecanumDrive_Cartesian(double y, double x, double rotation) { utils.m_mecanumDrive_Cartesian(y,x,rotation); }
		public static final String
				ShooterMotors = "ShooterMoters",
				DriveMotors = "DriveMotors",
				Gpad = "Gpad";

		private void m_display(String group) {
			switch (group) {
				case "ShooterMoters":
					telemetry.addLine("Shooter Motors" + "\r\n")
							.addData("intake", "%.3f", motors[0].getPower())
							.addData("conveyor", "%.3f", motors[1].getPower())
							.addData("shooter", "%.3f", motors[2].getPower());
					break;
				case "DriveMotors":
					telemetry.addLine("Drive Motors" + "\r\n")
							.addData("frontLeft", "%.3f", driveMotors[0].getPower())
							.addData("frontRight", "%.3f", driveMotors[1].getPower())
							.addData("backtLeft", "%.3f", driveMotors[2].getPower())
							.addData("backRight", "%.3f", driveMotors[3].getPower());
					break;
				case "Gpad":
					telemetry.addLine("Gamepad" + "\r\n")
							.addData("LeftX", "%.3f", gamepad1.left_stick_x)
							.addData("LeftY", "%.3f", gamepad1.left_stick_y)
							.addData("RightX", "%.3f", gamepad1.right_stick_x)
							.addData("RightY", "%.3f", gamepad1.right_stick_y);
					break;
			}
		}

		private void m_mecanumDrive_Cartesian(double y, double x, double rotation) {
			motorSpeeds[0] = y+x+rotation;
			motorSpeeds[1] = y-x-rotation;
			motorSpeeds[2] = y-x+rotation;
			motorSpeeds[3] = y+x-rotation;

			double top = Math.max(Math.abs(motorSpeeds[0]), Math.max(Math.abs(motorSpeeds[1]),
					Math.max(Math.abs(motorSpeeds[2]), Math.abs(motorSpeeds[3]))));


			// 1. Divides wheelSpeeds proportionally
			for (int i = 0; i < motorSpeeds.length; i++) {
				if (motorSpeeds[i]>1.0 || motorSpeeds[i]<-1.0) {
					motorSpeeds[i] = Double.parseDouble(new DecimalFormat("0.000").format(motorSpeeds[i] / top));
				}
			}

			// Send calculated power to motors
			m_display(DriveMotors);
			driveMotors[0].setPower(-motorSpeeds[0]);
			driveMotors[1].setPower(motorSpeeds[1]);
			driveMotors[2].setPower(-motorSpeeds[2]);
			driveMotors[3].setPower(motorSpeeds[3]);
		}

	}

	@Override
	public void stop() {
		// Motor SoftStop
		/*for (DcMotor motor: driveMotors) {
			while (motor.getPower()>=softStartThreshold) {
				motor.setPower(motor.getPower()-softStartThreshold);
				display("Motors");
			}
			while (motor.getPower()<=-softStartThreshold) {
				motor.setPower(motor.getPower()+softStartThreshold);
				display("Motors");
			}
			if (motor.getPower()<=softStartThreshold*2 && motor.getPower()>=-softStartThreshold*2) motor.setPower(0);
		} */
		Utils.display("Motors");
		telemetry.addData("Status", "Stopped");
	}
}