/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.teamcode;

import android.os.StrictMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import java.lang.reflect.Array;
//import java.util.Collections;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */

@Autonomous(name="Camera (P 2:10 11/10/2020)", group="Testing")
public class CameraPatch_210_111020 extends LinearOpMode {
	OpenCvCamera phoneCam;
	//String hsvRange;
	String fps;
	Pipeline pipeline;
	Telemetry.Item hsvTele, oIDTele, oStrTele;

	@Override
	public void runOpMode() {
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

		// OR...  Do Not Activate the Camera Monitor View
		//phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
		//phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

		phoneCam.setPipeline(new Pipeline());

		/*
		 * Open the connection to the camera device. New in v1.4.0 is the ability
		 * to open the camera asynchronously, and this is now the recommended way
		 * to do it. The benefits of opening async include faster init time, and
		 * better behavior when pressing stop during init (i.e. less of a chance
		 * of tripping the stuck watchdog)
		 *
		 * If you really want to open synchronously, the old method is still available.
		 */
		phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			                               @Override
			                               public void onOpened() {
				                               /*
				                                * Tell the camera to start streaming images to us! Note that you must make sure
				                                * the resolution you specify is supported by the camera. If it is not, an exception
				                                * will be thrown.
				                                *
				                                * Also, we specify the rotation that the camera is used in. This is so that the image
				                                * from the camera sensor can be rotated such that it is always displayed with the image upright.
				                                * For a front facing camera, rotation is defined assuming the user is looking at the screen.
				                                * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
				                                * away from the user.
				                                */

				                               phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
			                               }
		                               }
		);

		telemetry.addLine("Waiting for start");
		telemetry.update();

		/*
		 * Wait for the user to press start on the Driver Station
		 */
		waitForStart();
		resetStartTime();

		while (opModeIsActive()) {
			/*
			 * Send some stats to the telemetry
			 */
			//fps = String.format("%.2f", phoneCam.getFps());
			//telemetry.addData("Frame Count", phoneCam.getFrameCount());
			telemetry.addData("FPS","%.2f", phoneCam.getFps());
			telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
			telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
			//telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
			telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
			oIDTele = telemetry.addData("Current Mode ID", pipeline.oID + ": " + pipeline.oStr);
			hsvTele = telemetry.addData("HSV Range", pipeline.hsvlog);
			oStrTele = telemetry.addData("Stopwatch", getRuntime());
			telemetry.update();

			if(gamepad1.a) {
				phoneCam.stopStreaming();
				//phoneCam.closeCameraDevice();
			}

			/*
			 * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
			 * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
			 * anyway). Of course in a real OpMode you will likely not want to do this.
			 */
			sleep(100);
		}
		StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
		StrictMode.setThreadPolicy(policy);
	}

	//public void monitor() { }

	class Pipeline extends OpenCvPipeline {
		CameraPatch_210_111020 internalCamera1 = new CameraPatch_210_111020();

		// Highest/Lowest Individual Recorded values:
		// W/O Assistance:  Low(21,217,153)     High= HLV(31,236,230)
		// W/ Assistance :  Low(21,217,153)     High= HLV(49,217,230)
		int[] hsvhigh = new int[]{49,255,255};
		int[] hsvlow = new int[]{15,150,150};


		// get thresholding values from the UI
		// remember: H ranges 0-180, S and V range 0-255
		Scalar maxValues = new Scalar(hsvhigh[0], hsvhigh[1], hsvhigh[2]);
		Scalar minValues = new Scalar(hsvlow[0], hsvlow[1], hsvlow[2]);

		Mat blurredImage, hsvImage, mask, output, hierarchy, maskedImage, morphOutput, dst;
		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
		//Mat tmpMat = new Mat();

		MatOfPoint2f approxCurve,contour2f;
		MatOfPoint points;

		Rect rect;
		double approxDistance;
		int x = 0;
		String hsvlog;
		int oID = 0;
		String oStr = "";

		/*
		 * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
		 * highly recommended to declare them here as instance variables and re-use them for
		 * each invocation of processFrame(), rather than declaring them as new local variables
		 * each time through processFrame(). This removes the danger of causing a memory leak
		 * by forgetting to call mat.release(), and it also reduces memory pressure by not
		 * constantly allocating and freeing large chunks of memory.
		 */

		@Override
		public Mat processFrame(Mat input) {
			// remove some noise & convert the frame to HSV
			Imgproc.blur(input, blurredImage, new Size(7, 7));
			Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_RGB2HSV);

			// show the current selected HSV range
			hsvlog = "Hue range: " + minValues.val[0] + "-" + maxValues.val[0] + "\tSaturation range: "
					+ minValues.val[1] + "-" + maxValues.val[1] + "\tValue range: "
					+ minValues.val[2] + "-" + maxValues.val[2];

			// threshold HSV image to select tennis balls
			Core.inRange(hsvImage, minValues, maxValues, mask);

			// morphological operators
			// dilate with large element, erode with small ones
			// Try using MORPH_ELLIPSE
			Imgproc.erode(mask, morphOutput, erodeElement);
			Imgproc.erode(mask, morphOutput, erodeElement);

			Imgproc.dilate(mask, morphOutput, dilateElement);
			Imgproc.dilate(mask, morphOutput, dilateElement);

			// init
			List<MatOfPoint> contours = new ArrayList<>();

			// find contours
			Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

			// if any contour exist...
			if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
				// for each contour, display it in blue
				for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
					Imgproc.drawContours(maskedImage, contours, idx,
							new Scalar(250, 0, 0),10);
					// Minimum size allowed for consideration
					contour2f = new MatOfPoint2f( contours.get(idx).toArray());
					//Point[] testArray = contours.get(idx).toArray();

					//Processing on mMOP2f1 which is in type MatOfPoint2f
					approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
					Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

					//Convert back to MatOfPoint
					points = new MatOfPoint( approxCurve.toArray());

					// Get bounding rect of contour
					rect = Imgproc.boundingRect(points);
					Imgproc.rectangle(dst, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0, 100), 3);
				}
			}

			switch (oID) {
				case 0:
					output = dst.clone();
					oStr = "dst (default out)";
					//new InternalCamera1().monitor(oStr,oID,);
					//x++;
					break;
				case 1:
					output = input.clone();
					oStr = "input (unchanged in)";
					//x++;
					break;
				case 2:
					output = pipeline.blurredImage.clone();
					oStr = "blurredImage";
					//x++;
					break;
				case 3:
					output = hsvImage.clone();
					oStr = "hsvImage";
					//x++;
					break;
				case 4:
					output = mask.clone();
					oStr = "mask";
					//x++;
					break;
				case 5:
					output = morphOutput.clone();
					oStr = "morphOutput[0] (erode #1)";
					//x++;
					break;
				case 6:
					output = maskedImage.clone();
					oStr = "maskedImage";
					//x++;
					break;
				case 7:
					output = morphOutput.clone();
					oStr = "morphOutput";
					//x++;
					break;
				case 8:
					output = input.clone();
					oStr = "input (Unchanged Output)";
					//x = 1;
					break;
				default:
					throw new IllegalStateException("Unexpected value: " + oID);
			}

			// Set Telemetry
			/*
			oIDTele.setValue(oID);
			hsvTele.setValue(hsvlog);
			oStrTele.setValue(oStr);
			*/
			return output;
		}

		@Override
		public void onViewportTapped() {
			/*
			 * The viewport (if one was specified in the constructor) can also be dynamically "paused"
			 * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
			 * when you need your vision pipeline running, but do not require a live preview on the
			 * robot controller screen. For instance, this could be useful if you wish to see the live
			 * camera preview as you are initializing your robot, but you no longer require the live
			 * preview after you have finished your initialization process; pausing the viewport does
			 * not stop running your pipeline.
			 *
			 * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
			 */
			if (x>=0 && x<=7) oID = x;
			else if (x>7) oID = 0;
			else {
				telemetry.addData("Status","Exitting");
				telemetry.setAutoClear(false);
				telemetry.addData("Exit Status", "Welp, You broke it.  (onViewportTapped())");
				telemetry.addData("Exit Status", "Stopping everything...");
				phoneCam.stopStreaming();
				telemetry.addData("Status", "Stopped");
				stop();
			}


			//viewportPaused = !viewportPaused;
			//if (viewportPaused) phoneCam.pauseViewport();
			//else phoneCam.resumeViewport();
		}
	}
}