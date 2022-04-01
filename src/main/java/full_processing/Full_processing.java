package Full_processing;

import java.math.BigInteger;
import java.util.Arrays;
import java.util.*;

import com.kauailabs.vmx.*;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;




public class Full_processing{

	static {
		try {
			System.loadLibrary("vmxpi_hal_java");
		} catch (UnsatisfiedLinkError e) {
			System.err.println("Native code library failed to load. See the chapter on Dynamic Linking Problems in the SWIG Java documentation for help.\n" + e);
      			System.exit(1);
			}
		
		
  	}



	public static void main(String argv[]) {
		
		VisionThread targetThread;
		VisionThread driveThread;
		
		final Object targetLock = new Object();
		final Object driveLock = new Object();
		
		int[] totalContours = {42};
		int[] targetContours={555};
		int[] targetCenterX = {360};
		int[] ballContours={555};
		int[] ballCenterX = {400};
		int[] ballCenterY = {300};
		int[] ballLoopCount = {0};
		int distance = -10;
		int strength = -10;
		int temperature = -10;
		int[] vmxerr = {0};
		short[] uart_res_handle = {29};
		
		
		final String TargOutputVideoFilePath = ""; //"target_cam_"+(java.time.LocalDateTime.now())+".avi"; /* Set to null if video writing not desired */
		final String DriveOutputVideoFilePath = ""; //"drive_cam_"+(java.time.LocalDateTime.now())+".avi"; /* Set to null if video writing not desired */
		final String CombinedOutputVideoFilePath = ""; //"combined_cams_"+(java.time.LocalDateTime.now())+".avi"; /* Set to null if video writing not desired */

		/* Open communication to VMX-pi, to acquire IMU data */
		VMXPi vmx = new VMXPi(false, (byte)50);
		if (!vmx.IsOpen()) {
			System.out.println("Error:  Unable to open VMX Client.");
			System.out.println("");
			System.out.println("        - Is pigpio (or the system resources it requires) in use by another process?");
			System.out.println("        - Does this application have root privileges?");
			vmx.delete();  // Immediately dispose of all resources used by vmx object.
			System.exit(1);
		}

		// Adding from UART example

		

			VMXChannelInfo[] uart_channels = { new VMXChannelInfo(vmx.getIO().GetSoleChannelIndex(VMXChannelCapability.UART_TX),
								VMXChannelCapability.UART_TX),
							  new VMXChannelInfo(vmx.getIO().GetSoleChannelIndex(VMXChannelCapability.UART_RX),
								VMXChannelCapability.UART_RX) };
			
			UARTConfig uart_cfg = new UARTConfig(115200);
	
			if (!vmx.getIO().ActivateDualchannelResource(	uart_channels[0], 
															uart_channels[1], 
															uart_cfg, uart_res_handle, 
															vmxerr)) 
															{
				System.out.printf("Failed to Activate UART Resource.\n");

				vmx.getIO().DeallocateResource(uart_res_handle[0], vmxerr);
			} else {
				System.out.printf("Successfully Activated UART Resource with VMXChannels %d and %d\n", uart_channels[0].getIndex(), uart_channels[1].getIndex());
			}
	
			short bytes_available[] = {0};
			/* Allocate (native) Direct Buffer, large enough to hold the largest-possible transfer size */
			DirectBuffer rcv_bytes = new DirectBuffer(16384);
	


		//*******************************************************************************************************************

		/* Connect NetworkTables */
		/* Note:  actual IP address should be robot IP address */
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		//inst.startClient("10.205.0.148");
		inst.startClient("roboRIO-6591-FRC.local");

		
		
		
		/* Open connection to USB Camera (video device 0 [/dev/video0]) */
		UsbCamera targetCamera = new UsbCamera("targetcam", "/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0");
		UsbCamera driveCamera = new UsbCamera("drivecam", "/dev/v4l/by-id/usb-HD_Camera_Manufacturer_Stereo_Vision_1_Stereo_Vision_1-video-index0");


		

		/* Configure Target Camera */
		/* Note:  Higher resolution & framerate is possible, depending upon processing cpu usage */
		int targWidth = 800;
		int targHeight = 600;
		int targFrames_per_sec = 30;
		targetCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, targWidth, targHeight, targFrames_per_sec);
		targetCamera.setWhiteBalanceAuto();
		targetCamera.setBrightness(3);
		targetCamera.setExposureManual(3);

		/* Configure Drive Camera */
		int driveWidth = 800;
		int driveHeight = 600;
		int driveFrames_per_sec = 30;
		driveCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, driveWidth, driveHeight, driveFrames_per_sec);
		driveCamera.setWhiteBalanceAuto();
		driveCamera.setBrightness(5);
		driveCamera.setExposureAuto();		
		

		CvSink targetSink = new CvSink("targetSink");
		targetSink.setSource(targetCamera);

		CvSink driveSink = new CvSink("driveSink");
		driveSink.setSource(driveCamera);


		// Start target Video Streaming Server 
		CvSource targetSource = new CvSource("targetSource",
				VideoMode.PixelFormat.kMJPEG, targWidth, targHeight, targFrames_per_sec);

		MjpegServer targetVideoServer = new MjpegServer("target_video_server", 1181);
		targetVideoServer.setSource(targetCamera);

		

		// Start drive Video Streaming server 
		CvSource driveSource = new CvSource("driveSource",
				VideoMode.PixelFormat.kMJPEG, driveWidth, driveHeight, driveFrames_per_sec);
		
		MjpegServer driveVideoServer = new MjpegServer("drive_video_server", 1182);
		driveVideoServer.setSource(driveCamera);

		// Start combined Video Streaming sever
		CvSource combinedSource = new CvSource("driveSource",
				VideoMode.PixelFormat.kMJPEG, driveWidth+targWidth, driveHeight, driveFrames_per_sec);
		
		MjpegServer combinedVideoServer = new MjpegServer("combined_video_server", 1183);
		combinedVideoServer.setSource(combinedSource);


		// Create Video Writer, if enabled 
		Size targFrameSize = new Size(targWidth, targHeight);
		VideoWriter targVideoWriter = new VideoWriter(TargOutputVideoFilePath,
			VideoWriter.fourcc('M', 'J', 'P', 'G'), (double)targFrames_per_sec, targFrameSize, true);

		Size driveFrameSize = new Size(driveWidth, driveHeight);
		VideoWriter driveVideoWriter = new VideoWriter(DriveOutputVideoFilePath,
			VideoWriter.fourcc('M', 'J', 'P', 'G'), (double)driveFrames_per_sec, driveFrameSize, true);

		Size combinedFrameSize = new Size(driveWidth+targWidth, driveHeight);
		VideoWriter combinedVideoWriter = new VideoWriter(CombinedOutputVideoFilePath,
			VideoWriter.fourcc('M', 'J', 'P', 'G'), (double)driveFrames_per_sec, combinedFrameSize, true);
			






		//***************** Pre-allocate a video frames ***************************
		Mat targFrame = new Mat(targFrameSize, CvType.CV_8UC(3));
		Mat processedTargFrame = new Mat(targFrameSize, CvType.CV_8UC(3));
		
		Mat driveFrame = new Mat(driveFrameSize,CvType.CV_8UC(3));
		Mat processedDriveFrame = new Mat(driveFrameSize, CvType.CV_8UC(3));

		Mat combinedFrame = new Mat(combinedFrameSize, CvType.CV_8UC(3));
		
		Mat hierarchy = new Mat();

			
			
		//*******************************************************************************************
		targetThread = new VisionThread(targetCamera, new TargetPipeline(), targPipeline -> {

			synchronized (targetLock) {

				inst.getEntry("/vmx/ContourLoopCount").setNumber(targPipeline.contourloop);
				inst.getEntry("/vmx/FilterLoopCount").setNumber(targPipeline.filterloop);
				inst.getEntry("/vmx/TotalContoursFound").setNumber(targPipeline.findContoursOutput().size());
				inst.getEntry("/vmx/TargetContoursFound").setNumber(targPipeline.filterContoursOutput().size());
				inst.getEntry("/vmx/TargetCenterX").setNumber(targetCenterX[0]);

				targetSink.grabFrame(processedTargFrame);

				
				// Overlay Target contours
				for (int i = 0; i < targPipeline.filterContoursOutput().size(); ++i) {
					Scalar color = new Scalar(0,0,255);
					Imgproc.drawContours(processedTargFrame, targPipeline.filterContoursOutput(), i, color, 1, Imgproc.LINE_8, hierarchy, 3);
					Rect b = Imgproc.boundingRect(targPipeline.filterContoursOutput.get(i));
					Imgproc.rectangle(processedTargFrame, b.tl(), b.br(), color, 2);

				};

								
				if (targPipeline.filterContoursOutput().isEmpty()){
					targetCenterX[0]=5555;
								
				};
		
				
				if (!targPipeline.filterContoursOutput().isEmpty()) {
					
					if (targPipeline.filterContoursOutput().size() > 1) {
						//SmartDashboard.putString("Target Contour:", " Well Crap! " +String.valueOf(targPipeline.filterContoursOutput().size()) +" Targets Found!");
						totalContours[0] = targPipeline.findContoursOutput().size();
						targetContours[0] = targPipeline.filterContoursOutput().size();
						targetCenterX[0] = 9999;	
					}
					else if (targPipeline.filterContoursOutput().size() == 1) {
						//SmartDashboard.putString("Target Contour:", String.valueOf(targPipeline.filterContoursOutput().size()) +" Target Found!");

						Rect r = Imgproc.boundingRect(targPipeline.filterContoursOutput().get(0));				
							targetCenterX[0]= r.x + (r.width / 2);

											
						//SmartDashboard.putString("Target Center X: ", String.valueOf(targetCenterX[0]));
						totalContours[0] = targPipeline.findContoursOutput().size();
						targetContours[0] = targPipeline.filterContoursOutput().size();								
											

					};
				};	
				
			}
			
		});

		targetThread.start();



		//*******************************************************************************************
		driveThread = new VisionThread(driveCamera, new BallPipeline(), ballPipeline -> {

			synchronized (driveLock) {


				inst.getEntry("/vmx/BallContoursFound").setNumber(ballPipeline.filterContoursOutput().size());
				inst.getEntry("/vmx/BallCenterX").setNumber(ballCenterX[0]);
				inst.getEntry("/vmx/BallCentery").setNumber(ballCenterY[0]);

				driveSink.grabFrame(processedDriveFrame);
							
				ballContours[0] = ballPipeline.filterContoursOutput().size();	
				ballLoopCount[0] = ballPipeline.ballLoop;

				// Overlay Ball contours
				for (int i = 0; i < ballPipeline.filterContoursOutput().size(); ++i) {
					Scalar ballColor = new Scalar(0,255,0);
					Imgproc.drawContours(processedDriveFrame, ballPipeline.filterContoursOutput(), i, ballColor, 1, Imgproc.LINE_8, hierarchy, 3);
					Rect c = Imgproc.boundingRect(ballPipeline.filterContoursOutput.get(i));
						Imgproc.rectangle(processedDriveFrame, c.tl(), c.br(), ballColor, 2);

				};

				
				if (ballPipeline.filterContoursOutput().isEmpty()){
				ballCenterX[0]=5555;
				ballCenterY[0]=3333;
				
				};

				
				if (!ballPipeline.filterContoursOutput().isEmpty()) {
					
					if (ballPipeline.filterContoursOutput().size() > 1) {
						ballCenterX[0] = 9999;
						ballCenterY[0] = 9999;	
					}

					else if (ballPipeline.filterContoursOutput().size() == 1) {
						Rect s = Imgproc.boundingRect(ballPipeline.filterContoursOutput().get(0));				
							ballCenterX[0] = s.x + (s.width / 2);
							ballCenterY[0] = s.y + (s.height / 2);
					};
				};
					
			}
		});

		driveThread.start();


		//*******************************************************************************************************************

		int count = 0;
		while (true) {
			synchronized (targetLock) {		


				// Acquire new video targFrame **
				String videoTimestampString = null;
				long video_timestamp = targetSink.grabFrame(targFrame);
				if (video_timestamp == 0) {
					targetSink.getError();
					try {
						Thread.sleep((1000/targFrames_per_sec)/2, 0);
					} catch (InterruptedException e) {
						break;
						}
					continue;
				} else {
					videoTimestampString = "Vid Timestamp: " + BigInteger.valueOf(video_timestamp).toString();
				}			


				// Overlay timestamps & orientation data onto video **
				Imgproc.putText (
					processedTargFrame,                          // Video targFrame
					videoTimestampString,       	   // Text to be added
					new Point(30, 30),              // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);


				String imuTimestampString = "IMU Timestamp: " + Integer.toString(vmx.getAHRS().GetLastSensorTimestamp());
					Imgproc.putText (
					processedTargFrame,                          // Video targFrame
					imuTimestampString,        	   // Text to be added
					new Point(30, 50),              // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);


				String yawString = "Yaw: " + Double.toString(vmx.getAHRS().GetYaw());
					Imgproc.putText (
					processedTargFrame,                          // Video targFrame
					yawString,          	   	   // Text to be added
					new Point(30, 70),              // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);
				String pitchString = "Pitch: " + Double.toString(vmx.getAHRS().GetPitch());
					Imgproc.putText (
					processedTargFrame,                          // Video targFrame
					pitchString,          	   // Text to be added
					new Point(30, 90),              // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);
				String rollString = "Roll:" + Double.toString(vmx.getAHRS().GetRoll());
					Imgproc.putText (
					processedTargFrame,                          // Video targFrame
					rollString,          	   // Text to be added
					new Point(30, 110),             // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);

				String contoursString = "Contours: " + Integer.toString(totalContours[0]);
					Imgproc.putText (
					processedTargFrame,                          // Video targFrame
					contoursString,          	   // Text to be added
					new Point(30, 130),             // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);

				String targetString = "Targets: " + Integer.toString(targetContours[0]);
					Imgproc.putText (
					processedTargFrame,                          // Video targFrame
					targetString,          	   // Text to be added
					new Point(30, 150),             // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);

				String ballString = "Balls: " + Integer.toString(ballContours[0]);
					Imgproc.putText (
					processedDriveFrame,                          // Video targFrame
					ballString,          	   // Text to be added
					new Point(30, 30),             // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);		
			
				String ballXString = "Ball Cntr X: " + Integer.toString(ballCenterX[0]);
					Imgproc.putText (
					processedDriveFrame,                          // Video targFrame
					ballXString,          	   // Text to be added
					new Point(30, 50),             // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);	

					String ballYString = "Ball Cntr Y: " + Integer.toString(ballCenterY[0]);
					Imgproc.putText (
					processedDriveFrame,                          // Video targFrame
					ballYString,          	   // Text to be added
					new Point(30, 70),             // point
					Imgproc.FONT_HERSHEY_SIMPLEX ,     // front face
					0.35,                            // front scale
					new Scalar(255, 255, 255),      // Scalar object for color (RGB)
					2                               // Thickness
					);	

					//****************************************************************************************

					if (!vmx.getIO().UART_GetBytesAvailable(uart_res_handle[0], bytes_available, vmxerr)) {
						System.out.printf("Error getting UART bytes available.\n");
					} else {
						if (bytes_available[0] == 0) {
							System.out.printf("No UART data available.\n");
						} else {
							System.out.printf("Got %d bytes of UART data:  ", bytes_available[0]);
							short[] bytes_actually_read = {0};
							if (!vmx.getIO().UART_Read(uart_res_handle[0], rcv_bytes.cast(), bytes_available[0], bytes_actually_read, vmxerr)) {
								System.out.printf("ERROR READING RECEIVED BYTES.\n");
							} else {
								System.out.printf(" [Bytes actually read:  %d]", bytes_actually_read[0]);
								byte[] byteArray = new byte[bytes_actually_read[0]];
								for (int j = 0; j < bytes_actually_read[0]; j++) {
									byteArray[j] = (byte)rcv_bytes.getitem(j);
								}
								System.out.println(Arrays.toString(byteArray));
		
								if(byteArray[0] == 0x59 && byteArray[1] == 0x59) {
									int checksum = 0;
									for(int j = 0; j < 8; j++) {
										checksum += byteArray[j];
									}
									if(byteArray[8] == checksum % 256) {
										distance = byteArray[2] + byteArray[3];
										strength = byteArray[4] + byteArray[5];
									temperature = (byteArray[6] + byteArray[7])/8;
		
									System.out.printf("Distance %d Strenght %d temperature %d\n", distance, strength, temperature);
									}
		
								}
							}
						}					
					}
				
				// Update Network Tables with timestamps & orientation data
				inst.getEntry("/vmx/videoOSTimestamp").setNumber(BigInteger.valueOf(video_timestamp));
				inst.getEntry("/vmx/navxSensorTimestamp").setNumber(vmx.getAHRS().GetLastSensorTimestamp());
				inst.getEntry("/vmx/navxYaw").setNumber(vmx.getAHRS().GetYaw());
				inst.getEntry("/vmx/navxPitch").setNumber(vmx.getAHRS().GetPitch());
				inst.getEntry("/vmx/navxRoll").setNumber(vmx.getAHRS().GetRoll());
				inst.getEntry("/vmx/VMX/VMXLoopCount").setNumber(count);
				inst.getEntry("/vmx/ToFSensor/Distance").setNumber(distance);
				inst.getEntry("/vmx/ToFSensor/Strength").setNumber(strength);
				inst.getEntry("/vmx/ToFSensor/Temp").setNumber(temperature);


			
				System.out.println("Loop: "+ count + "  BLoop: "+ ballLoopCount[0] + "  Balls: " + ballContours[0] + " Ball X: " + ballCenterX[0] + " Ball Y: " + ballCenterY[0] + " Targets: " + targetContours[0] + " Target Cntr(x): " + targetCenterX[0] );
				
				List<Mat> src = Arrays.asList(processedDriveFrame, processedTargFrame);
				Core.hconcat(src, combinedFrame);

				//Write Frame to video 
				if (targVideoWriter != null) {
					targVideoWriter.write(processedTargFrame);
				}
				targetSource.putFrame(processedTargFrame);

				//Write Frame to video 
				if (driveVideoWriter != null) {
					driveVideoWriter.write(processedDriveFrame);
				}
				driveSource.putFrame(processedDriveFrame);

				//Write Frame to video 
				if (combinedVideoWriter != null) {
					combinedVideoWriter.write(combinedFrame);
				}
				combinedSource.putFrame(combinedFrame);

			}	
			





			//count++;
		}

		vmx.delete(); // Immediately dispose of all resources used by vmx object.
		targetVideoServer.close();
		driveVideoServer.close();
		combinedVideoServer.close();
		targetSink.close();
		targetSource.close();
		driveSink.close();
		driveSource.close();

		
		

	}
}