package org.usfirst.frc1073.PIDTest;

import java.security.GeneralSecurityException;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author techn
 *
 *         First PID Test.
 */
public class PIDThread implements Runnable {
	SpeedController backLeft;
	SpeedController backRight;
	SpeedController frontLeft;
	SpeedController frontRight;
	Encoder backLeftEncoder;
	Encoder backRightEncoder;
	Encoder frontLeftEncoder;
	Encoder frontRightEncoder;

	final double scaler = 1 / .707;
	final long dt = (5);
	static double kP;
	static double kI;
	static double kD;

	double previousErrorFL;
	double integralFL;
	double setpointFL;
	double encoderSpeedFL;
	double outputFL;

	double previousErrorFR;
	double integralFR;
	double setpointFR;
	double encoderSpeedFR;
	double outputFR;

	double previousErrorBL;
	double integralBL;
	double setpointBL;
	double encoderSpeedBL;
	double outputBL;

	double previousErrorBR;
	double integralBR;
	double setpointBR;
	double encoderSpeedBR;
	double outputBR;

	static boolean isPID;
	static boolean isPrintMoreData;
	static boolean isUpdatePID;
	static boolean isCapJoystick;
	static boolean isTopSpeedAdjusted;
	
	static float topSpeed;
	static double joystickCap;

	public PIDThread(double kP, double kI, double kD) {
		PIDThread.kP = kP;
		PIDThread.kI = kI;
		PIDThread.kD = kD;

		backLeft = new Jaguar(1);
		backRight = new Jaguar(2);
		frontLeft = new Jaguar(0);
		frontRight = new Jaguar(3);
		backLeftEncoder = new Encoder(7, 6, false, EncodingType.k4X);
		backRightEncoder = new Encoder(3, 2, false, EncodingType.k4X);
		frontLeftEncoder = new Encoder(5, 4, false, EncodingType.k4X);
		frontRightEncoder = new Encoder(9, 8, false, EncodingType.k4X);

		backLeftEncoder.setDistancePerPulse(0.017453);
		backRightEncoder.setDistancePerPulse(0.017453);
		frontLeftEncoder.setDistancePerPulse(0.017453);
		frontRightEncoder.setDistancePerPulse(0.017453);

		isPID = true;
		isPrintMoreData = false;
		isUpdatePID = false;
		isCapJoystick = false;
		isTopSpeedAdjusted = false;
		
		topSpeed = 9;
		joystickCap = 1;
	}

	@Override
	public void run() {
		try {
			while (true) {
				/*
				 * double y = Robot.oi.getdriver().getY(); if(Math.abs(y) <
				 * 0.04){ y = 0; }
				 * 
				 * setpointFL = y; setpointFR = -y; setpointBR = -y; setpointBL
				 * = y;
				 */

				double twist = Robot.oi.getdriver().getRawAxis(5);
				double mag = Robot.oi.getdriver().getMagnitude();
				double joyAngle = Robot.oi.getdriver().getDirectionDegrees();

				if (Math.abs(mag) < 0.05) {
					mag = 0;
				}
				if (Math.abs(twist) < 0.05) {
					twist = 0;
				}
				
				mag = mag * Math.abs(joystickCap);
				
				SmartDashboard.putNumber("Joystick Angle: ", joyAngle);
				SmartDashboard.putNumber("Joystick Magnitude: ", mag);
				SmartDashboard.putNumber("Joystick Twist: ", twist);
				
				SmartDashboard.putBoolean("Is PID Enabled? :", isPID);

				joyAngle = ((joyAngle + 45) * 3.14159) / 180;
				double sinAngle = Math.sin(joyAngle);
				double cosAngle = Math.cos(joyAngle);

				setpointFL = (sinAngle * mag + twist) * scaler;
				setpointFR = (cosAngle * mag - twist) * scaler;
				setpointBL = (cosAngle * mag + twist) * scaler;
				setpointBR = (sinAngle * mag - twist) * scaler;
				double[] setpoints = normalize(setpointFL, setpointFR,
						setpointBL, setpointBR);
				setpointFL = -1 * setpoints[0];
				setpointFR = setpoints[1];
				setpointBL = -1 * setpoints[2];
				setpointBR = setpoints[3];

				if (!isPID) {
					frontLeft.set(setpointFL);
					frontRight.set(setpointFR);
					backLeft.set(setpointBL);
					backRight.set(setpointBR);
					
					SmartDashboard.putNumber("setpoint for Front Left: ",
							setpointFL);
					SmartDashboard.putNumber("setpoint for Front Right: ",
							setpointFR);
					SmartDashboard.putNumber("setpoint for Back Left: ",
							setpointBL);
					SmartDashboard.putNumber("setpoint for Back Right: ",
							setpointBR);
					
				} else {
					/*
					 * setpointFL = -1 * (twist + mag * (Math.cos(joyAngle) +
					 * Math.sin(joyAngle))); setpointFR = (twist + mag *
					 * (Math.cos(joyAngle) - Math.sin(joyAngle))); setpointBL =
					 * -1 * (twist + mag * (Math.cos(joyAngle) -
					 * Math.sin(joyAngle))); setpointBR = (twist + mag *
					 * (Math.cos(joyAngle) + Math.sin(joyAngle)));
					 */

					SmartDashboard.putNumber("setpoint for Front Left: ",
							setpointFL);
					SmartDashboard.putNumber("setpoint for Front Right: ",
							setpointFR);
					SmartDashboard.putNumber("setpoint for Back Left: ",
							setpointBL);
					SmartDashboard.putNumber("setpoint for Back Right: ",
							setpointBR);

					encoderSpeedBL = (backLeftEncoder.getRate() / 12) / topSpeed;
					encoderSpeedBR = (backRightEncoder.getRate() / 12) / topSpeed;
					encoderSpeedFL = (frontLeftEncoder.getRate() / 12) / topSpeed;
					encoderSpeedFR = (frontRightEncoder.getRate() / 12) / topSpeed;

					SmartDashboard.putNumber("Encoder Speed Back Left: ",
							encoderSpeedBL);
					SmartDashboard.putNumber("Encoder Speed Back Right: ",
							encoderSpeedBR);
					SmartDashboard.putNumber("Encoder Speed Front Left: ",
							encoderSpeedFL);
					SmartDashboard.putNumber("Encoder Speed Front Right: ",
							encoderSpeedFR);

					double errorFL = setpointFL - encoderSpeedFL;
					integralFL = integralFL + (errorFL * dt);
					double derivativeFL = (errorFL - previousErrorFL) / dt;
					outputFL = (kP * errorFL) + (kI * integralFL)
							+ (kD * derivativeFL);
					previousErrorFL = errorFL;

					double errorFR = setpointFR - encoderSpeedFR;
					integralFR = integralFR + (errorFR * dt);
					double derivativeFR = (errorFR - previousErrorFR) / dt;
					outputFR = (kP * errorFR) + (kI * integralFR)
							+ (kD * derivativeFR);
					previousErrorFR = errorFR;

					double errorBL = setpointBL - encoderSpeedBL;
					integralBL = integralBL + (errorBL * dt);
					double derivativeBL = (errorBL - previousErrorBL) / dt;
					outputBL = (kP * errorBL) + (kI * integralBL)
							+ (kD * derivativeBL);
					previousErrorBL = errorBL;

					double errorBR = setpointBR - encoderSpeedBR;
					integralBR = integralBR + (errorBR * dt);
					double derivativeBR = (errorBR - previousErrorBR) / dt;
					outputBR = (kP * errorBR) + (kI * integralBR)
							+ (kD * derivativeBR);
					previousErrorBR = errorBR;

					SmartDashboard.putNumber("Output for Front Left: ",
							outputFL);
					SmartDashboard.putNumber("Output for Front Right: ",
							outputFR);
					SmartDashboard
							.putNumber("Output for Back Left: ", outputBL);
					SmartDashboard.putNumber("Output for Back Right: ",
							outputBR);

					frontLeft.set(outputFL);
					frontRight.set(outputFR);
					backLeft.set(outputBL);
					backRight.set(outputBR);
				}
				if(isPrintMoreData){
					SmartDashboard.putNumber("Front Left Drive Speed (Actual): ", encoderSpeedFL * topSpeed);
					SmartDashboard.putNumber("Front Right Drive Speed (Actual):", encoderSpeedFR * topSpeed);
					SmartDashboard.putNumber("Back Left Drive Speed (Actual): ", encoderSpeedBL * topSpeed);
					SmartDashboard.putNumber("Back Right Drive Speed (Actual):", encoderSpeedBR * topSpeed);
					
				}
				if(isUpdatePID){
					kP = SmartDashboard.getNumber("kP: ", kP);
					kI = SmartDashboard.getNumber("kI: ", kI);
					kD = SmartDashboard.getNumber("kD: ", kD);
				}
				if(isTopSpeedAdjusted){
					topSpeed = (float) SmartDashboard.getNumber("Top Speed: ", topSpeed);
				}
				if(isCapJoystick){
					joystickCap = SmartDashboard.getNumber("Joystick Cap: ", joystickCap);
				}
				
				Thread.sleep(dt);
			}
		} catch (InterruptedException iex) {
		}
	}

	private double[] normalize(double FL, double FR, double BL, double BR) {
		double[] setpoints = { FL, FR, BL, BR };
		double max = Math.abs(setpoints[0]);
		for (int i = 1; i < 4; i++) {
			double speed = Math.abs(setpoints[i]);
			if (speed > max) {
				max = speed;
			}
		}
		if (max > 1.0) {
			for (int i = 0; i < 4; i++) {
				setpoints[i] = (setpoints[i] / max);
			}
		}
		return setpoints;
	}
	public static void switchDriveMode(){
		isPID = !isPID;
	}
	public static void printMoreData(){
		isPrintMoreData = !isPrintMoreData;
	}
	public static void updatePIDValues(){
		isUpdatePID = !isUpdatePID;
	}
	public static void setTopSpeed(){
		isTopSpeedAdjusted = !isTopSpeedAdjusted;
	}
	public static void capJoystick(){
		isCapJoystick = !isCapJoystick;
	}
	

}
