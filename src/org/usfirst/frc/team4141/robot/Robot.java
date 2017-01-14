
package org.usfirst.frc.team4141.robot;

import org.team3309.lib.controllers.generic.FeedForwardWithPIDController;
import org.team3309.lib.controllers.statesandsignals.InputState;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public void robotInit() {
		SmartDashboard.putNumber("goal", 60);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
	}

	/**
	 * This function is called periodically during operator control
	 */
	public Counter counter = new Counter(5);
	public Counter elCounter = new Counter(4);
	//Talon motor1 = new Talon(0);
	//Talon motor2 = new Talon(1);
	Talon motor3 = new Talon(1);
	Talon elevator = new Talon(2);
	Talon motor4 = new Talon(3);
	XboxController driver = new XboxController(0);
	FeedForwardWithPIDController x = new FeedForwardWithPIDController(.053, 0, 0.765, 0, 0);
	FeedForwardWithPIDController elController = new FeedForwardWithPIDController(.06, 0, 0.001, 0, 0);
	double pastRPS = 0;
	double pastEl = 0;

	public void teleopPeriodic() {
		if (driver.getAButton()) {
			x.sendToSmartDash();
			double goal = SmartDashboard.getNumber("goal");
			double curRPS = (1 / counter.getPeriod()) / 10;
			if (Math.abs(curRPS - pastRPS) > 300 || curRPS == 0) {
				curRPS = pastRPS;
			}
			SmartDashboard.putNumber("RPS", curRPS);
			InputState input = new InputState();
			input.setError(goal - curRPS);
			x.setAimVel(goal);
			double output = x.getOutputSignal(input).getMotor();
			SmartDashboard.putNumber("out", output);
			// Elevator
			x.sendToSmartDash();
			double elGoal = SmartDashboard.getNumber("el goal");
			double curEl = (1 / elCounter.getPeriod()) / 10;
			if (Math.abs(curEl - pastEl) > 300 || curEl == 0) {
				curEl = pastEl;
			}
			SmartDashboard.putNumber("el RPS", curRPS);
			InputState elInput = new InputState();
			elInput.setError(elGoal - curEl);
			elController.setAimVel(elGoal);
			double elOutput = elController.getOutputSignal(elInput).getMotor();
			SmartDashboard.putNumber("el out", elOutput);
			pastEl = curEl;
			pastRPS = curRPS;
			elevator.set(elOutput);
			motor3.set(-output);
			motor4.set(output);
		} else {
			x.reset();
			motor3.set(0);
			elevator.set(0);
			motor4.set(0);
		}//jimy wuz here
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {

	}

}
