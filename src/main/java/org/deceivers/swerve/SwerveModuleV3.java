package org.deceivers.swerve;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleV3 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final AbsoluteEncoder mAzimuthAbsoluteEncoder;
    private final RelativeEncoder mAzimuthIncrementalEncoder;
    private final RelativeEncoder mDriveEncoder;
    private final SparkMaxPIDController mDrivePID;
    private final SparkMaxPIDController mAzimuthPID;
    private final Translation2d mLocation;
    private final String mName; 
    private boolean isInverted;
    private double setpoint;

    //need to update the speed to m/s

    public SwerveModuleV3(CANSparkMax azimuthMotor, CANSparkMax driveMotor,
            Translation2d location, String name) {

        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;

        //Rest motors to factory defaults to ensure correct parameters
        mDriveMotor.restoreFactoryDefaults();
        mAzimuthMotor.restoreFactoryDefaults();

        //Get encoders
        mAzimuthAbsoluteEncoder = mAzimuthMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mAzimuthIncrementalEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();

        //Get PIDs
        mDrivePID = mDriveMotor.getPIDController();
        mAzimuthPID = mAzimuthMotor.getPIDController();

        //Configure drive motor controller parameters
        mDriveMotor.setInverted(true);
        mDriveMotor.setClosedLoopRampRate(0);
        mDriveMotor.setOpenLoopRampRate(.1);
        mDriveMotor.setIdleMode(IdleMode.kCoast);
        mDriveMotor.setSmartCurrentLimit(40, 60, 5700);
        
        //Configure Drive Encoder
        mDriveEncoder.setPositionConversionFactor((12.0/18.0)*(15.0/40.0)*0.2393893602);
        mDriveEncoder.setVelocityConversionFactor((0.239/(4.5*60)));
        mDriveEncoder.setPosition(0);

        //Configure azimuth motor controller parameters
        mAzimuthMotor.setInverted(true);
        mAzimuthMotor.setClosedLoopRampRate(0);
        mAzimuthMotor.setOpenLoopRampRate(0);
        mAzimuthMotor.setIdleMode(IdleMode.kBrake);
        mAzimuthMotor.setSmartCurrentLimit(20);
        
        //Configure drive absolute encoder
        mAzimuthAbsoluteEncoder.setPositionConversionFactor(360);
        mAzimuthAbsoluteEncoder.setInverted(true);
        mAzimuthAbsoluteEncoder.setAverageDepth(1);

        //Configure azimuth incremental encoder
        mAzimuthIncrementalEncoder.setPositionConversionFactor(360.0/35.94);

        //Configure drive PID
        mDrivePID.setFF(0.30);
        mDrivePID.setP(.1);
        
        //Configure azimuth PID
        mAzimuthPID.setFeedbackDevice(mAzimuthAbsoluteEncoder);
        mAzimuthPID.setP(.05);
        mAzimuthPID.setPositionPIDWrappingEnabled(false);
        mAzimuthPID.setPositionPIDWrappingMinInput(0);
        mAzimuthPID.setPositionPIDWrappingMaxInput(360);

        //Burn flahs in case of power cycle
        mDriveMotor.burnFlash();
        mAzimuthMotor.burnFlash();
    }

    // Sets the drive motor speed in open loop mode
    public void setSpeed(double speed) {
        mDriveMotor.set(speed);
    }

    // Sets the rotation speed of the azimuth motor in open loop mode
    public void setRotation(double rotation) {
        mAzimuthMotor.set(rotation);
    }

    // Gets the speed of the drive motor
    public double getSpeed() {
        return mDriveEncoder.getVelocity();
    }

    // Gets the rotation position of the azimuth module
    public double getRotation() {
        return mAzimuthIncrementalEncoder.getPosition();
    }

    //Gets the x/y location of the module relative to the center of the robot
    @Override
    public Translation2d getModuleLocation() {
        return mLocation;
    }

    //Get the state (speed/rotation) of the swerve module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getRotation()));
    }

    //Run when swerve drive is first initialized
    @Override
    public void init() {
        
    }

    //Get the position of swerve modules (distance and angle)
    public SwerveModulePosition getPosition(){
        SwerveModulePosition position = new SwerveModulePosition();
        position.angle = Rotation2d.fromDegrees(getRotation());
        position.distanceMeters = getDistance();

        return position;
    }

    //Get the distance of the drive encoder
    public double getDistance(){
        return mDriveEncoder.getPosition();
    }

    //Log swerve data
    @Override
    public void log() {
        SmartDashboard.putNumber(mName + " Azimuth Position", mAzimuthIncrementalEncoder.getPosition());
        SmartDashboard.putNumber(mName + "Absolute Position", mAzimuthAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Incremental Position", mAzimuthIncrementalEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Velocity", mDriveEncoder.getVelocity());
        SmartDashboard.putNumber(mName + "Drive Encoder Position", mDriveEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Rotation Setpoint", setpoint);
        SmartDashboard.putNumber(mName + "Percent Output", mDriveMotor.get());
    }

    //Set the speed and direction of the swerve module
    @Override
    public void set(SwerveModuleState drive) {
        double Angle = drive.angle.getDegrees();
        SmartDashboard.putNumber(mName + " Given Setpoint", Angle);
        double Velocity = drive.speedMetersPerSecond;

        double azimuthPosition = mAzimuthAbsoluteEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(Angle - azimuthPosition, 360);
        SmartDashboard.putNumber(mName + " Azimuth Error", azimuthError);

        isInverted = Math.abs(azimuthError) > 90;
        if (isInverted) {
            azimuthError -= Math.copySign(180, azimuthError);
            Velocity = -Velocity;
        }
        setpoint = azimuthError + azimuthPosition;
        SmartDashboard.putNumber(mName + " Azimuth CalcSetPoint", setpoint);
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mDriveMotor.set(Velocity);
    }

    @Override
    public void setClosedLoop(SwerveModuleState drive){
        if (Math.abs(mAzimuthIncrementalEncoder.getPosition() - mAzimuthAbsoluteEncoder.getPosition()) > 1){
            //setAzimuthZero();
        }

        double Angle = drive.angle.getDegrees();
        SmartDashboard.putNumber(mName + " Given Setpoint", Angle);
        double Velocity = drive.speedMetersPerSecond;

        double azimuthPosition = mAzimuthIncrementalEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(Angle - azimuthPosition, 360);
        SmartDashboard.putNumber(mName + " Azimuth Error", azimuthError);

        isInverted = Math.abs(azimuthError) > 90;
        if (isInverted) {
            azimuthError -= Math.copySign(180, azimuthError);
            Velocity = -Velocity;
        }
        setpoint = azimuthError + azimuthPosition;
        SmartDashboard.putNumber(mName + " Azimuth CalcSetPoint", setpoint);
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        SmartDashboard.putNumber(mName + " Wheel Setpoint", Velocity);
        mDrivePID.setReference(Velocity, ControlType.kVelocity);
    }

    //Stop all motors
    @Override
    public void stop(){
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);
    }

    //set angle of swerve drive
	@Override
	public void setAngle(double angle) {
        mAzimuthPID.setReference(angle, ControlType.kPosition);
		
	}
    
}
