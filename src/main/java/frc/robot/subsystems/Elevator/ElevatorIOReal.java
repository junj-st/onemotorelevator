package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX elevatorMotor;
    private final TalonFXConfiguration motorConfig;

    private final StatusSignal<Current> elevatorCurrent;
    private final StatusSignal<Temperature> elevatorTemp;
    private final StatusSignal<AngularVelocity> elevatorAngularVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Angle> elevatorPos;
    private MotionMagicVoltage motionMagicRequest;
    private VoltageOut voltageOutRequest;
    private static final double GEAR_RATIO = 100.0;
    private static final double WHEEL_CIRCUMFERENCE_METERS = 0.1; // Adjust based on your pulley/sprocket
    private static final double STATOR_CURRENT_LIMIT = 80.0;
    private double setpointVolts;
    private double setpointMeters;
    
    public Elevator() {
        elevatorMotor = new TalonFX(15, "rio"); // Adjust CAN ID as needed
        motorConfig = new TalonFXConfiguration();
        
        elevatorCurrent = elevatorMotor.getStatorCurrent();
        elevatorTemp = elevatorMotor.getDeviceTemp();
        elevatorAngularVelocity = elevatorMotor.getRotorVelocity();
        motorVoltage = elevatorMotor.getMotorVoltage();
        elevatorPos = elevatorMotor.getRotorPosition();

        motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        setpointMeters = 0;
        setpointVolts = 0;

        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 300;
        motorConfig.MotionMagic.MotionMagicAcceleration = 450;
        motorConfig.MotionMagic.MotionMagicJerk = 10000;
        motorConfig.Slot0.kP = 2;
        motorConfig.Slot0.kD = 0;
        motorConfig.Slot0.kS = 0.088822;
        motorConfig.Slot0.kV = 0;
        motorConfig.Slot0.kG = 0.464;
        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotor.getConfigurator().apply(motorConfig);
       
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            elevatorCurrent,
            elevatorTemp,
            elevatorAngularVelocity,
            motorVoltage,
            elevatorPos
        );


        elevatorMotor.optimizeBusUtilization();
    }

    private double metersToRotations(double meters, double wheelCircumference, double gearRatio) {
        return (meters / wheelCircumference) * gearRatio;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorCurrent,
            elevatorTemp,
            elevatorAngularVelocity,
            motorVoltage,
            elevatorPos 
        );
    }

    public void requestVoltage(double volts) {
        this.setpointVolts = volts;
        elevatorMotor.setControl(voltageOutRequest.withOutput(volts));
    }

    public void requestMotionMagic(double meters) {
        this.setpointMeters = meters;
        double rotations = metersToRotations(meters, WHEEL_CIRCUMFERENCE_METERS, GEAR_RATIO);
        elevatorMotor.setControl(motionMagicRequest.withPosition(rotations));
    }

    public void zeroSensorMeters(double meters) {
        double rotations = metersToRotations(meters, WHEEL_CIRCUMFERENCE_METERS, GEAR_RATIO);
        elevatorMotor.setPosition(rotations);
    }

    public void resetMotionMagicConfigs(boolean down) {
        MotionMagicConfigs configs = new MotionMagicConfigs();
        if (down) {
            configs.MotionMagicCruiseVelocity = 15;
            configs.MotionMagicAcceleration = 30;
            configs.MotionMagicJerk = 5000;
        } else {
            configs.MotionMagicCruiseVelocity = 300;
            configs.MotionMagicAcceleration = 450;
            configs.MotionMagicJerk = 10000;
        }
        elevatorMotor.getConfigurator().apply(configs);
    }

    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(
            elevatorCurrent
            elevatorTemp,
            elevatorAngularVelocity,
            motorVoltage,
            elevatorPos);

            inputs.setpointMeters = this.setpointMeters;
            inputs.elevatorVelMPS = elevatorAngularVelocity.getValue().getValue() * (WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO) / 60.0;
            inputs.elevatorHeightMeters = elevatorPos.getValue().getValue() * (WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO);
            inputs.currentAmps = new double[] {elevatorCurrent.getValue().getValue()};
            inputs.tempF = new double[] { (elevatorTemp.getValue().getValue() * 9.0/5.0) + 32.0 };
            

    }


  
}