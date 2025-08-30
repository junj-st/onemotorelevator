package frc.robot.Example;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ExampleSubsystem extends SubsystemBase{
    TalonFX motor = new TalonFX(15, "rio");
    TalonFXConfiguration config = new TalonFXConfiguration();
    VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    public StatusSignal<Current> current = motor.getStatorCurrent();
    public StatusSignal<AngularVelocity> rps = motor.getRotorVelocity();
    public StatusSignal<Temperature> temp = motor.getDeviceTemp();

    public double setpointVolts = 0;

    public ExampleSubsystem(){
        config.CurrentLimits.StatorCurrentLimit = 70.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        

        motor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(50, current, rps, temp);
        }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current, rps, temp);
        SmartDashboard.putNumber("Motor Current", current.getValueAsDouble());
        SmartDashboard.putNumber("Motor Temperature", temp.getValueAsDouble());
        SmartDashboard.putNumber("Motor RPS", rps.getValueAsDouble());
    }

    public void requestVoltage(double voltage){
        motor.setControl(voltageRequest.withOutput(voltage));
        }

}
