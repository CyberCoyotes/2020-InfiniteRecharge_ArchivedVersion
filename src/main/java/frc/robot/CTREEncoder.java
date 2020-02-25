package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class CTREEncoder {

    WPI_TalonFX talon;
    boolean invert;

    public CTREEncoder(WPI_TalonFX _talon, boolean invert) {
        talon = _talon;
        talon.getSensorCollection();
        this.invert = invert;
    }

    public int get() {
        return invert ? -talon.getSelectedSensorPosition() : talon.getSelectedSensorPosition();
    }

    public int getVelocity() {
        return invert ? -talon.getSelectedSensorVelocity() : talon.getSelectedSensorVelocity();
    }

    public void setPosition(int pos) {
        talon.setSelectedSensorPosition(pos);
    }

    public void reset() {
        setPosition(0);
    }
}