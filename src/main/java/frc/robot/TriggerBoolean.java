package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TriggerBoolean extends SuperButton {
	private final Joystick joystick;
	private final int channel;

	public TriggerBoolean(Joystick joystick, int channel) {
		super(joystick, channel);
		this.joystick = joystick;
		this.channel = channel;
	}

	@Override
	public boolean get() {
		return joystick.getRawAxis(channel) > 0.90;
	}
}