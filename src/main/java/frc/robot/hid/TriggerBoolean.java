package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;

public class TriggerBoolean extends SuperButton {
	private final Joystick joystick;
	private final int channel;
	private static final double TRIGGER_DEPRESSION_THRESHOLD = 0.75;

	public TriggerBoolean(Joystick joystick, int channel) {
		super(joystick, channel);
		this.joystick = joystick;
		this.channel = channel;
	}

	@Override
	public boolean get() {
		return getTriggerDepression() > TRIGGER_DEPRESSION_THRESHOLD;
	}

	public double getTriggerDepression() {
		return joystick.getRawAxis(channel);
	}
}