package org.firstinspires.ftc.teamcode.hardware.common;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitch {

	private DigitalChannel limitSwitch;

	public LimitSwitch() {};

	public void setName(String name, HardwareMap hwMap) {
		limitSwitch = hwMap.get(DigitalChannel.class, name);
	}

	public boolean isPressed() {
		return limitSwitch.getState();
	}

}
