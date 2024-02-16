package org.firstinspires.ftc.teamcode.hardware.common;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BreakBeam {

	private DigitalChannel beam;

	public BreakBeam() {}

	public void setName(String name, HardwareMap hwMap) {
		beam = hwMap.get(DigitalChannel.class, name);
	}

	public boolean isBroken() {
		return !beam.getState();
	}

}
