package org.firstinspires.ftc.teamcode.hardware.common;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IRBreakBeam {

	private DigitalChannel beam;

	public IRBreakBeam() {}

	public void setName(String name, HardwareMap hwMap) {
		beam = hwMap.get(DigitalChannel.class, name);
	}

	public boolean isBroken() {
		return !beam.getState();
	}

}
