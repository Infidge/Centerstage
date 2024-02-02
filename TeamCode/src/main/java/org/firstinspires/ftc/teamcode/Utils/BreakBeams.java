package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BreakBeams
{
	DigitalChannel beam;

	public BreakBeams() {}

	public void setName(String name, HardwareMap hwMap)
	{
		beam = hwMap.get(DigitalChannel.class, name);
	}

	public boolean isBroken(){
		if (beam.getState())
			return false;
		else return true;
	}
}
