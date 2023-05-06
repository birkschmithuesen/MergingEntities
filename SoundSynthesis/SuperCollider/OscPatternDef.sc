OscPattern{
	var pattern_proxie, address;

	*new {arg pattern_proxie, address;
		^super.newCopyArgs(pattern_proxie, address).init;
	}

	init {
		("Connect " ++ pattern_proxie ++ " with OSC address " ++ address).postln;

		OSCFunc({
			arg msg;
			if (msg[1] != nil,
				{pattern_proxie[0].source = msg[1];},
			{("received wrong OSC message length on address: " ++ address)}
			);
			if (msg[2] != nil,
				{pattern_proxie[1].source = msg[2];},
			{("received wrong OSC message length on address: " ++ address)}
			);
			if (msg[3] != nil,
				{pattern_proxie[2].source = msg[3].linlin(0.0, 1.0, 0.0, 0.15);},
			{("received wrong OSC message length on address: " ++ address)}
			);
			if (msg[4] != nil,
				{pattern_proxie[3].source = msg[4].linlin(0.0, 1.0, 0.0, 0.25);},
			{("received wrong OSC message length on address: " ++ address)}
			);
		},
		(address)
		);

	}
}