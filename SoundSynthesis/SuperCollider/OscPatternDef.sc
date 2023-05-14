OscPattern{
	var pattern_proxie, address, synth;

	*new {arg pattern_proxie, address, synth;
		^super.newCopyArgs(pattern_proxie, address, synth).init;
	}

	init {
		("Connect " ++ pattern_proxie ++ " with OSC address " ++ address).postln;

		OSCFunc({
			arg msg;
			//msg.postln;
			if (msg[1] != nil,
				{pattern_proxie[0].source = msg[1];},
			{("received wrong OSC message length on address: " ++ address)}
			);
			if (msg[2] != nil,
				{pattern_proxie[1].source = msg[2];},
			{("received wrong OSC message length on address: " ++ address)}
			);
			if (msg[3] != nil,
				{pattern_proxie[2].source = msg[3];},
			{("received wrong OSC message length on address: " ++ address)}
			);
			if (msg[4] != nil,
				{pattern_proxie[3].source = msg[4];},
			{("received wrong OSC message length on address: " ++ address)}
			);
			if (msg[5] != nil,
				{synth.set(\mix, msg[5])},
			{("received wrong OSC message length on address: " ++ address)}
			);
		},
		(address)
		);

	}
}