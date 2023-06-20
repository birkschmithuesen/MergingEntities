OscPattern{
	var pattern_proxie, address, efx;

	*new {arg pattern_proxie, address, efx;
		^super.newCopyArgs(pattern_proxie, address, efx).init;
	}

	init {
		("Connect " ++ pattern_proxie ++ " with OSC address " ++ address).postln;

		OSCFunc({
			arg msg;

			// exciter
			if (msg[1] != nil,
				{pattern_proxie[0].source = msg[1];},
			{("received wrong OSC message length on address: " ++ address)}
			);

			// shape
			if (msg[2] != nil,
				{pattern_proxie[1].source = msg[2];},
			{("received wrong OSC message length on address: " ++ address)}
			);

			// attack
			if (msg[3] != nil,
				{pattern_proxie[2].source = msg[3];},
			{("received wrong OSC message length on address: " ++ address)}
			);

			// efx mix
			if (msg[4] != nil,
				{efx.set(\mix, msg[4])},
			{("received wrong OSC message length on address: " ++ address)}
			);

			// pan
			if (msg[5] != nil,
				{pattern_proxie[3].source = msg[5];},
			{("received wrong OSC message length on address: " ++ address)}
			);

			// amp
			if (msg[6] != nil,
				{pattern_proxie[4].source = msg[6].linlin(0.0, 1.0, 0.0, 2.0);
				},
			{("received wrong OSC message length on address: " ++ address)}
			);

			// spare
			if (msg[7] != nil,
				{//spare
				},
			{("received wrong OSC message length on address: " ++ address)}
			);
		},
		(address)
		);

	}
}