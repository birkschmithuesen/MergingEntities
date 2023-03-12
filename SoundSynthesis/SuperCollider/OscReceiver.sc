OscReceiver {
	var synth, address;

	*new { arg synth, address;
		^super.newCopyArgs(synth, address).init;
	}

    init {
		("Connect "++synth.synth++" with OSC address "++address).postln;
		OSCFunc(
			{ arg msg;
				synth.par1(msg[1]);
			},
			(address)
		);
		OSCFunc(
			{ arg msg;
				synth.par2(msg[2]);
			},
			(address)
		);
		OSCFunc(
			{ arg msg;
				synth.par3(msg[3]);
			},
			(address)
		);
		OSCFunc(
			{ arg msg;
				synth.par4(msg[4]);
			},
			(address)
		);
		OSCFunc(
			{ arg msg;
				synth.par5(msg[5]);
			},
			(address)
		);
		OSCFunc(
			{ arg msg;
				synth.par6(msg[6]);
			},
			(address)
		)
    }
}
