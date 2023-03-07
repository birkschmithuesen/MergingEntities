OscReceiver {
    connect {
		arg synth, name;
		OSCFunc(
			{ arg msg;
				synth.par1(msg[1]);
			},
			(name)
		);
		OSCFunc(
			{ arg msg;
				synth.par2(msg[2]);
			},
			(name)
		);
		OSCFunc(
			{ arg msg;
				synth.par3(msg[3]);
			},
			(name)
		);
		OSCFunc(
			{ arg msg;
				synth.par4(msg[4]);
			},
			(name)
		);
		OSCFunc(
			{ arg msg;
				synth.par5(msg[5]);
			},
			(name)
		)
    }
}
