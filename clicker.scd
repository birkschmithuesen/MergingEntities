s.meter

ServerOptions.devices; // all devices

Server.local.options.numOutputBusChannels = 4;
Server.default.options.outDevice_("Fireface UC Mac (23557042)");
s.reboot;

OSCFunc.trace(true); // Turn posting on
OSCFunc.trace(false); // Turn posting off



(
SynthDef.new(\click, {
	arg pulseSpeed = 0.5, decay = 0.01, bufnum = 0, delay = 0.2, feedback = 0.5, amp = 0.2, delayMin = 0.1, delayMax = 0.1, out = 0;
	var sig;
	sig = Ringz.ar(Impulse.ar(pulseSpeed), [110, 215, 330, 420], decaytime:decay);
	sig = Splay.ar(sig) * amp;1
	Out.ar(out,sig);
}
).add;



x = Synth.new(\click, [\out, 0]);
y = Synth.new(\click, [\out, 2]);

///////////////////////////////////////////
// HUMAN
///////////////////////////////////////////

OSCdef.new(\Positionx0, {
	arg msg;
	x.set(
		\pulseSpeed, msg[1].linlin(0,7,0,50)
	);
}, '/Positionx0'
);



OSCdef.new(\Positiony0, {
	arg msg;
	x.set(
		\decay, msg[1].linlin(0,3.6,0,0.5)
	);
}, '/Positiony0'
);

///////////////////////////////////////////
// MACHINE
///////////////////////////////////////////

OSCdef.new(\Positionx1, {
	arg msg;
	y.set(
		\pulseSpeed, msg[1].linlin(0,7,0,50)
	);
}, '/Positionx1'
);



OSCdef.new(\Positiony1, {
	arg msg;
	y.set(
		\decay, msg[1].linlin(0,3.6,0,0.5)
	);
}, '/Positiony1'
);

)