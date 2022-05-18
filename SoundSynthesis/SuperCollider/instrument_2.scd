
ServerOptions.devices; // all devices

Server.local.options.numOutputBusChannels = 2;
Server.default.options.outDevice_("Fireface UC Mac (23557042)");
Server.default.options.outDevice_("External Headphones");
s.reboot;

(
SynthDef(\tube, {
	arg exciter = 0.5, res_array = 0.5, res_delay = 0.5, out = 0, amp = 0.1;
	var sig;
	sig = NTube.ar(
		PinkNoise.ar * exciter,
		`[0.97,1.0,1.0,1.0,1.0,0.97],
		`[0.5,res_array,0.2,-0.4],
		`([0.01,0.02,0.01,0.005,0.05]*res_delay));
	sig = sig * amp;
	Out.ar(out, sig!2);
}).add
)


x = Synth.new(\tube);




(
OSCdef.new(\exciter, {
	arg msg;
	x.set(\exciter, msg[1]);
}, '/exciter'
);
)

(
OSCdef.new(\res_array, {
	arg msg;
	x.set(\res_array, msg[1].linlin(0, 1, -1, 1));
}, '/res_array'
);
)

(
OSCdef.new(\res_delay, {
	arg msg;
	x.set(\res_delay, msg[1].linexp(0, 1, 0.001, 1));
}, '/res_delay'
);
)




