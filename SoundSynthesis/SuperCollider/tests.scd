(
SynthDef(\test, {
	var sig;
	sig = SinOsc.ar();
	Out.ar(0, sig);
}).add
)

x = [2];
x[0]
x[0] = Synth(\test);

s.plotTree;

(
i = 0;
8.do{
	i=i+1;
	('Execute'++i).postln;
}
)



(
n=10;
n.do{arg i; i.postln};
)