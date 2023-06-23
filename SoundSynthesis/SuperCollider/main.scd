Server.killAll

ServerOptions.devices; // all devices
Server.default.options.inDevice_("Built-in Microph");
Server.default.options.outDevice_("BlackHole 64ch");
Server.default.options.outDevice_("MacBook Pro Speakers");
s.numOutputBusChannels = numChannels: 64;

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)

(
s.reboot{
	Server.default.options.outDevice_("BlackHole 64ch");
	s.options.numOutputBusChannels = 64;
}
)

(
s.reboot{
	Server.default.options.outDevice_("External Headphones"); //MacBook Pro Speakers
	s.options.numOutputBusChannels = 2;
}
)


(
// set global tempo
~tempo = 60/120 * 4; //60/90 * 4 //60sec / 128 BPM * 4 bars

// load Synth Defs and Pattern Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;
(~path++"efxlib.scd").load;

// define number of performers bodies
~numBodies = 4;
)


(
// PARTS -
// one instrument per body part
// each performer same instrument

var scale;

// create groups
~moogGroup = Group.new;
~noiseGroup = Group.new;
//~sawGroup = Group.new;
~leadGroup = Group.new;

// create Synths
scale = FloatArray[48, 55, 60, 67];//some 5ths starting from c4
~moogs = {arg i; Synth.head(~moogGroup, \moog, [\out, i*2])}!~numBodies;
{arg i;
	~moogs[i].set(\freq, scale[i].midicps);
	("Created "++~moogs[i]++" with freq: "++(300+ (i*52))).postln;
}!~numBodies;

~noises = {arg i; Synth.head(~noiseGroup,\noise, [\out, i*2])}!~numBodies;
{arg i; ("Created "++~noises[i]).postln}!~numBodies;

/*
~saws = {arg i; Synth.head(~performerGroup[i],\saw)}!~numBodies;
{arg i; ("Created "++~saws[i]).postln}!~numBodies;
*/

~leads = {arg i; Synth.head(~leadGroup, \lead, [\out, i*2])}!~numBodies;
{arg i; ("Created "++~leads[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscMoog = {arg i; OscMoog.new(~moogs[i], ('/moog/' ++ i), 0.0, 1.3)}!~numBodies;
~oscMoogGroup = OscStandartGroup.new(~moogGroup, ('/moog/group'), 0.0, 2.0);

~oscNoise = {arg i; OscNoise.new(~noises[i], ('/noise/' ++ i), 400 + (1000 * i), 1000 + (2000*i))}!~numBodies;
~oscNoiseGroup = OscStandartGroup.new(~noiseGroup, ('/noise/group'));

//~oscSaw = {arg i; OscSaw.new(~saws[i], ('/saw/' ++ i))}!~numBodies;
//~oscSawGroup = OscDetuneGroup.new(~sawGroup, ('/saw/group'), 1.01, 1.5);

~oscLead = {arg i; OscLead.new(~leads[i], ('/lead/' ++ i), -12 + (i*5), -4 + (i*6), 0.04, 2.0)}!~numBodies;
~oscLeadGroup = OscStandartGroup.new(~leadGroup, ('/lead/group'));
)


(
// WHEEL -
// one body one instrument
// each performer has the same instrument

// create groups
~klingGroup = Group.new;


// create Synths
~klings = {arg i; Synth.head(~klingGroup, \kling, [\out, i*2])}!~numBodies;
{arg i; ("Created "++~klings[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscKling = {arg i; OscKling.new(~klings[i], ('/kling/' ++ i))}!~numBodies;
~oscKlingGroup = OscStandartGroup.new(~klingGroup, ('/kling/group'));
)



(
// PATTERNS


// create pattern proxies
~exc = {PatternProxy.new}!~numBodies;
~shape = {PatternProxy.new}!~numBodies;
~atk = {PatternProxy.new}!~numBodies;
~rel = {PatternProxy.new}!~numBodies;
~pan = {PatternProxy.new}!~numBodies;


// initialize pattern proxies
{arg i; ~exc[i].source = 1.0}!~numBodies;
{arg i; ~shape[i].source = 0.5}!~numBodies;
{arg i; ~atk[i].source = 0.1}!~numBodies;
{arg i; ~rel[i].source = 0.5}!~numBodies;
{arg i; ~pan[i].source = 0.5}!~numBodies;

// create groups
~patternGroup = Group.new;
~fxGrp = Group.after(~patternGroup);

// create busses
~efxBus = {Bus.audio(s, 2)}!~numBodies;

// load fx
~efx = {0}!~numBodies;
~efx[0] = Synth.new(\delay, [\in, ~efxBus[0], \out, 0+8], ~fxGrp);
~efx[1] = Synth.new(\delay, [\in, ~efxBus[1], \out, 2+8], ~fxGrp);
~efx[2] = Synth.new(\delay, [\in, ~efxBus[2], \out, 4+8], ~fxGrp);
~efx[3] = Synth.new(\delay, [\in, ~efxBus[3], \out, 6+8], ~fxGrp);

// load & play patterns
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"patternlib.scd").load;


// make the patterns controllable via osc
~oscPattern = {arg i; OscPattern.new([~exc[i], ~shape[i], ~atk[i], ~pan[i]], '/sequencer/' ++ i, ~efx[i])}!~numBodies;
//~oscPatternGroup = OscStandartGroup.new(~patternGroup, ('/pattern/group'), );
)



(
// GRANULATORS

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;


// create groups
~grainGroup = Group.new;

// define number of performers bodies
~numBodies = 1;

// load samples
~b1 = Buffer.read(s, PathName(thisProcess.nowExecutingPath).parentPath ++ "Samples/test_1.aif");

// create Synths
~grains = {arg i; Synth.head(~grainGroup, \grain, [\out, i*2, \sndbuf, ~b1])}!~numBodies;
{arg i; ("Created "++~grains[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscGrain = {arg i; OscGrain.new(~grains[i], ('/grain/' ++ i))}!~numBodies;
~oscGrainGroup = OscStandartGroup.new(~grainGroup, ('/grain/group'));
)