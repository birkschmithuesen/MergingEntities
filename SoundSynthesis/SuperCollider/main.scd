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
// DOMINO -
// one instrument per body part
// each performer same instrument

var scale;

// load Synth Defs and Pattern Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;
(~path++"patternlib.scd").load;

// define number of performers bodies
~numBodies = 2;

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

~oscNoise = {arg i; OscNoise.new(~noises[i], ('/noise/' ++ i))}!~numBodies;
~oscNoiseGroup = OscStandartGroup.new(~noiseGroup, ('/noise/group'));

//~oscSaw = {arg i; OscSaw.new(~saws[i], ('/saw/' ++ i))}!~numBodies;
//~oscSawGroup = OscDetuneGroup.new(~sawGroup, ('/saw/group'), 1.01, 1.5);

~oscLead = {arg i; OscLead.new(~leads[i], ('/lead/' ++ i), 0, 13, 0.04, 2.0)}!~numBodies;
~oscLeadGroup = OscStandartGroup.new(~leadGroup, ('/lead/group'));
)


(
// MOVEMENT -
// one body one instrument
// each performer has the same instrument

// create groups
~klingGroup = Group.new;

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;

// define number of performers bodies
~numBodies = 2;

// create Synths
~klings = {arg i; Synth.head(~klingGroup, \kling, [\out, i*2])}!~numBodies;
{arg i; ("Created "++~klings[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscKling = {arg i; OscKling.new(~klings[i], ('/kling/' ++ i))}!~numBodies;
~oscKlingGroup = OscStandartGroup.new(~klingGroup, ('/kling/group'));
)



(
// PATTERNS

// load Synth Defs and Pattern Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;

// define number of performers bodies
~numBodies = 2;

~tempo = 60/120 * 4; //60/90 * 4 //60sec / 128 BPM * 4 bars


// create pattern proxies
~exc = {PatternProxy.new}!~numBodies;
~shape = {PatternProxy.new}!~numBodies;
~atk = {PatternProxy.new}!~numBodies;
~rel = {PatternProxy.new}!~numBodies;
// initialize pattern proxies
{arg i; ~exc[i].source = 0.3}!~numBodies;
{arg i; ~shape[i].source = 0.0}!~numBodies;
{arg i; ~atk[i].source = 0.5}!~numBodies;
{arg i; ~rel[i].source = 0.3}!~numBodies;

// load Pattern Defs
(~path++"patternlib.scd").load;

// create patterns
~pattern = [Pdef.new(\tripples), Pdef.new(\steady)];
// play the patterns
{arg i; ~pattern[i].play()}!~numBodies;

~pattern[0].reset;

~oscPattern = {arg i; OscPattern.new([~exc[i], ~shape[i], ~atk[i], ~rel[i]], ('/sequencer/' ++ i))}!~numBodies;
)