Server.killAll

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)


(
// create groups
~moogGroup = Group.new;
~noiseGroup = Group.new;

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;

// define number of performers bodies
~numBodies = 4;

// create Synths
~moogs = {Synth.head(~moogGroup,\moog)}!~numBodies;
{arg i;
	~moogs[i].set(\freq, 300 + (i*154));
	("Created "++~moogs[i]++" with freq: "++(300+ (i*52))).postln;
}!~numBodies;

~noises = {Synth.head(~noiseGroup,\noise)}!~numBodies;
{arg i; ("Created "++~noises[i]).postln}!~numBodies;

// make the Synth controllable via OSC
~oscMoog = {arg i; OscMoog.new(~moogs[i], ('/moog/' ++ i))}!~numBodies;
~oscMoogGroup = OscMoogGroup.new(~moogGroup, ('/moog/group'));

~oscNoise = {arg i; OscNoise.new(~noises[i], ('/noise/' ++ i))}!~numBodies;
~oscNoiseGroup = OscNoiseGroup.new(~noiseGroup, ('/noise/group'));
)