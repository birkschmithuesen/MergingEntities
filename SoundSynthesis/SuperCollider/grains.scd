
(
// GRANULATORS

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;


// create groups
~grainGroup = Group.new;

// define number of performers bodies
~numGrains = 3;

// load samples
~b1 = {0}!~numBodies;
~b1[0] = Buffer.read(s, PathName(thisProcess.nowExecutingPath).parentPath ++ "Samples/sampler/grainer/1.aif");
~b1[1] = Buffer.read(s, PathName(thisProcess.nowExecutingPath).parentPath ++ "Samples/sampler/grainer/2.aif");
~b1[2] = Buffer.read(s, PathName(thisProcess.nowExecutingPath).parentPath ++ "Samples/sampler/grainer/3.aif");

/*
~b1[0] = Buffer.read(s, PathName(thisProcess.nowExecutingPath).parentPath ++ "Samples/sampler/1/Shar_1.aif");
~b1[1] = Buffer.read(s, PathName(thisProcess.nowExecutingPath).parentPath ++ "Samples/sampler/2/Howool_1.aif");
~b1[2] = Buffer.read(s, PathName(thisProcess.nowExecutingPath).parentPath ++ "Samples/sampler/2/Hayato_1.aif");
*/

//~b1.play;

// create Synths
~grains = {arg i; Synth.head(~grainGroup, \grain, [\out, i*2, \sndbuf, ~b1[i]])}!~numGrains;
{arg i; ("Created "++~grains[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscGrain = {arg i; OscGrain.new(~grains[i], ('/grain/' ++ i))}!~numGrains;
~oscGrainGroup = OscStandartGroup.new(~grainGroup, ('/grain/group'));
)
