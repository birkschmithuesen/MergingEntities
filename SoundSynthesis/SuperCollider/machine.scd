
(
// PATTERNS
(~path++"grains.scd").load;

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
//~oscPattern = OscPattern.new([~exc[0], ~shape[0], ~atk[0], ~pan[0]], '/sequencer/' ++ 4, ~efx[0]);
//~oscPattern = OscPattern.new([~exc[1], ~shape[1], ~atk[1], ~pan[1]], '/sequencer/' ++ 1, ~efx[1]);
//~oscPattern = OscPattern.new([~exc[2], ~shape[2], ~atk[2], ~pan[2]], '/sequencer/' ++ 2, ~efx[2]);
//~oscPattern = OscPattern.new([~exc[3], ~shape[3], ~atk[3], ~pan[3]], '/sequencer/' ++ 0, ~efx[3]);
~oscPattern = {arg i; OscPattern.new([~exc[i], ~shape[i], ~atk[i], ~pan[i]], '/sequencer/' ++ i, ~efx[i])}!~numBodies;
//~oscPatternGroup = OscStandartGroup.new(~patternGroup, ('/pattern/group'), );
)