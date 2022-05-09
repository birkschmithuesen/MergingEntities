# me - this DAT
# 
# frame - the current frame
# state - True if the timeline is paused
# 
# Make sure the corresponding toggle is enabled in the Execute DAT.

def onStart():
	import sys
	spacer = "- " * 5

	for each in sys.path:
		print(each)

	print(spacer)

	python_ext = tdu.expandPath(ipar.ExtPython.Target)

	if python_ext not in sys.path:
		sys.path.append(python_ext)

	for each in sys.path:
		print(each)

	return

def onCreate():
	return

def onExit():
	return

def onFrameStart(frame):
	return

def onFrameEnd(frame):
	return

def onPlayStateChange(state):
	return

def onDeviceChange():
	return

def onProjectPreSave():
	return

def onProjectPostSave():
	return

	