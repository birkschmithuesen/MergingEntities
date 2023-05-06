# me - this DAT
# par - the Par object that has changed
# val - the current value
# prev - the previous value
# 
# Make sure the corresponding toggle is enabled in the Parameter Execute DAT.

def onValueChange(par, prev):
	# use par.eval() to get current value
	return

def onPulse(par):
	if par.name == 'Savescene':
		parent.Scene.Save()
	elif par.name == 'Reloadscene':
		parent.Scene.Reload()
	elif par.name == 'Fadein':
		parent.Scene.Activate(jump=False)
	elif par.name == 'Fadeout':
		parent.Scene.Deactivate(jump=False)
	elif par.name == 'Jumpin':
		parent.Scene.Activate(jump=True)
	elif par.name == 'Jumpout':
		parent.Scene.Deactivate(jump=True)
	elif par.name == 'Playme':
		parent.Scene.PlayMe(jump=False)
	elif par.name == 'Jumpme':
		parent.Scene.PlayMe(jump=True)
	return

def onExpressionChange(par, val, prev):
	return

def onExportChange(par, val, prev):
	return

def onEnableChange(par, val, prev):
	return

def onModeChange(par, val, prev):
	return
	