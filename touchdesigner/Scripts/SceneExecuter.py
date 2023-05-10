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
	elif par.name == 'Reinit':
		parent.Scene.par.reinitextensions.pulse()
		parent.Scene.op('states/timer1').par.initialize.pulse()
	elif par.name == 'Unloadallengines':
		parent.Scene.UnloadAllModelEngines()
	elif par.name == 'Loadallengines':
		parent.Scene.LoadAllModelEngines()
	elif par.name == 'Loadallmodels':
		parent.Scene.LoadAllModels()
	elif par.name == 'Turnoffcooking':
		parent.Scene.SetAllCooking(0)
	elif par.name == 'Turnoncooking':
		parent.Scene.SetAllCooking(1)
	elif par.name == 'Activate':
		debug("Activating: " + parent.Scene.name + " with ID: " + str(parent.Scene.par.Id))
		parent.Scene.TurnOn()
		parent.Scene.op('states/timer1').par.initialize.pulse()
		parent.Scene.op('states/timer1').par.start.pulse()
	elif par.name == 'Deactivate':
		parent.Scene.Deactivate()
		parent.Scene.op('states/timer1').par.initialize.pulse()
		if parent.Scene.Predict:
			parent.Scene.Predict.val = False
		parent.Scene.SetAllCooking(0)
		parent.Scene.UnloadAllModelEngines()
		parent.Scene.TurnOff()
	return

def onExpressionChange(par, val, prev):
	return

def onExportChange(par, val, prev):
	return

def onEnableChange(par, val, prev):
	return

def onModeChange(par, val, prev):
	return
	