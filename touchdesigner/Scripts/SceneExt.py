"""
Extension classes enhance TouchDesigner components with python. An
extension is accessed via ext.ExtensionClassName from any operator
within the extended component. If the extension is promoted via its
Promote Extension parameter, all its attributes with capitalized names
can be accessed externally, e.g. op('yourComp').PromotedFunction().

Help: search "Extensions" in wiki
"""

from TDStoreTools import StorageManager
TDF = op.TDModules.mod.TDFunctions

class SceneExt:
	"""
	SceneExt description
	"""
	def __init__(self, ownerComp):
		# The component to which this extension is attached
		self.ownerComp = ownerComp

		TDF.createProperty(self,'On',value=0,dependable=True,readOnly=False)
		TDF.createProperty(self,'Active',value=0,dependable=True,readOnly=False)

	def Save(self):
		parent.Scene.save('Scenes/' + parent.Scene.name + '.tox')

	def Reload(self):
		parent.Scene.par.reinitpulse.pulse()

	def TurnOn(self):
		self.On = 1
	
	def TurnOff(self):
		self.On = 0

	def Activate(self):
		self.Active = 1
	
	def Deactivate(self):
		self.Active = 0