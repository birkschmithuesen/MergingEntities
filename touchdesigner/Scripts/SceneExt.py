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

		TDF.createProperty(self,'AmountModels',value=iop.ListModels.numRows-1,dependable=True,readOnly=False)		
		TDF.createProperty(self,'Models',value=list(),dependable=True,readOnly=False)
		self.FillModelList()

		# Scene ML States
		TDF.createProperty(self,'EngineStates',value=list(),dependable="deep",readOnly=False)
		TDF.createProperty(self,'ModelStates',value=list(),dependable="deep",readOnly=False)
		TDF.createProperty(self,'CookingStates',value=list(),dependable="deep",readOnly=False)
		for i in range(self.AmountModels):
			self.EngineStates.append(0)
			self.ModelStates.append(0)
			self.CookingStates.append(0)

		# start the Scene "empty"
		self.UnloadAllModelEngines()	

		# prediction toggle of the engines
		self.Predict = tdu.Dependency(0)

	def FillModelList(self):
		self.AmountModels = iop.ListModels.numRows-1
		self.Models = list() #empty list first
		for i in range(self.AmountModels):
			self.Models.append(op('model_prediction_' + str(i+1)))

	def UnloadEngine(self, index):
		model = self.Models[index]
		model.op('predict_engine').par.unload.pulse()
		self.EngineStates.setItem(index,0)
		self.ModelStates.setItem(index,0)
		self.CookingStates.setItem(index,0)

	def UnloadAllModelEngines(self):
		self.FillModelList()
		for i in range(self.AmountModels):
			self.UnloadEngine(i)
	
	def LoadModelEngine(self,index):
		model = self.Models[index]
		self.EngineStates.setItem(index,0)
		self.CookingStates.setItem(index,0)
		model.op('predict_engine').par.initialize.pulse()
		model.op('predict_engine').par.play = 1
		
	def LoadAllModelEngines(self):
		self.FillModelList()
		for i in range(self.AmountModels):
			self.LoadModelEngine(i)

	def LoadModel(self, index):
		self.ModelStates.setItem(index,0)
		self.Models[index].par.Loadmodel.pulse()

	def LoadAllModels(self):
		self.FillModelList()
		for i in range(self.AmountModels):
			self.LoadModel(i)

	def SetCooking(self, index, cooking):
		model = self.Models[index]
		model.op('predict_engine').par.play = cooking
		self.CookingStates.setItem(index, cooking)
	
	def SetAllCooking(self, cooking):
		self.FillModelList()
		for i in range(self.AmountModels):
			self.SetCooking(i, cooking)

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