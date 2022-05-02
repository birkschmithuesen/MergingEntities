from TDStoreTools import StorageManager
import TDFunctions as TDF

class Utils:
	"""
	Utils for SystemFailed round_timer component
	"""
	def __init__(self, ownerComp):
		# The component to which this extension is attached
		self.ownerComp = ownerComp
		self.Intro = ownerComp.op('./intro')
		self.Round = ownerComp.op('./round')
		self.Outro = ownerComp.op('./outro')

	def Pause(self):
		op.Control.par.Timestop = 1

	def Play(self):
		op.Control.par.Timestop = 0

	def ReInit(self):
		self.Round.par.initialize.pulse()
		self.Outro.par.initialize.pulse()
		self.Intro.par.initialize.pulse()
		return
		
	def GoIntro(self):
		self.ReInit()
		self.Intro.par.start.pulse()

	def EndIntro(self):
		self.ReInit()
		self.Intro.par.gotodone.pulse()

	def GoRound(self):
		self.ReInit()
		self.Round.par.start.pulse()

	def EndRound(self):
		self.ReInit()
		self.Round.par.gotodone.pulse()

	def GoOutro(self):
		self.ReInit()
		self.Outro.par.start.pulse()

	def EndOutro(self):
		self.ReInit()
		self.Outro.par.gotodone.pulse()
