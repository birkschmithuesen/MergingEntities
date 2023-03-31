"""
Extension classes enhance TouchDesigner components with python. An
extension is accessed via ext.ExtensionClassName from any operator
within the extended component. If the extension is promoted via its
Promote Extension parameter, all its attributes with capitalized names
can be accessed externally, e.g. op('yourComp').PromotedFunction().

Help: search "Extensions" in wiki
"""

from io import StringIO

import tensorflow as tf
import keras
import tensorflow.experimental.numpy as tnp
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, LSTM, Dropout, Activation
from tensorflow.keras.optimizers import SGD
from tensorflow.python.client import device_lib

import mdn

import gc

import numpy as np
import cupy as cp

import json

from TDStoreTools import StorageManager
import TDFunctions as TDF

class PredictEngineExt:
	"""
	PredictEngineExt description
	"""
	def __init__(self, ownerComp):
		# The component to which this extension is attached
		op('debugTable').appendRow([absTime.frame, f'Ext Init Start'])
		self.ownerComp = ownerComp

		op('import_info').clear()

		self.Model = None

		gpus = tf.config.list_physical_devices('GPU')
		if gpus:
			try:
				for gpu in gpus:
					tf.config.experimental.set_memory_growth(gpu, True)
				logical_gpus = tf.config.list_logical_devices('GPU')
				op('debugTable').appendRow([absTime.frame, 'Physical GPUs: ' + str(len(gpus))])
				op('debugTable').appendRow([absTime.frame, 'Logical GPUs: ' + str(len(logical_gpus))])
				debug(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
			except RuntimeError as e:
				# Virtual devices must be set before GPUs have been initialized
				debug(e)
				op('debugTable').appendRow([absTime.frame, 'Logical Device Error: ' + str(e)])

		# enable NumPY behaviour for TensorFlow
		tnp.experimental_enable_numpy_behavior()

		# Model Settings
		self.Modeltype = tdu.Dependency(0)
		self.Modeltypes = ['linear_regression','lstm']
		self.Modelname = self.Modeltypes[self.Modeltype]

		self.MDNLayer = tdu.Dependency(0)
		self.MDNDistribution = tdu.Dependency(10)

		self.Inputdim = tdu.Dependency(67)
		self.Outputdim = tdu.Dependency(8)
		self.Batchsize= tdu.Dependency(1)
		self.Epochs = tdu.Dependency(1)
		self.Initialepochs = tdu.Dependency(1)
		self.Hiddendim = tdu.Dependency(16)
		self.Learningrate = tdu.Dependency(1)
		self.Timesteps = tdu.Dependency(8)

		# Selected Data Points
		self.Selectedfeatures = tdu.Dependency('^sound_*')
		self.Selectedtargets = tdu.Dependency('sound_*')
		self.DatapointsList = ''	

		# Training Data Handling
		self.Trainingdata = tdu.Dependency('')

		op('debugTable').appendRow([absTime.frame, f'Ext Init End'])

	def Getgpuinfo(self):
		built_with_cuda = tf.test.is_built_with_cuda()
		device_list = tf.config.list_physical_devices('GPU')
		vGpus_device_list = tf.config.list_logical_devices('GPU')

		info = op('import_info')
		info.clear()
		info.appendRow(['Built with CUDA', built_with_cuda])
		info.appendRow(['GPU Devices', device_list])
		info.appendRow(['GPU Logical Devices', vGpus_device_list])

	def ModelName(self):
		self.Modelname = self.Modeltypes[self.Modeltype]
		return self.Modelname

	def LoadSettingsFromConfigFile(self):
		#op('load_model_config').par.file = parent().par.Rootfolder + '/' + parent().par.Selectmodel + '/model_config.json'
		op('load_model_config').par.refreshpulse.pulse()
		model_config = op('model_config')
		self.Modeltype.val = model_config.result['Model_Type']
		try:
			self.MDNLayer.val = model_config.result['MDN_Layer']
		except:
			self.MDNLayer.val = 0
			debug("Old Model without MDN Setting")
		self.ModelName()
		self.Trainingdata.val = model_config.result['Training_Data']
		self.Selectedfeatures.val = model_config.result['Selected_Features']
		self.Selectedtargets.val = model_config.result['Selected_Targets']
		self.Inputdim.val = model_config.result['INPUT_DIM']
		self.Outputdim.val = model_config.result['OUTPUT_DIM']
		self.Batchsize.val = model_config.result['BATCH_SIZE']
		self.Epochs.val = model_config.result['EPOCHS']
		self.Initialepochs.val = model_config.result['INITIAL_EPOCHS']
		self.Hiddendim.val = model_config.result['HIDDEN_DIM']
		self.Learningrate.val = model_config.result['LEARNING_RATE']
		self.Timesteps.val = model_config.result['TIME_STEPS']

	def Loadmodel(self):
		model_folder = parent().par.Selectmodel.eval()
		op('debugTable').appendRow([absTime.frame, model_folder])
		with tf.device(tf.config.list_logical_devices('GPU')[0].name):
			self.Model = Sequential()
			self.Model = keras.models.load_model(model_folder)
			self.LoadSettingsFromConfigFile()
		op('debugTable').appendRow([absTime.frame, 'Loadmodel END'])
	
	def PredictTargets(self,features):
		targets = None
		with tf.device(tf.config.list_logical_devices('GPU')[0].name):
			targets = self.Model.predict(np.array([features]))
		return targets

	def PredictTargetsFromCHOP(self,features_chop):
		targets = None
		with tf.device(tf.config.list_logical_devices('GPU')[0].name):
			if self.Modelname == 'linear_regression':
				my_features = features_chop[0].vals
				targets = self.Model.predict(np.array([my_features]))
			elif self.Modelname == 'lstm':
				my_features = features_chop
				features = cp.ndarray([1,parent().Timesteps.val,parent().Inputdim.val],dtype='float32')
				for i in range(my_features.numChans):
					features[0][i] = cp.asarray(features_chop[i].numpyArray())
				features_in = cp.ndarray.get(features)
				if self.MDNLayer == 1:
					distributions = self.Model.predict(features_in)
					targets = np.apply_along_axis(mdn.sample_from_output,1,distributions,self.Outputdim.val,self.MDNDistribution,temp=1.0)
				else:
					targets = self.Model.predict(features_in)
		return targets

	def Testext(self):
		op('debugTable').appendRow([absTime.frame, 'Test Ext'])

	def Clear(self):
		op('debugTable').clear()
		op('error1').par.clear.pulse()