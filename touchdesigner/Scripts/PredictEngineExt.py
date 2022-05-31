"""
Extension classes enhance TouchDesigner components with python. An
extension is accessed via ext.ExtensionClassName from any operator
within the extended component. If the extension is promoted via its
Promote Extension parameter, all its attributes with capitalized names
can be accessed externally, e.g. op('yourComp').PromotedFunction().

Help: search "Extensions" in wiki
"""

import sys
import os
import platform

user = parent().par.Windowsusername
debug(user)
	 
condaEnv = parent().par.Condaenvironmentname
debug(condaEnv)
	
envFolder = parent().par.Environmentfolder
debug(envFolder)
			
if platform.system() == 'Windows':
	if sys.version_info.major >= 3 and sys.version_info.minor >= 8:
		"""
		Double check all the following paths, it could be that your anaconda 'envs' folder is not in your User folder depending on your conda install settings and conda version.
		"""
		os.add_dll_directory('C:/Users/'+user+'/anaconda3/envs/'+condaEnv+'/DLLs')
		os.add_dll_directory('C:/Users/'+user+'/anaconda3/envs/'+condaEnv+'/Library/bin')
	else:
		"""
		Double check all the following paths, it could be that your anaconda 'envs' folder is not in your User folder depending on your conda install settings and conda version.
		"""
		# Not the most elegant solution, but we need to control load order
		os.environ['PATH'] = 'C:/Users/'+user+'/anaconda3/envs/'+condaEnv+'/DLLs' + os.pathsep + os.environ['PATH']
		os.environ['PATH'] = 'C:/Users/'+user+'/anaconda3/envs/'+condaEnv+'/Library/bin' + os.pathsep + os.environ['PATH']

else:
	"""
	MacOS users should include path to .dlybs / MacOS binaries
	"""
	debug("No MAC os")

sys.path = ['C:/Users/'+user+'/anaconda3/envs/'+condaEnv+'/Lib/site-packages'] + sys.path

from io import StringIO

import tensorflow as tf
import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, LSTM, Dropout, Activation
from tensorflow.keras.optimizers import SGD
from tensorflow.python.client import device_lib

import numpy as np

import json


from TDStoreTools import StorageManager
import TDFunctions as TDF

class PredictEngineExt:
	"""
	PredictEngineExt description
	"""
	def __init__(self, ownerComp):
		# The component to which this extension is attached
		self.ownerComp = ownerComp

		op('import_info').clear(keepSize=True)

		self.Model = Sequential()

		# Model Settings
		self.Modeltype = tdu.Dependency(0)
		self.Modeltypes = ['linear_regression','lstm']
		self.Modelname = self.Modeltypes[self.Modeltype]

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

	def Getgpuinfo(self):
		built_with_cuda = tf.test.is_built_with_cuda()
		gpu_available = tf.test.is_gpu_available(cuda_only=False, min_cuda_compute_capability=None)
		num_gpus = len(tf.config.experimental.list_physical_devices('GPU'))
		device_list = device_lib.list_local_devices()

		info = op('import_info')
		info.clear(keepSize=True)
		info[0,0] = built_with_cuda
		info[1,0] = gpu_available
		info[2,0] = num_gpus
		info[3,0] = device_list

	def ModelName(self):
		self.Modelname = self.Modeltypes[self.Modeltype]
		return self.Modelname

	def LoadSettingsFromConfigFile(self):
		op('load_model_config').par.file = parent().par.Rootfolder + '/' + parent().par.Selectmodel + '/model_config.json'
		op('load_model_config').par.refreshpulse.pulse()
		model_config = op('model_config')
		self.Modeltype.val = model_config.result['Model_Type']
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

	def Printmodelinfo(self):
		debug("print info")

	def Loadmodel(self):
		model_folder = parent().par.Rootfolder + '/' + parent().par.Selectmodel
		self.Model = Sequential()
		self.Model = keras.models.load_model(model_folder)
		self.LoadSettingsFromConfigFile()
		op('model_info').text = "summary"
		debug("Loaded Model")
	
	def PredictTargets(self,features):
		return self.Model.predict(np.array([features]))