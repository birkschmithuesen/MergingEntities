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

import tensorflow as tf
import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, LSTM, Dropout, Activation
from tensorflow.keras.optimizers import SGD
from tensorflow.python.client import device_lib

import numpy as np

from TDStoreTools import StorageManager
import TDFunctions as TDF

class MLExtension:
	"""
	MLExtension description
	"""
	def __init__(self, ownerComp):
		# The component to which this extension is attached
		self.ownerComp = ownerComp

		# Environment Settings
		TDF.createProperty(self, 'Username', value='', dependable=True,
						   readOnly=False)
		
		TDF.createProperty(self, 'EnvName', value='', dependable=True,
						   readOnly=False)

		TDF.createProperty(self, 'EnvLocation', value='', dependable=True,
						   readOnly=False)

		TDF.createProperty(self, 'Model', value='', dependable=True,
						   readOnly=False)

		# Model Settings
		TDF.createProperty(self, 'INPUT_DIM', value='', dependable=True,
						   readOnly=False)
		
		TDF.createProperty(self, 'OUTPUT_DIM', value='', dependable=True,
						   readOnly=False)

		TDF.createProperty(self, 'BATCH_SIZE', value='', dependable=True,
						   readOnly=False)

		TDF.createProperty(self, 'EPOCHS', value='', dependable=True,
						   readOnly=False)

		TDF.createProperty(self, 'INITIAL_EPOCHS', value='', dependable=True,
						   readOnly=False)
		
		TDF.createProperty(self, 'HIDDEN_DIM', value='', dependable=True,
						   readOnly=False)
		
		TDF.createProperty(self, 'LEARNING_RATE', value='', dependable=True,
						   readOnly=False)

		# Training Settings
		TDF.createProperty(self, 'TrainingFileLocation', value='', dependable=True,
						   readOnly=False)
		TDF.createProperty(self, 'Features', value='', dependable=True,
						   readOnly=False)
		TDF.createProperty(self, 'Targets', value='', dependable=True,
						   readOnly=False)
		
		self.GetEnvSettings()

		debug('Is TensorFlow built with CUDA: ', tf.test.is_built_with_cuda())
		debug('Is GPU available: ', tf.test.is_gpu_available(cuda_only=False, min_cuda_compute_capability=None))
		debug('Num GPUs Available: ', len(tf.config.experimental.list_physical_devices('GPU')))
		debug('List of local devices: ', device_lib.list_local_devices())
		

	def GetEnvSettings(self):
		self.Username = parent.Ml.par.Windowsusername
		self.EnvName = parent.Ml.par.Condaenvironmentname
		self.EnvLocation = parent.Ml.par.Environmentfolder
		debug(self.Username, self.EnvName, self.EnvLocation)

	def InitiateCondaEnvironment(self):
		if platform.system() == 'Windows':
			if sys.version_info.major >= 3 and sys.version_info.minor >= 8:
				"""
				Double check all the following paths, it could be that your anaconda 'envs' folder is not in your User folder depending on your conda install settings and conda version.
				"""
				os.add_dll_directory('C:/Users/'+self.Username+'/anaconda3/envs/'+self.EnvName+'/DLLs')
				os.add_dll_directory('C:/Users/'+self.Username+'/anaconda3/envs/'+self.EnvName+'/Library/bin')
			else:
				"""
				Double check all the following paths, it could be that your anaconda 'envs' folder is not in your User folder depending on your conda install settings and conda version.
				"""
				# Not the most elegant solution, but we need to control load order
				os.environ['PATH'] = 'C:/Users/'+self.Username+'/anaconda3/envs/'+self.EnvName+'/DLLs' + os.pathsep + os.environ['PATH']
				os.environ['PATH'] = 'C:/Users/'+self.Username+'/anaconda3/envs/'+self.EnvName+'/Library/bin' + os.pathsep + os.environ['PATH']

		else:
			"""
			MacOS users should include path to .dlybs / MacOS binaries
			"""
			debug("No MAC os")

		sys.path = ['C:/Users/'+self.Username+'/anaconda3/envs/'+self.EnvName+'/Lib/site-packages'] + sys.path
		debug("Finished Initiation")
	
	def InitiateModel(self):
		self.Model = Sequential()
		debug("Initiated Model")
	
	def GetModelSettings(self):
		self.INPUT_DIM = int(parent.Ml.par.Inputdim)
		self.OUTPUT_DIM = int(parent.Ml.par.Outputdim)
		self.BATCH_SIZE = int(parent.Ml.par.Batchsize)
		self.EPOCHS = int(parent.Ml.par.Epochs)
		self.INITIAL_EPOCHS = int(parent.Ml.par.Initialepochs)
		self.HIDDEN_DIM = int(parent.Ml.par.Hiddendim)
		self.LEARNING_RATE = float(parent.Ml.par.Learningrate)
	
	def BuildModel(self):
		my_init=keras.initializers.RandomNormal(mean=0.0, stddev=0.05, seed=None)
		self.Model.add(Dense(self.HIDDEN_DIM, activation='sigmoid', input_dim=self.INPUT_DIM, kernel_initializer=my_init, bias_initializer=my_init))
		self.Model.add(Dense(self.OUTPUT_DIM, activation='sigmoid',kernel_initializer=my_init, bias_initializer=my_init))
		sgd = SGD(learning_rate=self.LEARNING_RATE, decay=1e-6, momentum=0.9, nesterov=True)
		self.Model.compile(loss='binary_crossentropy', optimizer=sgd, metrics=['accuracy'])
		debug("Built Model")

	def GetTrainingFileLocation(self):
		self.TrainingFileLocation = parent.Ml.par.Trainingdata

	def LoadTrainingData(self):
		file_loc = str(self.TrainingFileLocation)
		file = open(file_loc)
		values = np.loadtxt(file_loc, skiprows=1, dtype='float32')
		debug('Training Data Points: ', values.shape[0])
		self.Features, self.Targets = values[:,:-self.OUTPUT_DIM], values[:,self.INPUT_DIM:]
		debug('Training Features Shape: ', self.Features.shape, 'Training Targets Shape: ', self.Targets.shape)

	def FitModel(self):
		self.Model.fit(self.Features,self.Targets,epochs=self.INITIAL_EPOCHS,batch_size=self.BATCH_SIZE,shuffle=True)
		self.Model.summary()
		debug("Initial Training finished... ")
	
	def SaveModel(self):
		location = parent.Ml.par.Modelname
		self.Model.save('Models/' + location) #saving as SavedModel format

	def LoadModel(self):
		model_folder = str(parent.Ml.par.Selectmodel)
		self.Model = keras.models.load_model(model_folder)

	def PredictTargets(self,features):
		return self.Model.predict(np.array([features]))
