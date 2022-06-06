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

from io import StringIO

import tensorflow as tf
import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, LSTM, Dropout, Activation
from tensorflow.keras.optimizers import SGD
from tensorflow.python.client import device_lib

import gc

import numpy as np

import json

from TDStoreTools import StorageManager
import TDFunctions as TDF

class MLExtension:
	"""
	MLExtension description
	"""
	def __init__(self, ownerComp):
		# The component to which this extension is attached
		self.ownerComp = ownerComp
		
		# Model
		self.Model = None

		# Model Settings
		self.Modeltype = tdu.Dependency(0)
		self.Modeltypes = ['linear_regression','lstm']
		self.Modelname = self.Modeltypes[self.Modeltype.val]

		self.Inputdim = tdu.Dependency(67)
		self.Outputdim = tdu.Dependency(8)
		self.Batchsize= tdu.Dependency(1)
		self.Epochs = tdu.Dependency(1)
		self.Initialepochs = tdu.Dependency(1)
		self.Hiddendim = tdu.Dependency(10)
		self.Learningrate = tdu.Dependency(1)
		self.Timesteps = tdu.Dependency(8)

		# Selected Data Points
		self.Selectedfeatures = tdu.Dependency('^sound_*')
		self.Selectedtargets = tdu.Dependency('sound_*')
		self.DatapointsList = ''		   

		# Training Settings
		self.Trainingdata = tdu.Dependency('Entity-Recordings/default.txt')
		self.Features = ''
		self.Targets = ''

		# Config Attributes
		config_attributes = [ 			
			"Modeltype",
			"Modelname",
			"Inputdim",
			"Outputdim",
			"Batchsize",
			"Epochs",
			"Initialepochs",
			"Hiddendim",
			"Learningrate",
			"Timesteps"
		]

		# set tf gpu selection and memory growth
		gpus = tf.config.list_physical_devices('GPU')
		if gpus:
			try:
				# Currently, memory growth needs to be the same across GPUs
				for gpu in gpus:
					tf.config.experimental.set_memory_growth(gpu, True)
					debug('Set Memory Growth to True')
					logical_gpus = tf.config.list_logical_devices('GPU')
					debug(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
			except RuntimeError as e:
				# Memory growth must be set before GPUs have been initialized
				debug(e)

		self.PrintGPUInfo()
	
	def PrintGPUInfo(self):
		built_with_cuda = tf.test.is_built_with_cuda()
		device_list = tf.config.list_physical_devices('GPU')
		vGpus_device_list = tf.config.list_logical_devices('GPU')

		debug(['Built with CUDA', built_with_cuda])
		debug(['GPU Devices', device_list])
		debug(['GPU Logical Devices', vGpus_device_list])

	def InitiateModel(self):
		with tf.device(tf.config.list_logical_devices('GPU')[0].name):
			self.Model = Sequential()
			debug("Initiated Model")

	def ModelName(self):
		self.Modelname = self.Modeltypes[self.Modeltype]
		return self.Modelname
	
	def GetModelSettings(self):
		# Get Settings from Data Selection
		self.DatapointsList = op('selected_data_points').row(0)
		# Lookup Model Name from List
		self.ModelName()
		debug("Get Model Settings")
	
	def BuildModel(self):
		if self.Modelname == 'linear_regression':
			my_init=keras.initializers.RandomNormal(mean=0.0, stddev=0.05, seed=None)
			self.Model.add(Dense(self.Hiddendim.val, activation='sigmoid', input_dim=self.Inputdim.val, kernel_initializer=my_init, bias_initializer=my_init))
			self.Model.add(Dense(self.Outputdim.val, activation='sigmoid',kernel_initializer=my_init, bias_initializer=my_init))
			sgd = SGD(learning_rate=self.Learningrate.val, decay=1e-6, momentum=0.9, nesterov=True)
			self.Model.compile(loss='binary_crossentropy', optimizer=sgd, metrics=['accuracy'])
		elif self.Modelname == 'lstm':
			self.Model.add(LSTM(units=128, batch_input_shape=(self.Batchsize.val, self.Timesteps.val, self.Inputdim.val), stateful=True, return_sequences=True))
			self.Model.add(LSTM(units=128, batch_input_shape=(self.Batchsize.val, self.Timesteps.val, self.Inputdim.val), stateful=True, return_sequences=False))
			self.Model.add(Dense(units=self.Outputdim.val, activation='sigmoid'))
			self.Model.compile(optimizer='rmsprop',loss='mse')
		debug("Built ", self.ModelName(), " Model")

	def LoadTrainingData(self):
		#file_loc = str(self.TrainingFileLocation)
		#file = open(file_loc)
		#values = np.loadtxt(file_loc, skiprows=1, dtype='float32')
		string_values = StringIO(op('selected_data').text)
		values = np.loadtxt(string_values,skiprows=1,dtype='float32')
		debug('Training Data Points: ', values.shape[0])
		self.Features, self.Targets = values[:,:-self.Outputdim.val], values[:,self.Inputdim.val:]
		if self.Modelname == 'lstm':
			self.Features = self.rolling_window2D(self.Features,self.Timesteps.val)
			#self.Targets = self.Targets[self.Timesteps.val-1:]
			#adjusting to batch_size
			length, y, z = self.Features.shape
			offset = length % self.Batchsize
			feature_length = length - offset - self.Batchsize
			self.Features = self.Features[0:feature_length, ]
			self.Targets = self.Targets[self.Timesteps-1:feature_length+self.Timesteps-1, ]
		debug('Training Features Shape: ', self.Features.shape, 'Training Targets Shape: ', self.Targets.shape)

	def rolling_window2D(self,a,n):
		# a: 2D Input array 
		# n: Group/sliding window length
		return a[np.arange(a.shape[0]-n+1)[:,None] + np.arange(n)]

	def FitModel(self):
		debug("Fitting Model")
		if self.Modelname == 'linear_regression':
			debug("Starting Linear Regression Fit")
			self.Model.fit(self.Features,self.Targets,epochs=self.Initialepochs.val,batch_size=self.Batchsize.val,shuffle=True)
		elif self.Modelname == 'lstm':
			debug("Starting LSTM Fit")
			try:
				self.Model.fit(x=self.Features,y=self.Targets,batch_size=self.Batchsize.val,epochs=self.Initialepochs.val,use_multiprocessing=False)
				# save tmp weights to later set into new model with batchsize 1
				tmp_model_weights = self.Model.get_weights()
				# set new batchsize
				self.Batchsize.val = 1
				# make new model
				self.Model = Sequential()
				self.Model.add(LSTM(units=128, batch_input_shape=(self.Batchsize.val, self.Timesteps.val, self.Inputdim.val), stateful=True, return_sequences=True))
				self.Model.add(LSTM(units=128, batch_input_shape=(self.Batchsize.val, self.Timesteps.val, self.Inputdim.val), stateful=True, return_sequences=False))
				self.Model.add(Dense(units=self.Outputdim.val, activation='sigmoid'))
				# set weights from tmp model
				self.Model.set_weights(tmp_model_weights)
				self.Model.compile(optimizer='rmsprop',loss='mse')
			except ValueError as e:
				debug("Couldn't Fit Model", e)
		#self.Model.summary()
		debug("Initial Training finished... ")

	def SaveModel(self):
		location = parent.Ml.par.Modelname
		self.Model.save('Models/' + location) #saving as SavedModel format
		self.CreateModelConfigFile()
		debug("Saved Model")

	def CreateModelConfigFile(self):
		location = parent.Ml.par.Modelname
		json_config = {}
		json_config['Model_Type'] = self.Modeltype.val
		json_config['Training_Data'] = self.Trainingdata.val
		json_config['Selected_Features'] = self.Selectedfeatures.val
		json_config['Selected_Targets'] = self.Selectedtargets.val
		json_config['INPUT_DIM'] = self.Inputdim.val
		json_config['OUTPUT_DIM'] = self.Outputdim.val
		json_config['BATCH_SIZE'] = self.Batchsize.val
		json_config['EPOCHS'] = self.Epochs.val
		json_config['INITIAL_EPOCHS'] = self.Initialepochs.val
		json_config['HIDDEN_DIM'] = self.Hiddendim.val
		json_config['LEARNING_RATE'] = self.Learningrate.val
		json_config['TIME_STEPS'] = self.Timesteps.val
		with open('Models/' + location + '/model_config.json', 'w') as jsonFile:
			json.dump(json_config,jsonFile)

	def LoadModel(self):
		model_folder = str(parent.Ml.par.Selectmodel)
		self.Model = keras.models.load_model(model_folder)
		self.LoadSettingsFromConfigFile()
		debug("Loaded Model")

	def LoadSettingsFromConfigFile(self):
		op('load_model_config').par.refreshpulse.pulse()
		model_config = op('model_config')
		self.Modeltype.val = model_config.result['Model_Type']
		self.Modeltype.modified()
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

	def PredictTargets(self,features):
		return self.Model.predict(np.array([features]))
