"""
Extension classes enhance TouchDesigner components with python. An
extension is accessed via ext.ExtensionClassName from any operator
within the extended component. If the extension is promoted via its
Promote Extension parameter, all its attributes with capitalized names
can be accessed externally, e.g. op('yourComp').PromotedFunction().

Help: search "Extensions" in wiki
"""

import os
from datetime import datetime

from io import StringIO

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.optimizers import SGD

import mdn

import numpy as np

import json

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
		self.Modeltype = tdu.Dependency(parent.Ml.par.Modeltype.menuIndex)
		self.Modeltypes = ['linear_regression','lstm']
		self.Modelname = self.Modeltypes[self.Modeltype.val]

		self.MDNLayer = tdu.Dependency(parent.Ml.par.Mdnlayer.val)
		self.MDNDistribution = tdu.Dependency(parent.Ml.par.Mdndistribution.val)

		self.Cells = tdu.Dependency(parent.Ml.par.Cells.val)
		self.Inputdim = tdu.Dependency(parent.Ml.par.Inputdim.val)
		self.Outputdim = tdu.Dependency(parent.Ml.par.Outputdim.val)
		self.Batchsize= tdu.Dependency(parent.Ml.par.Batchsize.val)
		self.Epochs = tdu.Dependency(parent.Ml.par.Epochs.val)
		self.Initialepochs = tdu.Dependency(parent.Ml.par.Initialepochs.val)
		self.Hiddendim = tdu.Dependency(parent.Ml.par.Hiddendim.val)
		self.Learningrate = tdu.Dependency(parent.Ml.par.Learningrate.val)
		self.Timesteps = tdu.Dependency(parent.Ml.par.Timesteps.val)

		# Selected Data Points
		self.Selectedfeatures = tdu.Dependency(parent.Ml.par.Selectedfeatures.val)
		self.Selectedtargets = tdu.Dependency(parent.Ml.par.Selectedtargets.val)
		self.DatapointsList = ''		   

		# Training Settings
		self.Trainingdata = tdu.Dependency(parent.Ml.par.Trainingdata.val)
		self.Trainingdatatype = tdu.Dependency(parent.Ml.par.Datatype.val)
		self.SetTrainingDataType()
		self.Features = ''
		self.Targets = ''

		# Config Attributes
		config_attributes = [ 			
			"Modeltype",
			"Modelname",
			"Cells",
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

		# profiling
		self.Logs = None
		self.Tboard_callback = None

		# test model
		self.X_Test = None
	
	def SetTrainingDataType(self):
		location, extension = os.path.splitext(self.Trainingdata.val)
		if extension == '.txt':
			self.Trainingdatatype.val =  'file'
		else:
			self.Trainingdatatype.val = 'folder'
	
	def PrintGPUInfo(self):
		built_with_cuda = tf.test.is_built_with_cuda()
		device_list = tf.config.list_physical_devices('GPU')
		vGpus_device_list = tf.config.list_logical_devices('GPU')

		debug(['Built with CUDA', built_with_cuda])
		debug(['GPU Devices', device_list])
		debug(['GPU Logical Devices', vGpus_device_list])

	def InitiateModel(self):
		with tf.device(tf.config.list_logical_devices('GPU')[0].name):
			self.Model = keras.Sequential()
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
			self.Model.add(layers.Dense(self.Hiddendim.val, activation='sigmoid', input_dim=self.Inputdim.val, kernel_initializer=my_init, bias_initializer=my_init))
			self.Model.add(layers.Dense(self.Outputdim.val, activation='sigmoid',kernel_initializer=my_init, bias_initializer=my_init))
			sgd = SGD(learning_rate=self.Learningrate.val, decay=1e-6, momentum=0.9, nesterov=True)
			self.Model.compile(loss='binary_crossentropy', optimizer=sgd, metrics=['accuracy'])
		elif self.Modelname == 'lstm':
			self.Model.add(layers.LSTM(units=64, batch_input_shape=(self.Batchsize.val, self.Timesteps.val, self.Inputdim.val), activation='tanh', dropout=0.2))
			self.Model.add(layers.Dense(units=32, activation='relu'))
			self.Model.add(layers.Dense(units=16, activation='relu'))
			self.Model.add(layers.Dense(units=8, activation='relu'))
			self.Model.add(layers.Dense(units=4, activation='relu'))
			self.Model.add(layers.Dense(units=2, activation='relu'))
			self.Model.add(layers.Dense(units=1, activation='relu'))
			optimizer = keras.optimizers.Adam()
			if self.MDNLayer.val == 1:
				self.Model.add(mdn.MDN(self.Outputdim.val,self.MDNDistribution.val))
				self.Model.compile(loss=mdn.get_mixture_loss_func(self.Outputdim.val,self.MDNDistribution.val), optimizer=optimizer)
			else:
				self.Model.compile(optimizer=optimizer,loss='mean_squared_error')
		debug("Built ", self.ModelName(), " Model")

	def LoadTrainingData(self):
		#file_loc = str(self.TrainingFileLocation)
		#file = open(file_loc)
		#values = np.loadtxt(file_loc, skiprows=1, dtype='float32')
		string_values = StringIO(op('selected_data').text)
		values = np.loadtxt(string_values,skiprows=1,dtype='float16')
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

	def CreateTensorBoardCallback(self):
		self.Logs = "logs/" + datetime.now().strftime("%Y%m%d-%H%M%S")
		self.Tboard_callback = keras.callbacks.TensorBoard(log_dir = self.Logs,
											histogram_freq = 1,
											profile_batch = '500,520')
		
	def FitModel(self):
		#self.CreateTensorBoardCallback()
		debug("Fitting Model")
		if self.Modelname == 'linear_regression':
			debug("Starting Linear Regression Fit")
			self.Model.fit(self.Features,
		  				self.Targets,
						epochs=self.Initialepochs.val,
						batch_size=self.Batchsize.val,
						shuffle=True)
		elif self.Modelname == 'lstm':
			debug("Starting LSTM Fit")
			try:
				self.Model.fit(x=self.Features,
		   					y=self.Targets,
							batch_size=self.Batchsize.val,
							epochs=self.Initialepochs.val,
							use_multiprocessing=False,
							#validation_data=(self.Features,self.Targets),
							#callbacks=[self.Tboard_callback])
							)
				# save tmp weights to later set into new model with batchsize 1
				tmp_model_weights = self.Model.get_weights()
				# set new batchsize
				#self.Batchsize.val = 1
				# make new model
				self.Model = keras.Sequential()
				self.Model.add(layers.LSTM(units=64, batch_input_shape=(self.Batchsize.val, self.Timesteps.val, self.Inputdim.val), activation='relu', dropout=0.2))
				self.Model.add(layers.Dense(units=32, activation='relu'))
				self.Model.add(layers.Dense(units=16, activation='relu'))
				self.Model.add(layers.Dense(units=8, activation='relu'))
				self.Model.add(layers.Dense(units=4, activation='relu'))
				self.Model.add(layers.Dense(units=2, activation='relu'))
				self.Model.add(layers.Dense(units=1, activation='relu'))
				# set weights from tmp model
				self.Model.set_weights(tmp_model_weights)
				optimizer = tf.keras.optimizers.Adam()
				if self.MDNLayer.val == 1:
					self.Model.add(mdn.MDN(self.Outputdim.val,self.MDNDistribution.val))
					self.Model.compile(loss=mdn.get_mixture_loss_func(self.Outputdim.val,self.MDNDistribution.val), optimizer=optimizer)
				else:
					self.Model.compile(optimizer=optimizer,loss='mean_squared_error')
			except ValueError as e:
				debug("Couldn't Fit Model", e)
		self.Model.summary()
		debug("Initial Training finished... ")

	def CreateExample(self):
		self.Model = keras.Sequential()
		self.Model.add(layers.LSTM(128,input_shape=(None,28)))
		self.Model.add(layers.BatchNormalization())
		self.Model.add(layers.Dense(10))
		debug(self.Model.summary())
		# get data
		mnist = keras.datasets.mnist
		(x_train, y_train),  (self.X_test, y_test) = mnist.load_data()
		x_train, self.X_test = x_train/255.0, self.X_test/255.0
		x_validate, y_validate = self.X_test[:10], y_test[:10]
		self.X_test, y_test = self.X_test[-10:], y_test[-10:]
		self.Model.compile(
			loss=keras.losses.SparseCategoricalCrossentropy(from_logits=True),
    			optimizer="sgd",
    			metrics=["accuracy"],
		)
		self.Model.fit(x_train, y_train,validation_data=(self.X_test,y_test),batch_size=64,epochs=1)
		self.Model.fit(x_train, y_train,validation_data=(x_validate,y_validate),batch_size=64,epochs=10)
		debug(self.Model.summary())

	def SaveModel(self):
		location = parent.Ml.par.Modelname
		self.Model.save('Models/' + location) #saving as SavedModel format
		self.CreateModelConfigFile()
		debug("Saved Model")

	def CreateModelConfigFile(self):
		location = parent.Ml.par.Modelname
		json_config = {}
		json_config['Model_Type'] = self.Modeltype.val
		json_config['MDN_Layer'] = self.MDNLayer.val
		json_config['Training_Data'] = self.Trainingdata.val
		json_config['Selected_Features'] = self.Selectedfeatures.val
		json_config['Selected_Targets'] = self.Selectedtargets.val
		json_config['CELLS'] = self.Cells.val
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
		try:
			self.MDNLayer.val = model_config.result['MDN_Layer']
		except:
			self.MDNLayer.val = 0
			debug("Old Model without MDN Setting")
		self.Modeltype.modified()
		self.ModelName()
		self.Trainingdata.val = model_config.result['Training_Data']
		self.SetTrainingDataType()
		self.Selectedfeatures.val = model_config.result['Selected_Features']
		self.Selectedtargets.val = model_config.result['Selected_Targets']
		try:
			self.Cells.val = model_config.result['CELLS']
		except:
			self.Cells.val = 128
			debug("Old Model without Cells Par")
		self.Inputdim.val = model_config.result['INPUT_DIM']
		self.Outputdim.val = model_config.result['OUTPUT_DIM']
		self.Batchsize.val = model_config.result['BATCH_SIZE']
		self.Epochs.val = model_config.result['EPOCHS']
		self.Initialepochs.val = model_config.result['INITIAL_EPOCHS']
		self.Hiddendim.val = model_config.result['HIDDEN_DIM']
		self.Learningrate.val = model_config.result['LEARNING_RATE']
		self.Timesteps.val = model_config.result['TIME_STEPS']

	def PredictTargets(self,features):
		if self.MDNLayer == 1:
			distributions = self.Model.predict(np.array([features]))
			return np.apply_along_axis(mdn.sample_from_output,1,distributions,self.Outputdim.val,self.MDNDistribution,temp=1.0)
		else:
			with tf.device('/gpu:0'):
				return self.Model.predict(np.array([features]))