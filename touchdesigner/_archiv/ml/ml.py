import keras
from keras.models import Sequential
from keras.layers import Dense, LSTM, Dropout, Activation
from keras.optimizers import SGD

MODEL_LOAD_FILE_PATH = './model.h5'
MODEL_SAVE_FILE_PATH = './model_new.h5'

INPUT_DIM = 2
BATCH_SIZE = 2
EPOCHS = 250
INITIAL_EPOCHS = 50
HIDDEN1_DIM = 3
OUTPUT_DIM = 5
LEARNING_RATE = 3.2

model = Sequential()

def build_model():
    """
    Initialize a new network
    """
    my_init=keras.initializers.RandomNormal(mean=0.0, stddev=0.05, seed=None)
    model.add(Dense(HIDDEN1_DIM, activation='sigmoid', input_dim=INPUT_DIM, kernel_initializer=my_init, bias_initializer=my_init))
    model.add(Dense(OUTPUT_DIM, activation='sigmoid',kernel_initializer=my_init, bias_initializer=my_init))
    sgd = SGD(lr=LEARNING_RATE, decay=1e-6, momentum=0.9, nesterov=True)
    model.compile(loss='binary_crossentropy', optimizer=sgd, metrics=['accuracy'])

def load_trainingsdata():
    """
    loading the trainingsdata from a textfile. Convert the
    trainingpoints in the right dimension: One line in the text file is
    one trainingpoint with 30 FFT values and 13824 LED values, separated by tabulators
    """
    global training_input, training_output
    #import fft and led input data
    file_name = MODEL_TRAININGS_DATA_FILE_PATH
    file = open(file_name)
    print('Loading Trainingsdata from File:', file_name,'  ...')
    values = np.loadtxt(file_name, dtype='float32')
    print('Trainingsdata points: ', values.shape[0], "\n")
    #split into input and outputs
    training_input, training_output = values[:,:-OUTPUT_DIM], values[:,INPUT_DIM:]
    print('training_input shape: ', training_input.shape, 'training_output shape: ', training_output.shape)
    
def train_model():
    """
    trains the model with INITIAL_EPOCHS
    """
    global model, training_input, training_output
    model.fit(training_input, training_output, epochs=INITIAL_EPOCHS, batch_size=BATCH_SIZE, shuffle=True)
    model._make_predict_function()
    model.summary()
    print('Initial training finished...')

def continue_training():
    """
    trains the model with EPOCHS
    """
    global model, training_input, training_output
    model.fit(training_input, training_output, epochs=EPOCHS, batch_size=BATCH_SIZE, shuffle=True)
    model._make_predict_function()
    print('training finished...')
    print('')

def save_model():
    """
    saves the model to disk
    """
    model.save(MODEL_SAVE_FILE_PATH)
    print('Saved new model to path: ', MODEL_SAVE_FILE_PATH)
    model.summary()