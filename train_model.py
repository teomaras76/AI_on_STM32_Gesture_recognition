import numpy as np
np.random.seed(10)
#tf.set_random_seed(10)

from keras.models import Sequential
from keras.layers.core import Dense, Activation #, Dropout
from keras.layers.recurrent import LSTM #, SimpleRNN
from keras.layers.wrappers import TimeDistributed
from keras.layers import Convolution2D,Dropout,Flatten ,Reshape, Bidirectional, Lambda

import keras.backend as K
import argparse
import os
import datetime
import random
import csv
import time

from NN_model_Keras import *
from input_data_processing import*
#from keras.utils import plot_model
from sklearn.metrics import confusion_matrix
#from sklearn.metrics import classification_report

from keras.models import model_from_json


parser = argparse.ArgumentParser(description='Tensorflow  backend Gesture Classification Training on several datasets')
parser.add_argument('--data_raw', default = "Dataset original/ODR 52Hz", type= str,
                    help='path to RAW log file')
parser.add_argument('--data_proc', default = "Processed/ODR 52Hz", type= str,
                    help='path to processed dataset')
parser.add_argument('--log_path', default = "Train log", type= str,
                    help='path to processed dataset')
parser.add_argument('--epochs', default=150, type=int, metavar='N',
                    help='number of total epochs to run (default: 150)')
parser.add_argument('--batch_size', default=30, type=int,
                    metavar='N', help='mini-batch size (default: 30)')
parser.add_argument('--n_axis', default=6, type=int,
                   help='number of axis input (default: 6)')
parser.add_argument('--n_cells', default=64, type=int,
                    help='number of GRU cells for layer (default: 64)')
parser.add_argument('--n_layers', default=2, type=int,
                    help='number of GRU layers (default: 2)')
parser.add_argument('--seq_length', type=int, default=128, help='input sequence lenght (default:128)')
parser.add_argument('-n', '--n_classes', default=10, type=int,      # One dummy class=0 for not valid gestures
                    help='number of classes (default: 9)')
parser.add_argument('--retrain', default=False, help='retrain the NN or load saved parameters (default : False)')
parser.add_argument('--create', default=False, help='create the dataset from log files (default : False)')
parser.add_argument('--shuffle_dataset', default=False, help='reshuffle dataset for train and test sets (default : False)')
args = parser.parse_args()



def calculate_error (conf_matrix):
    err = 0
    for i in range (conf_matrix.shape[0]):
        for j in range (conf_matrix.shape[0]):
            if (i != j):
                err = err + conf_matrix[i,j]
    return err


# Create the dataset from log files
if args.create:
    create_dataset(args.data_raw, args.data_proc, args.seq_length, args.n_axis)

# Load dataset in X and Y
X = load_dataset(str(args.data_proc)+"/"+"dataset_x.csv")
print("X shape is = ")
print(X.shape)
Y = load_dataset(str(args.data_proc)+"/"+"dataset_y.csv")
print("Y shape is = ")
print(Y.shape)

# Y transformed in one_hot vector
Y = one_hot(Y)
print("Y after one_hot shape is = ")
print(Y.shape)

# Change structure of X
X=reshape_input_buffer(X,args.n_axis,args.seq_length)
print("X shape is = ")
print(X.shape)
print (X)
#X = X.reshape(X.shape[0], args.n_axis, args.seq_length, 1)
X = X.reshape(X.shape[0], args.seq_length, args.n_axis, 1)
print("X after Reshape shape is = ")
print(X.shape)


# Reshuffle and split dataset in training and test
x_train, x_test, y_train, y_test = split_dataset(X, Y, 80, args.shuffle_dataset)


#Save test data in .npy format for verification with Cube.AI tool
np.save(str(args.data_proc)+"/"+"testset_x.npy",x_test);
np.save(str(args.data_proc)+"/"+"testset_y.npy",y_test);


print("input train shape:{}".format(x_train.shape))
print("input test shape:{}".format(x_test.shape))
model = create_model(args)

if args.retrain:
    #Train thre model
    ts = time.time()
    train_log_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    print (train_log_time)
    save_path = "./Train log/"
        
    history = model.fit(x_train, y_train, batch_size=args.batch_size, verbose=2, epochs=args.epochs, validation_split=0.2, shuffle=False, validation_data=(x_test, y_test))
        
    #Save the model in .h5 file
    model.save('NN_gesture.h5')
    
    # Save the model and weights in separated files
    # save model as JSON
    json_string = model.to_json()
    with open("NN_gesture.json", "w") as json_file:
        json_file.write(json_string)
    model.save_weights('NN_gesture_weights.h5')

    # Plot the metrics after training
    fig = plt.figure()
    plt.subplot(2,1,1)
    plt.plot(history.history['acc'])
    plt.plot(history.history['val_acc'])
    plt.title('Model accuracy')
    plt.ylabel('accuracy')
    plt.xlabel('epochs')
    plt.legend(['train','test'],loc='lower right')

    plt.subplot(2,1,2)
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('loss')
    plt.xlabel('epochs')
    plt.legend(['train','test'],loc='upper right')
    
    plt.tight_layout()
    fig.savefig(str(args.log_path)+"/"+"train_results.png")

else:
    #Read json and create model with saved parameters
    json_file = open('NN_gesture.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    model = model_from_json(loaded_model_json)
    model.load_weights('NN_gesture_weights.h5')

# Evaluate model perfomance 
scores = model.evaluate(x_test,y_test,verbose=2)
print("%s: %.2f%%" % (model.metrics_names[0], scores[0]*100))
print("%s: %.2f%%" % (model.metrics_names[1], scores[1]*100))

    
# Make prediction for given input to generate Confusion matrix
y_pred = model.predict(x_test, verbose=1)

# Print the confusion matrix for test dataset
print()
print("Confusion matrix for test dataset")
conf_matrix = confusion_matrix(y_test.argmax(axis=1), y_pred.argmax(axis=1), labels=[1,2,3,4,5,6,7,8,9])
print(conf_matrix)
error=calculate_error(conf_matrix)
print("Errors in the test dataset")
print(error)
print("Accuracy for test dataset")
accuracy = 1 - (error / y_test.shape[0])
print(accuracy)

#print(classification_report(y_test.argmax(axis=1),y_pred.argmax(axis=1)))

print()
train_labels=np.zeros(y_train.shape[0])
for test_item in y_train:
    for j in range(10):
        if test_item[j]==1:
            train_labels[j]+=1

for i in range(10):
    print("Number of logs for training dataset:"+"Label_"+str(i)+" >> "+str(train_labels[i]))
			

test_labels=np.zeros(y_test.shape[0])
for test_item in y_test:
    for j in range(10):
        if test_item[j]==1:
            test_labels[j]+=1

for i in range(10):
    print("Number of logs for test dataset:"+"Label_"+str(i)+" >> "+str(test_labels[i]))

