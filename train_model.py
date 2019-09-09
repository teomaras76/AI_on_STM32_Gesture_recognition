#from __future__ import print_function
#import matplotlib.pyplot as plt
#import numpy as np
#import time
#import csv

#import numpy as np
#np.random.seed(10)
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
#import signal_processing as sp
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
parser.add_argument('--epochs', default=30, type=int, metavar='N',
                    help='number of total epochs to run (default: 30)')
parser.add_argument('--lr', default=0.00001, type=float,
                    help='initial learning rate')
parser.add_argument('--batch-size', default=50, type=int,
                    metavar='N', help='mini-batch size (default: 50)')
parser.add_argument('--n_axis', default=6, type=int,
                   help='number of input (default: 6)')
parser.add_argument('--n_cells', default=64, type=int,
                    help='number of GRU cells for layer (default: 64)')
parser.add_argument('--n_layers', default=2, type=int,
                    help='number of layers (default: 2)')
parser.add_argument('--seq_length', type=int, default=128)
parser.add_argument('-n', '--n_classes', default=10, type=int,      # To fix later 9 classes instead of 10 (class=0 not used)
                    help='number of classes (default: 9)')
parser.add_argument('--mode', default='train')
parser.add_argument('--pretrained', default = "", type= str,
                    help='path to pre-trained model')
parser.add_argument('--input_type', default = "raw", type= str,
                    help='select input type')
parser.add_argument('--m', default='ox' ,type = str,help='model')
parser.add_argument('--retrain', default=False, help='retrain the NN or load saved parameters (default : False)')
parser.add_argument('--create', default=False, help='create the dataset from log files (default : False)')
parser.add_argument('--shuffle_dataset', default=False, help='reshuffle dataset for train and test sets (default : False)')
args = parser.parse_args()

# Plot a graph with the rsult of Loss and Accuracy after the training
def plot_loss_acc(file_path):
    epoch = []
    loss = []
    acc = []
    val_loss = []
    val_acc = []
    test_loss = []
    test_acc = []
    i=0

    if sys.version_info[0] < 3:
        in_file = open (file_path, 'rb')
    else:
        in_file = open (file_path, 'r', newline='', encoding='utf8')
        
    with in_file as f:
        for row in f:
            seq=row
            if i>0:
                if 2 < i < args.epochs:
                    epoch.append(np.array(row.strip().split(","))[0].astype(float))
                    loss.append(np.array(row.strip().split(","))[1].astype(float))
                    acc.append(np.array(row.strip().split(","))[2].astype(float))
                    val_loss.append(np.array(row.strip().split(","))[3].astype(float))
                    val_acc.append(np.array(row.strip().split(","))[4].astype(float))
                    test_loss.append(np.array(row.strip().split(","))[5].astype(float))
                    test_acc.append(np.array(row.strip().split(","))[6].astype(float))
            i = i+1

    #loss = np.array(loss)
    #print(loss)
    x_axis = epoch
    fig = plt.figure()
    plt.subplot(211)
    plt.title('Training results')
    plt.ylabel('Loss')
    plt.grid(True)
    plt.plot(x_axis,loss,'b', label='Loss')
    plt.plot(x_axis,val_loss,'r', label='Validation Loss')
    plt.plot(x_axis,test_loss,'g', label='Test Loss')
    plt.legend(prop={'size': 8})

    plt.subplot(212)
    plt.ylabel('Accuracy')
    plt.xlabel('Epochs')
    plt.grid(True)
    plt.plot(x_axis,acc,'b', label='Accuracy')
    plt.plot(x_axis,val_acc,'r', label='Validation Accuracy')
    plt.plot(x_axis,test_acc,'g', label='Test Accuracy')
    plt.legend(prop={'size': 8})
    
    fig.savefig(str(args.log_path)+"\\"+"train_results.png")
    return fig

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
# X Reshape (m_samples, num_axis, seq_length, 1)
print("X shape is = ")
print(X.shape)
print (X)
#X = X.reshape(X.shape[0], args.n_axis, args.seq_length, 1)
X = X.reshape(X.shape[0], args.seq_length, args.n_axis, 1)
print("X after Reshape shape is = ")
print(X.shape)


# Reshuffle and split dataset in training and test
x_train, x_test, y_train, y_test = split_dataset(X, Y, 80, args.shuffle_dataset)

#x_train = x_train.reshape(-1, args.n_axis, args.seq_length, 1)
#x_test = x_test.reshape(-1, args.n_axis, args.seq_length, 1)

#Save test data in .npy format for verification with Cube.AI tool
np.save(str(args.data_proc)+"/"+"testset_x.npy",x_test);
np.save(str(args.data_proc)+"/"+"testset_y.npy",y_test);


print("input train shape:{}".format(x_train.shape))
print("input test shape:{}".format(x_test.shape))
#To uncomment in case of checkpoints used
#model, checkpoint ,save_path= create_model(args)
model = create_model(args)

if args.retrain:
    #Train thre model
    ts = time.time()
    train_log_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    print (train_log_time)
    save_path = "./Train log/"

    with open(os.path.join(save_path,"log.csv"),'w', newline='', encoding='utf8') as save_log_file:
        csvwiter = csv.writer(save_log_file)
        csvwiter.writerow(['epoch','loss','acc','val_loss','val_acc' ,'test_loss', 'test_acc'])    

    for i in range(args.epochs):
        #To uncomment in case of checkpoints used
        #hist = model.fit(x_train, y_train, batch_size=args.batch_size, verbose=1 ,callbacks=[checkpoint], epochs=1, validation_split=0.2)
        hist = model.fit(x_train, y_train, batch_size=args.batch_size, verbose=1, epochs=1, validation_split=0.2, shuffle=False)
        score = model.evaluate(x_test,y_test,batch_size=args.batch_size,verbose=1)
        print('[{}]Test loss :{} Test acc :{}'.format(i,score[0],score[1]))
    
        with open(os.path.join(save_path, "log.csv"), 'a', newline='', encoding='utf8') as save_log_file:
            csvwiter = csv.writer(save_log_file)
            csvwiter.writerow([i, hist.history['loss'][0], hist.history['acc'][0], hist.history['val_loss'][0],
                      hist.history['val_acc'][0],score[0],score[1]])
        
    #Save the model in .h5 file
    model.save('NN_gesture.h5')

    # Save the model and weights in separated files
    # save model as JSON
    json_string = model.to_json()
    with open("NN_gesture.json", "w") as json_file:
        json_file.write(json_string)
    model.save_weights('NN_gesture_weights.h5')

    # Plot the model
    #plot_model(model, to_file=os.path.join(save_path,'CNN_model.png'))

    # Plot Loss and Accuracy results after training
    img = plot_loss_acc(os.path.join(save_path, "log.csv"))
    #plt.show()

else:
    #Read json and create model with saved parameters
    json_file = open('NN_gesture.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    model = model_from_json(loaded_model_json)
    model.load_weights('NN_gesture_weights.h5')
    

# Make prediction for training set
y_pred = model.predict(x_train, batch_size=args.batch_size, verbose=1)
print("Confusion matrix for training dataset")
conf_matrix = confusion_matrix(y_train.argmax(axis=1), y_pred.argmax(axis=1), labels=[1,2,3,4,5,6,7,8,9])
print(conf_matrix)
print("Y_train shape")
print(y_train.shape)
print("Y_pred shape")
print(y_pred.shape)
error=calculate_error(conf_matrix)
print("Errors in the training dataset")
print(error)
print("Accuracy for training dataset")
accuracy = 1 - (error / y_train.shape[0])
print(accuracy)
    
# Make prediction for given input to generate Confusion matrix
y_pred = model.predict(x_test, batch_size=args.batch_size, verbose=1)
#print("Prediction for the test X input:")
#print(y_pred)
#print("Corresponding test Y one hot vector:")
#print(y_test)
#print("Predicted labels:")
#for j in range(y_pred.shape[0]):
#    print(np.argmax(y_pred[j,:]))

# Print the confusion matrix for test dataset
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

train_labels=np.zeros(y_train.shape[0])
for test_item in y_train:
    for j in range(10):
        if test_item[j]==1:
            #print(j)
            train_labels[j]+=1

for i in range(10):
    print("Number of logs for training dataset:"+"Label_"+str(i)+" >> "+str(train_labels[i]))
			

test_labels=np.zeros(y_test.shape[0])
for test_item in y_test:
    for j in range(10):
        if test_item[j]==1:
            #print(j)
            test_labels[j]+=1

for i in range(10):
    print("Number of logs for test dataset:"+"Label_"+str(i)+" >> "+str(test_labels[i]))
			

