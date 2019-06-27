#from __future__ import division, print_function
import sys

import os
#from matplotlib import pyplot
import matplotlib.pyplot as plt
#import glob

import numpy as np

# Data set in .csv format
import csv

from itertools import tee
#from skimage import util
import csv
#from sklearn import preprocessing
from sklearn.utils import shuffle
#from scipy.signal import resample
import math




# Scan the given folder and return list of files
def get_logfile_list(dir_path):
    files_list = os.listdir(dir_path)
    #print(files_list)
    return files_list

# Clean the .csv log file created by Sensortile eliminating the first 2 rows
def clean_file(input_file_path, output_file_path):
    if sys.version_info[0] < 3:
        in_file = open (input_file_path, 'rb')
        out_file = open (output_file_path, 'wb')
    else:
        in_file = open (input_file_path, 'r', newline='', encoding='utf8')
        out_file = open (output_file_path, 'w', newline='', encoding='utf8')
    i=0
    #csv.register_dialect('myDialect', lineterminator = '\n')    #Default is '\r\n' but insert one empty row in the file
    with in_file as inp, out_file as out:
        writer = csv.writer(out)
        reader = csv.reader(inp)
        for row in reader:
            i=i+1
            if i>2:
                writer.writerow(row)


# Open file_path and load log data:
# x_data from 3th to 8th column (acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z)
# y_data in 2th column
def load_data_from_log(file_path,sequence_length):
    x_data = []
    y_data = []
    i=0

    if sys.version_info[0] < 3:
        in_file = open (file_path, 'rb')
    else:
        in_file = open (file_path, 'r', newline='', encoding='utf8')
        
    with in_file as f:
        for row in f:
            seq=row
            #seq.strip()
            #print(seq)
            if (seq.strip().endswith(",,,,"))&(i==0):
                print("Label found")        # Just for debugging
                print(seq.strip())
                i = i+1
                y_data.append(np.array([seq[18]]))
                print(y_data)
            if i>0:
                i = i+1
                if 2 < i < sequence_length+3:
                    x_data.append(np.array(row.strip().split(","))[2:8].astype(float))
                    #print(x_data)
            
    #print("Shape of X data before transpose:")
    x_data = np.array(x_data,dtype=np.float32)
    y_data = np.array(y_data[0])
    #print(x_data.shape)
    #print(x_data)
    #print("Shape of Y data before transpose:")
    #print(y_data.shape)
    #plot_data(y_data)
    #plot_ACC_3axis_data(x_data[:,0:3])
    #plot_GYR_3axis_data(x_data[:,3:6])
    plot_img = plot_ACC_GYR_6axis_data(x_data[:,0:3], x_data[:,3:6],"Label = "+str(y_data[0]))

    #print(y_data)
##    #Transpose X and Y vectors
##    x_data = np.array(x_data,dtype=np.float32).transpose()
##    y_data = np.array(y_data,dtype=np.float32).transpose()
##    print("Shape of X data after transpose:")
##    print(x_data.shape)
##    print("Shape of Y data after transpose:")
##    print(y_data.shape)
    return x_data, y_data, plot_img

#def scale_data(data):
#    scaled_data = []
#    for row in data:
#        scaled = []
#        for d in row:
#            d = preprocessing.scale(d)
#            scaled.append(d)
#        scaled= np.array(scaled_x,dtype=np.float32)
#        scaled_data.append(scaled_x)
#    return np.array(scaled_data,dtype=np.float32)



def write_data_to_file(file_path, data):
    csv.register_dialect('myDialect', lineterminator = '\n')    #Default is '\r\n' but insert one empty row in the file
    #x_row_dim = x_data.shape[0]
    with open(file_path, mode='a') as file:
        file_writer = csv.writer(file, dialect='myDialect')
        file_writer.writerow(data)


# Accelerometer and Gyroscope RAW data PLOT functions

def plot_data(data):
    x_axis = np.arange(0, data.shape[0], 1)
    plt.figure(1)
    plt.title('Test with 1 axis data')
    plt.xlabel('Sample# [100 samples/sec]')
    plt.ylabel('Acc 1-axis [mg]')
    plt.grid(True)
    plt.plot(x_axis,data)
    #plt.show()

def plot_ACC_3axis_data(data):
    x_axis = np.arange(0, data.shape[0], 1)
    plt.figure(2)
    plt.subplot(311)
    plt.title('Input ACC data - RAW')
    plt.ylabel('Acc x-axis [mg]')
    plt.grid(True)
    plt.plot(x_axis,data[:,0],'b')
    plt.subplot(312)
    plt.ylabel('Acc y-axis [mg]')
    plt.grid(True)
    plt.plot(x_axis,data[:,1],'r')
    plt.subplot(313)
    plt.xlabel('Sample# [100 samples/sec]')
    plt.ylabel('Acc z-axis [mg]')
    plt.grid(True)
    plt.plot(x_axis,data[:,2],'g')
    #plt.show()
    
def plot_GYR_3axis_data(data):
    x_axis = np.arange(0, data.shape[0], 1)
    plt.figure(3)
    plt.subplot(311)
    plt.title('Input GYR data - RAW')
    plt.ylabel('Gyr x-axis [mdps]')
    plt.grid(True)
    plt.plot(x_axis,data[:,0],'b')
    plt.subplot(312)
    plt.ylabel('Gyr y-axis [mdps]')
    plt.grid(True)
    plt.plot(x_axis,data[:,1],'r')
    plt.subplot(313)
    plt.xlabel('Sample# [100 samples/sec]')
    plt.ylabel('Gyr z-axis [mdps]')
    plt.grid(True)
    plt.plot(x_axis,data[:,2],'g')


def plot_ACC_GYR_6axis_data(data_acc, data_gyr, graph_title):
    x_axis = np.arange(0, data_acc.shape[0], 1)
    fig = plt.figure()
    plt.subplot(211)
    plt.title(graph_title)
    plt.ylabel('Acc [mg]')
    plt.grid(True)
    plt.plot(x_axis,data_acc[:,0],'b', label='Acc_x')
    plt.plot(x_axis,data_acc[:,1],'r', label='Acc_y')
    plt.plot(x_axis,data_acc[:,2],'g', label='Acc_z')
    plt.legend(prop={'size': 8})
    
    plt.subplot(212)
    plt.ylabel('Gyr [mdps]')
    plt.grid(True)
    plt.plot(x_axis,data_gyr[:,0],'b', label='Gyr_x')
    plt.plot(x_axis,data_gyr[:,1],'r', label='Gyr_y')
    plt.plot(x_axis,data_gyr[:,2],'g', label='Gyr_z')
    plt.legend(prop={'size': 8})

    #fig.savefig(file_name)
    return fig


def load_dataset(file_path):
    data = []
    i=0

    if sys.version_info[0] < 3:
        in_file = open (file_path, 'rb')
    else:
        in_file = open (file_path, 'r', newline='', encoding='utf8')
        
    with in_file as f:
        for row in f:
            data.append(np.array(row.strip().split(",")).astype(float))

    data = np.array(data,dtype=np.float32)
    return data
    
def one_hot(y_):
    """
    Function to encode output labels from number indexes.
    E.g.: [[5], [0], [3]] --> [[0, 0, 0, 0, 0, 1], [1, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0]]
    """
    y_ = y_.reshape(len(y_))
    n_values = int(np.max(y_)) + 1
    return np.eye(n_values)[np.array(y_, dtype=np.int32)]  # Returns FLOATS



    
def create_dataset(dir_path_log_orig, dir_path_log_proc, seq_length, n_axis):
    log_labels=np.zeros(10)

    dataset_file_list=get_logfile_list(dir_path_log_orig)
    print("List of log files in "+str(dir_path_log_orig) + ":")
    for item in dataset_file_list:
        print (item)

    for item in dataset_file_list:
        print("Processing log file:")
        print(str(dir_path_log_orig)+str(item))
        clean_file(str(dir_path_log_orig)+"/"+str(item), str(dir_path_log_proc)+"/"+str(item)+"_mod.csv")
        X_data, Y_data, plot_img = load_data_from_log(str(dir_path_log_proc)+"/"+str(item)+"_mod.csv",seq_length)
        #plot_img.savefig(dir_path_log_proc+"/Images/"+item+".png")
        plot_img.savefig(dir_path_log_proc+"/Images/"+"Label_"+str(Y_data[0])+"_"+item+".png")
        # Load each row of X matrix as:
        # [acc_x[1..128],acc_y[1..128], ..., gyr_z[128]]
        X=np.zeros((1,seq_length*n_axis))
        for i in range(seq_length):
            for k in range (n_axis):
                X[0,(k*seq_length)+i]=X_data[i,k]
        #print("X shape = ")
        #print(X.shape)
        write_data_to_file(str(dir_path_log_proc)+"/"+"dataset_x.csv", X[0,:])
        write_data_to_file(str(dir_path_log_proc)+"/"+"dataset_y.csv", Y_data)

        # Count how many logs for each label
        log_labels[int(Y_data[0])]+=1
        
        #X_data = scale_data(X_data)
        #plt.show()

    for i in range(10):
        print("Number of logs for:"+"Label_"+str(i)+" >> "+str(log_labels[i]))


def split_dataset(x, y, split=80, shuffle_opt=True):
    
    if shuffle_opt:
        #x, y = shuffle(x, y, random_state=0) # With random_state=0 it will shuffle with always same seed and results
        x, y = shuffle(x, y)

    split_index = int(math.floor(len(x) * (split/100)))

    x_train = x[:split_index]
    x_test = x[split_index:]

    y_train = y[:split_index]
    y_test = y[split_index:]

    test_labels=np.zeros(y_test.shape[0])
    for test_item in y_test:
        for j in range(10):
            if test_item[j]==1:
                #print(j)
                test_labels[j]+=1

    for i in range(10):
        print("Number of logs for test dataset:"+"Label_"+str(i)+" >> "+str(test_labels[i]))
			
    return x_train, x_test , y_train, y_test

def reshape_input_buffer(x_input,num_axis,seq_length):
    x = np.zeros((x_input.shape[0],x_input.shape[1]))
    #x_in = np.array(x_input)
    #print(x_input.shape[0])
    #print(seq_length)
    #print(num_axis)
    for i in range(0,x_input.shape[0]):
        for j in range(0,seq_length):
            for k in range(0,num_axis):
                #print("i=")
                #print(i)
                #print("(k*seq_length)+j-1 = ")
                #print((k*seq_length)+j)
                #print("(j*num_axis)+k = ")
                #print((j*num_axis)+k)
                #print(x_input[i][(k*seq_length)+j])
                x[i][(j*num_axis)+k] = x_input[i][(k*seq_length)+j]
    print("i=")
    print(i)
    print("(k*seq_length)+j-1 = ")
    print((k*seq_length)+j)
    print("(j*num_axis)+k = ")
    print((j*num_axis)+k)           
    return x
                
    
