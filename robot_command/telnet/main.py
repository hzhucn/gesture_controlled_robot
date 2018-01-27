#!/usr/bin/env python 
import sys
import os
import cv2
import telnetlib
import numpy as np

from sklearn.externals import joblib
from toolbox.fourier_descriptors import truncate, normalize, get_descriptors
from toolbox.preprocess import threshold, get_contour

def batch_prediction(all_descriptors, models): 
    
    most_probable_class = None
    classes = models[0].classes_
    predictions = []
    for clf in models: 
        predictions.append(clf.predict_proba(all_descriptors))
    
    prediction_proba = sum(predictions) / len(models)
    prediction = [ classes[np.argmax(x)] for x in prediction_proba]
    
    predicted_class_occurences = {'STOP':list(prediction).count('STOP'),
                                  'RIGHT': list(prediction).count('RIGHT'),
                                  'LEFT' : list(prediction).count('LEFT'), 
                                  'GO' : list(prediction).count('GO')}
    
    most_probable_class = max(predicted_class_occurences.keys(), 
                              key=(lambda k: predicted_class_occurences[k]))
    return most_probable_class

def create_command_from_prediction(prediction):
    command = None
    if(prediction ==  "RIGHT") :
        command = "D,0,100"
    elif(prediction == "LEFT"): 
        command = "D,100,0"
    elif(prediction =="STOP"):
        command = "D,0,0"
    elif(prediction =="GO"):
        command = "D,100,100"  
    
    return command

def is_hand(image,centroids):
    seuil = 3
    check_if_hand = False
    for center in centroids :
        distance = np.linalg.norm(center-image)
        if(distance<seuil):
            check_if_hand = True
    return check_if_hand

class telnetlib_connection:
    
    def __init__(self,ip,port):
        try : 
            self.tn = telnetlib.Telnet(ip, port)
        except Exception as e:
            print("Impossible de se connecter: {}".format(e))
            sys.exit()
            
    def write_prediction(self,prediction):
        command = create_command_from_prediction(prediction)
        try:
            self.tn.write(command.encode('ascii')+ b"\r")
            output = self.tn.read_some()
            print("command received : {}".format(output))
        except Exception as e:
            print("Impossible d'envoyer un message: {}".format(e))
            self.tn.close()

if __name__ == '__main__':
    # Init the connection on khepera
    ip, port = 'khepera2.smart.metz.supelec.fr', 4100
    tn = telnetlib_connection(ip,port)    
    
    cam_number = sys.argv[1] if len(sys.argv)>1 else 0
    
    cap = cv2.VideoCapture(int(cam_number))
    cv2.startWindowThread()
    
    ramp_frames = 10
    number_of_images = 7
    
    all_descriptors = []
    kmax = 25
    
    path_to_model = os.getcwd()+'/toolbox/'
    clf_fnn = joblib.load(path_to_model+'model_fnn.pkl')
    clf_knn = joblib.load(path_to_model+'model_knn.pkl')
    clf_svm = joblib.load(path_to_model+'model_svm.pkl')
    models = [clf_fnn]
    classes = clf_fnn.classes_
    
    clf_kmeans = joblib.load(path_to_model+'model_kmeans.pkl')
    kmeans_centroids = clf_kmeans.cluster_centers_

    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cap.isOpened() :
            for it in range(ramp_frames):
                if it%ramp_frames==1:
                    ret, frame = cap.read()
                    
                    # Pre processing of the image
                    processed_image = threshold(frame, 15)
                    
                    hand_contour,thresh = get_contour(processed_image)
                    image_plot = thresh.copy()
                    cv2.drawContours(image_plot, [hand_contour], -1, (0,255,0), 3)
                    # Plot the processed image in real time
                    cv2.imshow('Processed Image', image_plot)
                    
                    while len(all_descriptors)!=number_of_images:
                        
                        fourier_descriptors,freqs = get_descriptors(hand_contour)
                        selected_descriptors, selected_freqs = truncate(fourier_descriptors, freqs, kmax)
                        normalized_descriptors = normalize(selected_descriptors, kmax)
                        
                        if(is_hand(normalized_descriptors, kmeans_centroids)):
                            all_descriptors.append(abs(normalized_descriptors))
                        
                    
                    if(len(all_descriptors)==number_of_images):
                        prediction = batch_prediction(all_descriptors, models)
                        print(prediction)
                        tn.write_prediction(prediction)
                        all_descriptors = []
            
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
