#!/usr/bin/env python 
import rospy
import os
import cv2
import sys
import numpy as np
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities
from sklearn.externals import joblib

from script.fourier_descriptors import truncate, normalize, get_descriptors
from script.preprocess import threshold, get_contour
from script.command_from_video import create_twist_from_prediction

def is_hand(image,centroids):
    seuil = 3
    check_if_hand = False
    for center in centroids :
        distance = np.linalg.norm(center-image)
        if(distance<seuil):
            check_if_hand = True
    return check_if_hand

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

if __name__ == '__main__':
    rospy.init_node('video_prediction_node')
    r = rospy.Rate(3)
    publisher = rospy.Publisher("pioneer_p3dx/wheelsDriver", Twist, queue_size=100)
    
    cam_number = sys.argv[1] if len(sys.argv)>1 else 0
    
    cap = cv2.VideoCapture(int(cam_number))
    cv2.startWindowThread()
    
    ramp_frames = 10
    number_of_images = 7
    
    all_descriptors = []
    kmax = 25
    
    path_to_model = os.getcwd()+'/src/khepera_command/src/script/'
    clf_fnn = joblib.load(path_to_model+'model_fnn.pkl')
    clf_knn = joblib.load(path_to_model+'model_knn.pkl')
    clf_svm = joblib.load(path_to_model+'model_svm.pkl')
    models = [clf_fnn]
    
    classes = clf_fnn.classes_
    
    clf_kmeans = joblib.load(path_to_model+'model_kmeans.pkl')
    kmeans_centroids = clf_kmeans.cluster_centers_
    while not rospy.is_shutdown():
        if cap.isOpened() :
            for it in range(ramp_frames):
                if it%ramp_frames==1:
                    ret, frame = cap.read()
                    
                    # Pre processing of the image
                    processed_image = threshold(frame, 15)
                    
                    hand_contour,thresh = get_contour(processed_image)
                    image_plot = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                    cv2.drawContours(image_plot, [hand_contour], -1, (0,0,255), 3)
                    np.concatenate((image_plot), axis=1)
                    # Plot the processed image in real time
                    cv2.imshow('Processed Image', image_plot)
                    
                    if len(all_descriptors)!=number_of_images:
                        
                        fourier_descriptors,freqs = get_descriptors(hand_contour)
                        selected_descriptors, selected_freqs = truncate(fourier_descriptors, freqs, kmax)
                        normalized_descriptors = normalize(selected_descriptors, kmax)
                        
                        if(is_hand(normalized_descriptors, kmeans_centroids)):
                            all_descriptors.append(abs(normalized_descriptors))
                        
                    
                    if(len(all_descriptors)==number_of_images):
                        prediction = batch_prediction(all_descriptors, models)
                        print(prediction)
                        
                        twist = create_twist_from_prediction(prediction) 
                        publisher.publish(twist)
                        all_descriptors = []
                        r.sleep()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
