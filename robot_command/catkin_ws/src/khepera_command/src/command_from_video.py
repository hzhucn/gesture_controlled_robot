#!/usr/bin/env python3

import cv2
import numpy as np
from toolbox.fourier_descriptors import truncate, normalize, get_descriptors
from toolbox.preprocess import threshold, get_contour
from sklearn.externals import joblib

def batch_prediction(number_of_images): 
    kmax = 25
    all_descriptors = []
    cap = cv2.VideoCapture(0)

    clf_fnn = joblib.load('model_fnn.pkl')
    clf_knn = joblib.load('model_fnn.pkl')
    classes = clf_fnn.classes_
        
    for i in range(7):
        # Capture frame-by-frame
        
        ret, img_bgr = cap.read()
        cv2.imshow('Color Frame'+str(i), img_bgr)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_rgb = threshold(img_rgb)
        
        hand_contour = get_contour(img_rgb)    
            
        fourier_descriptors,freqs = get_descriptors(hand_contour)
        
        selected_descriptors, selected_freqs = truncate(fourier_descriptors, freqs, kmax)
        
        normalized_descriptors = normalize(selected_descriptors, kmax)
        
        all_descriptors.append(abs(normalized_descriptors))
    
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
    prediction_fnn = clf_fnn.predict_proba(all_descriptors)
    prediction_knn = clf_knn.predict_proba(all_descriptors)
    
    prediction_proba = (prediction_fnn + prediction_knn)/2
    prediction = [ classes[np.argmax(x)] for x in prediction_proba]
    
    predicted_class_occurences = {'STOP':list(prediction).count('STOP'),
                                  'RIGHT': list(prediction).count('RIGHT'),
                                  'LEFT' : list(prediction).count('LEFT'), 
                                  'GO' : list(prediction).count('GO')}
    
    most_probable_class = max(predicted_class_occurences.keys(), 
                                key=(lambda k: predicted_class_occurences[k]))
    
    return most_probable_class