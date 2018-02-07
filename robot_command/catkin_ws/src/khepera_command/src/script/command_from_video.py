#!/usr/bin/env python2
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities
import numpy as np


def batch_prediction(all_descriptors, models): 
    
    most_probable_class = None
    classes = models[0].classes_
    predictions = []
    for clf in models: 
        predictions.append(clf.predict_proba(all_descriptors))
    
    prediction_proba = sum(predictions) / 3
    prediction = [ classes[np.argmax(x)] for x in prediction_proba]
    
    predicted_class_occurences = {'STOP':list(prediction).count('STOP'),
                                  'RIGHT': list(prediction).count('RIGHT'),
                                  'LEFT' : list(prediction).count('LEFT'), 
                                  'GO' : list(prediction).count('GO')}
    
    most_probable_class = max(predicted_class_occurences.keys(), 
                              key=(lambda k: predicted_class_occurences[k]))
    return most_probable_class

def create_twist_from_prediction(prediction):
    twist = Twist()
    if prediction =='RIGHT':
         twist.linear.x = 0
         twist.linear.y = 0
         twist.linear.z = 0
         twist.angular.x = 0
         twist.angular.y = 0
         twist.angular.z = -0.6
         
    elif prediction == 'LEFT':
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0.6
        
    elif prediction == 'GO':
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
    
    elif prediction == 'STOP':
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
    return twist
