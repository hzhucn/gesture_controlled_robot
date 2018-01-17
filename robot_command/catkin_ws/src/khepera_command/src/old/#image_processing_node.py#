import cv2
import rospy
from sklearn.externals import joblib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from toolbox.fourier_descriptors import truncate, normalize, get_descriptors
from toolbox.preprocess import threshold, get_contour

class ImageProcessor():
    
    def __init__(self):
        self.buffer= []
        self.max_buffer_size=5
        self.X = []
        self.kmax = 25
        self.clf_fnn = joblib.load('model_fnn.pkl')
        
    
    def on_msg(self,msg):
        if(len(self.buffer) >self.max_buffer_size):
            self.buffer =[]
            self.buffer.append(msg)
            
        elif (len(self.buffer) ==self.max_buffer_size):
            self.buffer.append(msg)

            for image in self.buffer:        
                img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                img_rgb = threshold(img_rgb)
                
                hand_contour = get_contour(img_rgb)    
                    
                fourier_descriptors,freqs = get_descriptors(hand_contour)
                
                selected_descriptors, selected_freqs = truncate(fourier_descriptors, freqs,self.kmax)
                
                normalized_descriptors = normalize(selected_descriptors, self.kmax)
                
                self.X.append(normalized_descriptors)
                self.predict()
        else:
            self.buffer.append(msg)
        
    def predict(self): 
        prediction = self.clf_fnn.predict(self.X)
        
        predicted_class_occurences = {'STOP':list(prediction).count('STOP'),
                                      'RIGHT': list(prediction).count('RIGHT'),
                                      'LEFT' : list(prediction).count('LEFT'), 
                                      'GO' : list(prediction).count('GO')}
        
        mostly_probable_class = max(predicted_class_occurences.keys(), 
                                    key=(lambda k: predicted_class_occurences[k]))
        
        self.publisher.publish(mostly_probable_class)
        rate.sleep()

if __name__ == '__main__':    
        
    image_processor = ImageProcessor()
    
    labels = []
    
    for k in range(image_processor.kmax-1): 
        labels.append("descr_number_{}_x".format(k))
        labels.append("descr_number_{}_y".format(k))
        
    publisher = rospy.Publisher('processing_result', String, queue_size=10)
    rospy.Subscriber("processing_result", Image, image_processor.on_msg)
    rospy.init_node('image_processing')
    
    
    rate = rospy.Rate(10) # 10hz
    rospy.spin()
        