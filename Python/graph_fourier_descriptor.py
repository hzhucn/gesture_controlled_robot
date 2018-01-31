import pandas 
import cv2
from matplotlib.pyplot import plt
from toolbox.fourier_descriptor import reconstruct

k = 25

dataset = pandas.read_csv('../dataset_norm.csv', delimiter=',').sort_values(by='move_type')
columns = [item for item in list(dataset.columns) if item!='move_type']
features = dataset[columns]

drawing = np.zeros(img_bgr.shape,np.uint8)

for hand_contour in features.values:    
    cv2.drawContours(drawing,[hand_contour],0,(0,255,0),2)
    
plt.imshow(drawing)
plt.show()