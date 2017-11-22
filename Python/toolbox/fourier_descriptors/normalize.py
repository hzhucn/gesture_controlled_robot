import numpy as np
import cmath
import math

def normalize(descriptors, kmax):
    '''Normalize the fourier descriptors according to 3 rules : 
        - Invariance to the direction of the contour path (I1) 
        - Invariance of scale (I2)
        - Invariance with rotation  (I3)
        
        Keyword arguments: 
         - descriptors : the truncated descriptors 
         - kmax : half the maximum number of descriptors you want 
        
        Return normalised coefficients 
    '''
    ### I1 : Invariance to the direction of the contour path
    nb_of_descriptors = len(descriptors)
    if nb_of_descriptors < 2*kmax+1:
        kmax = math.floor(nb_of_descriptors/2)
    # On commence à 1 car on ne compte pas a0
    #len(descriptors)-1 correspond au dernier element de la liste 

    if(np.abs(descriptors[1])< np.abs(descriptors[-1])):
        for i in range(kmax):        
            # for real part 
            desc_temp_real = descriptors.real[i+1]
            descriptors.real[i+1] = descriptors.real[-1-i]
            descriptors.real[nb_of_descriptors-1-i] = desc_temp_real 
            
            #for imaginary part
            desc_temp_imag = descriptors.imag[i+1]
            descriptors.imag[i+1] = descriptors.imag[-1-i]
            descriptors.imag[nb_of_descriptors-1-i] = desc_temp_imag
    
    ### I2 : Invariance of scale
    d1 = np.abs(descriptors[1])
    descriptors.real = descriptors.real/d1
    descriptors.imag = descriptors.imag/d1
    
    ### I3 : Invariance with rotation 
    phi = np.angle(descriptors[-1]*descriptors[1])/2
    descriptors = descriptors*cmath.exp(-1j*phi)
    theta = np.angle(descriptors[-1])
    for i in range(math.ceil(len(descriptors)/2)):
        descriptors[i] = descriptors[i]*cmath.exp(-1j*theta*i)    
    for i in range(math.ceil(len(descriptors)/2),len(descriptors)):
        k = -(len(descriptors)-i)
        descriptors[i] = descriptors[i]*cmath.exp(-1j*theta*k)    
    return descriptors
