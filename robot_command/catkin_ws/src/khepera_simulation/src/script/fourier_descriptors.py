import cv2
import cmath
import math
import numpy as np

def get_descriptors(contour):
    '''Get fourier descriptors from contour 
        Keyword arguments: 
         - contour : the contour of the object
        return descriptors  , freqs
    '''
    #Passage en complexes
    contour_array = contour[:, 0, :]
    contour_complex = np.empty(contour_array.shape[:-1], dtype=complex)
    contour_complex.real = contour_array[:, 0]
    contour_complex.imag = contour_array[:, 1]

    # Soustraction de la moyenne 
    contour_complex_centre = np.empty(contour_array.shape[:-1], dtype=complex)
    mean_real = np.mean(contour_complex.real)
    mean_imag = np.mean(contour_complex.imag)
    
    contour_complex_centre.real = contour_complex.real - mean_real 
    contour_complex_centre.imag = contour_complex.imag - mean_imag 


    freqs = np.fft.fftfreq(contour_complex_centre.real.size)
    # calcul de la fft 
    descriptors = np.fft.fft(contour_complex_centre)

    return descriptors, freqs

def truncate(descriptors, freqs, kmax):
    '''Function to truncate fourier descriptors between [-kmax, kmax]
    
        Keyword arguments: 
         - descriptors : the fourier descriptors given by fft
         - kmax : half the maximum number of descriptors you want 
    
        return selected_descriptors  , selected_freqs
    '''
    # Truncate between 0 and k 
    freqs_pos = [x for x in freqs if x>0]
    freqs_neg = [x for x in freqs if x<0]
    nb_tot_pos = len(freqs_pos)
    if kmax > len(freqs_neg):
        # TODO : Definir ce qu'on doit faire p.e mettre des zeros, ... etc
        
        #Contient pour le moment les descripteurs de fourier positifs 
        #on concatenera avec les descripteurs negatifs
        pos_descriptors = descriptors.copy()[:kmax+1]
        neg_descriptors = list(reversed(descriptors.copy()[nb_tot_pos:][:-kmax-1:-1]))
        
        selected_descriptors = np.concatenate((pos_descriptors,neg_descriptors))
        freq_pos = freqs.copy()[:kmax+1]
        freqs_neg = list(reversed(freqs.copy()[nb_tot_pos:][:-kmax-1:-1]))
        selected_freqs = np.concatenate((freq_pos, freqs_neg))

    else:
        #Contient pour le moment les descripteurs de fourier positifs 
        #on concatenera avec les descripteurs negatifs
        pos_descriptors = descriptors.copy()[:kmax+1]
        neg_descriptors = list(reversed(descriptors.copy()[nb_tot_pos:][:-kmax-1:-1]))
        
        selected_descriptors = np.concatenate((pos_descriptors,neg_descriptors))
        freq_pos = freqs.copy()[:kmax+1]
        freqs_neg = list(reversed(freqs.copy()[nb_tot_pos:][:-kmax-1:-1]))
        selected_freqs = np.concatenate((freq_pos, freqs_neg))

    return selected_descriptors, selected_freqs

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
    # On commence a 1 car on ne compte pas a0
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
    for i in range(int(math.ceil(len(descriptors)/2))):
        descriptors[i] = descriptors[i]*cmath.exp(-1j*theta*i)    
    for i in range(int(math.ceil(len(descriptors)/2)),len(descriptors)):
        k = -(len(descriptors)-i)
        descriptors[i] = descriptors[i]*cmath.exp(-1j*theta*k)    
    return descriptors

def reconstruct(descriptors):
    """ reconstruct the image using the normalized descriptors and plot it
        Keyword arguments: 
         - descriptors : the normalized fourier descriptors
    """
    contour_reconstruct = np.fft.ifft(descriptors)
    contour_reconstruct = np.array(
        [contour_reconstruct.real, contour_reconstruct.imag])
    contour_reconstruct = np.transpose(contour_reconstruct)
    contour_reconstruct = np.expand_dims(contour_reconstruct, axis=1)
    # make positive
    if contour_reconstruct.min() < 0:
        contour_reconstruct -= contour_reconstruct.min()
    # normalization
    contour_reconstruct *= 800 / contour_reconstruct.max()
    # type cast to int32
    contour_reconstruct = contour_reconstruct.astype(np.int32, copy=False)
    black = np.zeros((800, 800), np.uint8)
    # draw and visualize
    cv2.drawContours(black, contour_reconstruct, -1, 255, thickness=-1)
    cv2.imshow("black", black)
    cv2.waitKey(10000)
    cv2.imwrite("reconstruct_result.jpg", black)
    cv2.destroyAllWindows()
