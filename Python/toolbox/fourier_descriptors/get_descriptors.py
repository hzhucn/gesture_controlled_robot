import numpy as np
import matplotlib.pyplot as plt
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
