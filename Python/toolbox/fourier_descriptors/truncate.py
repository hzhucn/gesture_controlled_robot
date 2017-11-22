import numpy as np
import matplotlib.pyplot as plt
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
        # TODO : Définir ce qu'on doit faire p.e mettre des zéros, ... etc
        
        #Contient pour le moment les descripteurs de fourier positifs 
        #on concatènera avec les descripteurs négatifs
        pos_descriptors = descriptors.copy()[:kmax+1]
        neg_descriptors = list(reversed(descriptors.copy()[nb_tot_pos:][:-kmax-1:-1]))
        
        selected_descriptors = np.concatenate((pos_descriptors,neg_descriptors))
        freq_pos = freqs.copy()[:kmax+1]
        freqs_neg = list(reversed(freqs.copy()[nb_tot_pos:][:-kmax-1:-1]))
        selected_freqs = np.concatenate((freq_pos, freqs_neg))

    else:
        #Contient pour le moment les descripteurs de fourier positifs 
        #on concatènera avec les descripteurs négatifs
        pos_descriptors = descriptors.copy()[:kmax+1]
        neg_descriptors = list(reversed(descriptors.copy()[nb_tot_pos:][:-kmax-1:-1]))
        
        selected_descriptors = np.concatenate((pos_descriptors,neg_descriptors))
        freq_pos = freqs.copy()[:kmax+1]
        freqs_neg = list(reversed(freqs.copy()[nb_tot_pos:][:-kmax-1:-1]))
        selected_freqs = np.concatenate((freq_pos, freqs_neg))

    return selected_descriptors, selected_freqs
