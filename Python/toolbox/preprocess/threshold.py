def threshold(img_rgb, seuil=None):
    if seuil == None :
        seuil=15
    for i in range(len(img_rgb)):
        for j in range(len(img_rgb[i])):
            r = img_rgb[i][j][0]
            g = img_rgb[i][j][1]
            b = img_rgb[i][j][2]
            if (max(r,g,b)- min(r,g,b) < seuil) or (max(r,g,b)!=r):
                img_rgb[i][j]=[0,0,0]
    return img_rgb