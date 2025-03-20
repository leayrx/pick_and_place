# importing cv2 
#import cv2
#import numpy as np
#import matplotlib.pyplot as plt 
  
# path 
#path = '/home/rosdev/arouco.png'
  
# Reading an image in default mode
#image = cv2.imread(path)
#print(image.shape)#donne les coordonnees de la matrice

#arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
#arucoParams = cv2.aruco.DetectorParameters_create()
#(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

#print (corners)
#print(ids)
#print(rejected)

#def calculer_barycentre(corners):
#    barycentres = []
#    for corner in corners:
#        # Convertir les coins en coordonnées 2D
#        points = corner.reshape(-1, 2)  # reshape pour obtenir une liste de points (x, y)
#        
        # Calcul du barycentre pour ce marqueur
#        Gx = np.mean(points[:, 0])  # Moyenne des x
#        Gy = np.mean(points[:, 1])  # Moyenne des y
#        barycentres.append((Gx, Gy))
    
#    return barycentres

# Calcul du barycentre pour chaque marqueur détecté
#if len(corners) > 0:
#    barycentres = calculer_barycentre(corners)
#    for i, (Gx, Gy) in enumerate(barycentres):
#        print(f"Barycentre du marqueur {ids[i][0]} : ({Gx}, {Gy})")

# Affichage de l'image avec les barycentres marqués
#for i, (corner, (Gx, Gy)) in enumerate(zip(corners, barycentres)):
#    # Dessiner un cercle au barycentre
#    cv2.circle(image, (int(Gx), int(Gy)), 5, (0, 0, 255), -1)
#    # Afficher l'ID du marqueur et le barycentre
#    cv2.putText(image, f"ID: {ids[i][0]}", (int(Gx), int(Gy)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

# Afficher l'image avec les marqueurs et les barycentres
#cv2.imshow("Image avec barycentres", image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

# Taking a matrix of size 5 as the kernel
#kernel = np.ones((5,5), np.uint8)

# find frequency of pixels in range 0-255
#histr = cv2.calcHist([image],[0],None,[256],[0,256])

#half = cv2.resize(image, (0, 0), fx = 0.1, fy = 0.1)
#bigger = cv2.resize(image, (1050, 1610))
 
#stretch_near = cv2.resize(image, (780, 540),
#               interpolation = cv2.INTER_NEAREST)

#Titles =["Original", "Half", "Bigger", "Interpolation Nearest"]
#images =[image, half, bigger, stretch_near]
#count = 4
 
#for i in range(count):
#    plt.subplot(2, 2, i + 1)
#    plt.title(Titles[i])
#    plt.imshow(images[i])
 
#plt.show()
  
# Window name in which image is displayed
#window_name = 'visu_camera'
#img_erosion = cv2.erode(image, kernel, iterations=1)
#img_dilation = cv2.dilate(image, kernel, iterations=1)

# Using cv2.imshow() method 
# Displaying the image 
#cv2.imshow(window_name, image)
#cv2.imshow('Input', image)
#cv2.imshow('Erosion', img_erosion)
#cv2.imshow('Dilation', img_dilation)
  
#waits for user to press any key 
#(this is necessary to avoid Python kernel form crashing)
#cv2.waitKey(0) 

# Using cv2.imwrite() method
# Saving the image
##cv2.imwrite('~/arouco1.png', image)
  
#closing all open windows 
##cv2.destroyAllWindows() 

# show the plotting graph of an image
#plt.plot(histr)
#plt.show()
