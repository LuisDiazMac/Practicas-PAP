import cv2.aruco as aruco
import matplotlib.pyplot as plt
import Funciones_Codigos as fc

# Seleccionar el diccionario de marcadores de ArUco
# DICT_4x4_50, DICT_5x5_100, DICT_6x6_250, DICT_7x7_1000, DICT_ARUCO_ORIGINAL -> 1024
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

id_codigos = [1, 2, 3, 4, 5]   # ID de los códigos ArUco
tamaño_codigo = 300             # tamaño en pixeles

for x in id_codigos:
    imagen_codigo = aruco.generateImageMarker(aruco_dict, x, tamaño_codigo)

    # Muestra el marcador con matplotlib
    plt.imshow(imagen_codigo, cmap='gray')
    plt.axis('off')
    plt.title(f'ArUco ID {x}')
    plt.show()

    # Guardar el marcador como imagen PNG
    fc.guardar_Codigo_ArUco(imagen_codigo, x)