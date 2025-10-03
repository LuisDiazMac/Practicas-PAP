import cv2
import numpy as np
import Funciones_Codigos as fc


tamaño_patron = (7, 7) # Tamaño del patrón: número de esquinas internas (no cuadros)
longitud_cuadro = 0.024  # Tamaño real de cada cuadrado (en metros)

# Preparar coordenadas del patrón (puntos 3D en el mundo real)
objp = np.zeros((tamaño_patron[0] * tamaño_patron[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:tamaño_patron[0], 0:tamaño_patron[1]].T.reshape(-1, 2)
objp *= longitud_cuadro  # Escalar según el tamaño real de los cuadros

objpoints = []  # Puntos 3D en el mundo real
imgpoints = []  # Puntos 2D detectados en la imagen

# Filtrar archivos .jpg dentro de esa carpeta
imagenes = fc.filtrar_Archivos_JPG()

for fname in imagenes:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Encontrar esquinas del tablero
    ret, corners = cv2.findChessboardCorners(gray, tamaño_patron, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Dibujar las esquinas encontradas
        cv2.drawChessboardCorners(img, tamaño_patron, corners, ret)
        cv2.imshow('Detección', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Calibrar la cámara
ret, matriz_camara, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None )

print(f"Matriz de cámara (intrínseca): \n {matriz_camara} \n")
print(f"Coeficientes de distorsión:\n {dist_coeffs}")
print(f"Reporjection error: {ret}")

fc.guardar_Parametros_Camara(matriz_camara, dist_coeffs)
print("Se guardaron los parametros en el archivo json")
