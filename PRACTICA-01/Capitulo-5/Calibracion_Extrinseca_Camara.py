import cv2
import cv2.aruco as aruco
import numpy as np
import Funciones_Codigos as fc

# ------ Parámetros generales ------
tamaño_aruco = 0.143  # en metros
coordenadas_world = {
    1: np.array([0.0, 0.0, 0.0]),   # Esquina inferior izq.
    2: np.array([2.554, 0.0, 0.0]), # Esquina inferior der.
    3: np.array([0.0, 1.803, 0]),   # Esquina superior izq.
    5: np.array([2.554, 1.803, 0])  # Esquina superior der.
}

# ------ Configuración códigos ArUco ------
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ------ Obtención de los parámetros de la cámara ------
parametros_camara = fc.obtener_Parametros_Camara()

def obtener_imagen_camara(camara):
    ret, imagen = camara.read()
    if not ret:
        print("No se pudo tomar una imagen.")
        exit()
    return imagen

def detectar_codigos(imagen):
    obj_points = []  # Puntos 3D conocidos
    img_points = []  # Puntos 2D detectados en la imagen

    corners, ids, _ = detector.detectMarkers(imagen)
    if ids is None:
        print("No se detectaron ArUcos en la imagen.")
        exit()

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in coordenadas_world:
            c = corners[i][0]
            centro = c.mean(axis=0)
            img_points.append(centro)
            obj_points.append(coordenadas_world[marker_id])

    obj_points = np.array(obj_points, dtype=np.float32)
    img_points = np.array(img_points, dtype=np.float32)

    return obj_points, img_points

def obtener_rotacion_traslacion(obj_points, img_points, parametros_camara):
    _, rvec, tvec = cv2.solvePnP(
        obj_points, img_points,
        np.array(parametros_camara["matriz_camara"]),
        np.array(parametros_camara["coef_dist"])
    )

    R, _ = cv2.Rodrigues(rvec)
    print("Matriz de rotación (R):\n", R)
    print("Vector de traslación (t):\n", tvec)

    fc.guardar_Parametros_Extrinsecos_Camara(R, tvec)
    print("Se guardaron la R y t en el archivo json")

    return rvec, tvec

def dibujar_ejes(imagen, rvec, tvec, img_points, parametros_camara):
    axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1]])
    imgpts, _ = cv2.projectPoints(
        axis, rvec, tvec,
        np.array(parametros_camara["matriz_camara"]),
        np.array(parametros_camara["coef_dist"])
    )

    corner = tuple(img_points[0].astype(int))  # primer punto detectado
    imagen = cv2.line(imagen, corner, tuple(imgpts[0].ravel().astype(int)), (0,0,255), 3) # X
    imagen = cv2.line(imagen, corner, tuple(imgpts[1].ravel().astype(int)), (0,255,0), 3) # Y
    imagen = cv2.line(imagen, corner, tuple(imgpts[2].ravel().astype(int)), (255,0,0), 3) # Z

    cv2.imshow("Extrinseca con 4 ArUco", imagen)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# --- Captura de imagen y procesamiento ---
camara = cv2.VideoCapture(1)
camara.set(cv2.CAP_PROP_FRAME_HEIGHT, parametros_camara["resolucion_height"])
camara.set(cv2.CAP_PROP_FRAME_WIDTH, parametros_camara["resolucion_width"])

imagen = obtener_imagen_camara(camara)

obj_points, img_points = detectar_codigos(imagen)

# --- Obtenemos la rotación y traslación ---
rvec, tvec = obtener_rotacion_traslacion(obj_points, img_points, parametros_camara)

# --- Dibujar ejes en un ArUco de referencia ---
dibujar_ejes(imagen, rvec, tvec, img_points, parametros_camara)

camara.release()
cv2.destroyAllWindows()
