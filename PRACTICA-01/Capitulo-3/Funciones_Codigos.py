import cv2
import cv2.aruco as aruco
import pandas as pd
import numpy as np
import json
import os
from datetime import date

ARCHIVO_PARAMETROS_CAMARA = "parametros_camara.json"
ARCHIVO_VALORES_CSV ="valores.csv"
CARPETA_FOTOS_CALIBRACION = "Fotos_Calibracion"
CARPETA_CODIGOS_ARUCO = "Codigos_ArUco"
RESOLUCION_CAMARA = {
    "RESOLUCION_WIDTH": 1280,
    "RESOLUCION_HEIGHT": 720
}

def filtrar_Archivos_JPG():
    dir_carpeta_imagenes = juntar_DireccionAbs_Archivo(CARPETA_FOTOS_CALIBRACION)
    imagenes = [os.path.join(dir_carpeta_imagenes, f)
                for f in os.listdir(dir_carpeta_imagenes)
                if f.lower().endswith('.jpg')]
    
    return imagenes

def guardar_Resolucion_Camara(width, height):
    RESOLUCION_CAMARA["RESOLUCION_WIDTH"] = width
    RESOLUCION_CAMARA["RESOLUCION_HEIGHT"] = height

def obtener_Nombre_Carpeta_Fotos():
    return CARPETA_FOTOS_CALIBRACION

def crear_Carpeta_Fotos():
    crear_Carpeta(CARPETA_FOTOS_CALIBRACION)

def crear_Carpeta_ArUco():
    crear_Carpeta(CARPETA_CODIGOS_ARUCO)

def crear_Carpeta(nombre_carpeta):
    direccion_carpeta = juntar_DireccionAbs_Archivo(nombre_carpeta)
    os.makedirs(direccion_carpeta, exist_ok=True)
    
    if os.path.exists(direccion_carpeta):
        print(f"Se creo la carpeta '{nombre_carpeta}'")
    else:
        print(f"Error al crear la carpeta '{nombre_carpeta}'")

def guardar_Foto_Calibracion(imagen, contador):
    nombre_foto = f"foto_ajedrez_{contador:02}.jpg"
    
    guardar_Imagen(CARPETA_FOTOS_CALIBRACION, nombre_foto, imagen)
    
    print(f"Foto guardada como {nombre_foto}")

def guardar_Codigo_ArUco(imagen_codigo, id):
    nombre_aruco = f"Codigo_Aruco_{id:02}.jpg"
    
    guardar_Imagen(CARPETA_CODIGOS_ARUCO, nombre_aruco, imagen_codigo)

    print(f"Se creo el código ArUco con id: {id}")

def guardar_Imagen(direccion_carpeta, nombre_archivo, imagen):
    direccion_nombre = juntar_Direccion1_Direccion2(
        juntar_DireccionAbs_Archivo(direccion_carpeta), nombre_archivo 
        )
    
    cv2.imwrite(direccion_nombre, imagen)

def guardar_Parametros_Camara(matriz_camara, coeficientes_distorsion):
    
    estructura_datos = {
        "fecha_calibracion" : str(date.today()),
        "resolucion_width" : int(RESOLUCION_CAMARA["RESOLUCION_WIDTH"]),
        "resolucion_height" : int(RESOLUCION_CAMARA["RESOLUCION_HEIGHT"]),
        "matriz_camara" : matriz_camara.tolist(),
        "coef_dist" : coeficientes_distorsion.tolist()
    }

    direccion = juntar_DireccionAbs_Archivo(ARCHIVO_PARAMETROS_CAMARA)

    if os.path.exists(direccion):
        datos = obtener_Parametros_Camara()
        datos.update(estructura_datos)
    else:
        datos = estructura_datos

    with open(direccion, "w") as f:
        json.dump(datos, f, indent=4) 

def guardar_Parametros_Extrinsecos_Camara(matriz_rotacion, vector_traslancion):
    estructura_datos = {
        "matriz_rotacion" : matriz_rotacion.tolist(),
        "vector_traslacion" : vector_traslancion.tolist()
    }

    parametros_camara = obtener_Parametros_Camara()
    parametros_camara.update(estructura_datos)

    direccion = juntar_DireccionAbs_Archivo(ARCHIVO_PARAMETROS_CAMARA)
    
    with open(direccion, "w") as f:
        json.dump(parametros_camara, f, indent = 4)

def obtener_Parametros_Camara():
    direccion = juntar_DireccionAbs_Archivo(ARCHIVO_PARAMETROS_CAMARA)

    if not os.path.exists(direccion):
        raise FileNotFoundError(f"No se encontró el archivo de parámetros: {direccion}")

    with open(direccion, "r") as f:
        parametros_camara = json.load(f)

    return parametros_camara

def guardar_Datos_CSV(datos):
    if not datos:
        raise ValueError(f"Los datos proporcionados para guardar en CSV están vacíos.")

    direccion = juntar_DireccionAbs_Archivo(ARCHIVO_VALORES_CSV)
    df = pd.DataFrame(datos)
    df.to_csv(direccion, index = False)

def sacar_Datos_CSV():
    direccion = juntar_DireccionAbs_Archivo(ARCHIVO_VALORES_CSV)

    if not os.path.exists(direccion):
        raise FileNotFoundError(f"No se encontró el archivo CSV: {direccion}")
    
    return pd.read_csv(direccion)

def obtener_Direccion_Carpeta_Absoluta():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return os.getcwd()

def juntar_DireccionAbs_Archivo(nombre_archivo):
    return os.path.join(obtener_Direccion_Carpeta_Absoluta(), nombre_archivo)

def juntar_Direccion1_Direccion2(direccion_carpeta, nombre_archivo):
    return os.path.join(direccion_carpeta, nombre_archivo)