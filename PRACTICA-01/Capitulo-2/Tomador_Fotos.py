import cv2
import Funciones_Codigos as fc

# Se especifica la resolucion de la camara (se recomineda el valor de 1280x720)
resolucion_width = 1280
resolucion_height = 720
fc.guardar_Resolucion_Camara(resolucion_width, resolucion_height)

# Se crea la carpeta para guardar las fotos del tablero de ajedrez,
# se guardara en la direccion donde esta guardado este código
# (Por default el nombre de la carpeta es: 'Foto_Calibracion').
fc.crear_Carpeta_Fotos()

# Iniciamos la camara y ajustamos la resolución.
camara_web = cv2.VideoCapture(1)
camara_web.set(cv2.CAP_PROP_FRAME_WIDTH, resolucion_width)
camara_web.set(cv2.CAP_PROP_FRAME_HEIGHT, resolucion_height)

# Se verifica si se abrio la camara.
if not camara_web.isOpened():
    print("No se puede abrir la cámara")
    exit()

contador_fotos = 0

print("Presiona 'SPACE' para tomar una foto, 'ESC' para salir.")
while True:
    ret, imagen = camara_web.read()

    if not ret:
        print("No se puede recibir imagen. Saliendo ...")
        break

    # Mostrar el video en una ventana
    cv2.imshow('Presiona SPACE para tomar foto', imagen)
    key = cv2.waitKey(1)
    
    if key == 27:  # ESC para salir
        break
    elif key == 32:  # SPACE para tomar foto
        fc.guardar_Foto_Calibracion(imagen, contador_fotos)
        contador_fotos += 1

# Liberar la cámara y cerrar ventanas
camara_web.release()
cv2.destroyAllWindows()

print(f"Se guardaron {contador_fotos} fotos en la carpeta: {fc.obtener_Nombre_Carpeta_Fotos()} con una resolucion de {resolucion_width}x{resolucion_height}.")