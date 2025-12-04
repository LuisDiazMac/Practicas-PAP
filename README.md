# 🤖 Practicas-PAP: RobotMeshA - Sistema Autónomo con Percepción Dual

## 📝 Resumen del Proyecto PAP (Otoño 2025)

Este repositorio contiene la totalidad del código, la documentación técnica y los logs de validación del Proyecto de Aplicación Profesional (PAP) **RobotMeshA**. El proyecto se centra en el diseño e implementación de un vehículo móvil omnidireccional dotado de un sistema de **Percepción Dual** que fusiona la **Visión Artificial (OpenCV/ArUco)** para la navegación métrica precisa, con un **sensor LiDAR (RPLIDAR C1)** para la detección de obstáculos y la seguridad operacional.

El principal hito técnico fue la implementación de un **Control Jerárquico** donde la seguridad (detección LiDAR) anula la navegación (Control PI), logrando un sistema autónomo robusto y certificado para operar en entornos dinámicos compartidos.

---

## 🎯 Objetivos y Criterios de Aceptación

El proyecto se enfocó en resolver la problemática de la **imprecisión métrica** y el **riesgo operacional por latencia** en prototipos experimentales.

| Objetivo Táctico Clave | Tecnología Implementada | Criterio de Aceptación (CA) | Prácticas Involucradas |
| :--- | :--- | :--- | :--- |
| **Navegación Precisa** | Control PI de lazo cerrado y Calibración Métrica ArUco. | Lograr la capacidad de posicionamiento estable y métrico mediante retroalimentación de la cámara. | P1, P2 |
| **Seguridad Operacional** | Validación de $T_{reac}$ y Máquina de Estados Finita (FSM). | Certificar que el tiempo de reacción ante un obstáculo sea **$T_{reac} < 200$ ms**. | P3, P4 |
| **Integración Final** | Control Jerárquico (LiDAR > ArUco). | Fusión estable de ambos subsistemas, validando la anulación de la navegación por la señal de seguridad. | P5 |

---

## 🛠️ Tecnologías y Hardware Utilizados

| Tipo | Componente o Tecnología | Función |
| :--- | :--- | :--- |
| **Control Principal** | Raspberry Pi 5 | Unidad de procesamiento central a bordo. |
| **Actuación** | Motores Mecanum con Control I2C | Movimiento omnidireccional. |
| **Percepción** | Sensor LiDAR (RPLIDAR C1) | Detección de obstáculos y seguridad (Prioridad 1). |
| **Percepción** | Cámara Web (Cenital) | Localización métrica y retroalimentación para Control PI. |
| **Librerías SW** | Python (OpenCV, NumPy) | Procesamiento de imagen, calibración y detección ArUco. |
| **Comunicaciones** | Firebase Realtime Database | Telemetría en tiempo real y envío de comandos remotos. |
| **Gestión** | GitHub | Control de versiones y repositorio de evidencia técnica. |

---

## 📂 Estructura del Repositorio

| Directorio | Contenido |
| :--- | :--- |
| `Practicas PDFS/` | Documentación detallada (PDFs) de las 5 prácticas realizadas. |
| `P1_Calibracion_Vision/` | Scripts de Calibración Intrínseca y Extrínseca. Logs de Re-proyección. |
| `P2_Control_PI_Firebase/` | Código del Controlador PI, comunicación Firebase y scripts de telemetría. |
| `P3_Validacion_LiDAR/` | Scripts para el procesamiento de la nube de puntos y logs de validación de latencia ($<200$ ms). |
| `P4_Maquina_Estados_Evasion/` | Código de la Máquina de Estados y lógica de evasión direccional. |
| `P5_Integracion_Final/` | **Código Unificado (`rebote.py` / `main.py`)**, validación de Control Jerárquico y Pruebas de Robustez. |
| `Documentacion_Final/` | Reporte final de PAP, cartel de divulgación y fichas descriptivas del producto. |

---

## ⚙️ Instrucciones de Uso y Replicación

Para replicar el entorno de desarrollo y probar el sistema:

1.  **Clonar el Repositorio:**
    ```bash
    git clone [https://github.com/LuisDiazMac/Practicas-PAP.git](https://github.com/LuisDiazMac/Practicas-PAP.git)
    ```
2.  **Configuración del Entorno:**
    * Instalar dependencias Python (OpenCV, RPi.GPIO, Firebase Admin, NumPy, etc.) utilizando `pip`.
    * Configurar las credenciales de Firebase en el archivo de configuración (`config.json` o similar).
3.  **Calibración (P1):** Ejecutar primero los scripts de calibración para obtener la Matriz Intrínseca ($K$) y la Matriz de Rotación ($R$).
4.  **Ejecución (P5):** Ejecutar el script principal de integración (`P5_Integracion_Final/rebote.py`). Este script inicializa ambos subsistemas (Visión y LiDAR) y comienza la navegación autónoma con prioridad de seguridad activa.

## 👥 Equipo de Desarrollo

Este proyecto fue desarrollado por estudiantes de la Licenciatura en Ingeniería Mecatrónica del ITESO:

* **Luis Eduardo Díaz Macías**
* **Miguel de Jesús Flores González**
* **Pablo Pérez Sánchez**
* **Jesús Alejandro Osegueda Melin**

**Profesor PAP:** Dr. Jorge Alberto Lizarraga Rodriguez
