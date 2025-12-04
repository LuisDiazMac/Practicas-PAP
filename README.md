# 🤖 Practicas-PAP: RobotMesha - Sistema Autónomo con Percepción Dual

## 📝 Resumen del Proyecto PAP (Otoño 2025)

Este repositorio contiene la totalidad del código, la documentación técnica y los logs de validación del Proyecto de Aplicación Profesional (PAP) **RobotMesha**. El proyecto se centra en el diseño e implementación de un vehículo móvil omnidireccional dotado de un sistema de **Percepción Dual** que fusiona la **Visión Artificial (OpenCV/ArUco)** para la navegación métrica precisa, con un **sensor LiDAR (RPLIDAR C1)** para la detección de obstáculos y la seguridad operacional.

[cite_start]El principal hito técnico fue la implementación de un **Control Jerárquico** donde la seguridad (detección LiDAR) anula la navegación (Control PI), logrando un sistema autónomo robusto y certificado para operar en entornos dinámicos compartidos[cite: 146].

---

## 🎯 Objetivos y Criterios de Aceptación

[cite_start]El proyecto se enfocó en resolver la problemática de la **imprecisión métrica** y el **riesgo operacional por latencia** en prototipos experimentales[cite: 89].

| Objetivo Táctico Clave | Tecnología Implementada | Criterio de Aceptación (CA) | Prácticas Involucradas |
| :--- | :--- | :--- | :--- |
| **Navegación Precisa** | [cite_start]Control PI de lazo cerrado y Calibración Métrica ArUco[cite: 134, 131]. | [cite_start]Lograr la capacidad de posicionamiento estable y métrico mediante retroalimentación de la cámara[cite: 115]. | P1, P2 |
| **Seguridad Operacional** | [cite_start]Validación de $T_{reac}$ y Máquina de Estados Finita (FSM)[cite: 140, 143]. | [cite_start]Certificar que el tiempo de reacción ante un obstáculo sea **$T_{reac} < 200$ ms**[cite: 102]. | P3, P4 |
| **Integración Final** | [cite_start]Control Jerárquico (LiDAR > ArUco)[cite: 145]. | [cite_start]Fusión estable de ambos subsistemas, validando la anulación de la navegación por la señal de seguridad[cite: 146]. | P5, P6 |

---

## 🛠️ Tecnologías y Hardware Utilizados

| Tipo | Componente o Tecnología | Función |
| :--- | :--- | :--- |
| **Control Principal** | Raspberry Pi 5 | Unidad de procesamiento central a bordo. |
| **Actuación** | Motores Mecanum con Control I2C | Movimiento omnidireccional. |
| **Percepción** | Sensor LiDAR (RPLIDAR C1) | [cite_start]Detección de obstáculos y seguridad (Prioridad 1)[cite: 140]. |
| **Percepción** | Cámara Web (Cenital) | [cite_start]Localización métrica y retroalimentación para Control PI[cite: 116]. |
| **Librerías SW** | Python (OpenCV, NumPy) | [cite_start]Procesamiento de imagen, calibración y detección ArUco[cite: 133]. |
| **Comunicaciones** | Firebase Realtime Database | [cite_start]Telemetría en tiempo real y envío de comandos remotos[cite: 137]. |
| **Gestión** | GitHub | [cite_start]Control de versiones y repositorio de evidencia técnica[cite: 62]. |

---

## 📂 Estructura del Repositorio

| Directorio | Contenido |
| :--- | :--- |
| `Practicas PDFS/` | [cite_start]Documentación detallada (PDFs) de las 6 prácticas realizadas[cite: 39]. |
| `P1_Calibracion_Vision/` | Scripts de Calibración Intrínseca y Extrínseca. Logs de Re-proyección. |
| `P2_Control_PI_Firebase/` | [cite_start]Código del Controlador PI, comunicación Firebase y scripts de telemetría[cite: 134]. |
| `P3_Validacion_LiDAR/` | [cite_start]Scripts para el procesamiento de la nube de puntos y logs de validación de latencia ($<200$ ms)[cite: 140, 141]. |
| `P4_Maquina_Estados_Evasion/` | [cite_start]Código de la Máquina de Estados y lógica de evasión direccional[cite: 143]. |
| `P5_Integracion_Final/` | [cite_start]**Código Unificado (`rebote.py` / `main.py`)**, validación de Control Jerárquico y Pruebas de Robustez[cite: 145]. |
| `Documentacion_Final/` | [cite_start]Reporte final de PAP, cartel de divulgación y fichas descriptivas del producto[cite: 177, 179]. |

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

[cite_start]Este proyecto fue desarrollado por estudiantes de la Licenciatura en Ingeniería Mecatrónica del ITESO[cite: 11, 12, 13, 14]:

* **Luis Eduardo Díaz Macías**
* **Miguel de Jesús Flores González**
* **Pablo Pérez Sánchez**
* **Jesús Alejandro Osegueda Melin**

[cite_start]**Profesor PAP:** Dr. Jorge Alberto Lizarraga Rodriguez [cite: 15]
