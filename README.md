# UP Ros: Plataforma Educativa de Robótica Avanzada

> **"La educación no es el llenado de un cubo, sino el encendido de un fuego."** — W.B. Yeats

## 🎓 Introducción

**UP Ros** representa la vanguardia en la educación de ingeniería robótica. Concebida como una **plataforma educativa doctoral**, este proyecto trasciende la enseñanza tradicional para ofrecer una experiencia **lúdico-didáctica** inmersiva centrada en **ROS 2 (Robot Operating System 2)**.

Diseñada meticulosamente para estudiantes de ingeniería, la plataforma desmitifica la complejidad de la robótica moderna mediante una arquitectura pedagógica progresiva. Desde los fundamentos de Linux hasta el despliegue profesional con Docker y CI/CD, UP Ros guía al estudiante a través de un viaje interactivo, combinando teoría rigurosa con animaciones de alta calidad y ejercicios prácticos.

Esta tesis doctoral no es solo un software; es una metodología de enseñanza que busca formar a la próxima generación de arquitectos de robots.

---

## 🔬 Análisis Temático y Módulos

El currículo se estructura en una secuencia lógica de **9 módulos (0-8)**, diseñados para construir conocimiento capa por capa.

| Módulo | Título | Descripción Técnica |
| :--- | :--- | :--- |
| **0** | **Fundamentos Linux** | La base de todo. Dominio de la terminal, gestión de archivos, permisos, variables de entorno y el ecosistema de paquetes apt. Prepara el terreno para operar sistemas complejos. |
| **1** | **Programación** | Poliglotismo robótico. Scripts en Python y C++, compilación, gestión de dependencias y Bash scripting. Se establecen las bases del desarrollo de software para robótica. |
| **2** | **Formatos de Datos** | El lenguaje de las máquinas. Análisis profundo de XML, JSON y YAML, y su rol crítico en la configuración, descripción de robots (URDF) y comunicación web. |
| **3** | **Git y GitHub** | Control de versiones profesional. Desde `init` hasta Pull Requests. Enseña a gestionar la historia del código y a colaborar en entornos distribuidos modernos. |
| **4** | **ROS 2 Fundamentos** | El núcleo del sistema. Arquitectura de nodos, comunicación asíncrona (Tópicos), síncrona (Servicios) y tareas de larga duración (Acciones). |
| **5** | **Herramientas de Desarrollo** | El taller del ingeniero. Uso avanzado de la CLI, visualización con **RViz2**, diagnóstico con **RQT** y grabación de datos con **Rosbag2**. |
| **6** | **Simulación** | Laboratorio virtual. Modelado físico con **URDF/Xacro**, mundos virtuales en **Gazebo** y simulación de sensores (Lidar, Cámaras) y actuadores. |
| **7** | **Navegación Autónoma** | Inteligencia espacial. Implementación del stack **Nav2**, algoritmos de localización (**AMCL**), mapeo (**SLAM**) y planificación de trayectorias. |
| **8** | **Ingeniería de Software** | Nivel experto. Contenerización con **Docker**, integración continua (CI/CD) y orquestación de lanzamientos complejos (Launch System Pro). |

---

## 🏛️ Estructura del Proyecto

El proyecto está construido sobre el robusto framework **Quasar (Vue 3)**, garantizando una interfaz reactiva, moderna y multiplataforma.

### Arquitectura de Archivos

```plaintext
up-ros/
├── src/
│   ├── components/      # Componentes UI reutilizables (Bloques de código, alertas, etc.)
│   ├── layouts/         # Estructuras maestras de la interfaz (Menú lateral, Header)
│   ├── pages/
│   │   ├── course/      # El corazón del contenido educativo
│   │   │   ├── modulo0/ # Vistas y lecciones del Módulo 0
│   │   │   ├── ...      # Directorios para cada módulo hasta el 8
│   │   │   └── capstone/ # Proyecto final integrador
│   │   └── ...          # Páginas generales (Inicio, Glosario, Créditos)
│   ├── data/
│   │   └── courseStructure.ts # Definición programática del syllabus y rutas
│   └── ...
├── quasar.config.ts     # Configuración del compilador y plugins de Quasar
└── package.json         # Gestión de dependencias y scripts
```

El usuario encontrará en cada módulo una experiencia visual rica, donde los conceptos abstractos se traducen en interfaces animadas e interactivas.

---

## ⚙️ Requisitos del Sistema

Para garantizar el funcionamiento óptimo de la plataforma de desarrollo, asegúrese de cumplir con los siguientes requisitos:

*   **Node.js**: Se requiere una versión LTS activa. Según la configuración del proyecto, se soportan las versiones:
    *   `^20.0.0` (Recomendada)
    *   `^22.0.0`, `^24.0.0`, `^26.0.0`, `^28.0.0`
*   **Gestor de Paquetes**: `npm` (versión >= 6.13.4) o `yarn` (versión >= 1.21.1).
*   **Sistema Operativo**: Windows, macOS o Linux.

---

## 🚀 Guía de Instalación (Primeros Pasos)

Siga estas instrucciones **al pie de la letra** para desplegar el entorno de desarrollo local.

1.  **Descarga**: Descargue el código fuente del proyecto y descomprima el archivo en su ubicación de preferencia.
2.  **Terminal**: Abra una terminal (PowerShell, Bash, Zsh) y navegue hasta la **raíz de la carpeta descomprimida** (donde se encuentra el archivo `package.json`).

### Ejecución de Comandos

Ejecute los siguientes comandos en orden secuencial:

**Paso 1: Instalar dependencias del proyecto**
Esto descargará todas las librerías necesarias definidas en `package.json`.

```bash
npm install
```

**Paso 2: Instalar la CLI de Quasar globalmente**
Esta herramienta es fundamental para ejecutar el servidor de desarrollo y compilar la aplicación.

```bash
npm install -g @quasar/cli
```

---

## ▶️ Instrucciones de Ejecución

Una vez instaladas las dependencias, puede iniciar la plataforma en **modo desarrollo**. Esto levantará un servidor local con recarga en caliente (HMR), permitiéndole visualizar los cambios y las animaciones en tiempo real.

En la raíz del proyecto, ejecute:

```bash
quasar dev
```

El navegador debería abrirse automáticamente en `http://localhost:9000`. ¡Bienvenido a **UP Ros**!

---

## 📜 Licencia y Créditos

**Autor:** Alexander Calderon Leal
**Contacto:** edwin.calderon@unipamplona.edu.co
**Institución:** Universidad de Pamplona - Ingeniería Mecatrónica

*Desarrollado como opción de grado para el programa de Ingeniería Mecatrónica.*
