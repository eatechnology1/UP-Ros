/*
📁 Curso ROS 2 - Estructura Completa
│
├── 🏠 Dashboard
│   └── Vista general del progreso, últimas lecciones visitadas
│
├── 📖 Introducción
│   ├── Bienvenida al Curso
│   ├── Instalación del Entorno (Ubuntu/WSL/VM)
│   └── Configuración Inicial (Verificación de herramientas)
│
├── 📦 Módulo 0: Fundamentos del Sistema (Linux)
│   ├── 0.1 Navegación en la Terminal
│   │   ├── pwd (¿Dónde estoy?)
│   │   ├── ls (¿Qué hay aquí?)
│   │   └── cd (Moverse entre carpetas)
│   │
│   ├── 0.2 Gestión de Archivos y Directorios
│   │   ├── mkdir (Crear carpetas)
│   │   ├── touch (Crear archivos)
│   │   ├── cp (Copiar)
│   │   ├── mv (Mover/Renombrar)
│   │   └── rm (Eliminar con precaución)
│   │
│   ├── 0.3 Permisos y Usuarios
│   │   ├── chmod (Cambiar permisos)
│   │   ├── chown (Cambiar propietario)
│   │   └── sudo (Superpoderes temporales)
│   │
│   ├── 0.4 Editores de Texto en Terminal
│   │   ├── nano (Editor básico)
│   │   ├── vim (Mención introductoria)
│   │   └── code (VS Code desde terminal)
│   │
│   ├── 0.5 Variables de Entorno y Configuración
│   │   ├── ¿Qué es el PATH?
│   │   ├── Editar .bashrc
│   │   ├── Crear alias personalizados
│   │   └── source (Recargar configuración)
│   │
│   └── 0.6 Instalación de Software
│       ├── apt update & apt upgrade
│       ├── apt install (Gestión de paquetes)
│       └── Solución de dependencias rotas
│
├── 💻 Módulo 1: Programación Esencial
│   ├── 1.1 Python: Scripts y Ejecución
│   │   ├── Crear tu primer script.py
│   │   ├── Shebang (#!/usr/bin/env python3)
│   │   └── chmod +x (Hacer ejecutable)
│   │
│   ├── 1.2 Python: Módulos y Librerías
│   │   ├── import y gestión de módulos
│   │   ├── pip install (Instalar paquetes)
│   │   └── Entornos virtuales (venv)
│   │
│   ├── 1.3 C++: Compilación Básica
│   │   ├── Interpretado vs Compilado
│   │   ├── g++ hola.cpp -o hola
│   │   └── Ejecutar binarios ./hola
│   │
│   ├── 1.4 C++: Estructura de un Programa
│   │   ├── main() y return 0
│   │   ├── #include y librerías estándar
│   │   └── Tipos de datos (int, float, string)
│   │
│   └── 1.5 Bash Scripting (Automatización)
│       ├── Crear scripts .sh
│       ├── Variables y condicionales básicas
│       └── Lanzar múltiples comandos secuenciales
│
├── 📄 Módulo 2: Formatos de Datos y Configuración
│   ├── 2.1 XML (Lenguaje de Marcado)
│   │   ├── Estructura: Etiquetas y atributos
│   │   ├── Anidamiento jerárquico
│   │   └── Uso en ROS: package.xml y URDF
│   │
│   ├── 2.2 JSON (JavaScript Object Notation)
│   │   ├── Objetos {} y Arrays []
│   │   ├── Clave-Valor (key: value)
│   │   └── Uso en APIs y configuraciones web
│   │
│   ├── 2.3 YAML (YAML Ain't Markup Language)
│   │   ├── Indentación estricta (espacios, NO tabs)
│   │   ├── Listas y diccionarios
│   │   └── Uso en ROS 2: Parámetros y Launch Files
│   │
│   └── 2.4 Comparativa y Conversión
│       ├── ¿Cuándo usar cada formato?
│       ├── Herramientas de conversión online
│       └── Ejercicio: Robot Config en XML → YAML
│
├── 🌳 Módulo 3: Git y Control de Versiones
│   ├── 3.1 Conceptos Fundamentales
│   │   ├── ¿Qué es un repositorio?
│   │   ├── git init (Iniciar repo local)
│   │   └── git status (Estado actual)
│   │
│   ├── 3.2 Guardando Cambios (Commits)
│   │   ├── git add (Staging area)
│   │   ├── git commit -m "mensaje"
│   │   └── Buenas prácticas de mensajes
│   │
│   ├── 3.3 Historia y Navegación
│   │   ├── git log (Ver historial)
│   │   ├── git checkout (Viajar en el tiempo)
│   │   └── git reset (Deshacer con cuidado)
│   │
│   ├── 3.4 Ramas (Branching)
│   │   ├── git branch (Crear/Listar ramas)
│   │   ├── git switch (Cambiar de rama)
│   │   └── main vs develop (Flujo GitFlow básico)
│   │
│   ├── 3.5 Repositorios Remotos (GitHub)
│   │   ├── git remote add origin
│   │   ├── git push (Subir cambios)
│   │   ├── git pull (Traer cambios)
│   │   └── Autenticación SSH (Clave pública/privada)
│   │
│   ├── 3.6 Colaboración (Pull Requests)
│   │   ├── fork (Copiar repo de otro usuario)
│   │   ├── clone (Descargar a tu máquina)
│   │   ├── Crear rama feature/mi-cambio
│   │   └── Abrir Pull Request (PR)
│   │
│   └── 3.7 Gitignore (Archivos Excluidos)
│       ├── ¿Qué NO subir al repo?
│       ├── Sintaxis de .gitignore
│       └── Plantillas por lenguaje (Python, C++, ROS)
│
├── 🤖 Módulo 4: Fundamentos de ROS 2 (El Núcleo)
│   ├── 4.1 Arquitectura y Conceptos
│   │   ├── ¿Qué es el Middleware DDS?
│   │   ├── Grafo de Nodos (Nodes)
│   │   └── Dominio ID (Aislamiento de redes)
│   │
│   ├── 4.2 Workspace y Paquetes
│   │   ├── Estructura colcon_ws (src, build, install, log)
│   │   ├── colcon build (Compilación)
│   │   ├── ros2 pkg create (Python vs C++)
│   │   └── El archivo package.xml y CMakeLists.txt/setup.py
│   │
│   ├── 4.3 Comunicación: Topics (Pub/Sub)
│   │   ├── Concepto Publicador / Suscriptor
│   │   ├── ros2 topic list / echo / info / pub
│   │   ├── Tipos de mensajes estándar (std_msgs, geometry_msgs)
│   │   ├── Ejercicio: Talker/Listener en Python
│   │   └── Ejercicio: Talker/Listener en C++
│   │
│   ├── 4.4 Comunicación: Servicios (Req/Res)
│   │   ├── Concepto Cliente / Servidor (Síncrono vs Asíncrono)
│   │   ├── ros2 service list / call / type
│   │   ├── Interfaces .srv personalizadas
│   │   └── Ejercicio: Servidor de Suma de dos enteros
│   │
│   └── 4.5 Comunicación: Acciones (Long running tasks)
│       ├── Concepto Goal / Feedback / Result
│       ├── ros2 action list / send_goal
│       ├── Interfaces .action personalizadas
│       └── Ejercicio: Servidor de Acción (Mover robot a coordenada)
│
├── 🚀 Módulo 5: Herramientas y Debugging (El Taller)
│   ├── 5.1 Launch System (Automatización de Despliegue)
│   │   ├── ¿Qué son los Launch Files? (Python based)
│   │   ├── Lanzar múltiples nodos
│   │   ├── Remapeo de nombres y tópicos
│   │   └── Uso de argumentos y configuraciones condicionales
│   │
│   ├── 5.2 Parámetros (Configuración Dinámica)
│   │   ├── Concepto de Parameter Server distribuido
│   │   ├── ros2 param list / get / set / dump
│   │   ├── Cargar parámetros desde YAML
│   │   └── Declaración de parámetros en el código (Python/C++)
│   │
│   ├── 5.3 Visualización y Diagnóstico
│   │   ├── Rviz2: Visualizando el mundo del robot (Sensores, TF)
│   │   ├── Rqt_graph: Visualizando el grafo de nodos
│   │   ├── Rqt_plot: Gráficas de datos en tiempo real
│   │   └── Ros2 doctor: Diagnóstico del sistema
│   │
│   └── 5.4 Grabación de Datos (Rosbag2)
│       ├── ros2 bag record (Grabar tópicos)
│       ├── ros2 bag play (Reproducir escenarios)
│       ├── Formato SQLite3 vs MCAP
│       └── Análisis de bags (Info y filtrado)
│
├── 🦾 Módulo 6: Modelado y Simulación (El Mundo Virtual)
│   ├── 6.1 Transformaciones (TF2)
│   │   ├── El árbol de transformaciones (base_link, odom, map)
│   │   ├── Publicadores de TF (Static vs Dynamic)
│   │   ├── Visualización de TF en Rviz
│   │   └── Herramientas tf2_echo y view_frames
│   │
│   ├── 6.2 Descripción del Robot (URDF/Xacro)
│   │   ├── Estructura URDF (Links y Joints)
│   │   ├── Visual vs Collision vs Inertial
│   │   ├── Xacro: Macros para simplificar URDFs complejos
│   │   └── Publicar estado del robot (robot_state_publisher)
│   │
│   └── 6.3 Simulación Física (Gazebo / Ignition)
│       ├── Diferencias entre Gazebo Classic e Ignition
│       ├── Spawning del robot en el mundo
│       ├── Plugins de sensores (Lidar, Cámara, IMU)
│       └── Plugins de control (Diff Drive, Ackermann)
│
├── 🧠 Módulo 7: Navegación y Percepción (Nav2)
│   ├── 7.1 Stack de Navegación (Nav2)
│   │   ├── Arquitectura Behavior Trees (BT)
│   │   ├── Configuración de Costmaps (Global y Local)
│   │   ├── Planificadores (Planners) y Controladores (Controllers)
│   │   └── Configuración de AMCL (Localización Monte Carlo)
│   │
│   └── 7.2 SLAM (Simultaneous Localization and Mapping)
│       ├── Concepto de SLAM
│       ├── SLAM Toolbox (Generación de mapas 2D)
│       ├── Cartographer (Google)
│       └── Guardar y cargar mapas (Map Server)
│
└── 🐳 Módulo 8: Despliegue Profesional y DevOps (Nivel Ingeniero)
    ├── 8.1 Contenerización con Docker
    │   ├── ¿Por qué Docker en Robótica?
    │   ├── Escribir un Dockerfile para ROS 2
    │   ├── Docker Compose para sistemas multi-robot
    │   └── Uso de GPU en Docker (Nvidia Container Toolkit)
    │
    ├── 8.2 Integración Continua (CI/CD)
    │   ├── GitHub Actions para ROS 2
    │   ├── Linter (ament_lint, flake8, cpplint)
    │   └── Tests Automáticos (GTest, PyTest en ROS)
    │
    └── 8.3 Despliegue en Hardware Real (Edge)
        ├── Cross-Compilation (Compilación cruzada)
        ├── Configuración de Network (Discovery Server)
        ├── Optimización de DDS (FastDDS vs CycloneDDS)
        └── Gestión de servicios con Systemd (Autoarranque)

│
├── 📚 Glosario
│   └── Términos técnicos con definiciones breves
│
├── 🏆 Créditos
│   └── Autor, colaboradores, recursos utilizados
│
└── 🎨 Plantilla (Componentes UI)
    ├── SectionTitle.vue
    ├── TextBlock.vue
    ├── CodeBlock.vue
    ├── AlertBlock.vue
    ├── SplitBlock.vue
    └── StepsBlock.vue
*/
// src/data/courseStructure.ts

export interface CourseNode {
  title: string;
  path?: string; // Ruta relativa (ej: 'navegacion-terminal')
  icon?: string; // Icono de Material Icons
  children?: CourseNode[];
}

export const courseStructure: CourseNode[] = [
  {
    title: 'Inicio',
    path: 'home',
    icon: 'home',
  },
  {
    title: 'Módulo 0: Fundamentos Linux',
    icon: 'terminal',
    path: 'modulo-0',
    children: [
      { title: '0.1 Navegación Terminal', path: '01navsistemaPage' },
      { title: '0.2 Gestión de Archivos', path: '02gestionarchivosPage' },
      { title: '0.3 Permisos y Usuarios', path: '03permisosPage' },
      { title: '0.4 Editores de Texto', path: '04editoresPage' },
      { title: '0.5 Variables de Entorno', path: '05variablesentornoPage' },
      { title: '0.6 Instalación Software', path: '06instalacionPage' },
    ],
  },
  {
    title: 'Módulo 1: Programación',
    icon: 'code',
    path: 'modulo-1',
    children: [
      { title: '1.1 Python Scripts', path: '01python-scripts' },
      { title: '1.2 Módulos y Librerías', path: '02python-modulos' },
      { title: '1.3 Compilación C++', path: '03cpp-compilacion' },
      { title: '1.4 Estructura C++', path: '04cpp-estructura' },
      { title: '1.5 Bash Scripting', path: '05bash-scripting' },
    ],
  },
  {
    title: 'Módulo 2: Formatos de Datos',
    icon: 'data_object',
    path: 'modulo-2',
    children: [
      { title: '2.1 XML Básico', path: '01xmlPage' },
      { title: '2.2 JSON Básico', path: '02jsonPage' },
      { title: '2.3 YAML Básico', path: '03yamlPage' },
      { title: '2.4 Conversión y Uso', path: '04conversionPage' },
    ],
  },
  {
    title: 'Módulo 3: Git y GitHub',
    icon: 'call_split', // Icono de rama/branch
    path: 'modulo-3',
    children: [
      { title: '3.1 Conceptos Básicos', path: '01conceptosPage' },
      { title: '3.2 Commits', path: '02commitsPage' },
      { title: '3.3 Historial', path: '03historialPage' },
      { title: '3.4 Ramas (Branches)', path: '04ramasPage' },
      { title: '3.5 Remotos', path: '05remotosPage' },
      { title: '3.6 Pull Requests', path: '06pull-requestsPage' },
      { title: '3.7 Gitignore', path: '07gitignorePage' },
    ],
  },
  {
    title: 'Módulo 4: ROS 2 Fundamentos',
    icon: 'smart_toy', // Icono de robot
    path: 'modulo-4',
    children: [
      { title: '4.1 Arquitectura', path: '01arquitecturaPage' },
      { title: '4.2 Workspace y Paquetes', path: '02workspacePage' },
      { title: '4.3 Tópicos (Topics)', path: '03topicsPage' },
      { title: '4.4 Servicios', path: '04serviciosPage' },
      { title: '4.5 Acciones', path: '05accionesPage' },
    ],
  },
  {
    title: 'Módulo 5: Herramientas',
    icon: 'build',
    path: 'modulo-5',
    children: [
      { title: '5.1 Launch System', path: '01launchPage' },
      { title: '5.2 Parámetros', path: '02parametrosPage' },
      { title: '5.3 Visualización', path: '03visualizacionPage' },
      { title: '5.4 Rosbag', path: '04rosbagPage' },
    ],
  },
  {
    title: 'Módulo 6: Simulación',
    icon: 'public', // Icono de mundo
    path: 'modulo-6',
    children: [
      { title: '6.1 Transformaciones (TF)', path: 'tf2' },
      { title: '6.2 URDF y Xacro', path: 'urdf' },
      { title: '6.3 Gazebo', path: 'gazebo' },
    ],
  },
  {
    title: 'Módulo 7: Navegación (Nav2)',
    icon: 'explore',
    path: 'modulo-7',
    children: [
      { title: '7.1 Stack Nav2', path: 'nav2-stack' },
      { title: '7.2 SLAM y Mapas', path: 'slam' },
    ],
  },
  {
    title: 'Módulo 8: Despliegue Pro',
    icon: 'rocket_launch',
    path: 'modulo-8',
    children: [
      { title: '8.1 Docker', path: 'docker' },
      { title: '8.2 CI/CD', path: 'ci-cd' },
      { title: '8.3 Edge Deployment', path: 'edge' },
    ],
  },
  // {
  //   title: 'Recursos',
  //   icon: 'menu_book',
  //   path: 'recursos', // Carpeta contenedora
  //   children: [
  //     { title: 'Glosario', path: 'glosario' },
  //     { title: 'Créditos', path: 'creditos' },
  //   ],
  // },
];
