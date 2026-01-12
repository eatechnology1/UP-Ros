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
  description?: string; // Para el encabezado automático de LessonContainer

  // === DASHBOARD INTELLIGENCE METADATA ===
  tooltip?: string; // Resumen técnico rápido para el menú
  difficulty?: 'beginner' | 'intermediate' | 'advanced'; // Nivel de dificultad
  estimatedTime?: string; // Tiempo estimado (ej: '45 min', '2 horas')
  tags?: string[]; // Etiquetas técnicas (ej: ['CLI', 'Linux', 'Terminal'])
  prerequisite?: string[]; // Paths de lecciones prerequisito

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
    tooltip: 'Domina la terminal Linux: navegación, permisos, editores y gestión de paquetes',
    difficulty: 'beginner',
    estimatedTime: '6 horas',
    tags: ['Linux', 'CLI', 'Terminal', 'Bash'],
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
    tooltip: 'Python, C++ y Bash: los 3 lenguajes esenciales para ROS 2',
    difficulty: 'beginner',
    estimatedTime: '8 horas',
    tags: ['Python', 'C++', 'Bash', 'Scripting'],
    prerequisite: ['modulo-0'],
    children: [
      {
        title: '1.1 Python Scripts',
        path: '01python-scripts',
        description:
          'Un script de Python en reposo es solo texto. En ejecución, es el cerebro de tu robot. Aprende el ritual para darle vida: desde la "Línea Maestra" (Shebang) hasta la gestión de memoria.',
      },
      {
        title: '1.2 Módulos y Librerías',
        path: '02python-modulos',
        description:
          'Un robot complejo no se programa en un solo archivo gigante. Aprende a dividir tu código en piezas reutilizables (Módulos) y a organizarlas en cajas de herramientas (Paquetes).',
      },
      {
        title: '1.3 Compilación C++',
        path: '03cpp-compilacion',
        description:
          'Python lee tu código línea por línea mientras funciona. C++ no. Traduce todo a binario antes de empezar. Entender esta "traducción anticipada" es clave para sobrevivir a los errores de colcon build.',
      },
      {
        title: '1.4 Estructura C++',
        path: '04cpp-estructura',
        description:
          'Separamos las Promesas (.hpp) de las Acciones (.cpp). Esta estructura es obligatoria para crear nodos de ROS 2 limpios y compilables.',
      },
      {
        title: '1.5 Bash Scripting',
        path: '05bash-scripting',
        description:
          'Bash es el lenguaje nativo de tu terminal. Aprende a crear "macros" potentes para configurar tu entorno, lanzar múltiples nodos y automatizar tareas repetitivas.',
      },
    ],
  },
  {
    title: 'Módulo 2: Formatos de Datos',
    icon: 'data_object',
    path: 'modulo-2',
    tooltip: 'XML, JSON y YAML: serialización y configuración en robótica',
    difficulty: 'beginner',
    estimatedTime: '4 horas',
    tags: ['XML', 'JSON', 'YAML', 'Serialización'],
    prerequisite: ['modulo-1'],
    children: [
      {
        title: '2.1 XML Básico',
        path: '01xmlPage',
        description:
          'XML es el lenguaje que usamos para describir estructuras jerárquicas. En ROS 2, es la ley para definir quién es tu robot (física, articulaciones) y qué necesita tu software (dependencias).',
      },
      {
        title: '2.2 JSON Básico',
        path: '02jsonPage',
        description:
          'JSON es el estándar para mover datos en internet. En Robótica, es el puente entre tu máquina y el mundo humano: interfaces web, dashboards de control y bases de datos en la nube.',
      },
      {
        title: '2.3 YAML Básico',
        path: '03yamlPage',
        description:
          'YAML es el formato diseñado para humanos. En ROS 2, es el rey de la configuración. Se usa para definir parámetros de navegación, configurar simulaciones y guardar mapas.',
      },
      {
        title: '2.4 Conversión y Uso',
        path: '04conversionPage',
        description:
          'En el mundo real, los datos no se quedan quietos. Aprende el arte de la Serialización: cómo transformar cualquier formato YAML/JSON/XML en objetos manipulables por tu código (Python Dicts).',
      },
    ],
  },
  {
    title: 'Módulo 3: Git y GitHub',
    icon: 'call_split',
    path: 'modulo-3',
    tooltip: 'Control de versiones profesional: commits, ramas, colaboración y GitHub',
    difficulty: 'intermediate',
    estimatedTime: '7 horas',
    tags: ['Git', 'GitHub', 'Control de Versiones', 'Colaboración'],
    prerequisite: ['modulo-0'],
    children: [
      {
        title: '3.1 Conceptos Fundamentales',
        path: '01conceptosPage',
        description:
          'Imagina trabajar en tu tesis y guardar archivos como "tesis_final_v2.doc". Git es un sistema profesional para guardar "fotos" (snapshots) de tu código en el tiempo.',
      },
      {
        title: '3.2 Creando Commits',
        path: '02commitsPage',
        description:
          'Un "Commit" es mucho más que guardar archivos. Es una cápsula del tiempo sellada con un mensaje explicativo. Aprende el ritual sagrado: Status, Add y Commit.',
      },
      {
        title: '3.3 Historial y Viajes',
        path: '03historialPage',
        description:
          'De nada sirve guardar versiones si no sabes cómo volver a ellas. Git te permite moverte por la línea de tiempo de tu proyecto, viajar al pasado y deshacer errores fatales.',
      },
      {
        title: '3.4 Ramas (Branches)',
        path: '04ramasPage',
        description:
          'El desarrollo real no es lineal. Las Ramas son realidades paralelas donde puedes experimentar sin romper el código principal. Aprende a crear, cambiar y fusionar universos.',
      },
      {
        title: '3.5 Repositorios Remotos',
        path: '05remotosPage',
        description:
          'Tu código en tu laptop está aislado. Conecta tu repositorio local con GitHub usando los comandos sagrados: Remote, Push y Pull para sincronizar tu trabajo con la nube.',
      },
      {
        title: '3.6 Pull Requests',
        path: '06pull-requestsPage',
        description:
          'En equipos profesionales, nadie toca la rama main directamente. Usa "Solicitudes de Fusión" (Pull Requests) para que tus compañeros revisen y aprueben tu código antes de integrarlo.',
      },
      {
        title: '3.7 .gitignore',
        path: '07gitignorePage',
        description:
          'Git intenta guardar todo, incluso la basura. Configura el escudo .gitignore para mantener tu repositorio limpio de archivos compilados, entornos virtuales y configuraciones locales.',
      },
    ],
  },
  {
    title: 'Módulo 4: ROS 2 Fundamentos',
    icon: 'smart_toy',
    path: 'modulo-4',
    tooltip: 'Arquitectura distribuida: nodos, tópicos, servicios y acciones',
    difficulty: 'intermediate',
    estimatedTime: '10 horas',
    tags: ['ROS2', 'DDS', 'Middleware', 'Pub/Sub'],
    prerequisite: ['modulo-1', 'modulo-2'],
    children: [
      {
        title: '4.1 Arquitectura',
        path: '01arquitecturaPage',
        description:
          'Olvida la programación Arduino. ROS 2 es una red de "nodos" independientes. Si la cámara falla, las ruedas siguen girando. Bienvenido a la robustez distribuida.',
      },
      {
        title: '4.2 Workspace y Paquetes',
        path: '02workspacePage',
        description:
          'Estructura tu caos con Workspaces y Paquetes. Aprende el ciclo de vida sagrado: colcon build, source setup.bash y la diferencia entre paquetes Python y C++.',
      },
      {
        title: '4.3 Tópicos (Topics)',
        path: '03topicsPage',
        description:
          'Como una radio FM: unos hablan (Publishers) y otros escuchan (Subscribers). Tuberías de datos unidireccionales para streaming de video, láser y sensores.',
      },
      {
        title: '4.4 Servicios',
        path: '04serviciosPage',
        description:
          'Cuando necesitas una respuesta inmediata ("¿Estás listo?"). Comunicación síncrona Cliente-Servidor para transacciones puntuales y control lógico.',
      },
      {
        title: '4.5 Acciones',
        path: '05accionesPage',
        description:
          'Para tareas largas ("Ve a la cocina"). Combina objetivos, feedback en tiempo real y resultados finales. ¡Puedes cancelar la misión si te arrepientes!',
      },
    ],
  },
  {
    title: 'Módulo 5: Herramientas de Desarrollo',
    icon: 'handyman',
    path: 'modulo-5',
    tooltip: 'RViz2, RQT, Rosbag2 y TF2: diagnóstico y visualización profesional',
    difficulty: 'intermediate',
    estimatedTime: '6 horas',
    tags: ['RViz', 'RQT', 'Debugging', 'Visualización', 'TF2'],
    prerequisite: ['modulo-4'],
    children: [
      {
        title: '5.1 CLI Avanzada & Colcon',
        path: '01cli-colconPage',
        description:
          'Hasta ahora has usado comandos básicos. Pero, ¿qué pasa si el sistema va lento? ¿O si necesitas compilar solo un paquete entre 50? En este módulo aprenderás las herramientas de línea de comandos (CLI) que distinguen a un usuario de un ingeniero de ROS 2. Dominarás el diagnóstico de red y la compilación selectiva.',
      },
      {
        title: '5.2 RViz2: Visualización',
        path: '02rviz2Page',
        description:
          'Los robots "piensan" en números y matrices. Para que nosotros los entendamos, necesitamos convertir esos números en formas, líneas y colores. RViz (ROS Visualization) no es un simulador; es una ventana a la mente del robot. Si el robot cree que hay una pared delante, RViz te mostrará esa pared, exista o no en la realidad.',
      },
      {
        title: '5.3 RQT: Gráficas y Logs',
        path: '03rqtPage',
        description:
          'Si RViz son los ojos del robot, RQT (ROS Qt) es su monitor de signos vitales. Aquí no verás paredes ni mapas 3D. Verás la salud del sistema: quién habla con quién (Topología), la estabilidad de los sensores (Gráficas) y los gritos de auxilio internos (Logs).',
      },
      {
        title: '5.4 Rosbag2: Grabación',
        path: '04rosbagPage',
        description:
          'Los robots fallan. Y cuando fallan en el mundo real, es difícil saber por qué. ¿Fue un error del sensor? ¿Un fallo en el código? Rosbag2 te permite grabar todos los mensajes de los tópicos en un archivo. Luego, puedes "reproducir" esos datos en tu casa, engañando a tus nodos para que crean que el robot sigue funcionando.',
      },
      {
        title: '5.5 Debugging TF2',
        path: '05tf2-debugPage',
        description:
          'Un robot no es un punto en el espacio; es una colección de partes (ruedas, sensores, chasis) conectadas entre sí. Para que el robot sepa que un obstáculo visto por la cámara (frente) está a 2 metros de las ruedas (atrás), necesita matemáticas. TF2 es el bibliotecario que mantiene el registro de todas estas relaciones de coordenadas (Transforms) en el tiempo. Si el TF falla, el robot "se rompe" geométricamente.',
      },
    ],
  },
  {
    title: 'Módulo 6: Simulación (Lab Virtual)',
    icon: 'science',
    path: 'modulo-6',
    tooltip: 'URDF, Gazebo y plugins: construye robots virtuales con física realista',
    difficulty: 'advanced',
    estimatedTime: '8 horas',
    tags: ['URDF', 'Gazebo', 'Simulación', 'Física'],
    prerequisite: ['modulo-4', 'modulo-5'],
    children: [
      {
        title: '6.1 URDF: Modelado del Robot',
        path: '01urdf-modelingPage',
        description:
          'Antes de que un robot pueda moverse en una simulación, necesita un cuerpo. URDF (Unified Robot Description Format) es el estándar XML que define la geometría, la física y la cinemática. Sin él, ROS 2 no sabe si controlas un dron o una tostadora.',
      },
      {
        title: '6.2 Gazebo: Mundos Virtuales',
        path: '02gazebo-worldsPage',
        description:
          'Tu robot no sabe si existe en el mundo real o en una simulación. Solo procesa datos. Gazebo genera esa "alucinación consensuada": calcula gravedad, fricción, inercia y luz para engañar a tu robot y permitirte fallar sin costosos desastres de hardware.',
      },
      {
        title: '6.3 Sensores Simulados',
        path: '03simulated-sensorsPage',
        description:
          'Un robot en Gazebo es sordo y ciego por defecto. Para que pueda navegar, inyectamos Plugins. Son pequeños programas que leen la "Matrix" (la geometría perfecta de Gazebo) y generan mensajes ROS con errores calculados, engañando a tu nodo de navegación para que crea que está en el mundo real.',
      },
      {
        title: '6.4 Plugins de Control',
        path: '04gazebo-pluginsPage',
        description:
          'Un modelo URDF es estático como una estatua. Para moverlo, necesitamos inyectar vida. Los Plugins de Control son el puente inverso: escuchan comandos de ROS 2 (como "avanza a 1 m/s") y calculan las fuerzas físicas necesarias para girar las ruedas virtuales en Gazebo.',
      },
    ],
  },
  {
    title: 'Módulo 7: Navegación Autónoma (Nav2)',
    icon: 'explore',
    path: 'modulo-7',
    tooltip: 'SLAM, localización y planificación: navegación autónoma completa',
    difficulty: 'advanced',
    estimatedTime: '12 horas',
    tags: ['Nav2', 'SLAM', 'AMCL', 'Planificación', 'Autonomía'],
    prerequisite: ['modulo-6'],
    children: [
      {
        title: '7.1 SLAM: Mapeando el Mundo',
        path: '01slam-mappingPage',
        description:
          '¿Cómo puede un robot dibujar un mapa de un lugar que no conoce mientras intenta no perderse en él? SLAM (Simultaneous Localization and Mapping) es el algoritmo huevo-gallina que resuelve este dilema construyendo mapas a partir de rayos láser y matemáticas.',
      },
      {
        title: '7.2 Localización: ¿Dónde estoy?',
        path: '02localizationPage',
        description:
          'Tener un mapa no sirve de nada si no sabes tu ubicación en él. AMCL es el algoritmo probabilístico que usa una "nube de partículas" para adivinar dónde está el robot, filtrando hipótesis falsas hasta converger en la realidad.',
      },
      {
        title: '7.3 Nav2: Configuración',
        path: '03nav2-configPage',
        description:
          'Nav2 es la joya de la corona de ROS 2. Aprende a configurar sus Costmaps (mapas de obstáculos), Planners (GPS global) y Controllers (piloto local) para que tu robot navegue con elegancia y no como un conductor ebrio.',
      },
      {
        title: '7.4 Scripting de Misiones',
        path: '04nav-scriptingPage',
        description:
          'RViz es para humanos. Los robots autónomos necesitan código. Usaremos la Nav2 Simple Commander API en Python para programar misiones complejas: "Ve a la cocina, recoge la carga, y si hay un obstáculo, espera 5 segundos antes de buscar otra ruta".',
      },
    ],
  },
  {
    title: 'Módulo 8: Ingeniería de Software',
    icon: 'integration_instructions',
    path: 'modulo-8',
    tooltip: 'Launch avanzado, Docker y despliegue: ingeniería de nivel producción',
    difficulty: 'advanced',
    estimatedTime: '10 horas',
    tags: ['Docker', 'DevOps', 'CI/CD', 'Producción'],
    prerequisite: ['modulo-7'],
    children: [
      {
        title: '8.1 Launch System Pro',
        path: '01launch-proPage',
        description:
          'Un robot profesional no se arranca abriendo 20 pestañas de terminal. Se usa un Launch File maestro que orquesta el encendido de hardware, navegación y lógica, manejando reinicios automáticos si algo falla.',
      },
      {
        title: '8.2 Gestión de Configuración',
        path: '02yaml-configPage',
        description:
          'Hardcoding es pecado. Aprende a separar la lógica (C++/Python) de la configuración (YAML) para que tu mismo código funcione en diferentes robots sin recompilar.',
      },
      {
        title: '8.3 Docker para Robótica',
        path: '03docker-simPage',
        description:
          '¿"Funciona en mi máquina" pero no en la del cliente? Docker encapsula todo tu entorno (OS + ROS + Librerías) en una caja inmutable. Si corre en tu Docker, corre en cualquier parte.',
      },
      {
        title: '8.4 Proyecto Final Integrador',
        path: '04proyecto-finalPage',
        description:
          'El examen final. Diseña, construye y programa un robot móvil autónomo para operar en un almacén logístico. Sin guías paso a paso. Solo tú y el código.',
      },
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
