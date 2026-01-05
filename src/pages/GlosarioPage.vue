<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: EL DICCIONARIO -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Referencia Rápida</div>
        <h1 class="hero-title">Glosario de <span class="text-primary">Términos</span></h1>

        <TextBlock>
          La robótica tiene su propio lenguaje. Aquí encontrarás definiciones precisas de todos los
          conceptos, acrónimos y herramientas mencionados en el curso. Usa el buscador para filtrar
          rápidamente.
        </TextBlock>

        <!-- BARRA DE BÚSQUEDA -->
        <div class="search-container q-mt-lg">
          <q-input
            v-model="search"
            dark
            outlined
            rounded
            placeholder="Buscar término (ej: Nodo, DDS, Gazebo)..."
            class="search-input"
            bg-color="blue-grey-10"
          >
            <template v-slot:prepend>
              <q-icon name="search" color="primary" />
            </template>
            <template v-slot:append>
              <q-icon
                v-if="search"
                name="close"
                color="grey"
                class="cursor-pointer"
                @click="search = ''"
              />
            </template>
          </q-input>
        </div>
      </div>
    </section>

    <!-- 2. RESULTADOS (GRID) -->
    <div class="section-group self-stretch">
      <!-- Mensaje si no hay resultados -->
      <div v-if="filteredTerms.length === 0" class="text-center q-pa-xl text-grey-5">
        <q-icon name="sentiment_dissatisfied" size="4rem" class="q-mb-md" />
        <div class="text-h6">No encontramos definiciones para "{{ search }}"</div>
        <p>Intenta con otra palabra clave.</p>
      </div>

      <!-- Grid de Tarjetas -->
      <div class="glossary-grid">
        <transition-group name="list">
          <div
            v-for="term in filteredTerms"
            :key="term.name"
            class="glossary-card"
            :class="term.category"
          >
            <div class="card-header">
              <div class="term-name">{{ term.name }}</div>
              <q-badge :color="getCategoryColor(term.category)" class="term-badge">
                {{ term.category }}
              </q-badge>
            </div>
            <div class="term-def">
              {{ term.definition }}
            </div>
            <div class="term-context" v-if="term.context">
              <q-icon name="info" size="xs" class="q-mr-xs" /> {{ term.context }}
            </div>
          </div>
        </transition-group>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import TextBlock from 'components/content/TextBlock.vue';

// ESTADO DE BÚSQUEDA
const search = ref('');

// TIPOS
type Category = 'Core' | 'Comm' | 'Tools' | 'Sim' | 'Math';

interface Term {
  name: string;
  definition: string;
  category: Category;
  context?: string; // Ejemplo o uso
}

/// BASE DE DATOS DE TÉRMINOS (MASIVA)
const terms: Term[] = [
  // ===========================================================================
  // CATEGORÍA: CORE (Arquitectura, Ejecución, Sistema)
  // ===========================================================================
  {
    name: 'Node',
    category: 'Core',
    definition: 'La unidad mínima de procesamiento. Ejecutable que realiza una tarea específica.',
    context: 'Un robot tiene muchos nodos.',
  },
  {
    name: 'Package',
    category: 'Core',
    definition: 'Contenedor de código, configuración y dependencias. Unidad de instalación.',
    context: 'ros2 pkg create',
  },
  {
    name: 'Workspace',
    category: 'Core',
    definition: 'Directorio donde desarrollas y compilas tus paquetes (Overlay).',
    context: '~/ros2_ws',
  },
  {
    name: 'Underlay',
    category: 'Core',
    definition: 'La instalación base de ROS 2 sobre la cual se construye tu workspace.',
    context: '/opt/ros/jazzy',
  },
  {
    name: 'DDS',
    category: 'Core',
    definition: 'Data Distribution Service. Estándar de red industrial usado por ROS 2.',
    context: 'Middleware',
  },
  {
    name: 'Discovery',
    category: 'Core',
    definition: 'Proceso automático para encontrar nodos en la red sin servidor central.',
  },
  {
    name: 'Executor',
    category: 'Core',
    definition: 'Clase que coordina la ejecución de callbacks en uno o más hilos.',
    context: 'SingleThreadedExecutor',
  },
  {
    name: 'Callback',
    category: 'Core',
    definition: 'Función que se ejecuta en respuesta a un evento (mensaje recibido, timer).',
    context: 'Programación reactiva',
  },
  {
    name: 'Callback Group',
    category: 'Core',
    definition:
      'Agrupa callbacks para controlar su ejecución paralela (Reentrant vs Mutually Exclusive).',
  },
  {
    name: 'Lifecycle Node',
    category: 'Core',
    definition: 'Nodo con estados gestionados (Unconfigured, Inactive, Active, Finalized).',
    context: 'Nav2 lo usa mucho.',
  },
  {
    name: 'Managed Node',
    category: 'Core',
    definition: 'Sinónimo de Lifecycle Node. Nodo administrado por un orquestador.',
  },
  {
    name: 'Parameter',
    category: 'Core',
    definition: 'Variable de configuración de un nodo modificable en tiempo de ejecución.',
    context: 'Ej: max_speed',
  },
  {
    name: 'Parameter Server',
    category: 'Core',
    definition: 'Abstracción (en ROS 1 era un nodo, en ROS 2 es distribuido) para guardar configs.',
  },
  {
    name: 'Launch File',
    category: 'Core',
    definition: 'Script (Python/XML/YAML) para arrancar múltiples nodos con configuración.',
    context: 'ros2 launch',
  },
  {
    name: 'Component',
    category: 'Core',
    definition:
      'Nodo diseñado para cargarse dinámicamente en un contenedor en tiempo de ejecución.',
    context: 'Reduce uso de CPU.',
  },
  {
    name: 'Composition',
    category: 'Core',
    definition: 'Técnica de ejecutar múltiples nodos en un solo proceso para compartir memoria.',
  },
  {
    name: 'Domain ID',
    category: 'Core',
    definition: 'Número (0-232) que aísla redes de robots lógicamente en la misma red física.',
    context: 'export ROS_DOMAIN_ID=5',
  },
  {
    name: 'RMW',
    category: 'Core',
    definition: 'ROS Middleware Interface. Capa de abstracción para cambiar de implementación DDS.',
    context: 'rmw_cyclonedds',
  },
  {
    name: 'Client Library',
    category: 'Core',
    definition: 'Librerías de código que permiten escribir nodos (rclcpp, rclpy).',
  },
  {
    name: 'rclcpp',
    category: 'Core',
    definition: 'Biblioteca del cliente ROS 2 para C++.',
    context: 'Alto rendimiento',
  },
  {
    name: 'rclpy',
    category: 'Core',
    definition: 'Biblioteca del cliente ROS 2 para Python.',
    context: 'Prototipado rápido',
  },
  {
    name: 'Manifest',
    category: 'Core',
    definition: 'Archivo package.xml que define metadatos y dependencias.',
  },
  {
    name: 'Dependency',
    category: 'Core',
    definition: 'Paquete o librería necesaria para que otro paquete funcione.',
  },
  {
    name: 'Build System',
    category: 'Core',
    definition: 'Sistema que compila el código (ament_cmake, ament_python).',
  },
  {
    name: 'Overlay',
    category: 'Core',
    definition: 'Espacio de trabajo secundario que extiende al principal (Underlay).',
  },
  {
    name: 'Sourcing',
    category: 'Core',
    definition: 'Acción de cargar las variables de entorno de ROS en la terminal.',
    context: 'source install/setup.bash',
  },
  {
    name: 'Namespace',
    category: 'Core',
    definition:
      'Prefijo aplicado a nodos y tópicos para agruparlos o evitar colisiones de nombres.',
    context: '/robot1/camera',
  },
  {
    name: 'Remapping',
    category: 'Core',
    definition: 'Cambiar el nombre de un tópico, servicio o nodo al momento de la ejecución.',
    context: 'scan:=/scan_front',
  },
  {
    name: 'Entry Point',
    category: 'Core',
    definition: 'Punto de entrada definido en setup.py para ejecutables Python.',
  },
  {
    name: 'Spinning',
    category: 'Core',
    definition: 'Acción de mantener el nodo vivo esperando y procesando callbacks.',
    context: 'rclpy.spin(node)',
  },
  {
    name: 'Wall Timer',
    category: 'Core',
    definition: 'Temporizador basado en el reloj del sistema (tiempo real).',
  },
  {
    name: 'Sim Time',
    category: 'Core',
    definition: 'Tiempo simulado publicado por Gazebo (/clock). Permite pausar/acelerar el tiempo.',
    context: 'use_sim_time=True',
  },
  {
    name: 'Rate',
    category: 'Core',
    definition: 'Frecuencia de ejecución de un bucle (Hz).',
    context: 'rate.sleep()',
  },
  {
    name: 'Logger',
    category: 'Core',
    definition: 'Sistema de registro de mensajes en consola (Info, Warn, Error).',
    context: 'get_logger().info()',
  },
  { name: 'Extension', category: 'Core', definition: 'Sufijo de archivo (.py, .cpp, .launch.py).' },
  {
    name: 'Environment Var',
    category: 'Core',
    definition: 'Variables de sistema que configuran ROS (ROS_DISTRO, AMENT_PREFIX_PATH).',
  },
  {
    name: 'Verbosity',
    category: 'Core',
    definition: 'Nivel de detalle de los logs (DEBUG, INFO, WARN, ERROR, FATAL).',
  },
  {
    name: 'Introspection',
    category: 'Core',
    definition: 'Capacidad de analizar el sistema en tiempo real (ver tópicos, nodos).',
  },
  {
    name: 'Context',
    category: 'Core',
    definition: 'Objeto que maneja el estado global de la librería rcl.',
  },
  {
    name: 'Guard Condition',
    category: 'Core',
    definition: 'Evento que despierta al ejecutor sin ser un mensaje o timer.',
  },
  {
    name: 'Deadlock',
    category: 'Core',
    definition: 'Bloqueo mutuo donde dos procesos esperan al otro indefinidamente.',
  },
  {
    name: 'Real-time',
    category: 'Core',
    definition: 'Garantía de ejecución de tareas en tiempos deterministas (Soft/Hard).',
  },
  {
    name: 'Determinism',
    category: 'Core',
    definition:
      'Propiedad de un sistema de producir siempre la misma salida ante la misma entrada.',
  },
  {
    name: 'Middleware',
    category: 'Core',
    definition: 'Software "pegamento" que conecta componentes de software y aplicaciones.',
  },
  {
    name: 'Shared Memory',
    category: 'Core',
    definition: 'Método de transporte de datos sin copiar (Zero-Copy) en la misma RAM.',
  },
  {
    name: 'WaitSet',
    category: 'Core',
    definition: 'Estructura eficiente para esperar múltiples eventos simultáneamente.',
  },

  // ===========================================================================
  // CATEGORÍA: COMM (Comunicaciones, Interfaces, Mensajes)
  // ===========================================================================
  {
    name: 'Topic',
    category: 'Comm',
    definition: 'Canal de streaming de datos unidireccional (Pub/Sub).',
    context: 'N a N comunicación.',
  },
  {
    name: 'Service',
    category: 'Comm',
    definition: 'Comunicación síncrona de solicitud/respuesta (Req/Res).',
    context: '1 a 1 comunicación.',
  },
  {
    name: 'Action',
    category: 'Comm',
    definition: 'Tarea de larga duración con feedback periódico y capacidad de cancelación.',
  },
  { name: 'Publisher', category: 'Comm', definition: 'Entidad que envía mensajes a un tópico.' },
  { name: 'Subscriber', category: 'Comm', definition: 'Entidad que recibe mensajes de un tópico.' },
  { name: 'Client', category: 'Comm', definition: 'Entidad que envía una petición a un servicio.' },
  {
    name: 'Server',
    category: 'Comm',
    definition: 'Entidad que provee un servicio y responde peticiones.',
  },
  { name: 'Goal', category: 'Comm', definition: 'El objetivo enviado a un servidor de acciones.' },
  {
    name: 'Feedback',
    category: 'Comm',
    definition: 'Datos de progreso enviados periódicamente por una acción.',
  },
  {
    name: 'Result',
    category: 'Comm',
    definition: 'Datos finales enviados al terminar una acción.',
  },
  {
    name: 'Interface',
    category: 'Comm',
    definition: 'Definición de estructura de datos (.msg, .srv, .action).',
  },
  {
    name: 'IDL',
    category: 'Comm',
    definition: 'Interface Definition Language. Lenguaje neutral para definir tipos de datos.',
  },
  {
    name: 'Basic Types',
    category: 'Comm',
    definition: 'Tipos primitivos: bool, int32, float64, string, array.',
  },
  {
    name: 'Header',
    category: 'Comm',
    definition: 'Parte estándar de un mensaje con timestamp y frame_id.',
  },
  {
    name: 'std_msgs',
    category: 'Comm',
    definition: 'Paquete de mensajes básicos (String, Int32, Empty).',
  },
  {
    name: 'geometry_msgs',
    category: 'Comm',
    definition: 'Mensajes espaciales (Point, Pose, Twist, Transform).',
  },
  {
    name: 'sensor_msgs',
    category: 'Comm',
    definition: 'Datos de sensores (LaserScan, Image, Imu, PointCloud2).',
  },
  {
    name: 'nav_msgs',
    category: 'Comm',
    definition: 'Datos de navegación (Odometry, OccupancyGrid, Path).',
  },
  {
    name: 'Twist',
    category: 'Comm',
    definition: 'Mensaje de velocidad (lineal y angular).',
    context: 'cmd_vel usa esto.',
  },
  {
    name: 'Pose',
    category: 'Comm',
    definition: 'Posición (x,y,z) y Orientación (quaternion) de un objeto.',
  },
  {
    name: 'Odometry Msg',
    category: 'Comm',
    definition: 'Mensaje que combina Pose y Twist con covarianza.',
  },
  { name: 'LaserScan', category: 'Comm', definition: 'Datos de un barrido LIDAR 2D plano.' },
  {
    name: 'PointCloud2',
    category: 'Comm',
    definition: 'Nube de puntos 3D densa (LIDAR 3D o cámara de profundidad).',
  },
  { name: 'Image', category: 'Comm', definition: 'Matriz de píxeles cruda de una cámara.' },
  {
    name: 'CompressedImage',
    category: 'Comm',
    definition: 'Imagen comprimida (JPEG/PNG) para ahorrar ancho de banda.',
  },
  {
    name: 'QoS',
    category: 'Comm',
    definition: 'Quality of Service. Políticas de entrega de datos.',
  },
  {
    name: 'Reliability',
    category: 'Comm',
    definition: 'Política QoS: Reliable (TCP-like) vs Best Effort (UDP-like).',
  },
  {
    name: 'Durability',
    category: 'Comm',
    definition:
      'Política QoS: Volatile (olvida) vs Transient Local (recuerda último para nuevos subs).',
  },
  {
    name: 'History',
    category: 'Comm',
    definition: 'Política QoS: Keep Last (cola fija) vs Keep All (guarda todo).',
  },
  { name: 'Depth', category: 'Comm', definition: 'Tamaño de la cola de mensajes en QoS.' },
  {
    name: 'Liveliness',
    category: 'Comm',
    definition: 'Política QoS para detectar si un nodo sigue vivo.',
  },
  {
    name: 'Deadline',
    category: 'Comm',
    definition: 'Política QoS: Tiempo máximo esperado entre mensajes.',
  },
  {
    name: 'Zero Copy',
    category: 'Comm',
    definition: 'Transferencia de datos pasando punteros en lugar de copiar bytes en memoria.',
  },
  {
    name: 'Serialization',
    category: 'Comm',
    definition: 'Proceso de convertir un objeto en bytes para transmitirlo.',
  },
  { name: 'Deserialization', category: 'Comm', definition: 'Proceso inverso: bytes a objeto.' },
  { name: 'Request', category: 'Comm', definition: 'Parte de la petición en un archivo .srv.' },
  { name: 'Response', category: 'Comm', definition: 'Parte de la respuesta en un archivo .srv.' },
  {
    name: 'Handshake',
    category: 'Comm',
    definition: 'Proceso de negociación inicial entre nodos.',
  },
  {
    name: 'Multicast',
    category: 'Comm',
    definition: 'Envío de paquetes a un grupo de receptores simultáneamente.',
  },
  { name: 'Unicast', category: 'Comm', definition: 'Envío de paquetes a un único receptor.' },
  {
    name: 'Throughput',
    category: 'Comm',
    definition: 'Cantidad de datos transmitidos por segundo.',
  },
  {
    name: 'Latency',
    category: 'Comm',
    definition: 'Retardo temporal en la transmisión de un mensaje.',
  },
  { name: 'Jitter', category: 'Comm', definition: 'Variación en la latencia de los paquetes.' },
  {
    name: 'Bridge',
    category: 'Comm',
    definition:
      'Software que conecta ROS 1 con ROS 2 o ROS 2 con otros protocolos (MQTT, WebSocket).',
  },
  { name: 'Rosbridge', category: 'Comm', definition: 'Protocolo JSON para conectar webs con ROS.' },
  {
    name: 'Bag',
    category: 'Comm',
    definition: 'Formato de archivo para grabar y reproducir mensajes.',
  },

  // ===========================================================================
  // CATEGORÍA: TOOLS (Herramientas, CLI, Debugging)
  // ===========================================================================
  {
    name: 'Colcon',
    category: 'Tools',
    definition: 'Collective Construction. Herramienta de compilación.',
  },
  {
    name: 'Rosdep',
    category: 'Tools',
    definition: 'Instalador de dependencias del sistema operativo.',
  },
  {
    name: 'Bloom',
    category: 'Tools',
    definition: 'Herramienta para liberar paquetes binarios (.deb).',
  },
  {
    name: 'Ros2cli',
    category: 'Tools',
    definition: 'Conjunto de comandos de terminal (ros2 ...).',
  },
  {
    name: 'ros2 run',
    category: 'Tools',
    definition: 'Ejecuta un nodo específico de un paquete.',
    context: 'ros2 run <pkg> <exe>',
  },
  {
    name: 'ros2 launch',
    category: 'Tools',
    definition: 'Ejecuta un archivo de lanzamiento.',
    context: 'ros2 launch <pkg> <file>',
  },
  {
    name: 'ros2 topic list',
    category: 'Tools',
    definition: 'Lista los tópicos activos actualmente.',
  },
  {
    name: 'ros2 topic echo',
    category: 'Tools',
    definition: 'Imprime en pantalla los mensajes de un tópico.',
  },
  {
    name: 'ros2 topic hz',
    category: 'Tools',
    definition: 'Mide la frecuencia de publicación de un tópico.',
  },
  {
    name: 'ros2 topic bw',
    category: 'Tools',
    definition: 'Mide el ancho de banda usado por un tópico.',
  },
  { name: 'ros2 node list', category: 'Tools', definition: 'Lista los nodos activos.' },
  {
    name: 'ros2 node info',
    category: 'Tools',
    definition: 'Muestra detalles de un nodo (suscriptores, publicadores, servicios).',
  },
  {
    name: 'ros2 param set',
    category: 'Tools',
    definition: 'Cambia un parámetro en caliente.',
    context: 'ros2 param set <node> <param> <val>',
  },
  {
    name: 'ros2 service call',
    category: 'Tools',
    definition: 'Llama a un servicio manualmente desde terminal.',
  },
  {
    name: 'ros2 bag record',
    category: 'Tools',
    definition: 'Graba tópicos en un archivo rosbag (formato MCAP/SQLite).',
  },
  { name: 'ros2 bag play', category: 'Tools', definition: 'Reproduce un archivo rosbag grabado.' },
  {
    name: 'ros2 doctor',
    category: 'Tools',
    definition: 'Herramienta de diagnóstico para buscar problemas en la instalación o red.',
  },
  { name: 'RQt', category: 'Tools', definition: 'GUI framework basado en Qt con plugins.' },
  {
    name: 'rqt_graph',
    category: 'Tools',
    definition: 'Visualiza el grafo de nodos y tópicos como un diagrama.',
  },
  {
    name: 'rqt_plot',
    category: 'Tools',
    definition: 'Grafica valores numéricos de tópicos en el tiempo (XY plot).',
  },
  { name: 'rqt_console', category: 'Tools', definition: 'Visor avanzado de logs con filtros.' },
  { name: 'rqt_image_view', category: 'Tools', definition: 'Visor simple de cámaras.' },
  {
    name: 'rqt_reconfigure',
    category: 'Tools',
    definition: 'GUI para modificar parámetros dinámicamente.',
  },
  {
    name: 'RViz2',
    category: 'Tools',
    definition: 'ROS Visualization. Visualizador 3D de datos del robot.',
  },
  {
    name: 'Foxglove Studio',
    category: 'Tools',
    definition: 'Alternativa moderna y web a RViz y RQt.',
  },
  {
    name: 'PlotJuggler',
    category: 'Tools',
    definition: 'Herramienta poderosa para visualizar series temporales de datos.',
  },
  {
    name: 'GDB',
    category: 'Tools',
    definition: 'GNU Debugger. Para depurar código C++ (crashes, segfaults).',
  },
  {
    name: 'Valgrind',
    category: 'Tools',
    definition: 'Herramienta para detectar fugas de memoria.',
  },
  { name: 'PDB', category: 'Tools', definition: 'Python Debugger.' },
  { name: 'VS Code', category: 'Tools', definition: 'IDE recomendado con extensiones de ROS.' },
  {
    name: 'Dev Container',
    category: 'Tools',
    definition: 'Entorno de desarrollo dockerizado para VS Code.',
  },
  {
    name: 'Docker',
    category: 'Tools',
    definition: 'Plataforma de contenedores. Muy usada para desplegar ROS.',
  },
  {
    name: 'CI/CD',
    category: 'Tools',
    definition: 'Integración y Despliegue Continuo (GitHub Actions, GitLab CI).',
  },
  {
    name: 'Linter',
    category: 'Tools',
    definition: 'Analizador de código estático (ament_lint, flake8, cpplint).',
  },
  {
    name: 'Unit Test',
    category: 'Tools',
    definition: 'Prueba de una unidad pequeña de código (gtest, pytest).',
  },
  {
    name: 'Integration Test',
    category: 'Tools',
    definition: 'Prueba de múltiples componentes funcionando juntos.',
  },
  {
    name: 'Mock',
    category: 'Tools',
    definition: 'Objeto simulado usado en tests para imitar comportamiento.',
  },
  {
    name: 'Alias',
    category: 'Tools',
    definition: 'Atajo de terminal (ej: `sb` para `source ~/.bashrc`).',
  },
  { name: 'Htop', category: 'Tools', definition: 'Monitor de procesos del sistema.' },
  { name: 'Netcat', category: 'Tools', definition: 'Herramienta de red para debuguear puertos.' },
  {
    name: 'Wireshark',
    category: 'Tools',
    definition: 'Analizador de paquetes de red. Útil para debuguear DDS.',
  },

  // ===========================================================================
  // CATEGORÍA: SIM (Simulación, Modelado, Mundos)
  // ===========================================================================
  {
    name: 'Gazebo Classic',
    category: 'Sim',
    definition: 'Versión antigua de Gazebo (v11). Monolítica.',
  },
  {
    name: 'Gazebo Harmonic',
    category: 'Sim',
    definition: 'Nueva generación de Gazebo (antes Ignition). Modular.',
  },
  {
    name: 'Webots',
    category: 'Sim',
    definition: 'Simulador de robots open source alternativo a Gazebo.',
  },
  { name: 'CoppeliaSim', category: 'Sim', definition: 'Simulador versátil (antes V-REP).' },
  {
    name: 'Isaac Sim',
    category: 'Sim',
    definition: 'Simulador fotorrealista de NVIDIA con soporte RTX.',
  },
  {
    name: 'URDF',
    category: 'Sim',
    definition: 'Unified Robot Description Format. XML estándar para robots.',
  },
  {
    name: 'SDF',
    category: 'Sim',
    definition: 'Simulation Description Format. Formato nativo de Gazebo (incluye mundos).',
  },
  {
    name: 'Xacro',
    category: 'Sim',
    definition: 'XML Macros. Lenguaje para limpiar y modularizar URDFs.',
  },
  {
    name: 'Mesh',
    category: 'Sim',
    definition: 'Modelo 3D de la piel del robot (.stl, .dae, .obj).',
  },
  {
    name: 'Collision Mesh',
    category: 'Sim',
    definition: 'Modelo simplificado usado para cálculos físicos rápidos.',
  },
  {
    name: 'Visual Mesh',
    category: 'Sim',
    definition: 'Modelo detallado usado solo para renderizado gráfico.',
  },
  { name: 'Link', category: 'Sim', definition: 'Parte rígida del robot (cuerpo, rueda).' },
  { name: 'Joint', category: 'Sim', definition: 'Unión móvil entre dos links.' },
  {
    name: 'Joint Type',
    category: 'Sim',
    definition: 'Tipo de movimiento: revolute, continuous, prismatic, fixed, floating.',
  },
  {
    name: 'Plugin',
    category: 'Sim',
    definition: 'Código C++ inyectado en el simulador para dar funcionalidad (ej: sensor).',
  },
  {
    name: 'Model Plugin',
    category: 'Sim',
    definition: 'Plugin que controla un modelo específico.',
  },
  { name: 'World Plugin', category: 'Sim', definition: 'Plugin que controla el entorno global.' },
  { name: 'Sensor Plugin', category: 'Sim', definition: 'Simula datos de cámara, lidar o IMU.' },
  {
    name: 'Spawn',
    category: 'Sim',
    definition: 'Acción de instanciar un modelo en el mundo simulado.',
  },
  {
    name: 'Physics Engine',
    category: 'Sim',
    definition: 'Motor matemático de física (ODE, Bullet, Dart).',
  },
  {
    name: 'Inertia',
    category: 'Sim',
    definition: 'Matriz que define la resistencia de un cuerpo a rotar.',
  },
  { name: 'Friction', category: 'Sim', definition: 'Coeficiente de rozamiento (mu1, mu2).' },
  {
    name: 'Collision Bitmask',
    category: 'Sim',
    definition: 'Filtro para decidir qué objetos chocan con cuáles.',
  },
  {
    name: 'RTF',
    category: 'Sim',
    definition: 'Real Time Factor. Velocidad de simulación respecto al tiempo real (1.0 = igual).',
  },
  {
    name: 'Headless',
    category: 'Sim',
    definition: 'Ejecutar simulación sin interfaz gráfica (ahorra GPU).',
  },
  {
    name: 'ros_gz_bridge',
    category: 'Sim',
    definition: 'Puente oficial entre ROS 2 y Gazebo moderno.',
  },
  {
    name: 'World File',
    category: 'Sim',
    definition: 'Archivo .sdf que define el entorno (luces, suelo, muros).',
  },
  {
    name: 'Actor',
    category: 'Sim',
    definition: 'Entidad en simulación con animación predefinida (ej: persona caminando).',
  },
  {
    name: 'Joint State Publisher',
    category: 'Sim',
    definition: 'Nodo que publica el estado de las articulaciones (simulado o real).',
  },
  {
    name: 'Robot State Publisher',
    category: 'Sim',
    definition: 'Nodo que lee URDF y Joint States para publicar TFs.',
  },
  {
    name: 'Controller Manager',
    category: 'Sim',
    definition: 'Nodo de ros2_control que gestiona controladores.',
  },
  {
    name: 'Diff Drive',
    category: 'Sim',
    definition: 'Plugin para robots de dos ruedas diferenciales.',
  },
  { name: 'Ackermann', category: 'Sim', definition: 'Geometría de dirección tipo coche.' },
  {
    name: 'Omnidirectional',
    category: 'Sim',
    definition: 'Robot capaz de moverse en cualquier dirección (ruedas Mecanum).',
  },

  // ===========================================================================
  // CATEGORÍA: MATH (Matemáticas, Navegación, IA)
  // ===========================================================================
  // --- CINEMÁTICA Y GEOMETRÍA ---
  { name: 'TF2', category: 'Math', definition: 'Librería de transformaciones de coordenadas.' },
  { name: 'Frame', category: 'Math', definition: 'Sistema de referencia (ejes X,Y,Z).' },
  { name: 'Child Frame', category: 'Math', definition: 'Marco que se mueve respecto a un Padre.' },
  {
    name: 'Parent Frame',
    category: 'Math',
    definition: 'Marco de referencia estático relativo al Hijo.',
  },
  {
    name: 'Tree',
    category: 'Math',
    definition: 'Estructura jerárquica de TFs. No puede haber ciclos.',
  },
  { name: 'Translation', category: 'Math', definition: 'Desplazamiento lineal (x, y, z).' },
  { name: 'Rotation', category: 'Math', definition: 'Orientación angular.' },
  {
    name: 'Quaternion',
    category: 'Math',
    definition: 'Representación de rotación (x,y,z,w) sin bloqueo de cardán.',
  },
  {
    name: 'Euler Angles',
    category: 'Math',
    definition: 'Representación de rotación (Roll, Pitch, Yaw). Intuitiva pero problemática.',
  },
  { name: 'Roll', category: 'Math', definition: 'Rotación eje X (Alabeo).' },
  { name: 'Pitch', category: 'Math', definition: 'Rotación eje Y (Cabeceo).' },
  { name: 'Yaw', category: 'Math', definition: 'Rotación eje Z (Guiñada - Dirección).' },
  {
    name: 'Gimbal Lock',
    category: 'Math',
    definition: 'Pérdida de un grado de libertad en ángulos de Euler.',
  },
  { name: 'Matrix 3x3', category: 'Math', definition: 'Matriz de rotación.' },
  {
    name: 'Matrix 4x4',
    category: 'Math',
    definition: 'Matriz de transformación homogénea (Rotación + Traslación).',
  },
  { name: 'Vector', category: 'Math', definition: 'Entidad con magnitud y dirección.' },
  {
    name: 'Forward Kinematics',
    category: 'Math',
    definition: 'Calcular la posición final del efector dados los ángulos de joints.',
  },
  {
    name: 'Inverse Kinematics',
    category: 'Math',
    definition: 'Calcular los ángulos necesarios para llegar a una posición final.',
  },
  {
    name: 'Jacobian',
    category: 'Math',
    definition: 'Matriz que relaciona velocidades articulares con velocidad del efector.',
  },
  {
    name: 'Singularity',
    category: 'Math',
    definition: 'Posición donde el robot pierde grados de libertad o control.',
  },
  {
    name: 'DOF',
    category: 'Math',
    definition: 'Degrees of Freedom. Número de movimientos independientes posibles.',
  },

  // --- NAVEGACIÓN (NAV2) ---
  {
    name: 'Nav2',
    category: 'Math',
    definition: 'Navigation Stack 2. Framework de navegación autónoma.',
  },
  { name: 'SLAM', category: 'Math', definition: 'Simultaneous Localization and Mapping.' },
  { name: 'Odometry', category: 'Math', definition: 'Estimación de posición por encoders/IMU.' },
  {
    name: 'Localization',
    category: 'Math',
    definition: 'Determinar la posición del robot en un mapa conocido.',
  },
  {
    name: 'AMCL',
    category: 'Math',
    definition: 'Adaptive Monte Carlo Localization. Algoritmo probabilístico de localización 2D.',
  },
  {
    name: 'Particle Filter',
    category: 'Math',
    definition: 'Algoritmo usado por AMCL. Nube de posibles posiciones.',
  },
  { name: 'Map', category: 'Math', definition: 'Representación del entorno.' },
  {
    name: 'Occupancy Grid',
    category: 'Math',
    definition: 'Mapa de celdas donde 0=libre, 100=ocupado, -1=desconocido.',
  },
  {
    name: 'Costmap',
    category: 'Math',
    definition: 'Mapa de costes. Agrega obstáculos inflados alrededor de los reales.',
  },
  {
    name: 'Global Costmap',
    category: 'Math',
    definition: 'Mapa estático completo para planificación larga.',
  },
  {
    name: 'Local Costmap',
    category: 'Math',
    definition: 'Mapa dinámico pequeño alrededor del robot (rolling window).',
  },
  {
    name: 'Inflation Layer',
    category: 'Math',
    definition: 'Capa que añade "grosor" a los obstáculos para seguridad.',
  },
  {
    name: 'Planner',
    category: 'Math',
    definition: 'Calcula la ruta global (Global Path) del punto A al B.',
  },
  {
    name: 'Controller',
    category: 'Math',
    definition: 'Calcula la velocidad (cmd_vel) para seguir la ruta global (Local Planner).',
  },
  {
    name: 'Behavior Tree',
    category: 'Math',
    definition: 'Árbol de Comportamiento. Lógica de decisiones compleja usada en Nav2.',
  },
  {
    name: 'Recovery',
    category: 'Math',
    definition: 'Acción para desatascar el robot (ej: girar, retroceder).',
  },
  { name: 'Waypoint', category: 'Math', definition: 'Punto intermedio en una ruta.' },
  { name: 'NavFn', category: 'Math', definition: 'Planificador global clásico (Dijkstra/A*).' },
  {
    name: 'Smac Planner',
    category: 'Math',
    definition: 'Planificador global moderno y suave para Nav2.',
  },
  { name: 'DWB', category: 'Math', definition: 'Controlador local (Dynamic Window Approach).' },
  {
    name: 'MPPI',
    category: 'Math',
    definition: 'Model Predictive Path Integral. Controlador predictivo avanzado.',
  },
  {
    name: 'Pure Pursuit',
    category: 'Math',
    definition: 'Algoritmo geométrico simple para seguimiento de rutas.',
  },
  {
    name: 'Footprint',
    category: 'Math',
    definition: 'Polígono que define la forma física del robot en el suelo.',
  },

  // --- CONTROL Y VISIÓN ---
  { name: 'PID', category: 'Math', definition: 'Controlador Proporcional-Integral-Derivativo.' },
  {
    name: 'Kalman Filter',
    category: 'Math',
    definition: 'Algoritmo para estimar variables de estado (fusión de sensores).',
  },
  {
    name: 'EKF',
    category: 'Math',
    definition: 'Extended Kalman Filter. Para sistemas no lineales.',
  },
  {
    name: 'Sensor Fusion',
    category: 'Math',
    definition: 'Combinar datos de múltiples fuentes (IMU+Odom) para reducir error.',
  },
  {
    name: 'Computer Vision',
    category: 'Math',
    definition: 'Procesamiento de imágenes para extraer información.',
  },
  { name: 'OpenCV', category: 'Math', definition: 'Librería estándar de visión artificial.' },
  {
    name: 'PCL',
    category: 'Math',
    definition: 'Point Cloud Library. Procesamiento de nubes de puntos 3D.',
  },
  {
    name: 'Calibration',
    category: 'Math',
    definition: 'Proceso de corregir errores sistemáticos de sensores (intrínsecos/extrínsecos).',
  },
  {
    name: 'Intrinsics',
    category: 'Math',
    definition: 'Parámetros internos de cámara (focal length, centro óptico).',
  },
  { name: 'Extrinsics', category: 'Math', definition: 'Posición de la cámara respecto al robot.' },
  {
    name: 'Rectification',
    category: 'Math',
    definition: 'Corrección de la distorsión de lente en una imagen.',
  },
  {
    name: 'Stereo Vision',
    category: 'Math',
    definition: 'Calcular profundidad usando dos cámaras.',
  },
  {
    name: 'Disparity',
    category: 'Math',
    definition: 'Diferencia de posición de un objeto entre imagen izq y der.',
  },
  {
    name: 'Optical Flow',
    category: 'Math',
    definition: 'Patrón de movimiento aparente de objetos en imagen.',
  },
  {
    name: 'YOLO',
    category: 'Math',
    definition: 'You Only Look Once. Red neuronal para detección de objetos.',
  },
  { name: 'TensorFlow', category: 'Math', definition: 'Framework de Machine Learning.' },
  { name: 'PyTorch', category: 'Math', definition: 'Framework de Deep Learning.' },
  {
    name: 'Reinforcement Learning',
    category: 'Math',
    definition: 'Aprendizaje por refuerzo. Entrenar robots mediante premios/castigos.',
  },
  {
    name: 'Simulation-to-Real',
    category: 'Math',
    definition: 'Sim2Real. Transferir modelos aprendidos en simulación al mundo real.',
  },

  // --- HARDWARE ---
  {
    name: 'Microcontroller',
    category: 'Core',
    definition: 'Pequeño ordenador para control de bajo nivel (ESP32, Arduino).',
  },
  { name: 'micro-ROS', category: 'Core', definition: 'Versión de ROS 2 para microcontroladores.' },
  {
    name: 'Agent',
    category: 'Core',
    definition: 'Intermediario entre micro-ROS y la red ROS 2 normal.',
  },
  {
    name: 'Driver',
    category: 'Core',
    definition: 'Software que habla con el hardware y publica mensajes ROS.',
  },
  { name: 'LIDAR', category: 'Core', definition: 'Light Detection and Ranging. Sensor láser.' },
  {
    name: 'IMU',
    category: 'Core',
    definition: 'Inertial Measurement Unit (Acelerómetro + Giroscopio).',
  },
  { name: 'Encoder', category: 'Core', definition: 'Sensor que mide la rotación de la rueda.' },
  {
    name: 'Actuator',
    category: 'Core',
    definition: 'Dispositivo que produce movimiento (motor, servo).',
  },
  {
    name: 'ros2_control',
    category: 'Core',
    definition: 'Framework para control de hardware en tiempo real.',
  },
  {
    name: 'Hardware Interface',
    category: 'Core',
    definition: 'Capa de abstracción C++ en ros2_control para hablar con el device.',
  },
  { name: 'Resource Manager', category: 'Core', definition: 'Gestor de recursos en ros2_control.' },
  {
    name: 'Transmission',
    category: 'Core',
    definition: 'Relación mecánica entre motor y articulación (reductora).',
  },
];

// LÓGICA DE FILTRADO
const filteredTerms = computed(() => {
  // CASO 1: Sin búsqueda
  if (!search.value) {
    // Usamos [...terms] para crear una COPIA y no modificar el original
    return [...terms].sort((a, b) => a.name.localeCompare(b.name));
  }

  // CASO 2: Con búsqueda
  const query = search.value.toLowerCase();

  // .filter() ya crea un nuevo array, así que aquí el .sort() es seguro
  return terms
    .filter(
      (t) => t.name.toLowerCase().includes(query) || t.definition.toLowerCase().includes(query),
    )
    .sort((a, b) => a.name.localeCompare(b.name));
});

// COLORES POR CATEGORÍA
function getCategoryColor(cat: Category): string {
  switch (cat) {
    case 'Core':
      return 'purple';
    case 'Comm':
      return 'orange';
    case 'Tools':
      return 'cyan';
    case 'Sim':
      return 'green';
    case 'Math':
      return 'red';
    default:
      return 'grey';
  }
}
</script>

<style scoped>
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1000px;
  margin: 0 auto 2rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(16, 185, 129, 0.1), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* BUSCADOR */
.search-container {
  max-width: 600px;
  margin: 24px auto 0;
}

.search-input :deep(.q-field__control) {
  background: rgba(30, 41, 59, 0.8) !important;
  border: 1px solid rgba(148, 163, 184, 0.3);
}

/* GRID DE GLOSARIO */
.glossary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
  gap: 20px;
}

.glossary-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 20px;
  display: flex;
  flex-direction: column;
  transition: all 0.2s ease;
}

.glossary-card:hover {
  transform: translateY(-2px);
  background: rgba(30, 41, 59, 0.7);
  border-color: rgba(255, 255, 255, 0.1);
}

/* BORDES SUPERIORES POR CATEGORÍA */
.glossary-card.Core {
  border-top: 3px solid #a855f7;
}
.glossary-card.Comm {
  border-top: 3px solid #f97316;
}
.glossary-card.Tools {
  border-top: 3px solid #06b6d4;
}
.glossary-card.Sim {
  border-top: 3px solid #22c55e;
}
.glossary-card.Math {
  border-top: 3px solid #ef4444;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
}

.term-name {
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
}

.term-def {
  color: #cbd5e1;
  font-size: 0.95rem;
  line-height: 1.5;
  margin-bottom: 12px;
  flex: 1;
}

.term-context {
  font-size: 0.8rem;
  color: #94a3b8;
  background: rgba(0, 0, 0, 0.2);
  padding: 6px 10px;
  border-radius: 6px;
  display: inline-flex;
  align-items: center;
}

/* ANIMACIÓN DE LISTA */
.list-enter-active,
.list-leave-active {
  transition: all 0.3s ease;
}
.list-enter-from,
.list-leave-to {
  opacity: 0;
  transform: scale(0.9);
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
