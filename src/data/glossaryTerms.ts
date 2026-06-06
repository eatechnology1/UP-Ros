// 1. Exportamos los tipos para poder usarlos en el componente
export type Category = 'Core' | 'Comm' | 'Tools' | 'Sim' | 'Math' | 'Security' | 'AI';

export interface Term {
  name: string;
  definition: string;
  category: Category;
  context?: string; // Opcional
}

// 2. Exportamos la constante masiva con "export const"
export const glossaryTerms: Term[] = [
  // ===========================================================================
  // CATEGORÍA: CORE (Arquitectura, Ejecución, Sistema, Hardware, Distribuciones)
  // ===========================================================================

  // --- ARQUITECTURA FUNDAMENTAL ---
  {
    name: 'Node',
    category: 'Core',
    definition:
      'Unidad mínima de cómputo en ROS 2. Proceso ejecutable independiente que realiza una tarea específica y bien delimitada: leer un sensor, calcular trayectorias, controlar motores, etc. Los nodos se descubren automáticamente en la red y se comunican mediante tópicos, servicios y acciones a través del middleware DDS. Un sistema robótico típico está compuesto por docenas de nodos cooperando.',
    context: 'ros2 run mi_pkg mi_nodo',
  },
  {
    name: 'Package',
    category: 'Core',
    definition:
      'Unidad de organización y distribución de código en ROS 2. Contiene código fuente, archivos de configuración, lanzadores, interfaces y dependencias. Es la unidad mínima de instalación y compilación. Se crea con `ros2 pkg create` y se describe con `package.xml`.',
    context: 'ros2 pkg create --build-type ament_cmake mi_pkg',
  },
  {
    name: 'Workspace',
    category: 'Core',
    definition:
      'Directorio raíz donde el desarrollador crea, compila y prueba sus paquetes ROS 2 (también llamado Overlay). Contiene las carpetas `src/` (código fuente), `build/` (artefactos de compilación), `install/` (resultado final) y `log/` (registros de colcon).',
    context: 'mkdir -p ~/ros2_ws/src && cd ~/ros2_ws',
  },
  {
    name: 'Underlay',
    category: 'Core',
    definition:
      'La instalación base de ROS 2 sobre la cual se construye el workspace del desarrollador. Al hacer `source install/setup.bash`, el workspace actual extiende (overlays) al underlay, heredando todos sus paquetes y configuraciones.',
    context: 'source /opt/ros/jazzy/setup.bash',
  },
  {
    name: 'DDS',
    category: 'Core',
    definition:
      'Data Distribution Service. Estándar de middleware industrial definido por la OMG (Object Management Group) que ROS 2 usa como capa de transporte. Implementa el patrón publish-subscribe con calidad de servicio configurable (QoS), descubrimiento automático de participantes y comunicación peer-to-peer sin broker central.',
    context: 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
  },
  {
    name: 'Discovery',
    category: 'Core',
    definition:
      'Proceso automático mediante el cual los nodos se encuentran entre sí en la red sin necesitar un servidor central (a diferencia del Master de ROS 1). DDS usa multicast UDP para anunciar su presencia. Puede configurarse como "Simple Discovery" (multicast) o "Discovery Server" (para redes complejas).',
    context: 'ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET',
  },
  {
    name: 'Executor',
    category: 'Core',
    definition:
      'Clase que gestiona y despacha la ejecución de todos los callbacks de un nodo (o grupo de nodos). Mantiene un bucle interno esperando eventos (mensajes, timers, servicios) y los ejecuta según su estrategia de scheduling. Tipos principales: SingleThreadedExecutor, MultiThreadedExecutor, StaticSingleThreadedExecutor.',
    context: 'rclcpp::spin(node) usa SingleThreadedExecutor internamente',
  },
  {
    name: 'Callback',
    category: 'Core',
    definition:
      'Función o método que el Executor llama automáticamente en respuesta a un evento: llegada de un mensaje en un tópico suscrito, expiración de un timer, recepción de una petición de servicio o respuesta de un cliente. Es el mecanismo central de la programación reactiva en ROS 2.',
    context: 'void mi_callback(const std_msgs::msg::String::SharedPtr msg)',
  },
  {
    name: 'Callback Group',
    category: 'Core',
    definition:
      'Mecanismo para agrupar callbacks y controlar si pueden ejecutarse en paralelo. "MutuallyExclusive" garantiza que solo un callback del grupo se ejecuta a la vez (seguro sin mutex). "Reentrant" permite ejecución paralela de callbacks del mismo grupo (requiere thread-safety).',
    context: 'create_callback_group(rclcpp::CallbackGroupType::Reentrant)',
  },
  {
    name: 'Lifecycle Node',
    category: 'Core',
    definition:
      'Nodo con una máquina de estados gestionada que define su ciclo de vida: Unconfigured → Inactive → Active → Finalized. Permite una secuencia de arranque/parada determinista y coordinada. Muy usado en Nav2 y sistemas de producción donde el orden de inicialización importa.',
    context: 'class MiNodo : public rclcpp_lifecycle::LifecycleNode',
  },
  {
    name: 'Managed Node',
    category: 'Core',
    definition:
      'Sinónimo de Lifecycle Node. Nodo cuyo ciclo de vida es administrado externamente por un orquestador (ej: nav2_lifecycle_manager). El orquestador envía transiciones de estado (configure, activate, deactivate) a través de servicios estándar.',
  },
  {
    name: 'Parameter',
    category: 'Core',
    definition:
      'Variable de configuración con nombre, tipo y valor que pertenece a un nodo específico. Puede leerse y modificarse en tiempo de ejecución sin reiniciar el nodo. Soporta tipos: bool, int, double, string, byte_array, bool_array, int_array, double_array, string_array.',
    context: 'declare_parameter("max_speed", 1.5)',
  },
  {
    name: 'Parameter Server',
    category: 'Core',
    definition:
      'En ROS 2, los parámetros son distribuidos: cada nodo es su propio servidor de parámetros. No existe un nodo central como en ROS 1. Cada nodo expone servicios estándar para get/set/list/describe sus parámetros, accesibles remotamente via `ros2 param`.',
  },
  {
    name: 'Launch File',
    category: 'Core',
    definition:
      'Script de orquestación para arrancar múltiples nodos, cargar parámetros y configurar el sistema completo. En ROS 2 se escribe preferentemente en Python (.launch.py), aunque también se soporta XML y YAML. Permite expresar condiciones, sustituciones, namespaces y grupos de forma programática.',
    context: 'ros2 launch mi_pkg mi_robot.launch.py',
  },
  {
    name: 'Component',
    category: 'Core',
    definition:
      'Nodo que se compila como una librería compartida (.so) en lugar de un ejecutable, permitiendo cargarse dinámicamente en un proceso contenedor (ComponentContainer). Múltiples componentes en el mismo proceso comparten memoria directamente (intra-process), eliminando copias innecesarias.',
    context: 'ros2 component load /ComponentManager mi_pkg mi_pkg::MiNodo',
  },
  {
    name: 'Composition',
    category: 'Core',
    definition:
      'Técnica de ejecutar múltiples nodos (como componentes) dentro de un único proceso del SO para aprovechar comunicación en memoria compartida. Reduce drásticamente la latencia y el uso de CPU al evitar serialización/deserialización y copias de datos entre procesos.',
  },
  {
    name: 'Domain ID',
    category: 'Core',
    definition:
      'Número entero (0-101 en la práctica, 0-232 teórico) que segmenta lógicamente la red DDS. Dos robots con distinto Domain ID son completamente invisibles entre sí aunque estén en la misma red física. Esencial para trabajar con múltiples robots o estudiantes en el mismo laboratorio.',
    context: 'export ROS_DOMAIN_ID=42',
  },
  {
    name: 'RMW',
    category: 'Core',
    definition:
      'ROS Middleware Interface. Capa de abstracción C que desacopla el código de ROS 2 de la implementación DDS concreta. Permite cambiar de Fast DDS a CycloneDDS o Connext simplemente cambiando una variable de entorno, sin recompilar el código de usuario.',
    context: 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
  },
  {
    name: 'Client Library',
    category: 'Core',
    definition:
      'Librería de programación que proporciona la API para crear nodos, publishers, subscribers, servicios y acciones en un lenguaje específico. La capa inferior `rcl` (ROS Client Library en C) es compartida; sobre ella se construyen `rclcpp` (C++) y `rclpy` (Python).',
  },
  {
    name: 'rclcpp',
    category: 'Core',
    definition:
      'Biblioteca cliente ROS 2 para C++. Ofrece el mayor rendimiento y control, siendo ideal para algoritmos de tiempo real, drivers de hardware y procesamiento intensivo. Usa smart pointers (shared_ptr) y tipos modernos de C++17. Es la librería de referencia del ecosistema.',
    context: '#include "rclcpp/rclcpp.hpp"',
  },
  {
    name: 'rclpy',
    category: 'Core',
    definition:
      'Biblioteca cliente ROS 2 para Python. Perfecta para prototipado rápido, scripts de automatización, interfaces de usuario e integración con ecosistemas de ML/AI (NumPy, PyTorch). Comparte la misma capa RCL que rclcpp, garantizando compatibilidad total de interfaces.',
    context: 'import rclpy; from rclpy.node import Node',
  },
  {
    name: 'Manifest',
    category: 'Core',
    definition:
      'Archivo `package.xml` que declara los metadatos del paquete: nombre, versión, descripción, mantenedores, licencia y todas las dependencias (build, exec, test). Es leído por herramientas como rosdep, colcon y bloom para gestionar el ecosistema de paquetes.',
    context: '<package format="3"> ... </package>',
  },
  {
    name: 'Dependency',
    category: 'Core',
    definition:
      'Paquete externo o librería del sistema necesaria para compilar, ejecutar o probar un paquete ROS 2. Se declaran en `package.xml` como `<depend>`, `<build_depend>`, `<exec_depend>` o `<test_depend>`. Se resuelven automáticamente con `rosdep install`.',
  },
  {
    name: 'Build System',
    category: 'Core',
    definition:
      'Sistema que automatiza la compilación del código fuente. ROS 2 usa `ament` como meta-build system, que internamente usa CMake para C/C++ (`ament_cmake`) o setuptools para Python (`ament_python`). Colcon orquesta la compilación de todos los paquetes del workspace respetando el orden de dependencias.',
  },
  {
    name: 'Overlay',
    category: 'Core',
    definition:
      'Workspace de desarrollo que extiende otro workspace base (Underlay). Al hacer source de su `setup.bash`, los paquetes del overlay tienen prioridad sobre los del underlay, permitiendo sobreescribir paquetes de la instalación base durante el desarrollo.',
  },
  {
    name: 'Sourcing',
    category: 'Core',
    definition:
      'Acción de ejecutar un script de configuración de entorno que define variables como `ROS_DISTRO`, `AMENT_PREFIX_PATH` y `PYTHONPATH` en la sesión de terminal actual. Indispensable antes de usar cualquier comando `ros2` o ejecutar nodos.',
    context: 'source ~/ros2_ws/install/setup.bash',
  },
  {
    name: 'Namespace',
    category: 'Core',
    definition:
      'Prefijo jerárquico aplicado a nodos, tópicos y servicios para organizarlos y evitar colisiones de nombres. Permite instanciar el mismo nodo múltiples veces con identidades diferentes (ej: /robot_1/camera, /robot_2/camera). Se puede definir en el launch file o en la línea de comandos.',
    context: '--ros-args -r __ns:=/robot_1',
  },
  {
    name: 'Remapping',
    category: 'Core',
    definition:
      'Técnica para redirigir el nombre de un tópico, servicio o nodo en tiempo de ejecución sin modificar el código. Permite conectar nodos que esperan nombres distintos de forma transparente. Se especifica con `--ros-args -r old_name:=new_name` o en el launch file con `remappings=`.',
    context: 'ros2 run pkg nodo --ros-args -r scan:=/scan_front',
  },
  {
    name: 'Entry Point',
    category: 'Core',
    definition:
      'Punto de entrada registrado en `setup.py` de un paquete Python que mapea un nombre ejecutable a una función `main()` de un módulo. Permite ejecutar nodos Python con `ros2 run` como si fueran binarios, gracias a los scripts generados durante la instalación.',
    context: 'entry_points={"console_scripts": ["mi_nodo = mi_pkg.mi_nodo:main"]}',
  },
  {
    name: 'Spinning',
    category: 'Core',
    definition:
      'Acción de entregar el control al Executor para que comience a procesar eventos (callbacks de tópicos, timers, servicios). Bloquea el hilo principal hasta que el nodo se destruya. Alternativas: `spin_once()` para procesar un único evento, o `spin_until_future_complete()` para esperar un resultado asíncrono.',
    context: 'rclpy.spin(node) / rclcpp::spin(node)',
  },
  {
    name: 'Wall Timer',
    category: 'Core',
    definition:
      'Temporizador periódico basado en el reloj real del sistema operativo (wall clock time). Dispara su callback con la frecuencia especificada independientemente del tiempo de simulación. Para sincronizarse con Gazebo, usar `create_timer()` con el reloj ROS (que respeta `use_sim_time`).',
    context: 'create_wall_timer(500ms, callback)',
  },
  {
    name: 'Sim Time',
    category: 'Core',
    definition:
      'Tiempo de simulación publicado por Gazebo en el tópico `/clock`. Cuando `use_sim_time=true`, todos los timers y timestamps del nodo usan este reloj virtual, permitiendo pausar, acelerar o retroceder el tiempo de simulación de forma transparente.',
    context: 'ros2 param set /mi_nodo use_sim_time true',
  },
  {
    name: 'Rate',
    category: 'Core',
    definition:
      'Objeto que regula la frecuencia de un bucle de control mediante pausas sincronizadas con el reloj ROS. Calcula el tiempo de sleep necesario para mantener la frecuencia objetivo, compensando el tiempo que tardó la iteración anterior. Soporta sim_time automáticamente.',
    context: 'rate = node.create_rate(10)  # 10 Hz',
  },
  {
    name: 'Logger',
    category: 'Core',
    definition:
      'Sistema de registro de mensajes con niveles de severidad jerarquicos: DEBUG < INFO < WARN < ERROR < FATAL. Integrado con `rcutils_logging`. Los mensajes pueden filtrarse por nivel, publicarse en el tópico `/rosout` para monitoreo centralizado, y guardarse en archivos.',
    context: 'node.get_logger().info("Sistema iniciado")',
  },
  {
    name: 'Environment Variable',
    category: 'Core',
    definition:
      'Variables de entorno del sistema operativo que configuran el comportamiento de ROS 2. Las más importantes: `ROS_DOMAIN_ID` (aislamiento de red), `RMW_IMPLEMENTATION` (selección de DDS), `ROS_DISTRO` (distribución activa), `AMENT_PREFIX_PATH` (rutas de paquetes instalados).',
    context: 'export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
  },
  {
    name: 'Verbosity',
    category: 'Core',
    definition:
      'Nivel de detalle en el registro de mensajes. La jerarquía es: DEBUG (todo), INFO (informativos), WARN (advertencias), ERROR (errores recuperables), FATAL (errores críticos). Se puede cambiar dinámicamente con `ros2 param set /nodo rcl_log_level 10` (10=DEBUG).',
  },
  {
    name: 'Introspection',
    category: 'Core',
    definition:
      'Capacidad del sistema ROS 2 de inspeccionar su propio estado en tiempo real: listar nodos activos, ver tópicos con sus tipos y QoS, verificar conexiones publisher/subscriber, inspeccionar servicios y acciones. Fundamental para debugging. Accesible vía CLI (`ros2 topic list`) o API.',
  },
  {
    name: 'Context',
    category: 'Core',
    definition:
      'Objeto singleton que gestiona el estado global de la instancia de `rcl` (inicialización, señales de shutdown, argumentos de ROS). Normalmente gestionado automáticamente por `rclcpp::init()` / `rclpy.init()`. Permite gestionar múltiples instancias ROS en el mismo proceso.',
  },
  {
    name: 'Guard Condition',
    category: 'Core',
    definition:
      'Mecanismo de bajo nivel en `rcl` para despertar al Executor desde otro hilo sin enviar un mensaje. Se usa internamente por timers, señales del OS y eventos del sistema. El usuario rara vez interactúa directamente con Guard Conditions; se usan más en la implementación del Executor.',
  },
  {
    name: 'Deadlock',
    category: 'Core',
    definition:
      'Bloqueo permanente donde dos o más callbacks esperan mutuamente recursos que el otro retiene. Causa común: llamar a un servicio síncrono desde dentro de un callback del mismo SingleThreadedExecutor. Solución: usar MultiThreadedExecutor o servicios asíncronos (`async_send_request`).',
  },
  {
    name: 'Real-time',
    category: 'Core',
    definition:
      'Propiedad de un sistema de garantizar la ejecución de tareas dentro de un plazo de tiempo determinista (deadline). "Hard real-time": fallar el deadline es inaceptable (control de vuelo). "Soft real-time": fallos ocasionales son tolerables (robots industriales). Requiere kernel PREEMPT_RT y programación cuidadosa.',
  },
  {
    name: 'Determinism',
    category: 'Core',
    definition:
      'Propiedad de un sistema computacional de producir siempre la misma secuencia de salidas dado el mismo conjunto de entradas, independientemente de cuándo se ejecute. Crítico en sistemas robóticos de seguridad. Amenazado por: garbage collection, caches de CPU, interrupciones del SO y memoria dinámica.',
  },
  {
    name: 'Middleware',
    category: 'Core',
    definition:
      'Capa de software que actúa como "pegamento" entre el código de aplicación (nodos ROS 2) y la red/hardware subyacente. ROS 2 delega toda la comunicación al middleware DDS, que gestiona descubrimiento, transporte, serialización y QoS de forma transparente al desarrollador.',
  },
  {
    name: 'Shared Memory',
    category: 'Core',
    definition:
      'Mecanismo de transporte de datos entre procesos en la misma máquina sin copiar bytes: se pasan referencias (punteros) a una región de memoria común. En ROS 2, habilitado vía iceoryx (Eclipse) o la API de intra-process communication para nodos en el mismo proceso. Logra latencias de microsegundos.',
    context: 'export CYCLONE_DDS_URI=file://config_shm.xml',
  },
  {
    name: 'WaitSet',
    category: 'Core',
    definition:
      'Estructura de datos de `rcl` que permite esperar eficientemente a que ocurra cualquiera de un conjunto de eventos (subscriptores con datos, timers expirados, guard conditions). El Executor la usa internamente. Evita busy-waiting, poniendo el hilo a dormir hasta que llegue un evento.',
  },

  // --- RMW Y DDS INTERNALS ---
  {
    name: 'rcl',
    category: 'Core',
    definition:
      'ROS Client Library en C puro. Capa de abstracción compartida entre todas las librerías cliente (rclcpp, rclpy, rcljava, etc.). Implementa la lógica de nodos, parámetros, logging y discovery sobre la capa RMW. Garantiza comportamiento consistente entre lenguajes.',
    context: '#include "rcl/rcl.h"',
  },
  {
    name: 'rmw_fastrtps',
    category: 'Core',
    definition:
      'Implementación RMW sobre eProsima Fast DDS (antes Fast RTPS). Es el middleware predeterminado en la mayoría de distribuciones ROS 2. Ofrece excelente rendimiento en redes locales y soporte completo del estándar DDS. Se configura mediante perfiles XML.',
    context: 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
  },
  {
    name: 'rmw_cyclonedds',
    category: 'Core',
    definition:
      'Implementación RMW sobre Eclipse CycloneDDS. Destacado por su baja latencia, facilidad de configuración y excelente comportamiento en redes WiFi. Preferido por muchos usuarios para robótica móvil y aplicaciones de tiempo real. Soportado oficialmente por OSRF.',
    context: 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
  },
  {
    name: 'rmw_connextdds',
    category: 'Core',
    definition:
      'Implementación RMW sobre RTI Connext DDS, la solución DDS comercial de RTI con licencia gratuita para uso no comercial. Ofrece herramientas avanzadas de monitorización, rendimiento certificado para sistemas safety-critical e integración con RTI Admin Console.',
    context: 'export RMW_IMPLEMENTATION=rmw_connextdds',
  },
  {
    name: 'eProsima Fast DDS',
    category: 'Core',
    definition:
      'Implementación open-source del estándar DDS/RTPS desarrollada por eProsima. Es el RMW predeterminado de ROS 2. Soporta múltiples transportes (UDP, TCP, Shared Memory), descubrimiento configurable, seguridad DDS y perfiles XML para configuración avanzada.',
    context: 'fastdds shm enable',
  },
  {
    name: 'CycloneDDS',
    category: 'Core',
    definition:
      'Implementación open-source de DDS desarrollada por Eclipse Foundation. Arquitectura optimizada para baja latencia y facilidad de uso. Proyecto Eclipse IoT de primer nivel. Preferido en aplicaciones donde la configuración simple y el rendimiento en WiFi son prioritarios.',
  },
  {
    name: 'RTPS',
    category: 'Core',
    definition:
      'Real-Time Publish-Subscribe. Protocolo de wire-level definido por la OMG que implementa el estándar DDS sobre UDP. Define cómo se intercambian los datos entre entidades DDS a nivel de paquete de red. ROS 2 usa RTPS versión 2.x. No confundir con DDS (nivel de API).',
  },
  {
    name: 'Participant',
    category: 'Core',
    definition:
      'Entidad DDS raíz que representa a un proceso (o nodo) en la red DDS. Cada participant tiene un GUID único y es responsable del descubrimiento de otras entidades (publishers, subscribers) dentro del mismo Domain ID. Un proceso puede tener múltiples participants.',
  },
  {
    name: 'TypeSupport',
    category: 'Core',
    definition:
      'Código generado automáticamente por `rosidl_generator` que serializa y deserializa mensajes ROS 2 hacia/desde el formato CDR (Common Data Representation) usado por DDS. Es el "glue code" entre las interfaces .msg/.srv/.action y el middleware DDS.',
  },
  {
    name: 'LoanedMessage',
    category: 'Core',
    definition:
      'Mecanismo de publicación Zero-Copy donde el publisher pide prestado un buffer del middleware en lugar de allocar memoria propia. Cuando se publica, el middleware toma posesión del buffer directamente, evitando una copia. Requiere un RMW compatible (iceoryx, Fast DDS SHM).',
    context: 'auto loaned = pub->borrow_loaned_message();',
  },
  {
    name: 'XTypes',
    category: 'Core',
    definition:
      'Extensible and Dynamic Topic Types (DDS-XTypes). Estándar OMG que permite evolucionar los tipos de mensajes sin romper la compatibilidad con versiones anteriores. Habilita la introspección de tipos en runtime y es la base del Type Introspection API de ROS 2.',
  },

  // --- EJECUCIÓN Y SCHEDULING ---
  {
    name: 'MultiThreadedExecutor',
    category: 'Core',
    definition:
      'Executor de rclcpp/rclpy que usa un pool de hilos para ejecutar callbacks concurrentemente. Permite que múltiples callbacks de distintos Callback Groups se ejecuten en paralelo. Requiere que los callbacks sean thread-safe o que estén en grupos MutuallyExclusive para evitar condiciones de carrera.',
    context: 'rclcpp::executors::MultiThreadedExecutor executor;',
  },
  {
    name: 'StaticSingleThreadedExecutor',
    category: 'Core',
    definition:
      'Executor optimizado de rclcpp que analiza el grafo de entidades del nodo una sola vez al inicio (en lugar de en cada iteración). Ofrece mejor rendimiento que el SingleThreadedExecutor estándar para nodos con muchos publishers/subscribers, a costa de no soportar cambios dinámicos en las entidades.',
  },
  {
    name: 'EventsExecutor',
    category: 'Core',
    definition:
      'Executor experimental de rclcpp basado en eventos del sistema operativo (epoll/kqueue) en lugar de polling activo del WaitSet. Prometedor para latencias menores en sistemas con muchas entidades. Disponible como executor alternativo desde ROS 2 Iron.',
  },
  {
    name: 'Intra-process Communication',
    category: 'Core',
    definition:
      'Optimización que permite a publishers y subscribers en el mismo proceso intercambiar mensajes pasando punteros de memoria en lugar de serializar/deserializar y copiar datos. Se activa automáticamente cuando publisher y subscriber están en el mismo proceso y usan Smart Pointers. Reduce latencia y uso de CPU drásticamente.',
    context: 'rclcpp::NodeOptions().use_intra_process_comms(true)',
  },
  {
    name: 'Spin Once',
    category: 'Core',
    definition:
      'Variante de spinning que procesa un único callback pendiente y retorna el control al llamador. Útil para integrar ROS 2 en bucles de control propios donde no se puede ceder el hilo principal. `spin_once(timeout)` permite especificar un timeout de espera.',
    context: 'rclcpp::spin_some(node)',
  },
  {
    name: 'Future (async)',
    category: 'Core',
    definition:
      'Objeto que representa el resultado eventual de una operación asíncrona (como una llamada a servicio o acción). Permite hacer llamadas no bloqueantes con `async_send_request()` y esperar el resultado más tarde con `spin_until_future_complete()` sin bloquear el executor.',
    context: 'auto future = client->async_send_request(request);',
  },
  {
    name: 'Race Condition',
    category: 'Core',
    definition:
      'Error de concurrencia donde el resultado de una operación depende del orden de ejecución de múltiples hilos, que es no determinista. En ROS 2, ocurre cuando se usa MultiThreadedExecutor con callbacks que acceden a estado compartido sin protección de mutex.',
  },
  {
    name: 'ROSClock',
    category: 'Core',
    definition:
      'Reloj del nodo que puede operar en dos modos: "ROS_TIME" (usa el tiempo publicado en /clock cuando `use_sim_time=true`, o tiempo real en caso contrario) y "SYSTEM_TIME" (siempre tiempo real). Todos los timers y timestamps deberían usar este reloj para ser compatibles con simulación.',
  },
  {
    name: 'Duration',
    category: 'Core',
    definition:
      'Tipo de datos que representa un intervalo de tiempo como nanosegundos enteros de 64 bits. Usado en timers, timeouts y diferencias de timestamps. En rclcpp: `rclcpp::Duration`. En rclpy: `rclpy.duration.Duration`. Soporta aritmética con objetos Time.',
    context: 'rclcpp::Duration(std::chrono::seconds(5))',
  },

  // --- PAQUETES Y BUILD SYSTEM ---
  {
    name: 'ament_cmake',
    category: 'Core',
    definition:
      'Sistema de build para paquetes C/C++ en ROS 2. Es una extensión de CMake que añade macros y funciones específicas de ROS para instalar ejecutables, exportar interfaces, registrar pruebas y crear configuraciones de paquetes. El `build_type` más común para código C++.',
    context: '<build_type>ament_cmake</build_type>',
  },
  {
    name: 'ament_python',
    category: 'Core',
    definition:
      'Sistema de build para paquetes Python puros en ROS 2. Usa `setuptools` y `setup.py` para instalar módulos Python y registrar entry points como ejecutables de ROS. Más simple que ament_cmake pero limitado a Python.',
    context: '<build_type>ament_python</build_type>',
  },
  {
    name: 'ament_cmake_python',
    category: 'Core',
    definition:
      'Extensión de ament_cmake que permite combinar código C++ y Python en el mismo paquete. Usa `ament_python_install_package()` en CMakeLists.txt para instalar módulos Python junto con los binarios C++. Útil para paquetes con extensiones Python de código nativo.',
  },
  {
    name: 'CMakeLists.txt',
    category: 'Core',
    definition:
      'Archivo de configuración de compilación para paquetes ament_cmake. Define targets (ejecutables, librerías), dependencias, reglas de instalación y tests. En ROS 2, usa macros como `find_package(ament_cmake)`, `ament_target_dependencies()` y `ament_package()` al final.',
    context: 'ament_target_dependencies(mi_nodo rclcpp std_msgs)',
  },
  {
    name: 'package.xml',
    category: 'Core',
    definition:
      'Archivo XML de metadatos del paquete ROS (formato 3). Contiene: nombre, versión, descripción, mantenedores, licencia y todas las dependencias. Las dependencias se clasifican en `<depend>` (build+exec), `<build_depend>` (solo build), `<exec_depend>` (solo ejecución) y `<test_depend>` (solo tests).',
    context: '<depend>rclcpp</depend>',
  },
  {
    name: 'colcon build',
    category: 'Core',
    definition:
      'Comando principal del sistema de build de ROS 2 que compila todos los paquetes en el workspace respetando el orden de dependencias. Opciones clave: `--packages-select` (compilar paquetes específicos), `--symlink-install` (no copiar archivos Python, útil para desarrollo), `--cmake-args` (pasar opciones a CMake).',
    context: 'colcon build --symlink-install --packages-select mi_pkg',
  },
  {
    name: 'AMENT_PREFIX_PATH',
    category: 'Core',
    definition:
      'Variable de entorno que lista los directorios de instalación de paquetes ROS 2 (separados por ":"). Usada por `ament_index` para encontrar paquetes instalados. Se actualiza automáticamente al hacer source de `setup.bash`. Equivalente a `ROS_PACKAGE_PATH` de ROS 1.',
    context: 'echo $AMENT_PREFIX_PATH',
  },
  {
    name: 'COLCON_IGNORE',
    category: 'Core',
    definition:
      'Archivo vacío que, cuando se coloca en un directorio dentro de `src/`, le indica a colcon que ignore ese directorio y no intente compilar los paquetes que contiene. Útil para excluir temporalmente paquetes con errores o paquetes de terceros que no necesitas compilar.',
    context: 'touch src/paquete_roto/COLCON_IGNORE',
  },
  {
    name: 'Shadow Build',
    category: 'Core',
    definition:
      'Estrategia de compilación donde los artefactos de build se generan en un directorio separado del código fuente (el directorio `build/`). Mantiene el directorio `src/` limpio y permite compilar la misma fuente con múltiples configuraciones (Debug/Release) simultáneamente.',
  },
  {
    name: 'Symlink Install',
    category: 'Core',
    definition:
      'Opción de colcon (`--symlink-install`) que crea enlaces simbólicos en `install/` apuntando a los archivos fuente en lugar de copiarlos. Para scripts Python y archivos de configuración, los cambios son inmediatos sin recompilar. Esencial para desarrollo rápido.',
    context: 'colcon build --symlink-install',
  },

  // --- PARÁMETROS Y LAUNCH AVANZADO ---
  {
    name: 'Parameter Descriptor',
    category: 'Core',
    definition:
      'Estructura que enriquece la declaración de un parámetro con metadatos adicionales: descripción textual, rango de valores válidos (for_integer: min/max/step), valores de lista válidos (for_string), y si es read-only. Mejora la documentación y permite validación automática.',
    context: 'rcl_interfaces::msg::ParameterDescriptor desc; desc.description = "Velocidad máxima";',
  },
  {
    name: 'Parameter Event',
    category: 'Core',
    definition:
      'Mensaje publicado en el tópico `/parameter_events` cada vez que un parámetro de cualquier nodo cambia de valor. Permite a otros nodos monitorizar cambios de parámetros del sistema de forma reactiva sin hacer polling.',
  },
  {
    name: 'Declare Parameter',
    category: 'Core',
    definition:
      'Acción obligatoria (en modo strict) para registrar un parámetro en un nodo antes de poder usarlo. Define el nombre, tipo y valor por defecto. Si `allow_undeclared_parameters=false` (por defecto), intentar acceder a un parámetro no declarado lanza una excepción.',
    context: 'declare_parameter<double>("max_vel", 1.0)',
  },
  {
    name: 'Parameter File (YAML)',
    category: 'Core',
    definition:
      'Archivo YAML que almacena los valores de parámetros de uno o más nodos. Formato: `nombre_nodo: { ros__parameters: { param1: val1 } }`. Se carga en el launch file con `parameters=[...path...]` o en la línea de comandos con `--params-file`. Permite configurar sistemas completos sin tocar el código.',
    context: 'ros2 run pkg nodo --ros-args --params-file config/params.yaml',
  },
  {
    name: 'DeclareLaunchArgument',
    category: 'Core',
    definition:
      'Acción de launch que declara un argumento del launch file que puede pasarse desde la línea de comandos. Permite crear launch files configurables. Soporta valor por defecto y descripción. Accesible dentro del launch file con `LaunchConfiguration("nombre")`.',
    context: 'DeclareLaunchArgument("use_sim_time", default_value="true")',
  },
  {
    name: 'IncludeLaunchDescription',
    category: 'Core',
    definition:
      'Acción de launch que incluye y ejecuta otro launch file desde el actual. Permite modularizar sistemas complejos: un launch file raíz incluye launch files de subsistemas (navegación, percepción, manipulación). Soporta pasar argumentos al launch file incluido.',
    context: 'IncludeLaunchDescription(PythonLaunchDescriptionSource(...))',
  },
  {
    name: 'OpaqueFunction',
    category: 'Core',
    definition:
      'Acción de launch que ejecuta una función Python arbitraria durante la expansión del launch file. Permite lógica de Python compleja para generar dinámicamente acciones de launch. Recibe el contexto y puede retornar nuevas acciones basadas en condiciones evaluadas en runtime.',
    context: 'OpaqueFunction(function=mi_funcion_launch)',
  },
  {
    name: 'FindPackageShare',
    category: 'Core',
    definition:
      'Sustitución de launch que resuelve la ruta al directorio `share/` del paquete instalado. Fundamental para referenciar archivos de configuración, URDFs y mapas que residen en el paquete. Equivalent a `get_package_share_directory()` en Python.',
    context: 'FindPackageShare("mi_pkg")',
  },
  {
    name: 'LaunchDescription',
    category: 'Core',
    definition:
      'Objeto que agrupa y devuelve todas las acciones que componen un launch file. La función `generate_launch_description()` debe retornar una instancia de LaunchDescription. El launch system expande y ejecuta sus acciones en el orden correcto.',
    context: 'return LaunchDescription([node1, node2, arg1])',
  },
  {
    name: 'GroupAction',
    category: 'Core',
    definition:
      'Acción de launch que agrupa otras acciones bajo un namespace o condición común. Permite aplicar un namespace (`PushRosNamespace`) o condición (`IfCondition`) a un bloque de nodos sin repetir la configuración en cada uno.',
    context: 'GroupAction([PushRosNamespace("robot_1"), node_a, node_b])',
  },

  // --- LIFECYCLE / ESTADOS ---
  {
    name: 'Lifecycle State Machine',
    category: 'Core',
    definition:
      'Máquina de estados finita que gestiona el ciclo de vida de un LifecycleNode. Define los estados (Unconfigured, Inactive, Active, Finalized, ErrorProcessing) y las transiciones válidas entre ellos. Garantiza que los recursos se inicializan y liberan en el orden correcto.',
  },
  {
    name: 'Unconfigured State',
    category: 'Core',
    definition:
      'Estado inicial de un LifecycleNode al crearse. El nodo existe pero no ha asignado recursos ni configurado comunicaciones. Solo acepta la transición `configure()`. Es el estado al que se retorna después de `cleanup()`.',
  },
  {
    name: 'Inactive State',
    category: 'Core',
    definition:
      'Estado donde el LifecycleNode ha ejecutado `on_configure()` y tiene sus recursos preparados (publishers, subscribers creados) pero no está procesando datos. Permite inspección y reconfiguración sin carga computacional. Acepta `activate()` o `cleanup()`.',
  },
  {
    name: 'Active State',
    category: 'Core',
    definition:
      'Estado operacional del LifecycleNode donde procesa datos activamente: los callbacks están habilitados, los publishers emiten y los timers corren. Es el estado de "trabajo normal". Acepta `deactivate()` o `shutdown()`.',
  },
  {
    name: 'Finalized State',
    category: 'Core',
    definition:
      'Estado terminal del LifecycleNode después de ejecutar `shutdown()`. El nodo ha liberado todos sus recursos y no puede realizar más transiciones. Equivale a la destrucción controlada del nodo. Normalmente seguido de la eliminación del objeto.',
  },
  {
    name: 'CallbackReturn',
    category: 'Core',
    definition:
      'Tipo de retorno de los métodos de transición del LifecycleNode (`on_configure`, `on_activate`, etc.). Los valores posibles son `SUCCESS` (transición exitosa), `FAILURE` (fallo controlado, el nodo va a ErrorProcessing) y `ERROR` (error grave).',
    context: 'return CallbackReturn::SUCCESS;',
  },
  {
    name: 'Lifecycle Manager',
    category: 'Core',
    definition:
      'Nodo de Nav2 (`nav2_lifecycle_manager`) que orquesta el ciclo de vida de todos los nodos del stack de navegación en el orden correcto. Envía transiciones en secuencia y monitoriza el estado. Maneja errores y reintentos automáticamente.',
    context: 'lifecycle_manager.yaml en Nav2',
  },

  // --- HARDWARE Y DRIVERS ---
  {
    name: 'Microcontroller',
    category: 'Core',
    definition:
      'Ordenador de bajo costo en un único chip (MCU) optimizado para control de hardware en tiempo real: GPIO, PWM, ADC, UART, SPI, I2C. En robótica ROS 2, se usan con micro-ROS para integrarlos directamente en el grafo de nodos. Ejemplos: ESP32, STM32, Teensy, Arduino Mega.',
  },
  {
    name: 'micro-ROS',
    category: 'Core',
    definition:
      'Versión de ROS 2 diseñada para microcontroladores con recursos limitados (RAM < 100KB). Implementa un subconjunto de la API de rclcpp sobre el middleware XRCE-DDS (mucho más ligero que DDS). El microcontrolador ejecuta el cliente micro-ROS y se comunica con el agente micro-ROS Agent en el PC.',
    context: 'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888',
  },
  {
    name: 'micro-ROS Agent',
    category: 'Core',
    definition:
      'Proceso que actúa como intermediario entre el cliente micro-ROS (en el MCU) y el grafo DDS de ROS 2 (en el PC). Traduce el protocolo XRCE-DDS del microcontrolador al DDS completo de la red ROS 2. Soporta transporte serial (UART), UDP y TCP.',
    context: 'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0',
  },
  {
    name: 'XRCE-DDS',
    category: 'Core',
    definition:
      'eXtremely Resource Constrained Environments DDS. Protocolo de comunicación ultra-ligero del OMG para dispositivos embebidos con < 1MB de RAM. Es el middleware que usa micro-ROS. Usa un modelo cliente-agente: el cliente ligero en el MCU se comunica con el agente completo en el PC.',
  },
  {
    name: 'Agent',
    category: 'Core',
    definition:
      'Proceso intermediario (micro-ROS Agent) que actúa como puerta de enlace entre nodos micro-ROS en microcontroladores y la red DDS completa de ROS 2. Gestiona la creación de entidades DDS en nombre del cliente XRCE y reenvía mensajes en ambas direcciones.',
  },
  {
    name: 'Driver',
    category: 'Core',
    definition:
      'Nodo ROS 2 que abstrae el hardware: habla con el dispositivo físico (sensor, actuador) usando su protocolo nativo y publica/suscribe mensajes ROS estándar. Separa el "lenguaje del hardware" del "lenguaje de ROS". Permite cambiar el hardware sin modificar el resto del sistema.',
    context: 'lidar_driver publica sensor_msgs/LaserScan en /scan',
  },
  {
    name: 'LIDAR',
    category: 'Core',
    definition:
      'Light Detection and Ranging. Sensor que mide distancias emitiendo pulsos láser y midiendo el tiempo de retorno. En robótica: LIDAR 2D (barrido plano, como Hokuyo/RPLIDAR, para navegación) y LIDAR 3D (nube de puntos, como Velodyne/Ouster, para mapeo y detección). Publica `sensor_msgs/LaserScan` o `sensor_msgs/PointCloud2`.',
    context: '/scan topic -> sensor_msgs/LaserScan',
  },
  {
    name: 'IMU',
    category: 'Core',
    definition:
      'Inertial Measurement Unit. Sensor que combina un acelerómetro (mide aceleración lineal en 3 ejes) y un giroscopio (mide velocidad angular en 3 ejes). Opcionalmente incluye magnetómetro. En ROS 2 publica `sensor_msgs/Imu`. Fundamental para estimación de orientación y fusión sensorial.',
    context: '/imu/data -> sensor_msgs/Imu',
  },
  {
    name: 'Encoder',
    category: 'Core',
    definition:
      'Sensor que mide la posición o velocidad de rotación de un eje (rueda, articulación). Tipos: cuadratura (relativo, alta resolución), absoluto (posición absoluta sin referencia), óptico y magnético. Fuente primaria de odometría en robots móviles. Publica posición en `sensor_msgs/JointState`.',
  },
  {
    name: 'Actuator',
    category: 'Core',
    definition:
      'Dispositivo que convierte energía eléctrica en movimiento o fuerza física: motores DC, servos, motores paso a paso, motores brushless, actuadores lineales, etc. Recibe comandos de posición, velocidad o par (torque) del controlador ROS 2 vía `ros2_control`.',
  },
  {
    name: 'ros2_control',
    category: 'Core',
    definition:
      'Framework oficial de ROS 2 para control de hardware en tiempo real. Separa los algoritmos de control (controladores) del acceso al hardware (hardware interfaces) mediante una interfaz abstracta estandarizada. Permite intercambiar controladores en caliente y garantiza separación entre lógica de control e implementación de hardware.',
    context: 'ros2 control list_controllers',
  },
  {
    name: 'Hardware Interface',
    category: 'Core',
    definition:
      'Plugin C++ en ros2_control que abstrae el acceso al hardware real. Implementa la lectura de estados (posición, velocidad, par medido) y la escritura de comandos (posición, velocidad, par comandado). Debe ser lo más ligero posible ya que se ejecuta en el bucle de control de tiempo real.',
    context: 'class MiRobot : public hardware_interface::SystemInterface',
  },
  {
    name: 'Resource Manager',
    category: 'Core',
    definition:
      'Componente del Controller Manager en ros2_control que gestiona el ciclo de vida de los Hardware Interfaces: los carga como plugins, los inicializa, y administra el acceso exclusivo a las interfaces de command y state para evitar conflictos entre controladores.',
  },
  {
    name: 'Transmission',
    category: 'Core',
    definition:
      'Elemento en URDF/ros2_control que describe la relación mecánica entre el eje del motor y la articulación del robot (relación de transmisión, ej: reductora 1:100). Permite que ros2_control convierta comandos articulares a comandos de actuador y viceversa correctamente.',
  },
  {
    name: 'GPIO',
    category: 'Core',
    definition:
      'General Purpose Input/Output. Pines digitales de propósito general en microcontroladores y SBCs (Raspberry Pi). Pueden configurarse como entradas (leer sensores digitales, botones, finales de carrera) o salidas (controlar LEDs, relés). En ROS 2, se controlan desde drivers o micro-ROS.',
    context: 'Raspberry Pi: gpiod, RPi.GPIO libraries',
  },
  {
    name: 'UART',
    category: 'Core',
    definition:
      'Universal Asynchronous Receiver Transmitter. Protocolo de comunicación serial punto a punto. Usado para conectar microcontroladores al PC, comunicar sensores (GPS, algunos LIDARs) y programar MCUs. Velocidades típicas: 9600 a 115200 baudios. En Linux: `/dev/ttyUSB0`, `/dev/ttyACM0`.',
    context: 'serial.Serial("/dev/ttyUSB0", 115200)',
  },
  {
    name: 'CAN Bus',
    category: 'Core',
    definition:
      'Controller Area Network. Bus de comunicación industrial robusto para interconectar múltiples dispositivos (actuadores, sensores) en un robot. Permite hasta 1 Mbps, detección de errores y prioridad de mensajes. Muy usado en robótica industrial. En Linux: Socket CAN (`candump`, `cansend`).',
    context: 'ip link set can0 up type can bitrate 1000000',
  },
  {
    name: 'EtherCAT',
    category: 'Core',
    definition:
      'Ethernet for Control Automation Technology. Protocolo industrial de tiempo real basado en Ethernet estándar. Permite comunicación determinista con ciclos de 1ms o menos. Usado en robots industriales de alta gama (KUKA, Universal Robots). Soportado en ROS 2 vía `ethercat_ros2` y `SOEM`.',
  },
  {
    name: 'Raspberry Pi',
    category: 'Core',
    definition:
      'Single Board Computer (SBC) de bajo costo muy popular en proyectos robóticos educativos y de investigación. Ejecuta Linux completo y ROS 2. La versión 4 (4GB+ RAM) es suficiente para navegación básica con ROS 2. La versión 5 mejora significativamente el rendimiento. Disponible en ubuntu-server o Raspberry Pi OS.',
    context: 'ROS 2 Jazzy en RPi 5: ~150ms boot + ROS',
  },
  {
    name: 'NVIDIA Jetson',
    category: 'Core',
    definition:
      'Familia de módulos de cómputo embebido de NVIDIA con GPU integrada para inferencia de IA en el borde (edge). Jetson Nano (512 CUDA cores), Jetson Xavier NX, Jetson Orin NX (1024 CUDA cores, DLSS). Ejecutan Ubuntu + ROS 2 y son la plataforma preferida para robots con visión por computadora.',
    context: 'Orin NX: 100 TOPS AI performance',
  },
  {
    name: 'ESP32',
    category: 'Core',
    definition:
      'Microcontrolador Wi-Fi/Bluetooth de Espressif de bajo costo (< $5). Dual-core 240MHz, 520KB RAM, WiFi integrado. Muy popular para aplicaciones micro-ROS por su conectividad WiFi (permite transporte UDP sin cable). Programable con Arduino, ESP-IDF o FreeRTOS.',
    context: 'micro-ROS on ESP32 via WiFi UDP',
  },
  {
    name: 'STM32',
    category: 'Core',
    definition:
      'Familia de microcontroladores ARM Cortex-M de STMicroelectronics. Muy usados en robótica profesional por su rendimiento, periféricos (múltiples CAN, SPI, I2C, timers) y herramientas de desarrollo maduras. Compatible con micro-ROS via UART, USB o Ethernet.',
  },
  {
    name: 'PWM',
    category: 'Core',
    definition:
      'Pulse Width Modulation. Técnica para controlar potencia analógica con señales digitales variando el ancho del pulso (duty cycle). Usado para controlar velocidad de motores DC (0-100% duty = 0-máx velocidad), posición de servos (1-2ms de pulso = 0-180°) y brillo de LEDs.',
    context: 'Servo: 1.5ms pulso = 90° (posición neutra)',
  },
  {
    name: 'Depth Camera',
    category: 'Core',
    definition:
      'Cámara que capta imagen RGB y mapa de profundidad simultáneamente. Tecnologías: Time-of-Flight (ToF), luz estructurada (Intel RealSense D415/D435) o visión estéreo activa. Publica en ROS 2: `sensor_msgs/Image` (RGB), `sensor_msgs/Image` (profundidad 16UC1) y `sensor_msgs/PointCloud2`.',
    context: '/camera/depth/image_raw + /camera/color/image_raw',
  },
  {
    name: 'Intel RealSense',
    category: 'Core',
    definition:
      'Familia de cámaras de profundidad de Intel. Modelos populares en robótica: D435i (RGB-D + IMU integrado, ideal para VIO), D415 (mayor precisión a larga distancia), L515 (LiDAR ToF, exterior). Driver oficial: `realsense2_camera` para ROS 2.',
    context: 'ros2 launch realsense2_camera rs_launch.py',
  },
  {
    name: 'DYNAMIXEL',
    category: 'Core',
    definition:
      'Familia de servomotores inteligentes de ROBOTIS con bus serial (TTL/RS-485) y comunicación por protocolo propio (Dynamixel Protocol 1.0/2.0). Tienen control de posición, velocidad y par con retroalimentación. Muy usados en brazos robóticos y humanoides. Driver ROS 2: `dynamixel_sdk`, `dynamixel_hardware`.',
    context: 'dynamixel_hardware + ros2_control',
  },
  {
    name: 'FreeRTOS',
    category: 'Core',
    definition:
      'Sistema operativo de tiempo real open-source para microcontroladores. Provee scheduling con prioridades, colas de mensajes, semáforos y timers de software. Es el RTOS más usado en el ecosistema micro-ROS. Disponible en ESP32 (ESP-IDF) y STM32 (CubeMX).',
    context: 'micro-ROS funciona sobre FreeRTOS tasks',
  },

  // --- DISTRIBUCIONES ROS 2 ---
  {
    name: 'ROS 2 Jazzy Jalisco',
    category: 'Core',
    definition:
      'Distribución LTS (Long Term Support) de ROS 2 lanzada en mayo 2024. Soportada en Ubuntu 24.04 Noble Numbat hasta 2029. Trae mejoras en zenoh transport, type description service, nueva API de parámetros y mejor soporte para Windows. Distribución recomendada para nuevos proyectos.',
    context: 'source /opt/ros/jazzy/setup.bash',
  },
  {
    name: 'ROS 2 Humble Hawksbill',
    category: 'Core',
    definition:
      'Distribución LTS de ROS 2 lanzada en mayo 2022. Soportada en Ubuntu 22.04 Jammy hasta mayo 2027. Muy estable y ampliamente adoptada en producción. Compatible con Nav2, MoveIt 2, ros2_control. Distribución más popular actualmente en sistemas en producción.',
    context: 'source /opt/ros/humble/setup.bash',
  },
  {
    name: 'ROS 2 Iron Irwini',
    category: 'Core',
    definition:
      'Distribución de ROS 2 con soporte estándar (no LTS), lanzada en mayo 2023. Soportada en Ubuntu 22.04 hasta noviembre 2024. Introdujo el EventsExecutor, mejoras en component containers y la API type_description. Puente entre Humble (LTS) y Jazzy (LTS).',
  },
  {
    name: 'ROS 2 Rolling Ridley',
    category: 'Core',
    definition:
      'Distribución rolling-release de ROS 2: siempre contiene el código más reciente en desarrollo. No tiene fecha de fin de soporte fija. Usada por los desarrolladores del ecosistema para probar nuevas características antes de que lleguen a distribuciones estables. No recomendada para producción.',
    context: 'source /opt/ros/rolling/setup.bash',
  },
  {
    name: 'LTS (Long Term Support)',
    category: 'Core',
    definition:
      'Designación de una distribución ROS 2 con 5 años de soporte oficial (patches de seguridad y correcciones de bugs). Las distribuciones LTS coinciden con las versiones LTS de Ubuntu. Actuales: Humble (2022-2027) y Jazzy (2024-2029). Recomendadas para proyectos de producción.',
  },
  {
    name: 'EOL (End of Life)',
    category: 'Core',
    definition:
      'Estado de una distribución ROS que ya no recibe soporte oficial, actualizaciones de seguridad ni nuevos paquetes. Distribuciones EOL: Foxy (2023), Galactic (2022), Eloquent (2020), Dashing (2021). Migrar a una distribución activa es crítico para la seguridad del sistema.',
  },

  // --- CONFIGURACIÓN DEL SISTEMA ---
  {
    name: 'ROS_LOCALHOST_ONLY',
    category: 'Core',
    definition:
      'Variable de entorno que, cuando se pone a `1`, restringe la comunicación DDS solo a la interfaz loopback (127.0.0.1). Aísla completamente el robot de la red, evitando que otros máquinas descubran los nodos. Útil para desarrollo en máquina local sin interferir con otros robots.',
    context: 'export ROS_LOCALHOST_ONLY=1',
  },
  {
    name: 'Discovery Server',
    category: 'Core',
    definition:
      'Modo alternativo de descubrimiento DDS donde un servidor centralizado (en lugar de multicast) coordina la información de participantes. Ideal para redes complejas (WiFi empresarial, VPN) donde el multicast no funciona. Se configura con `fastdds discovery` y `ROS_DISCOVERY_SERVER`.',
    context: 'fastdds discovery -s 0 -i 0  # Inicia el servidor',
  },
  {
    name: 'PREEMPT_RT Patch',
    category: 'Core',
    definition:
      'Parche del kernel de Linux que convierte el scheduler estándar en un scheduler de tiempo real con prioridades deterministas. Reduce la latencia máxima de interrupciones de millisegundos a microsegundos. Esencial para aplicaciones Hard Real-Time con ROS 2 (control de robots industriales, cirugía robótica).',
    context: 'apt install linux-image-lowlatency-hwe-22.04',
  },
  {
    name: 'Udev Rules',
    category: 'Core',
    definition:
      'Reglas del sistema Linux (en `/etc/udev/rules.d/`) que asignan nombres fijos y permisos a dispositivos USB cuando se conectan. Permiten referenciar sensores por nombre estable (ej: `/dev/lidar` en lugar del volátil `/dev/ttyUSB0`). Imprescindibles en robots con múltiples dispositivos USB.',
    context: 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", SYMLINK+="lidar"',
  },
  {
    name: 'CPU Affinity',
    category: 'Core',
    definition:
      'Mecanismo del SO para asignar un proceso o hilo a un núcleo CPU específico. En sistemas de tiempo real con ROS 2, se asignan los hilos de control a cores dedicados (aislados del OS con `isolcpus`), minimizando la interferencia del sistema operativo en los tiempos de ejecución.',
    context: 'taskset -c 2,3 ros2 run mi_pkg mi_ctrl_nodo',
  },
  {
    name: 'NTP (Network Time Protocol)',
    category: 'Core',
    definition:
      'Protocolo para sincronizar los relojes de múltiples máquinas en red con precisión de ~1ms. Crítico en sistemas multi-robot o con sensores en PCs separados: los timestamps deben ser comparables entre máquinas. En ROS 2, el tiempo de los mensajes depende del reloj del sistema.',
    context: 'chronyc tracking  # Verificar sincronización',
  },
  {
    name: 'PTP (Precision Time Protocol)',
    category: 'Core',
    definition:
      'IEEE 1588. Protocolo de sincronización de tiempo de alta precisión (< 1 microsegundo) usando hardware especializado en tarjetas de red. Necesario para sistemas con múltiples sensores (fusión LiDAR+cámara) donde la sincronización sub-milisegundo es crítica.',
    context: 'ptp4l -i eth0 -m',
  },
  {
    name: 'Pluginlib',
    category: 'Core',
    definition:
      'Librería de ROS 2 para crear y cargar plugins C++ dinámicamente en tiempo de ejecución. Permite extender aplicaciones (Nav2, ros2_control, MoveIt) sin recompilar. Los plugins se registran via `PLUGINLIB_EXPORT_CLASS` y se describen en un XML de descripción.',
    context: 'pluginlib::ClassLoader<BaseClass> loader("pkg", "BaseClass");',
  },
  {
    name: 'SCHED_FIFO',
    category: 'Core',
    definition:
      'Política de scheduling del kernel Linux de tiempo real (First In, First Out). Los hilos con esta política y alta prioridad (1-99) no son desalojados por el scheduler del SO, solo por hilos de mayor prioridad o por llamadas de sistema bloqueantes. Usado en hilos de control de tiempo real en ROS 2.',
    context: 'pthread_setschedparam(thread, SCHED_FIFO, &param)',
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
