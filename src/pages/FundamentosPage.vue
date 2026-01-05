<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: EL GRAN CONCEPTO -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Enciclopedia Técnica</div>
        <h1 class="hero-title">Fundamentos del <span class="text-primary">Middleware</span></h1>

        <TextBlock>
          Dominar ROS 2 no es solo saber programar en Python; es entender cómo orquestar sistemas
          complejos. Aquí desglosamos la jerga técnica en conceptos digeribles, desde la capa de red
          (DDS) hasta la geometría del robot (TF2).
        </TextBlock>
      </div>
    </section>

    <!-- 2. NIVEL 1: ARQUITECTURA Y DDS (Lo que no se ve) -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Arquitectura Profunda (Middleware)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            A diferencia de ROS 1, ROS 2 se construye sobre estándares industriales. No existe un
            "Master" central; todo es distribuido.
          </TextBlock>
          <div class="glossary-grid-mini">
            <div class="term-card">
              <div class="term-title">DDS</div>
              <div class="term-desc">
                Data Distribution Service. El protocolo de red estándar que usa ROS 2 para mover
                datos. Es como el TCP/IP de los robots.
              </div>
            </div>
            <div class="term-card">
              <div class="term-title">Discovery</div>
              <div class="term-desc">
                El proceso automático donde los nodos se encuentran entre sí en la red sin necesidad
                de un servidor central.
              </div>
            </div>
            <div class="term-card">
              <div class="term-title">Domain ID</div>
              <div class="term-desc">
                Un número (0-100) que aísla redes de robots. Si dos robots tienen diferente ID, no
                se ven entre sí.
              </div>
            </div>
            <div class="term-card">
              <div class="term-title">RMW</div>
              <div class="term-desc">
                ROS Middleware Interface. La capa que permite cambiar de proveedor DDS (Eclipse
                Cyclone, FastDDS) sin cambiar tu código.
              </div>
            </div>
          </div>
        </template>
        <template #right>
          <AlertBlock type="warning" title="⚡ QoS (Quality of Service)">
            En ROS 2, no basta con enviar datos. Debes definir <strong>CÓMO</strong> viajan:
            <ul class="q-pl-md q-mt-sm">
              <li>
                <strong>Reliability:</strong> ¿Garantizo la entrega (TCP) o prefiero velocidad
                (UDP)?
              </li>
              <li><strong>Durability:</strong> ¿Un nodo nuevo recibe mensajes viejos (Latch)?</li>
              <li><strong>History:</strong> ¿Cuántos mensajes guardo en memoria?</li>
            </ul>
          </AlertBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. NIVEL 2: PATRONES DE COMUNICACIÓN (El Núcleo) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Patrones de Comunicación</SectionTitle>
      <TextBlock> Los bloques de construcción básicos para que los nodos interactúen. </TextBlock>

      <div class="definitions-grid">
        <div v-for="item in communicationTerms" :key="item.title" class="def-card">
          <div class="def-header">
            <q-icon :name="item.icon" size="sm" :color="item.color" />
            <h3 class="def-title">{{ item.title }}</h3>
          </div>
          <p class="def-body">{{ item.desc }}</p>
          <div class="def-meta">{{ item.meta }}</div>
        </div>
      </div>
    </div>

    <!-- ==============================================================
         3. NIVEL 3 EXPANDIDO: FÍSICA Y GEOMETRÍA
         ============================================================== -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Física y Geometría (El Espacio)</SectionTitle>

      <!-- 3.1 INTRODUCCIÓN -->
      <TextBlock>
        Un robot no vive en el vacío; ocupa un lugar en el espacio físico. Para que un robot sepa
        dónde está un obstáculo o cómo mover su brazo, necesita un modelo matemático robusto. ROS 2
        resuelve esto con dos pilares: <strong>TF2</strong> (Matemática) y
        <strong>URDF</strong> (Descripción).
      </TextBlock>

      <!-- 3.2 TF2: EL ÁRBOL GENEALÓGICO -->
      <div class="q-mt-lg">
        <h3 class="text-h6 text-secondary q-mb-md">A. El Árbol de Transformadas (TF Tree)</h3>
        <SplitBlock>
          <template #left>
            <TextBlock>
              Imagina que tienes una cámara pegada al techo de un coche.
              <ul>
                <li>Si el coche avanza 10 metros, la cámara también.</li>
                <li>Si el coche gira, la cámara gira.</li>
              </ul>
              En ROS 2, esto es una relación <strong>Padre -> Hijo</strong>. <br /><br />
              <strong>TF2</strong> es la librería que calcula estas matemáticas en segundo plano. Si
              le preguntas:
              <em>"¿Dónde está el obstáculo que ve la cámara respecto a la rueda trasera?"</em>, TF2
              recorre el árbol sumando posiciones y rotaciones para darte la respuesta exacta.
            </TextBlock>
          </template>
          <template #right>
            <CodeBlock
              title="Ejemplo: Jerarquía de un Rover"
              lang="bash"
              :content="tfTreeExample"
              :copyable="false"
            />
            <div class="text-caption text-grey-5 q-mt-xs">
              * `base_link` es el centro del robot. Todo lo demás cuelga de él.
            </div>
          </template>
        </SplitBlock>
      </div>

      <!-- 3.3 URDF: LA ANATOMÍA -->
      <div class="q-mt-xl">
        <h3 class="text-h6 text-secondary q-mb-md">B. Anatomía: Links y Joints (URDF)</h3>
        <SplitBlock>
          <template #left>
            <AlertBlock type="info" title="🦴 Huesos y Músculos">
              El archivo <strong>URDF</strong> (Unified Robot Description Format) es un XML que
              describe el cuerpo del robot: <br /><br />
              • <strong>Links (Eslabones):</strong> Las partes rígidas (Chasis, Rueda, Sensor).
              Tienen masa, inercia y color.<br />
              • <strong>Joints (Articulaciones):</strong> La unión flexible entre dos links. Definen
              el movimiento (fijo, rotatorio, lineal).
            </AlertBlock>
          </template>
          <template #right>
            <CodeBlock
              title="robot_description.urdf (Fragmento)"
              lang="xml"
              :content="urdfExample"
            />
          </template>
        </SplitBlock>
      </div>

      <!-- 3.4 ESTÁNDARES DE NAVEGACIÓN -->
      <div class="q-mt-xl">
        <h3 class="text-h6 text-secondary q-mb-md">C. Navegación: Map vs Odom</h3>
        <TextBlock>
          Esta es la duda #1 de todo principiante. ¿Por qué el robot tiene dos sistemas de
          coordenadas "mundo"? ROS define el estándar <strong>REP-105</strong>:
        </TextBlock>

        <div class="row q-col-gutter-md q-mt-sm">
          <div class="col-12 col-md-6">
            <div class="def-card">
              <div class="def-header">
                <q-icon name="speed" color="orange" size="sm" />
                <div class="def-title">Odom (Odometría)</div>
              </div>
              <div class="def-body">
                <strong>¿Qué es?</strong> Cálculo basado en cuánto giran las ruedas
                (Encoders/IMU).<br />
                <strong>Pros:</strong> Es continuo y suave. Nunca "salta".<br />
                <strong>Contras:</strong> Acumula error (Drift). Si las ruedas patinan, el robot
                cree que se movió pero no lo hizo.<br />
                <strong>Uso:</strong> Control de velocidad inmediato y evitar obstáculos cercanos.
              </div>
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="def-card">
              <div class="def-header">
                <q-icon name="map" color="green" size="sm" />
                <div class="def-title">Map (Mapa)</div>
              </div>
              <div class="def-body">
                <strong>¿Qué es?</strong> Posición absoluta basada en sensores externos (GPS, LIDAR)
                contra un mapa conocido.<br />
                <strong>Pros:</strong> No acumula error a largo plazo.<br />
                <strong>Contras:</strong> Puede "teletransportarse" (saltar) cuando el robot se
                relocaliza.<br />
                <strong>Uso:</strong> Planificación de rutas largas (ir de la cocina al baño).
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. NIVEL 4: EJECUCIÓN Y CÓDIGO -->
    <div class="section-group self-stretch">
      <SectionTitle>4. Ejecución (Runtime)</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-4">
          <div class="tech-card">
            <h4>Lifecycle Node</h4>
            <p>
              Nodos gestionados con estados (Configuring, Active, Inactive). Permiten arranque
              determinista del sistema.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="tech-card">
            <h4>Executor</h4>
            <p>
              El "jefe" que decide cuándo correr los callbacks. Puede ser
              <strong>SingleThreaded</strong> (uno a uno) o
              <strong>MultiThreaded</strong> (paralelo).
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="tech-card">
            <h4>Callback Group</h4>
            <p>
              Reglas que evitan que un callback (ej: procesar imagen) bloquee a otro (ej: frenado de
              emergencia).
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 6. NIVEL 5: SISTEMA DE ARCHIVOS Y BUILD -->
    <div class="section-group self-stretch">
      <SectionTitle>5. El Entorno de Desarrollo</SectionTitle>

      <div class="definitions-grid mini">
        <div class="def-card small" v-for="term in devTerms" :key="term.title">
          <div class="def-title text-primary">{{ term.title }}</div>
          <div class="def-body small">{{ term.desc }}</div>
        </div>
      </div>
    </div>

    <!-- 7. GLOSARIO FINAL (Referencia Rápida) -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>Diccionario Rápido </SectionTitle>
      <div class="text-caption text-grey-5 q-mb-md">
        Términos extra que escucharás constantemente.
      </div>

      <div class="glossary-chips">
        <q-chip
          v-for="term in quickTerms"
          :key="term"
          color="dark"
          text-color="grey-3"
          icon="label"
        >
          {{ term }}
        </q-chip>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// DATOS DE COMUNICACIÓN
const communicationTerms = [
  {
    title: 'Topic (Tópico)',
    desc: 'Canal de datos unidireccional para streaming continuo. N:N (muchos a muchos).',
    meta: 'Uso: Sensores, Estado del robot.',
    icon: 'podcasts',
    color: 'purple-4',
  },
  {
    title: 'Service (Servicio)',
    desc: 'Comunicación síncrona Cliente/Servidor. Pides algo y esperas respuesta.',
    meta: 'Uso: Cálculos rápidos, Cambios de estado.',
    icon: 'handshake',
    color: 'orange-4',
  },
  {
    title: 'Action (Acción)',
    desc: 'Para tareas largas. Permite Feedback (progreso) y Cancelación (abortar).',
    meta: 'Uso: Navegar a un punto, Mover brazo robótico.',
    icon: 'timelapse',
    color: 'cyan-4',
  },
  {
    title: 'Parameter',
    desc: 'Variables de configuración de un nodo que se pueden cambiar en tiempo real.',
    meta: 'Uso: Velocidad máxima, Puerto USB, Calibración.',
    icon: 'tune',
    color: 'green-4',
  },
];

// DATOS DE DESARROLLO
const devTerms = [
  {
    title: 'Colcon',
    desc: 'La herramienta de compilación (Build Tool). Itera sobre todos los paquetes y los compila en orden.',
  },
  {
    title: 'Overlay',
    desc: 'Tu espacio de trabajo actual (ros2_ws) que se monta "encima" de la instalación base de ROS 2.',
  },
  {
    title: 'Underlay',
    desc: 'La instalación base de ROS 2 (/opt/ros/jazzy). El cimiento del sistema.',
  },
  {
    title: 'package.xml',
    desc: 'El DNI del paquete. Define nombre, versión, autor y dependencias.',
  },
  {
    title: 'CMakeLists.txt',
    desc: 'Receta de cocina para compilar paquetes C++. Dice qué librerías linkear.',
  },
  { title: 'setup.py', desc: 'Equivalente al CMakeLists pero para paquetes Python.' },
];
const tfTreeExample = `
map (Mundo Global)
└── odom (Inicio del recorrido)
    └── base_link (Chasis del Robot)
        ├── left_wheel (Rueda Izq)
        ├── right_wheel (Rueda Der)
        └── camera_link (Sensor)
            └── camera_optical_frame
`.trim();

const urdfExample = `
<!-- Uniendo la cámara al chasis -->
<joint name="camera_joint" type="fixed">
    <!-- ¿Quién es el padre? -->
    <parent link="base_link"/>
    <!-- ¿Quién es el hijo? -->
    <child link="camera_link"/>
    <!-- La cámara está 20cm adelante y 10cm arriba -->
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
</joint>
`.trim();

// TÉRMINOS RÁPIDOS (50 TÉRMINOS ESENCIALES)
const quickTerms = [
  // --- HERRAMIENTAS CORE ---
  'Colcon (Build Tool)',
  'Rosdep (Gestor de dependencias)',
  'Ros2cli (Comandos de terminal)',
  'RQt (GUI de plugins)',
  'RViz2 (Visualizador 3D)',
  'Rosbags (Grabación de datos)',
  'Doctor (ros2 doctor check)',

  // --- ARCHIVOS Y FORMATOS ---
  'package.xml (Manifiesto)',
  'CMakeLists.txt (Build C++)',
  'setup.py (Build Python)',
  'Launch File (Python/XML/YAML)',
  'URDF (Modelo del Robot)',
  'Xacro (Macros para URDF)',
  'SDF (Simulation Description Format)',
  'SRDF (Semántica del Robot)',
  'YAML (Configuración)',

  // --- COMUNICACIÓN Y MIDDLEWARE ---
  'DDS (Data Distribution Service)',
  'IDL (Interface Definition Language)',
  'MSG (Definición de mensaje)',
  'SRV (Definición de servicio)',
  'Action (Definición de acción)',
  'QoS (Calidad de Servicio)',
  'Discovery (Hallazgo automático)',
  'RMW (Implementación Middleware)',
  'Serialized Message (Datos crudos)',

  // --- CONCEPTOS DE EJECUCIÓN ---
  'Node (Unidad de cómputo)',
  'Lifecycle Node (Máquina de estados)',
  'Component (Nodo componible)',
  'Executor (Gestor de hilos)',
  'Callback Group (Reglas de ejecución)',
  'Reentrant (Callback paralelo)',
  'Spinning (Bucle de eventos)',

  // --- GRAFO Y NOMBRES ---
  'Namespace (Agrupación de nodos)',
  'Remapping (Renombrado dinámico)',
  'Parameter Server (Configuración global)',
  'Topic (Canal de datos)',
  'Service (Cliente/Servidor)',

  // --- FÍSICA Y NAVEGACIÓN (NAV2) ---
  'TF2 (Árbol de transformadas)',
  'Frame (Sistema de coordenadas)',
  'Quaternion (Rotación matemática)',
  'Odometry (Estimación de posición)',
  'SLAM (Mapeo y localización)',
  'Costmap (Mapa de obstáculos)',
  'Behavior Tree (Árbol de comportamiento)',
  'Planner (Calculador de ruta)',
  'Controller (Seguidor de ruta)',
  'Recovery (Recuperación de fallos)',

  // --- CONTROL Y HARDWARE ---
  'ros2_control (Framework de control)',
  'Hardware Interface (Driver físico)',
  'Joint State (Estado de articulación)',
  'URscript (Controladores Universal Robots)',
];
</script>

<style scoped>
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(99, 102, 241, 0.15), transparent 60%),
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

/* GRIDS PERSONALIZADOS */

/* Grid Mini (Para DDS, Discovery) */
.glossary-grid-mini {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 16px;
  margin-top: 24px;
}

.term-card {
  background: rgba(30, 41, 59, 0.5);
  padding: 12px;
  border-radius: 12px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.term-title {
  color: #38bdf8;
  font-weight: 700;
  font-size: 0.9rem;
  margin-bottom: 4px;
}

.term-desc {
  font-size: 0.8rem;
  color: #94a3b8;
  line-height: 1.3;
}

/* Grid de Definiciones Grandes (Comm Patterns) */
.definitions-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
  gap: 20px;
}

.def-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.15);
  border-radius: 16px;
  padding: 24px;
  transition: all 0.3s ease;
  display: flex;
  flex-direction: column;
}

.def-card:hover {
  transform: translateY(-4px);
  background: rgba(30, 41, 59, 0.8);
  border-color: rgba(56, 189, 248, 0.3);
}

.def-header {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 12px;
}

.def-title {
  margin: 0;
  font-size: 1.1rem;
  color: #f1f5f9;
  font-weight: 600;
}

.def-body {
  color: #cbd5e1;
  font-size: 0.95rem;
  line-height: 1.5;
  flex: 1; /* Empuja el meta hacia abajo */
  margin-bottom: 16px;
}

.def-meta {
  font-size: 0.8rem;
  color: #64748b;
  font-family: 'Fira Code', monospace;
  background: rgba(0, 0, 0, 0.2);
  padding: 6px;
  border-radius: 6px;
}

/* Lista de Glosario Vertical */
.glossary-list {
  display: flex;
  flex-direction: column;
  gap: 12px;
  margin-top: 16px;
}

.glossary-item {
  padding: 12px;
  background: rgba(255, 255, 255, 0.03);
  border-left: 3px solid #6366f1; /* Indigo */
  border-radius: 0 8px 8px 0;
  color: #e2e8f0;
  font-size: 0.95rem;
}

/* Tech Cards (Runtime) */
.tech-card {
  height: 100%;
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.7), rgba(15, 23, 42, 0.7));
  padding: 20px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.05);
}

.tech-card h4 {
  margin: 0 0 10px 0;
  color: #a5b4fc;
  font-size: 1rem;
  font-weight: 700;
}

.tech-card p {
  margin: 0;
  font-size: 0.9rem;
  color: #94a3b8;
}

/* Grid Mini (Dev Terms) */
.definitions-grid.mini {
  grid-template-columns: repeat(auto-fill, minmax(180px, 1fr));
}

.def-card.small {
  padding: 16px;
}

.def-body.small {
  font-size: 0.85rem;
  margin-bottom: 0;
}

/* Chips Finales */
.glossary-chips {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .glossary-grid-mini {
    grid-template-columns: 1fr;
  }
}
</style>
