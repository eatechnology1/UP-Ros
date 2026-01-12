<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      ROS 2 implementa una <strong>arquitectura de grafo computacional distribuido</strong>, donde
      cada nodo representa un proceso independiente que se comunica mediante el middleware DDS (Data
      Distribution Service). Esta arquitectura desacoplada permite aislamiento de fallos,
      escalabilidad horizontal y heterogeneidad de lenguajes de programación.
    </TextBlock>

    <AlertBlock type="info" title="Paradigma Arquitectónico">
      <strong>Publish-Subscribe:</strong> Comunicación asíncrona desacoplada (Topics)
      <br />
      <strong>Request-Reply:</strong> Comunicación síncrona bloqueante (Services)
      <br />
      <strong>Goal-Oriented:</strong> Tareas de larga duración con feedback (Actions)
      <br />
      <strong>Middleware:</strong> DDS como capa de abstracción de red
    </AlertBlock>

    <!-- MONOLITO VS DISTRIBUIDO -->
    <div class="section-group">
      <SectionTitle>1. Arquitectura Monolítica vs. Distribuida</SectionTitle>

      <AlertBlock type="warning" title="Doctoral Insight: Node != Process" class="q-mb-md">
        Un error común es pensar que <strong>1 Nodo = 1 Proceso de SO</strong>. En sistemas
        avanzados, usamos <strong>Component Containers</strong> para ejecutar múltiples nodos en un
        solo proceso (Composition).
        <br />
        Esto combina el <em>desacoplamiento lógico</em> de la arquitectura distribuida con la
        <em>eficiencia de memoria</em> del monolito.
      </AlertBlock>

      <div class="architecture-comparison q-mt-md">
        <div class="arch-card monolithic">
          <div class="arch-header">
            <q-icon name="warning" size="2rem" />
            <span>Arquitectura Monolítica</span>
          </div>
          <div class="arch-visual">
            <div class="monolith-block">
              <div class="module">Percepción</div>
              <div class="separator">+</div>
              <div class="module">Control</div>
              <div class="separator">+</div>
              <div class="module">Navegación</div>
              <div class="separator">+</div>
              <div class="module">Planificación</div>
            </div>
          </div>
          <div class="arch-problems">
            <div class="problem-item">
              <q-icon name="close" color="red-4" />
              <span>Fallo en un módulo detiene todo el sistema</span>
            </div>
            <div class="problem-item">
              <q-icon name="close" color="red-4" />
              <span>Acoplamiento fuerte entre componentes</span>
            </div>
            <div class="problem-item">
              <q-icon name="close" color="red-4" />
              <span>Escalabilidad limitada (single process)</span>
            </div>
            <div class="problem-item">
              <q-icon name="close" color="red-4" />
              <span>Debugging complejo (stack traces entrelazados)</span>
            </div>
          </div>
        </div>

        <div class="arch-card distributed">
          <div class="arch-header">
            <q-icon name="check_circle" size="2rem" />
            <span>Arquitectura Distribuida (ROS 2)</span>
          </div>
          <div class="arch-visual">
            <div class="graph-nodes">
              <div class="node-item perception">
                <q-icon name="videocam" />
                <span>Percepción</span>
              </div>
              <div class="node-item control">
                <q-icon name="settings" />
                <span>Control</span>
              </div>
              <div class="node-item navigation">
                <q-icon name="explore" />
                <span>Navegación</span>
              </div>
              <div class="node-item planning">
                <q-icon name="psychology" />
                <span>Planificación</span>
              </div>
            </div>
            <div class="connections-overlay">
              <svg viewBox="0 0 400 200" class="connection-svg">
                <line
                  x1="100"
                  y1="50"
                  x2="300"
                  y2="50"
                  stroke="#00D9FF"
                  stroke-width="2"
                  stroke-dasharray="5,5"
                  class="data-flow"
                />
                <line
                  x1="100"
                  y1="150"
                  x2="300"
                  y2="150"
                  stroke="#00D9FF"
                  stroke-width="2"
                  stroke-dasharray="5,5"
                  class="data-flow"
                  style="animation-delay: 0.5s"
                />
                <line
                  x1="200"
                  y1="50"
                  x2="200"
                  y2="150"
                  stroke="#00FF88"
                  stroke-width="2"
                  stroke-dasharray="5,5"
                  class="data-flow"
                  style="animation-delay: 1s"
                />
              </svg>
            </div>
          </div>
          <div class="arch-benefits">
            <div class="benefit-item">
              <q-icon name="check" color="green-4" />
              <span>Aislamiento de fallos (fault isolation)</span>
            </div>
            <div class="benefit-item">
              <q-icon name="check" color="green-4" />
              <span>Escalabilidad horizontal (multi-core, multi-machine)</span>
            </div>
            <div class="benefit-item">
              <q-icon name="check" color="green-4" />
              <span>Heterogeneidad de lenguajes (C++/Python/Rust)</span>
            </div>
            <div class="benefit-item">
              <q-icon name="check" color="green-4" />
              <span>Desarrollo modular y testing independiente</span>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- DDS MIDDLEWARE -->
    <div class="section-group">
      <SectionTitle>2. DDS: El Middleware de Comunicación</SectionTitle>
      <TextBlock>
        ROS 2 utiliza <strong>DDS (Data Distribution Service)</strong> como middleware de
        comunicación. DDS es un estándar OMG (Object Management Group) que implementa el patrón
        publish-subscribe sobre UDP/TCP con QoS (Quality of Service) configurable.
      </TextBlock>

      <div class="dds-architecture q-mt-md">
        <div class="dds-layer">
          <div class="layer-title">Capa de Aplicación (ROS 2 API)</div>
          <div class="layer-content">
            <code>rclcpp::Node, rclpy.Node</code>
          </div>
        </div>
        <div class="dds-arrow">↓</div>
        <div class="dds-layer">
          <div class="layer-title">ROS 2 Middleware Interface (RMW)</div>
          <div class="layer-content">
            <code>rmw_fastrtps</code> (Default) vs <code>rmw_cyclonedds</code>
            <div class="text-caption q-mt-xs text-grey-4">
              Abstrae la implementación del vendor. Permite cambiar de backend en runtime (e.g.,
              <code>RMW_IMPLEMENTATION=rmw_cyclonedds_cpp</code>).
            </div>
          </div>
        </div>
        <div class="dds-arrow">↓</div>
        <div class="dds-layer">
          <div class="layer-title">DDS Implementation</div>
          <div class="layer-content">
            <code>FastDDS, CycloneDDS, RTI Connext</code>
          </div>
        </div>
        <div class="dds-arrow">↓</div>
        <div class="dds-layer">
          <div class="layer-title">Transporte (UDP/TCP/Shared Memory)</div>
          <div class="layer-content">
            <code>Multicast, Unicast, Intra-process</code>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Discovery Protocol</SectionTitle>
        <TextBlock>
          DDS utiliza un protocolo de descubrimiento automático que permite a los nodos encontrarse
          sin configuración centralizada. El proceso consta de dos fases:
        </TextBlock>

        <div class="discovery-phases q-mt-md">
          <div class="phase-card">
            <div class="phase-number">1</div>
            <div class="phase-content">
              <div class="phase-title">SPDP (Simple Participant Discovery Protocol)</div>
              <div class="phase-desc">
                Los participantes envían mensajes <strong>Multicast UDP</strong> periódicos.
                <br />
                <span class="text-caption text-orange-3">
                  ⚠️ Precaución en WiFi: El tráfico de descubrimiento escala con N², pudiendo
                  saturar redes inalámbricas ("Discovery Storm").
                </span>
              </div>
              <CodeBlock
                lang="cpp"
                content="// Configuración de discovery
auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
qos.reliable();
qos.transient_local();
qos.deadline(std::chrono::milliseconds(100));"
                :copyable="true"
              />
            </div>
          </div>

          <div class="phase-card">
            <div class="phase-number">2</div>
            <div class="phase-content">
              <div class="phase-title">SEDP (Simple Endpoint Discovery Protocol)</div>
              <div class="phase-desc">
                Una vez descubiertos los participantes, intercambian información sobre sus endpoints
                (publishers/subscribers). Se negocian QoS policies para establecer compatibilidad.
              </div>
              <CodeBlock
                lang="bash"
                content="# Ver discovery en tiempo real
ros2 daemon stop
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 daemon start --verbose"
                :copyable="true"
              />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- QOS PROFILES -->
    <div class="section-group">
      <SectionTitle>3. QoS Profiles: Control de Calidad de Servicio</SectionTitle>
      <TextBlock>
        Los <strong>QoS Profiles</strong> permiten configurar el comportamiento de la comunicación
        según los requisitos de la aplicación. ROS 2 expone 10 políticas principales de DDS.
      </TextBlock>

      <div class="qos-table q-mt-md">
        <div class="qos-row header">
          <div class="qos-cell">Policy</div>
          <div class="qos-cell">Opciones</div>
          <div class="qos-cell">Caso de Uso</div>
        </div>

        <div class="qos-row">
          <div class="qos-cell">
            <strong>Reliability</strong>
          </div>
          <div class="qos-cell"><code>RELIABLE</code> / <code>BEST_EFFORT</code></div>
          <div class="qos-cell">
            RELIABLE: Comandos de control
            <br />
            BEST_EFFORT: Datos de sensores (alta frecuencia)
          </div>
        </div>

        <div class="qos-row">
          <div class="qos-cell">
            <strong>Durability</strong>
          </div>
          <div class="qos-cell"><code>VOLATILE</code> / <code>TRANSIENT_LOCAL</code></div>
          <div class="qos-cell">
            TRANSIENT_LOCAL: Parámetros de configuración
            <br />
            VOLATILE: Streams de video
          </div>
        </div>

        <div class="qos-row">
          <div class="qos-cell">
            <strong>History</strong>
          </div>
          <div class="qos-cell"><code>KEEP_LAST(n)</code> / <code>KEEP_ALL</code></div>
          <div class="qos-cell">
            KEEP_LAST(1): Solo dato más reciente
            <br />
            KEEP_ALL: Logging completo
          </div>
        </div>

        <div class="qos-row">
          <div class="qos-cell">
            <strong>Deadline</strong>
          </div>
          <div class="qos-cell">
            <code>Duration</code>
          </div>
          <div class="qos-cell">Detectar publishers que no publican a tiempo esperado</div>
        </div>

        <div class="qos-row">
          <div class="qos-cell">
            <strong>Lifespan</strong>
          </div>
          <div class="qos-cell">
            <code>Duration</code>
          </div>
          <div class="qos-cell">Expirar mensajes antiguos (ej: datos de sensores obsoletos)</div>
        </div>

        <div class="qos-row">
          <div class="qos-cell">
            <strong>Liveliness</strong>
          </div>
          <div class="qos-cell"><code>AUTOMATIC</code> / <code>MANUAL</code></div>
          <div class="qos-cell">Detectar nodos que han fallado</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="warning" title="QoS Compatibility">
          Publisher y Subscriber deben tener QoS <strong>compatibles</strong>. Ejemplo:
          <br />
          - RELIABLE publisher puede comunicarse con RELIABLE subscriber ✅
          <br />
          - RELIABLE publisher NO puede con BEST_EFFORT subscriber ❌
          <br />
          <br />
          Usa <code>ros2 topic info /topic --verbose</code> para ver QoS de publishers/subscribers
        </AlertBlock>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Ejemplo: Configuración QoS para diferentes escenarios"
          lang="cpp"
          content="// Sensor data (alta frecuencia, pérdida aceptable)
auto sensor_qos = rclcpp::SensorDataQoS();
// Equivalente a:
// - BEST_EFFORT
// - VOLATILE
// - KEEP_LAST(5)

// Control commands (crítico, sin pérdida)
auto control_qos = rclcpp::QoS(10);
control_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
control_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
control_qos.deadline(std::chrono::milliseconds(100));

// Parameters (persistente, late-joiners)
auto param_qos = rclcpp::ParametersQoS();
// Equivalente a:
// - RELIABLE
// - TRANSIENT_LOCAL
// - KEEP_ALL"
          :copyable="true"
        />
      </div>
    </div>

    <!-- PERFORMANCE METRICS -->
    <div class="section-group">
      <SectionTitle>4. Performance: Latencia y Throughput</SectionTitle>
      <TextBlock>
        El rendimiento de ROS 2 depende del modo de comunicación utilizado. La elección del
        transporte (UDP vs SHM vs Intra-process) define la latencia del sistema.
      </TextBlock>

      <div class="q-my-xl">
        <RmwVisualizer />
      </div>

      <div class="performance-comparison q-mt-md">
        <div class="perf-card">
          <div class="perf-header">
            <q-icon name="memory" color="blue-4" size="2rem" />
            <span>Intra-Process</span>
          </div>
          <div class="perf-metrics">
            <div class="metric">
              <div class="metric-label">Latencia</div>
              <div class="metric-value low">&lt; 10 μs</div>
            </div>
            <div class="metric">
              <div class="metric-label">Throughput</div>
              <div class="metric-value high">&gt; 10 GB/s</div>
            </div>
            <div class="metric">
              <div class="metric-label">CPU Overhead</div>
              <div class="metric-value low">Mínimo</div>
            </div>
          </div>
          <div class="perf-desc">
            Comunicación dentro del mismo proceso mediante punteros compartidos. Zero-copy cuando es
            posible.
          </div>
          <CodeBlock
            lang="cpp"
            content="// Habilitar intra-process
auto options = rclcpp::NodeOptions();
options.use_intra_process_comms(true);
auto node = std::make_shared<MyNode>(options);"
            :copyable="true"
          />
        </div>

        <div class="perf-card">
          <div class="perf-header">
            <q-icon name="storage" color="green-4" size="2rem" />
            <span>Shared Memory (IPC)</span>
          </div>
          <div class="perf-metrics">
            <div class="metric">
              <div class="metric-label">Latencia</div>
              <div class="metric-value medium">50-200 μs</div>
            </div>
            <div class="metric">
              <div class="metric-label">Throughput</div>
              <div class="metric-value high">1-5 GB/s</div>
            </div>
            <div class="metric">
              <div class="metric-label">CPU Overhead</div>
              <div class="metric-value medium">Bajo</div>
            </div>
          </div>
          <div class="perf-desc">
            Comunicación entre procesos en la misma máquina mediante memoria compartida (FastDDS
            SHM).
          </div>
          <CodeBlock
            lang="xml"
            content="<!-- fastdds_shm.xml -->
<transport_descriptor>
  <transport_id>SHM</transport_id>
  <type>SHM</type>
</transport_descriptor>"
            :copyable="true"
          />
        </div>

        <div class="perf-card">
          <div class="perf-header">
            <q-icon name="wifi" color="orange-4" size="2rem" />
            <span>Network (UDP)</span>
          </div>
          <div class="perf-metrics">
            <div class="metric">
              <div class="metric-label">Latencia</div>
              <div class="metric-value high">0.5-5 ms</div>
            </div>
            <div class="metric">
              <div class="metric-label">Throughput</div>
              <div class="metric-value medium">100-1000 MB/s</div>
            </div>
            <div class="metric">
              <div class="metric-label">CPU Overhead</div>
              <div class="metric-value high">Alto</div>
            </div>
          </div>
          <div class="perf-desc">
            Comunicación entre máquinas mediante red (Ethernet/WiFi). Serialización completa de
            mensajes.
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Benchmarks Reales</SectionTitle>
        <div class="benchmark-table q-mt-md">
          <div class="benchmark-row header">
            <div class="benchmark-cell">Tamaño Mensaje</div>
            <div class="benchmark-cell">Intra-Process</div>
            <div class="benchmark-cell">Shared Memory</div>
            <div class="benchmark-cell">UDP (localhost)</div>
          </div>
          <div class="benchmark-row">
            <div class="benchmark-cell">1 KB</div>
            <div class="benchmark-cell">8 μs</div>
            <div class="benchmark-cell">45 μs</div>
            <div class="benchmark-cell">120 μs</div>
          </div>
          <div class="benchmark-row">
            <div class="benchmark-cell">100 KB</div>
            <div class="benchmark-cell">12 μs</div>
            <div class="benchmark-cell">180 μs</div>
            <div class="benchmark-cell">850 μs</div>
          </div>
          <div class="benchmark-row">
            <div class="benchmark-cell">1 MB</div>
            <div class="benchmark-cell">45 μs</div>
            <div class="benchmark-cell">1.2 ms</div>
            <div class="benchmark-cell">8.5 ms</div>
          </div>
          <div class="benchmark-row">
            <div class="benchmark-cell">10 MB</div>
            <div class="benchmark-cell">380 μs</div>
            <div class="benchmark-cell">9.8 ms</div>
            <div class="benchmark-cell">85 ms</div>
          </div>
        </div>
        <div class="benchmark-note q-mt-sm">
          * Benchmarks realizados en Intel i7-10700K, Ubuntu 22.04, ROS 2 Humble, FastDDS 2.10
        </div>
      </div>
    </div>

    <!-- NETWORK TOPOLOGY -->
    <div class="section-group">
      <SectionTitle>5. Topología de Red y Domain ID</SectionTitle>
      <TextBlock>
        ROS 2 utiliza <strong>Domain ID</strong> para aislar grupos de nodos. Todos los nodos con el
        mismo Domain ID pueden comunicarse entre sí, pero están aislados de otros dominios.
      </TextBlock>

      <div class="domain-visual q-mt-md">
        <div class="domain-group">
          <div class="domain-label">Domain 0 (Robot A)</div>
          <div class="domain-nodes">
            <div class="domain-node">Camera</div>
            <div class="domain-node">Lidar</div>
            <div class="domain-node">Control</div>
          </div>
        </div>

        <div class="domain-separator">
          <q-icon name="block" size="3rem" color="red-4" />
          <div>Aislados</div>
        </div>

        <div class="domain-group">
          <div class="domain-label">Domain 1 (Robot B)</div>
          <div class="domain-nodes">
            <div class="domain-node">Camera</div>
            <div class="domain-node">Lidar</div>
            <div class="domain-node">Control</div>
          </div>
        </div>
      </div>

      <CodeBlock
        title="Configurar Domain ID"
        lang="bash"
        content="# Método 1: Variable de entorno
export ROS_DOMAIN_ID=42

# Método 2: En código (C++)
auto context = rclcpp::Context::make_shared();
rclcpp::InitOptions init_options;
init_options.set_domain_id(42);
context->init(0, nullptr, init_options);

# Método 3: En código (Python)
import rclpy
rclpy.init(domain_id=42)"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="info" title="Rango de Domain IDs">
          - Rango válido: <strong>0-101</strong> (ROS 2 Galactic+)
          <br />
          - Rango antiguo: 0-232 (ROS 2 Foxy y anteriores)
          <br />
          - Default: <strong>0</strong>
          <br />
          <br />
          Cada Domain ID mapea a puertos UDP específicos. Usar diferentes dominios permite múltiples
          robots en la misma red sin interferencia.
        </AlertBlock>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="ROS 2 Architecture Deep Dive"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" color="blue-4" size="sm" />
          Reemplaza con video técnico sobre DDS y arquitectura ROS 2
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>DDS Middleware</code>
          <span>Publish-Subscribe sobre UDP/TCP</span>
        </div>
        <div class="summary-item">
          <code>QoS Profiles</code>
          <span>10 políticas configurables</span>
        </div>
        <div class="summary-item">
          <code>Intra-Process</code>
          <span>&lt;10μs latencia, zero-copy</span>
        </div>
        <div class="summary-item">
          <code>Shared Memory</code>
          <span>50-200μs, 1-5 GB/s</span>
        </div>
        <div class="summary-item">
          <code>Domain ID</code>
          <span>Aislamiento de robots (0-101)</span>
        </div>
        <div class="summary-item">
          <code>Discovery</code>
          <span>SPDP + SEDP automático</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist Arquitectónico" class="q-mt-lg">
        ✅ Entender trade-offs: monolito vs distribuido
        <br />
        ✅ Configurar QoS según requisitos de aplicación
        <br />
        ✅ Usar intra-process para comunicación local (&gt;10x speedup)
        <br />
        ✅ Aislar robots con Domain IDs diferentes
        <br />
        ✅ Monitorear latencia con <code>ros2 topic hz</code> y <code>ros2 topic bw</code>
        <br />
        ✅ Debuggear QoS mismatches con <code>--verbose</code>
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import RmwVisualizer from 'components/content/interactive/RmwVisualizer.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* ARCHITECTURE COMPARISON */
.architecture-comparison {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.arch-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.arch-card.monolithic {
  border-color: #ef4444;
}

.arch-card.distributed {
  border-color: #00d9ff;
}

.arch-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.arch-visual {
  min-height: 200px;
  display: flex;
  align-items: center;
  justify-content: center;
  position: relative;
}

.monolith-block {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  padding: 1.5rem;
  background: rgba(239, 68, 68, 0.1);
  border: 2px solid #ef4444;
  border-radius: 12px;
}

.module {
  padding: 0.75rem 1.5rem;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 6px;
  color: #fca5a5;
  font-weight: 700;
}

.separator {
  color: #ef4444;
  font-weight: 700;
}

.graph-nodes {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
  z-index: 2;
}

.node-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  padding: 1rem;
  background: rgba(0, 217, 255, 0.1);
  border: 1px solid #00d9ff;
  border-radius: 8px;
  color: #93c5fd;
  font-weight: 700;
  font-size: 0.9rem;
}

.connections-overlay {
  position: absolute;
  width: 100%;
  height: 100%;
  top: 0;
  left: 0;
  z-index: 1;
  opacity: 0.5;
}

.connection-svg {
  width: 100%;
  height: 100%;
}

@keyframes data-flow {
  0% {
    stroke-dashoffset: 0;
  }
  100% {
    stroke-dashoffset: -20;
  }
}

.data-flow {
  animation: data-flow 2s linear infinite;
}

.problem-item,
.benefit-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  font-size: 0.9rem;
  color: #cbd5e1;
}

/* DDS ARCHITECTURE */
.dds-architecture {
  display: flex;
  flex-direction: column;
  gap: 0;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.dds-layer {
  padding: 1.5rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.dds-layer:last-of-type {
  border-bottom: none;
}

.layer-title {
  font-weight: 700;
  color: #00d9ff;
  margin-bottom: 0.5rem;
}

.layer-content {
  font-family: 'Fira Code', monospace;
  color: #94a3b8;
  font-size: 0.9rem;
}

.dds-arrow {
  text-align: center;
  color: #00ff88;
  font-size: 1.5rem;
  padding: 0.5rem;
  background: rgba(0, 255, 136, 0.1);
}

/* DISCOVERY PHASES */
.discovery-phases {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.phase-card {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 2rem;
}

.phase-number {
  width: 50px;
  height: 50px;
  background: linear-gradient(135deg, #667eea, #764ba2);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.phase-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #00d9ff;
  margin-bottom: 0.75rem;
}

.phase-desc {
  color: #cbd5e1;
  margin-bottom: 1rem;
}

/* QOS TABLE */
.qos-table {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.qos-row {
  display: grid;
  grid-template-columns: 1fr 1.5fr 2fr;
  gap: 1rem;
  padding: 1rem 1.5rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.qos-row.header {
  background: rgba(0, 217, 255, 0.1);
  font-weight: 700;
  color: #00d9ff;
}

.qos-row:last-child {
  border-bottom: none;
}

.qos-cell {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.qos-cell code {
  background: rgba(0, 0, 0, 0.5);
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  color: #00ff88;
}

/* PERFORMANCE COMPARISON */
.performance-comparison {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
}

.perf-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.perf-header {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
}

.perf-metrics {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.metric {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
}

.metric-label {
  color: #94a3b8;
  font-size: 0.85rem;
}

.metric-value {
  font-weight: 700;
  font-family: 'Fira Code', monospace;
}

.metric-value.low {
  color: #00ff88;
}

.metric-value.medium {
  color: #fbbf24;
}

.metric-value.high {
  color: #ef4444;
}

.perf-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
}

/* BENCHMARK TABLE */
.benchmark-table {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.benchmark-row {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 1rem;
  padding: 1rem 1.5rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.benchmark-row.header {
  background: rgba(0, 217, 255, 0.1);
  font-weight: 700;
  color: #00d9ff;
}

.benchmark-cell {
  color: #cbd5e1;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.benchmark-note {
  color: #64748b;
  font-size: 0.85rem;
  font-style: italic;
}

/* DOMAIN VISUAL */
.domain-visual {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.domain-group {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.domain-label {
  font-weight: 700;
  color: #00d9ff;
  text-align: center;
  padding: 0.5rem;
  background: rgba(0, 217, 255, 0.1);
  border-radius: 6px;
}

.domain-nodes {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.domain-node {
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 6px;
  color: #cbd5e1;
  text-align: center;
  font-family: 'Fira Code', monospace;
}

.domain-separator {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  color: #ef4444;
  font-weight: 700;
}

/* VIDEO */
.video-container {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 12px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #94a3b8;
  font-size: 0.85rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: #00d9ff;
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 1024px) {
  .architecture-comparison,
  .performance-comparison {
    grid-template-columns: 1fr;
  }

  .domain-visual {
    grid-template-columns: 1fr;
  }

  .domain-separator {
    transform: rotate(90deg);
  }
}
</style>
