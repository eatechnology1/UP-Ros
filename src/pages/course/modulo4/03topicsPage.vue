<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Los <strong>Topics</strong> implementan el patrón publish-subscribe desacoplado sobre DDS.
      Permiten comunicación asíncrona 1-a-N con QoS configurable, fundamental para streaming de
      datos de sensores y comandos de control en sistemas robóticos distribuidos.
    </TextBlock>

    <AlertBlock type="info" title="Características Clave">
      <strong>Desacoplamiento:</strong> Publisher/Subscriber no se conocen entre sí
      <br />
      <strong>Asíncrono:</strong> No bloquea, fire-and-forget
      <br />
      <strong>1-a-N:</strong> Un publisher, múltiples subscribers
      <br />
      <strong>Tipado fuerte:</strong> Interfaces .msg definen estructura
    </AlertBlock>

    <!-- PUB/SUB PATTERN -->
    <div class="section-group">
      <SectionTitle>1. Patrón Publisher-Subscriber</SectionTitle>

      <div class="pubsub-visual q-mt-md">
        <div class="pubsub-node publisher">
          <div class="node-icon">
            <q-icon name="videocam" size="2.5rem" />
          </div>
          <div class="node-label">Publisher</div>
          <div class="node-desc">Camera Node</div>
          <div class="node-code">
            <code>pub.publish(msg)</code>
          </div>
        </div>

        <div class="pubsub-pipe">
          <div class="pipe-line"></div>
          <div class="pipe-packets">
            <div class="packet" style="animation-delay: 0s"></div>
            <div class="packet" style="animation-delay: 1s"></div>
            <div class="packet" style="animation-delay: 2s"></div>
          </div>
          <div class="pipe-label">
            <div class="topic-name">/camera/image_raw</div>
            <div class="topic-type">sensor_msgs/Image</div>
          </div>
        </div>

        <div class="pubsub-subscribers">
          <div class="subscriber-node">
            <q-icon name="monitor" />
            <span>Display</span>
          </div>
          <div class="subscriber-node">
            <q-icon name="save" />
            <span>Logger</span>
          </div>
          <div class="subscriber-node">
            <q-icon name="psychology" />
            <span>Vision AI</span>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <div class="code-comparison">
          <div class="code-col">
            <div class="code-header">Publisher (C++)</div>
            <CodeBlock
              lang="cpp"
              content='auto publisher = node->create_publisher<std_msgs::msg::String>(
  "/topic_name", 10);

auto msg = std_msgs::msg::String();
msg.data = "Hello ROS 2";
publisher->publish(msg);'
              :copyable="true"
            />
          </div>
          <div class="code-col">
            <div class="code-header">Subscriber (Python)</div>
            <CodeBlock
              lang="python"
              content="def callback(msg):
    print(f'Received: {msg.data}')

subscription = node.create_subscription(
    String, '/topic_name', callback, 10)"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- MESSAGE TYPES -->
    <div class="section-group">
      <SectionTitle>2. Message Types e IDL</SectionTitle>
      <TextBlock>
        Los mensajes ROS 2 se definen usando <strong>IDL (Interface Definition Language)</strong>,
        un estándar OMG que permite interoperabilidad entre lenguajes. Se compilan a código C++,
        Python, etc.
      </TextBlock>

      <div class="message-anatomy q-mt-md">
        <div class="msg-card">
          <div class="msg-header">
            <q-icon name="description" />
            <span>Primitive Types</span>
          </div>
          <div class="msg-types">
            <div class="type-item">
              <code>bool</code>
              <span>true/false</span>
            </div>
            <div class="type-item">
              <code>int8, int16, int32, int64</code>
              <span>Enteros con signo</span>
            </div>
            <div class="type-item">
              <code>uint8, uint16, uint32, uint64</code>
              <span>Enteros sin signo</span>
            </div>
            <div class="type-item">
              <code>float32, float64</code>
              <span>Punto flotante</span>
            </div>
            <div class="type-item">
              <code>string</code>
              <span>UTF-8 string</span>
            </div>
          </div>
        </div>

        <div class="msg-card">
          <div class="msg-header">
            <q-icon name="view_array" />
            <span>Arrays & Sequences</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# Fixed-size array
float64[3] position

# Bounded sequence (max 100 elements)
float64[<=100] trajectory

# Unbounded sequence
string[] names"
            :copyable="true"
          />
        </div>

        <div class="msg-card">
          <div class="msg-header">
            <q-icon name="account_tree" />
            <span>Nested Messages</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# geometry_msgs/Twist.msg
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular

# geometry_msgs/Vector3.msg
float64 x
float64 y
float64 z"
            :copyable="true"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Crear custom message"
          lang="bash"
          content='# 1. Crear estructura
mkdir -p my_msgs/msg
cat > my_msgs/msg/RobotStatus.msg << EOF
string robot_id
float32 battery_level
geometry_msgs/Pose pose
bool is_moving
EOF

# 2. Agregar a CMakeLists.txt
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES geometry_msgs
)

# 3. Compilar
colcon build --packages-select my_msgs'
          :copyable="true"
        />
      </div>
    </div>

    <!-- QOS DEEP DIVE -->
    <div class="section-group">
      <SectionTitle>3. QoS Profiles: Configuración Avanzada</SectionTitle>
      <TextBlock>
        Los QoS profiles controlan el comportamiento de la comunicación. La compatibilidad entre
        publisher y subscriber es crítica para establecer conexión.
      </TextBlock>

      <div class="qos-matrix q-mt-md">
        <div class="matrix-header">
          <div class="matrix-title">Matriz de Compatibilidad QoS</div>
        </div>
        <div class="matrix-table">
          <div class="matrix-row header">
            <div class="matrix-cell">Publisher ↓ / Subscriber →</div>
            <div class="matrix-cell">RELIABLE</div>
            <div class="matrix-cell">BEST_EFFORT</div>
          </div>
          <div class="matrix-row">
            <div class="matrix-cell">RELIABLE</div>
            <div class="matrix-cell compatible">✅ Compatible</div>
            <div class="matrix-cell incompatible">❌ Incompatible</div>
          </div>
          <div class="matrix-row">
            <div class="matrix-cell">BEST_EFFORT</div>
            <div class="matrix-cell compatible">✅ Compatible</div>
            <div class="matrix-cell compatible">✅ Compatible</div>
          </div>
        </div>
      </div>

      <div class="qos-profiles q-mt-lg">
        <div class="profile-card sensor">
          <div class="profile-header">
            <q-icon name="sensors" />
            <span>SensorDataQoS</span>
          </div>
          <div class="profile-config">
            <div class="config-item">Reliability: <strong>BEST_EFFORT</strong></div>
            <div class="config-item">Durability: <strong>VOLATILE</strong></div>
            <div class="config-item">History: <strong>KEEP_LAST(5)</strong></div>
          </div>
          <div class="profile-use">
            <strong>Uso:</strong> Datos de sensores de alta frecuencia (IMU, Lidar, Camera)
          </div>
          <CodeBlock
            lang="cpp"
            content='auto qos = rclcpp::SensorDataQoS();
auto pub = node->create_publisher<sensor_msgs::msg::Image>(
  "/camera", qos);'
            :copyable="true"
          />
        </div>

        <div class="profile-card system">
          <div class="profile-header">
            <q-icon name="settings" />
            <span>SystemDefaultsQoS</span>
          </div>
          <div class="profile-config">
            <div class="config-item">Reliability: <strong>RELIABLE</strong></div>
            <div class="config-item">Durability: <strong>VOLATILE</strong></div>
            <div class="config-item">History: <strong>KEEP_LAST(10)</strong></div>
          </div>
          <div class="profile-use">
            <strong>Uso:</strong> Comandos de control, mensajes de estado
          </div>
        </div>

        <div class="profile-card params">
          <div class="profile-header">
            <q-icon name="tune" />
            <span>ParametersQoS</span>
          </div>
          <div class="profile-config">
            <div class="config-item">Reliability: <strong>RELIABLE</strong></div>
            <div class="config-item">Durability: <strong>TRANSIENT_LOCAL</strong></div>
            <div class="config-item">History: <strong>KEEP_ALL</strong></div>
          </div>
          <div class="profile-use">
            <strong>Uso:</strong> Parámetros de configuración, late-joiners
          </div>
        </div>
      </div>
    </div>

    <!-- INTRA-PROCESS -->
    <div class="section-group">
      <SectionTitle>4. Intra-Process Communication</SectionTitle>
      <TextBlock>
        Cuando publisher y subscriber están en el mismo proceso, ROS 2 puede usar
        <strong>zero-copy</strong> mediante punteros compartidos, eliminando serialización y
        logrando latencias &lt;10μs.
      </TextBlock>

      <div class="ipc-comparison q-mt-md">
        <div class="ipc-card standard">
          <div class="ipc-header">Standard (Inter-Process)</div>
          <div class="ipc-flow">
            <div class="flow-step">Publisher</div>
            <div class="flow-arrow">→ Serialize</div>
            <div class="flow-step">DDS</div>
            <div class="flow-arrow">→ Deserialize</div>
            <div class="flow-step">Subscriber</div>
          </div>
          <div class="ipc-metrics">
            <div class="metric-item">Latencia: 50-200μs</div>
            <div class="metric-item">CPU: Alto (serialización)</div>
            <div class="metric-item">Memoria: 2x (copia)</div>
          </div>
        </div>

        <div class="ipc-card optimized">
          <div class="ipc-header">Intra-Process (Zero-Copy)</div>
          <div class="ipc-flow">
            <div class="flow-step">Publisher</div>
            <div class="flow-arrow">→ Shared Ptr</div>
            <div class="flow-step">Subscriber</div>
          </div>
          <div class="ipc-metrics">
            <div class="metric-item">Latencia: &lt;10μs</div>
            <div class="metric-item">CPU: Mínimo</div>
            <div class="metric-item">Memoria: 1x (sin copia)</div>
          </div>
        </div>
      </div>

      <CodeBlock
        title="Habilitar intra-process"
        lang="cpp"
        content='// Configurar nodo
auto options = rclcpp::NodeOptions();
options.use_intra_process_comms(true);
auto node = std::make_shared<MyNode>(options);

// Publisher con unique_ptr (permite zero-copy)
auto msg = std::make_unique<std_msgs::msg::String>();
msg->data = "Hello";
publisher->publish(std::move(msg));'
        :copyable="true"
      />
    </div>

    <!-- CLI TOOLS -->
    <div class="section-group">
      <SectionTitle>5. Herramientas CLI</SectionTitle>

      <div class="cli-grid q-mt-md">
        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="list" />
            <span>Listar Topics</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Listar todos los topics
ros2 topic list

# Con tipos
ros2 topic list -t

# Verbose (incluye QoS)
ros2 topic list -v"
            :copyable="true"
          />
        </div>

        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="info" />
            <span>Información</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Ver publishers/subscribers
ros2 topic info /topic_name

# Ver QoS profiles
ros2 topic info /topic_name --verbose"
            :copyable="true"
          />
        </div>

        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="visibility" />
            <span>Monitorear</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Ver mensajes en tiempo real
ros2 topic echo /topic_name

# Frecuencia (Hz)
ros2 topic hz /topic_name

# Bandwidth
ros2 topic bw /topic_name"
            :copyable="true"
          />
        </div>

        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="publish" />
            <span>Publicar</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Publicar una vez
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Publicar a 10Hz
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}}'"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="ROS 2 Topics Deep Dive"
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
          Reemplaza con video técnico sobre Topics y QoS
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Pub/Sub</code>
          <span>Desacoplado, asíncrono, 1-a-N</span>
        </div>
        <div class="summary-item">
          <code>IDL Messages</code>
          <span>Tipado fuerte, interoperable</span>
        </div>
        <div class="summary-item">
          <code>QoS Profiles</code>
          <span>Reliability, Durability, History</span>
        </div>
        <div class="summary-item">
          <code>Intra-Process</code>
          <span>&lt;10μs, zero-copy</span>
        </div>
        <div class="summary-item">
          <code>ros2 topic</code>
          <span>list, info, echo, hz, bw, pub</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usar SensorDataQoS para datos de alta frecuencia
        <br />
        ✅ Habilitar intra-process para comunicación local
        <br />
        ✅ Verificar compatibilidad QoS con <code>--verbose</code>
        <br />
        ✅ Limitar tamaño de mensajes (&lt;1MB para buen rendimiento)
        <br />
        ✅ Usar arrays bounded para evitar allocations dinámicas
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
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* PUBSUB VISUAL */
.pubsub-visual {
  display: grid;
  grid-template-columns: auto 1fr auto;
  gap: 2rem;
  align-items: center;
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(0, 217, 255, 0.3);
  border-radius: 16px;
  padding: 3rem 2rem;
}

.pubsub-node {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.node-icon {
  width: 80px;
  height: 80px;
  background: linear-gradient(135deg, #3b82f6, #2563eb);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  box-shadow: 0 0 30px rgba(59, 130, 246, 0.5);
}

.node-label {
  font-weight: 700;
  color: #00d9ff;
  font-size: 1.1rem;
}

.node-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

.node-code {
  padding: 0.5rem 1rem;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 6px;
  font-family: 'Fira Code', monospace;
  color: #00ff88;
  font-size: 0.85rem;
}

.pubsub-pipe {
  position: relative;
  height: 100px;
  display: flex;
  flex-direction: column;
  justify-content: center;
}

.pipe-line {
  height: 4px;
  background: linear-gradient(90deg, #3b82f6, #00d9ff);
  border-radius: 2px;
}

.pipe-packets {
  position: absolute;
  width: 100%;
  height: 100%;
}

.packet {
  position: absolute;
  width: 20px;
  height: 20px;
  background: #00ff88;
  border-radius: 4px;
  top: 50%;
  transform: translateY(-50%);
  animation: packet-flow 3s linear infinite;
  box-shadow: 0 0 15px rgba(0, 255, 136, 0.7);
}

@keyframes packet-flow {
  0% {
    left: 0%;
    opacity: 0;
  }
  10% {
    opacity: 1;
  }
  90% {
    opacity: 1;
  }
  100% {
    left: 100%;
    opacity: 0;
  }
}

.pipe-label {
  position: absolute;
  top: -40px;
  left: 50%;
  transform: translateX(-50%);
  text-align: center;
}

.topic-name {
  font-family: 'Fira Code', monospace;
  color: #00d9ff;
  font-weight: 700;
  font-size: 1.05rem;
}

.topic-type {
  color: #64748b;
  font-size: 0.75rem;
}

.pubsub-subscribers {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.subscriber-node {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem 1.5rem;
  background: rgba(168, 85, 247, 0.1);
  border: 1px solid #a855f7;
  border-radius: 8px;
  color: #c4b5fd;
  font-weight: 700;
}

/* CODE COMPARISON */
.code-comparison {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.code-header {
  padding: 0.75rem 1rem;
  background: rgba(0, 217, 255, 0.1);
  border-bottom: 1px solid rgba(0, 217, 255, 0.3);
  font-weight: 700;
  color: #00d9ff;
  text-align: center;
}

/* MESSAGE ANATOMY */
.message-anatomy {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.msg-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.msg-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

.msg-types {
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.type-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
}

.type-item code {
  font-family: 'Fira Code', monospace;
  color: #00ff88;
  font-weight: 700;
}

.type-item span {
  color: #94a3b8;
  font-size: 0.85rem;
}

/* QOS MATRIX */
.qos-matrix {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.matrix-header {
  padding: 1rem 1.5rem;
  background: rgba(0, 217, 255, 0.1);
  border-bottom: 1px solid rgba(0, 217, 255, 0.3);
}

.matrix-title {
  font-weight: 700;
  color: #00d9ff;
  font-size: 1.1rem;
}

.matrix-table {
  padding: 1.5rem;
}

.matrix-row {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 1rem;
  margin-bottom: 1rem;
}

.matrix-row.header {
  font-weight: 700;
  color: #f1f5f9;
}

.matrix-cell {
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  text-align: center;
  color: #cbd5e1;
}

.matrix-cell.compatible {
  background: rgba(0, 255, 136, 0.1);
  border: 1px solid #00ff88;
  color: #00ff88;
  font-weight: 700;
}

.matrix-cell.incompatible {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid #ef4444;
  color: #fca5a5;
  font-weight: 700;
}

/* QOS PROFILES */
.qos-profiles {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.profile-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.profile-card.sensor {
  border-color: #3b82f6;
}

.profile-card.system {
  border-color: #00ff88;
}

.profile-card.params {
  border-color: #a855f7;
}

.profile-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.profile-config {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.config-item {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.config-item strong {
  color: #00d9ff;
  font-family: 'Fira Code', monospace;
}

.profile-use {
  color: #94a3b8;
  font-size: 0.85rem;
}

/* IPC COMPARISON */
.ipc-comparison {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.ipc-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.ipc-card.standard {
  border-color: #ff6b35;
}

.ipc-card.optimized {
  border-color: #00ff88;
}

.ipc-header {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
  text-align: center;
}

.ipc-flow {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  flex-wrap: wrap;
}

.flow-step {
  padding: 0.75rem 1.5rem;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 6px;
  color: #cbd5e1;
  font-weight: 700;
}

.flow-arrow {
  color: #00d9ff;
  font-weight: 700;
}

.ipc-metrics {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.metric-item {
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  color: #cbd5e1;
  font-size: 0.9rem;
}

/* CLI GRID */
.cli-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.cli-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.cli-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
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
  .pubsub-visual {
    grid-template-columns: 1fr;
  }

  .code-comparison,
  .ipc-comparison,
  .cli-grid {
    grid-template-columns: 1fr;
  }
}
</style>
