<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Los <strong>Services</strong> implementan el patrón request-reply síncrono. A diferencia de
      Topics (fire-and-forget), los servicios esperan respuesta, ideales para operaciones
      transaccionales como spawn de entidades, cálculos bajo demanda, o configuración de parámetros.
    </TextBlock>

    <AlertBlock type="info" title="Características Clave">
      <strong>Síncrono:</strong> Cliente espera respuesta (blocking por defecto)
      <br />
      <strong>1-a-1:</strong> Un cliente, un servidor por llamada
      <br />
      <strong>Transaccional:</strong> Request → Processing → Response
      <br />
      <strong>Tipado fuerte:</strong> Interfaces .srv definen request/response
    </AlertBlock>

    <!-- CLIENT/SERVER PATTERN -->
    <div class="section-group">
      <SectionTitle>1. Patrón Cliente-Servidor</SectionTitle>

      <div class="service-visual q-mt-md">
        <div class="service-node client">
          <div class="node-icon">
            <q-icon name="person" size="2.5rem" />
          </div>
          <div class="node-label">Client</div>
          <div class="node-state">Waiting...</div>
        </div>

        <div class="service-channel">
          <div class="channel-request">
            <div class="arrow-line request"></div>
            <div class="packet req">REQUEST</div>
            <div class="channel-label">spawn_entity</div>
          </div>
          <div class="channel-response">
            <div class="arrow-line response"></div>
            <div class="packet res">RESPONSE</div>
            <div class="channel-time">~50ms</div>
          </div>
        </div>

        <div class="service-node server">
          <div class="node-icon">
            <q-icon name="dns" size="2.5rem" />
          </div>
          <div class="node-label">Server</div>
          <div class="node-state">Processing...</div>
        </div>
      </div>

      <AlertBlock type="warning" title="Anti-Pattern: Synchronous Calls">
        NUNCA llames a un servicio síncronamente (<code>client.call()</code>) dentro de un callback.
        <br />
        Esto causa <strong>Deadlocks</strong> porque el SingleThreadedExecutor no puede procesar la
        respuesta mientras está bloqueado esperando la llamada.
        <br />
        <strong>Solución:</strong> Usa `call_async()` y `spin_until_future_complete` (en main) o
        callbacks.
      </AlertBlock>

      <div class="q-mt-lg">
        <div class="code-comparison">
          <div class="code-col">
            <div class="code-header">Server (C++)</div>
            <CodeBlock
              lang="cpp"
              content='auto service = node->create_service<AddTwoInts>(
  "/add_two_ints",
  [](const Request::SharedPtr req,
     Response::SharedPtr res) {
    res->sum = req->a + req->b;
    RCLCPP_INFO(node->get_logger(),
      "Request: %ld + %ld = %ld",
      req->a, req->b, res->sum);
  });'
              :copyable="true"
            />
          </div>
          <div class="code-col">
            <div class="code-header">Client (Python)</div>
            <CodeBlock
              lang="python"
              content="client = node.create_client(AddTwoInts, '/add_two_ints')
client.wait_for_service()

request = AddTwoInts.Request()
request.a = 10
request.b = 20

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result()"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- SRV INTERFACE -->
    <div class="section-group">
      <SectionTitle>2. Interfaces .srv</SectionTitle>
      <TextBlock>
        Los archivos <code>.srv</code> definen el contrato request/response. Se dividen por tres
        guiones (<code>---</code>): arriba request, abajo response.
      </TextBlock>

      <div class="srv-examples q-mt-md">
        <div class="srv-card">
          <div class="srv-header">
            <q-icon name="calculate" />
            <span>AddTwoInts.srv</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# Request
int64 a
int64 b
---
# Response
int64 sum"
            :copyable="true"
          />
        </div>

        <div class="srv-card">
          <div class="srv-header">
            <q-icon name="smart_toy" />
            <span>SpawnEntity.srv</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# Request
string name
geometry_msgs/Pose initial_pose
---
# Response
bool success
string status_message"
            :copyable="true"
          />
        </div>

        <div class="srv-card">
          <div class="srv-header">
            <q-icon name="settings" />
            <span>SetParameters.srv</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# Request
Parameter[] parameters
---
# Response
SetParametersResult[] results"
            :copyable="true"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Crear custom service"
          lang="bash"
          content='# 1. Crear estructura
mkdir -p my_interfaces/srv
cat > my_interfaces/srv/ComputeTrajectory.srv << EOF
geometry_msgs/Pose start
geometry_msgs/Pose goal
float32 max_velocity
---
nav_msgs/Path trajectory
float32 estimated_time
EOF

# 2. Agregar a CMakeLists.txt
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeTrajectory.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# 3. Compilar
colcon build --packages-select my_interfaces'
          :copyable="true"
        />
      </div>
    </div>

    <!-- ASYNC CLIENTS -->
    <div class="section-group">
      <SectionTitle>3. Clientes Asíncronos</SectionTitle>
      <TextBlock>
        Por defecto, los clientes son <strong>bloqueantes</strong>. Para evitar congelar el nodo,
        usa clientes asíncronos con callbacks o futures.
      </TextBlock>

      <div class="async-patterns q-mt-md">
        <div class="pattern-card">
          <div class="pattern-header">
            <q-icon name="block" color="red-4" />
            <span>Síncrono (Blocking)</span>
          </div>
          <div class="pattern-desc">
            El nodo se congela hasta recibir respuesta. NO recomendado.
          </div>
          <CodeBlock
            lang="python"
            content="# ❌ Bloquea el nodo
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result()  # Espera aquí"
            :copyable="true"
          />
          <div class="pattern-warning">⚠️ Puede causar deadlocks</div>
        </div>

        <div class="pattern-card">
          <div class="pattern-header">
            <q-icon name="check_circle" color="green-4" />
            <span>Asíncrono (Callback)</span>
          </div>
          <div class="pattern-desc">
            Callback se ejecuta cuando llega respuesta. El nodo sigue procesando.
          </div>
          <CodeBlock
            lang="python"
            content="def response_callback(future):
    result = future.result()
    print(f'Sum: {result.sum}')

future = client.call_async(request)
future.add_done_callback(response_callback)
# Nodo continúa ejecutando"
            :copyable="true"
          />
          <div class="pattern-benefit">✅ No bloquea el nodo</div>
        </div>

        <div class="pattern-card">
          <div class="pattern-header">
            <q-icon name="timer" color="blue-4" />
            <span>Con Timeout</span>
          </div>
          <div class="pattern-desc">Evita esperar indefinidamente si el servidor no responde.</div>
          <CodeBlock
            lang="cpp"
            content='auto future = client->async_send_request(request);

// Esperar máximo 2 segundos
if (rclcpp::spin_until_future_complete(
      node, future, std::chrono::seconds(2)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  auto result = future.get();
} else {
  RCLCPP_ERROR(node->get_logger(), "Timeout!");
}'
            :copyable="true"
          />
          <div class="pattern-benefit">✅ Manejo robusto de errores</div>
        </div>
      </div>
    </div>

    <!-- LIFECYCLE NODES -->
    <div class="section-group">
      <SectionTitle>4. Lifecycle Nodes</SectionTitle>
      <TextBlock>
        Los <strong>Managed Nodes</strong> implementan una máquina de estados que permite control
        fino del ciclo de vida del nodo. Útil para sistemas críticos que requieren inicialización y
        apagado controlados.
      </TextBlock>

      <div class="q-my-xl">
        <LifecycleDashboard />
      </div>

      <div class="lifecycle-diagram q-mt-md">
        <div class="state-machine">
          <div class="state unconfigured">
            <div class="state-name">Unconfigured</div>
            <div class="state-desc">Estado inicial</div>
          </div>

          <div class="transition">
            <div class="transition-arrow">→</div>
            <div class="transition-label">configure()</div>
          </div>

          <div class="state inactive">
            <div class="state-name">Inactive</div>
            <div class="state-desc">Configurado pero no activo</div>
          </div>

          <div class="transition">
            <div class="transition-arrow">→</div>
            <div class="transition-label">activate()</div>
          </div>

          <div class="state active">
            <div class="state-name">Active</div>
            <div class="state-desc">Ejecutando normalmente</div>
          </div>

          <div class="transition">
            <div class="transition-arrow">→</div>
            <div class="transition-label">deactivate()</div>
          </div>

          <div class="state inactive">
            <div class="state-name">Inactive</div>
          </div>

          <div class="transition">
            <div class="transition-arrow">→</div>
            <div class="transition-label">cleanup()</div>
          </div>

          <div class="state unconfigured">
            <div class="state-name">Unconfigured</div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Implementar Lifecycle Node (C++)"
          lang="cpp"
          content='class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  CallbackReturn on_configure(const State &) override {
    // Inicializar recursos (abrir archivos, conectar sensores)
    RCLCPP_INFO(get_logger(), "Configuring...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const State &) override {
    // Activar publicación/suscripción
    RCLCPP_INFO(get_logger(), "Activating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const State &) override {
    // Pausar operaciones
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const State &) override {
    // Liberar recursos
    return CallbackReturn::SUCCESS;
  }
};'
          :copyable="true"
        />
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Controlar lifecycle desde CLI"
          lang="bash"
          content="# Ver estado actual
ros2 lifecycle get /my_node

# Transiciones
ros2 lifecycle set /my_node configure
ros2 lifecycle set /my_node activate
ros2 lifecycle set /my_node deactivate
ros2 lifecycle set /my_node cleanup"
          :copyable="true"
        />
      </div>
    </div>

    <!-- CLI TOOLS -->
    <div class="section-group">
      <SectionTitle>5. Herramientas CLI</SectionTitle>

      <div class="cli-grid q-mt-md">
        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="list" />
            <span>Listar Services</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Listar todos los servicios
ros2 service list

# Con tipos
ros2 service list -t"
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
            content="# Ver tipo de servicio
ros2 service type /service_name

# Ver interfaz
ros2 interface show example_interfaces/srv/AddTwoInts"
            :copyable="true"
          />
        </div>

        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="call" />
            <span>Llamar Servicio</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Llamar con argumentos
ros2 service call /add_two_ints \
  example_interfaces/srv/AddTwoInts \
  '{a: 10, b: 20}'

# Ver resultado
# sum: 30"
            :copyable="true"
          />
        </div>

        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="search" />
            <span>Encontrar Servicios</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Servicios de un tipo específico
ros2 service find std_srvs/srv/SetBool

# Servicios de un nodo
ros2 node info /my_node"
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
            src="https://youtu.be/Romc22GgusU"
            title="ROS 2 Services Deep Dive"
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
          Reemplaza con video técnico sobre Services y Lifecycle
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Request-Reply</code>
          <span>Síncrono, 1-a-1, transaccional</span>
        </div>
        <div class="summary-item">
          <code>.srv Files</code>
          <span>Request --- Response</span>
        </div>
        <div class="summary-item">
          <code>Async Clients</code>
          <span>Callbacks, futures, timeouts</span>
        </div>
        <div class="summary-item">
          <code>Lifecycle Nodes</code>
          <span>State machine controlada</span>
        </div>
        <div class="summary-item">
          <code>ros2 service</code>
          <span>list, type, call, find</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usar clientes asíncronos con callbacks (no bloquear nodo)
        <br />
        ✅ Implementar timeouts para evitar esperas infinitas
        <br />
        ✅ Usar Lifecycle Nodes para sistemas críticos
        <br />
        ✅ Mantener servicios rápidos (&lt;100ms idealmente)
        <br />
        ✅ Para operaciones largas, usar Actions en su lugar
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
import LifecycleDashboard from 'components/content/interactive/LifecycleDashboard.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* SERVICE VISUAL */
.service-visual {
  display: grid;
  grid-template-columns: auto 1fr auto;
  gap: 3rem;
  align-items: center;
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(168, 85, 247, 0.3);
  border-radius: 16px;
  padding: 3rem 2rem;
}

.service-node {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.node-icon {
  width: 80px;
  height: 80px;
  background: linear-gradient(135deg, #a855f7, #9333ea);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  box-shadow: 0 0 30px rgba(168, 85, 247, 0.5);
}

.node-label {
  font-weight: 700;
  color: #c4b5fd;
  font-size: 1.1rem;
}

.node-state {
  color: #94a3b8;
  font-size: 0.85rem;
  font-style: italic;
}

.service-channel {
  display: flex;
  flex-direction: column;
  gap: 2rem;
  position: relative;
}

.channel-request,
.channel-response {
  position: relative;
  height: 40px;
}

.arrow-line {
  position: absolute;
  top: 50%;
  width: 100%;
  height: 3px;
  transform: translateY(-50%);
}

.arrow-line.request {
  background: linear-gradient(90deg, #a855f7, transparent);
}

.arrow-line.response {
  background: linear-gradient(90deg, transparent, #00ff88);
}

.packet {
  position: absolute;
  top: 50%;
  transform: translateY(-50%);
  padding: 0.5rem 1rem;
  border-radius: 6px;
  font-weight: 700;
  font-size: 0.75rem;
  animation: packet-move 3s ease-in-out infinite;
}

.packet.req {
  left: 0;
  background: #a855f7;
  color: white;
}

.packet.res {
  right: 0;
  background: #00ff88;
  color: black;
}

@keyframes packet-move {
  0%,
  100% {
    opacity: 0;
  }
  50% {
    opacity: 1;
  }
}

.channel-label {
  position: absolute;
  top: -30px;
  left: 50%;
  transform: translateX(-50%);
  font-family: 'Fira Code', monospace;
  color: #c4b5fd;
  font-weight: 700;
}

.channel-time {
  position: absolute;
  bottom: -25px;
  right: 0;
  color: #64748b;
  font-size: 0.75rem;
}

/* CODE COMPARISON */
.code-comparison {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.code-header {
  padding: 0.75rem 1rem;
  background: rgba(168, 85, 247, 0.1);
  border-bottom: 1px solid rgba(168, 85, 247, 0.3);
  font-weight: 700;
  color: #c4b5fd;
  text-align: center;
}

/* SRV EXAMPLES */
.srv-examples {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.srv-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.srv-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  font-family: 'Fira Code', monospace;
  color: #f1f5f9;
}

/* ASYNC PATTERNS */
.async-patterns {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.pattern-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.pattern-card:nth-child(1) {
  border-color: #ef4444;
}

.pattern-card:nth-child(2) {
  border-color: #00ff88;
}

.pattern-card:nth-child(3) {
  border-color: #3b82f6;
}

.pattern-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.pattern-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.pattern-warning {
  padding: 1rem;
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid #ef4444;
  border-radius: 8px;
  color: #fca5a5;
  font-weight: 700;
  text-align: center;
}

.pattern-benefit {
  padding: 1rem;
  background: rgba(0, 255, 136, 0.1);
  border: 1px solid #00ff88;
  border-radius: 8px;
  color: #00ff88;
  font-weight: 700;
  text-align: center;
}

/* LIFECYCLE DIAGRAM */
.lifecycle-diagram {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 3rem 2rem;
}

.state-machine {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.state {
  padding: 1.5rem 3rem;
  border-radius: 12px;
  border: 2px solid;
  text-align: center;
  min-width: 250px;
}

.state.unconfigured {
  border-color: #64748b;
  background: rgba(100, 116, 139, 0.1);
}

.state.inactive {
  border-color: #fbbf24;
  background: rgba(251, 191, 36, 0.1);
}

.state.active {
  border-color: #00ff88;
  background: rgba(0, 255, 136, 0.1);
}

.state-name {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.2rem;
  margin-bottom: 0.5rem;
}

.state-desc {
  color: #94a3b8;
  font-size: 0.85rem;
}

.transition {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.transition-arrow {
  color: #00d9ff;
  font-size: 2rem;
  font-weight: 700;
}

.transition-label {
  font-family: 'Fira Code', monospace;
  color: #00d9ff;
  font-weight: 700;
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
  color: #a855f7;
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 1024px) {
  .service-visual {
    grid-template-columns: 1fr;
  }

  .code-comparison,
  .cli-grid {
    grid-template-columns: 1fr;
  }
}
</style>
