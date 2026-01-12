<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Las <strong>Actions</strong> son el mecanismo para tareas de larga duración con feedback
      continuo y capacidad de cancelación. Combinan lo mejor de Topics (feedback asíncrono) y
      Services (goal-oriented), ideales para navegación, manipulación, y tareas complejas.
    </TextBlock>

    <AlertBlock type="info" title="Características Clave">
      <strong>Goal-oriented:</strong> Cliente envía objetivo, servidor ejecuta
      <br />
      <strong>Feedback continuo:</strong> Reportes de progreso (0-100%)
      <br />
      <strong>Cancelable:</strong> Cliente puede abortar en cualquier momento
      <br />
      <strong>Preemptable:</strong> Nuevos goals pueden reemplazar anteriores
    </AlertBlock>

    <!-- ACTION ANATOMY -->
    <div class="section-group">
      <SectionTitle>1. Anatomía de una Action</SectionTitle>
      <TextBlock>
        Una Action no es un protocolo nuevo; es una composición inteligente de 3 canales DDS: 1
        servicio para goal, 1 tópico para feedback, 1 servicio para result.
      </TextBlock>

      <div class="action-visual q-mt-md">
        <div class="action-node client">
          <div class="node-icon">
            <q-icon name="flag" size="2.5rem" />
          </div>
          <div class="node-label">Action Client</div>
          <div class="node-state">Monitoring...</div>
        </div>

        <div class="action-channels">
          <div class="channel goal">
            <div class="channel-icon">
              <q-icon name="send" />
            </div>
            <div class="channel-info">
              <div class="channel-name">Goal (Service)</div>
              <div class="channel-desc">Navigate to (x, y, θ)</div>
            </div>
            <div class="channel-arrow">→</div>
          </div>

          <div class="channel feedback">
            <div class="channel-arrow">←</div>
            <div class="channel-info">
              <div class="channel-name">Feedback (Topic)</div>
              <div class="channel-desc">Progress: 45%, Distance: 2.3m</div>
            </div>
            <div class="channel-icon">
              <q-icon name="timeline" />
            </div>
          </div>

          <div class="channel result">
            <div class="channel-arrow">←</div>
            <div class="channel-info">
              <div class="channel-name">Result (Service)</div>
              <div class="channel-desc">Success! Final pose: (5, 3, 1.57)</div>
            </div>
            <div class="channel-icon">
              <q-icon name="check_circle" />
            </div>
          </div>
        </div>

        <div class="action-node server">
          <div class="node-icon">
            <q-icon name="smart_toy" size="2.5rem" />
          </div>
          <div class="node-label">Action Server</div>
          <div class="node-state">Executing...</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <div class="code-comparison">
          <div class="code-col">
            <div class="code-header">Server (C++)</div>
            <CodeBlock
              lang="cpp"
              content='auto action_server = rclcpp_action::create_server<Fibonacci>(
  node, "fibonacci",
  [](const GoalUUID &, std::shared_ptr<const Goal> goal) {
    // Accept/reject goal
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  },
  [](const std::shared_ptr<GoalHandle> goal_handle) {
    // Cancel callback
    return rclcpp_action::CancelResponse::ACCEPT;
  },
  [](const std::shared_ptr<GoalHandle> goal_handle) {
    // Execute callback
    auto result = std::make_shared<Result>();
    auto feedback = std::make_shared<Feedback>();

    for (int i = 0; i &lt; goal_handle->get_goal()->order; ++i) {
      feedback->partial_sequence.push_back(fib(i));
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(1s);
    }

    result->sequence = feedback->partial_sequence;
    goal_handle->succeed(result);
  });'
              :copyable="true"
            />
          </div>
          <div class="code-col">
            <div class="code-header">Client (Python)</div>
            <CodeBlock
              lang="python"
              content="client = ActionClient(node, Fibonacci, 'fibonacci')
client.wait_for_server()

goal = Fibonacci.Goal()
goal.order = 10

# Callbacks
def feedback_cb(feedback_msg):
    print(f'Progress: {feedback_msg.feedback.partial_sequence}')

def goal_response_cb(future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        print('Goal rejected')
        return

    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(result_cb)

def result_cb(future):
    result = future.result().result
    print(f'Final: {result.sequence}')

# Send goal
send_goal_future = client.send_goal_async(
    goal, feedback_callback=feedback_cb)
send_goal_future.add_done_callback(goal_response_cb)"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- ACTION INTERFACE -->
    <div class="section-group">
      <SectionTitle>2. Interfaces .action</SectionTitle>
      <TextBlock>
        Los archivos <code>.action</code> definen Goal, Result y Feedback. Se dividen por
        <code>---</code> en tres secciones.
      </TextBlock>

      <div class="action-examples q-mt-md">
        <div class="action-card">
          <div class="action-header">
            <q-icon name="calculate" />
            <span>Fibonacci.action</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence"
            :copyable="true"
          />
        </div>

        <div class="action-card">
          <div class="action-header">
            <q-icon name="navigation" />
            <span>NavigateToPose.action</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# Goal
geometry_msgs/PoseStamped pose
---
# Result
std_msgs/Empty
---
# Feedback
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
int16 number_of_recoveries
float32 distance_remaining"
            :copyable="true"
          />
        </div>

        <div class="action-card">
          <div class="action-header">
            <q-icon name="precision_manufacturing" />
            <span>PickPlace.action</span>
          </div>
          <CodeBlock
            lang="idl"
            content="# Goal
geometry_msgs/Pose pick_pose
geometry_msgs/Pose place_pose
---
# Result
bool success
string error_message
---
# Feedback
string current_state  # APPROACHING, GRASPING, LIFTING, etc.
float32 progress_percentage"
            :copyable="true"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Crear custom action"
          lang="bash"
          content='# 1. Crear estructura
mkdir -p my_interfaces/action
cat > my_interfaces/action/ComputePath.action << EOF
geometry_msgs/Pose start
geometry_msgs/Pose goal
---
nav_msgs/Path path
float32 total_distance
---
float32 progress_percentage
geometry_msgs/Pose current_waypoint
EOF

# 2. Agregar a CMakeLists.txt
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ComputePath.action"
  DEPENDENCIES geometry_msgs nav_msgs
)

# 3. Compilar
colcon build --packages-select my_interfaces'
          :copyable="true"
        />
      </div>
    </div>

    <!-- STATE MACHINE -->
    <div class="section-group">
      <SectionTitle>3. Action Server State Machine</SectionTitle>
      <TextBlock>
        El Action Server implementa una máquina de estados compleja que maneja múltiples goals
        concurrentes, cancelación y preemption.
      </TextBlock>

      <div class="state-diagram q-mt-md">
        <div class="state-flow">
          <div class="state idle">
            <div class="state-name">IDLE</div>
            <div class="state-desc">Esperando goal</div>
          </div>

          <div class="transition-line">↓ Goal received</div>

          <div class="state accepting">
            <div class="state-name">ACCEPTING</div>
            <div class="state-desc">Validando goal</div>
          </div>

          <div class="state-split">
            <div class="split-path">
              <div class="transition-line">← Rejected</div>
              <div class="state rejected">
                <div class="state-name">REJECTED</div>
              </div>
            </div>

            <div class="split-path">
              <div class="transition-line">→ Accepted</div>
              <div class="state executing">
                <div class="state-name">EXECUTING</div>
                <div class="state-desc">Publicando feedback</div>
              </div>
            </div>
          </div>

          <div class="state-outcomes">
            <div class="outcome succeeded">
              <div class="outcome-name">SUCCEEDED</div>
              <div class="outcome-desc">Goal completado</div>
            </div>
            <div class="outcome aborted">
              <div class="outcome-name">ABORTED</div>
              <div class="outcome-desc">Error interno</div>
            </div>
            <div class="outcome canceled">
              <div class="outcome-name">CANCELED</div>
              <div class="outcome-desc">Cliente canceló</div>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Design Choice: Goal Policy">
          Por defecto, <code>SimpleActionServer</code> (C++) usa <strong>Reject</strong> si ya hay
          un goal activo.
          <br />
          Para comportamientos avanzados (Navigation), debes implementar tu propia lógica de
          aceptación en `handle_goal`.
        </AlertBlock>

        <div class="q-my-xl">
          <ActionPolicyViz />
        </div>
      </div>
    </div>

    <!-- CANCELLATION -->
    <div class="section-group">
      <SectionTitle>4. Cancelación de Goals</SectionTitle>
      <TextBlock>
        Los clientes pueden cancelar goals en cualquier momento. El servidor debe manejar
        cancelación gracefully, liberando recursos y reportando estado final.
      </TextBlock>

      <div class="cancel-flow q-mt-md">
        <div class="cancel-step">
          <div class="step-number">1</div>
          <div class="step-content">
            <div class="step-title">Cliente solicita cancelación</div>
            <CodeBlock
              lang="python"
              content="# Cancelar goal específico
goal_handle.cancel_goal_async()

# Cancelar todos los goals
client.cancel_all_goals_async()"
              :copyable="true"
            />
          </div>
        </div>

        <div class="cancel-step">
          <div class="step-number">2</div>
          <div class="step-content">
            <div class="step-title">Servidor procesa cancelación</div>
            <CodeBlock
              lang="cpp"
              content='// Cancel callback
auto cancel_callback = [](const std::shared_ptr<GoalHandle> goal_handle) {
  RCLCPP_INFO(node->get_logger(), "Received cancel request");
  // Cleanup resources
  stop_motors();
  release_locks();
  return rclcpp_action::CancelResponse::ACCEPT;
};'
              :copyable="true"
            />
          </div>
        </div>

        <div class="cancel-step">
          <div class="step-number">3</div>
          <div class="step-content">
            <div class="step-title">Servidor reporta cancelación</div>
            <CodeBlock
              lang="cpp"
              content="// En execute callback
if (goal_handle->is_canceling()) {
  auto result = std::make_shared<Result>();
  result->success = false;
  goal_handle->canceled(result);
  return;
}"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- CLI TOOLS -->
    <div class="section-group">
      <SectionTitle>5. Herramientas CLI</SectionTitle>

      <div class="cli-grid q-mt-md">
        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="list" />
            <span>Listar Actions</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Listar todas las actions
ros2 action list

# Con tipos
ros2 action list -t"
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
            content="# Ver tipo de action
ros2 action type /fibonacci

# Ver interfaz
ros2 interface show example_interfaces/action/Fibonacci"
            :copyable="true"
          />
        </div>

        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="send" />
            <span>Enviar Goal</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Enviar goal y ver feedback
ros2 action send_goal /fibonacci \
  example_interfaces/action/Fibonacci \
  '{order: 10}' --feedback"
            :copyable="true"
          />
        </div>

        <div class="cli-card">
          <div class="cli-header">
            <q-icon name="search" />
            <span>Monitorear</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Ver información detallada
ros2 action info /fibonacci

# Incluye:
# - Número de action servers
# - Número de action clients"
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
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="ROS 2 Actions Deep Dive"
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
          Reemplaza con video técnico sobre Actions
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Goal-Oriented</code>
          <span>Tareas de larga duración</span>
        </div>
        <div class="summary-item">
          <code>3 Channels</code>
          <span>Goal, Feedback, Result</span>
        </div>
        <div class="summary-item">
          <code>Cancelable</code>
          <span>Abort en cualquier momento</span>
        </div>
        <div class="summary-item">
          <code>State Machine</code>
          <span>IDLE → EXECUTING → SUCCEEDED/ABORTED/CANCELED</span>
        </div>
        <div class="summary-item">
          <code>Preemption</code>
          <span>Manejo de múltiples goals</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usar Actions para tareas &gt;1 segundo (navegación, manipulación)
        <br />
        ✅ Publicar feedback frecuentemente (10-20 Hz)
        <br />
        ✅ Implementar cancelación graceful (liberar recursos)
        <br />
        ✅ Definir preemption policy clara (reject/abort/queue)
        <br />
        ✅ Incluir progress_percentage en feedback para UIs
        <br />
        ✅ Para tareas rápidas (&lt;100ms), usar Services en su lugar
      </AlertBlock>
    </div>

    <!-- ========== CTA FINAL ========== -->
    <div class="section-group q-mt-xl self-stretch column items-center">
      <div class="final-cta">
        <q-icon name="celebration" size="xl" color="primary" class="q-mb-md" />
        <h2 class="text-h4 text-white text-center q-mb-md">¡Has finalizado el módulo! 🎉</h2>
        <p class="text-body1 text-grey-4 text-center q-mb-lg">
          Has finalizado el módulo de Fundamentos de ROS2.
        </p>

        <div class="row q-gutter-md justify-center">
          <q-btn
            color="primary"
            unelevated
            rounded
            size="lg"
            padding="14px 40px"
            to="/modulo-5/01cli-colconPage"
            icon="rocket_launch"
            label="Comenzar con Herramientas de Desarrollo"
            class="text-weight-bold"
          />
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import ActionPolicyViz from 'components/content/interactive/ActionPolicyViz.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* ACTION VISUAL */
.action-visual {
  display: grid;
  grid-template-columns: auto 1fr auto;
  gap: 3rem;
  align-items: center;
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(251, 191, 36, 0.3);
  border-radius: 16px;
  padding: 3rem 2rem;
}

.action-node {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.node-icon {
  width: 80px;
  height: 80px;
  background: linear-gradient(135deg, #fbbf24, #f59e0b);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  box-shadow: 0 0 30px rgba(251, 191, 36, 0.5);
}

.node-label {
  font-weight: 700;
  color: #fde047;
  font-size: 1.1rem;
}

.node-state {
  color: #94a3b8;
  font-size: 0.85rem;
  font-style: italic;
}

.action-channels {
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.channel {
  display: grid;
  grid-template-columns: auto 1fr auto;
  gap: 1rem;
  align-items: center;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.channel.goal {
  border-left: 3px solid #3b82f6;
}

.channel.feedback {
  border-left: 3px solid #a855f7;
}

.channel.result {
  border-left: 3px solid #00ff88;
}

.channel-icon {
  color: #fbbf24;
}

.channel-info {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.channel-name {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 0.95rem;
}

.channel-desc {
  color: #94a3b8;
  font-size: 0.85rem;
  font-family: 'Fira Code', monospace;
}

.channel-arrow {
  color: #fbbf24;
  font-size: 1.5rem;
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
  background: rgba(251, 191, 36, 0.1);
  border-bottom: 1px solid rgba(251, 191, 36, 0.3);
  font-weight: 700;
  color: #fde047;
  text-align: center;
}

/* ACTION EXAMPLES */
.action-examples {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.action-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.action-header {
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

/* STATE DIAGRAM */
.state-diagram {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 3rem 2rem;
}

.state-flow {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1.5rem;
}

.state {
  padding: 1.5rem 3rem;
  border-radius: 12px;
  border: 2px solid;
  text-align: center;
  min-width: 200px;
}

.state.idle {
  border-color: #64748b;
  background: rgba(100, 116, 139, 0.1);
}

.state.accepting {
  border-color: #fbbf24;
  background: rgba(251, 191, 36, 0.1);
}

.state.rejected {
  border-color: #ef4444;
  background: rgba(239, 68, 68, 0.1);
}

.state.executing {
  border-color: #3b82f6;
  background: rgba(59, 130, 246, 0.1);
}

.state-name {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
  margin-bottom: 0.5rem;
}

.state-desc {
  color: #94a3b8;
  font-size: 0.85rem;
}

.transition-line {
  color: #00d9ff;
  font-weight: 700;
  text-align: center;
  margin: 0.5rem 0;
}

.state-split {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 3rem;
  width: 100%;
}

.split-path {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.state-outcomes {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
  width: 100%;
  margin-top: 2rem;
}

.outcome {
  padding: 1.5rem;
  border-radius: 12px;
  border: 2px solid;
  text-align: center;
}

.outcome.succeeded {
  border-color: #00ff88;
  background: rgba(0, 255, 136, 0.1);
}

.outcome.aborted {
  border-color: #ef4444;
  background: rgba(239, 68, 68, 0.1);
}

.outcome.canceled {
  border-color: #fbbf24;
  background: rgba(251, 191, 36, 0.1);
}

.outcome-name {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
  margin-bottom: 0.5rem;
}

.outcome-desc {
  color: #94a3b8;
  font-size: 0.85rem;
}

/* CANCEL FLOW */
.cancel-flow {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.cancel-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.step-number {
  width: 50px;
  height: 50px;
  background: linear-gradient(135deg, #ef4444, #dc2626);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.step-content {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.step-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
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
  color: #fbbf24;
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 1024px) {
  .action-visual {
    grid-template-columns: 1fr;
  }

  .code-comparison,
  .cli-grid,
  .state-outcomes {
    grid-template-columns: 1fr;
  }

  .state-split {
    grid-template-columns: 1fr;
  }
}
/* ========== CTA FINAL ========== */
.final-cta {
  text-align: center;
  margin: 0 auto;
}
</style>
