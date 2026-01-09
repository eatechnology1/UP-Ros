<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <div class="row items-center q-gutter-md">
        <div class="col-grow">
          <TextBlock>
            Imagina que pides una pizza. No es instantáneo (como un Servicio). Tarda tiempo.
            <br />
            Mientras esperas, quieres saber: "¿Ya amasaron?", "¿Ya entró al horno?", "¿Falta
            mucho?".
            <br />
            Y si de repente te arrepientes, quieres poder gritar: "¡CANCELA EL PEDIDO!".
            <br /><br />
            Esto es una <strong>Acción</strong>: Una tarea larga, con feedback en vivo y capacidad
            de cancelación.
          </TextBlock>
        </div>
        <div class="col-auto">
          <div class="text-h1">🍕</div>
        </div>
      </div>
    </div>

    <!-- 1. LA TRINIDAD DE LA ACCIÓN -->
    <div class="section-group">
      <SectionTitle>1. Anatomía de una Pizza (.action)</SectionTitle>
      <TextBlock>
        A diferencia de los servicios (2 partes), las acciones tienen **3 partes** separadas por
        guiones.
      </TextBlock>

      <div class="action-card q-my-md">
        <div class="action-header">CocinarPizza.action</div>
        <div class="action-body">
          <div class="part goal">
            <span class="badge">GOAL (Meta)</span>
            <code>int32 orden_id</code><br />
            <code>string tipo_pizza</code>
          </div>
          <div class="separator">---</div>
          <div class="part result">
            <span class="badge">RESULT (Final)</span>
            <code>bool pizza_lista</code><br />
            <code>string mensaje_chef</code>
          </div>
          <div class="separator">---</div>
          <div class="part feedback">
            <span class="badge">FEEDBACK (En vivo)</span>
            <code>string estado_actual</code><br />
            <code>int32 porcentaje_coccion</code>
          </div>
        </div>
      </div>
    </div>

    <!-- 2. SERVIDOR DE ACCIÓN (EL CHEF) -->
    <div class="section-group">
      <SectionTitle>2. El Chef Robot (Action Server)</SectionTitle>
      <TextBlock>
        El código de una Acción es más largo que un día sin pan. Vamos a analizar la lógica del
        Chef.
        <br />
        Usa la clase <code>ActionServer</code> y necesita definir qué hacer cuando recibe un pedido,
        cómo cancelarlo y cómo ejecutarlo.
      </TextBlock>

      <CodeBlock
        title="pizza_chef.py (Lógica simplificada)"
        lang="python"
        content="class PizzaActionServer(Node):
    def __init__(self):
        self._action_server = ActionServer(
            self,
            CocinarPizza,
            'cocinar_pizza',
            self.execute_callback) # <--- La magia ocurre aquí

    def execute_callback(self, goal_handle):
        self.get_logger().info('👨‍🍳 Pedido recibido. Iniciando horno...')

        feedback_msg = CocinarPizza.Feedback()

        # Simulamos el proceso largo (Cocinar)
        for i in range(1, 101):
            # 1. Verificar si el cliente canceló
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return CocinarPizza.Result(pizza_lista=False)

            # 2. Enviar Feedback (Barra de progreso)
            feedback_msg.porcentaje_coccion = i
            feedback_msg.estado_actual = 'Horneando...'
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1) # Simulamos tiempo de cocción

        # 3. Terminar con éxito
        goal_handle.succeed()
        result = CocinarPizza.Result()
        result.pizza_lista = True
        result.mensaje_chef = '¡Pizza Hawaiana lista para comer!'
        return result"
      />
    </div>

    <!-- 3. LLAMANDO A LA ACCIÓN (CLI) -->
    <div class="section-group">
      <SectionTitle>3. Pidiendo Pizza desde la Terminal</SectionTitle>
      <TextBlock>
        Como programar un cliente de acción en Python es muy avanzado (requiere callbacks de goal,
        feedback y result), vamos a usar la terminal para probar nuestro servidor.
      </TextBlock>

      <div class="terminal-demo">
        <div class="cmd">
          ros2 action send_goal /cocinar_pizza my_robot_interfaces/action/CocinarPizza "{tipo_pizza:
          'Hawaiana'}" --feedback
        </div>
        <div class="output">
          <span class="text-blue">[INFO]:</span> Goal accepted with ID: 1234abcd...<br />
          <span class="text-yellow">[FEEDBACK]:</span> porcentaje_coccion: 10<br />
          <span class="text-yellow">[FEEDBACK]:</span> porcentaje_coccion: 20<br />
          <span class="text-yellow">[FEEDBACK]:</span> porcentaje_coccion: 30<br />
          ...<br />
          <span class="text-green">[RESULT]:</span> pizza_lista: True, mensaje_chef: '¡Pizza
          Hawaiana lista para comer!'
        </div>
      </div>
    </div>

    <!-- 4. CANCELACIÓN -->
    <div class="section-group">
      <SectionTitle>4. ¡Espera, soy alérgico! (Cancelación)</SectionTitle>
      <TextBlock>
        La gran diferencia con los servicios es que aquí tienes un botón de
        <strong>ABORTAR</strong>.
        <br />
        En la vida real, esto es vital para la navegación: "Ve al punto X". "¡Espera, hay un
        precipicio! ¡DETENTE!".
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="visual-cancel text-center">
            <div class="text-h1">🛑</div>
            <p>Ctrl + C</p>
          </div>
        </template>
        <template #right>
          <TextBlock>
            Si ejecutas el comando anterior y presionas <code>Ctrl+C</code> a mitad de camino, el
            cliente enviará una señal de cancelación. <br /><br />
            El servidor (Chef) debe ser lo suficientemente inteligente para detectar esa señal
            (`is_cancel_requested`), apagar el horno y responder "Pedido Cancelado".
          </TextBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. ¿CUÁNDO USAR QUÉ? -->
    <div class="section-group">
      <SectionTitle>5. La Guía Definitiva de Comunicación</SectionTitle>
      <TextBlock> Ahora conoces las tres armas de ROS 2. ¿Cuál usas en cada caso? </TextBlock>

      <div class="comparison-table q-mt-md">
        <div class="row header">
          <div class="col-3">Tipo</div>
          <div class="col-3">Tiempo</div>
          <div class="col-3">Feedback</div>
          <div class="col-3">Ejemplo</div>
        </div>
        <div class="row row-item">
          <div class="col-3 text-weight-bold text-blue">Tópico</div>
          <div class="col-3">Continuo</div>
          <div class="col-3">N/A</div>
          <div class="col-3">Velocímetro</div>
        </div>
        <div class="row row-item">
          <div class="col-3 text-weight-bold text-green">Servicio</div>
          <div class="col-3">Rápido</div>
          <div class="col-3">No</div>
          <div class="col-3">Encender Luz</div>
        </div>
        <div class="row row-item">
          <div class="col-3 text-weight-bold text-purple">Acción</div>
          <div class="col-3">Largo</div>
          <div class="col-3">Sí</div>
          <div class="col-3">Navegar a Cocina</div>
        </div>
      </div>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Conceptual</SectionTitle>
      <TextBlock>
        En el robot <strong>Turtlesim</strong> existe una acción llamada
        <code>/turtle1/rotate_absolute</code>.
        <br />
        1. Ejecuta el simulador. 2. Usa `ros2 action list` para encontrarla. 3. Usa `ros2 action
        send_goal` para decirle: "Gira a 3.14 radianes (180 grados)". 4. Observa cómo la tortuga
        gira lentamente (¡Acción!) en lugar de teletransportarse (Servicio).
      </TextBlock>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

/* Visualización .action */
.action-card {
  background: #1e293b;
  border-radius: 12px;
  overflow: hidden;
  max-width: 500px;
  margin: 0 auto;
  box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
}
.action-header {
  background: #4f46e5; /* Indigo */
  color: white;
  padding: 10px;
  text-align: center;
  font-weight: bold;
  font-family: 'Fira Code', monospace;
}
.action-body {
  padding: 20px;
  font-family: 'Fira Code', monospace;
}
.part {
  margin-bottom: 10px;
  position: relative;
  padding-left: 20px;
}
.separator {
  color: #64748b;
  font-weight: bold;
  letter-spacing: 3px;
  margin: 10px 0;
  text-align: center;
}
.badge {
  position: absolute;
  left: -10px;
  top: 0;
  font-size: 0.6rem;
  padding: 2px 6px;
  border-radius: 4px;
  color: #1e293b;
  font-weight: bold;
  transform: rotate(-90deg) translateX(-10px);
}
.part.goal .badge {
  background: #fbbf24;
}
.part.result .badge {
  background: #4ade80;
}
.part.feedback .badge {
  background: #38bdf8;
}

/* Terminal Demo */
.terminal-demo {
  background: #000;
  border-radius: 8px;
  padding: 15px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}
.cmd {
  color: #a5f3fc;
  margin-bottom: 10px;
  border-bottom: 1px solid #333;
  padding-bottom: 5px;
}
.output {
  color: #ccc;
  line-height: 1.5;
}
.text-blue {
  color: #60a5fa;
}
.text-yellow {
  color: #fcd34d;
}
.text-green {
  color: #4ade80;
}

/* Tabla Comparativa */
.comparison-table {
  background: rgba(30, 41, 59, 0.5);
  border-radius: 8px;
  overflow: hidden;
}
.header {
  background: rgba(255, 255, 255, 0.1);
  padding: 10px;
  font-weight: bold;
  text-align: center;
}
.row-item {
  padding: 10px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  text-align: center;
  font-size: 0.9rem;
}
.row-item:last-child {
  border-bottom: none;
}

.visual-cancel {
  background: rgba(239, 68, 68, 0.1);
  border-radius: 12px;
  padding: 20px;
}
</style>
