<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <div class="row items-center q-gutter-md">
        <div class="col-grow">
          <TextBlock>
            El Radar está enviando datos. Ahora necesitamos un nodo que
            <strong>escuche</strong> esos datos y active los escudos automáticamente si hay peligro.
            <br /><br />
            En ROS 2, "escuchar" es un arte pasivo. Tú no buscas los datos; te sientas a esperar a
            que lleguen. A esto le llamamos <strong>Callback</strong>.
          </TextBlock>
        </div>
        <div class="col-auto">
          <div class="text-h1">🛡️</div>
        </div>
      </div>
    </div>

    <!-- 1. CONCEPTO: EL CALLBACK -->
    <div class="section-group">
      <SectionTitle>1. La Trampa del Ratón (Callback)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Un Subscriber no tiene un bucle infinito leyendo datos. Funciona por
            <strong>Eventos</strong>. <br /><br />
            Configuras una función especial (Callback) y le dices a ROS:
            <em
              >"Oye, cada vez que llegue un mensaje al tópico '/alerta', despiértame y ejecuta esta
              función"</em
            >. <br /><br />
            El resto del tiempo, el nodo está dormido ahorrando energía.
          </TextBlock>
        </template>
        <template #right>
          <div class="callback-visual q-pa-md text-center">
            <div class="envelope">📩 Mensaje</div>
            <div class="arrow-down">⬇ llega ⬇</div>
            <div class="function-box">
              <div class="func-name">def listener_callback(self, msg):</div>
              <div class="func-body">
                Procesar(msg)<br />
                Activar_Escudos()
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. CÓDIGO DEL SISTEMA DE DEFENSA -->
    <div class="section-group">
      <SectionTitle>2. Código: Escudos.py</SectionTitle>
      <TextBlock>
        Vamos a crear el archivo <code>sistema_escudos.py</code>. Nota cómo no hay ningún Timer
        aquí. El ritmo lo marca el Publisher (el Radar).
      </TextBlock>

      <CodeBlock
        title="sistema_escudos.py"
        lang="python"
        content="import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EscudosNode(Node):
    def __init__(self):
        super().__init__('escudos_node')

        # --- SUSCRIPCIÓN ---
        # 1. Tipo de mensaje (Debe coincidir con el Publisher)
        # 2. Nombre del tópico (Debe ser exacto)
        # 3. La función a ejecutar (Callback)
        # 4. Cola (QoS)
        self.subscription = self.create_subscription(
            String,
            'alerta_meteoros',
            self.procesar_alerta,
            10)

        # Evitar warning de variable no usada
        self.subscription

        self.get_logger().info('🛡️ Sistema de Escudos: ONLINE. Esperando datos...')

    # --- EL CALLBACK (Se ejecuta solo cuando llega info) ---
    def procesar_alerta(self, msg):
        # msg.data contiene el texto que envió el radar
        alerta = msg.data

        if 'IMPACTO' in alerta:
            self.get_logger().error(f'¡PELIGRO REAL! ACTIVANDO ESCUDOS AL 100% >>> {alerta}')
            # Aquí iría código real: self.motor_escudo.activar()
        else:
            self.get_logger().info(f'Monitoreando: {alerta} (Escudos en espera)')

def main(args=None):
    rclpy.init(args=args)
    nodo = EscudosNode()
    rclpy.spin(nodo) # Se queda dormido esperando mensajes
    rclpy.shutdown()"
        :copyable="true"
      />
    </div>

    <!-- 3. LA INTEGRACIÓN -->
    <div class="section-group">
      <SectionTitle>3. Prueba de Integración</SectionTitle>
      <TextBlock>
        Para que esto funcione, necesitamos que ambos nodos corran a la vez. (Recuerda registrar el
        nuevo nodo en <code>setup.py</code> y hacer <code>colcon build</code>).
      </TextBlock>

      <div class="integration-demo q-my-md">
        <div class="row q-col-gutter-md">
          <div class="col-12 col-md-6">
            <div class="terminal-mock sender">
              <div class="header">Terminal 1 (Radar)</div>
              <div class="output">
                [INFO] Enviando: Objeto a 5000km<br />
                [INFO] Enviando: Objeto a 4000km<br />
                [INFO] Enviando: ¡IMPACTO INMINENTE!
              </div>
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="terminal-mock receiver">
              <div class="header">Terminal 2 (Escudos)</div>
              <div class="output">
                [INFO] Monitoreando: Objeto a 5000km<br />
                [INFO] Monitoreando: Objeto a 4000km<br />
                <span class="text-red-13 text-weight-bold"
                  >[ERROR] ¡PELIGRO REAL! ACTIVANDO ESCUDOS >>></span
                >
              </div>
            </div>
          </div>
        </div>

        <!-- Flecha de conexión animada -->
        <div class="data-flow-arrow">
          <div class="particle"></div>
          Datos viajando por /alerta_meteoros
        </div>
      </div>
    </div>

    <!-- 4. ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>4. ¿Por qué no escucho nada?</SectionTitle>
      <TextBlock>
        A veces lanzas el Subscriber y se queda en silencio eterno. Aquí las causas probables:
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-4">
          <div class="error-card">
            <div class="icon">📛</div>
            <div class="title">Nombre del Tópico</div>
            <p>
              Si el Radar publica en <code>/alerta</code> y tú escuchas en
              <code>/alertas</code> (con 's'), nunca llegará nada. Usa
              <code>ros2 topic list</code> para verificar.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="error-card">
            <div class="icon">🧩</div>
            <div class="title">Tipo de Mensaje</div>
            <p>
              Si el Radar envía <code>String</code> y tú esperas <code>Int32</code>, ROS bloqueará
              la conexión por incompatibilidad.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="error-card">
            <div class="icon">📡</div>
            <div class="title">QoS Incompatible</div>
            <p>
              Si el Publisher es "Best Effort" (UDP) y tú eres "Reliable" (TCP), a veces no se
              conectan. (Lo veremos a fondo luego).
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Lógica</SectionTitle>
      <TextBlock>
        Modifica el <strong>Subscriber</strong> para que sea más inteligente:
        <br />
        1. Convierte el mensaje de texto a número (tendrás que modificar el Publisher para enviar
        solo el número, o usar <code>split</code> en el texto).
        <br />
        2. Si la distancia es &lt; 1000, imprime advertencia amarilla.
        <br />
        3. Si la distancia es &lt; 100, imprime error rojo (Choque).
        <br /><br />
        <em>Pista: Lo más fácil es cambiar el tipo de mensaje de String a Int32 en ambos nodos.</em>
      </TextBlock>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
// import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

/* Visualización Callback */
.callback-visual {
  background: var(--bg-surface-solid);
  border-radius: 12px;
  position: relative;
}
.envelope {
  background: #fbbf24;
  color: black;
  font-weight: bold;
  padding: 8px 16px;
  border-radius: 4px;
  display: inline-block;
  animation: float 3s ease-in-out infinite;
}
.arrow-down {
  color: var(--text-muted);
  margin: 10px 0;
  font-size: 0.9rem;
}
.function-box {
  background: var(--bg-surface-hover);
  border: 1px solid #475569;
  border-radius: 8px;
  text-align: left;
  overflow: hidden;
}
.func-name {
  background: var(--bg-surface-solid);
  color: #a5f3fc;
  padding: 5px 10px;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
}
.func-body {
  padding: 10px;
  color: var(--text-secondary);
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
}

@keyframes float {
  0%,
  100% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(-5px);
  }
}

/* Demo Integración */
.integration-demo {
  position: relative;
  background: var(--bg-surface-hover);
  padding: 20px;
  border-radius: 12px;
}
.terminal-mock {
  background: black;
  border-radius: 6px;
  padding: 10px;
  font-family: 'Fira Code', monospace;
  height: 120px;
  border: 1px solid #333;
}
.terminal-mock .header {
  color: #888;
  border-bottom: 1px solid #333;
  margin-bottom: 5px;
  font-size: 0.75rem;
}
.terminal-mock .output {
  color: #ccc;
  font-size: 0.8rem;
  line-height: 1.4;
}
.terminal-mock.sender {
  border-top: 3px solid #38bdf8;
}
.terminal-mock.receiver {
  border-top: 3px solid #ef4444;
}

.data-flow-arrow {
  text-align: center;
  color: var(--text-warning, #d97706);
  font-weight: bold;
  margin-top: 10px;
  font-size: 0.9rem;
  position: relative;
}

/* Tarjetas de Error */
.error-card {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1rem;
  height: 100%;
  text-align: center;
  transition: transform 0.2s;
}
.error-card:hover {
  transform: scale(1.05);
}
.error-card .icon {
  font-size: 2rem;
  margin-bottom: 5px;
}
.error-card .title {
  font-weight: bold;
  color: var(--text-primary);
  margin-bottom: 5px;
}
.error-card p {
  font-size: 0.85rem;
  color: var(--text-muted);
}
</style>
