<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <div class="row items-center q-gutter-md">
        <div class="col-grow">
          <TextBlock>
            Basta de teoría. Estás a cargo de los sensores de largo alcance de la estación espacial
            <strong>"Artemis V"</strong>. <br /><br />
            Tu misión: Escribir un nodo que detecte asteroides y transmita su posición a toda la red
            de la nave. Si fallas, chocamos.
          </TextBlock>
        </div>
        <div class="col-auto">
          <div class="text-h1">☄️</div>
        </div>
      </div>
    </div>

    <!-- 1. DISEÑO DEL SISTEMA -->
    <div class="section-group">
      <SectionTitle>1. El Plano del Radar</SectionTitle>
      <TextBlock>
        Antes de tirar código, visualiza el flujo. Tu nodo será el **Emisor (Publisher)**.
      </TextBlock>

      <div class="scenario-visual q-my-md text-center">
        <div class="node-box publisher pulse-animation">
          <div class="icon">📡</div>
          <div class="label">Radar_Node</div>
          <div class="role">PUBLISHER</div>
        </div>

        <div class="connection-line">
          <div class="topic-name">/alerta_meteoros</div>
          <div class="msg-type">[String]</div>
          <div class="arrow">⬇ DATOS ⬇</div>
        </div>

        <div class="cloud">
          <div class="label">La Red ROS 2</div>
          <div class="sublabel">(Cualquiera puede escuchar)</div>
        </div>
      </div>
    </div>

    <!-- 2. CÓDIGO COMENTADO -->
    <div class="section-group">
      <SectionTitle>2. Construyendo el Código</SectionTitle>
      <TextBlock>
        Abre tu editor. Vamos a crear el archivo <code>radar_asteroides.py</code>. Observa cómo
        usamos la **Herencia** que aprendimos antes. Heredamos de `Node` para tener superpoderes de
        comunicación.
      </TextBlock>

      <CodeBlock
        title="radar_asteroides.py"
        lang="python"
        content="import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Importamos el tipo de mensaje (texto)
import random

class RadarNode(Node): # <--- 1. Heredamos de la clase maestra
    def __init__(self):
        # 2. Bautizamos al nodo (nombre interno en la red)
        super().__init__('radar_node')

        # 3. Creamos el Publisher
        # Sintaxis: self.create_publisher(TipoMensaje, 'nombre_topico', cola_espera)
        self.publisher_ = self.create_publisher(String, 'alerta_meteoros', 10)

        # 4. Configuramos un Timer (para no bloquear el código con while True)
        # Cada 1.0 segundos, ejecutará la función 'escanear_espacio'
        self.timer = self.create_timer(1.0, self.escanear_espacio)

        self.get_logger().info('📡 Radar activo. Escaneando sector...')

    def escanear_espacio(self):
        # 5. Lógica del sensor (Simulada)
        distancia = random.randint(100, 5000)

        msg = String()
        msg.data = f'¡ALERTA! Objeto detectado a {distancia} km'

        # 6. Publicar el mensaje a la red
        self.publisher_.publish(msg)

        # Log para nosotros (el programador)
        self.get_logger().info(f'Enviando: {msg.data}')

def main(args=None):
    rclpy.init(args=args) # Inicia la comunicación ROS
    nodo = RadarNode()    # Crea el objeto
    rclpy.spin(nodo)      # Mantiene el nodo vivo escuchando (Loop infinito seguro)
    rclpy.shutdown()      # Cierra limpio al terminar

if __name__ == '__main__':
    main()"
        :copyable="true"
      />
    </div>

    <!-- 3. ANÁLISIS DE PUNTOS CLAVE -->
    <div class="section-group">
      <SectionTitle>3. Disección del Código</SectionTitle>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-h6 text-accent">create_publisher</div>
            <p>
              Es como abrir un canal de YouTube. Defines <strong>QUÉ</strong> transmites (String) y
              <strong>DÓNDE</strong> (Tópico). El número <code>10</code> es el "buffer": si
              transmites muy rápido, guarda los últimos 10 mensajes por si acaso.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="concept-card">
            <div class="text-h6 text-secondary">create_timer</div>
            <p>
              ¡Olvida el <code>time.sleep()</code>! Eso congela al robot.
              <br />
              En ROS usamos Timers. Es una alarma que suena cada X segundos y dispara una función,
              dejando libre al procesador el resto del tiempo.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-12">
          <div class="concept-card">
            <div class="text-h6 text-primary">rclpy.spin()</div>
            <p>
              Es el corazón que mantiene vivo al zombi. Sin <code>spin</code>, el script llegaría al
              final del archivo y se cerraría en milisegundos. <code>spin</code> dice: "Quédate aquí
              quieto y reacciona si pasa algo (como el Timer)".
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. EJECUCIÓN -->
    <div class="section-group">
      <SectionTitle>4. ¡Encender Sistemas!</SectionTitle>
      <TextBlock>
        Para probar esto, necesitamos compilar y registrar el nodo en <code>setup.py</code> (como
        vimos en la lección pasada).
        <br />
        Asumiendo que ya lo hiciste, es hora de ver la magia.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="terminal-window">
            <div class="terminal-header">Terminal 1: El Radar</div>
            <CodeBlock lang="bash" content="ros2 run my_bot_pkg radar_asteroides" />
            <div class="text-code-output q-mt-sm">
              [INFO]: 📡 Radar activo...<br />
              [INFO]: Enviando: ¡ALERTA! Objeto a 4200 km<br />
              [INFO]: Enviando: ¡ALERTA! Objeto a 120 km
            </div>
          </div>
        </template>
        <template #right>
          <div class="terminal-window">
            <div class="terminal-header">Terminal 2: La Pantalla de la Nave</div>
            <div class="text-caption q-mb-xs">¿Cómo sabemos que los datos viajan de verdad?</div>
            <CodeBlock lang="bash" content="ros2 topic echo /alerta_meteoros" />
            <div class="text-code-output q-mt-sm">
              data: "¡ALERTA! Objeto a 4200 km"<br />
              ---<br />
              data: "¡ALERTA! Objeto a 120 km"
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Misión Extra</SectionTitle>
      <TextBlock>
        El Capitán quiere mejorar el radar.
        <br /><br />
        1. Modifica el código para que publique un mensaje diferente si la distancia es menor a 500
        km:
        <strong>"¡IMPACTO INMINENTE! 🚨"</strong>.
        <br />
        2. Cambia la frecuencia del Timer a 0.5 segundos (modo pánico).
        <br />
        3. Recompila con <code>colcon build</code> (¡o no, si usaste symlink!) y prueba.
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

/* Animación y Estilos Visuales del Radar */
.scenario-visual {
  background: radial-gradient(circle, rgba(30, 41, 59, 0.8) 0%, rgba(15, 23, 42, 1) 100%);
  padding: 30px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.node-box {
  display: inline-block;
  background: var(--bg-surface);
  border: 2px solid #38bdf8;
  padding: 15px 30px;
  border-radius: 12px;
  position: relative;
  z-index: 2;
}
.pulse-animation {
  box-shadow: 0 0 0 0 rgba(56, 189, 248, 0.7);
  animation: radarPulse 2s infinite;
}

@keyframes radarPulse {
  0% {
    box-shadow: 0 0 0 0 rgba(56, 189, 248, 0.7);
  }
  70% {
    box-shadow: 0 0 0 20px rgba(56, 189, 248, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(56, 189, 248, 0);
  }
}

.node-box .icon {
  font-size: 2.5rem;
}
.node-box .label {
  font-weight: bold;
  color: var(--text-primary);
  margin-top: 5px;
}
.node-box .role {
  font-size: 0.7rem;
  color: #38bdf8;
  letter-spacing: 2px;
  font-weight: bold;
}

.connection-line {
  height: 80px;
  width: 2px;
  background: #cbd5e1;
  margin: 0 auto;
  position: relative;
}
.topic-name {
  position: absolute;
  top: 50%;
  left: 10px;
  transform: translateY(-50%);
  background: var(--bg-surface-hover);
  padding: 4px 8px;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  color: var(--text-warning, #d97706);
}
.msg-type {
  position: absolute;
  top: 50%;
  right: 10px;
  transform: translateY(-50%);
  font-size: 0.7rem;
  color: var(--text-muted);
}

.cloud {
  border: 2px dashed #64748b;
  border-radius: 50px;
  padding: 10px 40px;
  display: inline-block;
  background: var(--bg-surface-hover);
}

.concept-card {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}

.terminal-window {
  background: #000;
  border: 1px solid #333;
  padding: 10px;
  border-radius: 6px;
  font-family: 'Fira Code', monospace;
}
.terminal-header {
  color: #33ff00;
  border-bottom: 1px solid #333;
  margin-bottom: 5px;
  font-size: 0.8rem;
}
.text-code-output {
  color: #ccc;
  font-size: 0.85rem;
  line-height: 1.4;
}
</style>
