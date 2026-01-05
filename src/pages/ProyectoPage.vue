<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: EL DESAFÍO FINAL -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Capstone Project</div>
        <h1 class="hero-title">Misión: <span class="text-primary">Escape del Laberinto</span></h1>

        <TextBlock>
          Has aprendido la teoría, instalado las herramientas y simulado la física. Ahora toca la
          prueba de fuego: programar un robot móvil para que
          <strong>navegue autónomamente</strong> en un entorno desconocido usando solo sus sensores.
        </TextBlock>
      </div>
    </section>

    <!-- 2. DEFINICIÓN DE LA MISIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Objetivo (Briefing)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El robot empieza en el centro de un laberinto simulado en Gazebo. Tu código debe:
          </TextBlock>
          <div class="mission-list">
            <div class="mission-item">
              <q-icon name="radar" color="purple-4" size="sm" />
              <div>
                <strong>Percibir:</strong> Leer datos del láser (LIDAR) para detectar paredes.
              </div>
            </div>
            <div class="mission-item">
              <q-icon name="psychology" color="orange-4" size="sm" />
              <div><strong>Decidir:</strong> Si hay pared enfrente, girar. Si no, avanzar.</div>
            </div>
            <div class="mission-item">
              <q-icon name="sports_esports" color="green-4" size="sm" />
              <div><strong>Actuar:</strong> Enviar comandos de velocidad a las ruedas.</div>
            </div>
          </div>
        </template>
        <template #right>
          <AlertBlock type="warning" title="⚠️ Requisitos Previos">
            <ul class="q-pl-md q-mt-sm">
              <li>Tener instalado el paquete de simulación (ver módulo Simulación).</li>
              <li>Saber crear un paquete Python (<code>ros2 pkg create</code>).</li>
              <li>Entender tópicos Pub/Sub.</li>
            </ul>
          </AlertBlock>
          <div class="q-mt-md">
            <CodeBlock
              title="Comando para iniciar el mundo"
              lang="bash"
              content="ros2 launch upros_sim maze_world.launch.py"
              :copyable="true"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. PASO 1: CREAR EL PAQUETE -->
    <div class="section-group self-stretch">
      <SectionTitle>Paso 1: Estructura del Proyecto</SectionTitle>
      <TextBlock>
        Primero, creamos un contenedor limpio para nuestro código. No mezcles esto con tus ejemplos
        anteriores.
      </TextBlock>

      <CodeBlock title="Terminal" lang="bash" :content="createPkgCmd" />
    </div>

    <!-- 4. PASO 2: LECTURA DE SENSORES (SUSCRIPTOR) -->
    <div class="section-group self-stretch">
      <SectionTitle>Paso 2: Ojos Digitales (LIDAR)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El láser publica en <code>/scan</code>. El mensaje es de tipo <code>LaserScan</code>
            y contiene un array de distancias (rangos).
            <br /><br />
            Necesitamos filtrar ese array para saber qué hay <strong>enfrente</strong>, a la
            <strong>izquierda</strong> y a la <strong>derecha</strong>.
          </TextBlock>
        </template>
        <template #right>
          <ImageBlock
            src="src/assets/images/Nodes-TopicandService.gif"
            caption="El LIDAR barre 360 grados"
            :zoomable="false"
            class="opacity-80"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 5. PASO 3: EL CEREBRO (MÁQUINA DE ESTADOS) -->
    <div class="section-group self-stretch">
      <SectionTitle>Paso 3: El Algoritmo "Wall Follower"</SectionTitle>
      <TextBlock>
        Usaremos una lógica reactiva simple. El robot intentará mantener la pared a su derecha.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-4">
          <div class="state-card">
            <div class="state-icon">1</div>
            <h4>Buscar Pared</h4>
            <p>Si no veo nada cerca, avanzo y giro levemente a la derecha hasta encontrar algo.</p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="state-card">
            <div class="state-icon">2</div>
            <h4>Seguir Pared</h4>
            <p>Si tengo pared a la derecha, avanzo recto manteniendo la distancia.</p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="state-card">
            <div class="state-icon">3</div>
            <h4>Evitar Choque</h4>
            <p>Si tengo pared <strong>enfrente</strong>, giro a la izquierda inmediatamente.</p>
          </div>
        </div>
      </div>
    </div>

    <!-- 6. CÓDIGO FINAL -->
    <div class="section-group self-stretch">
      <SectionTitle>Paso 4: El Código Completo</SectionTitle>
      <TextBlock>
        Aquí está la implementación en Python. Analiza cómo convertimos la lógica anterior en
        <code>if/else</code> dentro del bucle de control.
      </TextBlock>

      <CodeBlock title="maze_solver.py" lang="python" :content="finalCode" :expandable="true" />
    </div>

    <!-- 7. RETO EXTRA -->
    <div class="section-group self-stretch q-mb-xl">
      <AlertBlock type="success" title="🏆 Reto para el Estudiante">
        Este código es básico y el robot oscila mucho (camina como borracho).<br />
        <strong>Tu misión:</strong> Implementa un controlador <strong>PID</strong>
        para que la distancia a la pared sea suave y constante.
      </AlertBlock>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import ImageBlock from 'components/content/ImageBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

const createPkgCmd = `
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python maze_escape --dependencies rclpy geometry_msgs sensor_msgs
cd ..
colcon build --packages-select maze_escape
source install/setup.bash
`.trim();

const finalCode = `
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')

        # Suscriptor al Láser
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publicador de Velocidad
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables de estado
        self.regions = {
            'front': float('inf'),
            'left':  float('inf'),
            'right': float('inf'),
        }

    def scan_callback(self, msg):
        # El laser devuelve 360 puntos. Tomamos promedios por sector.
        # Nota: Esto depende del modelo del lidar (indices pueden variar)
        self.regions = {
            'right':  min(min(msg.ranges[0:60]), 10),
            'front':  min(min(msg.ranges[60:120]), 10),
            'left':   min(min(msg.ranges[120:180]), 10),
        }
        self.decide_movement()

    def decide_movement(self):
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0

        d = 0.5  # Distancia de seguridad a la pared

        # LÓGICA DE CONTROL (Máquina de Estados Simple)

        if self.regions['front'] < d:
            # CASO 1: Pared enfrente -> Girar Izquierda (Emergencia)
            self.get_logger().info('¡Pared Enfrente! Girando...')
            linear_x = 0.0
            angular_z = 0.5

        elif self.regions['right'] > d:
            # CASO 2: Perdí la pared derecha -> Girar Derecha para buscarla
            self.get_logger().info('Buscando pared...')
            linear_x = 0.2
            angular_z = -0.3

        else:
            # CASO 3: Tengo pared a la derecha -> Seguir recto
            self.get_logger().info('Siguiendo pared...')
            linear_x = 0.3
            angular_z = 0.0

        # Enviar comando
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
`.trim();
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
    radial-gradient(circle at bottom left, rgba(245, 158, 11, 0.15), transparent 60%),
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

/* LISTA DE MISIONES */
.mission-list {
  display: flex;
  flex-direction: column;
  gap: 16px;
  margin-top: 24px;
}

.mission-item {
  display: flex;
  align-items: center;
  gap: 16px;
  background: rgba(30, 41, 59, 0.4);
  padding: 16px;
  border-radius: 12px;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.mission-item div {
  color: #cbd5e1;
  font-size: 0.95rem;
}

/* TARJETAS DE ESTADO (ALGORITMO) */
.state-card {
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.6), rgba(15, 23, 42, 0.8));
  padding: 24px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.05);
  height: 100%;
  position: relative;
  overflow: hidden;
}

.state-icon {
  width: 40px;
  height: 40px;
  background: rgba(56, 189, 248, 0.2);
  color: #38bdf8;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 900;
  font-size: 1.2rem;
  margin-bottom: 16px;
}

.state-card h4 {
  margin: 0 0 8px 0;
  color: #f1f5f9;
  font-size: 1.1rem;
  font-weight: 700;
}

.state-card p {
  margin: 0;
  color: #94a3b8;
  font-size: 0.9rem;
  line-height: 1.4;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
