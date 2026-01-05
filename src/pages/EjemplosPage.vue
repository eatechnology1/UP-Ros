<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: LABORATORIO DE CÓDIGO -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Code Lab</div>
        <h1 class="hero-title">Recetario de <span class="text-primary">Código ROS 2</span></h1>

        <TextBlock>
          La mejor forma de aprender ROS 2 es rompiendo código. Aquí tienes patrones de diseño
          probados, desde el "Hola Mundo" hasta sistemas de lanzamiento complejos. Copia, pega y
          adapta estas recetas a tu robot.
        </TextBlock>
      </div>
    </section>

    <!-- 2. HELLO WORLD: PYTHON VS C++ -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Nivel Cero: Python vs C++</SectionTitle>
      <TextBlock>
        ROS 2 es políglota. Puedes escribir nodos que hablen entre sí aunque estén en lenguajes
        distintos. ¿Cuál elegir?
        <ul>
          <li><strong>Python:</strong> Prototipado rápido, Inteligencia Artificial, Scripting.</li>
          <li><strong>C++:</strong> Drivers de hardware, Tiempo real crítico, Alto rendimiento.</li>
        </ul>
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="code-label python">🐍 Python (rclpy)</div>
          <CodeBlock title="simple_node.py" lang="python" :content="codePythonSimple" />
        </template>
        <template #right>
          <div class="code-label cpp">⚙️ C++ (rclcpp)</div>
          <CodeBlock title="simple_node.cpp" lang="cpp" :content="codeCppSimple" />
        </template>
      </SplitBlock>
    </div>

    <!-- 3. PATRÓN PUB/SUB (LA COLUMNA VERTEBRAL) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Patrón Publicador / Suscriptor</SectionTitle>
      <SplitBlock>
        <template #left>
          <AlertBlock type="info" title="📡 El escenario">
            Vamos a simular un <strong>Sensor de Temperatura</strong> (Publicador) y un
            <strong>Sistema de Alerta</strong> (Suscriptor) que reacciona a los datos.
          </AlertBlock>

          <div class="q-mt-md">
            <h3 class="text-subtitle1 text-primary text-weight-bold">Publisher (El que habla)</h3>
            <p class="text-grey-4">Envía mensajes tipo <code>String</code> cada 0.5 segundos.</p>
            <CodeBlock title="sensor_pub.py" lang="python" :content="codePub" />
          </div>
        </template>

        <template #right>
          <ImageBlock
            src="src/assets/images/Nodes-TopicandService.gif"
            caption="Flujo de datos unidireccional"
            :zoomable="false"
            class="q-mb-md opacity-80"
          />

          <h3 class="text-subtitle1 text-secondary text-weight-bold">
            Subscriber (El que escucha)
          </h3>
          <p class="text-grey-4">Recibe el mensaje y lo imprime en el log.</p>
          <CodeBlock title="alert_sub.py" lang="python" :content="codeSub" />
        </template>
      </SplitBlock>
    </div>

    <!-- 4. LAUNCH FILES (ORQUESTACIÓN) -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Launch Files (El Director de Orquesta)</SectionTitle>
      <TextBlock>
        Nadie arranca 50 nodos abriendo 50 terminales. Usamos archivos
        <strong>Launch</strong> (Python) para encender todo el sistema con un solo comando.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <CodeBlock title="robot_app.launch.py" lang="python" :content="codeLaunch" />
        </template>
        <template #right>
          <AlertBlock type="success" title="🚀 Ventajas del Launch System">
            <ul class="q-pl-md">
              <li><strong>Reusabilidad:</strong> Incluye otros launch files dentro del tuyo.</li>
              <li><strong>Configuración:</strong> Cambia parámetros al vuelo.</li>
              <li><strong>Eventos:</strong> "Si el nodo de cámara muere, reinicia el sistema".</li>
            </ul>
          </AlertBlock>
          <div class="q-mt-lg">
            <CodeBlock
              title="Terminal"
              lang="bash"
              content="ros2 launch mi_paquete robot_app.launch.py"
              :copyable="true"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. CHEAT SHEET DE COMANDOS -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>4. La Navaja Suiza (CLI Cheat Sheet)</SectionTitle>
      <div class="terminal-grid">
        <div class="terminal-card">
          <div class="term-header">🔍 Instrospección</div>
          <div class="term-cmd">ros2 node list</div>
          <div class="term-desc">Ver quién está vivo.</div>
          <div class="term-cmd">ros2 topic list -t</div>
          <div class="term-desc">Ver canales y tipos de mensaje.</div>
          <div class="term-cmd">ros2 topic echo /datos</div>
          <div class="term-desc">Espiar datos en tiempo real.</div>
        </div>

        <div class="terminal-card">
          <div class="term-header">🛠️ Desarrollo</div>
          <div class="term-cmd">colcon build --symlink-install</div>
          <div class="term-desc">Compilar creando enlaces simbólicos (vital para Python).</div>
          <div class="term-cmd">source install/setup.bash</div>
          <div class="term-desc">Cargar el entorno compilado.</div>
        </div>

        <div class="terminal-card">
          <div class="term-header">📦 Paquetes</div>
          <div class="term-cmd">ros2 pkg create --build-type ament_python mi_paquete</div>
          <div class="term-desc">Crear esqueleto Python.</div>
          <div class="term-cmd">ros2 pkg list</div>
          <div class="term-desc">Ver librerías instaladas.</div>
        </div>
      </div>
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

// 1. HELLO WORLD
const codePythonSimple = `
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_py')
        self.get_logger().info('¡Hola desde Python!')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HelloNode())
    rclpy.shutdown()
`.trim();

const codeCppSimple = `
#include "rclcpp/rclcpp.hpp"

class HelloNode : public rclcpp::Node {
public:
  HelloNode() : Node("hello_cpp") {
    RCLCPP_INFO(this->get_logger(), "¡Hola desde C++!");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloNode>());
  rclcpp::shutdown();
  return 0;
}
`.trim();

// 2. PUB/SUB
const codePub = `
# ... imports ...
class SensorPub(Node):
    def __init__(self):
        super().__init__('sensor_pub')
        # Crea publicador: Tópico "temp", Tipo String, Cola 10
        self.pub = self.create_publisher(String, 'temp', 10)
        self.timer = self.create_timer(0.5, self.send_data)

    def send_data(self):
        msg = String()
        msg.data = "Temperatura: 24°C"
        self.pub.publish(msg)
`.trim();

const codeSub = `
# ... imports ...
class AlertSub(Node):
    def __init__(self):
        super().__init__('alert_sub')
        # Suscripción al mismo tópico 'temp'
        self.sub = self.create_subscription(
            String, 'temp', self.listener_callback, 10)

    def listener_callback(self, msg):
        # Esta función se ejecuta CADA VEZ que llega un dato
        self.get_logger().warn(f'Alerta recibida: {msg.data}')
`.trim();

// 3. LAUNCH
const codeLaunch = `
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mi_sensor_pkg',
            executable='sensor_node',
            name='sensor_frontal',
            parameters=[{'velocidad': 5.0}]
        ),
        Node(
            package='mi_cerebro_pkg',
            executable='procesador_node',
            output='screen'
        )
    ])
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
    radial-gradient(circle at bottom right, rgba(16, 185, 129, 0.1), transparent 60%),
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

/* ETIQUETAS DE LENGUAJE */
.code-label {
  display: inline-block;
  padding: 4px 12px;
  border-radius: 6px 6px 0 0;
  font-size: 0.85rem;
  font-weight: 700;
  margin-bottom: -1px; /* Pegado al bloque */
}

.code-label.python {
  background: rgba(255, 215, 0, 0.15);
  color: #ffd700;
}

.code-label.cpp {
  background: rgba(56, 189, 248, 0.15);
  color: #38bdf8;
}

/* TERMINAL GRID (Cheat Sheet) */
.terminal-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
}

.terminal-card {
  background: #1e1e1e; /* Fondo terminal clásico */
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 12px;
  padding: 20px;
  font-family: 'Fira Code', monospace;
}

.term-header {
  color: #fff;
  font-weight: 700;
  margin-bottom: 16px;
  padding-bottom: 8px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.term-cmd {
  color: #4ade80; /* Green terminal */
  font-weight: 600;
  margin-top: 12px;
}

.term-cmd::before {
  content: '$ ';
  color: #6b7280;
}

.term-desc {
  color: #9ca3af;
  font-size: 0.85rem;
  margin-top: 4px;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
