<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: EL MATRIX DE LOS ROBOTS -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Entornos Virtuales</div>
        <h1 class="hero-title">Simulación y <span class="text-primary">Gemelos Digitales</span></h1>

        <TextBlock>
          Antes de arriesgar un robot de $5,000 USD, probamos el código en un mundo virtual donde la
          gravedad, la fricción y las colisiones son matemáticamente simuladas. Aquí dominarás
          <strong>Gazebo</strong> (para la física) y <strong>RViz2</strong>
          (para la visualización de datos).
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA GRAN DUDA: RVIZ VS GAZEBO -->
    <div class="section-group self-stretch">
      <SectionTitle>1. RViz2 vs Gazebo: ¿Cuál uso?</SectionTitle>
      <TextBlock>
        Es el error conceptual #1. Muchos creen que son lo mismo, pero son opuestos complementarios.
      </TextBlock>

      <div class="row q-col-gutter-xl q-mt-sm">
        <!-- RViz -->
        <div class="col-12 col-md-6">
          <div class="tool-card rviz">
            <div class="tool-header">
              <q-icon name="visibility" size="md" />
              <h2>RViz 2</h2>
              <span class="tool-badge">Los Ojos</span>
            </div>
            <p>
              <strong>Visualiza lo que el robot "cree" que ve.</strong><br />
              Muestra datos de sensores (LIDAR, Cámara) y estados internos. Si el robot está
              alucinando y cree que está en Marte, RViz te mostrará Marte.
            </p>
            <ul class="tool-list">
              <li>✅ Ver nubes de puntos (Lidar).</li>
              <li>✅ Ver el mapa construido (SLAM).</li>
              <li>✅ Debuggear TFs rotas.</li>
              <li>❌ NO tiene física (las cosas no caen).</li>
            </ul>
          </div>
        </div>

        <!-- Gazebo -->
        <div class="col-12 col-md-6">
          <div class="tool-card gazebo">
            <div class="tool-header">
              <q-icon name="public" size="md" />
              <h2>Gazebo (Sim)</h2>
              <span class="tool-badge">La Realidad</span>
            </div>
            <p>
              <strong>Simula la física del mundo.</strong><br />
              Calcula gravedad, inercia, fricción y luz. Genera los datos "falsos" que alimentan a
              tus sensores para engañar a tu robot.
            </p>
            <ul class="tool-list">
              <li>✅ Motores y articulaciones.</li>
              <li>✅ Sensores virtuales (Cámaras, IMU).</li>
              <li>✅ Colisiones con obstáculos.</li>
              <li>❌ Consume mucha CPU/GPU.</li>
            </ul>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. FLUJO DE TRABAJO EN SIMULACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Flujo de Trabajo Profesional</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Para simular un robot correctamente en ROS 2 Jazzy, necesitas orquestar varios paquetes.
            El flujo estándar es:
          </TextBlock>
          <StepsBlock :steps="simSteps" />
          <AlertBlock type="warning" title="⚠️ Nota sobre Gazebo Classic">
            En ROS 2 Jazzy, usamos <strong>Gazebo Harmonic</strong> (la versión moderna). El viejo
            "Gazebo Classic" (v11) está obsoleto. Asegúrate de instalar
            <code>ros-jazzy-ros-gz</code>.
          </AlertBlock>
        </template>
        <template #right>
          <div class="bg-dark q-pa-md rounded-borders">
            <div class="text-subtitle2 text-grey-5 q-mb-sm">
              Estructura típica de un paquete de simulación:
            </div>
            <CodeBlock title="File Tree" lang="bash" :content="simFileTree" :copyable="false" />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. SPAWNEANDO EL ROBOT (URDF + XACRO) -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Spawneando el Robot (URDF)</SectionTitle>
      <TextBlock>
        Para que Gazebo entienda tu robot, debes añadirle propiedades físicas (inercia, colisión) a
        tu modelo visual. Usamos etiquetas <code>&lt;gazebo&gt;</code> dentro del URDF.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <CodeBlock title="robot.urdf.xacro" lang="xml" :content="urdfGazeboCode" />
        </template>
        <template #right>
          <AlertBlock type="info" title="🧩 Plugins de Gazebo">
            Para que el robot se mueva, necesitamos un "Driver Virtual". El plugin
            <strong>DiffDrive</strong> es el más común para robots de ruedas. <br /><br />
            Este plugin lee <code>/cmd_vel</code> y hace girar las ruedas virtuales mágicamente.
          </AlertBlock>
          <div class="q-mt-md">
            <CodeBlock
              title="Terminal: Spawnear"
              lang="bash"
              content="ros2 launch ros_gz_sim_demos diff_drive.launch.py"
              :copyable="true"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. VISUALIZACIÓN AVANZADA EN RVIZ -->
    <div class="section-group self-stretch">
      <SectionTitle>4. Configurando RViz2</SectionTitle>
      <TextBlock>
        RViz2 no muestra nada al abrirse ("pantalla negra"). Debes añadir <strong>Displays</strong>
        manualmente. Aquí están los esenciales:
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-4">
          <div class="config-card">
            <q-icon name="gps_fixed" color="red-4" size="md" />
            <h4>Fixed Frame</h4>
            <p>
              La referencia del mundo. Normalmente se configura en <code>map</code> o
              <code>odom</code>. Si esto está mal, nada se ve.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="config-card">
            <q-icon name="android" color="green-4" size="md" />
            <h4>RobotModel</h4>
            <p>
              Lee el <code>/robot_description</code> y muestra el modelo 3D de tu robot. Si sale
              blanco, falta el TF.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="config-card">
            <q-icon name="radar" color="purple-4" size="md" />
            <h4>LaserScan / PointCloud</h4>
            <p>
              Muestra los puntos rojos de los obstáculos. Debes seleccionar el tópico correcto (ej:
              <code>/scan</code>).
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 6. TROUBLESHOOTING DE SIMULACIÓN -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>5. Zona de Desastre (Troubleshooting)</SectionTitle>
      <TextBlock> ¿Tu simulación explotó? Aquí están los errores clásicos. </TextBlock>

      <div class="trouble-grid">
        <div class="trouble-item">
          <div class="trouble-q">🤖 El robot atraviesa el suelo</div>
          <div class="trouble-a">
            Falta la caja de colisión (<code>&lt;collision&gt;</code>) en las ruedas o el suelo no
            tiene cuerpo estático.
          </div>
        </div>
        <div class="trouble-item">
          <div class="trouble-q">📡 RViz dice "No Transform"</div>
          <div class="trouble-a">
            El <code>robot_state_publisher</code> no está corriendo o el árbol TF está roto. Revisa
            <code>ros2 run tf2_tools view_frames</code>.
          </div>
        </div>
        <div class="trouble-item">
          <div class="trouble-q">🌑 El modelo se ve negro/sin texturas</div>
          <div class="trouble-a">
            Problema de iluminación en Gazebo o rutas de los Meshes (.dae/.stl) incorrectas en el
            <code>package.xml</code>.
          </div>
        </div>
        <div class="trouble-item">
          <div class="trouble-q">🐢 Va lentísimo (Real Time Factor &lt; 0.5)</div>
          <div class="trouble-a">
            Tu PC no aguanta. Reduce la calidad de las sombras o el "Physics Update Rate" en el
            archivo del mundo.
          </div>
        </div>
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
import StepsBlock from 'components/content/StepsBlock.vue';

const simSteps = [
  'Diseñar el robot en URDF/Xacro (Geometría visual y de colisión).',
  'Crear un "Mundo" en SDF (Paredes, luces, suelo).',
  'Lanzar Gazebo + Spawner (Poner el robot en el mundo).',
  'Lanzar robot_state_publisher (Publicar TFs).',
  'Abrir RViz2 para ver qué está pasando.',
];

const simFileTree = `
my_bot_sim/
├── launch/
│   ├── sim.launch.py       # Arranca todo
├── urdf/
│   ├── robot.xacro         # Modelo físico
│   ├── gazebo_plugins.xacro
├── worlds/
│   ├── mars_station.sdf    # Entorno 3D
├── models/                 # Objetos extra
├── config/
│   ├── rviz_config.rviz    # Config de vistas
`.trim();

const urdfGazeboCode = `
<!-- Propiedades de Gazebo para una Rueda -->
<gazebo reference="left_wheel_link">
    <!-- Color en Gazebo (Diferente a RViz) -->
    <material>Gazebo/Black</material>
    <!-- Fricción (mu1, mu2) -->
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
</gazebo>

<!-- Plugin de Control Diferencial -->
<gazebo>
    <plugin filename="libignition-gazebo-diff-drive-system.so"
            name="ignition::gazebo::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <topic>/cmd_vel</topic>
    </plugin>
</gazebo>
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
    radial-gradient(circle at top left, rgba(236, 72, 153, 0.15), transparent 60%),
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

/* CARDS RVIZ vs GAZEBO */
.tool-card {
  height: 100%;
  padding: 24px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.05);
  transition: transform 0.3s ease;
}

.tool-card:hover {
  transform: translateY(-5px);
}

.tool-card.rviz {
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.6), rgba(15, 23, 42, 0.8));
  border-top: 4px solid #4ade80; /* Green */
}

.tool-card.gazebo {
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.6), rgba(15, 23, 42, 0.8));
  border-top: 4px solid #f97316; /* Orange */
}

.tool-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 16px;
}

.tool-header h2 {
  margin: 0;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
}

.tool-badge {
  margin-left: auto;
  font-size: 0.75rem;
  text-transform: uppercase;
  letter-spacing: 1px;
  background: rgba(255, 255, 255, 0.1);
  padding: 4px 8px;
  border-radius: 4px;
}

.tool-list {
  list-style: none;
  padding: 0;
  margin-top: 16px;
  color: #cbd5e1;
}

.tool-list li {
  margin-bottom: 8px;
  display: flex;
  align-items: center;
  gap: 8px;
}

/* CONFIG CARDS (RVIZ) */
.config-card {
  background: rgba(30, 41, 59, 0.4);
  padding: 20px;
  border-radius: 12px;
  text-align: center;
  border: 1px solid rgba(148, 163, 184, 0.1);
}

.config-card h4 {
  margin: 12px 0 8px 0;
  color: #e2e8f0;
  font-size: 1.1rem;
  font-weight: 600;
}

.config-card p {
  color: #94a3b8;
  font-size: 0.9rem;
  line-height: 1.4;
  margin: 0;
}

/* TROUBLESHOOTING GRID */
.trouble-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
}

.trouble-item {
  background: rgba(239, 68, 68, 0.05); /* Red tint */
  border: 1px solid rgba(239, 68, 68, 0.2);
  border-radius: 12px;
  padding: 16px;
}

.trouble-q {
  color: #fca5a5; /* Red-300 */
  font-weight: 700;
  margin-bottom: 8px;
  font-size: 1rem;
}

.trouble-a {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.4;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
