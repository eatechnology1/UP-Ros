<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-amber-4 text-weight-bold q-mb-sm">
          MÓDULO 7.4: AUTOMATIZACIÓN DE MISIONES
        </div>
        <h1 class="hero-title">Python Commander: <span class="text-white">El Jefe</span></h1>
        <TextBlock>
          Hacer clic en RViz sirve para pruebas, pero un robot de almacén no tiene a un humano
          clicando todo el día. Necesitamos lógica:
          <em
            >"Ve al punto A, espera 5 segundos, ve al punto B, y si hay un obstáculo, vuelve a
            casa"</em
          >. <br /><br />
          Usaremos la API <strong>Nav2 Simple Commander</strong>, una librería de Python que
          convierte tareas complejas de ROS 2 (Action Clients) en comandos simples de una sola
          línea.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. La Librería Mágica</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Bajo el capó, Nav2 usa <strong>Actions</strong> (comunicaciones asíncronas complejas).
            Escribir un cliente de acción desde cero toma 100 líneas de código. <br /><br />
            La librería `nav2_simple_commander` envuelve todo eso.
            <br />
            <strong>Tu código se reduce a:</strong>
          </TextBlock>
          <CodeBlock
            lang="python"
            code="navigator = BasicNavigator()
navigator.goToPose(goal_pose)"
            :copyable="false"
          />
        </template>
        <template #right>
          <div
            class="tool-card bg-slate-900 flex flex-center h-full relative-position overflow-hidden"
          >
            <div class="absolute-center opacity-20">
              <q-icon name="settings" size="10rem" color="grey" class="spin-slow" />
            </div>

            <div
              class="python-wrapper bg-amber-5 text-black q-pa-lg rounded-borders shadow-amber z-top text-center cursor-pointer hover-scale"
            >
              <q-icon name="code" size="3rem" />
              <div class="text-h6 font-mono q-mt-sm">BasicNavigator()</div>
              <div class="text-caption">La "Caja Mágica"</div>
            </div>

            <div class="lines absolute-bottom full-width row justify-around q-pb-md">
              <div class="text-xxs text-grey-5">Action Client</div>
              <div class="text-xxs text-grey-5">Feedback</div>
              <div class="text-xxs text-grey-5">Result</div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Paso 1: Establecer la Posición Inicial</SectionTitle>
      <TextBlock>
        Antes de moverse, el robot necesita saber dónde está (similar al botón "2D Pose Estimate" de
        RViz, pero por código).
      </TextBlock>

      <CodeBlock
        title="1_setup.py"
        lang="python"
        code="from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy

rclpy.init()
nav = BasicNavigator()

# Definir dónde estamos (X=0, Y=0)
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = nav.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.orientation.w = 1.0 # Mirando al frente

nav.setInitialPose(initial_pose)
nav.waitUntilNav2Active() # Esperar a que el sistema arranque"
      />
      <AlertBlock type="info" title="Quaternion w=1.0">
        Recordatorio: En cuaterniones, <code>w=1.0</code> significa rotación cero (0 grados).
      </AlertBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Misión Simple: Ir a la Cocina</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            La orden básica. Le das una coordenada (x, y) y una orientación (w). El método
            <code>goToPose()</code> es <strong>no bloqueante</strong>. El robot empieza a moverse, y
            tu script sigue ejecutándose inmediatamente.
          </TextBlock>
        </template>
        <template #right>
          <div class="tool-card bg-black relative-position overflow-hidden h-full">
            <svg width="100%" height="100%" class="absolute">
              <line
                x1="20%"
                y1="80%"
                x2="80%"
                y2="20%"
                stroke="#facc15"
                stroke-width="2"
                stroke-dasharray="5,5"
              />
              <circle cx="20%" cy="80%" r="5" fill="#4ade80" />
              <circle cx="80%" cy="20%" r="5" fill="#f87171" />
            </svg>
            <div class="robot-moving bg-amber-5 shadow-amber"></div>
          </div>
        </template>
      </SplitBlock>

      <div class="q-mt-md">
        <CodeBlock
          lang="python"
          code="goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 2.5
goal_pose.pose.position.y = 1.0
goal_pose.pose.orientation.w = 1.0

nav.goToPose(goal_pose) # ¡El robot se empieza a mover!"
        />
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. El Bucle de Espera (Feedback)</SectionTitle>
      <TextBlock>
        Como <code>goToPose</code> no bloquea, si terminas el script ahí, el programa Python se
        cierra y el robot se detiene. Necesitas un bucle <code>while</code> para monitorear el
        progreso mientras el robot trabaja.
      </TextBlock>

      <div class="row q-col-gutter-lg">
        <div class="col-12 col-md-7">
          <CodeBlock
            lang="python"
            code="while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    if feedback:
        print(f'Distancia restante: {feedback.distance_remaining:.2f} m')

        # Lógica de seguridad
        if feedback.navigation_time > 600.0:
            nav.cancelTask() # Cancelar si tarda mucho

    # No quemar la CPU
    time.sleep(1.0)

result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print('¡Llegamos a la cocina!')"
          />
        </div>

        <div class="col-12 col-md-5">
          <div class="tool-card bg-slate-800 q-pa-md border-amber">
            <div class="text-caption text-grey-5 q-mb-sm">TERMINAL OUTPUT</div>
            <div
              class="console-log font-mono text-xs text-green-4 bg-black q-pa-sm rounded-borders"
              style="height: 150px; overflow: hidden"
            >
              <div>[INFO] Distancia restante: 3.45 m</div>
              <div>[INFO] Distancia restante: 2.80 m</div>
              <div>[INFO] Distancia restante: 1.95 m</div>
              <div>[INFO] Distancia restante: 1.10 m</div>
              <div class="text-white">...</div>
              <div class="typing-cursor">_</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. Misión Avanzada: Patrullaje de Seguridad</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En lugar de ir a un solo punto, podemos darle una lista de puntos. Nav2 calculará la
            ruta óptima para visitar A, luego B, luego C.

            <br /><br />
            El método es <code>followWaypoints(poses_list)</code>.
          </TextBlock>
        </template>
        <template #right>
          <div
            class="tool-card bg-slate-900 relative-position overflow-hidden h-full flex flex-center"
          >
            <div class="patrol-path relative-position">
              <div class="wp wp-1">A</div>
              <div class="wp wp-2">B</div>
              <div class="wp wp-3">C</div>
              <div class="robot-patrol bg-amber-5"></div>
            </div>
          </div>
        </template>
      </SplitBlock>

      <div class="q-mt-md">
        <CodeBlock
          lang="python"
          code="# Lista de puntos
wp1 = create_pose(2.0, 0.0)
wp2 = create_pose(2.0, 2.0)
wp3 = create_pose(0.0, 2.0)

waypoints = [wp1, wp2, wp3]
nav.followWaypoints(waypoints)"
        />
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>6. Pro Tip: Adiós a los Cuaterniones</SectionTitle>
      <AlertBlock type="success" title="Helper Function">
        Nadie piensa en cuaterniones ("Gira 0.707 en Z"). Pensamos en grados ("Gira 90 grados").
        Copia esta función en tus scripts para convertir <strong>Ángulos de Euler (Yaw)</strong> a
        Cuaterniones.
      </AlertBlock>

      <CodeBlock
        lang="python"
        code="import math
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(yaw_degrees):
    yaw = math.radians(yaw_degrees)
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

# Uso:
pose.orientation = euler_to_quaternion(90) # Mirar a la izquierda"
      />
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>7. Mi script no funciona</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12">
          <q-expansion-item
            icon="timer_off"
            label="El script se queda en 'Esperando Nav2' eternamente"
            header-class="bg-slate-800 text-white rounded-borders"
            class="q-mb-sm border-light"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4">
              Asegúrate de haber lanzado primero el stack de Nav2 y la simulación.
              <br /><code>ros2 launch nav2_bringup tb3_simulation_launch.py</code> <br />El script
              es solo el cliente, necesita que el servidor esté vivo.
            </div>
          </q-expansion-item>

          <q-expansion-item
            icon="cancel"
            label="Task Canceled / Failed inmediatamente"
            header-class="bg-slate-800 text-white rounded-borders"
            class="q-mb-sm border-light"
          >
            <div class="q-pa-md bg-slate-900 text-grey-4">
              ¿Estás enviando al robot a una coordenada fuera del mapa o dentro de una pared?
              <br />Verifica las coordenadas (X, Y) en RViz usando la herramienta "Publish Point"
              para ver sus valores reales antes de ponerlos en código.
            </div>
          </q-expansion-item>
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* GENERAL */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(251, 191, 36, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
  margin-bottom: 3rem;
}
.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  color: #f8fafc;
}

/* ANIMATIONS */
.spin-slow {
  animation: spin 10s infinite linear;
}
@keyframes spin {
  to {
    transform: rotate(360deg);
  }
}

.hover-scale {
  transition: transform 0.2s;
}
.hover-scale:hover {
  transform: scale(1.05);
}

.robot-moving {
  width: 20px;
  height: 20px;
  border-radius: 50%;
  position: absolute;
  animation: moveLinear 3s infinite ease-in-out;
}
@keyframes moveLinear {
  0% {
    left: 20%;
    top: 80%;
  }
  50% {
    left: 80%;
    top: 20%;
  }
  100% {
    left: 20%;
    top: 80%;
  }
}

/* PATROL ANIMATION */
.patrol-path {
  width: 200px;
  height: 200px;
  position: relative;
}
.wp {
  width: 30px;
  height: 30px;
  background: #334155;
  border-radius: 50%;
  position: absolute;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: bold;
  color: white;
  z-index: 2;
  border: 2px solid #facc15;
}
.wp-1 {
  bottom: 0;
  left: 0;
}
.wp-2 {
  bottom: 0;
  right: 0;
}
.wp-3 {
  top: 0;
  left: 50%;
  transform: translateX(-50%);
}

.robot-patrol {
  width: 15px;
  height: 15px;
  border-radius: 50%;
  position: absolute;
  offset-path: path('M 15 185 L 185 185 L 100 15 Z');
  animation: patrolMove 6s infinite linear;
}
@keyframes patrolMove {
  0% {
    offset-distance: 0%;
  }
  100% {
    offset-distance: 100%;
  }
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.shadow-amber {
  box-shadow: 0 0 20px rgba(251, 191, 36, 0.4);
}
.border-amber {
  border: 1px solid rgba(251, 191, 36, 0.3);
}
.text-xxs {
  font-size: 0.6rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.h-full {
  height: 100%;
}
.z-top {
  z-index: 10;
}
.typing-cursor {
  display: inline-block;
  animation: blink 1s infinite;
}
@keyframes blink {
  50% {
    opacity: 0;
  }
}
</style>
