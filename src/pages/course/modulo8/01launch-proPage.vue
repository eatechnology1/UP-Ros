<template>
  <LessonContainer>
    <div class="section-group self-stretch">
      <SectionTitle>1. El Problema de las 10 Terminales</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Imagina tener que iniciar estos procesos en orden cada vez que enciendes el robot:
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list text-caption">
            <li>1. Driver de Motores (esperar 2s)</li>
            <li>2. Driver Lidar</li>
            <li>3. Robot State Publisher (URDF)</li>
            <li>4. Nav2 Stack (esperar mapa)</li>
            <li>5. Rviz para visualizar</li>
          </ul>
          <TextBlock>
            Si el proceso #1 falla, los demás deben saberlo. Hacer esto a mano es propenso a errores
            humanos.
          </TextBlock>
        </template>
        <template #right>
          <div
            class="tool-card bg-slate-900 flex flex-center h-full relative-position overflow-hidden"
          >
            <div class="conductor column items-center z-top">
              <q-icon name="person_4" size="3rem" color="white" />
              <div class="baton"></div>
              <div class="text-caption text-blue-grey-2">bringup.launch.py</div>
            </div>

            <div class="musician m1 bg-red-9">Lidar</div>
            <div class="musician m2 bg-green-9">Motor</div>
            <div class="musician m3 bg-blue-9">Nav2</div>

            <div class="wave w1"></div>
            <div class="wave w2"></div>
            <div class="wave w3"></div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Anatomía Básica (Python)</SectionTitle>
      <TextBlock>
        En ROS 2, los launch files son scripts de Python. Esto significa que puedes usar lógica de
        programación (if/else, loops, encontrar rutas de archivos).
      </TextBlock>

      <CodeBlock
        title="simple_launch.py"
        lang="python"
        code="from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo 1: Un talker simple
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='mi_talker',
            parameters=[{'msg': 'Hola Ingeniero'}]
        ),
        # Nodo 2: Un listener
        Node(
            package='demo_nodes_py',
            executable='listener'
        )
    ])"
      />
      <AlertBlock type="info" title="Entry Point">
        La función <code>generate_launch_description()</code> es obligatoria. ROS 2 busca esta
        función para saber qué ejecutar.
      </AlertBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Composición: Divide y Vencerás</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            No escribas un archivo de 1000 líneas. Crea archivos pequeños y reutilizables.
            <br /><br />
            El patrón <strong>"Bringup"</strong> consiste en un archivo maestro que llama a otros
            archivos hijos (IncludeLaunchDescription).
          </TextBlock>
        </template>
        <template #right>
          <div class="tool-card bg-slate-900 q-pa-md h-full overflow-hidden flex flex-center">
            <div class="tree-structure">
              <div class="node-launch bg-white text-black text-weight-bold">
                robot_bringup.launch.py
              </div>
              <div class="vertical-line"></div>
              <div class="row justify-center q-gutter-md">
                <div class="branch column items-center">
                  <div class="node-launch bg-blue-grey-8 text-white">sensors.launch.py</div>
                  <div class="vertical-line-sm"></div>
                  <div class="row q-gutter-xs">
                    <div class="leaf bg-red-5">Lidar</div>
                    <div class="leaf bg-red-5">IMU</div>
                  </div>
                </div>
                <div class="branch column items-center">
                  <div class="node-launch bg-blue-grey-8 text-white">navigation.launch.py</div>
                  <div class="vertical-line-sm"></div>
                  <div class="leaf bg-blue-5">Nav2</div>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>

      <div class="q-mt-md">
        <CodeBlock
          lang="python"
          title="Cómo incluir otro launch"
          code="from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

# ... dentro de generate_launch_description
sensors_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('my_robot_sensors'), 'launch', 'sensors.launch.py')
    ])
)"
        />
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. Argumentos: Flexibilidad en Tiempo de Ejecución</SectionTitle>
      <TextBlock>
        No hardcodees valores. Usa <code>DeclareLaunchArgument</code> para permitir cambiar
        configuraciones desde la terminal sin editar el código.
      </TextBlock>

      <div class="tool-card cli-card bg-black q-pa-md border-light q-my-md">
        <div class="text-caption text-grey-5 font-mono q-mb-xs"># Ejecución en terminal</div>
        <div class="text-green-4 font-mono text-sm">
          $ ros2 launch my_robot bringup.launch.py
          <span class="text-yellow-4">sim:=true world:=mars.sdf</span>
        </div>
      </div>

      <CodeBlock
        lang="python"
        title="Uso de argumentos"
        code="from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# 1. Declarar (Lo que acepta la terminal)
sim_arg = DeclareLaunchArgument(
    'sim',
    default_value='false',
    description='Modo simulación'
)

# 2. Usar (Capturar el valor)
use_sim_time = LaunchConfiguration('sim')

# 3. Pasar al nodo
node = Node(
    package='nav2_bringup',
    executable='lifecycle_manager',
    parameters=[{'use_sim_time': use_sim_time}] # ¡Dinámico!
)"
      />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. Event Handlers: Cuando las cosas fallan</SectionTitle>
      <AlertBlock type="warning" title="El Escenario Catastrófico">
        ¿Qué pasa si el nodo del Lidar se cuelga (crachea)?
        <ul>
          <li><strong>Novato:</strong> El robot sigue andando ciego y choca.</li>
          <li>
            <strong>Pro:</strong> El sistema detecta la muerte del proceso y lo reinicia
            automáticamente (Respawn) o apaga todo el sistema por seguridad.
          </li>
        </ul>
      </AlertBlock>

      <div class="row q-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-900 border-red q-pa-md text-center">
            <q-icon name="restart_alt" size="3rem" color="red-4" />
            <div class="text-h6 text-red-1">Respawn (Fénix)</div>
            <p class="text-caption text-grey-4">"Si muero, revíveme después de 2 segundos."</p>
            <div
              class="code-snippet bg-black rounded text-left q-pa-sm text-xxs font-mono text-red-3"
            >
              Node(..., respawn=True, respawn_delay=2.0)
            </div>
          </div>
        </div>
        <div class="col-12 col-md-5">
          <div class="tool-card bg-slate-900 border-orange q-pa-md text-center">
            <q-icon name="power_settings_new" size="3rem" color="orange-4" />
            <div class="text-h6 text-orange-1">OnProcessExit</div>
            <p class="text-caption text-grey-4">"Si el nodo MAPA muere, apaga todo el sistema."</p>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          lang="python"
          title="Advanced Logic"
          code="from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=lidar_node,
        on_exit=[LogInfo(msg='¡Lidar murió! Apagando seguridad...'), shutdown_action]
    )
)"
        />
      </div>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>6. Mandamientos del Launch Engineer</SectionTitle>
      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-4">
          <div class="custom-card border-light q-pa-md h-full">
            <div class="text-weight-bold text-white">1. Parametriza todo</div>
            <p class="text-caption text-grey-4">
              Nunca escribas rutas absolutas like <code>/home/juan/robot</code>. Usa
              <code>FindPackageShare</code>.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="custom-card border-light q-pa-md h-full">
            <div class="text-weight-bold text-white">2. Usa Namespaces</div>
            <p class="text-caption text-grey-4">
              Si tienes 2 robots, lánzalos en namespaces <code>/robot1</code> y
              <code>/robot2</code> para evitar colisiones de topics.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-4">
          <div class="custom-card border-light q-pa-md h-full">
            <div class="text-weight-bold text-white">3. Lifecycle Nodes</div>
            <p class="text-caption text-grey-4">
              Usa nodos gestionados (Active/Inactive) para un arranque determinista (veremos en Nav2
              avanzado).
            </p>
          </div>
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* GENERAL */
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

/* ORCHESTRA ANIMATION */
.conductor {
  position: absolute;
  top: 20%;
  left: 50%;
  transform: translateX(-50%);
}
.baton {
  width: 4px;
  height: 30px;
  background: #facc15;
  margin-top: -10px;
  margin-left: 20px;
  transform-origin: bottom center;
  animation: conduct 1s infinite alternate ease-in-out;
}
@keyframes conduct {
  from {
    transform: rotate(-30deg);
  }
  to {
    transform: rotate(30deg);
  }
}

.musician {
  width: 60px;
  height: 30px;
  border-radius: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.7rem;
  color: white;
  font-weight: bold;
  position: absolute;
}
.m1 {
  bottom: 20%;
  left: 20%;
}
.m2 {
  bottom: 20%;
  left: 50%;
  transform: translateX(-50%);
}
.m3 {
  bottom: 20%;
  right: 20%;
}

.wave {
  position: absolute;
  border: 2px solid rgba(255, 255, 255, 0.2);
  border-radius: 50%;
  opacity: 0;
  top: 30%;
  left: 50%;
  transform: translateX(-50%);
}
.w1 {
  width: 50px;
  height: 50px;
  animation: ripple 2s infinite;
}
.w2 {
  width: 100px;
  height: 100px;
  animation: ripple 2s infinite 0.5s;
}
.w3 {
  width: 150px;
  height: 150px;
  animation: ripple 2s infinite 1s;
}
@keyframes ripple {
  0% {
    opacity: 1;
    transform: translateX(-50%) scale(0.5);
  }
  100% {
    opacity: 0;
    transform: translateX(-50%) scale(1.5);
  }
}

/* TREE STRUCTURE */
.node-launch {
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 0.8rem;
  margin-bottom: 5px;
}
.vertical-line {
  width: 2px;
  height: 20px;
  background: #64748b;
  margin: 0 auto;
}
.vertical-line-sm {
  width: 2px;
  height: 10px;
  background: #64748b;
  margin: 0 auto;
}
.leaf {
  padding: 2px 6px;
  border-radius: 3px;
  font-size: 0.7rem;
  color: black;
  margin-top: 5px;
}
.branch {
  min-width: 100px;
}

/* CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.text-xxs {
  font-size: 0.6rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.tool-list li {
  margin-bottom: 5px;
}
.h-full {
  height: 100%;
}
.z-top {
  z-index: 10;
}
.custom-card {
  background: rgba(30, 41, 59, 0.5);
  border-left: 2px solid rgba(255, 255, 255, 0.2);
  border-radius: 8px;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-red {
  border-color: #f44336;
}
.border-orange {
  border-color: #ff9800;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
