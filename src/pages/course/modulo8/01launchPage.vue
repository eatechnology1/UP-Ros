<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-teal-4 text-weight-bold q-mb-sm">
          MÓDULO 5.1: EL DIRECTOR DE ORQUESTA
        </div>

        <h1 class="hero-title">Launch <span class="text-teal-5">System</span></h1>

        <TextBlock>
          Imagina que tu robot necesita 7 nodos corriendo simultáneamente: cámara, motores, IA,
          simulación, logs, GUI y teleop. Sin Launch, abrirías 7 terminales y escribirías 7 comandos
          diferentes. Con <strong>Launch Files</strong>, defines una "Orquesta" que arranca todo con
          un solo comando: <code>ros2 launch mi_robot robot.launch.py</code>.
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA DEL LAUNCH FILE -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Partitura</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Los archivos Launch son archivos Python (`.py`) o XML (`.xml`) que definen qué nodos
            arrancar.
            <br /><br />
            <strong>La Estructura Básica:</strong>
            <ol class="q-pl-md q-mt-sm text-grey-4">
              <li><strong>LaunchDescription:</strong> El contenedor de toda la orquesta.</li>
              <li><strong>Node:</strong> Cada instrumento de la orquesta.</li>
              <li>
                <strong>Parámetros:</strong> Configuraciones que se pueden cambiar sin recompilar.
              </li>
            </ol>
          </TextBlock>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL ORCHESTRA -->
          <div class="tool-card orchestra-viz q-pa-lg bg-slate-900 relative-position">
            <div class="text-center text-h6 text-white q-mb-md">ros2 launch robot.launch.py</div>

            <!-- CONDUCTOR -->
            <div
              class="conductor absolute-top text-center"
              style="left: 50%; transform: translateX(-50%)"
            >
              <q-icon name="music_note" size="3rem" color="teal-4" />
              <div class="text-caption text-teal-3">Launch File</div>
            </div>

            <!-- INSTRUMENTS (NODES) -->
            <div class="row q-col-gutter-sm q-mt-xl">
              <div class="col-3 text-center">
                <div class="instrument-box bg-blue-9">
                  <q-icon name="videocam" />
                  <div class="text-xs text-blue-3">camera</div>
                </div>
              </div>

              <div class="col-3 text-center">
                <div class="instrument-box bg-green-9">
                  <q-icon name="directions_car" />
                  <div class="text-xs text-green-3">motors</div>
                </div>
              </div>

              <div class="col-3 text-center">
                <div class="instrument-box bg-purple-9">
                  <q-icon name="psychology" />
                  <div class="text-xs text-purple-3">brain</div>
                </div>
              </div>

              <div class="col-3 text-center">
                <div class="instrument-box bg-orange-9">
                  <q-icon name="computer" />
                  <div class="text-xs text-orange-3">gui</div>
                </div>
              </div>
            </div>

            <!-- SOUND WAVES -->
            <svg
              class="absolute-bottom waves"
              style="width: 100%; height: 40px; pointer-events: none"
            >
              <defs>
                <filter id="glow">
                  <feGaussianBlur stdDeviation="2.5" flood-color="#14b8a6" result="coloredBlur" />
                  <feMerge>
                    <feMergeNode in="coloredBlur" />
                    <feMergeNode in="SourceGraphic" />
                  </feMerge>
                </filter>
              </defs>
              <path
                d="M0,0 Q100,20 200,0 T400,0 V40 H0 Z"
                fill="#14b8a6"
                filter="url(#glow)"
                class="wave-anim"
              />
            </svg>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. SINTAXIS PYTHON LAUNCH -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Sintaxis Python</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            ROS 2 prefiere Launch files en Python porque son más flexibles. Puedes hacer bucles,
            condicionales, leer argumentos y llamar otros Launch files.
            <br /><br />
            <strong>El archivo debe:</strong>
            <ul class="q-pl-md q-mt-sm text-grey-4">
              <li>Tener la extensión <code>.py</code>.</li>
              <li>Estar en <code>launch/</code> de tu paquete.</li>
              <li>Retornar una <code>LaunchDescription</code>.</li>
            </ul>
          </TextBlock>
        </template>

        <template #right>
          <div class="tool-card code-card bg-slate-900 q-pa-none">
            <div class="window-header row justify-between q-pa-sm bg-black">
              <span class="text-grey-5">robot/launch/robot.launch.py</span>
              <q-icon name="play_arrow" color="teal-4" />
            </div>

            <CodeBlock
              lang="python"
              :show-line-numbers="true"
              content="#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Nodo 1: Cámara
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            output='screen'
        ),

        # Nodo 2: Controlador de Motores
        Node(
            package='mi_robot_control',
            executable='motor_driver',
            name='motors',
            parameters=['motors.yaml']
        ),

        # Nodo 3: GUI
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz'
        )
    ])"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. PARÁMETROS DINÁMICOS -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Parámetros: La Magia</SectionTitle>
      <TextBlock>
        Los parámetros permiten configurar nodos sin tocar el código fuente. Puedes pasárselos desde
        el Launch File o desde la línea de comandos.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-6">
          <div class="tool-card param-card bg-slate-800 q-pa-lg">
            <div class="text-teal-4 text-weight-bold q-mb-sm">En el Launch</div>
            <div class="font-mono text-xs text-green-3 bg-black q-pa-sm rounded-borders q-mb-md">
              parameters=[{'max_speed': 1.0, 'robot_id': 42}]
            </div>
            <div class="text-caption text-grey-5">
              El nodo lee su configuración del archivo YAML que generó el Launch.
            </div>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card param-card bg-slate-800 q-pa-lg">
            <div class="text-teal-4 text-weight-bold q-mb-sm">Desde Terminal</div>
            <div class="font-mono text-xs text-green-3 bg-black q-pa-sm rounded-borders q-mb-md">
              ros2 launch robot/robot.launch.py max_speed:=2.0 robot_id:=99
            </div>
            <div class="text-caption text-grey-5">Sobrescribe los valores del Launch File.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. EJECUTAR Y DEBUG -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. Ejecutar y Debug</SectionTitle>

      <div class="row q-col-gutter-md">
        <!-- LAUNCH -->
        <div class="col-12 col-md-4">
          <div class="tool-card launch-card bg-teal-9 q-pa-lg text-center cursor-pointer">
            <q-icon name="play_arrow" size="3rem" color="white" />
            <div class="text-h6 text-white q-mt-sm">ros2 launch</div>
            <div class="text-caption text-white-alpha-8">robot/robot.launch.py</div>
          </div>
        </div>

        <!-- KILL -->
        <div class="col-12 col-md-4">
          <div class="tool-card kill-card bg-orange-9 q-pa-lg text-center cursor-pointer">
            <q-icon name="stop" size="3rem" color="white" />
            <div class="text-h6 text-white q-mt-sm">ros2 launch</div>
            <div class="text-caption text-white-alpha-8">--kill</div>
          </div>
        </div>

        <!-- LIST -->
        <div class="col-12 col-md-4">
          <div class="tool-card list-card bg-slate-900 q-pa-lg text-center cursor-pointer">
            <q-icon name="list_alt" size="3rem" color="teal-4" />
            <div class="text-h6 text-teal-4 q-mt-sm">ros2 launch</div>
            <div class="text-caption text-teal-3">--show-args</div>
          </div>
        </div>
      </div>

      <div class="text-center q-mt-lg">
        <div
          class="cmd-box bg-black text-teal-4 q-pa-md rounded-borders font-mono text-body2 inline-block"
        >
          ros2 launch mi_robot robot.launch.py robot_name:=turtlebot3
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(20, 184, 166, 0.15), transparent 60%),
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

/* TOOL CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* ORCHESTRA VIZ */
.orchestra-viz {
  border-top: 4px solid #14b8a6;
}

.instrument-box {
  width: 60px;
  height: 60px;
  border-radius: 12px;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  margin: 0 auto;
  transition: transform 0.3s;
}
.instrument-box:hover {
  transform: scale(1.1);
}

.waves {
  opacity: 0.8;
}
.wave-anim {
  animation: wave 3s ease-in-out infinite;
}
@keyframes wave {
  0%,
  100% {
    transform: scaleX(1);
  }
  50% {
    transform: scaleX(1.1);
  }
}

/* CODE CARD */
.code-card {
  overflow: hidden;
  border: 1px solid #333;
}

/* PARAM CARDS */
.param-card {
  transition: transform 0.3s;
  cursor: pointer;
}
.param-card:hover {
  transform: translateY(-5px);
}

/* LAUNCH BUTTONS */
.launch-card:hover {
  transform: scale(1.05);
  box-shadow: 0 10px 25px rgba(20, 184, 166, 0.3);
}
.kill-card:hover {
  transform: scale(1.05);
  box-shadow: 0 10px 25px rgba(251, 146, 60, 0.3);
}
.list-card:hover {
  transform: scale(1.05);
  box-shadow: 0 10px 25px rgba(20, 184, 166, 0.2);
}

.cursor-pointer {
  cursor: pointer;
}
.shadow-green {
  box-shadow: 0 0 15px rgba(74, 222, 128, 0.4);
}

.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.text-sm {
  font-size: 0.9rem;
}
.inline-block {
  display: inline-block;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
