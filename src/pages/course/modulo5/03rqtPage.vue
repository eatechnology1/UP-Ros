<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-cyan-4 text-weight-bold q-mb-sm">
          MÓDULO 5.3: DIAGNÓSTICO Y GUI
        </div>

        <h1 class="hero-title">
          RQT Suite: El Panel de <span class="text-white">Instrumentación</span>
        </h1>

        <TextBlock>
          Si RViz son los ojos del robot, <strong>RQT (ROS Qt)</strong> es su monitor de signos
          vitales. Aquí no verás paredes ni mapas 3D. Verás la salud del sistema: quién habla con
          quién (Topología), la estabilidad de los sensores (Gráficas) y los gritos de auxilio
          internos (Logs).
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. RQT Graph: La Radiografía</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Imagina una red de tuberías compleja. Si hay una fuga, necesitas el plano de fontanería.
            Eso es <code>rqt_graph</code>. Muestra todos los Nodos (elípes) conectados por Tópicos
            (flechas).
          </TextBlock>
          <ul class="q-pl-md q-mt-sm text-grey-4 tool-list">
            <li>❓ "¿Por qué mi nodo de navegación no recibe datos?" (Flecha rota).</li>
            <li>❓ "¿Hay nodos zombis consumiendo RAM?" (Nodos huérfanos).</li>
          </ul>
        </template>
        <template #right>
          <div
            class="graph-viz bg-slate-900 q-pa-md rounded-borders shadow-2 overflow-hidden relative-position border-light"
          >
            <div class="absolute-top-right q-pa-sm text-xs text-grey-6 font-mono">
              rqt_graph live
            </div>
            <div class="row justify-center items-center full-height q-gutter-x-lg">
              <div class="column items-center slide-in-1">
                <div class="node-oval bg-blue-9 text-white shadow-blue">/camera_driver</div>
              </div>
              <div class="connection-line relative-position fade-in-2">
                <div class="topic-label text-xxs text-grey-4 bg-black q-px-xs rounded">
                  /image_raw
                </div>
                <div class="flow-particle"></div>
              </div>
              <div class="column items-center slide-in-3">
                <div class="node-oval bg-purple-9 text-white shadow-purple">/obj_detector</div>
              </div>
              <div class="connection-line relative-position fade-in-4">
                <div class="topic-label text-xxs text-grey-4 bg-black q-px-xs rounded">
                  /detections
                </div>
                <div class="flow-particle delay-1"></div>
              </div>
              <div class="column items-center slide-in-5">
                <div class="node-oval bg-green-9 text-white shadow-green">/nav_planner</div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>2. RQT Console: Filtrando el Ruido</SectionTitle>
      <TextBlock>
        Cuando tienes 50 nodos imprimiendo a la vez, la terminal es inútil.
        <strong>rqt_console</strong> atrapa los mensajes y te permite pausar el tiempo.
      </TextBlock>

      <div
        class="console-mockup bg-black border-grey q-mt-md shadow-2 overflow-hidden rounded-borders"
      >
        <div class="row items-center q-pa-sm bg-slate-800 border-bottom-dark">
          <div class="text-grey-5 text-xs q-mr-md font-mono">Logger Level:</div>
          <q-btn-group flat dense>
            <q-btn
              :color="logFilter === 'ALL' ? 'blue' : 'grey-7'"
              size="sm"
              label="ALL"
              @click="logFilter = 'ALL'"
            />
            <q-btn
              :color="logFilter === 'WARN' ? 'yellow-9' : 'grey-7'"
              size="sm"
              label="WARN+"
              @click="logFilter = 'WARN'"
            />
            <q-btn
              :color="logFilter === 'ERROR' ? 'red-9' : 'grey-7'"
              size="sm"
              label="ERROR"
              @click="logFilter = 'ERROR'"
            />
          </q-btn-group>
          <q-space />
          <div class="text-xs text-grey-6 font-mono q-mr-sm">Buffer: 6/1000</div>
          <q-btn flat round dense icon="delete_sweep" color="grey-5" size="sm" />
        </div>

        <div
          class="log-container q-pa-sm font-mono text-xs scroll relative-position"
          style="height: 200px"
        >
          <transition-group name="list">
            <div
              v-for="log in filteredLogs"
              :key="log.id"
              class="log-row q-mb-xs q-pa-xs rounded row items-start"
              :class="getLogClass(log.level)"
            >
              <div class="col-2 text-weight-bold">[{{ log.level }}]</div>
              <div class="col-3 opacity-80">{{ log.node }}:</div>
              <div class="col">{{ log.msg }}</div>
            </div>
          </transition-group>
          <div v-if="filteredLogs.length === 0" class="absolute-center text-grey-7 italic">
            -- No hay logs de nivel {{ logFilter }} --
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Topic Monitor: Salud de Datos</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            ¿Tus datos llegan? ¿Llegan a tiempo? El <strong>Topic Monitor</strong> te da métricas
            duras que <code>echo</code> no muestra.
          </TextBlock>
          <div class="row q-col-gutter-sm q-mt-sm">
            <div class="col-12">
              <div class="q-pa-sm bg-slate-800 rounded border-left-cyan">
                <div class="text-cyan-3 text-weight-bold">Hz (Frecuencia)</div>
                <div class="text-xs text-grey-4">
                  Estabilidad del ciclo. Si varía mucho, tu nodo está saturado.
                </div>
              </div>
            </div>
            <div class="col-12">
              <div class="q-pa-sm bg-slate-800 rounded border-left-blue">
                <div class="text-blue-3 text-weight-bold">BW (Bandwidth)</div>
                <div class="text-xs text-grey-4">Peso en MB/s. Crítico para WiFi.</div>
              </div>
            </div>
          </div>
        </template>
        <template #right>
          <div
            class="dashboard-viz bg-slate-900 q-pa-md rounded-borders shadow-2 border-light full-height flex column justify-center"
          >
            <div class="monitor-row row items-center q-mb-md bg-black q-pa-sm rounded border-light">
              <div class="col-grow">
                <div class="text-xs text-grey-5 font-mono">/scan (Lidar)</div>
                <div class="row q-gutter-x-md">
                  <span class="text-green-4 text-weight-bold">10.0 Hz</span>
                  <span class="text-grey-6">|</span>
                  <span class="text-blue-4">0.45 MB/s</span>
                </div>
              </div>
              <div class="led-indicator bg-green-5 shadow-green-glow pulse-fast"></div>
            </div>

            <div class="monitor-row row items-center q-mb-md bg-black q-pa-sm rounded border-light">
              <div class="col-grow">
                <div class="text-xs text-grey-5 font-mono">/camera/image_raw</div>
                <div class="row q-gutter-x-md">
                  <span class="text-yellow-4 text-weight-bold">14.2 Hz</span>
                  <span class="text-grey-6 text-xs">(exp: 30)</span>
                  <span class="text-red-4">25.0 MB/s</span>
                </div>
              </div>
              <div class="led-indicator bg-yellow-5 shadow-yellow-glow pulse-slow"></div>
            </div>

            <div
              class="monitor-row row items-center bg-black q-pa-sm rounded border-light opacity-50"
            >
              <div class="col-grow">
                <div class="text-xs text-grey-5 font-mono">/imu/data</div>
                <div class="text-red-9 text-xs">NO DATA RECEIVED</div>
              </div>
              <div class="led-indicator bg-red-9 border-red"></div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. PlotJuggler: El Osciloscopio</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            El estándar moderno para gráficas. Arrastra tópicos y visualiza series temporales en
            tiempo real.
            <br /><br />
            <strong>Ideal para PIDs:</strong> Compara lo que <em>quieres</em> (Setpoint) vs lo que
            <em>tienes</em> (State).
          </TextBlock>
          <CodeBlock
            title="Instalación"
            lang="bash"
            content="sudo apt install ros-humble-plotjuggler-ros"
            :copyable="true"
          />
        </div>
        <div class="col-12 col-md-7">
          <div
            class="plot-card bg-slate-900 q-pa-md border-blue shadow-2 relative-position overflow-hidden rounded-borders"
          >
            <div
              class="text-blue-4 text-xs font-mono absolute-top-left q-ma-sm"
              style="z-index: 10"
            >
              /joint_states/pos[0]
            </div>
            <svg viewBox="0 0 100 40" class="full-width" style="height: 180px">
              <defs>
                <pattern id="grid" width="10" height="10" patternUnits="userSpaceOnUse">
                  <path
                    d="M 10 0 L 0 0 0 10"
                    fill="none"
                    stroke="rgba(255,255,255,0.05)"
                    stroke-width="0.5"
                  />
                </pattern>
              </defs>
              <rect width="100" height="40" fill="url(#grid)" />

              <path
                d="M0 20 Q 15 5, 30 20 T 60 20 T 90 20"
                fill="none"
                stroke="#60a5fa"
                stroke-width="0.8"
                class="wave-path"
              />
              <path
                d="M0 20 Q 15 25, 30 20 T 60 20 T 90 20"
                fill="none"
                stroke="#f472b6"
                stroke-width="0.8"
                stroke-dasharray="2,1"
                class="wave-path delay-wave"
              />
            </svg>
            <div class="row justify-center q-mt-xs text-xs font-mono">
              <div class="row items-center q-mr-md">
                <div class="box-legend bg-blue-4 q-mr-xs"></div>
                Actual
              </div>
              <div class="row items-center">
                <div class="box-legend bg-pink-4 q-mr-xs"></div>
                Target
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mt-xl">
      <div class="row q-col-gutter-xl">
        <div class="col-12 col-md-6">
          <SectionTitle>5. Image View</SectionTitle>
          <TextBlock>Verifica cámaras sin la pesadez de RViz. Stream raw o comprimido.</TextBlock>

          <div
            class="camera-viz bg-black q-mt-md rounded-borders overflow-hidden relative-position border-grey shadow-2"
          >
            <div class="absolute-full static-noise opacity-20"></div>
            <div class="scan-line"></div>

            <div class="absolute-top-left q-pa-sm row items-center">
              <div class="rec-dot bg-red-6 pulse-fast q-mr-sm"></div>
              <div class="text-xs text-white font-mono">REC [640x480]</div>
            </div>
            <div class="absolute-center text-grey-8 text-h2 opacity-30">
              <q-icon name="videocam_off" />
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <SectionTitle>6. Service Caller</SectionTitle>
          <TextBlock
            >Llama servicios manualmente rellenando campos en lugar de escribir JSONs.</TextBlock
          >

          <div
            class="service-viz bg-slate-800 q-mt-md q-pa-md rounded-borders border-left-purple shadow-2"
          >
            <div class="text-xs text-purple-3 font-mono q-mb-sm">
              /spawn_turtle (turtlesim/srv/Spawn)
            </div>

            <div class="q-gutter-y-xs">
              <div class="row items-center">
                <div class="col-3 text-grey-4 text-xs text-right q-pr-sm">x:</div>
                <div
                  class="col bg-black text-white text-xs q-px-sm q-py-xs rounded border-light font-mono"
                >
                  5.5
                </div>
              </div>
              <div class="row items-center">
                <div class="col-3 text-grey-4 text-xs text-right q-pr-sm">y:</div>
                <div
                  class="col bg-black text-white text-xs q-px-sm q-py-xs rounded border-light font-mono"
                >
                  5.5
                </div>
              </div>
              <div class="row items-center">
                <div class="col-3 text-grey-4 text-xs text-right q-pr-sm">name:</div>
                <div
                  class="col bg-black text-green-4 text-xs q-px-sm q-py-xs rounded border-light font-mono"
                >
                  "my_turtle"
                </div>
              </div>
            </div>

            <div class="q-mt-md text-right">
              <q-btn size="sm" color="purple-9" label="Call Service" icon="send" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>Resumen de Comandos</SectionTitle>
      <div class="custom-card bg-slate-800 border-grey shadow-2">
        <div class="q-pa-md">
          <table class="cmd-table full-width text-left font-mono text-xs text-grey-4">
            <thead>
              <tr class="text-blue-4 border-bottom-dark">
                <th class="q-pb-sm">Tool</th>
                <th class="q-pb-sm">Command</th>
              </tr>
            </thead>
            <tbody>
              <tr class="hover-row">
                <td class="q-py-sm text-weight-bold">Graph</td>
                <td class="text-green-4">rqt_graph</td>
              </tr>
              <tr class="hover-row">
                <td class="q-py-sm text-weight-bold">Console</td>
                <td class="text-green-4">ros2 run rqt_console rqt_console</td>
              </tr>
              <tr class="hover-row">
                <td class="q-py-sm text-weight-bold">Plot</td>
                <td class="text-green-4">ros2 run plotjuggler plotjuggler</td>
              </tr>
              <tr class="hover-row">
                <td class="q-py-sm text-weight-bold">Image</td>
                <td class="text-green-4">ros2 run rqt_image_view rqt_image_view</td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';

// --- CONSOLE LOGIC ---
const logFilter = ref('ALL');
const logs = [
  { id: 1, level: 'INFO', node: '/camera', msg: 'Driver initialized. 30fps.' },
  { id: 2, level: 'WARN', node: '/nav_stack', msg: 'Control loop missed rate (20Hz).' },
  { id: 3, level: 'INFO', node: '/planner', msg: 'Path found: 45 waypoints.' },
  { id: 4, level: 'ERROR', node: '/serial', msg: 'Connection lost: /dev/ttyUSB0' },
  { id: 5, level: 'DEBUG', node: '/slam', msg: 'Particle filter conv: 0.045' },
  { id: 6, level: 'FATAL', node: '/core', msg: 'WATCHDOG TRIGGERED! ABORT.' },
];

const filteredLogs = computed(() => {
  if (logFilter.value === 'ALL') return logs;
  if (logFilter.value === 'WARN')
    return logs.filter((l) => ['WARN', 'ERROR', 'FATAL'].includes(l.level));
  if (logFilter.value === 'ERROR') return logs.filter((l) => ['ERROR', 'FATAL'].includes(l.level));
  return logs;
});

const getLogClass = (level: string) => {
  switch (level) {
    case 'INFO':
      return 'text-green-4';
    case 'WARN':
      return 'text-yellow-4 bg-yellow-9-soft';
    case 'ERROR':
      return 'text-red-4 bg-red-9-soft';
    case 'FATAL':
      return 'text-white bg-red-7 text-weight-bolder';
    case 'DEBUG':
      return 'text-blue-4';
    default:
      return 'text-grey-4';
  }
};
</script>

<style scoped>
/* --- ESTILOS GENERALES --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(59, 130, 246, 0.15), transparent 60%),
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
  color: #f8fafc;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.text-xxs {
  font-size: 0.6rem;
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-grey {
  border: 1px solid #334155;
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.rounded {
  border-radius: 6px;
}

/* GRAPH VIZ & ANIMATION */
.graph-viz {
  height: 200px;
}
.node-oval {
  padding: 8px 16px;
  border-radius: 20px;
  font-family: 'Fira Code';
  font-size: 0.8rem;
  border: 1px solid rgba(255, 255, 255, 0.2);
}
.shadow-blue {
  box-shadow: 0 0 15px rgba(59, 130, 246, 0.4);
}
.shadow-purple {
  box-shadow: 0 0 15px rgba(168, 85, 247, 0.4);
}
.shadow-green {
  box-shadow: 0 0 15px rgba(74, 222, 128, 0.4);
}
.connection-line {
  width: 80px;
  height: 2px;
  background: #475569;
  display: flex;
  align-items: center;
  justify-content: center;
}
.topic-label {
  transform: translateY(-12px);
}

/* Animation Classes */
.slide-in-1 {
  animation: slideUp 0.5s ease-out backwards;
}
.slide-in-3 {
  animation: slideUp 0.5s ease-out 0.4s backwards;
}
.slide-in-5 {
  animation: slideUp 0.5s ease-out 0.8s backwards;
}
.fade-in-2 {
  animation: fadeIn 0.5s ease-out 0.2s backwards;
}
.fade-in-4 {
  animation: fadeIn 0.5s ease-out 0.6s backwards;
}

@keyframes slideUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}
@keyframes fadeIn {
  from {
    opacity: 0;
  }
  to {
    opacity: 1;
  }
}

.flow-particle {
  position: absolute;
  width: 6px;
  height: 6px;
  background: #facc15;
  border-radius: 50%;
  box-shadow: 0 0 5px #facc15;
  animation: flowMove 2s infinite linear;
}
.delay-1 {
  animation-delay: 1s;
  background: #a855f7;
  box-shadow: 0 0 5px #a855f7;
}
@keyframes flowMove {
  0% {
    left: 0;
    opacity: 0;
  }
  10%,
  90% {
    opacity: 1;
  }
  100% {
    left: 100%;
    opacity: 0;
  }
}

/* CONSOLE */
.bg-yellow-9-soft {
  background: rgba(234, 179, 8, 0.15);
}
.bg-red-9-soft {
  background: rgba(239, 68, 68, 0.15);
}
.opacity-80 {
  opacity: 0.8;
}
.list-enter-active,
.list-leave-active {
  transition: all 0.3s ease;
}
.list-enter-from,
.list-leave-to {
  opacity: 0;
  transform: translateX(20px);
}

/* DASHBOARD VIZ */
.dashboard-viz {
  min-height: 200px;
}
.border-left-cyan {
  border-left: 3px solid #22d3ee;
}
.border-left-blue {
  border-left: 3px solid #3b82f6;
}
.led-indicator {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}
.shadow-green-glow {
  box-shadow: 0 0 8px #4ade80;
}
.shadow-yellow-glow {
  box-shadow: 0 0 8px #facc15;
}
.pulse-fast {
  animation: pulse 1s infinite;
}
.pulse-slow {
  animation: pulse 2s infinite;
}
@keyframes pulse {
  0% {
    opacity: 1;
    transform: scale(1);
  }
  50% {
    opacity: 0.5;
    transform: scale(0.8);
  }
  100% {
    opacity: 1;
    transform: scale(1);
  }
}
.opacity-50 {
  opacity: 0.5;
}

/* PLOT VIZ */
.plot-card {
  height: 180px;
}
.wave-path {
  stroke-dasharray: 100;
  animation: dash 3s linear infinite;
}
.delay-wave {
  animation-delay: 0.5s;
  stroke-dasharray: 100;
  stroke-dashoffset: 50;
}
@keyframes dash {
  from {
    stroke-dashoffset: 200;
  }
  to {
    stroke-dashoffset: 0;
  }
}
.box-legend {
  width: 10px;
  height: 10px;
  border-radius: 2px;
}

/* CAMERA VIZ */
.camera-viz {
  height: 180px;
  background: #000;
}
.static-noise {
  background-image: url('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAIAAAACCAYAAABytg0kAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAABZJREFUeNpi2r9//38gYGAEESAAEGAAasgJOgzOKCoAAAAASUVORK5CYII=');
}
.opacity-20 {
  opacity: 0.2;
}
.opacity-30 {
  opacity: 0.3;
}
.scan-line {
  width: 100%;
  height: 2px;
  background: rgba(34, 197, 94, 0.5);
  position: absolute;
  animation: scan 3s infinite linear;
  box-shadow: 0 0 10px rgba(34, 197, 94, 0.5);
}
@keyframes scan {
  0% {
    top: 0;
  }
  100% {
    top: 100%;
  }
}
.rec-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
}

/* SERVICE VIZ */
.service-viz {
  height: 100%;
}
.border-left-purple {
  border-left: 4px solid #9333ea;
}
.custom-card {
  border-radius: 12px;
}
.hover-row:hover {
  background: rgba(255, 255, 255, 0.03);
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .graph-viz,
  .dashboard-viz,
  .plot-card,
  .camera-viz,
  .service-viz {
    height: auto;
    min-height: 180px;
  }
  .connection-line {
    width: 30px;
  }
}
</style>
