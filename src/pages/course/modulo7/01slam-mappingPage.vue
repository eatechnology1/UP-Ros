<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-purple-4 text-weight-bold q-mb-sm">
          MÓDULO 7.1: CARTOGRAFÍA ROBÓTICA
        </div>

        <h1 class="hero-title">SLAM: El Arte de <span class="text-white">Crear Mundos</span></h1>

        <TextBlock>
          Imagina que te despiertas en una habitación oscura. Solo puedes tocar las paredes. Para
          dibujar un plano, necesitas saber cuánto has caminado. Pero si resbalas, el plano saldrá
          torcido. Y si el plano está torcido, no sabrás dónde estás.
          <br /><br />
          Este es el problema <strong>SLAM</strong>: Construir el mapa mientras te ubicas en él,
          corrigiendo errores en tiempo real.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. ¿Qué come el algoritmo?</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            <strong>SLAM Toolbox</strong> necesita fusionar dos fuentes de verdad contradictorias:
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>👀 <strong>Lidar (Scan):</strong> "Veo una pared a 2m". (Preciso pero local).</li>
            <li>🦶 <strong>Odometría (Odom):</strong> "Avancé 1m". (Acumula error).</li>
            <li>
              🧠 <strong>El Algoritmo:</strong> Compara ambas y dice: "Según las ruedas me moví,
              pero el láser dice que la pared no se acercó. ¡Patiné! Corregiré la posición".
            </li>
          </ul>
        </template>

        <template #right>
          <div
            class="tool-card slam-viz q-pa-lg bg-black relative-position overflow-hidden shadow-2"
          >
            <div
              class="row items-center justify-around full-height relative-position"
              style="z-index: 2"
            >
              <div class="column q-gutter-md">
                <div class="node-box bg-red-9 shadow-red transition-hover">
                  <div class="text-xs font-mono text-white">/scan</div>
                </div>
                <div class="node-box bg-blue-9 shadow-blue transition-hover">
                  <div class="text-xs font-mono text-white">/odom</div>
                </div>
              </div>

              <div class="column items-center">
                <div
                  class="node-box large-box bg-purple-9 shadow-purple border-light spin-border relative-position"
                >
                  <q-icon name="psychology" size="2.5rem" class="text-white" />
                </div>
                <div class="text-purple-4 font-mono text-xs q-mt-sm">SLAM Node</div>
              </div>

              <div class="column items-center">
                <div class="node-box bg-grey-3 shadow-1 text-black">
                  <div class="text-xs font-bold">/map</div>
                </div>
              </div>

              <div
                class="absolute-center"
                style="width: 80%; height: 2px; background: rgba(255, 255, 255, 0.1); z-index: -1"
              ></div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Anatomía de un Mapa</SectionTitle>
      <TextBlock>
        Para el robot, el mapa es una imagen de píxeles (Occupancy Grid). Cada píxel es una
        probabilidad (0-100).
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-5">
          <div class="grid-frame shadow-2 bg-grey-4">
            <div class="pixel-grid">
              <div v-for="n in 64" :key="n" class="pixel" :class="getPixelClass(n)"></div>
            </div>
            <div class="absolute-center">
              <q-icon name="navigation" class="text-blue-6 rotate-45" size="md" />
            </div>
          </div>
        </div>

        <div class="col-12 col-md-7">
          <div class="tool-card bg-slate-900 q-pa-md border-purple">
            <div class="row items-center q-mb-sm">
              <div class="pixel-legend bg-white border-black"></div>
              <div class="q-ml-sm text-grey-3"><strong>Blanco (0):</strong> Espacio Libre</div>
            </div>
            <div class="row items-center q-mb-sm">
              <div class="pixel-legend bg-black border-light"></div>
              <div class="q-ml-sm text-grey-3"><strong>Negro (100):</strong> Pared / Obstáculo</div>
            </div>
            <div class="row items-center">
              <div class="pixel-legend bg-grey-6 border-black"></div>
              <div class="q-ml-sm text-grey-3"><strong>Gris (-1):</strong> Desconocido</div>
            </div>
          </div>

          <div class="q-mt-md">
            <AlertBlock type="info" title="Resolución">
              Un estándar es <strong>0.05 m/px</strong>. Cada cuadradito son 5cm.
            </AlertBlock>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Ejecutando SLAM Toolbox</SectionTitle>
      <TextBlock> En ROS 2 Humble/Jazzy, usamos el modo asíncrono para robots reales. </TextBlock>

      <div class="tool-card cli-card bg-black q-pa-md q-mt-md border-purple-glow">
        <div class="window-dots row q-mb-md">
          <div class="dot bg-red-5 q-mr-xs"></div>
          <div class="dot bg-yellow-5 q-mr-xs"></div>
          <div class="dot bg-green-5"></div>
        </div>

        <div class="text-grey-6 font-mono text-xs q-mb-sm"># Terminal 1: Lanzar nodo</div>
        <div class="cmd-line q-pa-sm rounded bg-slate-900 q-mb-md">
          <span class="text-purple-4 font-mono text-sm">
            $ ros2 launch slam_toolbox online_async_launch.py
          </span>
        </div>

        <div class="text-white font-mono text-xs q-pl-sm border-left-purple">
          <div class="text-grey-5">[INFO] [slam_toolbox]: Node started</div>
          <div class="text-grey-5">[INFO] [slam_toolbox]: Using solver: sparse_pose_graph</div>
          <div class="text-green-4">Loop Closure: Enabled</div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. El "Déjà Vu" (Loop Closure)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El robot acumula error (drift). Al volver a un lugar conocido, reconoce la forma de la
            habitación.
            <br /><br />
            <strong>Loop Closure</strong> es el momento en que el algoritmo "jala" el mapa elástico
            para corregir ese error acumulado.
          </TextBlock>
        </template>
        <template #right>
          <div class="tool-card bg-black relative-position overflow-hidden h-300 flex flex-center">
            <div class="map-ghost border-red flex flex-center">
              <span class="text-red-3 text-caption font-mono">Drift</span>
            </div>
            <div class="map-real border-green flex flex-center">
              <span class="text-green-9 text-caption font-mono">Real</span>
            </div>

            <div class="absolute-bottom text-center q-mb-md">
              <div class="status-badge-slam font-mono text-xs"></div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>5. Guardar la Partida</SectionTitle>
      <AlertBlock type="warning" title="Memoria RAM">
        El mapa se perderá si apagas el nodo. Debes guardarlo en disco.
      </AlertBlock>

      <div class="row q-gutter-md q-mt-sm justify-center">
        <div class="file-card bg-slate-800 q-pa-md border-light text-center rounded-borders">
          <q-icon name="image" color="blue-4" size="md" />
          <div class="text-white font-mono q-mt-xs">map.pgm</div>
        </div>
        <div class="file-card bg-slate-800 q-pa-md border-light text-center rounded-borders">
          <q-icon name="description" color="yellow-4" size="md" />
          <div class="text-white font-mono q-mt-xs">map.yaml</div>
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

// Lógica para pintar el grid
const getPixelClass = (n: number) => {
  const row = Math.ceil(n / 8);
  const col = n % 8 === 0 ? 8 : n % 8;

  // Bordes desconocidos (Gris)
  if (row === 1 || row === 8 || col === 1 || col === 8) return 'bg-grey-6';
  // Obstáculo central (Negro)
  if ((row === 3 || row === 4) && (col === 3 || col === 4)) return 'bg-black';
  // Libre (Blanco)
  return 'bg-white';
};
</script>

<style scoped>
/* --- ESTILOS MAESTROS (Copiados de tu módulo funcional 4.4) --- */

.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(168, 85, 247, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8); /* Ajustado a Morado para SLAM */
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

.slam-viz {
  border-top: 4px solid #a855f7; /* Purple for SLAM */
  height: 320px;
}
.full-height {
  height: 100%;
}

/* NODE BOXES */
.node-box {
  width: 60px;
  height: 60px;
  border-radius: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
}
.large-box {
  width: 80px;
  height: 80px;
}

.shadow-red {
  box-shadow: 0 0 10px rgba(239, 68, 68, 0.5);
}
.shadow-blue {
  box-shadow: 0 0 10px rgba(59, 130, 246, 0.5);
}
.shadow-purple {
  box-shadow: 0 0 20px rgba(168, 85, 247, 0.4);
}

.spin-border {
  animation: pulsePurple 3s infinite;
}
@keyframes pulsePurple {
  0% {
    box-shadow: 0 0 0 0 rgba(168, 85, 247, 0.7);
  }
  70% {
    box-shadow: 0 0 0 15px rgba(168, 85, 247, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(168, 85, 247, 0);
  }
}

/* GRID VIZ */
.grid-frame {
  padding: 4px;
  border: 2px solid #333;
  display: inline-block;
  position: relative;
  width: 100%;
  max-width: 300px;
  aspect-ratio: 1;
}
.pixel-grid {
  display: grid;
  grid-template-columns: repeat(8, 1fr);
  width: 100%;
  height: 100%;
  gap: 1px;
}
.pixel-legend {
  width: 20px;
  height: 20px;
  border-radius: 4px;
}
.border-purple {
  border-left: 4px solid #a855f7;
}

/* CLI CARD */
.border-purple-glow {
  border: 1px solid #a855f7;
  box-shadow: 0 0 20px rgba(168, 85, 247, 0.15);
}
.border-left-purple {
  border-left: 2px solid #a855f7;
}
.window-dots .dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}

/* LOOP CLOSURE ANIMATION */
.h-300 {
  height: 300px;
}
.map-real {
  width: 120px;
  height: 120px;
  border: 3px solid #4ade80;
  position: absolute;
}
.map-ghost {
  width: 120px;
  height: 120px;
  border: 3px dashed #f87171;
  background: rgba(248, 113, 113, 0.1);
  position: absolute;
  animation: loopClose 6s infinite cubic-bezier(0.22, 1, 0.36, 1);
}

.status-badge-slam::after {
  content: 'DRIFTING...';
  color: #f87171;
  animation: statusText 6s infinite step-end;
}

@keyframes loopClose {
  0%,
  10% {
    transform: translate(-20px, -20px) rotate(-15deg);
    opacity: 0.6;
  }
  45% {
    transform: translate(-20px, -20px) rotate(-15deg);
    opacity: 1;
  }
  50% {
    transform: translate(0, 0) rotate(0deg);
    opacity: 0;
  } /* SNAP */
  90% {
    transform: translate(0, 0) rotate(0deg);
    opacity: 0;
  }
  100% {
    transform: translate(-20px, -20px) rotate(-15deg);
    opacity: 0.6;
  }
}

@keyframes statusText {
  0% {
    content: 'DRIFTING...';
    color: #f87171;
  }
  50% {
    content: 'LOOP CLOSURE!';
    color: #4ade80;
    font-weight: bold;
  }
  90% {
    content: 'LOOP CLOSURE!';
    color: #4ade80;
  }
  100% {
    content: 'DRIFTING...';
    color: #f87171;
  }
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
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
.border-black {
  border: 1px solid rgba(0, 0, 0, 0.5);
}
.transition-hover {
  transition: transform 0.2s;
}
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 12px;
  font-size: 1rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
