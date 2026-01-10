<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-primary text-weight-bold q-mb-sm">
          MÓDULO 4.1: FILOSOFÍA DE DISEÑO
        </div>

        <h1 class="hero-title">El Grafo de <span class="text-white">Computación</span></h1>

        <TextBlock>
          Olvida la idea de escribir un solo programa gigante llamado <code>main.py</code> que haga
          todo. Eso es frágil. ROS 2 propone una arquitectura distribuida: divides tu robot en
          pequeñas piezas independientes (<strong>Nodos</strong>) que conversan entre sí. Si la
          cámara falla, las ruedas siguen girando. Bienvenido a la robustez.
        </TextBlock>
      </div>
    </section>

    <!-- 2. MONOLITO VS DISTRIBUIDO -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Divide y Vencerás</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            En la programación clásica (Arduino), tienes un bucle infinito <code>loop()</code>. Si
            una línea falla, todo el robot se reinicia. <br /><br />
            En ROS 2, usamos un <strong>Grafo</strong>. Cada función vital (ver, moverse, pensar) es
            un programa separado.
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>
              🛡️ <strong>Aislamiento de Fallos:</strong> Si el nodo de Visión explota, el nodo de
              Motores frena el robot de forma segura.
            </li>
            <li>
              🌐 <strong>Lenguaje Agnóstico:</strong> El nodo de Visión puede estar en C++
              (velocidad) y el de Cerebro en Python (IA).
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL COMPARISON -->
          <div class="tool-card architecture-card bg-slate-900 q-pa-lg shadow-2">
            <div class="row items-center justify-around">
              <!-- MONOLITH -->
              <div
                class="text-center opacity-60 grayscale hover-color transition-all cursor-pointer group"
              >
                <div class="text-h6 text-red-4 q-mb-sm group-hover:text-red-3">El Monolito</div>
                <div
                  class="monolith-box bg-red-9 text-white column justify-center shadow-1 border-red-dim"
                >
                  <div class="q-my-xs">CÁMARA</div>
                  <div class="text-xs text-red-3">+</div>
                  <div class="q-my-xs">MOTORES</div>
                  <div class="text-xs text-red-3">+</div>
                  <div class="q-my-xs">LIDAR</div>
                </div>
                <div class="text-caption text-red-3 q-mt-sm font-italic">"Un error mata todo"</div>
              </div>

              <div class="vertical-separator bg-grey-8" style="width: 1px; height: 180px"></div>

              <!-- GRAPH (ROS) -->
              <div class="text-center">
                <div class="text-h6 text-green-4 q-mb-sm">El Grafo ROS</div>
                <div class="graph-container relative-position">
                  <!-- NODE 1 -->
                  <div
                    class="node-circle bg-blue-6 absolute shadow-glow-blue"
                    style="top: 0; left: 50%; transform: translateX(-50%)"
                  >
                    <q-icon name="videocam" size="sm" />
                  </div>

                  <!-- NODE 2 -->
                  <div
                    class="node-circle bg-orange-6 absolute shadow-glow-orange"
                    style="bottom: 0; left: 0"
                  >
                    <q-icon name="settings" size="sm" />
                  </div>

                  <!-- NODE 3 -->
                  <div
                    class="node-circle bg-purple-6 absolute shadow-glow-purple"
                    style="bottom: 0; right: 0"
                  >
                    <q-icon name="psychology" size="sm" />
                  </div>

                  <!-- CONNECTIONS (SVG LINES) -->
                  <svg
                    class="absolute-full"
                    style="pointer-events: none; width: 100%; height: 100%"
                  >
                    <!-- Top to Left -->
                    <line
                      x1="50%"
                      y1="20%"
                      x2="20%"
                      y2="80%"
                      stroke="#4ade80"
                      stroke-width="2"
                      stroke-dasharray="4"
                      class="flow-anim"
                    />
                    <!-- Top to Right -->
                    <line
                      x1="50%"
                      y1="20%"
                      x2="80%"
                      y2="80%"
                      stroke="#4ade80"
                      stroke-width="2"
                      stroke-dasharray="4"
                      class="flow-anim delay-1"
                    />
                    <!-- Left to Right -->
                    <line
                      x1="20%"
                      y1="80%"
                      x2="80%"
                      y2="80%"
                      stroke="#4ade80"
                      stroke-width="2"
                      stroke-dasharray="4"
                      class="flow-anim delay-2"
                    />
                  </svg>
                </div>
                <div class="text-caption text-green-3 q-mt-sm font-italic">
                  "Independientes y conectados"
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. ELEMENTOS DEL GRAFO -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Los Habitantes del Grafo</SectionTitle>
      <div class="row q-col-gutter-lg q-mt-sm">
        <!-- NODOS -->
        <div class="col-12 col-md-4">
          <div class="tool-card element-card border-blue bg-slate-800 transition-hover shadow-2">
            <div class="row items-center q-mb-md">
              <div class="icon-box bg-blue-9 shadow-1"><q-icon name="circle" color="white" /></div>
              <div class="text-h6 text-blue-4 q-ml-md">Nodos</div>
            </div>
            <p class="text-grey-4 text-sm col-grow">
              Son los <strong>Procesos</strong> (ejecutables). Hacen el trabajo real de computación.
            </p>
            <div
              class="bg-black q-pa-xs rounded text-center text-xs text-blue-2 font-mono border-blue-dim"
            >
              camera_driver
            </div>
          </div>
        </div>

        <!-- TOPICS -->
        <div class="col-12 col-md-4">
          <div class="tool-card element-card border-green bg-slate-800 transition-hover shadow-2">
            <div class="row items-center q-mb-md">
              <div class="icon-box bg-green-9 shadow-1">
                <q-icon name="alt_route" color="white" />
              </div>
              <div class="text-h6 text-green-4 q-ml-md">Topics</div>
            </div>
            <p class="text-grey-4 text-sm col-grow">
              Son las <strong>Tuberías</strong> de datos. Flujo continuo de información (Streaming).
            </p>
            <div
              class="bg-black q-pa-xs rounded text-center text-xs text-green-2 font-mono border-green-dim"
            >
              /camera/image_raw
            </div>
          </div>
        </div>

        <!-- SERVICES -->
        <div class="col-12 col-md-4">
          <div class="tool-card element-card border-yellow bg-slate-800 transition-hover shadow-2">
            <div class="row items-center q-mb-md">
              <div class="icon-box bg-yellow-9 shadow-1">
                <q-icon name="handshake" color="black" />
              </div>
              <div class="text-h6 text-yellow-4 q-ml-md">Services</div>
            </div>
            <p class="text-grey-4 text-sm col-grow">
              Son <strong>Solicitudes</strong> puntuales. Pregunta y respuesta síncrona.
            </p>
            <div
              class="bg-black q-pa-xs rounded text-center text-xs text-yellow-2 font-mono border-yellow-dim"
            >
              /reset_system
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. EL SECRETO: DDS (MIDDLEWARE) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. El Secreto Invisible: DDS</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            ¿Cómo se encuentran los nodos entre sí? No hay un servidor central (como en ROS 1).
            <br /><br />
            ROS 2 usa un estándar industrial llamado
            <strong>DDS (Data Distribution Service)</strong>. Es como magia: si dos nodos están en
            la misma red Wi-Fi y tienen el mismo <code>ROS_DOMAIN_ID</code>, se descubren
            automáticamente.
          </TextBlock>
        </template>

        <template #right>
          <div
            class="tool-card dds-visual bg-black q-pa-xl relative-position overflow-hidden shadow-2 flex flex-center"
          >
            <!-- BACKGROUND GRID -->
            <div class="grid-bg absolute-full"></div>

            <div
              class="row items-center justify-between relative-position full-width"
              style="z-index: 2"
            >
              <!-- NODE A -->
              <div class="column items-center">
                <div class="node-hex bg-slate-800 border-blue-neon shadow-glow-blue">
                  <q-icon name="smart_toy" color="blue-4" size="1.8rem" />
                </div>
                <div class="text-caption text-blue-4 q-mt-sm font-mono text-weight-bold">Robot</div>
              </div>

              <!-- WIFI WAVES -->
              <div class="column items-center justify-center col">
                <div class="relative-position flex flex-center">
                  <q-icon name="wifi" size="4rem" color="grey-7" class="wifi-pulse absolute" />
                  <q-icon
                    name="wifi"
                    size="4rem"
                    color="white"
                    class="wifi-pulse delay-1 absolute"
                    style="opacity: 0.5"
                  />
                </div>
                <div class="text-xs text-grey-5 q-mt-xl text-center bg-dark-soft q-px-sm rounded">
                  Discovery (Multicast)
                </div>
              </div>

              <!-- NODE B -->
              <div class="column items-center">
                <div class="node-hex bg-slate-800 border-purple-neon shadow-glow-purple">
                  <q-icon name="laptop" color="purple-4" size="1.8rem" />
                </div>
                <div class="text-caption text-purple-4 q-mt-sm font-mono text-weight-bold">
                  Laptop
                </div>
              </div>
            </div>

            <div class="absolute-bottom text-center q-mb-md">
              <div
                class="chip bg-grey-9 text-green-4 text-xs inline-block font-mono border-light q-px-md"
              >
                ID: 30
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. ROS 1 VS ROS 2 (LA EVOLUCIÓN) -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. ¿Por qué ROS 2?</SectionTitle>
      <div
        class="tool-card comparison-table bg-slate-900 q-pa-none shadow-2 overflow-hidden border-light"
      >
        <!-- HEADER -->
        <div class="row text-center bg-black q-py-md text-weight-bold border-bottom-dark">
          <div class="col-6 text-grey-5 text-subtitle1">ROS 1 (Legacy)</div>
          <div class="col-6 text-primary text-subtitle1">ROS 2 (Futuro)</div>
        </div>

        <!-- ROW 1 -->
        <div class="row q-pa-lg border-bottom-dark items-center bg-slate-800">
          <div class="col-6 text-center text-grey-4">Necesita un "Master" central</div>
          <div class="col-6 text-center text-white row items-center justify-center">
            <q-icon name="hub" color="green-4" class="q-mr-sm" size="sm" />
            <span class="text-weight-bold">Totalmente Descentralizado</span>
          </div>
        </div>

        <!-- ROW 2 -->
        <div class="row q-pa-lg border-bottom-dark items-center">
          <div class="col-6 text-center text-grey-4">Solo Linux (Ubuntu)</div>
          <div class="col-6 text-center text-white row items-center justify-center">
            <q-icon name="public" color="blue-4" class="q-mr-sm" size="sm" />
            <span>Windows, Mac, Linux, RTOS</span>
          </div>
        </div>

        <!-- ROW 3 -->
        <div class="row q-pa-lg items-center bg-slate-800">
          <div class="col-6 text-center text-grey-4">Python 2 / C++ 98</div>
          <div class="col-6 text-center text-white row items-center justify-center">
            <q-icon name="code" color="purple-4" class="q-mr-sm" size="sm" />
            <span>Python 3 / C++ Moderno (14/17)</span>
          </div>
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
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
    radial-gradient(circle at center, rgba(56, 189, 248, 0.15), transparent 60%),
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

/* ARCHITECTURE VISUAL */
.architecture-card {
  border-top: 4px solid #38bdf8;
}

.monolith-box {
  width: 120px;
  height: 160px;
  border-radius: 8px;
  margin: 0 auto;
  font-weight: bold;
  font-size: 0.85rem;
}
.border-red-dim {
  border: 1px solid rgba(248, 113, 113, 0.3);
}

.graph-container {
  width: 200px;
  height: 160px;
  margin: 0 auto;
}
.node-circle {
  width: 45px;
  height: 45px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  z-index: 2;
  border: 2px solid rgba(255, 255, 255, 0.2);
}

.flow-anim {
  stroke-dasharray: 4;
  animation: dash 1s linear infinite;
}
.delay-1 {
  animation-delay: 0.3s;
}
.delay-2 {
  animation-delay: 0.6s;
}

@keyframes dash {
  to {
    stroke-dashoffset: -8;
  }
}

.grayscale {
  filter: grayscale(100%);
  opacity: 0.5;
}
.hover-color:hover {
  filter: grayscale(0%);
  opacity: 1;
  transform: scale(1.05);
}
.transition-all {
  transition: all 0.3s ease;
}
.group:hover .monolith-box {
  box-shadow: 0 0 20px rgba(239, 68, 68, 0.5);
}

/* ELEMENT CARDS */
.element-card {
  padding: 24px;
  display: flex;
  flex-direction: column;
}
.transition-hover {
  transition:
    transform 0.3s,
    background 0.3s;
}
.transition-hover:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.8);
}
.border-blue {
  border-left: 4px solid #3b82f6;
}
.border-green {
  border-left: 4px solid #4ade80;
}
.border-yellow {
  border-left: 4px solid #facc15;
}
.col-grow {
  flex-grow: 1;
}
.border-blue-dim {
  border: 1px solid rgba(59, 130, 246, 0.3);
}
.border-green-dim {
  border: 1px solid rgba(74, 222, 128, 0.3);
}
.border-yellow-dim {
  border: 1px solid rgba(250, 204, 21, 0.3);
}

.icon-box {
  width: 48px;
  height: 48px;
  border-radius: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* DDS VISUAL */
.dds-visual {
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.grid-bg {
  background-image: radial-gradient(rgba(255, 255, 255, 0.15) 1px, transparent 1px);
  background-size: 24px 24px;
  opacity: 0.6;
}
.node-hex {
  width: 70px;
  height: 70px;
  clip-path: polygon(50% 0%, 100% 25%, 100% 75%, 50% 100%, 0% 75%, 0% 25%);
  display: flex;
  align-items: center;
  justify-content: center;
  background: #1e293b;
  position: relative;
}
/* Hex border hack */
.node-hex::before {
  content: '';
  position: absolute;
  inset: 0;
  background: inherit;
  z-index: -1;
  clip-path: polygon(50% 0%, 100% 25%, 100% 75%, 50% 100%, 0% 75%, 0% 25%);
}
.border-blue-neon {
  box-shadow: 0 0 0 2px #3b82f6 inset;
}
.border-purple-neon {
  box-shadow: 0 0 0 2px #a855f7 inset;
}

.shadow-glow-blue {
  box-shadow: 0 0 15px rgba(59, 130, 246, 0.6);
}
.shadow-glow-purple {
  box-shadow: 0 0 15px rgba(168, 85, 247, 0.6);
}
.shadow-glow-orange {
  box-shadow: 0 0 15px rgba(251, 146, 60, 0.6);
}

.wifi-pulse {
  animation: pulse 2.5s infinite ease-out;
}
@keyframes pulse {
  0% {
    opacity: 0;
    transform: scale(0.5);
  }
  50% {
    opacity: 1;
    transform: scale(1);
  }
  100% {
    opacity: 0;
    transform: scale(1.5);
  }
}

/* COMPARISON TABLE */
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.75rem;
}
.text-sm {
  font-size: 0.9rem;
}
.bg-dark-soft {
  background: rgba(0, 0, 0, 0.5);
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
