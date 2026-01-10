<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-green-4 text-weight-bold q-mb-sm">
          MÓDULO 4.3: EL FLUJO DE DATOS
        </div>

        <h1 class="hero-title">Tópicos <span class="text-white">(Topics)</span></h1>

        <TextBlock>
          Los nodos necesitan compartir información constantemente (ej: video a 30fps, datos de
          láser). Los <strong>Tópicos</strong> son tuberías unidireccionales de datos. Funcionan
          como una transmisión de radio: un nodo "Publica" información en un canal específico, y
          cualquier nodo interesado se "Suscribe" para escuchar.
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA PUB/SUB -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Publicador y Suscriptor</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            El patrón es <strong>Desacoplado</strong>. El Publicador (Cámara) no sabe quién es el
            Suscriptor (Pantalla). Ni siquiera sabe si existe. Solo lanza mensajes al "éter" (DDS)
            bajo un nombre de tema.
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>📢 <strong>1-a-N:</strong> Un publicador puede tener muchos suscriptores.</li>
            <li>
              💬 <strong>Tipado Fuerte:</strong> Todos deben hablar el mismo idioma (Tipo de
              Mensaje).
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL ANIMATION PUB/SUB -->
          <div
            class="tool-card topic-viz q-pa-lg bg-black relative-position overflow-hidden shadow-2"
          >
            <div
              class="row items-center justify-between relative-position full-height"
              style="z-index: 2"
            >
              <!-- PUBLISHER (LEFT) -->
              <div class="column items-center z-top">
                <div class="node-box bg-blue-9 shadow-blue border-light transition-hover">
                  <q-icon name="videocam" color="white" size="2rem" />
                </div>
                <div class="text-weight-bold text-blue-4 q-mt-sm font-mono text-xs">CamNode</div>
                <div class="badge-pill bg-blue-9-soft text-blue-2 text-xxs border-blue-dim">
                  PUB
                </div>
              </div>

              <!-- PIPE (MIDDLE) -->
              <div class="col relative-position text-center q-px-md">
                <!-- The Physical Pipe -->
                <div class="pipe-line bg-grey-8"></div>
                <div class="pipe-glow absolute-center"></div>

                <!-- MOVING MESSAGES -->
                <div class="msg-packet absolute" style="animation-delay: 0s"></div>
                <div class="msg-packet absolute" style="animation-delay: 1.2s"></div>
                <div class="msg-packet absolute" style="animation-delay: 2.4s"></div>

                <!-- Topic Label -->
                <div
                  class="topic-label bg-slate-900 q-px-sm q-py-xs rounded-borders text-green-4 font-mono text-xs inline-block relative-position shadow-1 border-green-dim"
                  style="z-index: 3; top: -25px"
                >
                  /image_raw
                </div>
              </div>

              <!-- SUBSCRIBERS (RIGHT) -->
              <div class="column q-gutter-lg z-top">
                <!-- Sub 1 -->
                <div class="row items-center">
                  <div
                    class="badge-pill bg-purple-9-soft text-purple-2 text-xxs q-mr-sm border-purple-dim"
                  >
                    SUB
                  </div>
                  <div class="column items-center">
                    <div
                      class="node-box bg-purple-9 shadow-purple scale-sm border-light transition-hover"
                    >
                      <q-icon name="monitor" color="white" size="1.2rem" />
                    </div>
                    <div class="text-caption text-purple-4 q-mt-xs font-mono text-xxs">GUI</div>
                  </div>
                </div>

                <!-- Sub 2 -->
                <div class="row items-center">
                  <div
                    class="badge-pill bg-orange-9-soft text-orange-2 text-xxs q-mr-sm border-orange-dim"
                  >
                    SUB
                  </div>
                  <div class="column items-center">
                    <div
                      class="node-box bg-orange-9 shadow-orange scale-sm border-light transition-hover"
                    >
                      <q-icon name="save" color="white" size="1.2rem" />
                    </div>
                    <div class="text-caption text-orange-4 q-mt-xs font-mono text-xxs">Log</div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. MENSAJES (EL IDIOMA) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Idioma (Interfaces)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Si la cámara envía una imagen, pero el suscriptor espera un texto, el sistema falla. Por
            eso existen las <strong>Interfaces (.msg)</strong>. Son contratos estrictos.
            <br /><br />
            Ejemplo estándar: <code>geometry_msgs/Twist</code>. Se usa para mover robots. Define
            velocidad lineal (x,y,z) y angular (x,y,z).
          </TextBlock>
        </template>

        <template #right>
          <div
            class="tool-card msg-card bg-slate-900 q-pa-none border-green shadow-2 overflow-hidden"
          >
            <div
              class="row justify-between items-center q-px-md q-py-sm bg-black border-bottom-dark"
            >
              <span class="text-grey-5 font-mono text-xs">geometry_msgs/msg/Twist</span>
              <q-icon name="description" color="green-4" size="xs" />
            </div>

            <div class="code-structure font-mono text-sm q-pa-md bg-slate-800">
              <!-- Block 1 -->
              <div class="row items-center q-mb-xs">
                <span class="text-blue-4 q-mr-md text-weight-bold">Vector3</span>
                <span class="text-white">linear</span>
              </div>
              <div class="q-pl-lg text-grey-5 text-xs q-mb-sm border-left-grey">
                <div>float64 x</div>
                <div>float64 y</div>
                <div>float64 z</div>
              </div>

              <!-- Block 2 -->
              <div class="row items-center q-mb-xs">
                <span class="text-blue-4 q-mr-md text-weight-bold">Vector3</span>
                <span class="text-white">angular</span>
              </div>
              <div class="q-pl-lg text-grey-5 text-xs border-left-grey">
                <div>float64 x</div>
                <div>float64 y</div>
                <div>float64 z</div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. HERRAMIENTAS CLI (RQT Y TERMINAL) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Escuchando la Radio (CLI Tools)</SectionTitle>
      <TextBlock>
        Como humano, no puedes ver los datos viajar por el cable. Necesitas herramientas de
        espionaje.
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-sm">
        <!-- TOOL 1: LIST -->
        <div class="col-12 col-md-6">
          <div class="tool-card cli-card bg-black q-pa-md shadow-2 cursor-pointer group">
            <div class="window-dots row q-mb-sm">
              <div class="dot bg-red-5 q-mr-xs"></div>
              <div class="dot bg-yellow-5 q-mr-xs"></div>
              <div class="dot bg-green-5"></div>
            </div>
            <div class="text-green-4 font-mono text-sm border-bottom-dark q-pb-xs">
              $ ros2 topic list
            </div>
            <div
              class="text-grey-4 font-mono text-xs q-mt-sm leading-relaxed opacity-80 group-hover:opacity-100 transition-opacity"
            >
              /cmd_vel<br />
              /camera/image_raw<br />
              /rosout<br />
              /tf
            </div>
            <div class="text-caption text-grey-6 q-mt-md text-right font-italic">
              "¿Qué canales están transmitiendo?"
            </div>
          </div>
        </div>

        <!-- TOOL 2: ECHO -->
        <div class="col-12 col-md-6">
          <div
            class="tool-card cli-card bg-black q-pa-md border-green-glow shadow-glow-green cursor-pointer group"
          >
            <div class="window-dots row q-mb-sm">
              <div class="dot bg-red-5 q-mr-xs"></div>
              <div class="dot bg-yellow-5 q-mr-xs"></div>
              <div class="dot bg-green-5"></div>
            </div>
            <div class="text-green-4 font-mono text-sm border-bottom-dark q-pb-xs">
              $ ros2 topic echo /cmd_vel
            </div>
            <div class="text-white font-mono text-xs q-mt-sm leading-relaxed">
              linear:<br />
              &nbsp;&nbsp;x: <span class="text-orange-4">0.5</span><br />
              &nbsp;&nbsp;y: 0.0<br />
              angular:<br />
              &nbsp;&nbsp;z: <span class="text-orange-4">-1.2</span>
              <span class="blinking-cursor">_</span>
            </div>
            <div class="text-caption text-green-3 q-mt-md text-right font-italic">
              "¡Interceptando datos en vivo!"
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. RQT GRAPH (EL MAPA) -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <div class="bg-slate-800 q-pa-lg rounded-borders border-accent text-center shadow-2">
        <div class="row justify-center items-center q-mb-lg">
          <div class="bg-slate-700 q-pa-sm rounded-borders q-mr-md">
            <q-icon name="hub" size="md" color="white" />
          </div>
          <div class="text-h6 text-white">Visualización Total: RQT Graph</div>
        </div>

        <div
          class="graph-mockup bg-white q-pa-md rounded-borders relative-position overflow-hidden shadow-inner"
          style="height: 180px"
        >
          <!-- BACKGROUND GRID -->
          <div
            class="absolute-full"
            style="
              background-image: radial-gradient(#ccc 1px, transparent 1px);
              background-size: 10px 10px;
              opacity: 0.5;
            "
          ></div>

          <!-- FAKE GRAPH NODES -->
          <div
            class="absolute bg-blue-1 border-blue-dark q-px-md q-py-sm text-xs text-black rounded shadow-1 font-mono text-weight-bold"
            style="top: 40%; left: 10%; z-index: 2"
          >
            /teleop_key
          </div>

          <div
            class="absolute bg-green-1 border-green-dark q-px-md q-py-sm text-xs text-black rounded shadow-1 font-mono text-weight-bold"
            style="top: 40%; right: 10%; z-index: 2"
          >
            /turtlesim
          </div>

          <!-- ARROW -->
          <div
            class="absolute bg-grey-2 border-grey-4 q-px-sm text-xxs text-black rounded font-mono"
            style="top: 32%; left: 50%; transform: translateX(-50%); z-index: 2"
          >
            /cmd_vel
          </div>

          <svg class="absolute-full" style="pointer-events: none; z-index: 1">
            <line
              x1="22%"
              y1="50%"
              x2="78%"
              y2="50%"
              stroke="#333"
              stroke-width="2"
              marker-end="url(#arrowhead)"
            />
            <defs>
              <marker
                id="arrowhead"
                markerWidth="10"
                markerHeight="7"
                refX="9"
                refY="3.5"
                orient="auto"
              >
                <polygon points="0 0, 10 3.5, 0 7" fill="#333" />
              </marker>
            </defs>
          </svg>
        </div>

        <p class="text-grey-4 q-mt-lg text-subtitle2">
          Ejecuta <code class="bg-black q-px-xs rounded text-pink-4">rqt_graph</code> para ver quién
          habla con quién en un diagrama automático.
        </p>
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
    radial-gradient(circle at center, rgba(74, 222, 128, 0.15), transparent 60%),
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

/* TOPIC VIZ */
.topic-viz {
  border-top: 4px solid #4ade80;
  height: 300px;
}
.full-height {
  height: 100%;
}
.z-top {
  z-index: 5;
}

.node-box {
  width: 70px;
  height: 70px;
  border-radius: 14px;
  display: flex;
  align-items: center;
  justify-content: center;
  position: relative;
}
.scale-sm {
  width: 50px;
  height: 50px;
  border-radius: 10px;
}

.pipe-line {
  position: absolute;
  top: 50%;
  left: 0;
  width: 100%;
  height: 6px;
  transform: translateY(-50%);
  z-index: 1;
  border-radius: 3px;
}
.pipe-glow {
  width: 100%;
  height: 2px;
  background: rgba(74, 222, 128, 0.2);
  box-shadow: 0 0 10px rgba(74, 222, 128, 0.2);
}

.msg-packet {
  width: 12px;
  height: 12px;
  background: #4ade80;
  border-radius: 50%;
  top: 50%;
  margin-top: -6px;
  box-shadow:
    0 0 15px #4ade80,
    0 0 5px #fff;
  animation: travel 3.6s linear infinite;
  opacity: 0;
}

@keyframes travel {
  0% {
    left: 5%;
    opacity: 0;
    transform: scale(0.5);
  }
  10% {
    opacity: 1;
    transform: scale(1);
  }
  90% {
    opacity: 1;
    transform: scale(1);
  }
  100% {
    left: 85%;
    opacity: 0;
    transform: scale(0.5);
  }
}

.badge-pill {
  padding: 3px 6px;
  border-radius: 4px;
  font-weight: bold;
  font-family: 'Fira Code', monospace;
}
.bg-blue-9-soft {
  background: rgba(30, 58, 138, 0.5);
}
.bg-purple-9-soft {
  background: rgba(88, 28, 135, 0.5);
}
.bg-orange-9-soft {
  background: rgba(124, 45, 18, 0.5);
}

.border-blue-dim {
  border: 1px solid rgba(59, 130, 246, 0.3);
}
.border-purple-dim {
  border: 1px solid rgba(168, 85, 247, 0.3);
}
.border-orange-dim {
  border: 1px solid rgba(251, 146, 60, 0.3);
}
.border-green-dim {
  border: 1px solid rgba(74, 222, 128, 0.3);
}

.shadow-blue {
  box-shadow: 0 0 20px rgba(59, 130, 246, 0.4);
}
.shadow-purple {
  box-shadow: 0 0 15px rgba(168, 85, 247, 0.4);
}
.shadow-orange {
  box-shadow: 0 0 15px rgba(251, 146, 60, 0.4);
}

.transition-hover {
  transition: transform 0.2s;
}
.transition-hover:hover {
  transform: scale(1.05);
}

/* MSG CARD */
.border-green {
  border-left: 4px solid #4ade80;
}
.border-left-grey {
  border-left: 2px solid rgba(255, 255, 255, 0.1);
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.bg-slate-800 {
  background: #1e293b;
}
.bg-slate-900 {
  background: #0f172a;
}

/* CLI CARD */
.cli-card {
  border: 1px solid #333;
  transition: transform 0.3s;
}
.cli-card:hover {
  transform: translateY(-5px);
  border-color: #4ade80;
}
.border-green-glow {
  border: 1px solid #4ade80;
}
.shadow-glow-green {
  box-shadow: 0 0 20px rgba(74, 222, 128, 0.15);
}
.window-dots .dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}
.leading-relaxed {
  line-height: 1.6;
}
.blinking-cursor {
  animation: blink 1s step-end infinite;
}
@keyframes blink {
  50% {
    opacity: 0;
  }
}

/* GRAPH MOCKUP */
.border-blue-dark {
  border: 1px solid #1d4ed8;
}
.border-green-dark {
  border: 1px solid #15803d;
}
.text-xxs {
  font-size: 0.65rem;
}
.shadow-inner {
  box-shadow: inset 0 0 20px rgba(0, 0, 0, 0.1);
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.text-sm {
  font-size: 0.9rem;
}
.bg-slate-700 {
  background: #334155;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
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
