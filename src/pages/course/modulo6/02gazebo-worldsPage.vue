<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-orange-4 text-weight-bold q-mb-sm">
          MÓDULO 6.2: SIMULACIÓN FÍSICA
        </div>
        <h1 class="hero-title">Gazebo: Bienvenido a <span class="text-white">La Matrix</span></h1>
        <TextBlock>
          Tu robot no sabe si existe en el mundo real o en una simulación. Solo procesa datos.
          <strong>Gazebo</strong> genera esa "alucinación consensuada": calcula gravedad, fricción,
          inercia y luz para engañar a tu robot y permitirte fallar sin costosos desastres de
          hardware.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. La Pregunta del Millón: ¿Gazebo o RViz?</SectionTitle>

      <div class="row q-col-gutter-lg q-mt-md items-stretch">
        <div class="col-12 col-md-5">
          <TextBlock>
            Confundirlos es el error #1.
            <br /><br />
            👁️ <strong>RViz (Los Ojos):</strong> Muestra lo que el robot <em>cree</em> ver
            (Sensores). Si el sensor falla, RViz no muestra nada.<br /><br />
            🌍 <strong>Gazebo (La Verdad):</strong> Muestra la realidad objetiva. Si el robot choca
            con una pared invisible, es culpa de la física de Gazebo.
          </TextBlock>
        </div>

        <div class="col-12 col-md-7">
          <div
            class="tool-card bg-black relative-position overflow-hidden shadow-2"
            style="height: 280px"
          >
            <div
              class="absolute-left bg-slate-900 border-r-light"
              style="width: 50%; height: 100%; border-right: 2px dashed #475569"
            >
              <div class="absolute-top-left q-pa-sm text-blue-4 font-mono text-xs">
                MODE: RVIZ (DATA)
              </div>
              <div class="absolute-center wireframe-cube"></div>
              <div class="absolute-bottom text-center text-grey-6 text-xxs q-pb-sm">
                Wireframe / PointCloud
              </div>
            </div>

            <div class="absolute-right bg-slate-800" style="width: 50%; height: 100%">
              <div class="absolute-top-right q-pa-sm text-orange-4 font-mono text-xs">
                MODE: GAZEBO (PHYSICS)
              </div>
              <div class="absolute-center solid-cube shadow-orange-glow"></div>
              <div class="floor-grid absolute-bottom full-width"></div>
              <div class="absolute-bottom text-center text-grey-6 text-xxs q-pb-sm">
                Rendering / Collision
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Arquitectura: Cerebro y Cara</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Gazebo separa la lógica de la vista. Esto permite correr simulaciones en servidores sin
            pantalla (Headless).
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list q-mt-md">
            <li>⚙️ <strong>gzserver:</strong> El músculo. Calcula física y sensores.</li>
            <li>
              🖥️ <strong>gzclient:</strong> La piel. Dibuja la GUI. Puedes cerrarlo y la simulación
              sigue.
            </li>
          </ul>
        </template>

        <template #right>
          <div
            class="tool-card bg-slate-900 flex flex-center relative-position overflow-hidden border-green"
            style="height: 250px"
          >
            <div class="row items-center justify-around full-width q-px-md z-top">
              <div class="column items-center">
                <div
                  class="server-icon bg-slate-800 border-green shadow-green q-pa-md rounded-borders"
                >
                  <q-icon name="dns" size="3rem" color="green-4" />
                </div>
                <div class="text-caption text-green-4 q-mt-sm font-mono">gzserver</div>
              </div>

              <div class="connection-stream relative-position">
                <div class="packet p1"></div>
                <div class="packet p2"></div>
                <div class="packet p3"></div>
              </div>

              <div class="column items-center">
                <div class="client-icon bg-slate-800 border-blue q-pa-md rounded-borders">
                  <q-icon name="desktop_windows" size="3rem" color="blue-4" />
                </div>
                <div class="text-caption text-blue-4 q-mt-sm font-mono">gzclient</div>
              </div>
            </div>
            <div class="binary-bg absolute-full opacity-10"></div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. El Motor de Física (ODE)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Gazebo calcula colisiones y fuerzas 1000 veces por segundo. Tú defines las propiedades
            materiales en el URDF/SDF.
          </TextBlock>
          <div class="q-mt-md bg-slate-800 q-pa-sm rounded border-l-orange">
            <div class="text-orange-3 text-weight-bold">Parámetros Críticos</div>
            <ul class="q-pl-lg text-grey-4 text-xs q-mb-none">
              <li><strong>Fricción (mu):</strong> ¿Hielo o Caucho?</li>
              <li><strong>Restitución:</strong> ¿Pelota de tenis o bola de boliche?</li>
            </ul>
          </div>
        </template>
        <template #right>
          <div class="tool-card bg-grid relative-position overflow-hidden" style="height: 260px">
            <div class="absolute-top-right q-pa-sm text-xxs font-mono text-grey-5">
              PHYSICS_ENGINE: <span class="text-orange-4">ACTIVE</span>
            </div>

            <div class="ball-container absolute-center">
              <div class="ball bg-orange-5 shadow-orange-glow"></div>
              <div class="ball-shadow"></div>
            </div>

            <div
              class="floor-line absolute-bottom bg-grey-6 full-width"
              style="height: 2px; bottom: 40px"
            ></div>
            <div class="text-center absolute-bottom q-pb-xs text-xxs text-grey-6">
              Ground Plane (Static)
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. RTF: El Tiempo Relativo</SectionTitle>
      <AlertBlock type="warning" title="Lag Temporal">
        Si tu PC no aguanta los cálculos, Gazebo ralentiza el tiempo. 1 segundo real != 1 segundo
        simulado.
      </AlertBlock>

      <div class="row justify-center q-mt-md">
        <div
          class="rtf-dashboard bg-black border-grey shadow-2 q-pa-md rounded-borders relative-position overflow-hidden"
          style="width: 100%; max-width: 500px"
        >
          <div class="row items-end justify-between">
            <div class="column">
              <div class="text-xxs text-grey-5 font-mono">SIMULATION TIME</div>
              <div class="text-h4 text-white font-mono counter-anim">00:12:45</div>
            </div>
            <div class="column items-end">
              <div class="text-xxs text-grey-5 font-mono">REAL TIME FACTOR</div>
              <div class="text-h3 text-green-4 font-mono rtf-flicker">0.98</div>
            </div>
          </div>
          <div class="q-mt-md">
            <div class="row justify-between text-xxs text-grey-6 q-mb-xs">
              <span>CPU LOAD</span><span>PHYSICS THREAD</span>
            </div>
            <div class="load-bar-bg bg-grey-9 rounded-full overflow-hidden" style="height: 6px">
              <div class="load-bar-fill bg-orange-5"></div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. Plugins: Dando Vida a los Sensores</SectionTitle>
      <TextBlock>
        Un cubo en Gazebo es ciego. Para que sea un Lidar, necesitas un
        <strong>Plugin C++</strong> que simule los rayos.
      </TextBlock>

      <div
        class="tool-card bg-slate-900 q-mt-md relative-position overflow-hidden border-cyan shadow-cyan"
        style="height: 300px"
      >
        <div class="absolute-top-left q-pa-sm text-cyan-4 font-mono text-xs z-top">
          PLUGIN: libgazebo_ros_laser.so
        </div>

        <div class="absolute-center full-width full-height">
          <div
            class="lidar-device absolute-center bg-grey-8 rounded-borders z-top flex flex-center"
            style="width: 40px; height: 40px"
          >
            <div class="scanning-eye bg-cyan-4"></div>
          </div>

          <div
            class="obstacle-box bg-red-9 absolute border-red"
            style="top: 20%; right: 20%; width: 50px; height: 50px"
          >
            <div class="absolute-bottom text-center text-xxs text-white bg-black">Wall</div>
          </div>

          <div class="ray-container absolute-center">
            <div class="laser-beam"></div>
          </div>

          <div class="hit-point absolute" style="top: 25%; right: 25%">
            <div class="data-dot bg-red-5"></div>
            <div class="data-label text-red-4 text-xxs font-mono">/scan point</div>
          </div>
        </div>

        <div
          class="absolute-bottom bg-black q-px-md q-py-sm font-mono text-xxs text-green-4 opacity-80"
        >
          [INFO] [gzserver]: Publishing /scan data (range: 1.2m)
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>6. Spawning (Inyección)</SectionTitle>
      <TextBlock>Inyecta tu robot en el mundo sin reiniciar la simulación.</TextBlock>
      <CodeBlock
        lang="bash"
        code="ros2 run gazebo_ros spawn_entity.py \
  -topic robot_description \
  -entity my_bot \
  -x 0.0 -y 0.0 -z 0.1"
      />
      <div class="text-caption text-grey-5 q-mt-xs font-italic">
        Tip: Siempre spawnea un poco por encima del suelo (z=0.1) para evitar que el robot quede
        atrapado en el piso.
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
    radial-gradient(circle at center, rgba(249, 115, 22, 0.15), transparent 60%),
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
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.bg-grid {
  background-color: #0f172a;
  background-image:
    linear-gradient(rgba(255, 255, 255, 0.03) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255, 255, 255, 0.03) 1px, transparent 1px);
  background-size: 30px 30px;
}
.border-r-light {
  border-right: 1px solid rgba(255, 255, 255, 0.1);
}
.border-l-orange {
  border-left: 4px solid #f97316;
}
.border-green {
  border-color: #22c55e;
}
.border-blue {
  border-color: #3b82f6;
}
.border-red {
  border: 2px solid #ef4444;
}
.border-cyan {
  border: 1px solid #06b6d4;
}
.shadow-orange-glow {
  box-shadow: 0 0 20px rgba(249, 115, 22, 0.4);
}
.shadow-green {
  box-shadow: 0 0 15px rgba(34, 197, 94, 0.3);
}
.shadow-cyan {
  box-shadow: 0 0 15px rgba(6, 182, 212, 0.3);
}
.text-xxs {
  font-size: 0.7rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.z-top {
  z-index: 5;
}

/* 1. RVIZ VS GAZEBO */
.wireframe-cube {
  width: 60px;
  height: 60px;
  border: 1px solid #3b82f6;
  transform: rotate(45deg);
  background: transparent;
  box-shadow: inset 0 0 0 1px #3b82f6;
}
.solid-cube {
  width: 60px;
  height: 60px;
  background: #f97316;
  transform: rotate(45deg);
}
.floor-grid {
  height: 40px;
  background-image:
    linear-gradient(#475569 1px, transparent 1px),
    linear-gradient(90deg, #475569 1px, transparent 1px);
  background-size: 20px 20px;
  transform: perspective(100px) rotateX(45deg);
  opacity: 0.5;
}

/* 2. ARCHITECTURE */
.connection-stream {
  width: 100px;
  height: 10px;
  border-bottom: 2px dashed #475569;
}
.packet {
  width: 8px;
  height: 8px;
  background: #22c55e;
  border-radius: 50%;
  position: absolute;
  top: 1px;
  opacity: 0;
}
.p1 {
  animation: stream 2s infinite linear;
}
.p2 {
  animation: stream 2s infinite linear 0.6s;
}
.p3 {
  animation: stream 2s infinite linear 1.2s;
}
@keyframes stream {
  0% {
    left: 0;
    opacity: 1;
  }
  100% {
    left: 100%;
    opacity: 0;
  }
}

/* 3. PHYSICS BALL */
.ball-container {
  width: 100px;
  height: 200px;
  margin-top: -100px;
}
.ball {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  margin: 0 auto;
  animation: bounceBall 1.5s infinite cubic-bezier(0.28, 0.84, 0.42, 1);
}
.ball-shadow {
  width: 40px;
  height: 10px;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 50%;
  margin: 150px auto 0;
  animation: shadowScale 1.5s infinite cubic-bezier(0.28, 0.84, 0.42, 1);
}
@keyframes bounceBall {
  0% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(150px) scale(1.1, 0.9);
  }
  100% {
    transform: translateY(0);
  }
}
@keyframes shadowScale {
  0% {
    transform: scale(0.5);
    opacity: 0.2;
  }
  50% {
    transform: scale(1);
    opacity: 0.8;
  }
  100% {
    transform: scale(0.5);
    opacity: 0.2;
  }
}

/* 4. RTF DASHBOARD */
.rtf-flicker {
  animation: textFlicker 0.1s infinite alternate;
}
@keyframes textFlicker {
  from {
    opacity: 1;
  }
  to {
    opacity: 0.9;
  }
}
.load-bar-fill {
  width: 60%;
  height: 100%;
  animation: loadFluctuate 2s infinite ease-in-out;
}
@keyframes loadFluctuate {
  0% {
    width: 60%;
  }
  50% {
    width: 85%;
    background: #ef4444;
  }
  100% {
    width: 60%;
  }
}

/* 5. LIDAR VISUALIZER */
.lidar-device {
  border: 2px solid #334155;
}
.scanning-eye {
  width: 8px;
  height: 8px;
  border-radius: 50%;
}
.ray-container {
  width: 300px;
  height: 300px;
  pointer-events: none;
  animation: rayRotate 4s infinite linear;
}
.laser-beam {
  width: 50%;
  height: 2px;
  background: linear-gradient(90deg, transparent, #06b6d4);
  position: absolute;
  top: 50%;
  left: 0;
  transform-origin: 100% 50%;
}
@keyframes rayRotate {
  from {
    transform: rotate(0deg);
  }
  to {
    transform: rotate(360deg);
  }
}

/* Hit point visualization timed to sync with rotation hitting the box */
.hit-point {
  opacity: 0;
  animation: hitFlash 4s infinite linear;
}
@keyframes hitFlash {
  0%,
  75% {
    opacity: 0;
  }
  80% {
    opacity: 1;
    transform: scale(1.5);
  } /* Flash when beam hits top-right quadrant approx */
  85% {
    opacity: 0;
    transform: scale(1);
  }
  100% {
    opacity: 0;
  }
}
.data-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  margin-bottom: 5px;
  box-shadow: 0 0 10px #ef4444;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .tool-card {
    height: auto !important;
    min-height: 250px;
  }
}
</style>
