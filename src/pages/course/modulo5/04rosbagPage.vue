<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-red-4 text-weight-bold q-mb-sm">
          MÓDULO 5.4: VIAJE EN EL TIEMPO
        </div>

        <h1 class="hero-title">Rosbag2: La <span class="text-white">Caja Negra</span></h1>

        <TextBlock>
          Los robots fallan. Y cuando fallan en el mundo real, es difícil saber por qué. ¿Fue un
          error del sensor? ¿Un fallo en el código?
          <br /><br />
          <strong>Rosbag2</strong> te permite grabar todos los mensajes de los tópicos en un
          archivo. Luego, puedes "reproducir" esos datos en tu casa, engañando a tus nodos para que
          crean que el robot sigue funcionando.
        </TextBlock>
      </div>
    </section>

    <!-- 2. CONCEPTO RECORD / PLAY -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Grabar y Reproducir</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            El flujo de trabajo es simple pero poderoso.
            <br /><br />
            🔴 <strong>Record:</strong> Capturas los datos en vivo (cámaras, lidar, odometría) y los
            guardas en una base de datos (SQLite). <br /><br />
            ▶️ <strong>Play:</strong> Rosbag inyecta esos mensajes de nuevo en el sistema. Tus
            algoritmos de navegación no saben que es una grabación; procesan los datos como si
            fueran frescos.
          </TextBlock>

          <div class="q-mt-md">
            <AlertBlock type="warning" title="Espacio en Disco">
              Grabar video o nubes de puntos (Lidar) consume Gigabytes por minuto. ¡Vigila tu disco
              duro!
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <!-- VISUALIZATION: TAPE RECORDER METAPHOR -->
          <div
            class="tool-card bg-slate-900 q-pa-lg shadow-2 border-red relative-position overflow-hidden"
            style="height: 280px"
          >
            <!-- TAPE REELS ANIMATION -->
            <div class="absolute-center row q-gutter-x-xl z-top" style="top: 45%">
              <!-- Left Reel -->
              <div class="reel reel-spin">
                <div class="reel-inner bg-grey-9"></div>
                <div class="reel-holes">
                  <div class="hole bg-slate-900"></div>
                  <div class="hole bg-slate-900"></div>
                  <div class="hole bg-slate-900"></div>
                </div>
              </div>
              <!-- Right Reel -->
              <div class="reel reel-spin delay-spin">
                <div class="reel-inner bg-grey-9"></div>
                <div class="reel-holes">
                  <div class="hole bg-slate-900"></div>
                  <div class="hole bg-slate-900"></div>
                  <div class="hole bg-slate-900"></div>
                </div>
              </div>
            </div>

            <!-- TAPE LINE -->
            <div
              class="absolute-center bg-grey-8"
              style="width: 200px; height: 6px; top: 45%; z-index: 1"
            ></div>

            <!-- DATA PACKETS ON TAPE -->
            <div class="packet-stream absolute" style="top: 41%; left: 15%; width: 70%; z-index: 2">
              <div class="data-dot bg-blue-5"></div>
              <div class="data-dot bg-green-5 delay-1"></div>
              <div class="data-dot bg-orange-5 delay-2"></div>
            </div>

            <!-- CONTROLS UI -->
            <div
              class="absolute-bottom bg-black q-py-sm row justify-center q-gutter-x-lg border-top-dark"
            >
              <div class="control-btn bg-red-6 shadow-glow-red scale-hover">
                <q-icon name="fiber_manual_record" color="white" />
              </div>
              <div class="control-btn bg-green-6 scale-hover">
                <q-icon name="play_arrow" color="white" />
              </div>
              <div class="control-btn bg-grey-8 scale-hover">
                <q-icon name="pause" color="white" />
              </div>
            </div>

            <!-- LCD DISPLAY -->
            <div
              class="absolute-top-right q-ma-sm bg-black q-px-sm q-py-xs rounded border-red-dim row items-center"
            >
              <div class="text-red-5 font-mono text-xs blink q-mr-sm">REC ●</div>
              <div class="text-grey-5 font-mono text-xs">00:04:23</div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. COMANDOS CLI -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Comandos Esenciales</SectionTitle>
      <div class="row q-col-gutter-md">
        <!-- RECORD ALL -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-red h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-subtitle1 text-white text-weight-bold">Grabar Todo</div>
              <q-icon name="save" color="red-4" size="sm" />
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-xs q-mb-md">
                Captura todos los tópicos activos. Útil, pero genera archivos gigantes.
              </p>
              <CodeBlock lang="bash" content="$ ros2 bag record -a" :copyable="true" />
            </div>
          </div>
        </div>

        <!-- RECORD SPECIFIC -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-orange h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-subtitle1 text-white text-weight-bold">Grabar Específico</div>
              <q-icon name="filter_alt" color="orange-4" size="sm" />
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-xs q-mb-md">
                Captura solo lo que necesitas (ej: sensores) y ponle nombre al archivo.
              </p>
              <CodeBlock
                lang="bash"
                content="$ ros2 bag record -o test_1 /cmd_vel /odom"
                :copyable="true"
              />
            </div>
          </div>
        </div>

        <!-- INFO -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-blue h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-subtitle1 text-white text-weight-bold">Inspeccionar</div>
              <q-icon name="info" color="blue-4" size="sm" />
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-xs q-mb-md">
                Ver cuánto dura, cuánto pesa y qué tópicos hay dentro del bag.
              </p>
              <CodeBlock lang="bash" content="$ ros2 bag info test_1" :copyable="true" />
            </div>
          </div>
        </div>

        <!-- PLAY -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-green h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-subtitle1 text-white text-weight-bold">Reproducir</div>
              <q-icon name="play_circle" color="green-4" size="sm" />
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-xs q-mb-md">
                La opción <code>--loop</code> reproduce en bucle infinito.
              </p>
              <CodeBlock lang="bash" content="$ ros2 bag play test_1 --loop" :copyable="true" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. EL FORMATO MCAP (MODERNO) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Formatos: SQLite3 vs MCAP</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Por defecto, ROS 2 guarda los datos en una base de datos
            <strong>SQLite3</strong> (archivos `.db3`). Es estándar pero puede ser lento para
            escribir datos masivos. <br /><br />
            La industria se está moviendo hacia <strong>MCAP</strong> (Foxglove), un formato
            optimizado para robótica de alto rendimiento.
          </TextBlock>
          <div class="q-mt-sm">
            <div class="code-label bash">Instalar Plugin MCAP</div>
            <CodeBlock
              lang="bash"
              content="$ sudo apt install ros-jazzy-rosbag2-storage-mcap"
              :copyable="true"
            />
          </div>
        </template>

        <template #right>
          <div class="tool-card bg-slate-800 q-pa-lg border-left-purple shadow-2 rounded-borders">
            <div class="row items-center q-mb-md">
              <q-icon name="folder_open" color="purple-4" size="sm" class="q-mr-sm" />
              <div class="text-purple-4 text-subtitle1 text-weight-bold">
                Estructura de Carpetas
              </div>
            </div>

            <!-- Folder Tree Viz -->
            <div class="font-mono text-xs text-grey-4 bg-black q-pa-md rounded border-light">
              <div class="row items-center"><span class="text-blue-4">mi_grabacion/</span></div>
              <div class="row items-center q-pl-md q-mt-xs">
                <span class="text-grey-7">├──</span>
                <q-icon name="description" color="yellow-6" class="q-mx-sm" size="xs" />
                <span class="text-white">metadata.yaml</span>
              </div>
              <div class="row items-center q-pl-md q-mt-xs">
                <span class="text-grey-7">└──</span>
                <q-icon name="database" color="green-5" class="q-mx-sm" size="xs" />
                <span class="text-green-3">mi_grabacion_0.db3</span>
              </div>
            </div>

            <div class="q-mt-md row items-start bg-slate-900 q-pa-sm rounded border-light">
              <q-icon name="lightbulb" color="yellow-5" size="xs" class="q-mr-sm q-mt-xs" />
              <div class="text-xxs text-grey-5">
                Nota: Un "Bag" no es un archivo único, es una <strong>carpeta</strong> que contiene
                los datos fragmentados y la metadata.
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. CASO DE USO REAL (DEBUGGING) -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <div
        class="bg-gradient-dark q-pa-lg rounded-borders border-dashed-red text-center relative-position overflow-hidden"
      >
        <!-- Background Pattern -->
        <div class="absolute-full opacity-5 bg-pattern-dots"></div>

        <div class="relative-position z-top">
          <q-icon name="bug_report" size="md" color="red-4" class="q-mb-sm" />
          <div class="text-h6 text-white text-weight-bold">El Truco del Desarrollador Senior</div>
          <p class="text-grey-4 q-mb-md text-body2" style="max-width: 600px; margin: 1rem auto">
            No desarrolles tus algoritmos conectando el robot real cada vez.
            <br /><br />
            1. Graba 5 minutos de datos reales con el robot.<br />
            2. Siéntate en tu escritorio con un café.<br />
            3. Reproduce el bag en bucle mientras programas.<br />
            <br />
            <strong>¡Es como tener el robot dentro de tu laptop!</strong>
          </p>
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
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
    radial-gradient(circle at center, rgba(239, 68, 68, 0.15), transparent 60%),
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

/* TAPE RECORDER ANIMATION */
.tool-card {
  border-radius: 12px;
}
.border-red {
  border: 1px solid #ef4444;
}
.border-red-dim {
  border: 1px solid rgba(239, 68, 68, 0.3);
}
.shadow-glow-red {
  box-shadow: 0 0 15px rgba(239, 68, 68, 0.6);
}

.reel {
  width: 70px;
  height: 70px;
  border-radius: 50%;
  border: 6px solid #1e293b;
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #000;
}
.reel-inner {
  width: 25px;
  height: 25px;
  border-radius: 50%;
}
.reel-holes {
  position: absolute;
  width: 100%;
  height: 100%;
  animation: spinReel 4s linear infinite;
}
.hole {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  position: absolute;
}
.hole:nth-child(1) {
  top: 8px;
  left: 50%;
  transform: translateX(-50%);
}
.hole:nth-child(2) {
  bottom: 12px;
  left: 12px;
}
.hole:nth-child(3) {
  bottom: 12px;
  right: 12px;
}

@keyframes spinReel {
  from {
    transform: rotate(0deg);
  }
  to {
    transform: rotate(360deg);
  }
}
.delay-spin {
  animation-delay: -1s;
}

.packet-stream {
  height: 10px;
  pointer-events: none;
}
.data-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  position: absolute;
  animation: moveTape 2s linear infinite;
  box-shadow: 0 0 5px currentColor;
}
.delay-1 {
  animation-delay: 0.6s;
  left: 0;
}
.delay-2 {
  animation-delay: 1.2s;
  left: 0;
}

@keyframes moveTape {
  from {
    transform: translateX(0);
    opacity: 0;
  }
  10% {
    opacity: 1;
  }
  90% {
    opacity: 1;
  }
  to {
    transform: translateX(120px);
    opacity: 0;
  }
}

.control-btn {
  width: 45px;
  height: 45px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: transform 0.1s;
}
.scale-hover:hover {
  transform: scale(1.1);
}
.blink {
  animation: blinkRec 1s infinite;
}
@keyframes blinkRec {
  50% {
    opacity: 0;
  }
}

/* CLI CARDS */
.custom-card {
  border-radius: 12px;
  overflow: hidden;
}
.border-orange {
  border-left: 4px solid #f97316;
}
.border-blue {
  border-left: 4px solid #3b82f6;
}
.border-green {
  border-left: 4px solid #22c55e;
}
.border-red-card {
  border-left: 4px solid #ef4444;
}
.h-full {
  height: 100%;
}
.hover-lift {
  transition: transform 0.2s;
}
.hover-lift:hover {
  transform: translateY(-5px);
}

/* FOLDER VIZ */
.border-left-purple {
  border-left: 4px solid #a855f7;
}
.border-dashed-red {
  border: 2px dashed rgba(248, 113, 113, 0.4);
}
.bg-gradient-dark {
  background: linear-gradient(145deg, #0f172a, #1e293b);
}
.bg-pattern-dots {
  background-image: radial-gradient(#ffffff 1px, transparent 1px);
  background-size: 20px 20px;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xxs {
  font-size: 0.7rem;
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
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-dark {
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}
.z-top {
  z-index: 5;
}
.opacity-5 {
  opacity: 0.05;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
