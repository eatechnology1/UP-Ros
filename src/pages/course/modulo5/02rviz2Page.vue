<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-orange-4 text-weight-bold q-mb-sm">
          MÓDULO 5.2: VISUALIZACIÓN
        </div>

        <h1 class="hero-title">RViz2: Los Ojos del <span class="text-white">Robot</span></h1>

        <TextBlock>
          Los robots "piensan" en números y matrices. Para que nosotros los entendamos, necesitamos
          convertir esos números en formas, líneas y colores.
          <strong>RViz (ROS Visualization)</strong> no es un simulador; es una ventana a la mente
          del robot. Si el robot cree que hay una pared delante, RViz te mostrará esa pared, exista
          o no en la realidad.
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA DE RVIZ (Interactivo) -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Anatomía de la Interfaz</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-4">
          <TextBlock>
            La interfaz de RViz puede intimidar al principio. Se divide en tres zonas críticas que
            debes dominar para no perderte.
          </TextBlock>

          <div class="q-mt-md">
            <div class="info-row q-mb-md">
              <q-badge color="orange-9" class="q-pa-xs q-mr-sm text-weight-bold shadow-1"
                >1. Displays</q-badge
              >
              <span class="text-grey-4 text-body2"
                >Panel izquierdo. Activa capas para ver qué sucede.
                <strong>¡Prueba los checkboxes!</strong></span
              >
            </div>
            <div class="info-row q-mb-md">
              <q-badge color="blue-9" class="q-pa-xs q-mr-sm text-weight-bold shadow-1"
                >2. Global Options</q-badge
              >
              <span class="text-grey-4 text-body2"
                >Donde defines el <strong>Fixed Frame</strong>. Si esto está mal, nada se ve.</span
              >
            </div>
            <div class="info-row">
              <q-badge color="green-9" class="q-pa-xs q-mr-sm text-weight-bold shadow-1"
                >3. Tools</q-badge
              >
              <span class="text-grey-4 text-body2"
                >Barra superior. Para enviar objetivos de navegación o medir.</span
              >
            </div>
          </div>
        </div>

        <div class="col-12 col-md-8">
          <!-- MOCKUP RVIZ INTERFACE -->
          <div
            class="rviz-mockup bg-grey-9 rounded-borders overflow-hidden shadow-2 relative-position border-light"
          >
            <!-- Top Bar -->
            <div class="bg-grey-10 q-pa-xs row items-center border-bottom-dark">
              <div class="q-px-sm text-grey-5 text-xs cursor-pointer hover-text-white">File</div>
              <div class="q-px-sm text-grey-5 text-xs cursor-pointer hover-text-white">Panels</div>
              <div class="q-px-sm text-grey-5 text-xs cursor-pointer hover-text-white">Help</div>
              <q-space />
              <div class="row q-gutter-x-xs">
                <q-btn flat dense icon="gps_fixed" color="green-4" size="sm" class="bg-grey-8" />
                <q-btn flat dense icon="near_me" color="pink-4" size="sm" class="bg-grey-8" />
              </div>
            </div>

            <div class="row" style="height: 350px">
              <!-- Left Panel (Displays) -->
              <div class="col-4 bg-grey-10 border-right-dark q-pa-sm column scroll">
                <div class="text-weight-bold text-orange-4 text-xs q-mb-sm">Displays</div>

                <!-- Fixed Frame Warning -->
                <div class="bg-slate-800 q-pa-xs rounded q-mb-sm border-l-blue shadow-1">
                  <div class="text-xxs text-grey-5">Global Options</div>
                  <div class="row items-center no-wrap">
                    <span class="text-xxs text-grey-4 q-mr-xs">Fixed Frame:</span>
                    <span class="text-xxs text-blue-3 font-mono">map</span>
                  </div>
                </div>

                <!-- Interactive Layers -->
                <div class="layer-item row items-center q-mb-xs hover-bg-slate">
                  <q-checkbox v-model="chkGrid" dense size="xs" color="grey" dark />
                  <span class="text-xxs text-white q-ml-xs">Grid</span>
                </div>
                <div class="layer-item row items-center q-mb-xs hover-bg-slate">
                  <q-checkbox v-model="chkRobot" dense size="xs" color="orange" dark />
                  <span class="text-xxs text-white q-ml-xs">RobotModel</span>
                </div>
                <div
                  class="layer-item row items-center q-mb-xs bg-red-9-soft rounded hover-bg-red-soft"
                >
                  <q-checkbox v-model="chkLaser" dense size="xs" color="red" dark />
                  <span class="text-xxs text-white q-ml-xs">LaserScan</span>
                  <q-icon name="warning" color="red" size="xs" class="q-ml-auto" />
                </div>

                <q-btn
                  outline
                  size="sm"
                  label="Add"
                  color="grey-6"
                  class="full-width q-mt-md text-capitalize"
                />
              </div>

              <!-- 3D Viewport -->
              <div class="col-8 bg-black relative-position flex flex-center overflow-hidden">
                <!-- Grid Background (Controlled by v-if) -->
                <transition name="fade">
                  <div v-if="chkGrid" class="grid-bg absolute-full opacity-20"></div>
                </transition>

                <!-- Robot Icon -->
                <transition name="scale">
                  <div v-if="chkRobot" class="robot-icon absolute-center text-center z-top">
                    <q-icon name="smart_toy" color="orange" size="lg" />
                    <div
                      class="text-xxs text-orange-4 font-mono bg-black q-px-xs rounded opacity-80"
                    >
                      base_link
                    </div>
                  </div>
                </transition>

                <!-- Laser Scan (Animated) -->
                <transition name="fade">
                  <div v-if="chkLaser" class="laser-scan absolute-center"></div>
                </transition>

                <!-- Error Message Overlay (Conditional logic: if Laser is ON but Robot is OFF -> TF Error simulation) -->
                <transition name="slide-up">
                  <div v-if="chkLaser && !chkRobot" class="absolute-bottom q-pa-sm z-max">
                    <div
                      class="bg-red-9 text-white text-xxs q-pa-xs rounded text-center border-red shadow-up-2 blink-warning"
                    >
                      ⚠️ No transform from [laser] to [map]
                    </div>
                  </div>
                </transition>
              </div>
            </div>
          </div>
          <div class="text-caption text-center text-grey-6 q-mt-sm font-italic">
            Simulación interactiva: Desactiva 'RobotModel' para simular un error de TF.
          </div>
        </div>
      </div>
    </div>

    <!-- 3. DISPLAYS COMUNES -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Displays Esenciales</SectionTitle>
      <TextBlock>
        Para visualizar datos, debes añadir el "Display" correcto. Es como elegir las gafas
        adecuadas para ver un tipo de luz.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <!-- CARD 1: RobotModel -->
        <div class="col-12 col-md-4">
          <div class="custom-card bg-slate-800 border-orange shadow-2 full-height transition-hover">
            <div class="card-header q-pa-md border-bottom-dark row items-center bg-slate-900">
              <div class="icon-box bg-orange-9 q-mr-md shadow-1">
                <q-icon name="smart_toy" color="white" size="sm" />
              </div>
              <div class="text-subtitle2 text-white">RobotModel</div>
            </div>
            <div class="q-pa-md text-xs text-grey-4">
              Lee el archivo <strong>URDF</strong> del robot y dibuja su cuerpo en 3D.
              <div class="q-mt-sm text-yellow-2 bg-black q-pa-sm rounded font-italic border-light">
                Requiere: <span class="text-orange-3">/robot_description</span>
              </div>
            </div>
          </div>
        </div>

        <!-- CARD 2: LaserScan -->
        <div class="col-12 col-md-4">
          <div class="custom-card bg-slate-800 border-red shadow-2 full-height transition-hover">
            <div class="card-header q-pa-md border-bottom-dark row items-center bg-slate-900">
              <div class="icon-box bg-red-9 q-mr-md shadow-1">
                <q-icon name="wifi_tethering" color="white" size="sm" />
              </div>
              <div class="text-subtitle2 text-white">LaserScan</div>
            </div>
            <div class="q-pa-md text-xs text-grey-4">
              Muestra líneas o puntos donde el sensor detecta obstáculos.
              <div class="q-mt-sm text-red-2 bg-black q-pa-sm rounded font-italic border-light">
                Requiere: Tópico <span class="text-red-3">sensor_msgs/LaserScan</span>
              </div>
            </div>
          </div>
        </div>

        <!-- CARD 3: TF -->
        <div class="col-12 col-md-4">
          <div class="custom-card bg-slate-800 border-purple shadow-2 full-height transition-hover">
            <div class="card-header q-pa-md border-bottom-dark row items-center bg-slate-900">
              <div class="icon-box bg-purple-9 q-mr-md shadow-1">
                <q-icon name="account_tree" color="white" size="sm" />
              </div>
              <div class="text-subtitle2 text-white">TF (Transform)</div>
            </div>
            <div class="q-pa-md text-xs text-grey-4">
              Dibuja los sistemas de coordenadas (ejes XYZ) de cada articulación.
              <div class="q-mt-sm text-purple-2 bg-black q-pa-sm rounded font-italic border-light">
                Vital para depurar errores de posición.
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. EL CONCEPTO DE FIXED FRAME -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. El Problema del "Fixed Frame"</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Este es el error #1 de los principiantes. <br /><br />
            RViz necesita un <strong>punto de referencia inmóvil</strong> para dibujar el mundo.
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li class="row items-start">
              <q-icon name="public" color="blue-4" size="xs" class="q-mt-xs q-mr-sm" />
              <div><strong>map:</strong> Úsalo si tienes un mapa estático (navegación).</div>
            </li>
            <li class="row items-start">
              <q-icon name="place" color="green-4" size="xs" class="q-mt-xs q-mr-sm" />
              <div><strong>odom:</strong> Úsalo si el robot se mueve pero no tienes mapa.</div>
            </li>
            <li class="row items-start">
              <q-icon name="smart_toy" color="orange-4" size="xs" class="q-mt-xs q-mr-sm" />
              <div><strong>base_link:</strong> Úsalo si quieres vista en primera persona.</div>
            </li>
          </ul>
        </template>
        <template #right>
          <AlertBlock type="warning" title="¿Pantalla negra o error de TF?">
            Si ves el mensaje
            <span class="text-weight-bold text-black bg-warning q-px-xs rounded"
              >No transform from [X] to [map]</span
            >, significa que has elegido "map" como Frame Fijo, pero tu sistema TF no sabe dónde
            está el robot respecto al mapa. <br /><br />
            <strong>Solución rápida:</strong> Cambia el Fixed Frame a <code>base_link</code> o
            <code>odom</code>.
          </AlertBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. GUARDAR CONFIGURACIÓN -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. Automatización (.rviz)</SectionTitle>
      <TextBlock>
        Configurar los displays cada vez que abres RViz es una pérdida de tiempo. Guarda tu setup en
        un archivo <code>.rviz</code> y cárgalo automáticamente.
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="code-label bash">💻 Terminal (Guardar)</div>
          <CodeBlock
            title="Guardar manualmente"
            lang="bash"
            content="# En RViz: File -> Save Config As...
# Guárdalo en tu paquete:
~/ros2_ws/src/my_robot/rviz/my_config.rviz"
            :copyable="false"
          />
        </div>
        <div class="col-12 col-md-6">
          <div class="code-label python">🚀 Launch File (Cargar)</div>
          <CodeBlock
            title="launch_rviz.py"
            lang="python"
            content="Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', [os.path.join(pkg_path, 'rviz', 'my_config.rviz')]]
)"
            :copyable="true"
          />
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { ref } from 'vue';
import TextBlock from 'components/content/TextBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';

// Reactive state for the mock interface checkboxes
const chkGrid = ref(true);
const chkRobot = ref(true);
const chkLaser = ref(true);
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
    radial-gradient(circle at center, rgba(251, 146, 60, 0.15), transparent 60%),
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

/* RVIZ MOCKUP & ANIMATIONS */
.rviz-mockup {
  border: 1px solid #334155;
}
.grid-bg {
  background-size: 40px 40px;
  background-image:
    linear-gradient(to right, rgba(255, 255, 255, 0.1) 1px, transparent 1px),
    linear-gradient(to bottom, rgba(255, 255, 255, 0.1) 1px, transparent 1px);
}
.laser-scan {
  width: 250px;
  height: 250px;
  border-radius: 50%;
  border: 2px dashed rgba(239, 68, 68, 0.6);
  animation: radarSpin 4s linear infinite;
  box-shadow: 0 0 30px rgba(239, 68, 68, 0.1) inset;
}
@keyframes radarSpin {
  from {
    transform: translate(-50%, -50%) rotate(0deg);
  }
  to {
    transform: translate(-50%, -50%) rotate(360deg);
  }
}

/* VUE TRANSITIONS */
.fade-enter-active,
.fade-leave-active {
  transition: opacity 0.3s;
}
.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}

.scale-enter-active,
.scale-leave-active {
  transition:
    transform 0.3s,
    opacity 0.3s;
}
.scale-enter-from,
.scale-leave-to {
  transform: translate(-50%, -50%) scale(0);
  opacity: 0;
}

.slide-up-enter-active,
.slide-up-leave-active {
  transition:
    transform 0.3s,
    opacity 0.3s;
}
.slide-up-enter-from,
.slide-up-leave-to {
  transform: translateY(20px);
  opacity: 0;
}

.blink-warning {
  animation: blink 1s infinite alternate;
}
@keyframes blink {
  from {
    opacity: 1;
  }
  to {
    opacity: 0.7;
  }
}

/* INTERFACE STYLING */
.bg-red-9-soft {
  background: rgba(127, 29, 29, 0.3);
}
.hover-bg-red-soft:hover {
  background: rgba(127, 29, 29, 0.5);
}
.hover-bg-slate:hover {
  background: rgba(255, 255, 255, 0.05);
}
.hover-text-white:hover {
  color: white !important;
}

.border-l-blue {
  border-left: 3px solid #3b82f6;
}
.border-right-dark {
  border-right: 1px solid rgba(255, 255, 255, 0.1);
}
.z-top {
  z-index: 5;
}
.z-max {
  z-index: 10;
}

/* CUSTOM CARDS */
.custom-card {
  border-left: 4px solid;
  border-radius: 12px;
}
.border-orange {
  border-color: #f97316;
}
.border-red {
  border-color: #ef4444;
}
.border-purple {
  border-color: #a855f7;
}
.icon-box {
  width: 36px;
  height: 36px;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
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
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 12px;
  font-size: 1rem;
}
.opacity-20 {
  opacity: 0.2;
}
.opacity-80 {
  opacity: 0.8;
}
.transition-hover {
  transition: transform 0.2s;
}
.transition-hover:hover {
  transform: translateY(-3px);
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
