<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-purple-4 text-weight-bold q-mb-sm">
          MÓDULO 5.5: GEOMETRÍA DEL ROBOT
        </div>

        <h1 class="hero-title">Debugging <span class="text-purple-5">TF2</span></h1>

        <TextBlock>
          Un robot no es un punto en el espacio; es una colección de partes (ruedas, sensores,
          chasis) conectadas entre sí. Para que el robot sepa que un obstáculo visto por la cámara
          (frente) está a 2 metros de las ruedas (atrás), necesita matemáticas.
          <br /><br />
          <strong>TF2</strong> es el bibliotecario que mantiene el registro de todas estas
          relaciones de coordenadas (Transforms) en el tiempo. Si el TF falla, el robot "se rompe"
          geométricamente.
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL ÁRBOL DE TRANSFORMACIONES (TF TREE) -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Árbol Sagrado (TF Tree)</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            La regla de oro de TF2:
            <strong
              >Todos los marcos de referencia (Frames) deben estar conectados en un solo
              árbol.</strong
            >
            <br /><br />
            No puede haber islas. No puede haber ciclos (hijo -> padre -> hijo).
            <br /><br />
            Si tu sensor Lidar está "flotando" desconectado del chasis (`base_link`), ROS no podrá
            usar sus datos para navegar, porque no sabe dónde está montado el sensor.
          </TextBlock>

          <div class="q-mt-md">
            <AlertBlock type="danger" title="El Error Clásico">
              <span class="font-mono text-xs">"Frame [laser] does not exist"</span>
              <br />
              Esto significa que el árbol está roto. La rama del láser se cayó.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <!-- VISUALIZATION: BROKEN VS CONNECTED TREE -->
          <div
            class="tool-card bg-slate-900 q-pa-lg shadow-2 border-purple relative-position overflow-hidden"
            style="height: 320px"
          >
            <div class="row full-height">
              <!-- BROKEN TREE -->
              <div class="col-6 border-right-grey relative-position q-pr-md">
                <div
                  class="text-center text-red-4 text-weight-bold q-mb-lg bg-red-9-soft rounded q-py-xs"
                >
                  ROTO ❌
                </div>

                <!-- Tree Diagram (CSS) -->
                <div class="column items-center q-gutter-y-lg relative-position">
                  <!-- Root -->
                  <div class="tf-node bg-grey-8 shadow-1">odom</div>
                  <!-- Link -->
                  <div class="link-line"></div>
                  <!-- Child -->
                  <div class="tf-node bg-blue-9 shadow-blue">base_link</div>
                </div>

                <!-- Disconnected Node -->
                <div class="absolute-bottom-right q-mb-xl q-mr-md column items-center">
                  <div class="tf-node bg-orange-9 shake-anim opacity-70">
                    lidar
                    <q-icon
                      name="link_off"
                      color="white"
                      class="absolute-top-right q-ma-xs"
                      size="xs"
                    />
                  </div>
                  <div class="text-caption text-red-3 text-center q-mt-sm font-italic text-xs">
                    "¿Dónde estoy?"
                  </div>
                </div>
              </div>

              <!-- FIXED TREE -->
              <div class="col-6 relative-position q-pl-md">
                <div
                  class="text-center text-green-4 text-weight-bold q-mb-lg bg-green-9-soft rounded q-py-xs"
                >
                  SANO ✅
                </div>

                <div class="column items-center q-gutter-y-lg relative-position">
                  <!-- Root -->
                  <div class="tf-node bg-grey-8 shadow-1">odom</div>
                  <!-- Link -->
                  <div class="link-line bg-green-5"></div>
                  <!-- Child -->
                  <div class="tf-node bg-blue-9 shadow-blue">base_link</div>

                  <!-- Branching Logic for Lidar -->
                  <div
                    class="absolute"
                    style="top: 100%; left: 50%; width: 2px; height: 20px; background: #4ade80"
                  ></div>
                  <div
                    class="absolute"
                    style="top: 120px; left: 50%; width: 40px; height: 2px; background: #4ade80"
                  ></div>
                  <div
                    class="absolute"
                    style="
                      top: 120px;
                      left: calc(50% + 40px);
                      width: 2px;
                      height: 20px;
                      background: #4ade80;
                    "
                  ></div>

                  <div
                    class="tf-node bg-orange-9 shadow-orange absolute"
                    style="top: 140px; left: 40px"
                  >
                    lidar
                  </div>
                </div>

                <div class="absolute-bottom text-center q-mb-md">
                  <div class="text-caption text-green-3 font-italic text-xs">
                    "Conectado a base_link"
                  </div>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. ESTÁTICO VS DINÁMICO -->
    <div class="section-group self-stretch">
      <SectionTitle>2. ¿Tornillos o Motores?</SectionTitle>
      <TextBlock>
        No todas las transformaciones son iguales. Entender la diferencia ahorra muchísima CPU.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <!-- STATIC -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-blue h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-h6 text-white">Static Transform</div>
              <div class="icon-box bg-blue-9">
                <q-icon name="push_pin" color="white" size="sm" />
              </div>
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-sm q-mb-md">
                Para cosas atornilladas que <strong>nunca se mueven</strong> (Ej: Lidar atornillado
                al chasis).
                <br />
                Se publica una sola vez ("Latched"). Muy eficiente.
              </p>
              <div class="code-label bash">Arreglo rápido (One-liner):</div>
              <CodeBlock
                lang="bash"
                content="ros2 run tf2_ros static_transform_publisher 0.1 0 0 0 0 0 base_link lidar_link"
                :copyable="true"
              />
            </div>
          </div>
        </div>

        <!-- DYNAMIC -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-orange h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-h6 text-white">Dynamic Transform</div>
              <div class="icon-box bg-orange-9">
                <q-icon name="motion_photos_on" color="white" size="sm" />
              </div>
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-sm q-mb-md">
                Para cosas que <strong>se mueven constantemente</strong> (Ej: Ruedas girando, brazo
                robótico, el robot en el mapa).
                <br />
                Se publica a alta frecuencia (10Hz - 100Hz).
              </p>
              <div class="bg-black q-pa-sm rounded font-mono text-xs text-orange-3 border-light">
                <q-icon name="info" size="xs" class="q-mr-xs" />
                Gestionado usualmente por `robot_state_publisher` o nodos de Odometría.
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. EL TIEMPO Y LA EXTRAPOLACIÓN -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. El Viaje en el Tiempo (Time Buffering)</SectionTitle>
      <TextBlock>
        TF2 no solo sabe dónde están las cosas AHORA. Guarda un historial (Buffer) de 10 segundos.
        Esto permite preguntar: <em>"¿Dónde estaba la cámara hace 2 segundos?"</em>.
      </TextBlock>

      <div class="q-mt-lg">
        <div
          class="tool-card bg-slate-800 q-pa-lg border-left-red shadow-2 rounded-borders relative-position overflow-hidden"
        >
          <!-- Subtle Warning BG -->
          <div
            class="absolute-right full-height bg-red-9 opacity-5"
            style="width: 100px; transform: skewX(-20deg)"
          ></div>

          <div class="row items-center q-mb-md">
            <q-icon name="warning" color="red-5" size="md" class="q-mr-sm" />
            <div class="text-red-4 text-h6 text-weight-bold">
              El Error que verás en tus pesadillas
            </div>
          </div>

          <div
            class="terminal-box bg-black q-pa-md rounded border-grey shadow-inset font-mono text-xs text-red-3 text-break q-mb-md"
          >
            [ERROR] [tf2]: Lookup would require extrapolation into the future.<br />
            Requested time 1630.500 but the latest data is at time 1630.400.
          </div>

          <div class="row q-col-gutter-md">
            <div class="col-12 col-md-8">
              <div class="text-grey-3 text-sm">
                <strong>Traducción:</strong> Estás pidiendo la posición del robot en el futuro (o en
                un tiempo que el buffer aún no tiene).
              </div>
            </div>
            <div class="col-12 col-md-4">
              <div class="bg-slate-900 q-pa-sm rounded">
                <div class="text-xs text-grey-5 q-mb-xs">Culpables Habituales:</div>
                <ul class="q-pl-md q-my-none text-xs text-grey-4">
                  <li>⏱️ Desincronización NTP</li>
                  <li>🐢 Lag de Wi-Fi</li>
                  <li>🐛 Sim Time (use_sim_time)</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. HERRAMIENTAS DE DIAGNÓSTICO -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. Kit Médico para TF2</SectionTitle>
      <TextBlock>
        No adivines. Usa estas herramientas para hacer una radiografía a tu árbol.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <!-- TOOL 1: VIEW FRAMES -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-purple h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-subtitle1 text-white">Radiografía (PDF)</div>
              <q-icon name="picture_as_pdf" color="purple-4" size="sm" />
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-xs q-mb-sm">
                Genera un diagrama visual de todo el árbol. Es la prueba definitiva de "islas".
              </p>
              <CodeBlock lang="bash" content="$ ros2 run tf2_tools view_frames" :copyable="true" />
              <div class="text-xxs text-grey-6 q-mt-xs text-right">Genera: frames.pdf</div>
            </div>
          </div>
        </div>

        <!-- TOOL 2: TF2 ECHO -->
        <div class="col-12 col-md-6">
          <div class="custom-card border-green h-full bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row justify-between items-center">
              <div class="text-subtitle1 text-white">Cinta Métrica</div>
              <q-icon name="straighten" color="green-4" size="sm" />
            </div>
            <div class="q-pa-md">
              <p class="text-grey-4 text-xs q-mb-sm">
                Te da los números exactos (traslación y rotación) entre dos marcos cualesquiera.
              </p>
              <CodeBlock
                lang="bash"
                content="$ ros2 run tf2_ros tf2_echo map base_link"
                :copyable="true"
              />
            </div>
          </div>
        </div>

        <!-- TOOL 3: RVIZ -->
        <div class="col-12">
          <div class="custom-card border-pink bg-slate-800 shadow-2 hover-lift">
            <div class="card-header q-pa-md border-bottom-dark row items-center">
              <q-icon name="visibility" color="pink-4" size="sm" class="q-mr-sm" />
              <div class="text-subtitle1 text-white">Inspección Visual (RViz)</div>
            </div>
            <div class="q-pa-md row items-center q-col-gutter-x-lg">
              <div class="col-12 col-sm-auto q-mb-sm-none q-mb-sm">
                <div
                  class="axis-icon bg-black q-pa-sm rounded border-light text-center"
                  style="min-width: 120px"
                >
                  <span class="text-red-5 font-bold font-mono">X</span>
                  <span class="text-green-5 font-bold font-mono q-mx-sm">Y</span>
                  <span class="text-blue-5 font-bold font-mono">Z</span>
                </div>
              </div>
              <div class="col text-grey-4 text-xs">
                Añade el display "TF" en RViz. Verás los ejes de coordenadas flotando en cada
                articulación.
                <br />
                <strong>Regla RGB-XYZ:</strong> Rojo apunta al Frente (X), Verde a la Izquierda (Y),
                Azul al Cielo (Z).
              </div>
            </div>
          </div>
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
    radial-gradient(circle at center, rgba(168, 85, 247, 0.15), transparent 60%),
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

/* TF TREE VISUALIZATION */
.tool-card {
  border-radius: 12px;
}
.border-purple {
  border: 1px solid #a855f7;
}
.border-right-grey {
  border-right: 1px dashed rgba(255, 255, 255, 0.2);
}
.full-height {
  height: 100%;
}

.tf-node {
  padding: 8px 16px;
  border-radius: 6px;
  font-family: monospace;
  font-size: 0.8rem;
  color: white;
  z-index: 2;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.link-line {
  width: 2px;
  height: 30px;
  background: #94a3b8;
}
.shadow-blue {
  box-shadow: 0 0 10px rgba(59, 130, 246, 0.4);
}
.shadow-orange {
  box-shadow: 0 0 10px rgba(249, 115, 22, 0.4);
}

.shake-anim {
  animation: shake 2s infinite;
}
@keyframes shake {
  0%,
  100% {
    transform: rotate(0deg);
  }
  25% {
    transform: rotate(5deg);
  }
  75% {
    transform: rotate(-5deg);
  }
}
.opacity-70 {
  opacity: 0.7;
}
.bg-red-9-soft {
  background: rgba(239, 68, 68, 0.2);
}
.bg-green-9-soft {
  background: rgba(34, 197, 94, 0.2);
}

/* CARDS */
.custom-card {
  border-radius: 12px;
  overflow: hidden;
}
.border-blue {
  border-left: 4px solid #3b82f6;
}
.border-orange {
  border-left: 4px solid #f97316;
}
.border-purple {
  border-left: 4px solid #a855f7;
}
.border-green {
  border-left: 4px solid #22c55e;
}
.border-pink {
  border-left: 4px solid #ec4899;
}
.border-left-red {
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
.icon-box {
  width: 32px;
  height: 32px;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-break {
  word-break: break-all;
}
.text-sm {
  font-size: 0.9rem;
}
.text-xs {
  font-size: 0.8rem;
}
.text-xxs {
  font-size: 0.7rem;
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
.border-grey {
  border: 1px solid #334155;
}
.shadow-inset {
  box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.5);
}
.code-label {
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  margin-bottom: 4px;
  color: #cbd5e1;
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
