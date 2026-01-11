<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-amber-4 text-weight-bold q-mb-sm">
          MÓDULO 8.4: EL GRAN CIERRE
        </div>
        <h1 class="hero-title">
          Proyecto Final: <span class="text-white">Operación Warehouse</span>
        </h1>
        <TextBlock>
          Felicidades por llegar hasta aquí. Has aprendido las herramientas individuales. Ahora,
          tienes que usarlas todas juntas.
          <br /><br />
          Tu misión, si decides aceptarla, es diseñar, construir y programar un robot móvil autónomo
          capaz de operar en un almacén logístico simulado. No hay guías paso a paso. Eres tú contra
          el código.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. El Requerimiento del Cliente</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            La empresa "LogiBot Inc." necesita un robot para mover cajas.
            <br /><strong>Especificaciones Funcionales:</strong>
          </TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list">
            <li>📍 <strong>Inicio:</strong> El robot nace en la zona de carga (Dock A).</li>
            <li>🗺️ <strong>Mapeo:</strong> Debe generar un mapa del almacén desconocido.</li>
            <li>
              📦 <strong>Misión:</strong> Ir al Estante 3, esperar 5 segundos (carga simulada), ir
              al Dock B y volver a casa.
            </li>
            <li>
              🚧 <strong>Seguridad:</strong> Debe esquivar obstáculos dinámicos (otros
              robots/personas).
            </li>
          </ul>
        </template>
        <template #right>
          <div class="tool-card bg-slate-900 relative-position overflow-hidden h-full">
            <div class="absolute-center full-width text-center">
              <svg width="300" height="200" viewBox="0 0 300 200">
                <rect
                  x="20"
                  y="20"
                  width="60"
                  height="40"
                  stroke="#4ade80"
                  fill="none"
                  stroke-dasharray="4"
                />
                <text x="50" y="45" fill="#4ade80" font-size="10" text-anchor="middle">DOCK A</text>

                <rect
                  x="220"
                  y="20"
                  width="60"
                  height="40"
                  stroke="#facc15"
                  fill="none"
                  stroke-dasharray="4"
                />
                <text x="250" y="45" fill="#facc15" font-size="10" text-anchor="middle">
                  SHELF 3
                </text>

                <rect
                  x="220"
                  y="140"
                  width="60"
                  height="40"
                  stroke="#f87171"
                  fill="none"
                  stroke-dasharray="4"
                />
                <text x="250" y="165" fill="#f87171" font-size="10" text-anchor="middle">
                  DOCK B
                </text>

                <path
                  id="missionPath"
                  d="M 50 60 L 50 100 L 250 60 L 250 140 L 50 100 L 50 60"
                  fill="none"
                  stroke="#3b82f6"
                  stroke-width="2"
                  stroke-dasharray="5,5"
                />

                <circle r="8" fill="#3b82f6">
                  <animateMotion dur="6s" repeatCount="indefinite">
                    <mpath href="#missionPath" />
                  </animateMotion>
                </circle>
              </svg>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Tu Caja de Herramientas</SectionTitle>
      <TextBlock>
        Para aprobar, tu repositorio debe contener los siguientes paquetes y tecnologías:
      </TextBlock>

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-3">
          <div class="tech-card border-pink">
            <q-icon name="view_in_ar" size="3rem" color="pink-4" class="q-mb-sm" />
            <div class="text-h6 text-white">1. URDF Custom</div>
            <p class="text-caption text-grey-4">
              Diseña tu propio robot (chasis + 2 ruedas + lidar). No uses el Turtlebot
              predeterminado.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-3">
          <div class="tech-card border-orange">
            <q-icon name="public" size="3rem" color="orange-4" class="q-mb-sm" />
            <div class="text-h6 text-white">2. Gazebo World</div>
            <p class="text-caption text-grey-4">
              Crea un mundo <code>.sdf</code> con paredes y cajas. Usa modelos de Gazebo Fuel.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-3">
          <div class="tech-card border-indigo">
            <q-icon name="explore" size="3rem" color="indigo-4" class="q-mb-sm" />
            <div class="text-h6 text-white">3. Nav2 Config</div>
            <p class="text-caption text-grey-4">
              Configura SLAM Toolbox y Nav2. Sintoniza los Costmaps para que no choque.
            </p>
          </div>
        </div>
        <div class="col-12 col-md-3">
          <div class="tech-card border-green">
            <q-icon name="code" size="3rem" color="green-4" class="q-mb-sm" />
            <div class="text-h6 text-white">4. Python Node</div>
            <p class="text-caption text-grey-4">
              Script de orquestación (Simple Commander) que ejecute la secuencia de la misión.
            </p>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Arquitectura del Proyecto</SectionTitle>
      <TextBlock>
        Un ingeniero senior se reconoce por la limpieza de su workspace. Sigue esta estructura de
        paquetes:
      </TextBlock>

      <div class="tool-card bg-slate-900 q-pa-lg">
        <div class="row q-col-gutter-md">
          <div class="col-12 col-md-6">
            <div class="font-mono text-sm text-grey-3">
              <div class="text-blue-4">src/</div>
              <div class="q-pl-md">
                <div class="text-yellow-4">my_robot_description/</div>
                <div class="q-pl-md text-grey-5"># URDF, Meshes, Rviz Config</div>
              </div>
              <div class="q-pl-md">
                <div class="text-yellow-4">my_robot_gazebo/</div>
                <div class="q-pl-md text-grey-5"># Mundos, Launch de simulación</div>
              </div>
              <div class="q-pl-md">
                <div class="text-yellow-4">my_robot_navigation/</div>
                <div class="q-pl-md text-grey-5"># Mapas, Params Nav2, Launch Nav2</div>
              </div>
              <div class="q-pl-md">
                <div class="text-yellow-4">my_robot_mission/</div>
                <div class="q-pl-md text-grey-5"># Script Python principal</div>
              </div>
            </div>
          </div>
          <div class="col-12 col-md-6 flex flex-center">
            <AlertBlock type="warning" title="Launch Único">
              Debes crear un archivo <code>bringup_all.launch.py</code> que lance TODO con un solo
              comando. <br />¡No hagas al usuario abrir 4 terminales!
            </AlertBlock>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. Tu Hoja de Ruta</SectionTitle>
      <StepsBlock
        :steps="[
          'Día 1: Construye el robot (URDF) y visualízalo en RViz.',
          'Día 2: Ponlo en Gazebo y haz que se mueva con teleop (Plugins).',
          'Día 3: Mapea el entorno con SLAM Toolbox y guarda el mapa.',
          'Día 4: Configura Nav2 y prueba enviar metas con el ratón.',
          'Día 5: Escribe el script de Python para automatizar la patrulla.',
          'Día 6: Crea el Launch File maestro y el Dockerfile (Opcional).',
        ]"
      />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. ¿Cómo presentar?</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En el mundo real, tu jefe no va a revisar tu código línea por línea. Quiere ver
            resultados.
            <br /><br />
            <strong>Entregable:</strong> Un video de 2 minutos (captura de pantalla) mostrando: 1.
            El comando de lanzamiento. 2. El robot completando la misión en Gazebo + RViz (pantalla
            dividida). 3. El robot esquivando un obstáculo inesperado.
          </TextBlock>
        </template>
        <template #right>
          <div class="tool-card bg-black flex flex-center h-full relative-position border-red">
            <div class="absolute-top-right q-ma-md row items-center">
              <div class="rec-dot bg-red-5 q-mr-sm"></div>
              <div class="text-white font-mono text-xs">REC 00:01:23</div>
            </div>
            <q-icon name="movie" size="4rem" color="grey-8" />
            <div class="absolute-bottom text-center q-pb-md text-grey-5">demo_final.mp4</div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <div class="certificate-container relative-position q-pa-xl text-center overflow-hidden">
        <div class="confetti-bg absolute-full"></div>

        <div class="relative-position z-top">
          <q-icon
            name="workspace_premium"
            size="6rem"
            class="text-gradient-gold q-mb-md drop-shadow-lg"
          />
          <h2 class="text-h3 text-white text-weight-bolder q-my-none">¡Estás Listo!</h2>
          <p class="text-h6 text-amber-2 q-mt-sm">Nivel Desbloqueado: ROS 2 Developer</p>

          <TextBlock
            class="q-mt-lg"
            style="max-width: 600px; margin-left: auto; margin-right: auto"
          >
            Al completar este proyecto, habrás superado la barrera que detiene al 90% de los
            estudiantes. Ya no solo "sabes" ROS 2. <strong>Haces</strong> ROS 2. <br /><br />
            Prepara tu café, abre tu IDE y... ¡Buena suerte, Ingeniero!
          </TextBlock>

          <q-btn
            label="Comenzar Proyecto"
            color="amber-5"
            text-color="black"
            size="lg"
            class="q-mt-xl text-weight-bold shadow-gold hover-lift"
            to="/modulo-0/01navsistemaPage"
          />
          <div class="text-caption text-grey-5 q-mt-sm">(O vuelve al inicio para repasar)</div>
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
import StepsBlock from 'components/content/StepsBlock.vue';
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
    radial-gradient(circle at center, rgba(251, 191, 36, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
  margin-bottom: 3rem;
}
.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  color: #f8fafc;
}

/* TECH CARDS */
.tech-card {
  background: rgba(30, 41, 59, 0.5);
  border-radius: 12px;
  padding: 20px;
  text-align: center;
  height: 100%;
  border-top: 4px solid;
  transition: transform 0.2s;
}
.tech-card:hover {
  transform: translateY(-5px);
}
.border-pink {
  border-color: #ec4899;
}
.border-orange {
  border-color: #f97316;
}
.border-indigo {
  border-color: #6366f1;
}
.border-green {
  border-color: #4ade80;
}

/* VISUALIZATIONS */
.tool-card {
  border-radius: 12px;
  height: 100%;
}
.h-full {
  height: 100%;
}
.rec-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  animation: blink 1s infinite;
}
.border-red {
  border: 1px solid #ef4444;
}

/* CERTIFICATE */
.certificate-container {
  background: linear-gradient(135deg, #1e293b 0%, #0f172a 100%);
  border-radius: 24px;
  border: 2px solid rgba(251, 191, 36, 0.3);
  box-shadow: 0 0 50px rgba(251, 191, 36, 0.1);
}
.text-gradient-gold {
  background: -webkit-linear-gradient(#fcd34d, #d97706);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}
.drop-shadow-lg {
  filter: drop-shadow(0 4px 6px rgba(0, 0, 0, 0.5));
}
.shadow-gold {
  box-shadow: 0 4px 20px rgba(245, 158, 11, 0.4);
}
.hover-lift:hover {
  transform: translateY(-3px);
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.text-xxs {
  font-size: 0.7rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.z-top {
  z-index: 10;
}

@keyframes blink {
  50% {
    opacity: 0;
  }
}
</style>
