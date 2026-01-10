<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-cyan-4 text-weight-bold q-mb-sm">
          MÓDULO 3.4: EL MULTIVERSO
        </div>

        <h1 class="hero-title">Ramas <span class="text-white">(Branches)</span></h1>

        <TextBlock>
          Hasta ahora has trabajado en una línea recta. Pero el desarrollo real no es lineal. Las
          <strong>Ramas</strong> son realidades paralelas. Puedes tener una realidad donde tu robot
          usa cámaras, y otra donde usa láseres. Trabaja sin miedo: lo que pasa en una rama, se
          queda en la rama... hasta que tú decidas lo contrario.
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA REGLA DE ORO: MAIN ES SAGRADO -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Regla de Oro</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            Tu rama principal (llamada <code>main</code> o antiguamente <code>master</code>) es
            sagrada. <br /><br />
            <strong>El código en Main siempre debe compilar y funcionar.</strong>
            Nunca experimentes directamente en Main. Si rompes Main, detienes el trabajo de todo el
            equipo.
          </TextBlock>
          <div class="q-mt-md">
            <AlertBlock type="warning" title="Zona de No-Combate">
              Para cualquier cambio (arreglar un bug, añadir un sensor), crea SIEMPRE una rama
              nueva.
            </AlertBlock>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <!-- VISUAL TRAFFIC LIGHT -->
          <div
            class="tool-card safety-card q-pa-lg row items-center justify-around bg-slate-900 shadow-2"
          >
            <!-- MAIN -->
            <div class="column items-center">
              <div class="traffic-light bg-green-9 shadow-green q-mb-md animate-pulse-slow">
                <q-icon name="check" color="white" size="2.5rem" />
              </div>
              <div class="text-h6 text-green-4 text-weight-bold">main</div>
              <div class="text-caption text-grey-5 text-center q-mt-xs font-mono">
                Estable.<br />Probado.<br />Sagrado.
              </div>
            </div>

            <div class="vertical-separator bg-grey-8" style="width: 1px; height: 120px"></div>

            <!-- FEATURE -->
            <div class="column items-center">
              <div class="traffic-light bg-yellow-9 shadow-yellow q-mb-md">
                <q-icon name="construction" color="black" size="2.5rem" />
              </div>
              <div class="text-h6 text-yellow-4 text-weight-bold">feature/*</div>
              <div class="text-caption text-grey-5 text-center q-mt-xs font-mono">
                Caos.<br />Experimentos.<br />Riesgo.
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. EL CICLO DE VIDA DE UNA RAMA -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Ciclo de Vida: Crear, Cambiar, Fusionar</SectionTitle>
      <TextBlock>
        Para trabajar con ramas, usamos comandos modernos (`switch`) que son más claros que el
        antiguo `checkout`.
      </TextBlock>

      <div class="lifecycle-grid q-mt-lg">
        <!-- PASO 1: CREAR -->
        <div class="step-card bg-slate-800 border-cyan">
          <div class="step-number bg-cyan-9 shadow-1">1</div>
          <div class="text-subtitle1 text-cyan-4 text-weight-bold">Crear y Saltar</div>
          <p class="text-caption text-grey-4 q-my-sm">
            Creas el universo paralelo y te teletransportas a él.
          </p>
          <div class="cmd-box border-cyan-dim">git switch -c feature/nuevo-sensor</div>
        </div>

        <!-- PASO 2: TRABAJAR -->
        <div class="step-card bg-slate-800 border-yellow">
          <div class="step-number bg-yellow-9 shadow-1">2</div>
          <div class="text-subtitle1 text-yellow-4 text-weight-bold">Hacer Cambios</div>
          <p class="text-caption text-grey-4 q-my-sm">
            Modificas archivos y haces commits normales. Main no se entera.
          </p>
          <div class="cmd-box border-yellow-dim">git commit -m "Añadir driver"</div>
        </div>

        <!-- PASO 3: VOLVER -->
        <div class="step-card bg-slate-800 border-green">
          <div class="step-number bg-green-9 shadow-1">3</div>
          <div class="text-subtitle1 text-green-4 text-weight-bold">Volver a Casa</div>
          <p class="text-caption text-grey-4 q-my-sm">Regresas a la realidad original (Main).</p>
          <div class="cmd-box border-green-dim">git switch main</div>
        </div>

        <!-- PASO 4: FUSIONAR -->
        <div class="step-card bg-slate-800 border-purple">
          <div class="step-number bg-purple-9 shadow-1">4</div>
          <div class="text-subtitle1 text-purple-4 text-weight-bold">Fusionar (Merge)</div>
          <p class="text-caption text-grey-4 q-my-sm">Traes los cambios de la rama a Main.</p>
          <div class="cmd-box border-purple-dim">git merge feature/nuevo-sensor</div>
        </div>
      </div>
    </div>

    <!-- 4. VISUALIZACIÓN DE MERGE -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Visualizando la Fusión (Merge)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Cuando haces <strong>Merge</strong>, Git toma la historia de tu rama experimental y la
            inyecta en <code>main</code>. <br /><br />
            Si todo sale bien (Fast-forward), es como si nunca te hubieras desviado. Si hay cambios
            complejos, Git crea un "Merge Commit" que une los dos caminos.
          </TextBlock>
        </template>

        <template #right>
          <div class="tool-card visual-merge q-pa-lg bg-slate-900 flex flex-center">
            <svg viewBox="0 0 400 160" class="merge-svg">
              <!-- MAIN LINE -->
              <line
                x1="20"
                y1="130"
                x2="380"
                y2="130"
                stroke="#4ade80"
                stroke-width="4"
                stroke-linecap="round"
              />
              <text
                x="20"
                y="155"
                fill="#4ade80"
                font-family="monospace"
                font-size="12"
                font-weight="bold"
              >
                main
              </text>

              <!-- FEATURE LINE -->
              <path
                d="M 80 130 Q 120 130 120 60 L 280 60 Q 320 60 320 120"
                stroke="#22d3ee"
                stroke-width="4"
                fill="none"
                stroke-dasharray="8,4"
              />
              <text
                x="140"
                y="50"
                fill="#22d3ee"
                font-family="monospace"
                font-size="12"
                font-weight="bold"
              >
                feature/sensor
              </text>

              <!-- COMMITS MAIN -->
              <circle cx="50" cy="130" r="7" fill="#4ade80" stroke="#0f172a" stroke-width="2" />
              <circle cx="80" cy="130" r="7" fill="#4ade80" stroke="#0f172a" stroke-width="2" />
              <!-- Divergence -->

              <!-- COMMITS FEATURE -->
              <circle cx="160" cy="60" r="7" fill="#22d3ee" stroke="#0f172a" stroke-width="2" />
              <circle cx="240" cy="60" r="7" fill="#22d3ee" stroke="#0f172a" stroke-width="2" />

              <!-- MERGE POINT -->
              <circle cx="320" cy="130" r="12" fill="#a855f7" stroke="white" stroke-width="2" />
              <text
                x="305"
                y="105"
                fill="#a855f7"
                font-family="monospace"
                font-size="12"
                font-weight="bold"
              >
                MERGE
              </text>
            </svg>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. CONFLICTOS (EL VILLANO) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. Pesadilla: Conflictos de Fusión</SectionTitle>
      <AlertBlock type="danger" title="Cuando dos personas tocan la misma línea">
        Si tú cambias la línea 10 de un archivo en tu rama, y otro compañero cambia la MISMA línea
        10 en Main... Git entra en pánico. No sabe a quién creerle y te pide ayuda.
      </AlertBlock>

      <div class="tool-card conflict-card q-mt-lg bg-black q-pa-none overflow-hidden shadow-2">
        <div
          class="bg-dark-soft q-px-md q-py-xs border-bottom-dark text-caption text-grey-5 font-mono flex items-center"
        >
          <q-icon name="warning" color="red" size="xs" class="q-mr-sm" />
          CONFLICTO: robot_config.py
        </div>

        <div class="conflict-viewer font-mono text-sm q-pa-md">
          <div class="text-grey-4">class Robot:</div>

          <!-- HEAD (MAIN) -->
          <div class="conflict-block head">
            <div class="marker bg-green-9 text-white">
              &lt;&lt;&lt;&lt;&lt;&lt;&lt; HEAD (Lo que hay en Main)
            </div>
            <div class="content text-green-2 q-pl-lg">velocidad = 10</div>
          </div>

          <!-- SEPARATOR -->
          <div class="marker bg-grey-8 text-white">=======</div>

          <!-- INCOMING (RAMA) -->
          <div class="conflict-block incoming">
            <div class="content text-cyan-2 q-pl-lg">velocidad = 20</div>
            <div class="marker bg-cyan-9 text-white">
              &gt;&gt;&gt;&gt;&gt;&gt;&gt; feature/sensor (Lo que tú traes)
            </div>
          </div>

          <div class="text-grey-4">def iniciar(self):</div>
        </div>

        <div class="bg-dark-soft q-pa-sm text-center border-top-dashed">
          <p class="text-grey-5 text-caption q-mb-none">
            💡 Solución: Borra las marcas (<code>&lt;&lt;&lt;</code>, <code>===</code>,
            <code>&gt;&gt;&gt;</code>) y deja solo el código correcto.
          </p>
        </div>
      </div>
    </div>

    <!-- 6. LIMPIEZA -->
    <div class="section-group self-stretch q-mb-xl q-mt-xl">
      <div class="bg-slate-800 q-pa-lg rounded-borders border-dashed text-center shadow-1">
        <q-icon name="cleaning_services" size="md" color="grey-5" class="q-mb-sm" />
        <div class="text-h6 text-white">Limpia tu Laboratorio</div>
        <p class="text-grey-4 q-mb-md">
          Una vez que fusionas una rama, esa rama ya cumplió su propósito. No la acumules.
        </p>
        <div
          class="bg-black q-pa-md rounded-borders font-mono text-body2 text-red-4 inline-block border-light"
        >
          git branch -d feature/nuevo-sensor
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
    radial-gradient(circle at center, rgba(34, 211, 238, 0.15), transparent 60%),
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

/* SAFETY CARD */
.safety-card {
  border-top: 4px solid #4ade80;
}
.traffic-light {
  width: 70px;
  height: 70px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
}
.shadow-green {
  box-shadow: 0 0 30px rgba(74, 222, 128, 0.4);
}
.shadow-yellow {
  box-shadow: 0 0 30px rgba(250, 204, 21, 0.4);
}
.animate-pulse-slow {
  animation: pulse 3s infinite;
}
@keyframes pulse {
  0% {
    opacity: 1;
  }
  50% {
    opacity: 0.7;
  }
  100% {
    opacity: 1;
  }
}

/* LIFECYCLE GRID */
.lifecycle-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 20px;
}
.step-card {
  padding: 24px 20px;
  border-radius: 12px;
  position: relative;
  border-top: 4px solid;
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  transition: transform 0.2s;
}
.step-card:hover {
  transform: translateY(-5px);
}

.step-number {
  position: absolute;
  top: -15px;
  right: 15px;
  width: 30px;
  height: 30px;
  border-radius: 50%;
  color: white;
  font-weight: bold;
  font-size: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 2px solid #1e293b;
}
.border-cyan {
  border-top-color: #22d3ee;
}
.border-yellow {
  border-top-color: #facc15;
}
.border-green {
  border-top-color: #4ade80;
}
.border-purple {
  border-top-color: #a855f7;
}

.cmd-box {
  background: #0f172a;
  color: #e2e8f0;
  padding: 8px;
  border-radius: 6px;
  font-family: 'Fira Code', monospace;
  font-size: 0.75rem;
  text-align: center;
  margin-top: auto;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-cyan-dim {
  border-color: rgba(34, 211, 238, 0.3);
  color: #22d3ee;
}
.border-yellow-dim {
  border-color: rgba(250, 204, 21, 0.3);
  color: #facc15;
}
.border-green-dim {
  border-color: rgba(74, 222, 128, 0.3);
  color: #4ade80;
}
.border-purple-dim {
  border-color: rgba(168, 85, 247, 0.3);
  color: #a855f7;
}

/* VISUAL MERGE */
.visual-merge {
  background: #0f172a;
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.merge-svg {
  width: 100%;
  height: auto;
  max-width: 500px;
}

/* CONFLICT VIEWER */
.conflict-card {
  border-left: 4px solid #ef4444;
}
.conflict-viewer {
  background: #000;
}

.conflict-block {
  width: 100%;
}
.marker {
  padding: 4px 10px;
  font-weight: bold;
  opacity: 0.9;
}
.content {
  padding: 8px 0;
  background: rgba(255, 255, 255, 0.02);
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-sm {
  font-size: 0.9rem;
}
.inline-block {
  display: inline-block;
}
.bg-slate-800 {
  background: #1e293b;
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-dark-soft {
  background: rgba(255, 255, 255, 0.05);
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-dashed {
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-dashed {
  border-top: 1px dashed rgba(255, 255, 255, 0.1);
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .lifecycle-grid {
    grid-template-columns: 1fr;
  }
}
</style>
