<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-purple-4 text-weight-bold q-mb-sm">
          MÓDULO 3.3: NAVEGACIÓN TEMPORAL
        </div>

        <h1 class="hero-title">Explorando la <span class="text-white">Historia</span></h1>

        <TextBlock>
          De nada sirve guardar versiones si no sabes cómo volver a ellas. Git te permite moverte
          por la línea de tiempo de tu proyecto. Puedes viajar al pasado para ver cómo funcionaba tu
          código hace un mes, o deshacer ese cambio fatal que rompió tu compilación.
        </TextBlock>
      </div>
    </section>

    <!-- 2. GIT LOG AVANZADO -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Mapa Estelar (git log)</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            El comando <code>git log</code> por defecto es feo y difícil de leer. Los profesionales
            usan "flags" (opciones) para convertir esa lista aburrida en un árbol gráfico
            comprensible.
          </TextBlock>
          <div class="bg-slate-800 q-pa-md rounded-borders q-mt-md border-purple">
            <div class="text-caption text-grey-4 q-mb-xs">La Navaja Suiza:</div>
            <code class="text-code text-purple-3">git log --oneline --graph --all</code>
          </div>
        </div>

        <div class="col-12 col-md-7">
          <!-- TERMINAL VISUALIZER -->
          <div class="tool-card log-visualizer bg-black q-pa-md font-mono text-sm shadow-2">
            <div class="window-header row justify-between q-mb-md border-bottom-dark q-pb-xs">
              <span class="text-grey-6 text-caption">bash</span>
              <span class="text-grey-7 text-caption">--oneline --graph</span>
            </div>

            <!-- COMMIT TREE (Custom HTML for perfect control) -->
            <div class="tree-line row items-center no-wrap">
              <span class="text-purple-4 text-weight-bold">*</span>
              <span class="text-yellow-4 q-mx-sm">a1b2c</span>
              <span class="text-white text-xs"
                >Fix: sensor lidar
                <span class="text-green-4 text-weight-bold">(HEAD -> main)</span></span
              >
            </div>

            <div class="tree-line row items-center no-wrap">
              <span class="text-purple-4 text-weight-bold">|</span>
              <span class="text-grey-6 q-ml-sm q-mr-xs">\</span>
            </div>

            <div class="tree-line row items-center no-wrap">
              <span class="text-purple-4 text-weight-bold">|</span>
              <span class="text-grey-6 q-mx-sm">*</span>
              <span class="text-yellow-4 q-mr-sm">d4e5f</span>
              <span class="text-grey-4 text-xs"
                >Feat: nueva cámara <span class="text-orange-4">(feature/cam)</span></span
              >
            </div>

            <div class="tree-line row items-center no-wrap">
              <span class="text-purple-4 text-weight-bold">*</span>
              <span class="text-grey-6 q-ml-sm">/</span>
              <span class="text-yellow-4 q-mx-sm">g7h8i</span>
              <span class="text-white text-xs">Init: configuración base</span>
            </div>

            <div
              class="q-mt-md q-pa-xs bg-dark-soft rounded-borders text-center text-grey-5 text-caption font-italic"
            >
              👆 Así se ven las ramas divergiendo y convergiendo.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. VIAJAR AL PASADO (CHECKOUT/SWITCH) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Viajando en el Tiempo (Checkout)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Imagina que quieres probar la versión de la semana pasada (Hash: <code>g7h8i</code>).
            <br /><br />
            Usas <code>git checkout g7h8i</code>.
          </TextBlock>
          <div class="q-mt-md">
            <AlertBlock type="warning" title="¡CUIDADO! Detached HEAD">
              Entras en "Modo Fantasma". Puedes mirar, compilar y ejecutar, pero si haces cambios
              aquí, <strong>se perderán</strong> al volver al presente a menos que crees una rama
              nueva.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="tool-card time-travel relative-position q-pa-lg bg-slate-900">
            <!-- TIMELINE -->
            <div class="timeline-container relative-position full-width">
              <div class="line absolute-center"></div>

              <!-- PAST POINT -->
              <div class="point past absolute" style="top: 80%; left: 50%">
                <div class="dot bg-grey-6"></div>
                <div class="label text-grey-5 font-mono text-xs">g7h8i (Pasado)</div>
              </div>

              <!-- PRESENT POINT -->
              <div class="point present absolute" style="top: 20%; left: 50%">
                <div class="dot bg-green-5 shadow-glow-green"></div>
                <div class="label text-green-4 font-mono text-xs">main (Presente)</div>
              </div>

              <!-- GHOST HEAD ANIMATION -->
              <div
                class="ghost-head absolute bg-purple-6 text-white text-center q-pa-xs rounded-borders shadow-glow-purple row items-center justify-center"
              >
                <span class="text-weight-bold q-mr-xs text-xs">HEAD</span>
                <q-icon name="visibility" size="xs" />
              </div>
            </div>

            <div
              class="text-caption text-center q-mt-md text-purple-3 bg-dark-soft q-pa-sm rounded-borders"
            >
              HEAD es el "ojo" que mira tu código. Mover HEAD es viajar en el tiempo.
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. DESHACER CAMBIOS (RESTORE / REVERT) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Botones de Pánico: Deshacer</SectionTitle>
      <AlertBlock type="danger" title="¡Rompí todo! ¿Cómo vuelvo?">
        Hay 3 niveles de "Arrepentimiento" en Git. Elige tu medicina con cuidado.
      </AlertBlock>

      <div class="row q-col-gutter-md q-mt-sm items-stretch">
        <!-- NIVEL 1: ARCHIVO SUCIO -->
        <div class="col-12 col-md-4">
          <div class="tool-card undo-card level-1 full-height">
            <div>
              <div class="text-subtitle1 text-yellow-4 q-mb-xs text-weight-bold">
                Nivel 1: Archivo Sucio
              </div>
              <p class="text-caption text-grey-4">
                "Modifiqué un archivo pero <strong>aún no hice commit</strong>".
              </p>
            </div>
            <div>
              <div class="cmd-box bg-black text-green-4 q-mt-md shadow-1">
                git restore archivo.py
              </div>
              <div class="text-xs text-grey-5 q-mt-sm text-center">
                Vuelve al estado del último commit.
              </div>
            </div>
          </div>
        </div>

        <!-- NIVEL 2: CAMBIAR ÚLTIMO COMMIT -->
        <div class="col-12 col-md-4">
          <div class="tool-card undo-card level-2 full-height">
            <div>
              <div class="text-subtitle1 text-orange-4 q-mb-xs text-weight-bold">
                Nivel 2: Commit Erróneo
              </div>
              <p class="text-caption text-grey-4">
                "Hice commit pero olvidé un archivo o me equivoqué en el mensaje".
              </p>
            </div>
            <div>
              <div class="cmd-box bg-black text-green-4 q-mt-md shadow-1">git commit --amend</div>
              <div class="text-xs text-grey-5 q-mt-sm text-center">
                Reescribe la historia reciente.
              </div>
            </div>
          </div>
        </div>

        <!-- NIVEL 3: REVERTIR PÚBLICAMENTE -->
        <div class="col-12 col-md-4">
          <div class="tool-card undo-card level-3 full-height">
            <div>
              <div class="text-subtitle1 text-red-4 q-mb-xs text-weight-bold">
                Nivel 3: Commit Antiguo
              </div>
              <p class="text-caption text-grey-4">
                "El commit de hace 3 días tiene un bug, pero ya hay trabajo nuevo encima".
              </p>
            </div>
            <div>
              <div class="cmd-box bg-black text-green-4 q-mt-md shadow-1">
                git revert &lt;hash&gt;
              </div>
              <div class="text-xs text-grey-5 q-mt-sm text-center">
                Crea un <strong>nuevo commit</strong> que deshace lo malo.
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. DIFF (COMPARAR) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. Detectives de Código (Git Diff)</SectionTitle>
      <TextBlock>
        Antes de hacer commit, siempre debes preguntar: "¿Qué cambié exactamente?".
        <br /><code>git diff</code> te muestra las diferencias línea por línea.
      </TextBlock>

      <div class="tool-card diff-card bg-slate-900 q-pa-md q-mt-md font-mono text-sm shadow-2">
        <div class="text-grey-5 text-caption border-bottom-dark q-mb-xs">
          diff --git a/robot.py b/robot.py
        </div>
        <div class="text-blue-4 text-caption q-mb-sm">@@ -10,2 +10,2 @@ def mover():</div>

        <div class="row bg-red-9-soft q-px-sm text-grey-3 q-py-xs">
          - velocidad = 100 <span class="text-grey-5 q-ml-sm"># Muy rápido</span>
        </div>
        <div class="row bg-green-9-soft q-px-sm text-white q-py-xs">
          + velocidad = 50 <span class="text-grey-4 q-ml-sm"># Velocidad segura</span>
        </div>

        <div class="text-grey-5 q-mt-sm text-caption font-italic border-top-dashed q-pt-xs">
          El rojo es lo que se fue. El verde es lo que llegó.
        </div>
      </div>
    </div>

    <!-- 6. ALIAS RECOMENDADOS -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <div
        class="bg-slate-800 q-pa-lg rounded-borders border-purple text-center shadow-glow-purple"
      >
        <q-icon name="stars" size="md" color="purple-4" class="q-mb-sm" />
        <div class="text-h6 text-white">El Comando Definitivo</div>
        <p class="text-grey-4 q-mb-md">
          Nadie escribe <code>git log --oneline --graph --all --decorate</code> cada vez. Crea un
          alias global llamado <code>git lg</code>.
        </p>
        <div
          class="bg-black q-pa-md rounded-borders font-mono text-body2 text-green-4 text-break border-light"
        >
          git config --global alias.lg "log --oneline --graph --all --decorate"
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

/* TOOL CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* LOG VISUALIZER */
.log-visualizer {
  border-top: 4px solid #a855f7;
}
.tree-line {
  line-height: 1.6;
}

/* TIME TRAVEL ANIMATION */
.time-travel {
  overflow: hidden;
  height: 320px;
  display: flex;
  align-items: center;
  justify-content: center;
}
.timeline-container {
  height: 240px;
}
.line {
  width: 2px;
  height: 100%;
  background: #475569;
}

.point {
  transform: translate(-50%, -50%);
  display: flex;
  align-items: center;
  gap: 12px;
}
.point.past .label {
  position: absolute;
  left: 24px;
  white-space: nowrap;
}
.point.present .label {
  position: absolute;
  left: 24px;
  white-space: nowrap;
}

.dot {
  width: 14px;
  height: 14px;
  border-radius: 50%;
  border: 2px solid #0f172a;
}

.ghost-head {
  width: 80px;
  top: 50%;
  left: 50%;
  transform: translate(-140%, -50%);
  animation: floatTime 4s infinite alternate ease-in-out;
  z-index: 10;
}
@keyframes floatTime {
  0% {
    top: 20%;
  }
  100% {
    top: 80%;
  }
}
.shadow-glow-purple {
  box-shadow: 0 0 15px rgba(168, 85, 247, 0.5);
}
.shadow-glow-green {
  box-shadow: 0 0 10px rgba(74, 222, 128, 0.5);
}

/* UNDO CARDS */
.undo-card {
  padding: 24px;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
}
.level-1 {
  border-top: 4px solid #facc15;
  background: rgba(30, 41, 59, 0.4);
}
.level-2 {
  border-top: 4px solid #fb923c;
  background: rgba(30, 41, 59, 0.4);
}
.level-3 {
  border-top: 4px solid #f87171;
  background: rgba(30, 41, 59, 0.4);
}

.cmd-box {
  padding: 10px;
  border-radius: 6px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  text-align: center;
}

/* DIFF CARD */
.diff-card {
  border-left: 4px solid #3b82f6;
}
.bg-red-9-soft {
  background: rgba(185, 28, 28, 0.3);
}
.bg-green-9-soft {
  background: rgba(21, 128, 61, 0.3);
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-sm {
  font-size: 0.9rem;
}
.text-xs {
  font-size: 0.8rem;
}
.text-break {
  word-break: break-all;
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
.border-purple {
  border: 1px solid #a855f7;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
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
}
</style>
