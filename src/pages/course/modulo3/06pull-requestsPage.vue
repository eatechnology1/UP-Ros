<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-green-4 text-weight-bold q-mb-sm">
          MÓDULO 3.6: EL CONTROL DE CALIDAD
        </div>

        <h1 class="hero-title">Pull <span class="text-white">Requests</span></h1>

        <TextBlock>
          En equipos profesionales, nadie toca la rama <code>main</code> directamente. Está
          prohibido. Para introducir cambios, debes crear una "Solicitud de Fusión" (Pull Request).
          Es una sala de espera virtual donde tus compañeros revisan, critican y aprueban tu código
          antes de que toque el robot real.
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL CONCEPTO: ¿QUÉ ES UN PR? -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Sala de Juicios</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Imagina un filtro de seguridad.
            <br /><br />
            Tú (el desarrollador) dices:
            <em
              >"He terminado la función de navegación en mi rama. Por favor, llévenla (Pull) a la
              rama principal"</em
            >. <br /><br />
            GitHub genera una página web donde se ve <strong>la diferencia exacta</strong> (Diff)
            entre tu rama y Main. Ahí empieza la discusión.
          </TextBlock>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL PR CARD (DARK MODE GITHUB SIMULATION) -->
          <div
            class="tool-card pr-visual relative-position q-pa-lg bg-slate-900 text-white shadow-2"
          >
            <!-- HEADER -->
            <div class="row items-center q-mb-md border-bottom-dark q-pb-sm">
              <q-icon name="merge_type" color="grey-5" size="1.8rem" class="q-mr-sm" />
              <div class="text-subtitle1">
                Add Lidar Sensor Driver <span class="text-grey-6">#42</span>
              </div>
            </div>

            <div class="row items-center q-mb-md">
              <div
                class="chip-status bg-green-9 text-white q-px-sm rounded-borders text-caption text-weight-bold border-green-glow row items-center"
              >
                <q-icon name="adjust" size="xs" class="q-mr-xs" /> Open
              </div>
              <div class="text-caption text-grey-4 q-ml-md">
                <strong>alex-robot</strong> wants to merge 1 commit into
                <code class="bg-slate-800 q-px-xs rounded border-light font-mono">main</code> from
                <code class="bg-blue-9-soft q-px-xs rounded font-mono text-blue-2"
                  >feature/lidar</code
                >
              </div>
            </div>

            <!-- CONVERSATION -->
            <div class="conversation q-pl-md border-left-grey q-gutter-y-md relative-position">
              <!-- Comentario 1 -->
              <div class="row items-start">
                <q-avatar size="28px" color="blue-7" text-color="white" class="q-mr-sm shadow-1"
                  >A</q-avatar
                >
                <div
                  class="bg-slate-800 q-pa-sm rounded-borders text-caption col border-light speech-bubble"
                >
                  He añadido el driver del RPLidar A1. Ya compila.
                </div>
              </div>

              <!-- Comentario 2 (Review) -->
              <div class="row items-start">
                <q-avatar size="28px" color="orange-8" text-color="white" class="q-mr-sm shadow-1"
                  >S</q-avatar
                >
                <div
                  class="bg-red-9-soft q-pa-sm rounded-borders text-caption col border-red-subtle speech-bubble"
                >
                  <div class="row items-center q-mb-xs">
                    <q-icon name="cancel" color="red-4" size="xs" class="q-mr-xs" />
                    <span class="text-red-4 text-weight-bold">Changes requested</span>
                  </div>
                  Falta añadir la dependencia en <code>package.xml</code>. El robot no va a saber
                  instalarlo.
                </div>
              </div>
            </div>

            <!-- BUTTON -->
            <div class="q-mt-lg border-top-dark q-pt-md">
              <div
                class="row items-center justify-between bg-slate-800 q-pa-sm rounded-borders opacity-50 cursor-not-allowed border-light"
              >
                <div class="row items-center">
                  <div
                    class="bg-green-7 text-white q-px-md q-py-xs rounded-borders text-weight-bold text-caption q-mr-sm"
                  >
                    Merge pull request
                  </div>
                  <q-icon name="arrow_drop_down" color="white" />
                </div>
                <div class="text-xs text-grey-5 q-mr-sm">
                  <q-icon name="lock" size="xs" /> Blocked
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. CODE REVIEW (LA REVISIÓN) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Code Review: No es Personal</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Recibir comentarios en tu código duele al ego, pero es la forma más rápida de aprender.
            <br /><br />
            En un PR, tus compañeros pueden seleccionar una línea específica y dejar un comentario.
            Hasta que no marques esos comentarios como "Resolved", el botón de
            <strong>Merge</strong> estará bloqueado (si así se configuró).
          </TextBlock>
          <div class="q-mt-lg">
            <AlertBlock type="info" title="CI/CD Checks">
              GitHub también puede ejecutar "robots jueces" automáticos. Si tu código no compila o
              tiene errores de sintaxis, el robot bloqueará el PR antes de que un humano lo vea.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="tool-card review-flow bg-slate-900 q-pa-lg">
            <div class="column items-center q-gutter-y-sm">
              <!-- STEP 1 -->
              <div class="row items-center full-width group-hover">
                <div class="col-2 text-center">
                  <q-icon name="upload" color="blue-4" size="md" />
                </div>
                <div
                  class="col bg-slate-800 q-pa-md rounded-borders border-left-blue transition-hover"
                >
                  <div class="text-blue-4 text-weight-bold text-subtitle2">1. Push</div>
                  <div class="text-xs text-grey-4">Subes tu rama a GitHub.</div>
                </div>
              </div>

              <div class="vertical-line bg-grey-8"></div>

              <!-- STEP 2 -->
              <div class="row items-center full-width group-hover">
                <div class="col-2 text-center">
                  <q-icon name="rate_review" color="yellow-4" size="md" />
                </div>
                <div
                  class="col bg-slate-800 q-pa-md rounded-borders border-left-yellow transition-hover"
                >
                  <div class="text-yellow-4 text-weight-bold text-subtitle2">2. Review</div>
                  <div class="text-xs text-grey-4">Discusión, críticas y correcciones.</div>
                </div>
              </div>

              <div class="vertical-line bg-grey-8"></div>

              <!-- STEP 3 -->
              <div class="row items-center full-width group-hover">
                <div class="col-2 text-center">
                  <q-icon name="check_circle" color="green-4" size="md" />
                </div>
                <div
                  class="col bg-slate-800 q-pa-md rounded-borders border-left-green shadow-glow-green transition-hover"
                >
                  <div class="text-green-4 text-weight-bold text-subtitle2">3. Merge</div>
                  <div class="text-xs text-grey-4">Aprobado. El código entra a Main.</div>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. FORKING (COLABORACIÓN OPEN SOURCE) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Forking: Colaborando con Desconocidos</SectionTitle>
      <TextBlock>
        ¿Qué pasa si quieres mejorar un proyecto famoso (como `navigation2`) pero no eres del equipo
        oficial? No tienes permiso para crear ramas ahí.
        <br /><br />
        Usas el <strong>Fork</strong>: una copia completa bajo tu control.
      </TextBlock>

      <div class="row q-mt-lg justify-center">
        <div class="col-12">
          <div class="tool-card fork-visual q-pa-xl bg-slate-900 relative-position overflow-hidden">
            <div class="row items-center justify-around relative-position z-top">
              <!-- OFFICIAL REPO -->
              <div class="repo-box bg-slate-800 border-red text-center">
                <q-icon name="verified_user" color="grey-5" size="2rem" />
                <div class="text-caption text-grey-4 q-mt-sm font-mono">ros-planning/nav2</div>
                <div class="text-xs text-red-4 text-weight-bold q-mt-xs">🔒 Read Only</div>
              </div>

              <!-- ARROWS CENTER -->
              <div class="column items-center justify-center q-mx-md gt-xs">
                <div class="row items-center q-mb-md">
                  <span class="text-xs text-grey-5 q-mr-sm">FORK</span>
                  <q-icon name="arrow_forward" color="white" />
                </div>
                <div class="row items-center">
                  <q-icon name="arrow_back" color="yellow-4" />
                  <span class="text-xs text-yellow-4 q-ml-sm">PULL REQUEST</span>
                </div>
              </div>

              <!-- MOBILE ARROWS -->
              <div class="col-12 text-center q-my-md lt-sm">
                <q-icon name="swap_vert" color="grey-6" size="2rem" />
              </div>

              <!-- YOUR REPO -->
              <div class="repo-box bg-slate-800 border-green text-center">
                <q-icon name="face" color="green-4" size="2rem" />
                <div class="text-caption text-green-1 q-mt-sm font-mono">alex/nav2</div>
                <div class="text-xs text-green-4 text-weight-bold q-mt-xs">🔓 Write Access</div>
              </div>
            </div>

            <!-- BACKGROUND DECORATION -->
            <div class="absolute-full flex flex-center" style="opacity: 0.05; pointer-events: none">
              <q-icon name="call_split" size="15rem" color="white" />
            </div>
          </div>
          <div class="text-center text-caption text-grey-5 q-mt-md">
            Haces cambios en TU copia (Derecha), y luego pides un PR hacia el original (Izquierda).
          </div>
        </div>
      </div>
    </div>

    <!-- 5. CHECKLIST ANTES DEL PR -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <div class="tool-card checklist-card q-pa-lg border-green-dashed bg-green-9-soft">
        <div class="text-h6 text-green-2 q-mb-lg text-center row items-center justify-center">
          <q-icon name="playlist_add_check" class="q-mr-sm" size="md" />
          Checklist Antes del PR
        </div>

        <div class="row q-col-gutter-lg">
          <div class="col-12 col-md-6">
            <div class="check-item row items-center bg-slate-900 q-pa-md rounded-borders shadow-1">
              <q-icon name="check_box" color="green-4" class="q-mr-md" size="sm" />
              <span class="text-grey-3">¿El código compila en mi PC?</span>
            </div>
            <div
              class="check-item row items-center bg-slate-900 q-pa-md rounded-borders shadow-1 q-mt-md"
            >
              <q-icon name="check_box" color="green-4" class="q-mr-md" size="sm" />
              <span class="text-grey-3">¿Eliminé los <code>print()</code> de depuración?</span>
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="check-item row items-center bg-slate-900 q-pa-md rounded-borders shadow-1">
              <q-icon name="check_box" color="green-4" class="q-mr-md" size="sm" />
              <span class="text-grey-3">¿El título del PR explica qué hace?</span>
            </div>
            <div
              class="check-item row items-center bg-slate-900 q-pa-md rounded-borders shadow-1 q-mt-md"
            >
              <q-icon name="check_box" color="green-4" class="q-mr-md" size="sm" />
              <span class="text-grey-3">¿Hice <code>pull origin main</code> antes?</span>
            </div>
          </div>
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

/* VISUAL PR CARD */
.pr-visual {
  border-top: 4px solid #4ade80;
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-dark {
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-green-glow {
  border: 1px solid #4ade80;
  box-shadow: 0 0 5px rgba(74, 222, 128, 0.3);
}
.border-left-grey {
  border-left: 2px solid #334155;
}
.bg-blue-9-soft {
  background: rgba(30, 58, 138, 0.5);
}
.bg-red-9-soft {
  background: rgba(127, 29, 29, 0.2);
}
.border-red-subtle {
  border: 1px solid rgba(248, 113, 113, 0.3);
}
.opacity-50 {
  opacity: 0.5;
}
.cursor-not-allowed {
  cursor: not-allowed;
}

/* REVIEW FLOW */
.review-flow {
  border-left: 4px solid #3b82f6;
}
.vertical-line {
  width: 2px;
  height: 25px;
  margin: 0 auto;
}
.border-left-blue {
  border-left: 3px solid #60a5fa;
}
.border-left-yellow {
  border-left: 3px solid #facc15;
}
.border-left-green {
  border-left: 3px solid #4ade80;
}
.shadow-glow-green {
  box-shadow: 0 0 15px rgba(74, 222, 128, 0.1);
}
.transition-hover {
  transition: transform 0.2s;
}
.group-hover:hover .transition-hover {
  transform: translateX(5px);
}

/* FORK VISUAL */
.fork-visual {
  border: 1px dashed rgba(255, 255, 255, 0.1);
}
.repo-box {
  padding: 20px;
  border-radius: 12px;
  width: 200px;
  transition: transform 0.3s;
}
.repo-box:hover {
  transform: translateY(-5px);
}
.border-red {
  border-top: 3px solid #ef4444;
}
.border-green {
  border-top: 3px solid #4ade80;
}
.z-top {
  z-index: 2;
}

/* CHECKLIST CARD */
.checklist-card {
  background: rgba(20, 83, 45, 0.1);
}
.border-green-dashed {
  border: 1px dashed rgba(74, 222, 128, 0.4);
}
.bg-green-9-soft {
  background: rgba(20, 83, 45, 0.15);
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

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .fork-visual .row {
    flex-direction: column;
  }
  .repo-box {
    width: 100%;
    margin: 10px 0;
  }
}
</style>
