<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-orange-4 text-weight-bold q-mb-sm">
          MÓDULO 3.1: LA MÁQUINA DEL TIEMPO
        </div>

        <h1 class="hero-title">Conceptos <span class="text-white">Fundamentales</span></h1>

        <TextBlock>
          Imagina trabajar en tu tesis y guardar archivos como <code>tesis_final.doc</code>,
          <code>tesis_final_v2.doc</code>, <code>tesis_ahora_si_final.doc</code>. Eso es el caos.
          <br /><br />
          <strong>Git</strong> es un sistema profesional para guardar "fotos" (snapshots) de tu
          código en el tiempo. Si rompes tu robot hoy, puedes regresar al estado exacto de ayer en
          milisegundos.
        </TextBlock>
      </div>
    </section>

    <!-- 2. LAS 3 ZONAS DE GIT (EL FLUJO DE TRABAJO) -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Las Tres Dimensiones</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Git no guarda todo automáticamente. Tienes que mover tus archivos conscientemente a
            través de tres zonas de seguridad.
            <br /><br />
            Es como preparar un paquete para envío:
          </TextBlock>
          <ol class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li><strong>Working Directory:</strong> Tu mesa de trabajo (Caos).</li>
            <li><strong>Staging Area:</strong> La caja de envío (Selección).</li>
            <li><strong>Repository:</strong> El camión de salida (Guardado Eterno).</li>
          </ol>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL FLOW -->
          <div class="tool-card flow-card relative-position q-pa-xl">
            <div class="row items-center justify-between relative-position">
              <!-- ZONA 1 -->
              <div class="col-3 text-center zone-box">
                <div class="icon-circle bg-red-9 shadow-2">
                  <q-icon name="edit_document" size="1.8rem" color="white" />
                </div>
                <div class="text-weight-bold text-red-4 q-mt-sm">Working</div>
                <div class="text-caption text-grey-6 text-xs">Sin guardar</div>
              </div>

              <!-- ARROW 1 -->
              <div class="col-auto text-center column flex-center" style="flex: 1">
                <div class="arrow-line"></div>
                <div class="cmd-pill bg-slate-700 text-code">git add</div>
              </div>

              <!-- ZONA 2 -->
              <div class="col-3 text-center zone-box">
                <div class="icon-circle bg-yellow-9 shadow-2">
                  <q-icon name="shopping_cart" size="1.8rem" color="black" />
                </div>
                <div class="text-weight-bold text-yellow-4 q-mt-sm">Staging</div>
                <div class="text-caption text-grey-6 text-xs">Preparado</div>
              </div>

              <!-- ARROW 2 -->
              <div class="col-auto text-center column flex-center" style="flex: 1">
                <div class="arrow-line"></div>
                <div class="cmd-pill bg-slate-700 text-code">git commit</div>
              </div>

              <!-- ZONA 3 -->
              <div class="col-3 text-center zone-box">
                <div class="icon-circle bg-green-9 shadow-2">
                  <q-icon name="inventory_2" size="1.8rem" color="white" />
                </div>
                <div class="text-weight-bold text-green-4 q-mt-sm">Repo</div>
                <div class="text-caption text-grey-6 text-xs">Historial</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. EL COMMIT (LA FOTO) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Anatomía de un Commit</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Un <strong>Commit</strong> no es solo "guardar". Es una captura completa del estado de
            todo tu proyecto en ese instante. <br /><br />
            Cada commit tiene un identificador único (Hash) que parece basura hexadecimal
            (<code>a1b2c3d</code>), pero es la huella digital matemática de tu código.
          </TextBlock>
          <div class="q-mt-lg">
            <AlertBlock type="info" title="El Mensaje es Vital">
              Debes explicar <em>qué</em> cambiaste y <em>por qué</em>. Un buen mensaje salva vidas.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="tool-card commit-card relative-position q-pa-lg">
            <!-- COMMIT NODE (CURRENT) -->
            <div class="row items-start no-wrap">
              <div class="col-auto column items-center">
                <div class="commit-circle bg-orange-5 shadow-glow-orange"></div>
                <div class="commit-line bg-grey-7"></div>
              </div>
              <div class="col q-pl-md">
                <div class="row items-center justify-between">
                  <span
                    class="text-orange-4 font-mono text-weight-bold bg-dark-soft q-px-sm rounded-borders"
                    >commit a1b2c3d</span
                  >
                  <span class="text-grey-5 text-caption">Hace 2 min</span>
                </div>
                <div class="text-white text-weight-bold q-my-sm text-subtitle1">
                  ✨ Añadir sensor LIDAR al robot
                </div>
                <div
                  class="text-grey-4 text-caption font-mono bg-black q-pa-sm rounded-borders border-light"
                >
                  Author: Alexander &lt;alex@robot.com&gt;<br />
                  <span class="text-green-4">+ src/lidar_driver.py</span><br />
                  <span class="text-green-4">+ config/params.yaml</span>
                </div>
              </div>
            </div>

            <!-- PREVIOUS COMMIT (FADED) -->
            <div class="row items-start no-wrap q-mt-sm opacity-50">
              <div class="col-auto column items-center">
                <div class="commit-circle bg-grey-6"></div>
              </div>
              <div class="col q-pl-md q-pt-xs">
                <div class="text-grey-5 font-mono text-weight-bold text-caption">
                  commit 9f8e7d6
                </div>
                <div class="text-grey-4 text-body2">🐛 Corregir velocidad de motores</div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. RAMAS (BRANCHES) - UNIVERSOS PARALELOS -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Universos Paralelos (Branches)</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            Aquí es donde Git brilla. Digamos que tienes un robot funcional (Rama
            <strong>Main</strong>). <br /><br />
            Quieres probar una nueva cámara experimental. Si lo haces en <code>main</code>, podrías
            romper el robot. <br /><br />
            En su lugar, creas una rama <code>feature/camara</code>. Es una copia exacta de la
            realidad. Puedes destrozar todo en esa rama; el <code>main</code> seguirá intacto.
          </TextBlock>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card visual-branch q-pa-lg relative-position overflow-hidden">
            <svg viewBox="0 0 400 160" class="branch-svg">
              <!-- LINEA MAIN -->
              <path d="M 20 120 L 380 120" stroke="#4ade80" stroke-width="3" fill="none" />
              <text
                x="30"
                y="110"
                fill="#4ade80"
                font-family="monospace"
                font-weight="bold"
                font-size="14"
              >
                main
              </text>

              <!-- PUNTOS MAIN -->
              <circle cx="50" cy="120" r="6" fill="#4ade80" />
              <circle cx="150" cy="120" r="6" fill="#4ade80" />
              <circle cx="350" cy="120" r="6" fill="#4ade80" />

              <!-- LINEA FEATURE -->
              <path
                d="M 150 120 Q 200 120 200 60 L 300 60 Q 350 60 350 110"
                stroke="#facc15"
                stroke-width="3"
                fill="none"
                stroke-dasharray="8,4"
              />
              <text
                x="210"
                y="50"
                fill="#facc15"
                font-family="monospace"
                font-weight="bold"
                font-size="14"
              >
                feature/camara
              </text>

              <!-- PUNTOS FEATURE -->
              <circle cx="250" cy="60" r="6" fill="#facc15" />

              <!-- ICONO MERGE -->
              <circle cx="350" cy="120" r="12" fill="none" stroke="white" stroke-width="2" />
            </svg>
            <div class="text-center text-caption text-grey-5 q-mt-xs font-italic">
              "Desviarse, experimentar y fusionar"
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. GIT VS GITHUB -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. Git no es GitHub</SectionTitle>
      <AlertBlock type="warning" title="No los confundas">
        Es el error más común.
        <br />
        <strong>Git</strong> es el software instalado en tu PC (Local).
        <br />
        <strong>GitHub</strong> es una página web propiedad de Microsoft donde subes tus
        repositorios (Nube).
      </AlertBlock>

      <div class="row q-mt-lg justify-center items-center q-gutter-xl text-center">
        <!-- PC -->
        <div class="column items-center">
          <div class="bg-slate-800 q-pa-lg rounded-borders border-light shadow-2">
            <q-icon name="computer" size="4rem" color="grey-5" />
          </div>
          <div class="text-h6 text-white q-mt-sm">Tu PC</div>
          <div class="text-caption text-orange-4 font-mono">(Git)</div>
        </div>

        <!-- ARROWS -->
        <div class="column items-center justify-center q-gutter-md">
          <div class="row items-center">
            <div class="cmd-pill bg-blue-9 q-mr-sm shadow-2">git push</div>
            <q-icon name="arrow_forward" size="1.5rem" color="blue-4" />
          </div>
          <div class="row items-center">
            <q-icon name="arrow_back" size="1.5rem" color="blue-4" />
            <div class="cmd-pill bg-blue-9 q-ml-sm shadow-2">git pull</div>
          </div>
        </div>

        <!-- CLOUD -->
        <div class="column items-center">
          <div class="bg-slate-800 q-pa-lg rounded-borders border-light shadow-2">
            <q-icon name="cloud_circle" size="4rem" color="white" />
          </div>
          <div class="text-h6 text-white q-mt-sm">Nube</div>
          <div class="text-caption text-orange-4 font-mono">(GitHub)</div>
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

/* FLOW CARD */
.flow-card {
  border-top: 4px solid #f97316;
}

.zone-box {
  transition: transform 0.3s;
  display: flex;
  flex-direction: column;
  align-items: center;
}
.zone-box:hover {
  transform: translateY(-5px);
}

.icon-circle {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 8px;
}

.arrow-line {
  height: 2px;
  background: rgba(255, 255, 255, 0.1);
  width: 100%;
  margin-bottom: -12px;
}
.cmd-pill {
  padding: 6px 12px;
  border-radius: 20px;
  font-family: 'Fira Code', monospace;
  font-size: 0.75rem;
  color: #cbd5e1;
  z-index: 2;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.bg-slate-700 {
  background: #334155;
}

/* COMMIT CARD */
.commit-card {
  border-left: 4px solid #f97316;
  background: #0f172a;
}

.commit-circle {
  width: 16px;
  height: 16px;
  border-radius: 50%;
  border: 2px solid #1e293b;
  z-index: 2;
}
.commit-line {
  width: 2px;
  height: 100%;
  min-height: 50px;
  margin-top: -2px;
  opacity: 0.3;
}
.opacity-50 {
  opacity: 0.5;
}
.shadow-glow-orange {
  box-shadow: 0 0 15px rgba(249, 115, 22, 0.5);
}

/* VISUAL BRANCH */
.visual-branch {
  background: #0f172a;
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.branch-svg {
  width: 100%;
  height: auto;
  max-height: 160px;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.bg-slate-800 {
  background: #1e293b;
}
.bg-dark-soft {
  background: rgba(255, 255, 255, 0.05);
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.text-xs {
  font-size: 0.8rem;
}
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 8px;
  font-size: 1rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .zone-box .text-caption {
    display: none;
  }
  .cmd-pill {
    font-size: 0.6rem;
    padding: 4px 8px;
  }
}
</style>
