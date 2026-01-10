<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-orange-4 text-weight-bold q-mb-sm">
          MÓDULO 3.2: LA BITÁCORA
        </div>

        <h1 class="hero-title">Creando <span class="text-white">Commits</span></h1>

        <TextBlock>
          Un "Commit" es mucho más que guardar archivos. Es una cápsula del tiempo sellada con un
          mensaje explicativo. Aprende el ritual sagrado de los tres pasos:
          <strong>Status, Add y Commit</strong>, y descubre por qué escribir "cambios varios" en el
          mensaje puede hacer que te despidan.
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL RITUAL DE LOS 3 PASOS -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Ritual de Confirmación</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Git no adivina qué quieres guardar. Tienes que decírselo explícitamente.
            <br /><br />
            Este ciclo se repite docenas de veces al día:
          </TextBlock>
          <ol class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>🔍 <strong>Check:</strong> ¿Qué he modificado? (`git status`)</li>
            <li>📦 <strong>Stage:</strong> Elegir qué archivos van a la caja (`git add`)</li>
            <li>📸 <strong>Snap:</strong> Sellar la caja y etiquetarla (`git commit`)</li>
          </ol>
        </div>

        <div class="col-12 col-md-7">
          <!-- TERMINAL SIMULATION FLOW -->
          <div class="tool-card terminal-flow bg-slate-900 q-pa-none overflow-hidden shadow-2">
            <!-- Header Fake -->
            <div
              class="bg-black q-px-md q-py-xs row items-center justify-between border-bottom-dark"
            >
              <div class="row q-gutter-xs">
                <div class="control red"></div>
                <div class="control yellow"></div>
                <div class="control green"></div>
              </div>
              <div class="text-caption text-grey-7">bash — 80x24</div>
            </div>

            <div class="q-pa-md font-mono text-sm">
              <!-- STEP 1: STATUS -->
              <div class="cmd-block q-mb-md">
                <div class="text-green-4 text-weight-bold">$ git status</div>
                <div class="text-red-4 q-pl-md q-mt-xs">
                  modified: src/motor_driver.py<br />
                  modified: config/params.yaml
                </div>
                <div
                  class="text-caption text-grey-6 text-right italic q-mt-xs border-top-dashed q-pt-xs"
                >
                  // "¡Mira, cambiaste esto!"
                </div>
              </div>

              <!-- STEP 2: ADD -->
              <div class="cmd-block q-mb-md">
                <div class="text-green-4 text-weight-bold">$ git add src/motor_driver.py</div>
                <div
                  class="text-caption text-grey-6 text-right italic q-mt-xs border-top-dashed q-pt-xs"
                >
                  // "Ok, prepara solo el motor. Deja la config."
                </div>
              </div>

              <!-- STEP 3: COMMIT -->
              <div class="cmd-block">
                <div class="text-green-4 text-weight-bold">
                  $ git commit -m "Corregir velocidad"
                </div>
                <div class="text-white q-pl-md q-mt-xs">
                  [main a1b2c] Corregir velocidad<br />
                  1 file changed, 12 insertions(+)
                </div>
                <div
                  class="text-caption text-grey-6 text-right italic q-mt-xs border-top-dashed q-pt-xs"
                >
                  // "¡Guardado en la historia!"
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. MENSAJES DE COMMIT (BUENOS VS MALOS) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Arte del Mensaje</SectionTitle>
      <AlertBlock type="info" title="Regla de Oro">
        Escribe el mensaje pensando en tu "Yo del futuro". Dentro de 6 meses, cuando el robot falle,
        ¿te servirá leer "arreglos"? No.
      </AlertBlock>

      <div class="row q-col-gutter-lg q-mt-sm">
        <!-- BAD EXAMPLES -->
        <div class="col-12 col-md-6">
          <div class="tool-card bad-commit relative-position full-height">
            <div class="absolute-top-right q-pa-md">
              <q-icon name="thumb_down" color="red-4" size="1.5rem" />
            </div>
            <div class="text-subtitle1 text-red-4 q-mb-md text-weight-bold">❌ El Vagabundo</div>
            <ul class="example-list text-grey-4">
              <li>"cambios"</li>
              <li>"update"</li>
              <li>"listo"</li>
              <li>"asdfgh"</li>
              <li>"arreglando el bug de ayer"</li>
            </ul>
            <div
              class="q-mt-auto text-caption text-red-3 bg-dark-soft q-pa-sm rounded-borders q-mt-md"
            >
              No dicen <strong>QUÉ</strong> cambió ni <strong>POR QUÉ</strong>.
            </div>
          </div>
        </div>

        <!-- GOOD EXAMPLES -->
        <div class="col-12 col-md-6">
          <div class="tool-card good-commit relative-position full-height">
            <div class="absolute-top-right q-pa-md">
              <q-icon name="thumb_up" color="green-4" size="1.5rem" />
            </div>
            <div class="text-subtitle1 text-green-4 q-mb-md text-weight-bold">
              ✅ El Profesional
            </div>
            <ul class="example-list text-white">
              <li>"Fix: corregir división por cero"</li>
              <li>"Feat: añadir soporte para cámara"</li>
              <li>"Docs: actualizar diagrama"</li>
              <li>"Refactor: simplificar bucle"</li>
            </ul>
            <div
              class="q-mt-auto text-caption text-green-3 bg-dark-soft q-pa-sm rounded-borders q-mt-md"
            >
              Usa verbos imperativos y prefijos estándar.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. ATOMIC COMMITS -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Divide y Vencerás (Atomic Commits)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Imagina que arreglas la rueda del robot Y TAMBIÉN cambias el color de la interfaz en el
            mismo trabajo.
            <br /><br />
            Si luego te das cuenta de que el cambio de color rompió la pantalla, no puedes
            deshacerlo sin deshacer también el arreglo de la rueda.
          </TextBlock>
          <div class="q-mt-md">
            <AlertBlock type="warning" title="La Regla Atómica">
              Un Commit = Una Tarea Lógica Indivisible.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div
            class="visual-atomic row q-col-gutter-md justify-center bg-slate-900 q-pa-lg rounded-borders border-light"
          >
            <!-- BIG BAD BLOCK -->
            <div class="col-12 text-center q-mb-md">
              <div class="text-caption text-red-4 q-mb-xs text-weight-bold">
                Commit Monolítico (Mal)
              </div>
              <div
                class="bg-red-9 q-pa-md rounded-borders text-white shadow-2 row justify-center q-gutter-x-md items-center"
              >
                <div>🛠️ Ruedas</div>
                <div class="text-grey-4">+</div>
                <div>🎨 Colores</div>
                <div class="text-grey-4">+</div>
                <div>📄 Docs</div>
              </div>
              <div class="text-xs text-red-3 q-mt-sm">Imposible de separar después</div>
            </div>

            <div class="col-12 flex flex-center">
              <q-icon name="arrow_downward" color="grey-6" size="2rem" />
            </div>

            <!-- ATOMIC BLOCKS -->
            <div class="col-4 text-center">
              <div
                class="bg-green-8 q-pa-sm rounded-borders text-xs text-white shadow-2 full-height flex flex-center column"
              >
                <div class="text-weight-bold q-mb-xs">Commit 1</div>
                <div>🛠️ Ruedas</div>
              </div>
            </div>
            <div class="col-4 text-center">
              <div
                class="bg-green-8 q-pa-sm rounded-borders text-xs text-white shadow-2 full-height flex flex-center column"
              >
                <div class="text-weight-bold q-mb-xs">Commit 2</div>
                <div>🎨 Colores</div>
              </div>
            </div>
            <div class="col-4 text-center">
              <div
                class="bg-green-8 q-pa-sm rounded-borders text-xs text-white shadow-2 full-height flex flex-center column"
              >
                <div class="text-weight-bold q-mb-xs">Commit 3</div>
                <div>📄 Docs</div>
              </div>
            </div>
            <div class="col-12 text-center text-xs text-green-4 q-mt-sm text-weight-bold">
              ✨ Flexibilidad Total
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. VER LA HISTORIA (GIT LOG) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. Viajando al Pasado: Git Log</SectionTitle>
      <TextBlock>
        ¿De qué sirve escribir bitácoras si nunca las lees? Usa <code>git log</code> para ver la
        historia.
      </TextBlock>

      <div class="tool-card log-card q-mt-md bg-black shadow-glow">
        <div class="bg-dark-soft q-px-sm q-py-xs row items-center border-bottom-dark">
          <q-icon name="history" color="grey-6" size="xs" class="q-mr-xs" />
          <span class="text-caption text-grey-6 font-mono">git log --oneline --graph</span>
        </div>

        <div class="q-pa-md font-mono text-sm">
          <div class="row items-center q-mb-xs">
            <span class="text-red-4 q-mr-sm">*</span>
            <span class="text-yellow-4 q-mr-sm">a1b2c3d</span>
            <span class="text-white text-weight-bold">(HEAD -> main) Fix: corregir odometría</span>
          </div>
          <div class="row items-center q-mb-xs">
            <span class="text-red-4 q-mr-sm">*</span>
            <span class="text-yellow-4 q-mr-sm">4f5g6h7</span>
            <span class="text-grey-4">Feat: añadir nodo de cámara</span>
          </div>
          <div class="row items-center">
            <span class="text-red-4 q-mr-sm">*</span>
            <span class="text-yellow-4 q-mr-sm">8i9j0k1</span>
            <span class="text-grey-5">Docs: readme inicial</span>
          </div>
        </div>
      </div>

      <div class="text-center q-mt-lg">
        <div class="cmd-pill bg-slate-700 text-grey-4 border-light inline-block shadow-1">
          💡 Pro Tip: Usa la tecla <strong>'q'</strong> para salir del log
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

/* TERMINAL FLOW */
.terminal-flow {
  border-left: 4px solid #4ade80;
}
.control {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}
.control.red {
  background: #ff5f56;
}
.control.yellow {
  background: #ffbd2e;
}
.control.green {
  background: #27c93f;
}

.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-dashed {
  border-top: 1px dashed rgba(255, 255, 255, 0.1);
}

/* GOOD/BAD COMMITS */
.bad-commit {
  border: 1px solid rgba(248, 113, 113, 0.3);
  background: rgba(248, 113, 113, 0.05);
  padding: 24px;
}
.good-commit {
  border: 1px solid rgba(74, 222, 128, 0.3);
  background: rgba(74, 222, 128, 0.05);
  padding: 24px;
}

.example-list {
  padding-left: 20px;
  margin: 0;
}
.example-list li {
  margin-bottom: 8px;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
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
.italic {
  font-style: italic;
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-dark-soft {
  background: rgba(0, 0, 0, 0.2);
}
.bg-slate-700 {
  background: #334155;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.shadow-glow {
  box-shadow: 0 0 20px rgba(0, 0, 0, 0.5);
}
.inline-block {
  display: inline-block;
}
.cmd-pill {
  padding: 6px 16px;
  border-radius: 20px;
  font-size: 0.85rem;
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
