<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 1.3: C++ BAJO EL CAPÓ
        </div>

        <h1 class="hero-title">Compilación en <span class="text-primary">C++</span></h1>

        <TextBlock>
          Python lee tu código línea por línea mientras funciona. C++ no. C++ traduce todo tu código
          a lenguaje de máquina (binario) antes de empezar. Entender este proceso de "traducción
          anticipada" es la clave para sobrevivir a los errores de <code>colcon build</code> en ROS
          2.
        </TextBlock>
      </div>
    </section>

    <!-- 2. INTERPRETADO VS COMPILADO -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Traductor vs. Intérprete</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            Imagina que tienes un libro en Ruso (Código Fuente) y quieres leerlo.
          </TextBlock>

          <ul class="tool-list q-mt-md">
            <li>
              <div class="row no-wrap items-start">
                <span class="q-mr-sm text-h6">🐍</span>
                <div>
                  <strong>Python (Intérprete):</strong> Contratas a un traductor que se sienta a tu
                  lado y te traduce frase por frase mientras lees. Si hay un error en la página 100,
                  no te enteras hasta que llegas ahí.
                </div>
              </div>
            </li>
            <li class="q-mt-md">
              <div class="row no-wrap items-start">
                <span class="q-mr-sm text-h6">⚙️</span>
                <div>
                  <strong>C++ (Compilador):</strong> Le das el libro al traductor. Él se lo lleva
                  una semana, lo traduce entero, lo imprime y te da un libro nuevo en Español
                  (Binario). Si hay un error, te lo dice <strong>antes</strong> de entregarte el
                  libro.
                </div>
              </div>
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-6">
          <!-- VISUALIZACIÓN COMPARATIVA -->
          <div class="tool-card visual-compare relative-position q-pa-lg">
            <div class="row items-stretch justify-around full-height">
              <!-- PYTHON FLOW -->
              <div class="column items-center justify-between" style="min-height: 200px">
                <div class="text-center">
                  <q-icon name="code" color="yellow-9" size="3rem" />
                  <div class="text-caption text-grey-4 q-mt-xs font-mono">Source.py</div>
                </div>

                <div class="column items-center text-grey-7">
                  <q-icon name="arrow_downward" size="1.5rem" />
                  <div class="text-caption text-italic">Tiempo Real</div>
                  <q-icon name="arrow_downward" size="1.5rem" />
                </div>

                <div
                  class="bg-yellow-9 text-black q-px-md q-py-sm rounded-borders text-weight-bold shadow-2"
                >
                  RUN 🏃
                </div>
              </div>

              <div
                class="vertical-separator bg-grey-8 self-center"
                style="width: 1px; height: 180px"
              ></div>

              <!-- C++ FLOW -->
              <div class="column items-center justify-between" style="min-height: 200px">
                <div class="text-center">
                  <q-icon name="code" color="blue-5" size="3rem" />
                  <div class="text-caption text-grey-4 q-mt-xs font-mono">Source.cpp</div>
                </div>

                <div class="column items-center">
                  <q-icon name="arrow_downward" color="accent" size="1.5rem" />
                  <div
                    class="bg-blue-9 text-white q-px-sm q-py-xs rounded-borders text-caption text-weight-bold q-my-xs"
                  >
                    COMPILE
                  </div>
                  <q-icon name="arrow_downward" color="accent" size="1.5rem" />
                </div>

                <div
                  class="bg-green-6 text-black q-px-md q-py-sm rounded-borders text-weight-bold shadow-glow"
                >
                  BINARY 🤖
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. EL PIPELINE DE COMPILACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El Ciclo de Vida: De Texto a Metal</SectionTitle>
      <AlertBlock type="info" title="🏭 La Fábrica Invisible">
        Cuando das click a "Compilar", ocurren 3 pasos invisibles. Los errores de ROS 2 suelen tener
        apellidos diferentes dependiendo de en qué paso fallaron.
      </AlertBlock>

      <div class="pipeline-container q-mt-lg row justify-center q-col-gutter-md">
        <!-- STEP 1: PREPROCESSING -->
        <div class="col-12 col-md-3">
          <div class="tool-card pipeline-step step-1 full-height">
            <div class="step-header text-yellow-4">1. Preprocesado</div>
            <div class="step-icon"><q-icon name="content_cut" /></div>
            <p class="text-caption text-grey-4 q-mb-none">
              Copia y pega todos los <code>#include</code> dentro de tu archivo. <br /><br /><span
                class="text-grey-6 text-italic"
                >Resultado: Un archivo gigante de texto puro.</span
              >
            </p>
          </div>
        </div>

        <!-- ARROW (Desktop) -->
        <div class="col-auto self-center gt-sm">
          <q-icon name="arrow_forward" color="grey-6" size="2rem" />
        </div>

        <!-- STEP 2: COMPILATION -->
        <div class="col-12 col-md-3">
          <div class="tool-card pipeline-step step-2 full-height">
            <div class="step-header text-blue-4">2. Compilación</div>
            <div class="step-icon"><q-icon name="g_translate" /></div>
            <p class="text-caption text-grey-4 q-mb-none">
              Traduce C++ a código de máquina, pero <strong>por separado</strong> para cada archivo.
              <br /><br /><span class="text-grey-6 text-italic"
                >Resultado: Archivos Objeto (<code>.o</code>).</span
              >
            </p>
          </div>
        </div>

        <!-- ARROW (Desktop) -->
        <div class="col-auto self-center gt-sm">
          <q-icon name="arrow_forward" color="grey-6" size="2rem" />
        </div>

        <!-- STEP 3: LINKING -->
        <div class="col-12 col-md-3">
          <div class="tool-card pipeline-step step-3 full-height">
            <div class="step-header text-green-4">3. Linker (Enlazado)</div>
            <div class="step-icon"><q-icon name="link" /></div>
            <p class="text-caption text-grey-4 q-mb-none">
              Une todos los pedazos (<code>.o</code>) y las librerías de ROS en un solo ejecutable.
              <br /><br /><span class="text-red-3 text-weight-bold"
                >Aquí ocurren el 90% de los errores feos.</span
              >
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 4. EL ARQUITECTO: CMAKE -->
    <div class="section-group self-stretch">
      <SectionTitle>3. El Arquitecto: CMake</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En un proyecto real de robótica, tienes cientos de archivos. Compilarlos a mano uno por
            uno sería una locura.
            <br /><br />
            <strong>CMake</strong> es la herramienta que automatiza esto. El archivo
            <code>CMakeLists.txt</code> es la "Receta" que le dice al compilador:
          </TextBlock>
          <ul class="tool-list q-pl-md q-mt-md">
            <li>📝 "Toma este archivo <code>cerebro.cpp</code>..."</li>
            <li>📚 "Mézclalo con la librería <code>rclcpp</code> de ROS..."</li>
            <li>🍳 "Y cocina un ejecutable llamado <code>nodo_cerebro</code>".</li>
          </ul>
        </template>

        <template #right>
          <div class="tool-card code-card q-pa-none overflow-hidden no-border-radius">
            <div class="bg-dark-soft q-px-md q-py-sm row items-center border-bottom-light">
              <q-icon name="description" color="grey-5" size="xs" class="q-mr-sm" />
              <span class="text-xs text-grey-5 font-mono">CMakeLists.txt</span>
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="cmake"
              content="cmake_minimum_required(VERSION 3.8)
project(mi_robot_cpp)

# 1. Encuentra las piezas de ROS
find_package(rclcpp REQUIRED)

# 2. Define el ejecutable
add_executable(nodo_cerebro src/cerebro.cpp)

# 3. Pega las piezas (Linker)
target_link_libraries(nodo_cerebro
  ${rclcpp_LIBRARIES}
)"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. ERRORES COMUNES -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>4. Diccionario de Errores</SectionTitle>
      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="tool-card error-card compilation-error full-height">
            <div class="row items-center q-mb-md">
              <div class="bg-red-9 q-pa-xs rounded-borders q-mr-sm">
                <q-icon name="cancel" color="white" size="1.2rem" />
              </div>
              <div class="text-subtitle1 text-white text-weight-bold">Compilation Error</div>
            </div>
            <p class="text-body2 text-grey-4">
              <em>"Syntax error", "Missing ;", "Undeclared variable".</em>
            </p>
            <div class="text-caption text-grey-5 q-mt-sm border-top-light q-pt-sm">
              <strong>Causa:</strong> Escribiste mal el código. El traductor no entiende tu
              gramática o vocabulario.
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="tool-card error-card linker-error full-height">
            <div class="row items-center q-mb-md">
              <div class="bg-orange-9 q-pa-xs rounded-borders q-mr-sm">
                <q-icon name="link_off" color="white" size="1.2rem" />
              </div>
              <div class="text-subtitle1 text-white text-weight-bold">Linker Error</div>
            </div>
            <p class="text-body2 text-grey-4">
              <em>"Undefined reference to...", "Symbol not found".</em>
            </p>
            <div class="text-caption text-grey-5 q-mt-sm border-top-light q-pt-sm">
              <strong>Causa:</strong> El código está bien escrito, pero
              <strong>falta una pieza</strong>. Olvidaste añadir la librería en el
              <code>target_link_libraries</code>.
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
import CodeBlock from 'components/content/CodeBlock.vue';
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
    radial-gradient(circle at center, rgba(59, 130, 246, 0.15), transparent 60%),
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

/* COMPARATIVE VISUAL */
.tool-card.visual-compare {
  border-top: 4px solid #facc15;
  background: linear-gradient(180deg, rgba(30, 41, 59, 0.4) 0%, rgba(15, 23, 42, 0.6) 100%);
}
.shadow-glow {
  box-shadow: 0 0 20px rgba(74, 222, 128, 0.3);
}

/* PIPELINE STEPS */
.pipeline-step {
  padding: 24px 16px;
  text-align: center;
  border-top: 4px solid #475569;
  transition: all 0.3s;
  display: flex;
  flex-direction: column;
  align-items: center;
}
.pipeline-step:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.7);
}

.pipeline-step.step-1 {
  border-color: #facc15;
}
.pipeline-step.step-2 {
  border-color: #3b82f6;
}
.pipeline-step.step-3 {
  border-color: #4ade80;
}

.step-icon {
  font-size: 2.5rem;
  color: #e2e8f0;
  margin: 16px 0;
  background: rgba(255, 255, 255, 0.05);
  padding: 12px;
  border-radius: 50%;
}
.step-header {
  font-weight: 800;
  font-family: 'Fira Code', monospace;
  margin-bottom: 8px;
  font-size: 1.1rem;
}

/* ERROR CARDS */
.tool-card.error-card {
  padding: 24px;
  border-left: 4px solid transparent;
  background: rgba(15, 23, 42, 0.6);
}
.tool-card.error-card.compilation-error {
  border-left-color: #ef4444;
  background: linear-gradient(90deg, rgba(239, 68, 68, 0.05), transparent);
}
.tool-card.error-card.linker-error {
  border-left-color: #f97316;
  background: linear-gradient(90deg, rgba(249, 115, 22, 0.05), transparent);
}

/* UTILIDADES */
.bg-dark-soft {
  background: rgba(0, 0, 0, 0.3);
}
.border-bottom-light {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-top-light {
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.75rem;
}

.tool-list {
  list-style: none;
  padding: 0;
  color: #cbd5e1;
}
.tool-list li {
  margin-bottom: 12px;
  font-size: 0.95rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
