<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-blue-4 text-weight-bold q-mb-sm">
          MÓDULO 2.3: EL ZEN MINIMALISTA
        </div>

        <h1 class="hero-title">YAML <span class="text-white">Básico</span></h1>

        <TextBlock>
          YAML (YAML Ain't Markup Language) es el formato diseñado para <strong>humanos</strong>. En
          ROS 2, es el rey de la configuración. Se usa para definir parámetros de navegación,
          configurar simulaciones en Gazebo y guardar mapas. Su superpoder: elimina todo el ruido
          visual (llaves, comillas, etiquetas). Su debilidad:
          <strong>la indentación es ley</strong>.
        </TextBlock>
      </div>
    </section>

    <!-- 2. FILOSOFÍA: MENOS ES MÁS -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Anatomía: La Tiranía del Espacio</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock> YAML usa la sangría (indentación) para definir la jerarquía. </TextBlock>

          <ul class="tool-list q-mt-md">
            <li>
              🚫 <strong>Tabuladores Prohibidos:</strong> Nunca uses la tecla TAB. YAML solo acepta
              espacios (usualmente 2 espacios por nivel).
            </li>
            <li>
              ➖ <strong>Guiones para Listas:</strong> Si ves un guion <code>-</code>, es un
              elemento de una lista.
            </li>
            <li>
              🔡 <strong>Clave-Valor Limpio:</strong> <code>key: value</code>. Nota el espacio
              obligatorio después de los dos puntos.
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL RULER (INDENTATION) -->
          <div class="tool-card ruler-card relative-position q-pa-xl overflow-hidden">
            <div class="font-mono text-body1 relative-position" style="z-index: 2">
              <!-- NIVEL 0 -->
              <div class="row items-center">
                <span class="text-blue-4 text-weight-bold">robot_config:</span>
              </div>

              <!-- NIVEL 1 -->
              <div class="row items-center">
                <span class="indent-guide">··</span>
                <span class="text-purple-4">sensores:</span>
              </div>

              <!-- NIVEL 2 (LISTA) -->
              <div class="row items-center">
                <span class="indent-guide">··</span>
                <span class="indent-guide">··</span>
                <span class="text-grey-5">-</span>
                <span class="text-green-4 q-ml-xs">lidar_frontal</span>
              </div>
              <div class="row items-center">
                <span class="indent-guide">··</span>
                <span class="indent-guide">··</span>
                <span class="text-grey-5">-</span>
                <span class="text-green-4 q-ml-xs">camara_rgb</span>
              </div>

              <!-- NIVEL 1 (PROPIEDAD) -->
              <div class="row items-center q-mt-sm">
                <span class="indent-guide">··</span>
                <span class="text-purple-4">velocidad_max:</span>
                <span class="text-orange-4 q-ml-sm">2.5</span>
              </div>
            </div>

            <!-- VISUAL GUIDES (LINES) -->
            <div class="absolute-full row" style="top: 30px; left: 56px; pointer-events: none">
              <div class="col-auto bg-white opacity-10" style="width: 1px; height: 80%"></div>
              <div
                class="col-auto bg-white opacity-10"
                style="width: 1px; height: 60%; margin-left: 19px"
              ></div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. EL PELIGRO DE LOS TABS -->
    <div class="section-group self-stretch">
      <AlertBlock type="danger" title="💀 Muerte por Tabulador">
        Si mezclas espacios y tabs en un archivo <code>.yaml</code>, ROS 2 te lanzará un error de
        "Parser Error" que no te dice dónde fallaste. Configura tu editor (VS Code) para que
        convierta automáticamente los Tabs en Espacios.
      </AlertBlock>
    </div>

    <!-- 4. CASO REAL: PARÁMETROS ROS 2 -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>2. En la Práctica: Archivos de Parámetros (.yaml)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En ROS 2, no configuramos variables dentro del código C++ o Python (eso requeriría
            recompilar). Usamos archivos externos <code>params.yaml</code>. <br /><br />
            <strong>La Estructura Mágica:</strong>
            Para que ROS 2 lea el archivo, debes seguir una jerarquía estricta:
          </TextBlock>
          <ol class="q-pl-md q-mt-sm text-grey-4 tool-list">
            <li>Nombre del Nodo (o Namespace)</li>
            <li>Palabra clave <code>ros__parameters</code> (doble guion bajo)</li>
            <li>Tus variables reales.</li>
          </ol>
        </template>

        <template #right>
          <div class="tool-card code-card q-pa-none border-blue">
            <div
              class="bg-blue-9 text-white q-px-md q-py-sm text-subtitle2 flex justify-between items-center border-bottom-light"
            >
              <span class="font-mono text-weight-bold">navigation_params.yaml</span>
              <q-icon name="tune" />
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="yaml"
              content='amcl:  # <--- Nombre del Nodo
  ros__parameters:  # <--- ¡OBLIGATORIO!
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2

    # Objeto anidado
    laser_model:
      type: "likelihood_field"
      max_beams: 30

controller_server:
  ros__parameters:
    max_vel_x: 0.26'
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. ANCLAS Y ALIAS (DRY) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Pro Tip: No te Repitas (Anclas)</SectionTitle>
      <TextBlock>
        Imagina que tienes 4 ruedas y todas necesitan la misma configuración PID. YAML permite
        copiar y pegar configuraciones usando <strong>Anclas (&)</strong> y
        <strong>Alias (*)</strong>.
      </TextBlock>

      <div class="row q-mt-md">
        <div class="col-12">
          <div class="tool-card bg-slate-900 q-pa-lg">
            <div class="row q-col-gutter-lg">
              <!-- DEFINICIÓN -->
              <div class="col-12 col-md-6 border-right-dashed">
                <div class="text-subtitle2 text-blue-4 q-mb-sm text-weight-bold">
                  DEFINICIÓN (&)
                </div>
                <!-- CORREGIDO: lang & content -->
                <CodeBlock
                  lang="yaml"
                  :show-line-numbers="false"
                  content="config_rueda: &conf_comun
  radio: 0.15
  friccion: 0.8
  pid: [10.0, 0.1, 0.5]"
                />
                <p class="text-caption text-grey-5 q-mt-sm">
                  Creamos un molde llamado <code>conf_comun</code>.
                </p>
              </div>

              <!-- USO -->
              <div class="col-12 col-md-6">
                <div class="text-subtitle2 text-green-4 q-mb-sm text-weight-bold">
                  REUTILIZACIÓN (*)
                </div>
                <!-- CORREGIDO: lang & content -->
                <CodeBlock
                  lang="yaml"
                  :show-line-numbers="false"
                  content="rueda_izquierda:
  <<: *conf_comun   # Copia todo

rueda_derecha:
  <<: *conf_comun   # Copia todo"
                />
                <p class="text-caption text-grey-5 q-mt-sm">
                  ROS leerá esto como si hubieras escrito todo el bloque dos veces.
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 6. RESUMEN VISUAL -->
    <div class="section-group self-stretch q-mb-xl q-mt-xl">
      <div class="tool-card full-width q-pa-lg border-blue row items-center bg-gradient-blue">
        <div class="col-12 col-md-8">
          <div class="text-h6 text-white">¿Por qué usamos YAML en ROS 2?</div>
          <p class="text-grey-2 q-mt-sm">
            Porque permite cambiar el comportamiento del robot (velocidad, mapa, sensores)
            <strong>sin tocar una sola línea de código C++ o Python</strong>. Es la interfaz entre
            el programador y el operador.
          </p>
        </div>
        <div class="col-12 col-md-4 text-center">
          <q-icon name="settings_remote" size="5rem" color="white" class="opacity-50" />
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

/* RULER CARD (VISUAL INDENTATION) */
.ruler-card {
  border-top: 4px solid #3b82f6;
  background: #0f172a;
}

.indent-guide {
  color: rgba(255, 255, 255, 0.1);
  margin-right: 4px;
  font-family: monospace;
  user-select: none;
}
.opacity-10 {
  opacity: 0.1;
}

/* CODE CARD */
.tool-card.code-card {
  overflow: hidden;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.tool-card.border-blue {
  border-color: #3b82f6;
}

/* ANCHOR CARD */
.bg-slate-900 {
  background: #0f172a;
}
.border-right-dashed {
  border-right: 1px dashed rgba(255, 255, 255, 0.2);
}

/* GRADIENT CARD */
.bg-gradient-blue {
  background: linear-gradient(135deg, rgba(30, 41, 59, 0.6) 0%, rgba(37, 99, 235, 0.2) 100%);
  border: 1px solid #3b82f6;
}
.opacity-50 {
  opacity: 0.5;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.border-bottom-light {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.tool-list {
  list-style: none;
  padding: 0;
  color: #cbd5e1;
}
.tool-list li {
  margin-bottom: 8px;
  font-size: 0.95rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .border-right-dashed {
    border-right: none;
    border-bottom: 1px dashed rgba(255, 255, 255, 0.2);
    padding-bottom: 24px;
    margin-bottom: 24px;
  }
}
</style>
