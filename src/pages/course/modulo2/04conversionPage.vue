<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-purple-4 text-weight-bold q-mb-sm">
          MÓDULO 2.4: INGENIERÍA DE DATOS
        </div>

        <h1 class="hero-title">Conversión y <span class="text-white">Flujo</span></h1>

        <TextBlock>
          En el mundo real, los datos no se quedan quietos. Una configuración nace en
          <strong>YAML</strong>, es procesada por tu nodo en <strong>Python</strong> y termina
          visualizada en una web en <strong>JSON</strong>. Aprende el arte de la
          <strong>Serialización</strong>: cómo transformar cualquier formato en objetos manipulables
          y viceversa.
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL CONCEPTO: EL DICCIONARIO UNIVERSAL -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Diccionario como Hub Central</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            No intentamos "traducir" de YAML directamente a JSON. Eso es ineficiente.
            <br /><br />
            Lo que hacemos es llevar todo a un terreno neutral:
            <strong>El Diccionario de Python</strong> (o Hash Map en C++). <br /><br />
            Una vez que los datos están en una variable de Python, ya no importa de dónde vinieron.
            Son tuyos para manipular.
          </TextBlock>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL HUB -->
          <div class="tool-card hub-visual relative-position q-pa-xl">
            <div class="row justify-center items-center relative-position">
              <!-- INPUTS -->
              <div class="col-3 column q-gutter-md">
                <div class="format-badge bg-blue-9 text-caption text-white shadow-2">YAML</div>
                <div class="format-badge bg-yellow-9 text-black text-caption shadow-2">JSON</div>
                <div class="format-badge bg-red-9 text-caption text-white shadow-2">XML</div>
              </div>

              <!-- ARROWS IN -->
              <div class="col-2 column items-center justify-center q-gutter-sm">
                <q-icon name="chevron_right" size="1.5rem" color="grey-6" />
                <q-icon name="chevron_right" size="1.5rem" color="grey-6" />
                <q-icon name="chevron_right" size="1.5rem" color="grey-6" />
              </div>

              <!-- THE HUB (PYTHON) -->
              <div class="col-4 text-center z-top">
                <div class="python-hub shadow-glow">
                  <q-icon name="code" size="3rem" color="white" />
                  <div class="text-white text-weight-bold q-mt-xs font-mono">DICT</div>
                </div>
              </div>

              <!-- ARROWS OUT -->
              <div class="col-2 column items-center justify-center">
                <q-icon name="chevron_right" size="2rem" color="grey-6" />
              </div>

              <!-- OUTPUT -->
              <div class="col-1">
                <div
                  class="format-badge bg-green-6 text-black text-caption text-weight-bold shadow-2"
                >
                  APP
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. CÓDIGO: LEYENDO ARCHIVOS (DESERIALIZACIÓN) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Leyendo Archivos (Deserialización)</SectionTitle>
      <AlertBlock type="info" title="Librerías Estándar">
        Python ya trae baterías incluidas. No necesitas instalar nada extra para JSON. Para YAML,
        necesitarás <code>PyYAML</code> (que ROS 2 ya instala por ti).
      </AlertBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <!-- JSON READER -->
        <div class="col-12 col-md-6">
          <div class="tool-card code-card border-yellow">
            <div
              class="bg-yellow-9 text-black q-px-md q-py-sm text-subtitle2 flex justify-between items-center"
            >
              <span class="text-weight-bold">Leer JSON</span>
              <q-icon name="data_object" />
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="python"
              content="import json

# 1. Abrir el archivo
with open('data.json', 'r') as f:
    # 2. Convertir a Diccionario
    datos = json.load(f)

print(datos['velocidad'])  # Acceso fácil"
            />
          </div>
        </div>

        <!-- YAML READER -->
        <div class="col-12 col-md-6">
          <div class="tool-card code-card border-blue">
            <div
              class="bg-blue-9 text-white q-px-md q-py-sm text-subtitle2 flex justify-between items-center"
            >
              <span class="text-weight-bold">Leer YAML</span>
              <q-icon name="settings" />
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="python"
              content="import yaml

# 1. Abrir el archivo
with open('config.yaml', 'r') as f:
    # 2. Convertir (Safe Load es vital)
    config = yaml.safe_load(f)

print(config['ros__parameters'])"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- 4. FLUJO COMPLETO: EL CASO DE USO ROS 2 -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. El Pipeline de la Vida Real</SectionTitle>
      <TextBlock>
        Imagina este escenario común en robótica móvil: Cargas la configuración máxima de velocidad
        desde un archivo <strong>YAML</strong>, tu nodo de <strong>Python</strong> calcula la ruta,
        y envías el estado actual a una tablet vía Wi-Fi en formato <strong>JSON</strong>.
      </TextBlock>

      <div class="tool-card pipeline-card q-mt-lg q-pa-lg">
        <div class="row items-center justify-between q-col-gutter-md">
          <!-- STEP 1 -->
          <div class="col-12 col-md-3 text-center">
            <div class="text-h6 text-blue-4 q-mb-sm">1. Entrada</div>
            <div class="bg-slate-900 q-pa-md rounded-borders border-blue">
              <q-icon name="description" size="2.5rem" color="blue-4" />
              <div class="code-mini bg-black q-mt-sm text-blue-2">speed: 5.0</div>
              <div class="text-caption text-grey-5 q-mt-xs font-mono">config.yaml</div>
            </div>
          </div>

          <div class="col-auto gt-sm">
            <q-icon name="arrow_forward" size="2rem" color="grey-7" />
          </div>

          <!-- STEP 2 -->
          <div class="col-12 col-md-4 text-center">
            <div class="text-h6 text-green-4 q-mb-sm">2. Proceso (Python)</div>
            <div
              class="bg-slate-900 q-pa-md rounded-borders font-mono text-xs text-left border-green shadow-lg"
            >
              <span class="text-purple-4">def</span>
              <span class="text-yellow-4">main</span>():<br />
              &nbsp;&nbsp;limit = yaml_data[<span class="text-green-4">'speed'</span>]<br />
              &nbsp;&nbsp;current = get_sensor()<br />
              &nbsp;&nbsp;<span class="text-grey-5 text-italic"># Lógica...</span><br />
              &nbsp;&nbsp;msg = {<span class="text-green-4">'val'</span>: current}
            </div>
          </div>

          <div class="col-auto gt-sm">
            <q-icon name="arrow_forward" size="2rem" color="grey-7" />
          </div>

          <!-- STEP 3 -->
          <div class="col-12 col-md-3 text-center">
            <div class="text-h6 text-yellow-4 q-mb-sm">3. Salida</div>
            <div class="bg-slate-900 q-pa-md rounded-borders border-yellow">
              <q-icon name="wifi" size="2.5rem" color="yellow-4" />
              <div class="code-mini bg-black q-mt-sm text-yellow-2">{"val": 2.1}</div>
              <div class="text-caption text-grey-5 q-mt-xs font-mono">WebSocket</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. HERRAMIENTAS DE CONSOLA (BONUS) -->
    <div class="section-group self-stretch q-mb-xl q-mt-xl">
      <SectionTitle>4. Herramientas de Supervivencia (CLI)</SectionTitle>
      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-800 q-pa-md row items-center no-wrap">
            <div class="col-auto q-mr-md">
              <div
                class="bg-black q-pa-sm rounded-borders text-h5 text-green-4 font-mono border-green"
              >
                jq
              </div>
            </div>
            <div class="col">
              <div class="text-weight-bold text-white">JSON Processor</div>
              <div class="text-caption text-grey-4">
                Comando mágico para leer, filtrar y colorear JSON en la terminal.
              </div>
              <div
                class="text-code q-mt-xs bg-black q-px-sm rounded-borders text-grey-5 font-mono text-caption q-py-xs inline-block"
              >
                cat data.json | jq
              </div>
            </div>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-800 q-pa-md row items-center no-wrap">
            <div class="col-auto q-mr-md">
              <div
                class="bg-black q-pa-sm rounded-borders text-h5 text-blue-4 font-mono border-blue"
              >
                yq
              </div>
            </div>
            <div class="col">
              <div class="text-weight-bold text-white">YAML Processor</div>
              <div class="text-caption text-grey-4">
                Lo mismo que jq, pero para YAML. Imprescindible para ver archivos de configuración
                largos.
              </div>
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

/* HUB VISUAL */
.hub-visual {
  border-top: 4px solid #a855f7;
  background: #0f172a;
}

.format-badge {
  padding: 8px;
  border-radius: 6px;
  text-align: center;
  font-weight: bold;
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.python-hub {
  width: 100px;
  height: 100px;
  background: linear-gradient(135deg, #a855f7 0%, #7c3aed 100%);
  border-radius: 50%;
  border: 4px solid #1e293b;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  margin: 0 auto;
}
.shadow-glow {
  box-shadow: 0 0 40px rgba(168, 85, 247, 0.4);
}

/* CODE CARDS */
.tool-card.code-card {
  overflow: hidden;
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-yellow {
  border-color: #facc15;
}
.border-blue {
  border-color: #3b82f6;
}
.border-green {
  border-color: #22c55e;
}

/* PIPELINE CARD */
.pipeline-card {
  border: 1px dashed rgba(255, 255, 255, 0.2);
  background: rgba(15, 23, 42, 0.6);
}
.code-mini {
  padding: 4px;
  border-radius: 4px;
  font-family: monospace;
  font-size: 0.8rem;
}
.bg-slate-900 {
  background: #0f172a;
}

.text-xs {
  font-size: 0.75rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.z-top {
  z-index: 10;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .hub-visual .row {
    flex-direction: column;
    gap: 10px;
  }
  .hub-visual .col-2 {
    transform: rotate(90deg);
    margin: 10px 0;
  }
  .pipeline-card .row {
    flex-direction: column;
    gap: 20px;
  }
}
</style>
