<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 1.1: EL LENGUAJE DE LAS MÁQUINAS
        </div>

        <h1 class="hero-title">Scripts de <span class="text-primary">Python</span></h1>

        <TextBlock>
          Un script de Python en reposo es solo texto. En ejecución, es el cerebro de tu robot.
          Aprende el ritual para darle vida: desde la "Línea Maestra" (Shebang) hasta la gestión de
          memoria.
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA DE UN SCRIPT -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Anatomía de un Archivo Ejecutable</SectionTitle>
      <div class="row q-col-gutter-lg items-stretch">
        <!-- CÓDIGO VISUAL INTERACTIVO -->
        <div class="col-12 col-md-7">
          <div class="tool-card code-anatomy q-pa-none overflow-hidden">
            <!-- Barra de título ventana -->
            <div class="window-bar row items-center q-px-md q-py-sm bg-dark-soft">
              <div class="row q-gutter-xs">
                <div class="circle bg-red-5"></div>
                <div class="circle bg-yellow-5"></div>
                <div class="circle bg-green-5"></div>
              </div>
              <div class="col text-center text-caption text-grey-5">robot_brain.py</div>
            </div>

            <!-- Contenido del Código -->
            <div class="code-content q-pa-md font-mono text-body2">
              <!-- SHEBANG -->
              <div class="line-group relative-position">
                <span class="line-num">1</span>
                <span class="token-comment">#!/usr/bin/env python3</span>
                <q-tooltip anchor="center right" self="center left" class="bg-pink-9 text-body2">
                  <strong>EL SHEBANG:</strong> Le dice a Linux qué intérprete usar.
                </q-tooltip>
              </div>

              <div class="line-group"><span class="line-num">2</span></div>

              <!-- IMPORTS -->
              <div class="line-group relative-position">
                <span class="line-num">3</span> <span class="token-keyword">import</span> time
                <q-tooltip anchor="center right" self="center left" class="bg-purple-9 text-body2">
                  <strong>LIBRERÍAS:</strong> Herramientas necesarias.
                </q-tooltip>
              </div>

              <div class="line-group"><span class="line-num">4</span></div>

              <!-- MAIN FUNCTION -->
              <div class="line-group">
                <span class="line-num">5</span> <span class="token-keyword">def</span>
                <span class="token-func">main</span>():
              </div>
              <div class="line-group">
                <span class="line-num">6</span> &nbsp;&nbsp;&nbsp;&nbsp;<span class="token-func"
                  >print</span
                >(<span class="token-string">"Sistema Iniciado..."</span>)
              </div>
              <div class="line-group">
                <span class="line-num">7</span> &nbsp;&nbsp;&nbsp;&nbsp;<span class="token-comment"
                  ># Tu lógica de robot aquí</span
                >
              </div>

              <div class="line-group"><span class="line-num">8</span></div>

              <!-- ENTRY POINT -->
              <div class="line-group highlight-bg relative-position">
                <span class="line-num">9</span> <span class="token-keyword">if</span> __name__ ==
                <span class="token-string">'__main__'</span>:
                <q-tooltip anchor="center right" self="center left" class="bg-blue-9 text-body2">
                  <strong>ENTRY POINT:</strong> El seguro del arma. Evita ejecución accidental al
                  importar.
                </q-tooltip>
              </div>
              <div class="line-group">
                <span class="line-num">10</span>&nbsp;&nbsp;&nbsp;&nbsp;<span class="token-func"
                  >main</span
                >()
              </div>
            </div>
          </div>
        </div>

        <!-- EXPLICACIÓN LATERAL -->
        <div class="col-12 col-md-5">
          <div class="column justify-center full-height">
            <TextBlock>
              A diferencia de un script web, un script de robótica es un
              <strong>proceso continuo</strong>. <br /><br />
              Observa la estructura. No escribimos código suelto. Todo vive dentro de funciones.
              <br /><br />
              El bloque <code>if __name__ == '__main__':</code> es vital en ROS 2. Garantiza que el
              código solo corra si tú lo llamas directamente.
            </TextBlock>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. EL SHEBANG -->
    <div class="section-group self-stretch">
      <SectionTitle>2. El "Shebang": La Línea Sagrada</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En Linux, la extensión <code>.py</code> es decorativa. El sistema lee la primera línea
            (Shebang <code>#!</code>) para saber cómo ejecutar el archivo. <br /><br />
            Sin ella, Linux intentará leer tu Python como si fuera Bash y fallará.
          </TextBlock>
        </template>
        <template #right>
          <AlertBlock type="info" title="✅ La Forma Correcta">
            Usa la versión "env". Busca Python donde sea que esté instalado (vital para entornos
            virtuales).
          </AlertBlock>
          <!-- CORREGIDO: language -> lang, code -> content -->
          <CodeBlock lang="python" content="#!/usr/bin/env python3" />

          <div class="q-mt-sm">
            <AlertBlock type="danger" title="❌ La Forma Rígida">
              Evita rutas absolutas como <code>#!/usr/bin/python3</code>. Puede fallar en otros PCs.
            </AlertBlock>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. PERMISOS DE EJECUCIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Convertir Texto en Acción (+x)</SectionTitle>
      <TextBlock>
        Por seguridad, Linux crea los archivos como "Solo lectura/escritura".
        <strong>Nadie puede ejecutarlos por defecto.</strong> Debes bendecir el archivo con
        <code>chmod</code>.
      </TextBlock>

      <div class="row items-center justify-center q-gutter-xl q-mt-lg">
        <!-- ESTADO INACTIVO -->
        <div class="tool-card file-state dead text-center" style="width: 160px">
          <q-icon name="description" size="3rem" class="text-grey-6" />
          <div class="text-weight-bold q-mt-sm text-grey-5">script.py</div>
          <div class="text-caption font-mono text-grey-6">rw-r--r--</div>
          <q-badge color="grey-8" class="q-mt-sm">Texto Muerto</q-badge>
        </div>

        <!-- TRANSICIÓN -->
        <div class="column items-center">
          <div class="text-accent text-h6 font-mono text-weight-bold">chmod +x</div>
          <q-icon name="arrow_forward" size="2rem" color="accent" />
        </div>

        <!-- ESTADO ACTIVO -->
        <div class="tool-card file-state alive text-center" style="width: 160px">
          <q-icon name="terminal" size="3rem" class="text-green-4" />
          <div class="text-weight-bold q-mt-sm text-white">script.py</div>
          <div class="text-caption font-mono text-green-3">rwx-r-xr-x</div>
          <q-badge color="green-6" class="q-mt-sm">Ejecutable</q-badge>
        </div>
      </div>
    </div>

    <!-- 5. FORMAS DE EJECUTAR -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>4. Formas de Invocar a la Bestia</SectionTitle>

      <div class="column q-gutter-md q-mt-sm">
        <!-- Método 1 -->
        <div class="tool-card row items-center q-pa-md no-border-top">
          <div class="col-12 col-md-4">
            <div
              class="bg-black text-green-4 font-mono q-pa-sm rounded-borders text-center border-grey"
            >
              python3 script.py
            </div>
          </div>
          <div class="col-12 col-md-8 q-pl-md">
            <div class="text-subtitle1 text-white">Manual</div>
            <div class="text-caption text-grey-4">
              Ignora el Shebang y permisos. Útil para pruebas rápidas.
            </div>
          </div>
        </div>

        <!-- Método 2 -->
        <div class="tool-card row items-center q-pa-md no-border-top">
          <div class="col-12 col-md-4">
            <div
              class="bg-black text-green-4 font-mono q-pa-sm rounded-borders text-center border-grey"
            >
              ./script.py
            </div>
          </div>
          <div class="col-12 col-md-8 q-pl-md">
            <div class="text-subtitle1 text-white">Directo (Estilo Linux)</div>
            <div class="text-caption text-grey-4">
              Requiere Shebang y <code>chmod +x</code>. El <code>./</code> es obligatorio por
              seguridad.
            </div>
          </div>
        </div>

        <!-- Método 3 -->
        <div class="tool-card ros-method row items-center q-pa-md">
          <div class="col-12 col-md-4">
            <div
              class="bg-black text-accent font-mono q-pa-sm rounded-borders text-center border-accent"
            >
              ros2 run pkg node
            </div>
          </div>
          <div class="col-12 col-md-8 q-pl-md">
            <div class="text-subtitle1 text-primary text-weight-bold">
              Estilo ROS 2 (Profesional)
            </div>
            <div class="text-caption text-grey-3">
              ROS busca el ejecutable en tu workspace instalado. No importa en qué carpeta estés.
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

/* TOOL CARDS GENÉRICAS */
.tool-card {
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* CODE ANATOMY STYLES (Solución al código invisible) */
.tool-card.code-anatomy {
  border-top: 4px solid #a855f7; /* Purple border */
}

.bg-dark-soft {
  background: rgba(15, 23, 42, 0.6);
}
.circle {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  display: inline-block;
}

.line-group {
  padding: 4px 8px;
  border-radius: 4px;
  transition: background 0.2s;
  white-space: pre; /* Mantiene espacios */
}
.line-group:hover {
  background-color: rgba(255, 255, 255, 0.05);
  cursor: help;
}

.line-num {
  color: #64748b;
  margin-right: 12px;
  user-select: none;
}
.token-comment {
  color: #f472b6;
  font-style: italic;
} /* Pink */
.token-keyword {
  color: #c084fc;
  font-weight: bold;
} /* Purple */
.token-func {
  color: #fbbf24;
} /* Yellow/Amber */
.token-string {
  color: #4ade80;
} /* Green */
.highlight-bg {
  background-color: rgba(56, 189, 248, 0.1);
}

/* FILE STATE CARDS */
.tool-card.file-state {
  padding: 20px;
  transition: transform 0.3s;
}
.tool-card.file-state:hover {
  transform: translateY(-5px);
}
.tool-card.file-state.dead {
  border-top: 4px solid #475569;
}
.tool-card.file-state.alive {
  border-top: 4px solid #4ade80;
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.6), rgba(20, 83, 45, 0.4));
  box-shadow: 0 0 15px rgba(74, 222, 128, 0.1);
}

/* EXECUTION METHODS */
.tool-card.ros-method {
  border: 1px solid rgba(56, 189, 248, 0.3);
  background: rgba(56, 189, 248, 0.05);
}
.border-grey {
  border: 1px solid #334155;
}
.border-accent {
  border: 1px solid #38bdf8;
}
.tool-card.no-border-top {
  border-top: 1px solid rgba(255, 255, 255, 0.05);
}

.font-mono {
  font-family: 'Fira Code', monospace;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
