<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 1.2: ARQUITECTURA DE CÓDIGO
        </div>

        <h1 class="hero-title">Módulos y <span class="text-primary">Paquetes</span></h1>

        <TextBlock>
          Un robot complejo no se programa en un solo archivo gigante. Aprende a dividir tu código
          en piezas reutilizables (Módulos) y a organizarlas en cajas de herramientas (Paquetes).
          Este es el cimiento para entender cómo funciona ROS 2 por dentro.
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA MAGIA DEL IMPORT -->
    <div class="section-group self-stretch">
      <SectionTitle>1. La Magia del "Import"</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            Cuando escribes <code>import math</code>, no es magia negra. Python simplemente busca un
            archivo llamado <code>math.py</code> en tu disco duro y lo lee.
          </TextBlock>

          <ul class="tool-list q-mt-md">
            <li>
              📄 <strong>Módulo:</strong> Un solo archivo <code>.py</code> (ej:
              <code>motor.py</code>).
            </li>
            <li>
              📦 <strong>Paquete:</strong> Una carpeta con muchos módulos y un archivo especial
              <code>__init__.py</code>.
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-6">
          <!-- VISUALIZACIÓN DE IMPORTACIÓN -->
          <div class="tool-card visual-import relative-position q-pa-lg">
            <div class="text-center text-caption text-grey-5 q-mb-md font-mono">FILESYSTEM</div>

            <div class="row items-center justify-center q-gutter-md">
              <!-- Archivo Origen -->
              <div class="file-box bg-grey-9 shadow-2">
                <div class="text-xs text-grey-5 font-mono">calculos.py</div>
                <div class="text-code text-green-4 font-mono">def sumar(a,b):</div>
              </div>

              <!-- Flecha -->
              <div class="column items-center">
                <div class="text-caption text-accent text-weight-bold font-mono q-mb-xs">
                  import calculos
                </div>
                <q-icon name="arrow_forward" color="accent" size="2rem" />
              </div>

              <!-- Archivo Destino -->
              <div class="file-box bg-blue-9 shadow-2">
                <div class="text-xs text-blue-2 font-mono">main.py</div>
                <div class="text-code text-white font-mono">calculos.sumar(1,2)</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. INIT.PY -->
    <div class="section-group self-stretch">
      <AlertBlock type="info" title="🧩 El Secreto de ROS 2: __init__.py">
        Si creas una carpeta llena de scripts, Python no la verá como un paquete importable a menos
        que pongas un archivo llamado <code>__init__.py</code> dentro. <br /><br />
        En ROS 2 (Python), <strong>cada paquete que crees necesitará este archivo</strong>. Su
        presencia le dice a Python: "Esta carpeta es una librería, trátala con respeto".
      </AlertBlock>
    </div>

    <!-- 4. ESTILOS DE IMPORTACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Estilos de Importación</SectionTitle>

      <div class="row q-col-gutter-md q-mt-sm items-stretch">
        <!-- ESTILO 1: RECOMENDADO -->
        <div class="col-12 col-md-4">
          <div class="tool-card import-style good">
            <h4 class="text-subtitle1 text-green-4 q-mt-none q-mb-sm">
              ✅ El Ordenado (Namespace)
            </h4>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="python"
              content="import numpy as np

# Sabes de dónde viene
x = np.array([1, 2, 3])"
            />
            <p class="text-caption text-grey-4 q-mt-md">
              Mantienes el código limpio. Sabes que <code>array</code> pertenece a <code>np</code>.
              Estándar en robótica.
            </p>
          </div>
        </div>

        <!-- ESTILO 2: ACEPTABLE -->
        <div class="col-12 col-md-4">
          <div class="tool-card import-style warn">
            <h4 class="text-subtitle1 text-yellow-4 q-mt-none q-mb-sm">⚠️ El Específico</h4>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="python"
              content="from math import sqrt

# Directo al grano
y = sqrt(9)"
            />
            <p class="text-caption text-grey-4 q-mt-md">
              Útil para funciones únicas. Riesgo de conflicto si ya tienes otra función llamada
              <code>sqrt</code>.
            </p>
          </div>
        </div>

        <!-- ESTILO 3: PROHIBIDO -->
        <div class="col-12 col-md-4">
          <div class="tool-card import-style bad">
            <h4 class="text-subtitle1 text-red-4 q-mt-none q-mb-sm">❌ El Caótico (Wildcard)</h4>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="python"
              content="from rclpy.node import *

# ¿De dónde salió esto?
create_publisher(...)"
            />
            <p class="text-caption text-grey-4 q-mt-md">
              <strong>NUNCA LO HAGAS.</strong> Importas cientos de nombres invisibles. En ROS 2 esto
              es pecado capital.
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. PYTHONPATH -->
    <div class="section-group self-stretch">
      <SectionTitle>3. El Mapa del Tesoro: PYTHONPATH</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            A veces tienes el archivo ahí mismo, haces <code>import mi_robot</code> y Python grita:
            <em>"ModuleNotFoundError"</em>. <br /><br />
            Python solo busca en las carpetas listadas en la variable <code>PYTHONPATH</code>.
            <br /><br />
            <strong>Aquí entra ROS 2:</strong> Cuando haces <code>source install/setup.bash</code>,
            ROS agrega tus carpetas al <code>PYTHONPATH</code> para que Python pueda ver tus nodos.
          </TextBlock>
        </template>

        <template #right>
          <div class="nano-terminal q-pa-md font-mono text-caption rounded-borders shadow-2">
            <div class="text-grey-5"># Inspeccionando dónde busca Python</div>
            <div class="text-green-4">import <span class="text-white">sys</span></div>
            <div class="text-green-4">print(<span class="text-white">sys.path</span>)</div>
            <br />
            <div class="text-yellow-8 output-box q-pa-sm rounded-borders">
              ['', <br />
              '/usr/lib/python3.10', <br />
              '/opt/ros/jazzy/lib/python3.10/site-packages', <br />
              '/home/alex/ros2_ws/install/local/lib/python3.10/dist-packages']
            </div>
            <div class="q-mt-sm text-grey-6 text-center">👆 ¡Ahí están tus librerías de ROS!</div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 6. ESTRUCTURA DE PROYECTO -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>4. Estructura Típica de un Proyecto</SectionTitle>
      <div class="tool-card file-tree-card row items-center justify-center">
        <div class="col-auto">
          <ul class="file-tree font-mono text-body2">
            <li><q-icon name="folder" color="blue-4" /> mi_proyecto_robot/</li>
            <li>
              <ul>
                <li>
                  <q-icon name="description" color="grey-5" /> setup.py
                  <span class="text-grey-6 text-caption q-ml-sm">(Instalador)</span>
                </li>
                <li>
                  <q-icon name="folder" color="blue-4" /> mi_paquete/
                  <span class="text-accent text-caption q-ml-sm">(Código Fuente)</span>
                </li>
                <li>
                  <ul>
                    <li>
                      <q-icon name="description" color="yellow-8" /> __init__.py
                      <span class="text-yellow-6 text-caption text-weight-bold q-ml-sm"
                        >← ¡CRUCIAL!</span
                      >
                    </li>
                    <li><q-icon name="description" color="green-5" /> controlador.py</li>
                    <li><q-icon name="description" color="green-5" /> vision.py</li>
                  </ul>
                </li>
                <li><q-icon name="folder" color="blue-4" /> test/</li>
              </ul>
            </li>
          </ul>
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
    radial-gradient(circle at center, rgba(34, 197, 94, 0.15), transparent 60%),
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

.tool-card.visual-import {
  border-top: 4px solid #3b82f6;
} /* Blue */

/* IMPORT STYLES */
.tool-card.import-style {
  padding: 20px;
  display: flex;
  flex-direction: column;
}
.tool-card.import-style.good {
  border-top: 4px solid #4ade80;
}
.tool-card.import-style.warn {
  border-top: 4px solid #facc15;
}
.tool-card.import-style.bad {
  border-top: 4px solid #f87171;
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

/* VISUALIZACIÓN IMPORTACIÓN */
.file-box {
  width: 130px;
  height: 90px;
  border-radius: 8px;
  padding: 12px;
  display: flex;
  flex-direction: column;
  justify-content: center;
  border: 1px solid rgba(255, 255, 255, 0.1);
}

/* FILE TREE */
.tool-card.file-tree-card {
  padding: 32px;
  border-top: 4px solid #94a3b8;
}
.file-tree {
  list-style: none;
  padding-left: 0;
  color: #e2e8f0;
  margin: 0;
}
.file-tree ul {
  list-style: none;
  padding-left: 24px;
  border-left: 1px solid #475569;
  margin-top: 8px;
}
.file-tree li {
  margin: 8px 0;
  display: flex;
  align-items: center;
  gap: 8px;
}

/* TERMINAL SIMULATION */
.nano-terminal {
  background-color: #0f172a; /* Slate 900 */
  border: 1px solid #334155;
}
.output-box {
  background: rgba(253, 224, 71, 0.1);
  border-left: 3px solid #facc15;
  color: #fef08a;
}

.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-code {
  font-size: 0.8rem;
}
.text-xs {
  font-size: 0.75rem;
  margin-bottom: 6px;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
