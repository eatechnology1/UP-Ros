<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 1.5: AUTOMATIZACIÓN
        </div>

        <h1 class="hero-title">Bash <span class="text-primary">Scripting</span></h1>

        <TextBlock>
          ¿Te imaginas tener que escribir 20 comandos cada vez que enciendes tu robot? Bash es el
          lenguaje nativo de tu terminal. Aprende a crear "macros" potentes para configurar tu
          entorno, lanzar múltiples nodos y automatizar tareas repetitivas.
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA LISTA DE COMPRAS (BATCH) -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Agrupando Comandos</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            Un script de Bash (<code>.sh</code>) es esencialmente una lista de comandos que la
            terminal lee de arriba a abajo, como si tú los escribieras muy rápido.
          </TextBlock>

          <div class="q-mt-lg">
            <AlertBlock type="info" title="Shebang Obligatorio">
              Al igual que en Python, la primera línea debe ser:
              <br /><code class="text-weight-bold">#!/bin/bash</code>
            </AlertBlock>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <!-- VISUAL COMPARISON -->
          <div class="tool-card visual-bash relative-position q-pa-lg">
            <div class="row q-col-gutter-md">
              <!-- MANUAL -->
              <div class="col-6">
                <div class="text-center text-grey-5 q-mb-sm text-caption">Manual (Lento 🐌)</div>
                <div class="terminal-stack">
                  <div class="cmd-box">source /opt/ros/jazzy...</div>
                  <div class="cmd-box">export ROS_DOMAIN_ID=30</div>
                  <div class="cmd-box">colcon build</div>
                  <div class="cmd-box">source install/setup.bash</div>
                </div>
              </div>

              <!-- SCRIPT -->
              <div class="col-6 relative-position">
                <!-- Flecha central (solo desktop) -->
                <div class="absolute-center-left gt-sm">
                  <div class="bg-accent text-black rounded-borders q-pa-xs shadow-2">
                    <q-icon name="arrow_forward" size="1.5rem" />
                  </div>
                </div>

                <div class="text-center text-green-4 q-mb-sm text-caption text-weight-bold">
                  Script (Rápido ⚡)
                </div>
                <div class="file-script bg-green-9 text-white shadow-glow cursor-pointer">
                  <q-icon name="description" size="2rem" />
                  <div class="text-weight-bold q-mt-sm font-mono text-body2">./setup.sh</div>
                  <q-icon name="play_circle" size="2.5rem" class="play-icon absolute-center" />
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. VARIABLES Y ARGUMENTOS -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Argumentos: Hablando con el Script</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Tus scripts pueden recibir información desde fuera. Esto es vital para hacer scripts
            reutilizables (ej: un script para mover un robot, donde la velocidad es variable).
          </TextBlock>
          <ul class="tool-list q-mt-md">
            <li><code>$1, $2...</code>: El primer, segundo, tercer argumento.</li>
            <li><code>$#</code>: El número total de argumentos.</li>
          </ul>
        </template>

        <template #right>
          <div class="nano-terminal q-pa-md font-mono text-caption rounded-borders shadow-2">
            <div
              class="bg-dark-soft q-px-sm q-py-xs text-grey-5 q-mb-sm rounded-borders row items-center"
            >
              <q-icon name="description" size="xs" class="q-mr-xs" /> saludar.sh
            </div>

            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="bash"
              content='#!/bin/bash
NOMBRE=$1
EDAD=$2

echo "Hola $NOMBRE, tienes $EDAD años"'
            />

            <div class="q-mt-md border-top-light q-pt-sm">
              <div class="text-green-4">user@ros:~$ ./saludar.sh Alex 25</div>
              <div class="text-white">Hola Alex, tienes 25 años</div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. EL RETO DE ROS 2 (BACKGROUND) -->
    <div class="section-group self-stretch">
      <SectionTitle>3. El Arte del Segundo Plano (&)</SectionTitle>
      <AlertBlock type="warning" title="🚨 El Bloqueo de la Terminal">
        Si pones <code>ros2 run...</code> en un script, el script se detendrá ahí hasta que cierres
        ese nodo. Si quieres lanzar 3 nodos a la vez, debes usar el símbolo
        <strong>Ampersand (&)</strong>.
      </AlertBlock>

      <div class="row q-col-gutter-lg q-mt-sm">
        <!-- WRONG WAY -->
        <div class="col-12 col-md-6">
          <div class="tool-card bad-practice bg-slate-900 full-height">
            <div class="row items-center q-mb-md border-bottom-red q-pb-sm">
              <q-icon name="cancel" color="red-4" class="q-mr-sm" size="1.5rem" />
              <span class="text-red-4 text-weight-bold">Forma Incorrecta</span>
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="bash"
              content="#!/bin/bash
# El script se CONGELA aquí
ros2 run turtlesim turtlesim_node

# ¡Esto NUNCA se ejecuta!
ros2 run turtlesim turtle_teleop_key"
            />
            <p class="text-caption text-red-3 q-mt-md">
              El segundo comando espera eternamente a que el primero termine.
            </p>
          </div>
        </div>

        <!-- RIGHT WAY -->
        <div class="col-12 col-md-6">
          <div class="tool-card good-practice bg-slate-900 full-height">
            <div class="row items-center q-mb-md border-bottom-green q-pb-sm">
              <q-icon name="check_circle" color="green-4" class="q-mr-sm" size="1.5rem" />
              <span class="text-green-4 text-weight-bold">Forma Correcta</span>
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="bash"
              content="#!/bin/bash
# Lanza y sigue ('&' al fondo)
ros2 run turtlesim turtlesim_node &

# Espera 2 seg a que cargue
sleep 2

# Lanza el control
ros2 run turtlesim turtle_teleop_key"
            />
            <p class="text-caption text-green-3 q-mt-md">
              El <code>&</code> libera la terminal para que el script continúe.
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. ALIAS: TUS ATAJOS DE TECLADO -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>4. Pro-Tip: Los Alias</SectionTitle>
      <div class="tool-card alias-card row items-center justify-between q-pa-lg">
        <div class="col-12 col-md-6">
          <TextBlock>
            ¿Cansado de escribir <code>source install/setup.bash</code> 50 veces al día?
            <br />
            Abre tu archivo <code>~/.bashrc</code> y crea un atajo permanente.
          </TextBlock>
        </div>
        <div class="col-12 col-md-6">
          <div class="bg-black q-pa-md rounded-borders border-purple shadow-glow relative-position">
            <div class="absolute-top-right q-pa-xs">
              <span class="text-caption text-grey-6">~/.bashrc</span>
            </div>
            <div class="text-code text-white font-mono q-mt-sm">
              <span class="text-purple-4">alias</span> <span class="text-green-4">sb</span>="source
              ~/.bashrc"<br />
              <span class="text-purple-4">alias</span> <span class="text-green-4">cb</span>="colcon
              build --symlink-install"
            </div>
          </div>
          <div class="text-caption text-center q-mt-sm text-grey-4">
            Ahora solo escribes <span class="text-green-4 text-weight-bold">'cb'</span> para
            compilar.
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

/* VISUAL BASH */
.tool-card.visual-bash {
  border-top: 4px solid #4ade80;
  background: linear-gradient(180deg, rgba(30, 41, 59, 0.4) 0%, rgba(15, 23, 42, 0.6) 100%);
}

.terminal-stack {
  display: flex;
  flex-direction: column;
  gap: 6px;
}
.cmd-box {
  padding: 8px 12px;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  font-size: 0.75rem;
  color: #a1a1aa;
  border: 1px solid #3f3f46;
  background: rgba(0, 0, 0, 0.3);
}

.file-script {
  height: 140px;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  border-radius: 12px;
  position: relative;
  transition: transform 0.2s;
}
.file-script:hover {
  transform: scale(1.02);
}

.play-icon {
  opacity: 0;
  transition: opacity 0.3s;
}
.file-script:hover .play-icon {
  opacity: 1;
}

.shadow-glow {
  box-shadow: 0 0 25px rgba(74, 222, 128, 0.25);
}

/* GOOD/BAD PRACTICE */
.bad-practice {
  border-left: 4px solid #ef4444;
  padding: 24px;
}
.good-practice {
  border-left: 4px solid #22c55e;
  padding: 24px;
}
.border-bottom-red {
  border-bottom: 1px solid rgba(239, 68, 68, 0.3);
}
.border-bottom-green {
  border-bottom: 1px solid rgba(34, 197, 94, 0.3);
}

/* ALIAS */
.tool-card.alias-card {
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.border-purple {
  border: 1px solid #c084fc;
}

/* TERMINAL SIMULATION */
.nano-terminal {
  background-color: #0f172a;
  border: 1px solid #334155;
}
.bg-dark-soft {
  background: rgba(255, 255, 255, 0.05);
}
.border-top-light {
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
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

.absolute-center-left {
  position: absolute;
  top: 50%;
  left: -14px;
  transform: translateY(-50%);
  z-index: 10;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
