<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Bash scripting es el <strong>pegamento</strong> que une todos los comandos de Linux. En ROS 2,
      usarás scripts para automatizar compilaciones, lanzar múltiples nodos, configurar entornos, y
      crear flujos de trabajo complejos. <br /><br />
      Esta lección te enseña a escribir scripts Bash profesionales, manejar argumentos, ejecutar
      procesos en segundo plano, y automatizar tareas repetitivas de ROS 2.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué Bash y no Python?">
      <strong>Bash:</strong> Perfecto para orquestar comandos del sistema, lanzar procesos, y
      automatización de terminal
      <br />
      <strong>Python:</strong> Mejor para lógica compleja, procesamiento de datos, y nodos ROS 2
      <br /><br />
      En ROS 2, usarás <strong>ambos</strong>. Bash para lanzar, Python para pensar.
    </AlertBlock>

    <!-- ANATOMÍA DE UN SCRIPT -->
    <div class="section-group">
      <SectionTitle>1. Anatomía de un Script Bash</SectionTitle>

      <div class="script-anatomy q-mt-md">
        <div class="anatomy-header">
          <q-icon name="description" color="green-4" size="lg" />
          <span>setup_robot.sh</span>
        </div>
        <div class="anatomy-content">
          <div class="anatomy-line shebang">
            <div class="line-num">1</div>
            <div class="line-code">#!/bin/bash</div>
            <div class="line-note">🔧 Shebang: Indica que es un script Bash</div>
          </div>
          <div class="anatomy-line comment">
            <div class="line-num">2</div>
            <div class="line-code"># Script para configurar entorno ROS 2</div>
            <div class="line-note">📝 Comentario: Documenta qué hace</div>
          </div>
          <div class="anatomy-line empty">
            <div class="line-num">3</div>
            <div class="line-code"></div>
          </div>
          <div class="anatomy-line variable">
            <div class="line-num">4</div>
            <div class="line-code">ROS_DISTRO="humble"</div>
            <div class="line-note">📦 Variable: Almacena datos</div>
          </div>
          <div class="anatomy-line command">
            <div class="line-num">5</div>
            <div class="line-code">source /opt/ros/$ROS_DISTRO/setup.bash</div>
            <div class="line-note">⚙️ Comando: Ejecuta acción</div>
          </div>
          <div class="anatomy-line command">
            <div class="line-num">6</div>
            <div class="line-code">echo "Entorno ROS 2 $ROS_DISTRO cargado"</div>
            <div class="line-note">💬 Output: Muestra mensaje</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Hacer el Script Ejecutable">
          <CodeBlock
            lang="bash"
            content="chmod +x setup_robot.sh
./setup_robot.sh"
            :copyable="true"
          />
        </AlertBlock>
      </div>
    </div>

    <!-- VARIABLES Y ARGUMENTOS -->
    <div class="section-group">
      <SectionTitle>2. Variables y Argumentos</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="variable-card">
            <div class="variable-header">
              <q-icon name="data_object" size="md" />
              <span>Variables Locales</span>
            </div>
            <CodeBlock
              lang="bash"
              content="#!/bin/bash
WORKSPACE_PATH=~/ros2_ws
PACKAGE_NAME=mi_robot

cd $WORKSPACE_PATH
colcon build --packages-select $PACKAGE_NAME"
              :copyable="true"
            />
            <div class="variable-note">
              <strong>Uso:</strong> Almacenar valores que usarás múltiples veces
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="variable-card">
            <div class="variable-header">
              <q-icon name="input" size="md" />
              <span>Argumentos de Línea de Comandos</span>
            </div>
            <CodeBlock
              lang="bash"
              content="#!/bin/bash
# $1 = primer argumento
# $2 = segundo argumento
# $# = número total de argumentos

PACKAGE=$1
VELOCITY=$2

echo Lanzando $PACKAGE con velocidad $VELOCITY
ros2 run $PACKAGE nodo --ros-args -p vel:=$VELOCITY"
              :copyable="true"
            />
            <div class="variable-note">
              <strong>Ejecutar:</strong> <code>./launch.sh turtlesim 2.0</code>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <SectionTitle>Variables Especiales de Bash</SectionTitle>
        <div class="special-vars">
          <div class="var-item">
            <code>$0</code>
            <span>Nombre del script</span>
          </div>
          <div class="var-item">
            <code>$1, $2, $3...</code>
            <span>Argumentos posicionales</span>
          </div>
          <div class="var-item">
            <code>$#</code>
            <span>Número de argumentos</span>
          </div>
          <div class="var-item">
            <code>$@</code>
            <span>Todos los argumentos (como lista)</span>
          </div>
          <div class="var-item">
            <code>$?</code>
            <span>Código de salida del último comando</span>
          </div>
          <div class="var-item">
            <code>$$</code>
            <span>PID del script actual</span>
          </div>
        </div>
      </div>
    </div>

    <!-- PROCESOS EN SEGUNDO PLANO -->
    <div class="section-group">
      <SectionTitle>3. Procesos en Segundo Plano: El Ampersand (&)</SectionTitle>
      <TextBlock>
        En ROS 2, frecuentemente necesitas lanzar múltiples nodos simultáneamente. El símbolo
        <code>&</code> ejecuta un comando en segundo plano, liberando la terminal para continuar.
      </TextBlock>

      <div class="background-demo q-mt-md">
        <div class="demo-wrong">
          <div class="demo-label">❌ Sin & (Bloqueante)</div>
          <CodeBlock
            lang="bash"
            content="#!/bin/bash
ros2 run turtlesim turtlesim_node
# ⚠️ El script se CONGELA aquí
# El siguiente comando NUNCA se ejecuta

ros2 run turtlesim turtle_teleop_key"
          />
          <div class="demo-note error">
            El primer nodo bloquea la ejecución. El segundo nunca arranca.
          </div>
        </div>

        <div class="demo-arrow">
          <q-icon name="arrow_forward" size="2rem" color="yellow-6" />
        </div>

        <div class="demo-correct">
          <div class="demo-label">✅ Con & (No Bloqueante)</div>
          <CodeBlock
            lang="bash"
            content="#!/bin/bash
# Lanza en segundo plano
ros2 run turtlesim turtlesim_node &

# Espera a que cargue
sleep 2

# Lanza el control (primer plano)
ros2 run turtlesim turtle_teleop_key"
          />
          <div class="demo-note success">
            Ambos nodos corren simultáneamente. El <code>&</code> libera la terminal.
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="warning" title="Importante: Limpieza de Procesos">
          Los procesos en segundo plano siguen corriendo aunque el script termine. Usa
          <code>trap</code> para limpiarlos:
          <CodeBlock
            lang="bash"
            content="#!/bin/bash
# Función de limpieza
cleanup() {
  echo Deteniendo nodos...
  killall -9 turtlesim_node
}

# Ejecutar cleanup al salir (Ctrl+C)
trap cleanup EXIT

ros2 run turtlesim turtlesim_node &
sleep 2
ros2 run turtlesim turtle_teleop_key"
            :copyable="true"
          />
        </AlertBlock>
      </div>
    </div>

    <!-- CONDICIONALES -->
    <div class="section-group">
      <SectionTitle>4. Condicionales: if/else</SectionTitle>

      <CodeBlock
        title="Validar argumentos"
        lang="bash"
        content="#!/bin/bash

if [ $# -eq 0 ]; then
  echo Error: Debes proporcionar el nombre del paquete
  echo Uso: $0 nombre_paquete
  exit 1
fi

PACKAGE=$1

if [ -d ~/ros2_ws/src/$PACKAGE ]; then
  echo Compilando $PACKAGE...
  cd ~/ros2_ws
  colcon build --packages-select $PACKAGE
else
  echo Error: El paquete $PACKAGE no existe
  exit 1
fi"
        :copyable="true"
      />

      <div class="q-mt-md">
        <SectionTitle>Operadores de Comparación</SectionTitle>
        <div class="row q-col-gutter-md">
          <div class="col-12 col-md-6">
            <div class="operator-card">
              <div class="operator-title">Números</div>
              <div class="operator-list">
                <div class="operator-item">
                  <code>-eq</code>
                  <span>Igual (equal)</span>
                </div>
                <div class="operator-item">
                  <code>-ne</code>
                  <span>No igual (not equal)</span>
                </div>
                <div class="operator-item">
                  <code>-lt</code>
                  <span>Menor que (less than)</span>
                </div>
                <div class="operator-item">
                  <code>-gt</code>
                  <span>Mayor que (greater than)</span>
                </div>
              </div>
            </div>
          </div>

          <div class="col-12 col-md-6">
            <div class="operator-card">
              <div class="operator-title">Archivos/Directorios</div>
              <div class="operator-list">
                <div class="operator-item">
                  <code>-f</code>
                  <span>Archivo existe</span>
                </div>
                <div class="operator-item">
                  <code>-d</code>
                  <span>Directorio existe</span>
                </div>
                <div class="operator-item">
                  <code>-x</code>
                  <span>Archivo es ejecutable</span>
                </div>
                <div class="operator-item">
                  <code>-z</code>
                  <span>String vacío</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- LOOPS -->
    <div class="section-group">
      <SectionTitle>5. Loops: for y while</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="loop-card">
            <div class="loop-title">Loop for</div>
            <CodeBlock
              lang="bash"
              content="#!/bin/bash
# Compilar múltiples paquetes
PACKAGES=(robot_control robot_vision robot_nav)

for pkg in ${PACKAGES[@]}; do
  echo Compilando $pkg...
  colcon build --packages-select $pkg
done"
              :copyable="true"
            />
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="loop-card">
            <div class="loop-title">Loop while</div>
            <CodeBlock
              lang="bash"
              content="#!/bin/bash
# Esperar a que un nodo esté listo
while ! ros2 node list | grep -q /turtlesim; do
  echo Esperando a turtlesim...
  sleep 1
done
echo Turtlesim está listo!"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- ALIAS -->
    <div class="section-group">
      <SectionTitle>6. Alias: Atajos Permanentes</SectionTitle>
      <TextBlock>
        Los alias son atajos de teclado que defines en <code>~/.bashrc</code>. Se cargan
        automáticamente cada vez que abres una terminal.
      </TextBlock>

      <CodeBlock
        title="~/.bashrc (agregar al final)"
        lang="bash"
        content="# ROS 2 Aliases
alias sb='source ~/.bashrc'
alias sr='source /opt/ros/humble/setup.bash'
alias si='source install/setup.bash'
alias cb='colcon build --symlink-install'
alias cbs='colcon build --symlink-install --packages-select'
alias ct='colcon test'
alias ws='cd ~/ros2_ws'

# Alias con argumentos (funciones)
build() {
  cd ~/ros2_ws
  colcon build --packages-select $1
  source install/setup.bash
}

# Uso: build mi_paquete"
        :copyable="true"
      />

      <div class="q-mt-md">
        <AlertBlock type="success" title="Aplicar Cambios">
          Después de editar <code>~/.bashrc</code>:
          <CodeBlock lang="bash" content="source ~/.bashrc" :copyable="true" />
        </AlertBlock>
      </div>
    </div>

    <!-- EJEMPLO PRÁCTICO -->
    <div class="section-group">
      <SectionTitle>7. Ejemplo Práctico: Script de Lanzamiento ROS 2</SectionTitle>

      <CodeBlock
        title="launch_robot.sh"
        lang="bash"
        content="#!/bin/bash
set -e  # Salir si hay error

# Colores para output
RED='\\033[0;31m'
GREEN='\\033[0;32m'
YELLOW='\\033[1;33m'
NC='\\033[0m'  # No Color

# Función de limpieza
cleanup() {
  echo -e ${YELLOW}Deteniendo todos los nodos...${NC}
  killall -9 ros2 2>/dev/null || true
  exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Validar argumentos
if [ $# -lt 1 ]; then
  echo -e ${RED}Error: Falta el nombre del robot${NC}
  echo Uso: $0 robot_name [velocidad]
  exit 1
fi

ROBOT_NAME=$1
VELOCITY=${2:-1.0}  # Default: 1.0

echo -e ${GREEN}=== Lanzando Robot $ROBOT_NAME ===${NC}

# Configurar entorno
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# Lanzar nodos
echo Iniciando nodo de control...
ros2 run robot_control control_node --ros-args -p name:=$ROBOT_NAME &
sleep 2

echo Iniciando nodo de navegación...
ros2 run robot_nav nav_node --ros-args -p velocity:=$VELOCITY &
sleep 2

echo -e ${GREEN}Robot $ROBOT_NAME listo!${NC}
echo Presiona Ctrl+C para detener

# Mantener script vivo
wait"
        :copyable="true"
      />
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Bash Scripting para ROS 2"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" color="blue-4" size="sm" />
          Video En progreso
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Comandos Esenciales</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>#!/bin/bash</code>
          <span>Shebang (primera línea)</span>
        </div>
        <div class="summary-item">
          <code>chmod +x script.sh</code>
          <span>Hacer ejecutable</span>
        </div>
        <div class="summary-item">
          <code>$1, $2, $3</code>
          <span>Argumentos posicionales</span>
        </div>
        <div class="summary-item">
          <code>comando &</code>
          <span>Ejecutar en segundo plano</span>
        </div>
        <div class="summary-item">
          <code>if [ condición ]; then</code>
          <span>Condicional</span>
        </div>
        <div class="summary-item">
          <code>for var in lista; do</code>
          <span>Loop for</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Script Profesional" class="q-mt-lg">
        ✅ Shebang en la primera línea
        <br />
        ✅ Comentarios explicativos
        <br />
        ✅ Validación de argumentos
        <br />
        ✅ Función de limpieza con <code>trap</code>
        <br />
        ✅ <code>set -e</code> para salir en errores
        <br />
        ✅ Mensajes de output informativos
        <br />
        ✅ Permisos de ejecución (<code>chmod +x</code>)
      </AlertBlock>
    </div>
    <!-- ========== CTA FINAL ========== -->
    <div class="section-group q-mt-xl self-stretch column items-center">
      <div class="final-cta">
        <q-icon name="celebration" size="xl" color="primary" class="q-mb-md" />
        <h2 class="text-h4 text-white text-center q-mb-md">¡Has finalizado el módulo! 🎉</h2>
        <p class="text-body1 text-grey-4 text-center q-mb-lg">
          Has finalizado el módulo de fundamentos de programación.
        </p>

        <div class="row q-gutter-md justify-center">
          <q-btn
            color="primary"
            unelevated
            rounded
            size="lg"
            padding="14px 40px"
            to="/modulo-2/01xmlPage"
            icon="rocket_launch"
            label="Comenzar con Formato de datos"
            class="text-weight-bold"
          />
        </div>
      </div>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* SCRIPT ANATOMY */
.script-anatomy {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.anatomy-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  color: var(--text-primary);
}

.anatomy-content {
  padding: 1.5rem;
}

.anatomy-line {
  display: grid;
  grid-template-columns: 3rem 1fr 1fr;
  gap: 1rem;
  padding: 0.75rem;
  border-radius: 6px;
  margin-bottom: 0.5rem;
  transition: background 0.2s;
}

.anatomy-line:hover {
  background: var(--bg-surface-hover);
}

.anatomy-line.shebang {
  background: rgba(244, 114, 182, 0.1);
  border-left: 3px solid #f472b6;
}

.anatomy-line.comment {
  background: rgba(100, 116, 139, 0.1);
}

.anatomy-line.variable {
  background: rgba(234, 179, 8, 0.1);
  border-left: 3px solid #eab308;
}

.anatomy-line.command {
  background: rgba(59, 130, 246, 0.1);
  border-left: 3px solid #3b82f6;
}

.line-num {
  color: #64748b;
  font-family: 'Fira Code', monospace;
  text-align: right;
}

.line-code {
  font-family: 'Fira Code', monospace;
  color: var(--text-primary);
  font-size: 0.9rem;
}

.line-note {
  color: var(--text-muted);
  font-size: 0.85rem;
}

/* VARIABLE CARDS */
.variable-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
}

.variable-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
}

.variable-note {
  padding: 1rem 1.5rem;
  background: rgba(59, 130, 246, 0.1);
  font-size: 0.85rem;
  color: #93c5fd;
}

/* SPECIAL VARS */
.special-vars {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
  margin-top: 1rem;
}

.var-item {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.var-item code {
  font-family: 'Fira Code', monospace;
  color: var(--text-warning, #d97706);
  font-size: 1.1rem;
}

.var-item span {
  color: var(--text-secondary);
  font-size: 0.85rem;
}

/* BACKGROUND DEMO */
.background-demo {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
}

.demo-label {
  font-size: 1.1rem;
  font-weight: 700;
  margin-bottom: 1rem;
  text-align: center;
}

.demo-note {
  margin-top: 1rem;
  padding: 0.75rem;
  border-radius: 6px;
  font-size: 0.85rem;
}

.demo-note.error {
  background: rgba(239, 68, 68, 0.1);
  border-left: 3px solid #ef4444;
  color: var(--text-danger, #dc2626);
}

.demo-note.success {
  background: rgba(34, 197, 94, 0.1);
  border-left: 3px solid #22c55e;
  color: var(--text-code);
}

.demo-arrow {
  color: var(--text-warning, #d97706);
}

/* OPERATOR CARDS */
.operator-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}

.operator-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: var(--text-primary);
  margin-bottom: 1rem;
}

.operator-list {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.operator-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.5rem;
  background: var(--bg-surface-hover);
  border-radius: 6px;
}

.operator-item code {
  font-family: 'Fira Code', monospace;
  color: var(--text-info, #2563eb);
  font-weight: 700;
}

.operator-item span {
  color: var(--text-muted);
  font-size: 0.85rem;
}

/* LOOP CARDS */
.loop-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
}

.loop-title {
  padding: 1rem 1.5rem;
  background: var(--bg-surface-hover);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: var(--text-primary);
}

/* VIDEO */
.video-container {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 12px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: var(--text-muted);
  font-size: 0.85rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: var(--text-code);
  font-size: 1rem;
}

.summary-item span {
  color: var(--text-secondary);
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .background-demo {
    grid-template-columns: 1fr;
  }

  .demo-arrow {
    transform: rotate(90deg);
  }

  .anatomy-line {
    grid-template-columns: 1fr;
  }

  .line-note {
    margin-left: 3rem;
  }
}
/* ========== CTA FINAL ========== */
.final-cta {
  text-align: center;
  margin: 0 auto;
}
</style>
