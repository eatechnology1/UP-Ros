<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Bash es el <strong>pegamento</strong> del sistema Linux. En ROS 2 lo usarás para
        automatizar compilaciones, lanzar múltiples nodos a la vez, configurar el entorno y
        orquestar tu robot completo con un solo comando. No necesitas ser experto — con los
        7 conceptos de esta lección manejas el 95% de los scripts reales.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 ANATOMÍA DE UN SCRIPT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Anatomía de un Script Bash
      </SectionTitle>

      <TextBlock>
        Todo script bash es un archivo de texto con instrucciones que el shell ejecuta
        línea por línea. Cada parte tiene un papel:
      </TextBlock>

      <div class="anatomy-block q-mt-lg">
        <div class="ab-header">
          <q-icon name="description" size="16px" color="positive" />
          <span>setup_robot.sh</span>
        </div>
        <div class="ab-lines">
          <div v-for="line in anatomyLines" :key="line.num"
            class="ab-line" :class="'al-' + line.type"
            :style="{ '--al-color': line.color }">
            <span class="abl-num">{{ line.num }}</span>
            <code class="abl-code">{{ line.code }}</code>
            <span v-if="line.note" class="abl-note">{{ line.note }}</span>
          </div>
        </div>
      </div>

      <!-- chmod + source vs execute -->
      <div class="run-methods q-mt-xl">
        <div class="rm-title">
          <q-icon name="play_arrow" size="16px" color="positive" />
          Tres formas de ejecutar un script
        </div>
        <div class="rm-grid">
          <div v-for="rm in runMethods" :key="rm.cmd" class="rm-card"
            :style="{ '--rm-color': rm.color }">
            <div class="rmc-label" :style="{ color: rm.color }">{{ rm.label }}</div>
            <CodeBlock :hide-header="true" lang="bash" :content="rm.cmd" :copyable="true" />
            <div class="rmc-note">{{ rm.note }}</div>
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="source ≠ ejecutar — la diferencia importa mucho" class="q-mt-lg">
        <strong>Ejecutar</strong> (<code>./script.sh</code>) crea un proceso hijo — los cambios de
        variables y <code>cd</code> <em>no afectan</em> la terminal actual.
        <strong>Source</strong> (<code>source script.sh</code>) corre el script en la terminal
        actual — sí cambia el entorno. Por eso debes <code>source install/setup.bash</code>,
        no simplemente ejecutarlo.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         02 VARIABLES Y ARGUMENTOS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Variables y Argumentos
      </SectionTitle>

      <SplitBlock>
        <template #left>
          <div class="var-panel">
            <div class="vp-header vp-local">
              <q-icon name="data_object" size="15px" />
              Variables locales — convención MAYÚSCULAS
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="localVarsCode" :copyable="true" />
          </div>
        </template>
        <template #right>
          <div class="var-panel">
            <div class="vp-header vp-args">
              <q-icon name="input" size="15px" />
              Argumentos de línea de comandos
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="argsCode" :copyable="true" />
          </div>
        </template>
      </SplitBlock>

      <!-- Special variables grid -->
      <div class="special-vars q-mt-xl">
        <div class="sv-title">
          <q-icon name="auto_awesome" size="16px" color="primary" />
          Variables especiales de Bash — siempre disponibles
        </div>
        <div class="sv-grid">
          <div v-for="sv in specialVars" :key="sv.var" class="sv-card"
            :style="{ '--sv-color': sv.color }">
            <code class="svc-var" :style="{ color: sv.color }">{{ sv.var }}</code>
            <div class="svc-name">{{ sv.name }}</div>
            <div class="svc-example">{{ sv.example }}</div>
          </div>
        </div>
      </div>

      <!-- String operations -->
      <div class="string-ops q-mt-xl">
        <div class="so-title">
          <q-icon name="edit_note" size="16px" color="primary" />
          Operaciones con strings — expansión de variables
        </div>
        <CodeBlock :hide-header="true" lang="bash" :content="stringOpsCode" :copyable="true" />
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 CONTROL DE FLUJO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Control de Flujo — if, for, while, case
      </SectionTitle>

      <div class="flow-grid">
        <div v-for="flow in flowBlocks" :key="flow.title" class="flow-card"
          :style="{ '--fc-color': flow.color }">
          <div class="fcc-header">
            <q-icon :name="flow.icon" size="18px" :style="{ color: flow.color }" />
            <span class="fcc-title">{{ flow.title }}</span>
            <span class="fcc-use">{{ flow.use }}</span>
          </div>
          <CodeBlock :hide-header="true" lang="bash" :content="flow.code" :copyable="true" />
        </div>
      </div>

      <!-- Operators reference -->
      <div class="operators q-mt-xl">
        <div class="op-title">
          <q-icon name="compare" size="16px" color="primary" />
          Referencia de operadores para <code>[ ]</code>
        </div>
        <div class="op-tables">
          <div v-for="group in opGroups" :key="group.title" class="op-table"
            :style="{ '--op-color': group.color }">
            <div class="opt-header" :style="{ color: group.color }">{{ group.title }}</div>
            <div v-for="op in group.ops" :key="op.sym" class="opt-row">
              <code class="opt-sym">{{ op.sym }}</code>
              <span class="opt-desc">{{ op.desc }}</span>
              <code class="opt-ex">{{ op.ex }}</code>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 FUNCIONES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        Funciones — Reutilizar y Organizar
      </SectionTitle>

      <TextBlock>
        Las funciones agrupan comandos reutilizables. En un script de lanzamiento ROS 2,
        una función <code>launch_node()</code> evita repetir la misma lógica
        para cada nodo que quieras arrancar.
      </TextBlock>

      <CodeBlock title="Funciones en bash — patrón para ROS 2"
        lang="bash" :content="functionsCode" :copyable="true" />

      <AlertBlock type="info" title="Variables locales con 'local'" class="q-mt-lg">
        Dentro de una función, usa <code>local var=valor</code> para que la variable no
        contamine el scope global del script. Sin <code>local</code>, todas las variables
        son globales — un bug clásico cuando tienes múltiples funciones con variables del mismo nombre.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         05 PROCESOS EN SEGUNDO PLANO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Procesos en Segundo Plano — &, wait y trap
      </SectionTitle>

      <TextBlock>
        En ROS 2 necesitas lanzar varios nodos al mismo tiempo. El símbolo <code>&</code>
        manda un proceso al fondo sin bloquear el script. <code>wait</code> espera a que
        terminen. <code>trap</code> limpia todo cuando presionas Ctrl+C.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="bg-panel bg-bad">
            <div class="bgp-header">
              <q-icon name="block" size="14px" color="negative" />
              Sin & — bloqueante
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="blockingCode" />
            <div class="bgp-note bgp-note-bad">
              El primer nodo congela el script. El segundo nunca arranca.
            </div>
          </div>
        </template>
        <template #right>
          <div class="bg-panel bg-good">
            <div class="bgp-header">
              <q-icon name="check_circle" size="14px" color="positive" />
              Con & + wait — paralelo
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="parallelCode" />
            <div class="bgp-note bgp-note-good">
              Los tres nodos corren en paralelo. <code>wait</code> espera a que todos terminen.
            </div>
          </div>
        </template>
      </SplitBlock>

      <!-- trap + $! pattern -->
      <div class="trap-section q-mt-xl">
        <div class="ts-title">
          <q-icon name="cleaning_services" size="16px" color="primary" />
          Limpieza con <code>trap</code> — imprescindible en scripts ROS 2
        </div>
        <CodeBlock :hide-header="true" lang="bash" :content="trapCode" :copyable="true" />
        <div class="ts-notes q-mt-md">
          <div v-for="note in trapNotes" :key="note.var" class="ts-note"
            :style="{ '--tn-color': note.color }">
            <code class="tsn-var">{{ note.var }}</code>
            <div class="tsn-desc">{{ note.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         06 SHELL SAFETY Y DEBUGGING
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Shell Safety y Debugging
      </SectionTitle>

      <TextBlock>
        Un script sin protecciones puede fallar silenciosamente o ejecutar lógica incorrecta
        sin que te enteres. Estas tres opciones convierten un script amateur en uno profesional.
      </TextBlock>

      <!-- set -euo pipefail visual -->
      <div class="safety-flags q-mt-lg">
        <div class="sf-header">
          <q-icon name="shield" size="16px" color="warning" />
          <code>set -euo pipefail</code> — la línea de seguridad
        </div>
        <div class="sf-grid">
          <div v-for="flag in safetyFlags" :key="flag.flag" class="sf-card"
            :style="{ '--sf-color': flag.color }">
            <code class="sfc-flag">{{ flag.flag }}</code>
            <div class="sfc-name">{{ flag.name }}</div>
            <div class="sfc-desc">{{ flag.desc }}</div>
            <CodeBlock :hide-header="true" lang="bash" :content="flag.code" />
          </div>
        </div>
      </div>

      <!-- Debugging -->
      <div class="debug-section q-mt-xl">
        <div class="ds-title">
          <q-icon name="bug_report" size="16px" color="primary" />
          Técnicas de debugging
        </div>
        <div class="ds-grid">
          <div v-for="d in debugTechs" :key="d.cmd" class="ds-card"
            :style="{ '--ds-color': d.color }">
            <code class="dsc-cmd">{{ d.cmd }}</code>
            <div class="dsc-desc">{{ d.desc }}</div>
            <CodeBlock :hide-header="true" lang="bash" :content="d.code" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         07 VARIABLES DE ENTORNO ROS 2
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">07</span>
        Variables de Entorno ROS 2
      </SectionTitle>

      <TextBlock>
        ROS 2 usa variables de entorno para configurar el comportamiento de la red,
        el middleware DDS y el descubrimiento de paquetes. Saber cuáles existen
        y qué hacen te evita horas de debugging cuando los nodos "no se ven" entre sí.
      </TextBlock>

      <div class="ros-env-grid q-mt-lg">
        <div v-for="ev in rosEnvVars" :key="ev.var" class="rev-card"
          :style="{ '--rev-color': ev.color }">
          <div class="revc-header">
            <code class="revc-var" :style="{ color: ev.color }">{{ ev.var }}</code>
            <span v-if="ev.default" class="revc-default">default: {{ ev.default }}</span>
          </div>
          <div class="revc-desc">{{ ev.desc }}</div>
          <CodeBlock :hide-header="true" lang="bash" :content="ev.code" />
        </div>
      </div>

      <!-- ~/.bashrc aliases ROS 2 -->
      <div class="aliases-section q-mt-xl">
        <div class="as-title">
          <q-icon name="flash_on" size="16px" color="warning" />
          Alias recomendados para <code>~/.bashrc</code>
        </div>
        <CodeBlock :hide-header="true" lang="bash" :content="aliasesCode" :copyable="true" />
        <AlertBlock type="info" title="Aplicar cambios en la terminal actual" class="q-mt-md">
          Después de editar <code>~/.bashrc</code>:
          <code>source ~/.bashrc</code>
          — o abre una terminal nueva. Los alias se cargan solo al iniciar bash.
        </AlertBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SCRIPT COMPLETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Script Profesional Completo — Lanzamiento ROS 2</SectionTitle>

      <TextBlock>
        Combinando todos los conceptos anteriores: validación, funciones, procesos paralelos,
        trap para limpieza, colores en el output y variables de entorno ROS 2.
      </TextBlock>

      <CodeBlock title="launch_robot.sh — Plantilla profesional"
        lang="bash" :content="fullLaunchCode" :copyable="true" />
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Diccionario de Errores de Bash</SectionTitle>

      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item"
          :style="{ '--err-color': err.color }">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num" :style="{ background: err.color + '18', color: err.color }">
                {{ i + 1 }}
              </div>
              <div>
                <div class="err-type" :style="{ color: err.color }">{{ err.type }}</div>
                <code class="err-msg">{{ err.msg }}</code>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'" size="20px"
              style="color:var(--text-muted)" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Causa:</strong> {{ err.cause }}
            </div>
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive" />
              <div>
                <strong>Solución:</strong>
                <ol><li v-for="s in err.steps" :key="s">{{ s }}</li></ol>
              </div>
            </div>
            <CodeBlock v-if="err.code" :hide-header="true" lang="bash" :content="err.code" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Script de Deploy de Robot</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Escribe un script completo de lanzamiento</div>
            <div class="challenge-subtitle">
              Con validación, funciones, procesos paralelos, trap y alias
            </div>
          </div>
          <div class="challenge-badge">40 min</div>
        </div>

        <div class="challenge-steps q-mt-md">
          <div class="cs-title">Pasos:</div>
          <div class="cs-list">
            <div v-for="step in challengeSteps" :key="step.num" class="cs-item">
              <div class="cs-num" :style="{ background: step.color }">{{ step.num }}</div>
              <div class="cs-text">{{ step.text }}</div>
            </div>
          </div>
        </div>

        <CodeBlock title="Especificación del reto" lang="bash"
          :content="challengeCode" :copyable="true" class="q-mt-md" />

        <q-expansion-item icon="lightbulb" label="Ver pistas"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div v-for="h in challengeHints" :key="h" class="answer-row">
              <q-icon name="chevron_right" size="14px" style="color:#4ade80" />
              {{ h }}
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         VIDEO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>Bash scripting para ROS 2 Jazzy — automatización de lanzamiento:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Bash Scripting ROS 2" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowfullscreen></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" size="16px" color="info" class="q-mr-sm" />
          Video en progreso — será reemplazado con contenido del curso.
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RESUMEN
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Resumen — Comandos y Patrones Esenciales</SectionTitle>
      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.cmd" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="terminal" size="12px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         CTA FINAL
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <div class="final-cta">
        <div class="cta-icon">
          <q-icon name="celebration" size="36px" color="primary" />
        </div>
        <div class="cta-title">¡Has completado el Módulo 1!</div>
        <div class="cta-subtitle">
          Dominas Python, C++ y Bash scripting — las 3 herramientas de programación de ROS 2.
          Es hora de los formatos de datos y comunicación.
        </div>
        <q-btn color="primary" unelevated rounded size="md" padding="12px 32px"
          to="/modulo-2/01xmlPage" icon="rocket_launch"
          label="Comenzar Módulo 2 — Formatos de Datos"
          class="q-mt-lg text-weight-bold" />
      </div>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import { reactive } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// ── Fact pills ─────────────────────────────────────────────────
const facts = [
  { icon: '⚙️', label: 'Bash — automatiza cualquier flujo de trabajo en Linux' },
  { icon: '🚀', label: '& — lanza múltiples nodos ROS 2 en paralelo' },
  { icon: '🛡️', label: 'set -euo pipefail — detecta errores automáticamente' },
];

// ── Anatomy lines ──────────────────────────────────────────────
const anatomyLines = [
  { num: 1, code: '#!/bin/bash', note: 'Shebang — indica al SO qué intérprete usar (obligatorio)', color: '#f472b6', type: 'shebang' },
  { num: 2, code: '# Script de lanzamiento para ROS 2 Jazzy', note: 'Comentario — documenta el propósito del script', color: '#64748b', type: 'comment' },
  { num: 3, code: 'set -euo pipefail', note: 'Safety — salir en error, variable no definida o fallo de pipe', color: '#f87171', type: 'safety' },
  { num: 4, code: '', note: '', color: 'transparent', type: 'empty' },
  { num: 5, code: 'WORKSPACE=~/ros2_ws', note: 'Variable local — MAYÚSCULAS por convención', color: '#fbbf24', type: 'variable' },
  { num: 6, code: 'ROS_DISTRO="jazzy"', note: 'Variable — comillas para strings con espacios', color: '#fbbf24', type: 'variable' },
  { num: 7, code: 'source /opt/ros/${ROS_DISTRO}/setup.bash', note: 'Source — carga en el entorno actual (no subprocess)', color: '#c084fc', type: 'source' },
  { num: 8, code: 'echo "ROS 2 ${ROS_DISTRO} listo ✓"', note: 'Output — imprime en terminal con expansión de variable', color: '#60a5fa', type: 'command' },
];

// ── Run methods ────────────────────────────────────────────────
const runMethods = [
  { label: '① Dar permisos + ejecutar', color: '#4ade80', cmd: 'chmod +x setup_robot.sh\n./setup_robot.sh', note: 'El más común. chmod +x se hace solo una vez.' },
  { label: '② Con bash explícito', color: '#60a5fa', cmd: 'bash setup_robot.sh', note: 'No necesita chmod +x. Útil para depurar.' },
  { label: '③ Source (cargar entorno)', color: '#c084fc', cmd: 'source setup_robot.sh\n# o equivalente:\n. setup_robot.sh', note: 'Afecta la terminal actual. Usa para setup de entorno.' },
];

// ── Special variables ──────────────────────────────────────────
const specialVars = [
  { var: '$0', name: 'Nombre del script', color: '#60a5fa', example: '"./launch.sh"' },
  { var: '$1 $2 $3', name: 'Argumentos posicionales', color: '#4ade80', example: '$1 = "mi_robot"' },
  { var: '$#', name: 'Número de argumentos', color: '#fbbf24', example: '3 (si pasas 3 args)' },
  { var: '$@', name: 'Todos los argumentos (lista)', color: '#c084fc', example: '"arg1" "arg2" "arg3"' },
  { var: '$?', name: 'Código salida del último cmd', color: '#f97316', example: '0 = OK, ≠0 = error' },
  { var: '$$', name: 'PID del script actual', color: '#94a3b8', example: '12345' },
  { var: '$!', name: 'PID del último proceso &', color: '#f87171', example: '12346 (del último &)' },
  { var: '${VAR:-default}', name: 'Valor por defecto si VAR vacía', color: '#22d3ee', example: '${SPEED:-1.0}' },
];

// ── Flow blocks ────────────────────────────────────────────────
const flowBlocks = [
  {
    title: 'if / elif / else', icon: 'alt_route', color: '#4ade80',
    use: 'Validar argumentos y condiciones',
    code: [
      '#!/bin/bash',
      'if [ $# -lt 1 ]; then',
      '  echo "Uso: $0 <paquete>"',
      '  exit 1',
      'elif [ -d ~/ros2_ws/src/$1 ]; then',
      '  echo "Compilando $1..."',
      '  cd ~/ros2_ws && colcon build --packages-select "$1"',
      'else',
      '  echo "Error: paquete $1 no encontrado"',
      '  exit 1',
      'fi',
    ].join('\n'),
  },
  {
    title: 'for', icon: 'loop', color: '#60a5fa',
    use: 'Iterar paquetes, archivos, listas',
    code: [
      '#!/bin/bash',
      'PAQUETES=(robot_control robot_vision robot_nav)',
      '',
      'for pkg in "${PAQUETES[@]}"; do',
      '  echo "→ Compilando $pkg"',
      '  colcon build --packages-select "$pkg"',
      'done',
      '',
      'echo "Todos los paquetes compilados ✓"',
    ].join('\n'),
  },
  {
    title: 'while', icon: 'refresh', color: '#fbbf24',
    use: 'Esperar a que un nodo esté listo',
    code: [
      '#!/bin/bash',
      '# Esperar a que turtlesim arranque',
      'echo "Esperando a turtlesim..."',
      '',
      'while ! ros2 node list 2>/dev/null | grep -q /turtlesim; do',
      '  echo -n "."',
      '  sleep 1',
      'done',
      '',
      'echo " ¡Listo!"',
    ].join('\n'),
  },
  {
    title: 'case', icon: 'list_alt', color: '#c084fc',
    use: 'Seleccionar acción por argumento',
    code: [
      '#!/bin/bash',
      'ACCION="${1:-help}"',
      '',
      'case "$ACCION" in',
      '  build)   colcon build ;;',
      '  clean)   rm -rf build/ install/ log/ ;;',
      '  launch)  ros2 launch mi_pkg robot.launch.py ;;',
      '  *)       echo "Uso: $0 {build|clean|launch}" ;;',
      'esac',
    ].join('\n'),
  },
];

// ── Operator groups ────────────────────────────────────────────
const opGroups = [
  {
    title: 'Números', color: '#60a5fa',
    ops: [
      { sym: '-eq', desc: 'Igual', ex: '[ $# -eq 0 ]' },
      { sym: '-ne', desc: 'No igual', ex: '[ $? -ne 0 ]' },
      { sym: '-lt', desc: 'Menor que', ex: '[ $N -lt 10 ]' },
      { sym: '-gt', desc: 'Mayor que', ex: '[ $N -gt 0 ]' },
      { sym: '-le', desc: 'Menor o igual', ex: '[ $N -le 5 ]' },
      { sym: '-ge', desc: 'Mayor o igual', ex: '[ $N -ge 1 ]' },
    ],
  },
  {
    title: 'Archivos y directorios', color: '#4ade80',
    ops: [
      { sym: '-f', desc: 'Archivo existe', ex: '[ -f CMakeLists.txt ]' },
      { sym: '-d', desc: 'Directorio existe', ex: '[ -d ~/ros2_ws ]' },
      { sym: '-x', desc: 'Es ejecutable', ex: '[ -x ./script.sh ]' },
      { sym: '-r', desc: 'Se puede leer', ex: '[ -r ~/.bashrc ]' },
      { sym: '-z', desc: 'String vacío', ex: '[ -z "$VAR" ]' },
      { sym: '-n', desc: 'String no vacío', ex: '[ -n "$VAR" ]' },
    ],
  },
  {
    title: 'Strings', color: '#fbbf24',
    ops: [
      { sym: '=', desc: 'Strings iguales', ex: '[ "$A" = "$B" ]' },
      { sym: '!=', desc: 'Strings distintos', ex: '[ "$A" != "ok" ]' },
      { sym: '&&', desc: 'Y lógico', ex: '[ -f f ] && cmd' },
      { sym: '||', desc: 'O lógico', ex: '[ -d d ] || mkdir d' },
    ],
  },
];

// ── Safety flags ───────────────────────────────────────────────
const safetyFlags = [
  {
    flag: '-e', name: 'exit on error', color: '#f87171',
    desc: 'El script se detiene inmediatamente si un comando retorna código ≠ 0.',
    code: '# Sin -e:\ncomando_que_falla\necho "Esto se ejecuta igual"  # ← bug silencioso\n\n# Con -e:\ncomando_que_falla\necho "Esto NUNCA se ejecuta"  # ← se detuvo arriba',
  },
  {
    flag: '-u', name: 'error on undefined', color: '#fbbf24',
    desc: 'Error si usas una variable no definida. Previene bugs por typos en nombres de variables.',
    code: '# Sin -u: bug silencioso\necho $PAQUETE  # → vacío (sin error)\n\n# Con -u: falla claramente\necho $PAQUETE  # → error: PAQUETE: unbound variable',
  },
  {
    flag: '-o pipefail', name: 'fail on pipe error', color: '#c084fc',
    desc: 'Por defecto, solo el último comando de un pipe determina el código de salida. Con pipefail, falla si cualquier comando del pipe falla.',
    code: '# Sin pipefail:\ncomando_que_falla | grep "ok"  # → exit 0 (!!)\n\n# Con pipefail:\ncomando_que_falla | grep "ok"  # → exit 1 (correcto)',
  },
];

// ── Debug techniques ───────────────────────────────────────────
const debugTechs = [
  {
    cmd: 'bash -x script.sh', color: '#fbbf24',
    desc: 'Modo trace — imprime cada comando antes de ejecutarlo. Ver exactamente qué pasa.',
    code: '# Ejecutar en modo debug:\nbash -x ./launch_robot.sh mi_robot\n# Cada línea aparece con + antes:  + echo "Iniciando..."',
  },
  {
    cmd: 'set -x / set +x', color: '#60a5fa',
    desc: 'Activar/desactivar debug dentro del script — solo en secciones problemáticas.',
    code: 'set -x   # activar trace\ncomando_problematico\nset +x   # desactivar trace',
  },
  {
    cmd: 'echo "DEBUG: $VAR"', color: '#4ade80',
    desc: 'El clásico: imprimir valores de variables para verificar que son lo que esperas.',
    code: 'echo "DEBUG: WORKSPACE=${WORKSPACE}"\necho "DEBUG: ARGS: $@"\necho "DEBUG: cwd: $(pwd)"',
  },
];

// ── ROS 2 environment variables ────────────────────────────────
const rosEnvVars = [
  {
    var: 'ROS_DOMAIN_ID', color: '#4ade80', default: '0',
    desc: 'Aísla la red ROS 2. Nodos con diferente DOMAIN_ID no se "ven". Útil para trabajar en clase sin interferir con compañeros.',
    code: 'export ROS_DOMAIN_ID=42\n# Todos los nodos del script usarán domain 42\n# rango: 0–232',
  },
  {
    var: 'RMW_IMPLEMENTATION', color: '#60a5fa', default: 'rmw_fastrtps_cpp',
    desc: 'Selecciona la implementación de DDS (el middleware de comunicación). FastRTPS es el default. Cyclone DDS es popular por su rendimiento.',
    code: 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n# sudo apt install ros-jazzy-rmw-cyclonedds-cpp',
  },
  {
    var: 'ROS_LOCALHOST_ONLY', color: '#fbbf24', default: '0 (desactivado)',
    desc: 'Con valor 1, restringe ROS 2 a solo comunicarse en localhost. Útil en laptops en redes públicas.',
    code: 'export ROS_LOCALHOST_ONLY=1\n# Los nodos no pueden verse desde otras máquinas',
  },
  {
    var: 'AMENT_PREFIX_PATH', color: '#c084fc', default: 'auto (set por setup.bash)',
    desc: 'Lista de rutas donde ROS 2 busca paquetes instalados. Se actualiza automáticamente con cada source de setup.bash.',
    code: 'echo $AMENT_PREFIX_PATH\n# /opt/ros/jazzy:/home/user/ros2_ws/install/mi_pkg\n# No editar manualmente — usar source setup.bash',
  },
];

// ── Trap notes ─────────────────────────────────────────────────
const trapNotes = [
  { var: '$!', color: '#fbbf24', desc: 'PID del último proceso lanzado con &. Guárdalo para matar solo ese proceso.' },
  { var: 'SIGINT', color: '#f87171', desc: 'Ctrl+C. La señal más común para detener scripts interactivos.' },
  { var: 'SIGTERM', color: '#f97316', desc: 'Señal de terminación. Enviada por kill y systemd al hacer stop.' },
  { var: 'EXIT', color: '#60a5fa', desc: 'Se dispara siempre al salir el script — por error, por trap o normalmente.' },
];

// ── Challenge ──────────────────────────────────────────────────
const challengeSteps = [
  { num: 1, color: '#60a5fa', text: 'Crea deploy_robot.sh con #!/bin/bash y set -euo pipefail' },
  { num: 2, color: '#fbbf24', text: 'Valida que se pasan al menos 2 argumentos: nombre_robot y velocidad' },
  { num: 3, color: '#4ade80', text: 'Crea función launch_node(pkg, node) que lanza en background y guarda el PID' },
  { num: 4, color: '#c084fc', text: 'Usa trap para matar todos los PIDs guardados al recibir SIGINT o EXIT' },
  { num: 5, color: '#f97316', text: 'Source /opt/ros/jazzy/setup.bash y ~/ros2_ws/install/setup.bash al inicio' },
  { num: 6, color: '#f87171', text: 'Agrega 3 alias en ~/.bashrc: deploy, rb (ros2 bag record) y nt (ros2 node list)' },
];

const challengeHints = [
  'Guarda PIDs en array: PIDS=() y luego PIDS+=($!)',
  'En la función: local pkg="$1"; local node="$2"; ros2 run "$pkg" "$node" &',
  'Para matar todos: for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done',
  'ROS_DOMAIN_ID puede ser el tercer argumento con default: "${3:-42}"',
  'Para que wait no falle si un proceso ya terminó: wait 2>/dev/null || true',
  'Alias con argumento como función: rb() { ros2 bag record "$@"; }',
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    type: 'Error de permisos',
    msg: '-bash: ./launch.sh: Permission denied',
    summary: 'El script no tiene permiso de ejecución',
    color: '#f87171',
    cause: 'Por defecto, los archivos creados con touch o nano no son ejecutables. Necesitas dar permiso explícitamente.',
    steps: [
      'Ejecuta: chmod +x ./launch.sh',
      'Verifica: ls -la launch.sh — debe aparecer -rwxr-xr-x',
      'Alternativa sin chmod: bash ./launch.sh',
    ],
    code: 'chmod +x ./launch.sh\nls -la launch.sh  # verificar permisos\n./launch.sh       # ahora funciona',
    open: false,
  },
  {
    type: 'Comando no encontrado',
    msg: 'command not found: ros2',
    summary: 'ROS 2 no está en el PATH — falta hacer source',
    color: '#fbbf24',
    cause: 'ros2, colcon y demás comandos de ROS 2 no están disponibles porque no se ha ejecutado el script de setup de ROS 2 en esta terminal (o en el script).',
    steps: [
      'Agrega al inicio del script: source /opt/ros/jazzy/setup.bash',
      'Si el script lanza el workspace: source ~/ros2_ws/install/setup.bash',
      'En la terminal interactiva: verifica si está en ~/.bashrc',
    ],
    code: '#!/bin/bash\n# Siempre al inicio:\nsource /opt/ros/jazzy/setup.bash\nsource ~/ros2_ws/install/setup.bash\n\n# Ahora ros2 y colcon están disponibles\nros2 node list',
    open: false,
  },
  {
    type: 'Variable no definida',
    msg: 'script.sh: line 8: PAQUETE: unbound variable',
    summary: 'set -u activo y se usa una variable vacía o no definida',
    color: '#c084fc',
    cause: 'Tienes set -u (o set -euo pipefail) y usas una variable que nunca fue asignada, o cuyo nombre está mal escrito.',
    steps: [
      'Verifica el typo: ¿es $PAQUETE o $PACKAGE?',
      'Dale un valor por defecto: ${PAQUETE:-mi_robot}',
      'O valida antes de usar: [ -n "$PAQUETE" ] || { echo "Error"; exit 1; }',
    ],
    code: '# Valor por defecto si no está definida:\nPAQUETE="${1:-mi_robot}"\n\n# O solo si está vacía:\nPAQUETE="${PAQUETE:-}"  # nunca falla con -u',
    open: false,
  },
  {
    type: 'Source no funciona en subproceso',
    msg: 'ROS 2 no disponible en scripts hijo — variables no se propagan',
    summary: 'Se hizo source dentro de un script ejecutado, no en la terminal actual',
    color: '#60a5fa',
    cause: 'Cuando ejecutas ./setup.sh, corre en un proceso hijo. Cualquier export o source hecho ahí NO afecta la terminal padre. La terminal sigue sin ROS 2.',
    steps: [
      'Ejecuta con source: source ./setup.sh (o . ./setup.sh)',
      'O agrega los exports al ~/.bashrc para que persistan',
      'O ejecuta con bash: bash setup.sh (pero igualmente no propaga al padre)',
    ],
    code: '# ❌ No propaga a la terminal:\n./setup.sh\nros2 node list  # command not found\n\n# ✅ Sí propaga:\nsource ./setup.sh\nros2 node list  # funciona',
    open: false,
  },
  {
    type: 'Procesos huérfanos',
    msg: 'Los nodos ROS 2 siguen corriendo después de Ctrl+C',
    summary: 'No hay trap de limpieza — los procesos en background quedan activos',
    color: '#4ade80',
    cause: 'Cuando el script termina (por error, Ctrl+C o fin normal), los procesos lanzados con & sobreviven independientemente y siguen consumiendo recursos.',
    steps: [
      'Agrega trap cleanup EXIT SIGINT SIGTERM al inicio',
      'En cleanup(): usa kill ${PIDS[@]} para matar los PIDs guardados',
      'Si no tienes los PIDs: pkill -f ros2 (matar todos los procesos ros2)',
    ],
    code: 'PIDS=()\n\ncleanup() {\n    echo "Limpiando procesos..."\n    for pid in "${PIDS[@]}"; do\n        kill "$pid" 2>/dev/null || true\n    done\n}\ntrap cleanup EXIT SIGINT SIGTERM\n\nros2 run turtlesim turtlesim_node &\nPIDS+=($!)',
    open: false,
  },
]);

// ── Summary ────────────────────────────────────────────────────
const summaryItems = [
  { cmd: '#!/bin/bash', desc: 'Shebang — primera línea obligatoria', example: 'Indica el intérprete del script', color: '#f472b6' },
  { cmd: 'set -euo pipefail', desc: 'Safety total — salir en error/var vacía/pipe', example: 'Segunda línea de todo script serio', color: '#f87171' },
  { cmd: 'chmod +x', desc: 'Dar permiso de ejecución al script', example: 'chmod +x ./launch.sh', color: '#4ade80' },
  { cmd: 'source script.sh', desc: 'Ejecutar en el shell actual — propaga entorno', example: 'source install/setup.bash', color: '#c084fc' },
  { cmd: 'comando &', desc: 'Ejecutar en segundo plano sin bloquear', example: 'ros2 run turtlesim node &', color: '#60a5fa' },
  { cmd: 'trap fn EXIT', desc: 'Limpiar procesos al salir (Ctrl+C, error, fin)', example: 'trap cleanup EXIT SIGINT', color: '#f97316' },
  { cmd: '${VAR:-default}', desc: 'Valor por defecto si la variable está vacía', example: 'SPEED="${1:-1.0}"', color: '#fbbf24' },
  { cmd: 'ROS_DOMAIN_ID', desc: 'Aislar red ROS 2 entre equipos/proyectos', example: 'export ROS_DOMAIN_ID=42', color: '#22d3ee' },
  { cmd: 'bash -x script.sh', desc: 'Modo trace — debuggear paso a paso', example: 'bash -x ./launch.sh mi_robot', color: '#94a3b8' },
];

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════════════════

const localVarsCode = [
  '#!/bin/bash',
  'set -euo pipefail',
  '',
  '# MAYÚSCULAS por convención para variables globales',
  'WORKSPACE=~/ros2_ws',
  'PACKAGE="mi_robot"',
  'LAUNCH_FILE="robot.launch.py"',
  '',
  '# Usar las variables:',
  'cd "${WORKSPACE}"',
  'colcon build --packages-select "${PACKAGE}"',
  'source install/setup.bash',
  'ros2 launch "${PACKAGE}" "${LAUNCH_FILE}"',
].join('\n');

const argsCode = [
  '#!/bin/bash',
  '# Uso: ./launch.sh <nombre_robot> <velocidad>',
  '',
  '# Acceder a los argumentos:',
  'ROBOT_NAME="${1}"        # primer argumento',
  'VELOCITY="${2:-1.0}"    # segundo arg, default 1.0',
  '',
  '# Validar que se pasó al menos 1:',
  'if [ $# -lt 1 ]; then',
  '  echo "Uso: $0 <robot_name> [velocidad]"',
  '  exit 1',
  'fi',
  '',
  'echo "Robot: ${ROBOT_NAME}, vel: ${VELOCITY}"',
].join('\n');

const stringOpsCode = [
  '#!/bin/bash',
  'ARCHIVO="mi_robot_pkg_v2.tar.gz"',
  '',
  '# Longitud del string',
  'echo ${#ARCHIVO}          # → 23',
  '',
  '# Extraer substring: ${var:inicio:longitud}',
  'echo ${ARCHIVO:0:12}      # → mi_robot_pkg',
  '',
  '# Eliminar sufijo (extensión)',
  'echo ${ARCHIVO%.tar.gz}   # → mi_robot_pkg_v2',
  '',
  '# Eliminar prefijo',
  'echo ${ARCHIVO#mi_}       # → robot_pkg_v2.tar.gz',
  '',
  '# Reemplazar ocurrencia',
  'echo ${ARCHIVO/v2/v3}     # → mi_robot_pkg_v3.tar.gz',
  '',
  '# Convertir a mayúsculas/minúsculas',
  'VAR="jazzy"; echo ${VAR^^}  # → JAZZY',
  'VAR="JAZZY"; echo ${VAR,,}  # → jazzy',
].join('\n');

const functionsCode = [
  '#!/bin/bash',
  'set -euo pipefail',
  '',
  'source /opt/ros/jazzy/setup.bash',
  '',
  '# ── Definir función ────────────────────────────────────',
  'launch_node() {',
  '    local pkg="${1}"    # local = no contamina scope global',
  '    local node="${2}"',
  '    local args="${3:-}" # argumento opcional',
  '',
  '    echo "→ Lanzando ${pkg}/${node}..."',
  '    ros2 run "${pkg}" "${node}" ${args} &',
  '    local pid=$!',
  '    echo "  PID: ${pid}"',
  '    return 0',
  '}',
  '',
  'check_workspace() {',
  '    if [ ! -d ~/ros2_ws/src ]; then',
  '        echo "Error: workspace no existe"',
  '        return 1  # retornar código de error',
  '    fi',
  '    return 0',
  '}',
  '',
  '# ── Llamar funciones ───────────────────────────────────',
  'check_workspace   # retorna 1 si falla → set -e detiene el script',
  '',
  'source ~/ros2_ws/install/setup.bash',
  '',
  'launch_node turtlesim turtlesim_node',
  'sleep 2',
  'launch_node turtlesim turtle_teleop_key',
  '',
  'wait  # esperar a que todos terminen',
].join('\n');

const blockingCode = [
  '#!/bin/bash',
  'ros2 run turtlesim turtlesim_node',
  '# ⚠️ El script se congela aquí',
  '# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━',
  '# Este comando NUNCA se ejecuta:',
  'ros2 run turtlesim turtle_teleop_key',
].join('\n');

const parallelCode = [
  '#!/bin/bash',
  'PIDS=()',
  '',
  'ros2 run turtlesim turtlesim_node &',
  'PIDS+=($!)  # guardar PID',
  '',
  'ros2 run robot_nav nav_node &',
  'PIDS+=($!)',
  '',
  'ros2 run robot_ctrl ctrl_node &',
  'PIDS+=($!)',
  '',
  'echo "Nodos activos: ${#PIDS[@]}"',
  'wait  # esperar a todos',
].join('\n');

const trapCode = [
  '#!/bin/bash',
  'PIDS=()  # array de PIDs para limpiar',
  '',
  '# ── Función de limpieza ─────────────────',
  'cleanup() {',
  '    echo ""',
  '    echo "Deteniendo ${#PIDS[@]} nodos..."',
  '    for pid in "${PIDS[@]}"; do',
  '        kill "${pid}" 2>/dev/null || true',
  '    done',
  '    echo "Limpieza completa."',
  '}',
  '',
  '# Registrar cleanup para estas señales:',
  'trap cleanup EXIT SIGINT SIGTERM',
  '',
  '# Lanzar nodos y guardar PIDs',
  'ros2 run turtlesim turtlesim_node &',
  'PIDS+=($!)',
  '',
  'ros2 run mi_pkg control_node &',
  'PIDS+=($!)',
  '',
  'echo "Presiona Ctrl+C para detener todo"',
  'wait',
].join('\n');

const aliasesCode = [
  '# ── Agregar al final de ~/.bashrc ──────────────────────',
  '',
  '# Activar ROS 2 Jazzy (no humble)',
  'source /opt/ros/jazzy/setup.bash',
  '',
  '# Atajos esenciales',
  'alias sb="source ~/.bashrc"',
  'alias si="source install/setup.bash"',
  'alias cb="colcon build --symlink-install"',
  'alias ws="cd ~/ros2_ws"',
  '',
  '# Funciones con argumentos',
  'cbs() { colcon build --symlink-install --packages-select "$@"; }',
  'ros2_kill() { pkill -f ros2; }',
  '',
  '# ROS 2 Jazzy — domain por defecto',
  'export ROS_DOMAIN_ID=42',
  '',
  '# ── Aplicar cambios ────────────────────────────────────',
  '# source ~/.bashrc',
].join('\n');

const fullLaunchCode = [
  '#!/bin/bash',
  '# launch_robot.sh — Plantilla profesional ROS 2 Jazzy',
  'set -euo pipefail',
  '',
  '# ── Colores ────────────────────────────────────────────',
  'RED="\\033[0;31m"; GREEN="\\033[0;32m"',
  'YELLOW="\\033[1;33m"; BLUE="\\033[0;34m"',
  'NC="\\033[0m"  # No Color',
  '',
  '# ── Variables con defaults ─────────────────────────────',
  'ROBOT_NAME="${1:-}"',
  'VELOCITY="${2:-1.0}"',
  'DOMAIN_ID="${3:-42}"',
  'WORKSPACE="${HOME}/ros2_ws"',
  '',
  '# ── Validación ─────────────────────────────────────────',
  'if [ -z "${ROBOT_NAME}" ]; then',
  '    printf "%bError: falta el nombre del robot%b\\n" "${RED}" "${NC}"',
  '    echo "Uso: $0 <nombre_robot> [velocidad] [domain_id]"',
  '    exit 1',
  'fi',
  '',
  '# ── PIDs para limpieza ─────────────────────────────────',
  'PIDS=()',
  '',
  'cleanup() {',
  '    printf "%b\\nDeteniendo nodos...%b\\n" "${YELLOW}" "${NC}"',
  '    for pid in "${PIDS[@]:-}"; do',
  '        kill "${pid}" 2>/dev/null || true',
  '    done',
  '}',
  'trap cleanup EXIT SIGINT SIGTERM',
  '',
  '# ── Función de lanzamiento ─────────────────────────────',
  'launch_node() {',
  '    local pkg="${1}"; local node="${2}"; local args="${3:-}"',
  '    printf "  %b→%b %s/%s\\n" "${BLUE}" "${NC}" "${pkg}" "${node}"',
  '    ros2 run "${pkg}" "${node}" ${args} &',
  '    PIDS+=($!)',
  '    sleep 1',
  '}',
  '',
  '# ── Setup de entorno ───────────────────────────────────',
  'source /opt/ros/jazzy/setup.bash',
  'source "${WORKSPACE}/install/setup.bash"',
  'export ROS_DOMAIN_ID="${DOMAIN_ID}"',
  '',
  'printf "%b=== Lanzando %s (domain=%s) ===%b\\n" "${GREEN}" "${ROBOT_NAME}" "${DOMAIN_ID}" "${NC}"',
  '',
  '# ── Lanzar nodos ───────────────────────────────────────',
  'launch_node robot_control control_node "--ros-args -p name:=${ROBOT_NAME}"',
  'launch_node robot_nav     nav_node     "--ros-args -p velocity:=${VELOCITY}"',
  'launch_node robot_vision  vision_node',
  '',
  'printf "%b%s listo! Ctrl+C para detener.%b\\n" "${GREEN}" "${ROBOT_NAME}" "${NC}"',
  '',
  'wait',
].join('\n');

const challengeCode = [
  '#!/bin/bash',
  '# deploy_robot.sh — Tu script de lanzamiento',
  '# Uso: ./deploy_robot.sh <nombre_robot> <velocidad> [domain_id]',
  '',
  '# TODO: Agregar set -euo pipefail',
  '',
  '# TODO: Validar argumentos (mínimo 2)',
  '',
  '# TODO: Crear función launch_node() con PID tracking',
  '',
  '# TODO: Registrar trap cleanup con EXIT SIGINT SIGTERM',
  '',
  '# TODO: Source ROS 2 Jazzy y workspace',
  '',
  '# TODO: Exportar ROS_DOMAIN_ID (default: 42)',
  '',
  '# TODO: Lanzar 3 nodos con la función',
  '',
  '# TODO: wait al final',
].join('\n');
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background: var(--bg-code); color: var(--text-code);
  padding: 2px 7px; border-radius: 5px;
  font-family: 'Fira Code', monospace; font-size: .9em;
}
.cmd-badge {
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: .75rem; font-weight: 800; margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }

.fact-pills { display: flex; gap: 10px; flex-wrap: wrap; }
.fact-pill {
  display: flex; align-items: center; gap: 8px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 999px; padding: 7px 16px; font-size: .84rem; color: var(--text-secondary);
  transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fact-icon { font-size: 1rem; }

/* ══════════════════════════════════════════
   ANATOMY BLOCK
══════════════════════════════════════════ */
.anatomy-block { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden; }
.ab-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 16px;
  background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle);
  font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--text-primary);
}
.ab-lines { padding: 8px 0; }
.ab-line {
  display: grid; grid-template-columns: 3rem 1fr 1fr;
  gap: 12px; padding: 8px 16px; align-items: center;
  border-bottom: 1px solid var(--border-subtle); transition: background .15s;
}
.ab-line:last-child { border-bottom: none; }
.ab-line:hover { background: var(--bg-surface-hover); }
.ab-line:not(.al-empty) { border-left: 3px solid var(--al-color, transparent); }
.al-shebang  { background: color-mix(in srgb, var(--al-color) 8%, transparent); }
.al-variable { background: color-mix(in srgb, var(--al-color) 7%, transparent); }
.al-source   { background: color-mix(in srgb, var(--al-color) 7%, transparent); }
.al-command  { background: color-mix(in srgb, var(--al-color) 6%, transparent); }
.al-safety   { background: color-mix(in srgb, var(--al-color) 8%, transparent); }
.al-comment  { opacity: .75; }
.abl-num  { font-family: 'Fira Code', monospace; font-size: .78rem; color: var(--text-muted); text-align: right; }
.abl-code { font-family: 'Fira Code', monospace; font-size: .86rem; color: var(--text-primary); background: none; padding: 0; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.abl-note { font-size: .78rem; color: var(--text-muted); line-height: 1.3; }

/* Run methods */
.run-methods { }
.rm-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.rm-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.rm-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--rm-color, var(--border-medium));
  border-radius: 12px; padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.rmc-label { font-size: .82rem; font-weight: 700; }
.rmc-note  { font-size: .77rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 6px 9px; }

/* ══════════════════════════════════════════
   VARIABLES
══════════════════════════════════════════ */
.var-panel { display: flex; flex-direction: column; gap: 8px; }
.vp-header {
  display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700;
  padding: 8px 12px; border-radius: 8px;
}
.vp-local { background: rgba(251,191,36,.1); color: #fbbf24; }
.vp-args  { background: rgba( 96,165,250,.1); color: #60a5fa; }

.special-vars { }
.sv-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.sv-grid  { display: grid; grid-template-columns: repeat(4,1fr); gap: 10px; }
.sv-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--sv-color, var(--border-medium));
  border-radius: 10px; padding: 10px 12px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.svc-var     { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; background: none; padding: 0; display: block; }
.svc-name    { font-size: .79rem; font-weight: 600; color: var(--text-primary); }
.svc-example { font-family: 'Fira Code', monospace; font-size: .73rem; color: var(--text-muted); }

.string-ops { }
.so-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }

/* ══════════════════════════════════════════
   CONTROL FLOW
══════════════════════════════════════════ */
.flow-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 14px; }
.flow-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--fc-color, var(--border-medium));
  border-radius: 14px; overflow: hidden; min-width: 0;
}
.fcc-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle);
}
.fcc-title { font-family: 'Fira Code', monospace; font-size: .92rem; font-weight: 700; color: var(--text-primary); }
.fcc-use   { font-size: .76rem; color: var(--text-muted); margin-left: auto; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }

.operators { }
.op-title  { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.op-tables { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.op-table  { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-top: 3px solid var(--op-color); border-radius: 10px; overflow: hidden; min-width: 0; }
.opt-header { padding: 8px 12px; font-size: .82rem; font-weight: 700; border-bottom: 1px solid var(--border-subtle); background: var(--bg-surface-solid); }
.opt-row    { display: grid; grid-template-columns: 6rem 1fr; gap: 6px; padding: 6px 12px; border-bottom: 1px solid var(--border-subtle); align-items: center; }
.opt-row:last-child { border-bottom: none; }
.opt-sym  { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; color: var(--op-color); background: none; padding: 0; }
.opt-desc { font-size: .77rem; color: var(--text-secondary); }
.opt-ex   { display: none; }

/* ══════════════════════════════════════════
   BACKGROUND PROCESSES
══════════════════════════════════════════ */
.bg-panel { display: flex; flex-direction: column; gap: 8px; }
.bgp-header {
  display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700;
  padding: 8px 12px; border-radius: 8px;
}
.bg-bad  .bgp-header { background: rgba(248,113,113,.1); color: #f87171; }
.bg-good .bgp-header { background: rgba( 74,222,128,.1); color: #4ade80; }
.bgp-note { font-size: .81rem; color: var(--text-secondary); border-radius: 7px; padding: 8px 12px; border: 1px solid var(--border-subtle); }
.bgp-note-bad  { background: rgba(248,113,113,.06); }
.bgp-note-good { background: rgba( 74,222,128,.06); }

.trap-section { }
.ts-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.ts-notes { display: grid; grid-template-columns: repeat(4,1fr); gap: 10px; }
.ts-note  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--tn-color, var(--border-medium));
  border-radius: 10px; padding: 10px 12px; min-width: 0;
}
.tsn-var  { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--tn-color); background: none; padding: 0; display: block; margin-bottom: 4px; }
.tsn-desc { font-size: .78rem; color: var(--text-muted); line-height: 1.4; }

/* ══════════════════════════════════════════
   SAFETY FLAGS
══════════════════════════════════════════ */
.safety-flags { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.25rem; }
.sf-header { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 14px; }
.sf-grid   { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.sf-card   {
  background: var(--bg-surface-hover); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--sf-color, var(--border-medium));
  border-radius: 10px; padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.sfc-flag { font-family: 'Fira Code', monospace; font-size: 1.1rem; font-weight: 800; color: var(--sf-color); background: none; padding: 0; }
.sfc-name { font-size: .82rem; font-weight: 700; color: var(--text-primary); }
.sfc-desc { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }

.debug-section { }
.ds-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.ds-grid  { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.ds-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ds-color, var(--border-medium));
  border-radius: 12px; padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.dsc-cmd  { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--ds-color); background: none; padding: 0; }
.dsc-desc { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }

/* ══════════════════════════════════════════
   ROS 2 ENV VARS
══════════════════════════════════════════ */
.ros-env-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 14px; }
.rev-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--rev-color, var(--border-medium));
  border-radius: 12px; padding: 1.1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.revc-header  { display: flex; align-items: baseline; gap: 10px; flex-wrap: wrap; }
.revc-var     { font-family: 'Fira Code', monospace; font-size: .92rem; font-weight: 700; background: none; padding: 0; }
.revc-default { font-size: .74rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 999px; padding: 1px 8px; white-space: nowrap; }
.revc-desc    { font-size: .82rem; color: var(--text-secondary); line-height: 1.4; }

.aliases-section { }
.as-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }

/* ══════════════════════════════════════════
   ERROR LIST
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--err-color); border-radius: 12px; overflow: hidden; }
.err-header { display: flex; align-items: center; justify-content: space-between; padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s; }
.err-header:hover { background: var(--bg-surface-hover); }
.err-left   { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num    { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center; }
.err-type   { font-size: .7rem; font-weight: 700; letter-spacing: .05em; text-transform: uppercase; margin-bottom: 1px; }
.err-msg    { font-size: .82rem; display: block; margin-bottom: 2px; color: #f87171; background: none; padding: 0; word-break: break-all; }
.err-summary { font-size: .78rem; color: var(--text-muted); }
.err-body   { padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 10px; }
.err-cause, .err-fix { font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }
.err-fix ol { margin: 4px 0 0 14px; }
.err-fix li { margin-bottom: 4px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX
══════════════════════════════════════════ */
.challenge-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b; }
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon { width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.challenge-title   { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge  { margin-left: auto; font-size: .72rem; font-weight: 800; letter-spacing: .07em; padding: 4px 12px; border-radius: 999px; white-space: nowrap; background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3); }
.challenge-steps  { background: var(--bg-surface-hover); border-radius: 12px; padding: 1rem 1.25rem; }
.cs-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 10px; }
.cs-list  { display: flex; flex-direction: column; gap: 8px; }
.cs-item  { display: flex; align-items: flex-start; gap: 10px; }
.cs-num   { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .82rem; font-weight: 800; color: #1e1e1e; display: flex; align-items: center; justify-content: center; }
.cs-text  { font-size: .87rem; color: var(--text-secondary); padding-top: 3px; }
:deep(.answer-header) { background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25); border-radius: 10px; color: #22c55e; }
.answer-body { background: var(--bg-surface-hover); padding: 1.1rem 1.25rem; border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px; }
.answer-row  { display: flex; align-items: baseline; gap: 8px; font-size: .88rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   VIDEO
══════════════════════════════════════════ */
.video-card    { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.25rem; overflow: hidden; }
.video-wrapper { position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 10px; background: #000; }
.video-wrapper iframe { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.video-caption { display: flex; align-items: center; margin-top: 12px; font-size: .82rem; color: var(--text-muted); padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }

/* ══════════════════════════════════════════
   SUMMARY
══════════════════════════════════════════ */
.summary-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.summary-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 4px solid var(--sc-color); border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s; }
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; }
.sc-desc    { font-size: .81rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   FINAL CTA
══════════════════════════════════════════ */
.final-cta { text-align: center; background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 20px; padding: 3rem 2rem; }
.cta-icon    { margin-bottom: 1rem; }
.cta-title   { font-size: 1.6rem; font-weight: 800; color: var(--text-primary); margin-bottom: .75rem; }
.cta-subtitle { font-size: .95rem; color: var(--text-secondary); max-width: 500px; margin: 0 auto; line-height: 1.5; }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1100px) {
  .sv-grid   { grid-template-columns: repeat(4,1fr); }
  .ts-notes  { grid-template-columns: repeat(2,1fr); }
}
@media (max-width: 900px) {
  .rm-grid   { grid-template-columns: 1fr; }
  .sv-grid   { grid-template-columns: repeat(2,1fr); }
  .flow-grid { grid-template-columns: 1fr; }
  .op-tables { grid-template-columns: 1fr; }
  .sf-grid   { grid-template-columns: 1fr; }
  .ds-grid   { grid-template-columns: 1fr; }
  .ros-env-grid { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: repeat(2,1fr); }
}
@media (max-width: 768px) {
  .ab-line   { grid-template-columns: 2rem 1fr; }
  .abl-note  { display: none; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
  .ts-notes  { grid-template-columns: repeat(2,1fr); }
}
@media (max-width: 480px) {
  .sv-grid   { grid-template-columns: repeat(2,1fr); }
  .summary-grid { grid-template-columns: 1fr; }
}
</style>
