<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        YAML (YAML Ain't Markup Language) es el <strong>lenguaje de configuración nativo</strong>
        de ROS 2. Casi todos los parámetros de nodos, configuraciones de Nav2, SLAM y controladores
        se escriben en YAML. A diferencia de JSON es legible para humanos, permite comentarios y
        soporta multilínea sin escapar caracteres.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div v-for="f in facts" :key="f.label" class="fact-pill">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 ANATOMÍA + TIPOS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Anatomía — La Tiranía del Espacio
      </SectionTitle>

      <TextBlock>
        En YAML la <strong>estructura ES la sintaxis</strong>. No hay llaves ni corchetes:
        la jerarquía se define exclusivamente con espacios. Un solo tab introduce un error fatal.
      </TextBlock>

      <!-- Anatomy visual — data-driven -->
      <div class="yaml-anatomy q-mt-lg">
        <div class="ya-lines">
          <div v-for="line in anatomyLines" :key="line.content" class="ya-line"
            :style="{ paddingLeft: (line.indent * 18 + 12) + 'px' }">
            <div class="yal-code" :style="{ '--yal-color': line.color }">
              <span class="yal-indent" v-if="line.indent > 0">{{ '· '.repeat(line.indent) }}</span>
              <span class="yal-text" :style="{ color: line.color }">{{ line.content }}</span>
            </div>
            <div class="yal-annot">{{ line.annot }}</div>
          </div>
        </div>
        <div class="ya-legend">
          <div v-for="leg in yamlLegend" :key="leg.label" class="yal-entry"
            :style="{ '--leg-color': leg.color }">
            <div class="yale-dot" :style="{ background: leg.color }"></div>
            <span>{{ leg.label }}</span>
          </div>
        </div>
      </div>

      <!-- 6 types -->
      <div class="types-section q-mt-xl">
        <div class="ts-title">
          <q-icon name="category" size="16px" color="primary" />
          Los 6 tipos de dato en YAML
        </div>
        <div class="types-grid">
          <div v-for="t in yamlTypes" :key="t.type" class="type-card"
            :style="{ '--tc-color': t.color }">
            <div class="tc-header">
              <q-icon :name="t.icon" size="18px" :style="{ color: t.color }" />
              <span class="tc-type">{{ t.type }}</span>
              <span class="tc-quirk" v-if="t.quirk">{{ t.quirk }}</span>
            </div>
            <CodeBlock :hide-header="true" lang="yaml" :content="t.example" />
            <div class="tc-use">{{ t.use }}</div>
          </div>
        </div>
      </div>

      <!-- YAML vs JSON comparison -->
      <div class="vs-table q-mt-xl">
        <div class="vt-title">
          <q-icon name="compare" size="16px" color="primary" />
          YAML vs JSON — cuándo usar cada uno
        </div>
        <div class="vt-body">
          <div class="vt-row vt-header">
            <div class="vt-cell">Característica</div>
            <div class="vt-cell vt-json">JSON</div>
            <div class="vt-cell vt-yaml-h">YAML</div>
          </div>
          <div v-for="row in vsRows" :key="row.feature" class="vt-row">
            <div class="vt-cell vt-feat">{{ row.feature }}</div>
            <div class="vt-cell">
              <span :class="row.jsonGood ? 'vt-good' : 'vt-bad'">{{ row.json }}</span>
            </div>
            <div class="vt-cell">
              <span :class="row.yamlGood ? 'vt-good' : 'vt-bad'">{{ row.yaml }}</span>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         02 REGLAS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Las 5 Reglas de YAML Válido
      </SectionTitle>

      <TextBlock>
        YAML parece más simple que JSON pero tiene reglas de whitespace que lo hacen
        <strong>extremadamente sensible al formato</strong>. Una indentación incorrecta
        no da error de sintaxis — parsea como algo completamente diferente:
      </TextBlock>

      <div class="rules-grid q-mt-lg">
        <div v-for="rule in yamlRules" :key="rule.num" class="rule-card"
          :style="{ '--rule-color': rule.color }">
          <div class="rc-num" :style="{ background: rule.color }">{{ rule.num }}</div>
          <q-icon :name="rule.icon" size="22px" :style="{ color: rule.color }" />
          <div class="rc-title">{{ rule.title }}</div>
          <div class="rc-bad">
            <q-icon name="cancel" size="12px" color="negative" class="q-mr-xs" />
            <code>{{ rule.bad }}</code>
          </div>
          <div class="rc-good">
            <q-icon name="check_circle" size="12px" color="positive" class="q-mr-xs" />
            <code>{{ rule.good }}</code>
          </div>
          <div class="rc-note">{{ rule.note }}</div>
        </div>
      </div>

      <AlertBlock type="danger" title="FATAL: Tabs en YAML" class="q-mt-lg">
        YAML lanza un error críptico si mezclas espacios y tabs. Configura VS Code para evitarlo:
        <code>Settings → "editor.insertSpaces" → true</code> y
        <code>"editor.tabSize" → 2</code>. También instala la extensión
        <strong>YAML (Red Hat)</strong> que resalta errores en tiempo real.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         03 PARÁMETROS ROS 2
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Parámetros ROS 2 — La Estructura Obligatoria
      </SectionTitle>

      <TextBlock>
        En ROS 2, los archivos de parámetros YAML deben seguir una estructura exacta:
        <code>nombre_nodo → ros__parameters → tus_variables</code>.
        Permite cambiar el comportamiento de un nodo sin recompilar el código.
      </TextBlock>

      <CodeBlock title="robot_params.yaml — Estructura completa con comentarios"
        lang="yaml" :content="yamlParamsCode" :copyable="true" />

      <!-- ros2 param commands -->
      <div class="param-cmds q-mt-xl">
        <div class="pc-title">
          <q-icon name="terminal" size="16px" color="primary" />
          Comandos <code>ros2 param</code> — gestión en caliente
        </div>
        <div class="pc-grid">
          <div v-for="cmd in paramCommands" :key="cmd.cmd" class="pc-card"
            :style="{ '--pc-color': cmd.color }">
            <code class="pcc-cmd">{{ cmd.cmd }}</code>
            <div class="pcc-desc">{{ cmd.desc }}</div>
          </div>
        </div>
        <CodeBlock title="Comandos ros2 param en terminal" lang="bash"
          :content="ros2ParamCode" :copyable="true" class="q-mt-md" />
      </div>

      <!-- Load from terminal + launch -->
      <div class="load-ways q-mt-xl">
        <div class="lw-title">
          <q-icon name="upload_file" size="16px" color="primary" />
          Cargar el YAML en el nodo
        </div>
        <SplitBlock>
          <template #left>
            <div class="lw-panel">
              <div class="lwp-header">
                <q-icon name="terminal" size="14px" />
                Desde terminal
              </div>
              <CodeBlock :hide-header="true" lang="bash" :content="loadTerminalCode" :copyable="true" />
            </div>
          </template>
          <template #right>
            <div class="lw-panel">
              <div class="lwp-header">
                <q-icon name="rocket_launch" size="14px" />
                Desde launch file Python
              </div>
              <CodeBlock :hide-header="true" lang="python" :content="loadLaunchCode" :copyable="true" />
            </div>
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 RCLPY + PARÁMETROS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        rclpy — Leer y Cambiar Parámetros en el Nodo
      </SectionTitle>

      <TextBlock>
        En el nodo Python, debes <strong>declarar</strong> cada parámetro con su tipo y valor
        por defecto antes de poder leerlo. Esto permite cargar valores del YAML,
        cambiarlos en caliente con <code>ros2 param set</code> y definir callbacks de actualización.
      </TextBlock>

      <CodeBlock title="controller_node.py — declare_parameter + callback de hot-reload"
        lang="python" :content="rclpyNodeCode" :copyable="true" />

      <!-- Parameter types -->
      <div class="param-types q-mt-xl">
        <div class="pt-title">
          <q-icon name="data_object" size="16px" color="primary" />
          Métodos de lectura según el tipo
        </div>
        <div class="pt-grid">
          <div v-for="pt in paramTypes" :key="pt.method" class="pt-card"
            :style="{ '--pt-color': pt.color }">
            <code class="ptc-method">{{ pt.method }}</code>
            <div class="ptc-type">{{ pt.type }}</div>
            <code class="ptc-yaml">{{ pt.yaml }}</code>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 ANCLAS Y ALIAS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Anclas y Alias — DRY para Robots con Componentes Repetidos
      </SectionTitle>

      <TextBlock>
        Un robot diferencial tiene ruedas idénticas. Un brazo robótico tiene articulaciones
        con el mismo controlador PID. <strong>Anclas (<code>&</code>)</strong> y
        <strong>alias (<code>*</code>)</strong> permiten definir una vez y reutilizar
        en todo el YAML sin copy-paste.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="anchor-panel anchor-bad">
            <div class="ap-header">
              <q-icon name="warning" size="14px" color="warning" />
              Sin anclas — repetitivo, frágil
            </div>
            <CodeBlock :hide-header="true" lang="yaml" :content="anclasBadCode" />
            <div class="ap-note ap-note-bad">
              Cambiar el radio significa editar 4 lugares.
              Con 6 ruedas son 6 bugs potenciales.
            </div>
          </div>
        </template>
        <template #right>
          <div class="anchor-panel anchor-good">
            <div class="ap-header">
              <q-icon name="check_circle" size="14px" color="positive" />
              Con anclas y alias — DRY y mantenible
            </div>
            <CodeBlock :hide-header="true" lang="yaml" :content="anclasGoodCode" />
            <div class="ap-note ap-note-good">
              Cambias <code>radio</code> en un lugar y se propaga.
              El <code>&lt;&lt;:</code> (merge key) sobrescribe campos específicos.
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ══════════════════════════════════════════
         06 STRINGS MULTILÍNEA
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Strings Multilínea — El Poder Único de YAML
      </SectionTitle>

      <TextBlock>
        YAML tiene dos operadores para texto multilínea que no existen en JSON.
        Son perfectos para comandos de shell, descripciones largas y scripts
        dentro de archivos de configuración:
      </TextBlock>

      <div class="multiline-grid q-mt-lg">
        <div v-for="ml in multilineTypes" :key="ml.op" class="ml-card"
          :style="{ '--ml-color': ml.color }">
          <div class="mlc-header">
            <code class="mlc-op">{{ ml.op }}</code>
            <span class="mlc-name">{{ ml.name }}</span>
          </div>
          <CodeBlock :hide-header="true" lang="yaml" :content="ml.code" />
          <div class="mlc-behavior">
            <q-icon name="info" size="13px" class="q-mr-xs" :style="{ color: ml.color }" />
            {{ ml.behavior }}
          </div>
          <div class="mlc-use">Ideal para: {{ ml.use }}</div>
        </div>
      </div>

      <CodeBlock title="Ejemplo práctico — launch_config.yaml con multilínea"
        lang="yaml" :content="multilinePracticeCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         07 VALIDACIÓN
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">07</span>
        Validación — yamllint y yaml.safe_load
      </SectionTitle>

      <div class="validation-grid q-mt-lg">
        <div v-for="tool in validationTools" :key="tool.name" class="vt-card-v"
          :style="{ '--vtv-color': tool.color }">
          <div class="vtvc-header">
            <q-icon :name="tool.icon" size="18px" :style="{ color: tool.color }" />
            <span class="vtvc-name">{{ tool.name }}</span>
          </div>
          <div class="vtvc-desc">{{ tool.desc }}</div>
          <CodeBlock :hide-header="true" :lang="tool.lang" :content="tool.code" :copyable="true" />
        </div>
      </div>

      <AlertBlock type="warning" title="yaml.safe_load() — nunca yaml.load()" class="q-mt-lg">
        <code>yaml.load()</code> sin <code>Loader</code> puede ejecutar código arbitrario
        si el YAML viene de una fuente no confiable (vulnerabilidad CVE-2017-18342).
        <strong>Siempre usa <code>yaml.safe_load()</code></strong> en código de producción
        y en nodos ROS 2 que leen archivos de configuración externos.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Diccionario de Errores YAML/ROS 2</SectionTitle>

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
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'"
              size="20px" style="color:var(--text-muted); flex-shrink:0" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Causa:</strong> {{ err.cause }}
            </div>
            <div class="err-pair">
              <CodeBlock :hide-header="true" lang="yaml" :content="err.bad" />
              <CodeBlock :hide-header="true" lang="yaml" :content="err.good" />
            </div>
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive" />
              <strong>Solución:</strong> {{ err.fix }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — YAML + Parámetros ROS 2</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea el YAML de parámetros de un controlador PID</div>
            <div class="challenge-subtitle">
              Con hot-reload usando <code>add_on_set_parameters_callback</code>
            </div>
          </div>
          <div class="challenge-badge">50 min</div>
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

        <CodeBlock title="Esqueleto para completar" lang="python"
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
      <TextBlock>YAML y parámetros en ROS 2 Jazzy:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="YAML en ROS 2" frameborder="0"
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
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Conceptos y Herramientas</SectionTitle>
      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.cmd" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="arrow_right" size="14px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
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
  { icon: '⚙️', label: 'ros__parameters — clave obligatoria con doble guion bajo' },
  { icon: '📐', label: '2 espacios por nivel — tabs = error fatal' },
  { icon: '♻️', label: 'Anclas & alias — reutiliza configs sin copy-paste' },
];

// ── Anatomy lines ──────────────────────────────────────────────
const anatomyLines = [
  { indent: 0, color: '#94a3b8', content: '# Parámetros de ROS 2 Jazzy', annot: 'Comentario — solo en YAML, no en JSON ni XML' },
  { indent: 0, color: '#60a5fa', content: 'robot_controller:',           annot: 'Clave raíz — nombre del nodo ROS 2' },
  { indent: 1, color: '#f87171', content: 'ros__parameters:',            annot: '¡Doble guion bajo! Obligatorio en ROS 2' },
  { indent: 2, color: '#60a5fa', content: 'max_speed: 2.0',              annot: 'Número — sin comillas' },
  { indent: 2, color: '#4ade80', content: "robot_name: 'TurtleBot'",     annot: 'String — comillas opcionales' },
  { indent: 2, color: '#c084fc', content: 'use_sim_time: false',         annot: 'Boolean — minúscula' },
  { indent: 2, color: '#94a3b8', content: 'error_code: ~',               annot: 'Null — ~ equivale a null' },
  { indent: 2, color: '#fbbf24', content: 'pid_config:',                 annot: 'Sub-objeto anidado' },
  { indent: 3, color: '#60a5fa', content: 'kp: 1.5',                    annot: '2 espacios más por nivel' },
  { indent: 3, color: '#60a5fa', content: 'ki: 0.05',                   annot: 'Todos los hijos al mismo nivel' },
  { indent: 2, color: '#f97316', content: 'waypoints:',                  annot: 'Lista de valores' },
  { indent: 3, color: '#f97316', content: '- [0.0, 0.0, 0.0]',          annot: 'Elemento — guión + espacio (obligatorio)' },
  { indent: 3, color: '#f97316', content: '- [2.0, 1.5, 1.57]',         annot: 'Inline array con []' },
];

const yamlLegend = [
  { label: 'Comentario #',         color: '#94a3b8' },
  { label: 'Clave raíz / nodo',    color: '#60a5fa' },
  { label: 'ros__parameters',      color: '#f87171' },
  { label: 'String',               color: '#4ade80' },
  { label: 'Boolean',              color: '#c084fc' },
  { label: 'Sub-objeto',           color: '#fbbf24' },
  { label: 'Lista / array',        color: '#f97316' },
];

// ── YAML Types ─────────────────────────────────────────────────
const yamlTypes = [
  { type: 'String',  icon: 'text_fields', color: '#4ade80', quirk: 'sin comillas',
    use: 'Nombres, rutas, mensajes. Comillas solo si contiene : o #',
    example: 'nombre: TurtleBot\npath: /home/user/ws\ndesc: "nodo: control"' },
  { type: 'Number',  icon: 'numbers',     color: '#60a5fa', quirk: '',
    use: 'Velocidades, posiciones, PIDs, timeouts',
    example: 'vel_max: 2.5\npuerto: 9090\ntemperatura: -15.3' },
  { type: 'Boolean', icon: 'toggle_on',   color: '#c084fc', quirk: 'true/false/yes/no',
    use: 'Flags de activación. En ROS 2 usa true/false por consistencia',
    example: 'use_sim_time: false\ndebug: true\n# También válido:\n# activo: yes' },
  { type: 'Null',    icon: 'block',       color: '#94a3b8', quirk: 'null o ~',
    use: 'Campos opcionales sin valor por defecto',
    example: 'error: ~\ndestino: null\n# ~ es el null idiomático de YAML' },
  { type: 'Lista',   icon: 'list',        color: '#f97316', quirk: 'bloque o inline',
    use: 'Waypoints, lista de sensores, plugins de Nav2',
    example: 'sensores:\n  - lidar\n  - camara\n# Inline:\npos: [1.0, 2.5, 0.0]' },
  { type: 'Objeto',  icon: 'data_object', color: '#fbbf24', quirk: 'anidado',
    use: 'Sub-configs — PID, batería, parámetros del controlador',
    example: 'pid:\n  kp: 1.5\n  ki: 0.05\n  kd: 0.01' },
];

// ── YAML vs JSON ───────────────────────────────────────────────
const vsRows = [
  { feature: 'Comentarios',         json: '❌ no soportado', jsonGood: false, yaml: '✅ con #',              yamlGood: true },
  { feature: 'Comillas en strings', json: 'Obligatorias',    jsonGood: false, yaml: 'Opcionales',            yamlGood: true },
  { feature: 'Multilínea',          json: '❌ sin soporte',  jsonGood: false, yaml: '✅ | y >',              yamlGood: true },
  { feature: 'Anchors/DRY',         json: '❌ no soportado', jsonGood: false, yaml: '✅ & y *',              yamlGood: true },
  { feature: 'Parseo Web/API',       json: '✅ universal',    jsonGood: true,  yaml: 'menos común',          yamlGood: false },
  { feature: 'Verbosidad',          json: 'Mayor (llaves)',   jsonGood: false, yaml: 'Menor (solo espacios)', yamlGood: true },
];

// ── YAML Rules ─────────────────────────────────────────────────
const yamlRules = [
  { num: 1, icon: 'space_bar',         color: '#f87171', title: 'Solo espacios — jamás tabs',
    bad: 'nombre:\\tBot', good: 'nombre: Bot  # 2 espacios', note: 'Configura VS Code: editor.insertSpaces = true' },
  { num: 2, icon: 'more_vert',         color: '#fbbf24', title: 'Espacio después de :',
    bad: 'velocidad:2.5',  good: 'velocidad: 2.5', note: 'sin espacio → YAML lo trata como string complejo' },
  { num: 3, icon: 'format_indent_increase', color: '#60a5fa', title: 'Indentación consistente',
    bad: '  nombre: Bot\\n    vel: 2',  good: '  nombre: Bot\\n  vel: 2', note: 'Todos los hijos del mismo padre al mismo nivel' },
  { num: 4, icon: 'keyboard_double_arrow_right', color: '#c084fc', title: 'ros__parameters (doble _)',
    bad: 'ros_parameters:', good: 'ros__parameters:', note: 'El error más común en ROS 2 — guion simple no funciona' },
  { num: 5, icon: 'format_quote',      color: '#4ade80', title: 'Comillas para strings especiales',
    bad: 'desc: key: value', good: 'desc: "key: value"', note: 'Si el string contiene : # [] {} o empieza con número, usa comillas' },
];

// ── param commands ─────────────────────────────────────────────
const paramCommands = [
  { cmd: 'ros2 param list',  color: '#4ade80', desc: 'Listar todos los parámetros del nodo' },
  { cmd: 'ros2 param get',   color: '#60a5fa', desc: 'Leer el valor de un parámetro' },
  { cmd: 'ros2 param set',   color: '#fbbf24', desc: 'Cambiar un valor en caliente (hot-reload)' },
  { cmd: 'ros2 param dump',  color: '#c084fc', desc: 'Exportar parámetros actuales a YAML' },
  { cmd: 'ros2 param load',  color: '#f97316', desc: 'Cargar un archivo YAML en un nodo activo' },
  { cmd: '--params-file',    color: '#f87171', desc: 'Flag para pasar YAML al lanzar con ros2 run' },
];

// ── Multiline types ────────────────────────────────────────────
const multilineTypes = [
  {
    op: '|', name: 'Literal Block — preserva saltos de línea', color: '#60a5fa',
    behavior: 'Cada línea se convierte en \\n. Espacios y saltos se preservan exactamente.',
    use: 'Comandos bash, scripts, mensajes con formato exacto',
    code: 'startup_cmd: |\n  source /opt/ros/jazzy/setup.bash\n  source ~/robot_ws/install/setup.bash\n  ros2 launch mi_pkg robot.launch.py\n\n# Resultado: string con \\n entre cada línea',
  },
  {
    op: '>', name: 'Folded Block — une líneas en párrafo', color: '#4ade80',
    behavior: 'Los saltos de línea se convierten en espacio. El resultado es un solo párrafo.',
    use: 'Descripciones largas legibles en el archivo pero compactas en el string',
    code: 'description: >\n  TurtleBot3 es un robot móvil diferencial\n  diseñado para educación e investigación.\n  Incluye LIDAR, IMU y encoders.\n\n# Resultado: un string sin \\n internos',
  },
];

// ── Param types ────────────────────────────────────────────────
const paramTypes = [
  { method: '.integer_value',    color: '#60a5fa', type: 'int Python',    yaml: 'puerto: 9090' },
  { method: '.double_value',     color: '#4ade80', type: 'float Python',  yaml: 'max_speed: 2.5' },
  { method: '.string_value',     color: '#fbbf24', type: 'str Python',    yaml: 'nombre: Robot' },
  { method: '.bool_value',       color: '#c084fc', type: 'bool Python',   yaml: 'activo: true' },
  { method: '.integer_array_value', color: '#f97316', type: 'List[int]', yaml: 'puertos: [8080, 9090]' },
  { method: '.double_array_value',  color: '#f87171', type: 'List[float]',yaml: 'pos: [1.0, 2.0, 0.0]' },
];

// ── Validation tools ───────────────────────────────────────────
const validationTools = [
  {
    name: 'yamllint — linter de YAML', icon: 'check_circle', color: '#4ade80', lang: 'bash',
    desc: 'Detecta errores de sintaxis, indentación inconsistente, líneas muy largas y más.',
    code: '# Instalar\npip install yamllint\n\n# Validar\nyamllint robot_params.yaml\n\n# Con config personalizada\nyamllint -d "{extends: default, rules: {line-length: {max: 120}}}" params.yaml\n\n# Validar todos los YAML del workspace\nfind ~/ros2_ws -name "*.yaml" -exec yamllint {} \\;',
  },
  {
    name: 'yaml.safe_load() — Python', icon: 'code', color: '#60a5fa', lang: 'python',
    desc: 'Parsea YAML en Python. safe_load() es obligatorio — yaml.load() tiene vulnerabilidad de seguridad.',
    code: 'import yaml\n\nwith open("robot_params.yaml", "r") as f:\n    try:\n        # ¡safe_load, NO yaml.load()!\n        config = yaml.safe_load(f)\n        nodo = list(config.keys())[0]\n        params = config[nodo]["ros__parameters"]\n        print(f"max_speed: {params[\'max_speed\']}")\n    except yaml.YAMLError as e:\n        print(f"Error YAML: {e}")',
  },
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    type: 'Error de indentación', msg: 'yaml.scanner.ScannerError: mapping values are not allowed here',
    summary: 'Tab en lugar de espacios o nivel de indentación incorrecto',
    color: '#f87171',
    cause: 'YAML usa espacios para definir la jerarquía. Un tab (carácter \\t) rompe el parser porque su tamaño es ambiguo.',
    bad: '# ❌ Con tab (invisible pero fatal)\nrobot:\n\\tnombre: Bot  # \\t es tab',
    good: '# ✅ Con 2 espacios\nrobot:\n  nombre: Bot  # 2 espacios',
    fix: 'Configura VS Code con "editor.insertSpaces": true. Instala la extensión YAML (Red Hat) que muestra tabs en rojo.',
    open: false,
  },
  {
    type: 'ros__parameters con guion simple', msg: "ParameterNotDeclaredException: Parameter 'max_speed' not declared",
    summary: 'El YAML usa ros_parameters (1 guion) en lugar de ros__parameters (2 guiones)',
    color: '#fbbf24',
    cause: 'ROS 2 busca exactamente la clave "ros__parameters" con doble guion bajo. Con un solo guion el archivo se carga pero los parámetros no se reconocen.',
    bad: '# ❌ Guion simple — silenciosamente ignorado\nrobot_controller:\n  ros_parameters:\n    max_speed: 2.0',
    good: '# ✅ Doble guion bajo\nrobot_controller:\n  ros__parameters:\n    max_speed: 2.0',
    fix: 'Busca "ros_parameters" en el archivo y reemplaza con "ros__parameters". Es el error #1 de estudiantes.',
    open: false,
  },
  {
    type: 'String sin comillas con caracteres especiales', msg: "yaml.scanner.ScannerError: could not find expected ':' in mapping",
    summary: 'Un string contiene : o # sin estar entre comillas',
    color: '#c084fc',
    cause: 'YAML interpreta el : como separador clave-valor y el # como inicio de comentario. Strings con estos caracteres deben ir entre comillas.',
    bad: "# ❌ : dentro del string sin comillas\ndescripcion: Robot: modelo Burger\ntopic: /robot/cmd_vel  # OK solo si es el valor completo",
    good: '# ✅ Con comillas dobles\ndescripcion: "Robot: modelo Burger"\n# Strings normales sin : no necesitan comillas\ntipo: diferencial',
    fix: 'Envuelve en comillas dobles cualquier valor que contenga :, #, [, ], {, } o que empiece con un número que no sea numérico.',
    open: false,
  },
  {
    type: 'Boolean ambiguo — yes/no/on/off', msg: "TypeError: expected str, got bool — error al leer el YAML en Python",
    summary: 'YAML 1.1 parsea "yes", "no", "on", "off" como booleanos — no como strings',
    color: '#f97316',
    cause: 'YAML 1.1 (PyYAML por defecto) considera "yes", "no", "on", "off" como True/False. Si necesitas el string literal, debes usar comillas.',
    bad: '# ❌ "yes" se parsea como True (booleano)\nconfirmation: yes\nstatus: on\n# config["confirmation"] → True, no "yes"',
    good: '# ✅ Strings literales entre comillas\nconfirmation: "yes"\nstatus: "on"\n# O mejor: usar true/false explícito\nenabled: true',
    fix: 'Usa true/false para booleanos y comillas para strings que suenen a booleano. Evita yes/no/on/off en configs de ROS 2.',
    open: false,
  },
  {
    type: 'Nombre de nodo incorrecto en YAML', msg: "ParameterNotDeclaredException: No parameter found",
    summary: 'El nodo en el YAML no coincide exactamente con el nombre del nodo en ejecución',
    color: '#4ade80',
    cause: 'ROS 2 busca parámetros bajo el nombre exacto del nodo. Si el nodo se llama "robot_controller" pero el YAML tiene "robotController", no carga nada.',
    bad: '# ❌ Nombre en camelCase\nrobotController:\n  ros__parameters:\n    max_speed: 2.0',
    good: '# ✅ Nombre exacto del nodo (snake_case)\nrobot_controller:\n  ros__parameters:\n    max_speed: 2.0',
    fix: 'Ejecuta ros2 param list /nombre_nodo para ver el nombre exacto. Cópialo exactamente en el YAML.',
    open: false,
  },
]);

// ── Challenge ──────────────────────────────────────────────────
const challengeSteps = [
  { num: 1, color: '#60a5fa', text: 'Crea pid_params.yaml con kp=1.5, ki=0.05, kd=0.01, max_output=100.0, robot_name="mi_robot"' },
  { num: 2, color: '#fbbf24', text: 'Valida con yamllint pid_params.yaml — sin errores' },
  { num: 3, color: '#4ade80', text: 'Completa el nodo Python: declare_parameter para cada parámetro con valor por defecto' },
  { num: 4, color: '#c084fc', text: 'Implementa params_callback que actualice max_output cuando cambie' },
  { num: 5, color: '#f97316', text: 'Ejecuta con: ros2 run mi_pkg pid_ctrl --ros-args --params-file pid_params.yaml' },
  { num: 6, color: '#f87171', text: 'Prueba hot-reload: ros2 param set /pid_controller max_output 80.0 — verifica el log' },
];

const challengeHints = [
  'El nodo en el YAML debe llamarse exactamente como super().__init__("pid_controller")',
  'Para un float: self.get_parameter("kp").get_parameter_value().double_value',
  'Para un string: self.get_parameter("robot_name").get_parameter_value().string_value',
  'add_on_set_parameters_callback recibe una lista de Parameter — itera con for p in params',
  'El callback debe retornar SetParametersResult(successful=True)',
  'ros2 param list /pid_controller confirma que el YAML se cargó correctamente',
];

const summaryItems = [
  { cmd: 'ros__parameters',       desc: 'Clave obligatoria en YAML de ROS 2 (doble _)',       example: 'bajo el nombre del nodo',         color: '#f87171' },
  { cmd: '2 espacios',            desc: 'Indentación estándar — tabs = error fatal',           example: 'editor.insertSpaces: true',       color: '#fbbf24' },
  { cmd: 'clave: valor',          desc: 'Par básico — espacio después de : obligatorio',       example: 'velocidad: 2.5',                  color: '#4ade80' },
  { cmd: '- item',                desc: 'Elemento de lista — guión + espacio',                 example: '- lidar',                         color: '#f97316' },
  { cmd: '&anchor / *alias',      desc: 'DRY — define una vez, reutiliza N veces',             example: '<<: *rueda_base',                 color: '#60a5fa' },
  { cmd: '| y >',                 desc: 'Strings multilínea — literal vs folded',              example: 'startup_cmd: |',                  color: '#c084fc' },
  { cmd: 'declare_parameter()',   desc: 'rclpy — declarar parámetro con tipo y default',       example: "('max_speed', 2.0)",              color: '#22d3ee' },
  { cmd: 'get_parameter()',       desc: 'rclpy — leer el valor del parámetro',                 example: '.get_parameter_value()',          color: '#fbbf24' },
  { cmd: 'yaml.safe_load()',      desc: 'Python — parsear YAML de forma segura',               example: 'nunca yaml.load()',               color: '#4ade80' },
];

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════════════════

const yamlParamsCode = [
  '# robot_params.yaml — ROS 2 Jazzy',
  '# Estructura: nodo → ros__parameters → valores',
  '',
  'robot_controller:',
  '  ros__parameters:  # ¡DOBLE guion bajo!',
  '',
  '    # Parámetros básicos',
  "    robot_name: 'TurtleBot Diferencial'",
  '    max_speed: 2.0       # m/s',
  '    min_speed: 0.1       # m/s',
  '    use_sim_time: false',
  '',
  '    # Controlador PID (sub-objeto)',
  '    pid_config:',
  '      kp: 1.5',
  '      ki: 0.05',
  '      kd: 0.01',
  '',
  '    # Waypoints (lista de arrays inline)',
  '    waypoints:',
  '      - [0.0, 0.0, 0.0]     # [x, y, theta]',
  '      - [2.0, 1.5, 1.57]',
  '      - [4.0, 0.0, 0.0]',
  '',
  '# Segundo nodo en el mismo archivo',
  'navigation:',
  '  ros__parameters:',
  '    robot_radius: 0.3      # m',
  '    inflation_radius: 0.5  # m',
  '    max_vel_x: 0.5         # m/s',
].join('\n');

const ros2ParamCode = [
  '# Listar todos los parámetros del nodo',
  'ros2 param list /robot_controller',
  '',
  '# Leer un parámetro específico',
  'ros2 param get /robot_controller max_speed',
  '',
  '# Cambiar un parámetro en caliente (hot-reload)',
  'ros2 param set /robot_controller max_speed 3.0',
  '',
  '# Exportar parámetros actuales a archivo YAML',
  'ros2 param dump /robot_controller --output-dir .',
  '# Genera: robot_controller.yaml',
  '',
  '# Cargar YAML en nodo ya en ejecución',
  'ros2 param load /robot_controller robot_params.yaml',
].join('\n');

const loadTerminalCode = [
  '# Opción 1: ros2 run con params-file',
  'ros2 run mi_paquete robot_controller \\',
  '  --ros-args \\',
  '  --params-file robot_params.yaml',
  '',
  '# Opción 2: múltiples archivos YAML',
  'ros2 run mi_paquete robot_controller \\',
  '  --ros-args \\',
  '  --params-file base_params.yaml \\',
  '  --params-file nav_params.yaml',
].join('\n');

const loadLaunchCode = [
  'import os',
  'from ament_index_python.packages import get_package_share_directory',
  'from launch import LaunchDescription',
  'from launch_ros.actions import Node',
  '',
  'def generate_launch_description():',
  '    params_file = os.path.join(',
  "        get_package_share_directory('mi_paquete'),",
  "        'config', 'robot_params.yaml'",
  '    )',
  '    return LaunchDescription([',
  '        Node(',
  "            package='mi_paquete',",
  "            executable='robot_controller',",
  '            parameters=[params_file]',
  '        ),',
  '    ])',
].join('\n');

const rclpyNodeCode = [
  '#!/usr/bin/env python3',
  'import rclpy',
  'from rclpy.node import Node',
  'from rcl_interfaces.msg import SetParametersResult',
  '',
  '',
  'class ControlNode(Node):',
  '    def __init__(self):',
  "        super().__init__('robot_controller')",
  '',
  '        # Declarar parámetros con valores por defecto',
  "        self.declare_parameter('max_speed', 2.0)",
  "        self.declare_parameter('robot_name', 'unnamed')",
  "        self.declare_parameter('use_sim_time', False)",
  '        # Parámetros anidados: pid_config.kp en YAML',
  "        self.declare_parameter('pid_config.kp', 1.0)",
  "        self.declare_parameter('pid_config.ki', 0.1)",
  "        self.declare_parameter('pid_config.kd', 0.05)",
  '',
  '        # Leer valores (del YAML si se cargó, si no del default)',
  "        self.max_speed = self.get_parameter('max_speed') \\",
  '            .get_parameter_value().double_value',
  "        nombre = self.get_parameter('robot_name') \\",
  '            .get_parameter_value().string_value',
  "        kp = self.get_parameter('pid_config.kp') \\",
  '            .get_parameter_value().double_value',
  '',
  "        self.get_logger().info(f'Robot: {nombre}')",
  "        self.get_logger().info(f'max_speed={self.max_speed}, kp={kp}')",
  '',
  '        # Hot-reload: callback cuando cambia un parámetro',
  '        self.add_on_set_parameters_callback(self.params_callback)',
  '',
  '    def params_callback(self, params):',
  '        for p in params:',
  "            if p.name == 'max_speed':",
  '                self.max_speed = p.value',
  "                self.get_logger().info(f'max_speed → {self.max_speed}')",
  '        return SetParametersResult(successful=True)',
  '',
  '',
  'def main():',
  '    rclpy.init()',
  '    rclpy.spin(ControlNode())',
  '    rclpy.shutdown()',
].join('\n');

const anclasBadCode = [
  '# ❌ Sin anclas — repetitivo',
  'rueda_izq_front:',
  '  radio: 0.15',
  '  friccion: 0.8',
  '  pid: {kp: 10.0, ki: 0.1, kd: 0.5}',
  '',
  'rueda_der_front:',
  '  radio: 0.15   # ← copiar y pegar',
  '  friccion: 0.8',
  '  pid: {kp: 10.0, ki: 0.1, kd: 0.5}',
  '',
  'rueda_izq_back:',
  '  radio: 0.15   # ← 4 copias = 4 bugs',
  '  friccion: 0.8',
  '  pid: {kp: 10.0, ki: 0.1, kd: 0.5}',
].join('\n');

const anclasGoodCode = [
  '# ✅ Con anclas y alias — DRY',
  '# Definir una vez con &nombre_ancla',
  'rueda_base: &rueda',
  '  radio: 0.15',
  '  friccion: 0.8',
  '  pid: {kp: 10.0, ki: 0.1, kd: 0.5}',
  '',
  '# Reutilizar con <<: *nombre_ancla',
  'rueda_izq_front:',
  '  <<: *rueda  # Copia todo de rueda_base',
  '',
  'rueda_der_front:',
  '  <<: *rueda',
  '',
  'rueda_izq_back:',
  '  <<: *rueda',
  "  radio: 0.20  # Override: solo cambia radio",
  '# Cambias radio en rueda_base → aplica a todas',
].join('\n');

const multilinePracticeCode = [
  '# launch_config.yaml — multilínea en ROS 2',
  'robot_bringup:',
  '  ros__parameters:',
  '',
  '    # | preserva exactamente los \\n',
  '    startup_script: |',
  '      #!/bin/bash',
  '      source /opt/ros/jazzy/setup.bash',
  '      source ~/robot_ws/install/setup.bash',
  '      export ROS_DOMAIN_ID=42',
  '      ros2 launch mi_robot bringup.launch.py',
  '',
  '    # > une líneas en un solo párrafo (+ legible en el archivo)',
  '    robot_description: >',
  '      TurtleBot3 Burger es un robot móvil diferencial',
  '      diseñado para educación e investigación.',
  '      Incluye LIDAR RPLidar A1, IMU MPU9250',
  '      y encoders de cuadratura en las ruedas.',
].join('\n');

const challengeCode = [
  '#!/usr/bin/env python3',
  '"""',
  'RETO: Completa este nodo PID con parámetros YAML y hot-reload.',
  'Crea también pid_params.yaml con la estructura correcta.',
  '"""',
  'import rclpy',
  'from rclpy.node import Node',
  'from rcl_interfaces.msg import SetParametersResult',
  '',
  '',
  'class PIDController(Node):',
  '    def __init__(self):',
  "        super().__init__('pid_controller')",
  '',
  '        # TODO 1: Declarar parámetros (kp, ki, kd, max_output, robot_name)',
  '        # self.declare_parameter("kp", 1.0)',
  '        # ...',
  '',
  '        # TODO 2: Leer valores',
  '        # self.kp = self.get_parameter("kp").get_parameter_value().double_value',
  '        # ...',
  '',
  '        # TODO 3: Loggear los valores al iniciar',
  '        # self.get_logger().info(f"PID Controller: {robot_name}")',
  '        # self.get_logger().info(f"kp={self.kp} ki={self.ki} kd={self.kd}")',
  '',
  '        # TODO 4: Registrar callback de hot-reload',
  '        # self.add_on_set_parameters_callback(self.params_callback)',
  '',
  '    def params_callback(self, params):',
  '        # TODO 5: Actualizar self.max_output cuando cambie el parámetro',
  '        # for p in params:',
  '        #     if p.name == "max_output":',
  '        #         ...',
  '        return SetParametersResult(successful=True)',
  '',
  '',
  'def main():',
  '    rclpy.init()',
  '    rclpy.spin(PIDController())',
  '    rclpy.shutdown()',
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
   YAML ANATOMY
══════════════════════════════════════════ */
.yaml-anatomy {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; overflow: hidden;
}
.ya-lines { padding: 8px 0; }
.ya-line  {
  display: flex; align-items: baseline; gap: 12px;
  padding: 5px 16px; border-bottom: 1px solid var(--border-subtle);
  transition: background .15s;
}
.ya-line:last-child { border-bottom: none; }
.ya-line:hover { background: var(--bg-surface-hover); }
.yal-code { display: flex; align-items: baseline; gap: 4px; min-width: 300px; flex-shrink: 0; }
.yal-indent { font-family: 'Fira Code', monospace; font-size: .8rem; color: var(--border-medium); letter-spacing: 2px; }
.yal-text { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 600; white-space: nowrap; }
.yal-annot { font-size: .75rem; color: var(--text-muted); flex: 1; }
.ya-legend {
  display: flex; flex-wrap: wrap; gap: 8px;
  padding: 10px 16px; background: var(--bg-surface-solid);
  border-top: 1px solid var(--border-subtle);
}
.yal-entry { display: flex; align-items: center; gap: 6px; font-size: .75rem; color: var(--text-muted); }
.yale-dot  { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }

/* ══════════════════════════════════════════
   TYPES GRID
══════════════════════════════════════════ */
.types-section { }
.ts-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.types-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
.type-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--tc-color); border-radius: 12px;
  display: flex; flex-direction: column; overflow: hidden; min-width: 0;
  transition: all .2s;
}
.type-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.tc-header { display: flex; align-items: center; gap: 8px; padding: 10px 14px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); flex-shrink: 0; flex-wrap: wrap; }
.tc-type  { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.tc-quirk { font-size: .7rem; color: var(--text-muted); font-style: italic; }
.tc-use   { padding: 8px 14px; font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); margin-top: auto; line-height: 1.3; }

/* ══════════════════════════════════════════
   YAML vs JSON TABLE
══════════════════════════════════════════ */
.vs-table { }
.vt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.vt-body  { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.vt-row   { display: grid; grid-template-columns: 1.2fr 1fr 1fr; border-bottom: 1px solid var(--border-subtle); }
.vt-row:last-child { border-bottom: none; }
.vt-header { background: var(--bg-surface-solid); font-size: .82rem; font-weight: 700; color: var(--text-primary); }
.vt-cell  { padding: 9px 14px; border-right: 1px solid var(--border-subtle); font-size: .82rem; color: var(--text-secondary); display: flex; align-items: center; }
.vt-cell:last-child { border-right: none; }
.vt-feat  { font-weight: 600; color: var(--text-primary); }
.vt-json, .vt-yaml-h { color: var(--text-primary); }
.vt-good { color: #4ade80; }
.vt-bad  { color: #f87171; }

/* ══════════════════════════════════════════
   RULES GRID
══════════════════════════════════════════ */
.rules-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 12px; }
.rule-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--rule-color); border-radius: 14px;
  padding: 1rem; display: flex; flex-direction: column; align-items: center; gap: 7px; text-align: center;
  transition: all .2s;
}
.rule-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.rc-num  { width: 28px; height: 28px; border-radius: 50%; font-size: .85rem; font-weight: 800; color: #1e1e1e; display: flex; align-items: center; justify-content: center; }
.rc-title { font-size: .85rem; font-weight: 700; color: var(--text-primary); }
.rc-bad  { display: flex; align-items: center; font-size: .73rem; color: var(--text-muted); }
.rc-good { display: flex; align-items: center; font-size: .73rem; color: var(--text-secondary); }
.rc-bad code, .rc-good code { font-size: .7rem; background: none; padding: 0; }
.rc-note { font-size: .71rem; color: var(--text-muted); font-style: italic; line-height: 1.3; }

/* ══════════════════════════════════════════
   PARAM COMMANDS
══════════════════════════════════════════ */
.param-cmds { }
.pc-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.pc-grid  { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-bottom: 0; }
.pc-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--pc-color); border-radius: 10px;
  padding: 10px 12px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.pcc-cmd  { font-family: 'Fira Code', monospace; font-size: .8rem; font-weight: 700; color: var(--pc-color); background: none; padding: 0; }
.pcc-desc { font-size: .78rem; color: var(--text-secondary); line-height: 1.3; }

/* Load ways */
.load-ways { }
.lw-title  { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.lw-panel  { display: flex; flex-direction: column; gap: 8px; }
.lwp-header { display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700; padding: 8px 12px; border-radius: 8px; background: var(--bg-surface-hover); color: var(--text-secondary); }

/* Param types */
.param-types { }
.pt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.pt-grid  { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
.pt-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--pt-color); border-radius: 10px;
  padding: 10px 12px; display: flex; flex-direction: column; gap: 4px; min-width: 0;
}
.ptc-method { font-family: 'Fira Code', monospace; font-size: .8rem; font-weight: 700; color: var(--pt-color); background: none; padding: 0; }
.ptc-type   { font-size: .78rem; color: var(--text-secondary); }
.ptc-yaml   { font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); background: none; padding: 0; }

/* ══════════════════════════════════════════
   ANCHORS
══════════════════════════════════════════ */
.anchor-panel { display: flex; flex-direction: column; gap: 8px; }
.ap-header {
  display: flex; align-items: center; gap: 7px; font-size: .84rem;
  font-weight: 700; padding: 8px 12px; border-radius: 8px;
}
.anchor-bad  .ap-header { background: rgba(251,191,36,.1); color: #fbbf24; }
.anchor-good .ap-header { background: rgba(74,222,128,.1);  color: #4ade80; }
.ap-note { font-size: .8rem; color: var(--text-secondary); border-radius: 7px; padding: 8px 12px; border: 1px solid var(--border-subtle); line-height: 1.4; }
.ap-note-bad  { background: rgba(251,191,36,.05); }
.ap-note-good { background: rgba( 74,222,128,.05); }

/* ══════════════════════════════════════════
   MULTILINE
══════════════════════════════════════════ */
.multiline-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.ml-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ml-color); border-radius: 12px;
  display: flex; flex-direction: column; overflow: hidden; min-width: 0;
}
.mlc-header { display: flex; align-items: center; gap: 10px; padding: 10px 14px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); flex-shrink: 0; }
.mlc-op { font-family: 'Fira Code', monospace; font-size: 1.2rem; font-weight: 900; background: none; padding: 0; color: var(--ml-color); }
.mlc-name { font-size: .85rem; font-weight: 700; color: var(--text-primary); }
.mlc-behavior { display: flex; align-items: flex-start; padding: 8px 14px; font-size: .78rem; color: var(--text-muted); line-height: 1.4; border-top: 1px solid var(--border-subtle); }
.mlc-use { padding: 6px 14px 10px; font-size: .76rem; color: var(--text-muted); font-style: italic; }

/* ══════════════════════════════════════════
   VALIDATION
══════════════════════════════════════════ */
.validation-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.vt-card-v {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--vtv-color); border-radius: 14px;
  padding: 1.1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.vtvc-header { display: flex; align-items: center; gap: 8px; }
.vtvc-name   { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.vtvc-desc   { font-size: .81rem; color: var(--text-secondary); line-height: 1.4; }

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
.err-summary{ font-size: .78rem; color: var(--text-muted); }
.err-body   { padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 10px; }
.err-cause  { font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }
.err-pair   { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
.err-fix    { font-size: .85rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX
══════════════════════════════════════════ */
.challenge-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b; }
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon   { width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.challenge-title  { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
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
.summary-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
.summary-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 4px solid var(--sc-color); border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s; }
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; }
.sc-desc    { font-size: .81rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1100px) {
  .rules-grid { grid-template-columns: repeat(3, 1fr); }
  .pc-grid    { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 900px) {
  .types-grid { grid-template-columns: repeat(2, 1fr); }
  .rules-grid { grid-template-columns: repeat(2, 1fr); }
  .pt-grid    { grid-template-columns: repeat(2, 1fr); }
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
  .err-pair   { grid-template-columns: 1fr; }
  .yal-code   { min-width: 200px; }
}
@media (max-width: 768px) {
  .rules-grid  { grid-template-columns: 1fr; }
  .vt-row      { grid-template-columns: 1fr; }
  .vt-cell     { border-right: none; border-bottom: 1px solid var(--border-subtle); }
  .multiline-grid { grid-template-columns: 1fr; }
  .validation-grid { grid-template-columns: 1fr; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
  .yal-annot   { display: none; }
}
@media (max-width: 480px) {
  .types-grid   { grid-template-columns: 1fr; }
  .pc-grid      { grid-template-columns: 1fr; }
  .pt-grid      { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: 1fr; }
}
</style>
