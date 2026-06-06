<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        XML (eXtensible Markup Language) es el <strong>lenguaje de configuración</strong> de ROS 2.
        No es código ejecutable — son datos estructurados que describen cómo debe comportarse tu robot:
        qué paquetes necesita, cómo está construido físicamente y qué nodos deben ejecutarse.
        Aparece en <code>package.xml</code>, en archivos URDF y en launch files.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 ANATOMÍA
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Anatomía de una Etiqueta XML
      </SectionTitle>

      <TextBlock>
        Todo en XML se construye con <strong>etiquetas</strong> (tags).
        Cada etiqueta tiene partes bien definidas:
      </TextBlock>

      <!-- Tag breakdown visual -->
      <div class="tag-anatomy q-mt-lg">
        <div class="ta-row">
          <div v-for="part in tagParts" :key="part.label" class="ta-part"
            :style="{ '--ta-color': part.color }">
            <div class="tap-token">{{ part.token }}</div>
            <div class="tap-arrow">↑</div>
            <div class="tap-label">{{ part.label }}</div>
          </div>
        </div>
        <div class="ta-full-tag">
          <span v-for="part in tagParts" :key="part.label + 'c'" class="ta-token-inline"
            :style="{ color: part.color }">{{ part.token }}</span>
        </div>
      </div>

      <!-- Self-closing vs paired -->
      <div class="tag-types q-mt-xl">
        <div class="tt-title">
          <q-icon name="compare" size="16px" color="primary" />
          Dos formas de etiqueta
        </div>
        <div class="tt-grid">
          <div v-for="tt in tagTypes" :key="tt.type" class="tt-card"
            :style="{ '--tt-color': tt.color }">
            <div class="ttc-type" :style="{ color: tt.color }">{{ tt.type }}</div>
            <CodeBlock :hide-header="true" lang="xml" :content="tt.code" />
            <div class="ttc-use">{{ tt.use }}</div>
          </div>
        </div>
      </div>

      <!-- Attributes vs content -->
      <div class="attr-vs-content q-mt-xl">
        <div class="avc-title">
          <q-icon name="compare_arrows" size="16px" color="primary" />
          Atributos vs Contenido — cuándo usar cada uno
        </div>
        <SplitBlock>
          <template #left>
            <div class="avc-panel">
              <div class="avcp-header avcp-attr">
                <q-icon name="settings" size="15px" />
                Atributos — valores cortos
              </div>
              <CodeBlock :hide-header="true" lang="xml" :content="attrCode" />
              <div class="avcp-bullets">
                <div v-for="b in attrBullets" :key="b" class="avcp-item">
                  <q-icon name="check" size="13px" color="positive" />{{ b }}
                </div>
              </div>
            </div>
          </template>
          <template #right>
            <div class="avc-panel">
              <div class="avcp-header avcp-content">
                <q-icon name="text_fields" size="15px" />
                Contenido — texto o elementos hijos
              </div>
              <CodeBlock :hide-header="true" lang="xml" :content="contentCode" />
              <div class="avcp-bullets">
                <div v-for="b in contentBullets" :key="b" class="avcp-item">
                  <q-icon name="check" size="13px" color="positive" />{{ b }}
                </div>
              </div>
            </div>
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         02 REGLAS DE ORO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Las 5 Reglas de XML Válido (Well-Formed)
      </SectionTitle>

      <TextBlock>
        A diferencia de HTML, XML es <strong>estricto</strong>. Un solo error rompe el parser
        completamente. Estas 5 reglas determinan si tu XML es "well-formed" (válido):
      </TextBlock>

      <div class="rules-grid q-mt-lg">
        <div v-for="rule in xmlRules" :key="rule.num" class="rule-card"
          :style="{ '--rule-color': rule.color }">
          <div class="rc-num" :style="{ background: rule.color }">{{ rule.num }}</div>
          <div class="rc-icon">
            <q-icon :name="rule.icon" size="24px" :style="{ color: rule.color }" />
          </div>
          <div class="rc-title">{{ rule.title }}</div>
          <div class="rc-bad">
            <q-icon name="cancel" size="13px" color="negative" class="q-mr-xs" />
            <code>{{ rule.bad }}</code>
          </div>
          <div class="rc-good">
            <q-icon name="check_circle" size="13px" color="positive" class="q-mr-xs" />
            <code>{{ rule.good }}</code>
          </div>
        </div>
      </div>

      <!-- Special chars -->
      <div class="entities-section q-mt-xl">
        <div class="es-title">
          <q-icon name="code" size="16px" color="primary" />
          Caracteres especiales — entidades XML
        </div>
        <div class="es-grid">
          <div v-for="ent in xmlEntities" :key="ent.entity" class="es-card">
            <code class="esc-entity">{{ ent.entity }}</code>
            <span class="esc-arrow">→</span>
            <code class="esc-char">{{ ent.char }}</code>
            <span class="esc-name">{{ ent.name }}</span>
          </div>
        </div>
        <AlertBlock type="warning" title="Regla: dentro de XML, los caracteres < > & deben escaparse siempre" class="q-mt-md">
          Si tu URDF tiene expresiones matemáticas como <code>v &lt; 2.0</code> dentro de texto,
          escríbelas como <code>v &amp;lt; 2.0</code>. En atributos numéricos esto no aplica.
        </AlertBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 PACKAGE.XML
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        package.xml — El Manifiesto del Paquete
      </SectionTitle>

      <TextBlock>
        Cada paquete ROS 2 necesita un <code>package.xml</code> que declara su identidad y
        todas sus dependencias. Es el equivalente a <code>package.json</code> de Node.js
        o <code>pyproject.toml</code> de Python.
      </TextBlock>

      <CodeBlock title="package.xml — Formato 3 (actual en ROS 2 Jazzy)"
        lang="xml" :content="packageXmlCode" :copyable="true" />

      <!-- Dependency types breakdown -->
      <div class="dep-types q-mt-xl">
        <div class="dt-title">
          <q-icon name="account_tree" size="16px" color="primary" />
          Tipos de dependencia — cuándo usar cada una
        </div>
        <div class="dt-grid">
          <div v-for="dep in depTypes" :key="dep.tag" class="dt-card"
            :style="{ '--dt-color': dep.color }">
            <code class="dtc-tag">{{ dep.tag }}</code>
            <div class="dtc-when">{{ dep.when }}</div>
            <div class="dtc-example">ej: {{ dep.example }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 URDF
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        URDF — Descripción Física del Robot
      </SectionTitle>

      <TextBlock>
        URDF (Unified Robot Description Format) es el estándar XML para describir la estructura
        física de un robot: sus piezas (<strong>links</strong>) y cómo se conectan
        (<strong>joints</strong>). Lo usa RViz para visualizar el robot y MoveIt para planificar movimientos.
      </TextBlock>

      <CodeBlock title="robot.urdf — Estructura completa" lang="xml"
        :content="urdfCode" :copyable="true" />

      <!-- Joint types -->
      <div class="joint-types q-mt-xl">
        <div class="jt-title">
          <q-icon name="settings_input_component" size="16px" color="primary" />
          Tipos de Joint — cómo se mueven las piezas
        </div>
        <div class="jt-grid">
          <div v-for="jt in jointTypes" :key="jt.type" class="jt-card"
            :style="{ '--jt-color': jt.color }">
            <div class="jtc-header">
              <q-icon :name="jt.icon" size="20px" :style="{ color: jt.color }" />
              <code class="jtc-type">{{ jt.type }}</code>
              <div class="jtc-dof" :style="{ background: jt.color + '18', color: jt.color }">{{ jt.dof }}</div>
            </div>
            <div class="jtc-desc">{{ jt.desc }}</div>
            <div class="jtc-use">
              <q-icon name="smart_toy" size="12px" class="q-mr-xs" />{{ jt.use }}
            </div>
          </div>
        </div>
      </div>

      <!-- URDF tree visual -->
      <div class="urdf-tree q-mt-xl">
        <div class="ut-title">
          <q-icon name="account_tree" size="16px" color="primary" />
          Árbol de un robot diferencial simple
        </div>
        <div class="ut-body">
          <div v-for="(node, i) in urdfTree" :key="i" class="ut-node"
            :style="{ paddingLeft: (node.level * 28 + 12) + 'px', '--ut-color': node.color }">
            <div class="utn-main">
              <q-icon :name="node.type === 'link' ? 'crop_square' : 'sync_alt'"
                size="15px" :style="{ color: node.color }" />
              <code class="utn-tag" :style="{ color: node.color }">
                {{ node.type === 'link' ? '&lt;link' : '&lt;joint' }} name="{{ node.name }}"
                {{ node.type === 'joint' ? 'type="' + node.jtype + '"' : '' }}&gt;
              </code>
            </div>
            <div class="utn-desc">{{ node.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 XACRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        XACRO — Macros XML para URDF Mantenibles
      </SectionTitle>

      <TextBlock>
        Un robot real tiene decenas de ruedas, articulaciones y sensores idénticos.
        XACRO (XML Macros) permite definirlos <em>una vez</em> y reutilizarlos con parámetros.
        Casi todos los URDF reales en ROS 2 usan XACRO.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="xacro-panel xacro-bad">
            <div class="xcp-header">
              <q-icon name="warning" size="14px" color="warning" />
              Sin XACRO — repetitivo y difícil de mantener
            </div>
            <CodeBlock :hide-header="true" lang="xml" :content="urdfRepeatCode" />
            <div class="xcp-note xcp-note-bad">
              Si cambias el radio de la rueda, debes modificar 4 lugares.
              Con 6 ruedas son 6 cambios — bug garantizado.
            </div>
          </div>
        </template>
        <template #right>
          <div class="xacro-panel xacro-good">
            <div class="xcp-header">
              <q-icon name="check_circle" size="14px" color="positive" />
              Con XACRO — una sola definición, reutilizable
            </div>
            <CodeBlock :hide-header="true" lang="xml" :content="xacroCode" />
            <div class="xcp-note xcp-note-good">
              Cambias el radio en un solo lugar (<code>wheel_r</code>) y aplica a todas las ruedas.
            </div>
          </div>
        </template>
      </SplitBlock>

      <!-- XACRO features -->
      <div class="xacro-features q-mt-xl">
        <div class="xf-title">
          <q-icon name="auto_awesome" size="16px" color="primary" />
          Características clave de XACRO
        </div>
        <div class="xf-grid">
          <div v-for="feat in xacroFeatures" :key="feat.name" class="xf-card"
            :style="{ '--xf-color': feat.color }">
            <code class="xfc-syntax">{{ feat.syntax }}</code>
            <div class="xfc-name" :style="{ color: feat.color }">{{ feat.name }}</div>
            <div class="xfc-desc">{{ feat.desc }}</div>
            <CodeBlock :hide-header="true" lang="xml" :content="feat.code" />
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="XACRO se procesa antes de cargar el URDF" class="q-mt-lg">
        XACRO no es URDF puro — es una plantilla que se procesa primero:
        <code>xacro robot.urdf.xacro &gt; robot.urdf</code>
        o directamente en ROS 2:
        <code>ros2 launch mi_pkg display.launch.py model:=robot.urdf.xacro</code>.
        Los archivos XACRO usan la extensión <code>.urdf.xacro</code>.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         06 LAUNCH FILES XML
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Launch Files XML — Orquestación de Nodos
      </SectionTitle>

      <TextBlock>
        Los launch files XML permiten lanzar múltiples nodos con configuraciones específicas,
        pasar argumentos y remapear topics — todo declarativamente sin código Python.
      </TextBlock>

      <CodeBlock title="robot.launch.xml — Ejemplo completo" lang="xml"
        :content="launchXmlCode" :copyable="true" />

      <!-- Launch elements -->
      <div class="launch-elements q-mt-xl">
        <div class="le-title">
          <q-icon name="auto_stories" size="16px" color="primary" />
          Elementos principales del launch XML
        </div>
        <div class="le-grid">
          <div v-for="el in launchElements" :key="el.tag" class="le-card"
            :style="{ '--le-color': el.color }">
            <code class="lec-tag">{{ el.tag }}</code>
            <div class="lec-desc">{{ el.desc }}</div>
            <div class="lec-ex">{{ el.example }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         07 VALIDACIÓN
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">07</span>
        Validación — Detectar Errores Antes de Correr
      </SectionTitle>

      <div class="validation-grid q-mt-lg">
        <div v-for="tool in validationTools" :key="tool.name" class="vt-card"
          :style="{ '--vt-color': tool.color }">
          <div class="vtc-header">
            <q-icon :name="tool.icon" size="18px" :style="{ color: tool.color }" />
            <span class="vtc-name">{{ tool.name }}</span>
          </div>
          <div class="vtc-desc">{{ tool.desc }}</div>
          <CodeBlock :hide-header="true" lang="bash" :content="tool.code" :copyable="true" />
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Diccionario de Errores XML/URDF</SectionTitle>

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
            <CodeBlock v-if="err.code" :hide-header="true" lang="xml" :content="err.code" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — package.xml y URDF de tu Robot</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea el package.xml y URDF de un robot diferencial</div>
            <div class="challenge-subtitle">
              Con XACRO para las ruedas y joints configurables
            </div>
          </div>
          <div class="challenge-badge">45 min</div>
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

        <CodeBlock title="Especificación del reto" lang="xml"
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
      <TextBlock>XML, URDF y XACRO en ROS 2 Jazzy:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="XML en ROS 2" frameborder="0"
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
  { icon: '📄', label: 'package.xml — el manifiesto de cada paquete ROS 2' },
  { icon: '🤖', label: 'URDF — describe la forma física y joints del robot' },
  { icon: '🔧', label: 'XACRO — macros XML para URDF reutilizable' },
];

// ── Tag anatomy ────────────────────────────────────────────────
const tagParts = [
  { token: '<',          label: 'apertura',  color: '#64748b' },
  { token: 'link',       label: 'elemento',  color: '#f87171' },
  { token: ' name=',     label: 'atributo',  color: '#f97316' },
  { token: '"base_link"', label: 'valor',    color: '#4ade80' },
  { token: '>',          label: 'cierre >',  color: '#64748b' },
  { token: '...',        label: 'contenido', color: '#94a3b8' },
  { token: '</link>',    label: 'cierre tag', color: '#f87171' },
];

const tagTypes = [
  {
    type: 'Elemento con contenido', color: '#60a5fa',
    code: '<description>\n  Robot móvil de 4 ruedas\n</description>',
    use: 'Texto largo, listas, elementos hijo',
  },
  {
    type: 'Elemento auto-cerrado', color: '#4ade80',
    code: '<cylinder radius="0.1" length="0.5"/>',
    use: 'Sin contenido — solo atributos. La / antes de > cierra la etiqueta',
  },
];

const attrCode = '<cylinder radius="0.2" length="0.6"/>\n<joint name="wheel_joint" type="continuous"/>';
const contentCode = '<description>\n  Robot móvil autónomo con sensores\n  de profundidad y LiDAR 360°\n</description>';

const attrBullets  = ['Valores numéricos', 'IDs y nombres únicos', 'Flags (true/false)', 'Metadatos cortos'];
const contentBullets = ['Descripciones de texto', 'Listas de elementos', 'Estructuras anidadas', 'Datos complejos'];

// ── XML Rules ──────────────────────────────────────────────────
const xmlRules = [
  { num: 1, title: 'Cierre obligatorio',    icon: 'lock',        color: '#f87171', bad: '<link>',              good: '<link></link> o <link/>' },
  { num: 2, title: 'Case sensitive',         icon: 'text_format', color: '#f97316', bad: '<Robot> ≠ <robot>',   good: 'minúsculas siempre' },
  { num: 3, title: 'Una sola raíz',          icon: 'account_tree',color: '#fbbf24', bad: 'dos <robot> al mismo nivel', good: 'un solo <robot> raíz' },
  { num: 4, title: 'Comillas en atributos',  icon: 'format_quote',color: '#4ade80', bad: 'name=base',           good: 'name="base"' },
  { num: 5, title: 'Anidación correcta',     icon: 'layers',      color: '#60a5fa', bad: '<a><b></a></b>',      good: '<a><b></b></a>' },
];

const xmlEntities = [
  { entity: '&lt;',   char: '<', name: 'menor que' },
  { entity: '&gt;',   char: '>', name: 'mayor que' },
  { entity: '&amp;',  char: '&', name: 'ampersand' },
  { entity: '&quot;', char: '"', name: 'comilla doble' },
  { entity: '&apos;', char: "'", name: 'comilla simple' },
];

// ── Dependency types ───────────────────────────────────────────
const depTypes = [
  { tag: '<depend>',          color: '#4ade80', when: 'Build + ejecución (el más común)', example: 'rclcpp, std_msgs' },
  { tag: '<build_depend>',    color: '#60a5fa', when: 'Solo para compilar',               example: 'rosidl_default_generators' },
  { tag: '<exec_depend>',     color: '#fbbf24', when: 'Solo en tiempo de ejecución',      example: 'ros2launch' },
  { tag: '<buildtool_depend>',color: '#c084fc', when: 'Herramientas de build',            example: 'ament_cmake, ament_python' },
  { tag: '<test_depend>',     color: '#f97316', when: 'Solo para tests',                  example: 'ament_lint_auto' },
];

// ── Joint types ────────────────────────────────────────────────
const jointTypes = [
  { type: 'fixed',      icon: 'lock',        color: '#94a3b8', dof: '0 DOF', desc: 'Sin movimiento. Las piezas están soldadas.',                 use: 'Cámara al chassis, sensor al brazo' },
  { type: 'revolute',   icon: 'rotate_right', color: '#60a5fa', dof: '1 DOF', desc: 'Rotación con límites (tiene <limit>). ',                    use: 'Articulaciones de brazo robótico' },
  { type: 'continuous', icon: 'refresh',      color: '#4ade80', dof: '1 DOF', desc: 'Rotación sin límites. Gira libre.',                          use: 'Ruedas de robot diferencial' },
  { type: 'prismatic',  icon: 'open_in_full', color: '#fbbf24', dof: '1 DOF', desc: 'Movimiento lineal con límites.',                             use: 'Actuador lineal, elevador' },
  { type: 'floating',   icon: 'open_with',    color: '#c084fc', dof: '6 DOF', desc: 'Libre en los 6 grados de libertad.',                        use: 'Base móvil en simulación' },
  { type: 'planar',     icon: 'crop_free',    color: '#f97316', dof: '3 DOF', desc: 'Movimiento en un plano (x, y, rotZ).',                      use: 'Robot sobre superficie plana' },
];

// ── URDF tree ──────────────────────────────────────────────────
const urdfTree = [
  { level: 0, type: 'link',  name: 'base_link',        color: '#60a5fa', desc: 'Cuerpo principal del robot — raíz del árbol URDF' },
  { level: 1, type: 'joint', name: 'left_wheel_joint',  color: '#fbbf24', jtype: 'continuous', desc: 'Joint rueda izquierda — rota libremente' },
  { level: 2, type: 'link',  name: 'left_wheel',        color: '#4ade80', desc: 'Rueda izquierda — cilindro de 0.1m radio' },
  { level: 1, type: 'joint', name: 'right_wheel_joint', color: '#fbbf24', jtype: 'continuous', desc: 'Joint rueda derecha' },
  { level: 2, type: 'link',  name: 'right_wheel',       color: '#4ade80', desc: 'Rueda derecha — misma geometría que la izquierda' },
  { level: 1, type: 'joint', name: 'lidar_joint',       color: '#c084fc', jtype: 'fixed', desc: 'LiDAR fijo al chassis — no se mueve' },
  { level: 2, type: 'link',  name: 'lidar_link',        color: '#f97316', desc: 'Frame de referencia del LiDAR' },
  { level: 1, type: 'joint', name: 'camera_joint',      color: '#c084fc', jtype: 'fixed', desc: 'Cámara fija — orientada al frente' },
  { level: 2, type: 'link',  name: 'camera_link',       color: '#f97316', desc: 'Frame de referencia de la cámara' },
];

// ── XACRO features ─────────────────────────────────────────────
const xacroFeatures = [
  {
    name: 'Propiedades', color: '#fbbf24', syntax: 'xacro:property',
    desc: 'Variables reutilizables. Cambiar el valor en un lugar afecta a todo el archivo.',
    code: '<xacro:property name="wheel_r" value="0.1"/>\n<xacro:property name="wheel_l" value="0.05"/>\n<!-- Usar con ${} -->\n<cylinder radius="${wheel_r}" length="${wheel_l}"/>',
  },
  {
    name: 'Macros', color: '#60a5fa', syntax: 'xacro:macro',
    desc: 'Bloques reutilizables con parámetros. Como funciones en código.',
    code: '<xacro:macro name="wheel" params="name side">\n  <link name="${name}">\n    <visual>\n      <geometry>\n        <cylinder radius="${wheel_r}"\n                  length="${wheel_l}"/>\n      </geometry>\n    </visual>\n  </link>\n</xacro:macro>\n<!-- Invocar la macro -->\n<xacro:wheel name="left_wheel" side="-1"/>',
  },
  {
    name: 'Includes', color: '#4ade80', syntax: 'xacro:include',
    desc: 'Divide el URDF en archivos — sensores en un archivo, base en otro.',
    code: '<!-- robot.urdf.xacro -->\n<xacro:include filename="$(find mi_pkg)/urdf/sensors.urdf.xacro"/>\n<xacro:include filename="$(find mi_pkg)/urdf/base.urdf.xacro"/>',
  },
  {
    name: 'Expresiones matemáticas', color: '#c084fc', syntax: '${expr}',
    desc: 'Cálculos directamente en el XML. Usa pi, +, *, / en atributos numéricos.',
    code: '<!-- Posicionar ruedas simétricamente -->\n<origin xyz="${wheel_offset} 0 0"\n         rpy="${pi/2} 0 0"/>\n\n<xacro:property name="wheel_offset" value="0.15"/>\n<!-- ${pi} = 3.14159... -->',
  },
];

// ── Launch elements ────────────────────────────────────────────
const launchElements = [
  { tag: '<arg name="" default=""/>',    color: '#4ade80', desc: 'Parámetro configurable desde CLI', example: 'ros2 launch pkg f.xml robot_name:=r2d2' },
  { tag: '<node pkg="" exec="" name=""/>', color: '#60a5fa', desc: 'Lanza un nodo de un paquete',   example: '<node pkg="turtlesim" exec="turtlesim_node">' },
  { tag: '<param name="" value=""/>',    color: '#fbbf24', desc: 'Parámetro para el nodo',          example: '<param name="max_speed" value="2.0"/>' },
  { tag: '<remap from="" to=""/>',       color: '#c084fc', desc: 'Redirige un topic a otro nombre', example: 'from="/cmd_vel" to="/robot/cmd_vel"' },
  { tag: '<include file=""/>',           color: '#f97316', desc: 'Incluye otro launch file',         example: '$(find-pkg-share pkg)/launch/f.xml' },
  { tag: '$(var name)',                  color: '#f87171', desc: 'Sustituye el valor de un arg',     example: '$(var robot_name)' },
];

// ── Validation tools ───────────────────────────────────────────
const validationTools = [
  {
    name: 'xmllint — XML genérico', icon: 'check_circle', color: '#4ade80',
    desc: 'Verifica que el XML sea well-formed. Sin output = sin errores.',
    code: '# Instalar\nsudo apt install libxml2-utils\n\n# Validar package.xml\nxmllint --noout package.xml\n\n# Validar con formato amigable\nxmllint --format robot.urdf | head -20',
  },
  {
    name: 'check_urdf — URDF específico', icon: 'precision_manufacturing', color: '#60a5fa',
    desc: 'Verifica la semántica URDF: joints referenciando links que no existen, árboles ciclicos, etc.',
    code: '# Instalar\nsudo apt install liburdfdom-tools\n\n# Validar URDF\ncheck_urdf robot.urdf\n\n# Generar árbol visual\nurdf_to_graphiz robot.urdf\nevince robot.gv.pdf',
  },
  {
    name: 'xacro — procesar XACRO', icon: 'transform', color: '#c084fc',
    desc: 'Convierte el archivo .urdf.xacro en un URDF puro para inspección o validación.',
    code: '# Instalar (ya incluido en ros-jazzy-xacro)\nsudo apt install ros-jazzy-xacro\n\n# Convertir XACRO → URDF\nxacro robot.urdf.xacro > robot.urdf\n\n# Validar el resultado\ncheck_urdf robot.urdf',
  },
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    type: 'Error de parser XML',
    msg: 'XML parse error: mismatched tag at line 42',
    summary: 'Etiqueta abierta sin cerrar o mal anidada',
    color: '#f87171',
    cause: 'El parser encontró una etiqueta de cierre que no coincide con la última apertura. Es el error más común en XML.',
    steps: [
      'Usa xmllint --noout archivo.xml para ubicar la línea exacta',
      'Verifica que cada <etiqueta> tiene su </etiqueta> correspondiente',
      'VS Code con extensión XML resalta automáticamente las etiquetas sin cerrar',
    ],
    code: '<!-- ❌ Mal anidado -->\n<link>\n  <visual>\n</link>     <!-- visual no fue cerrado antes -->\n</visual>\n\n<!-- ✅ Correcto -->\n<link>\n  <visual>\n  </visual>\n</link>',
    open: false,
  },
  {
    type: 'Error de URDF — link no encontrado',
    msg: "check_urdf: Link 'wheel_link' was not found",
    summary: 'Un joint referencia un link que no está definido en el URDF',
    color: '#fbbf24',
    cause: 'El atributo parent/child en un joint usa un nombre de link que no existe o está mal escrito.',
    steps: [
      'Verifica que el name= del link y el link= del joint sean idénticos (case sensitive)',
      'Ejecuta check_urdf y lee el mensaje completo — indica qué joint tiene el problema',
      'Lista todos los links: grep "link name=" robot.urdf',
    ],
    code: '<!-- ❌ Nombre inconsistente -->\n<link name="wheel_link"/>\n<joint name="j1">\n  <parent link="base"/>\n  <child link="Wheel_link"/>  <!-- M mayúscula -->\n</joint>\n\n<!-- ✅ Nombres exactamente iguales -->\n<child link="wheel_link"/>',
    open: false,
  },
  {
    type: 'Error de XACRO — propiedad no definida',
    msg: "xacro: Undefined property: wheel_r",
    summary: 'Se usa ${wheel_r} pero la propiedad no fue declarada antes',
    color: '#c084fc',
    cause: 'Las propiedades XACRO deben declararse con xacro:property ANTES de usarlas en el archivo.',
    steps: [
      'Agrega <xacro:property name="wheel_r" value="0.1"/> al inicio del archivo',
      'Si la propiedad viene de un include, verifica que el include esté antes del uso',
      'Verifica typos: el nombre en xacro:property y en ${} deben ser idénticos',
    ],
    code: '<!-- ❌ Usa antes de definir -->\n<cylinder radius="${wheel_r}"/>\n<xacro:property name="wheel_r" value="0.1"/>\n\n<!-- ✅ Definir antes de usar -->\n<xacro:property name="wheel_r" value="0.1"/>\n<cylinder radius="${wheel_r}"/>',
    open: false,
  },
  {
    type: 'Error de package.xml',
    msg: "Package 'rclcpp' not found — ament could not resolve dependencies",
    summary: 'Una dependencia declarada en package.xml no está instalada',
    color: '#f97316',
    cause: 'El paquete está en package.xml pero no está instalado en el sistema (apt) ni está en el workspace.',
    steps: [
      'Instala la dependencia: sudo apt install ros-jazzy-rclcpp',
      'O usa rosdep: rosdep install --from-paths src --ignore-src -r -y',
      'Verifica el nombre exacto: ros2 pkg list | grep rclcpp',
    ],
    code: '<!-- package.xml -->\n<depend>rclcpp</depend>\n\n# Instalar dependencias automáticamente:\ncd ~/ros2_ws\nrosdep install --from-paths src --ignore-src -r -y\n\n# Luego compilar:\ncolcon build',
    open: false,
  },
  {
    type: 'Error de caracteres especiales',
    msg: 'XML parse error: not well-formed (invalid token) at line 15',
    summary: 'Un carácter especial (< > &) aparece sin escapar dentro de texto o atributos',
    color: '#4ade80',
    cause: 'El parser XML interpreta < como inicio de etiqueta y & como inicio de entidad. Si aparecen en texto deben escaparse.',
    steps: [
      'Reemplaza < por &lt; dentro de texto',
      'Reemplaza & por &amp;',
      'En atributos numéricos como radius="0.1" no hay problema — las entidades solo aplican dentro de texto y valores de atributos con esos caracteres',
    ],
    code: '<!-- ❌ < dentro de texto -->\n<condition>speed < 2.0</condition>\n\n<!-- ✅ Escapado -->\n<condition>speed &lt; 2.0</condition>\n\n<!-- En atributos numéricos está bien -->\n<limit effort="100" velocity="2.0"/>',
    open: false,
  },
]);

// ── Challenge ──────────────────────────────────────────────────
const challengeSteps = [
  { num: 1, color: '#60a5fa', text: 'Crea package.xml para "mi_robot_description" con dependencias: rclcpp, urdf, xacro' },
  { num: 2, color: '#fbbf24', text: 'Crea robot.urdf.xacro con propiedad wheel_radius=0.05 y wheel_width=0.04' },
  { num: 3, color: '#4ade80', text: 'Define xacro:macro "wheel" con params "name side" y un joint continuous' },
  { num: 4, color: '#c084fc', text: 'Instancia 2 ruedas: left_wheel (side=1) y right_wheel (side=-1)' },
  { num: 5, color: '#f97316', text: 'Procesa el XACRO con: xacro robot.urdf.xacro > robot.urdf' },
  { num: 6, color: '#f87171', text: 'Valida con: check_urdf robot.urdf — sin errores' },
];

const challengeHints = [
  'El header del xacro: <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mi_robot">',
  'En el macro: <origin xyz="${side * 0.15} 0 0" rpy="0 0 0"/> para posicionar las ruedas',
  'Cada rueda necesita su link Y su joint referenciando parent=base_link',
  'El joint continuous no lleva <limit> — las ruedas giran libremente',
  'rosdep install --from-paths src --ignore-src -r -y instala las dependencias automáticamente',
  'urdf_to_graphiz robot.urdf genera un PDF del árbol de links y joints',
];

// ── Summary ────────────────────────────────────────────────────
const summaryItems = [
  { cmd: 'package.xml',    desc: 'Manifiesto del paquete — metadatos y dependencias',     example: '<depend>rclcpp</depend>',     color: '#60a5fa' },
  { cmd: '<depend>',       desc: 'Dependencia de build + ejecución (más común)',           example: '<depend>std_msgs</depend>',   color: '#4ade80' },
  { cmd: 'URDF',           desc: 'Descripción física del robot en XML',                   example: '<link> + <joint>',            color: '#fbbf24' },
  { cmd: '<link>',         desc: 'Pieza física del robot con geometría e inercia',         example: '<cylinder radius="0.1"/>',    color: '#c084fc' },
  { cmd: '<joint>',        desc: 'Conexión entre dos links — define el movimiento',        example: 'type="continuous"',           color: '#f97316' },
  { cmd: 'xacro:property', desc: 'Variable reutilizable en XACRO',                        example: 'name="wheel_r" value="0.1"', color: '#f87171' },
  { cmd: 'xacro:macro',    desc: 'Bloque reutilizable con parámetros',                    example: 'params="name side"',          color: '#22d3ee' },
  { cmd: 'xmllint',        desc: 'Validador de XML genérico',                              example: 'xmllint --noout pkg.xml',     color: '#94a3b8' },
  { cmd: 'check_urdf',     desc: 'Validador semántico de URDF',                           example: 'check_urdf robot.urdf',       color: '#4ade80' },
];

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════════════════

const packageXmlCode = [
  '<?xml version="1.0"?>',
  '<package format="3">',
  '  <!-- ── Metadatos obligatorios ───────────────────── -->',
  '  <name>mi_robot_control</name>',
  '  <version>1.0.0</version>',
  '  <description>Sistema de control para robot diferencial</description>',
  '',
  '  <!-- ── Contacto (obligatorio) ───────────────────── -->',
  '  <maintainer email="alex@robot.com">Alexander</maintainer>',
  '  <license>Apache-2.0</license>',
  '',
  '  <!-- ── Herramienta de build ──────────────────────── -->',
  '  <buildtool_depend>ament_cmake</buildtool_depend>',
  '  <!-- Para Python: ament_python en lugar de ament_cmake -->',
  '',
  '  <!-- ── Build + runtime (el más común) ───────────── -->',
  '  <depend>rclcpp</depend>',
  '  <depend>std_msgs</depend>',
  '  <depend>geometry_msgs</depend>',
  '  <depend>sensor_msgs</depend>',
  '',
  '  <!-- ── Solo compilación ──────────────────────────── -->',
  '  <build_depend>rosidl_default_generators</build_depend>',
  '',
  '  <!-- ── Solo ejecución ────────────────────────────── -->',
  '  <exec_depend>ros2launch</exec_depend>',
  '  <exec_depend>rosidl_default_runtime</exec_depend>',
  '',
  '  <!-- ── Testing ───────────────────────────────────── -->',
  '  <test_depend>ament_lint_auto</test_depend>',
  '  <test_depend>ament_cmake_gtest</test_depend>',
  '',
  '  <export>',
  '    <build_type>ament_cmake</build_type>',
  '  </export>',
  '</package>',
].join('\n');

const urdfCode = [
  '<?xml version="1.0"?>',
  '<robot name="differential_robot">',
  '',
  '  <!-- ══ LINK: cuerpo principal ═════════════════════ -->',
  '  <link name="base_link">',
  '    <visual>',
  '      <geometry>',
  '        <box size="0.4 0.3 0.1"/>',
  '      </geometry>',
  '      <material name="gray">',
  '        <color rgba="0.5 0.5 0.5 1.0"/>',
  '      </material>',
  '    </visual>',
  '    <collision>       <!-- para física de simulación -->',
  '      <geometry>',
  '        <box size="0.4 0.3 0.1"/>',
  '      </geometry>',
  '    </collision>',
  '    <inertial>        <!-- masa e inercia (obligatorio en simulación) -->',
  '      <mass value="5.0"/>',
  '      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>',
  '    </inertial>',
  '  </link>',
  '',
  '  <!-- ══ LINK: rueda izquierda ═══════════════════════ -->',
  '  <link name="left_wheel">',
  '    <visual>',
  '      <geometry>',
  '        <cylinder radius="0.05" length="0.04"/>',
  '      </geometry>',
  '      <material name="black">',
  '        <color rgba="0.1 0.1 0.1 1.0"/>',
  '      </material>',
  '    </visual>',
  '    <collision>',
  '      <geometry>',
  '        <cylinder radius="0.05" length="0.04"/>',
  '      </geometry>',
  '    </collision>',
  '  </link>',
  '',
  '  <!-- ══ JOINT: base → rueda izquierda ═══════════════ -->',
  '  <joint name="left_wheel_joint" type="continuous">',
  '    <parent link="base_link"/>',
  '    <child link="left_wheel"/>',
  '    <origin xyz="0 0.17 0" rpy="-1.5708 0 0"/>',
  '    <!-- rpy="-pi/2 0 0" orienta el cilindro horizontalmente -->',
  '    <axis xyz="0 0 1"/>',
  '  </joint>',
  '',
  '</robot>',
].join('\n');

const urdfRepeatCode = [
  '<!-- Sin XACRO: cada rueda copiada a mano -->',
  '<link name="left_wheel">',
  '  <visual><geometry>',
  '    <cylinder radius="0.1" length="0.05"/>',
  '  </geometry></visual>',
  '</link>',
  '',
  '<link name="right_wheel">',
  '  <visual><geometry>',
  '    <!-- Copiar y pegar — error si cambia 0.1 -->',
  '    <cylinder radius="0.1" length="0.05"/>',
  '  </geometry></visual>',
  '</link>',
  '',
  '<!-- Con 6 ruedas → 6 copias = mantenimiento imposible -->',
].join('\n');

const xacroCode = [
  '<?xml version="1.0"?>',
  '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"',
  '       name="differential_robot">',
  '',
  '  <!-- ── Propiedades (variables) ─────────────────── -->',
  '  <xacro:property name="wheel_r" value="0.1"/>',
  '  <xacro:property name="wheel_l" value="0.05"/>',
  '  <xacro:property name="wheel_offset" value="0.15"/>',
  '',
  '  <!-- ── Macro reutilizable ───────────────────────── -->',
  '  <xacro:macro name="wheel" params="name side">',
  '    <link name="${name}">',
  '      <visual><geometry>',
  '        <cylinder radius="${wheel_r}" length="${wheel_l}"/>',
  '      </geometry></visual>',
  '    </link>',
  '    <joint name="${name}_joint" type="continuous">',
  '      <parent link="base_link"/>',
  '      <child link="${name}"/>',
  '      <origin xyz="0 ${side * wheel_offset} 0"/>',
  '      <axis xyz="0 0 1"/>',
  '    </joint>',
  '  </xacro:macro>',
  '',
  '  <!-- ── Invocar macro ────────────────────────────── -->',
  '  <xacro:wheel name="left_wheel"  side="1"/>',
  '  <xacro:wheel name="right_wheel" side="-1"/>',
  '  <!-- Cambiar wheel_r arriba → afecta AMBAS ruedas -->',
].join('\n');

const launchXmlCode = [
  '<launch>',
  '  <!-- ── Argumentos configurables ─────────────────── -->',
  '  <arg name="robot_name" default="robot1"/>',
  '  <arg name="use_sim"    default="true"/>',
  '  <arg name="max_speed"  default="2.0"/>',
  '',
  '  <!-- ── Nodo de control ───────────────────────────── -->',
  '  <node pkg="robot_control"',
  '        exec="control_node"',
  '        name="$(var robot_name)_controller">',
  '    <param name="robot_name" value="$(var robot_name)"/>',
  '    <param name="max_speed"  value="$(var max_speed)"/>',
  '    <!-- Remap: el nodo publica /cmd_vel → /robot1/cmd_vel -->',
  '    <remap from="/cmd_vel" to="/$(var robot_name)/cmd_vel"/>',
  '  </node>',
  '',
  '  <!-- ── Nodo de navegación ────────────────────────── -->',
  '  <node pkg="robot_nav" exec="nav_node" name="navigator">',
  '    <param name="use_sim" value="$(var use_sim)"/>',
  '  </node>',
  '',
  '  <!-- ── Incluir otro launch ───────────────────────── -->',
  '  <include file="$(find-pkg-share robot_sensors)/launch/sensors.launch.xml">',
  '    <arg name="use_lidar" value="true"/>',
  '  </include>',
  '',
  '</launch>',
  '',
  '<!-- Ejecutar con argumento:',
  '     ros2 launch mi_pkg robot.launch.xml robot_name:=r2d2 -->',
].join('\n');

const challengeCode = [
  '<!-- Objetivo: package.xml + robot.urdf.xacro con macros -->',
  '',
  '<!-- package.xml -->',
  '<package format="3">',
  '  <name>mi_robot_description</name>',
  '  <!-- TODO: versión, descripción, maintainer, license -->',
  '  <!-- TODO: dependencias: rclcpp, urdf, xacro -->',
  '</package>',
  '',
  '<!-- robot.urdf.xacro -->',
  '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mi_robot">',
  '  <!-- TODO: propiedad wheel_radius=0.05 -->',
  '  <!-- TODO: propiedad wheel_width=0.04 -->',
  '',
  '  <!-- TODO: link base_link (caja 0.3×0.2×0.1) -->',
  '',
  '  <!-- TODO: macro "wheel" con params "name side" -->',
  '  <!--   - link ${name} con cilindro radius/length -->',
  '  <!--   - joint continuous base_link → ${name} -->',
  '  <!--   - origin xyz="0 ${side*0.12} 0" -->',
  '',
  '  <!-- TODO: invocar macro para left_wheel (side=1) -->',
  '  <!-- TODO: invocar macro para right_wheel (side=-1) -->',
  '</robot>',
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
   TAG ANATOMY
══════════════════════════════════════════ */
.tag-anatomy {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 2rem 1.5rem;
}
.ta-row { display: flex; align-items: flex-end; gap: 2px; justify-content: center; flex-wrap: wrap; margin-bottom: 1.5rem; }
.ta-part { display: flex; flex-direction: column; align-items: center; gap: 4px; }
.tap-token { font-family: 'Fira Code', monospace; font-size: 1.1rem; font-weight: 700; color: var(--ta-color); background: color-mix(in srgb, var(--ta-color) 10%, transparent); border-radius: 6px; padding: 4px 8px; white-space: nowrap; }
.tap-arrow { font-size: .9rem; color: var(--ta-color); opacity: .7; }
.tap-label { font-size: .72rem; font-weight: 700; color: var(--ta-color); text-align: center; }
.ta-full-tag { text-align: center; font-family: 'Fira Code', monospace; font-size: .82rem; color: var(--text-muted); border-top: 1px dashed var(--border-subtle); padding-top: 1rem; }
.ta-token-inline { padding: 1px 2px; }

.tag-types { }
.tt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.tt-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 12px; }
.tt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--tt-color, var(--border-medium));
  border-radius: 12px; padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.ttc-type { font-size: .85rem; font-weight: 700; }
.ttc-use  { font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 6px 9px; }

.attr-vs-content { }
.avc-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.avc-panel { display: flex; flex-direction: column; gap: 8px; }
.avcp-header { display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700; padding: 8px 12px; border-radius: 8px; }
.avcp-attr    { background: rgba(249,115,22,.1); color: #f97316; }
.avcp-content { background: rgba(96,165,250,.1);  color: #60a5fa; }
.avcp-bullets { display: flex; flex-direction: column; gap: 5px; padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }
.avcp-item    { display: flex; align-items: center; gap: 6px; font-size: .82rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   RULES GRID
══════════════════════════════════════════ */
.rules-grid { display: grid; grid-template-columns: repeat(5,1fr); gap: 12px; }
.rule-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--rule-color); border-radius: 14px;
  padding: 1rem; display: flex; flex-direction: column; align-items: center; gap: 8px; text-align: center;
  transition: all .2s;
}
.rule-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.rc-num {
  width: 28px; height: 28px; border-radius: 50%;
  font-size: .85rem; font-weight: 800; color: #1e1e1e;
  display: flex; align-items: center; justify-content: center;
}
.rc-icon  { }
.rc-title { font-size: .85rem; font-weight: 700; color: var(--text-primary); }
.rc-bad   { display: flex; align-items: center; font-size: .75rem; color: var(--text-muted); }
.rc-good  { display: flex; align-items: center; font-size: .75rem; color: var(--text-secondary); }
.rc-bad code, .rc-good code { font-size: .72rem; background: none; padding: 0; }

.entities-section { }
.es-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.es-grid  { display: grid; grid-template-columns: repeat(5,1fr); gap: 10px; }
.es-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 10px; padding: 10px 12px;
  display: flex; flex-direction: column; align-items: center; gap: 4px; text-align: center;
}
.esc-entity { font-size: .82rem; font-weight: 700; color: #f97316; background: none; padding: 0; }
.esc-arrow  { font-size: .9rem; color: var(--text-muted); }
.esc-char   { font-size: 1rem; font-weight: 800; color: var(--text-primary); background: none; padding: 0; }
.esc-name   { font-size: .7rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   DEPENDENCY TYPES
══════════════════════════════════════════ */
.dep-types { }
.dt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.dt-grid  { display: grid; grid-template-columns: repeat(5,1fr); gap: 10px; }
.dt-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--dt-color); border-radius: 10px;
  padding: 10px 12px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.dtc-tag     { font-family: 'Fira Code', monospace; font-size: .8rem; font-weight: 700; color: var(--dt-color); background: none; padding: 0; word-break: break-all; }
.dtc-when    { font-size: .78rem; color: var(--text-secondary); line-height: 1.3; }
.dtc-example { font-size: .72rem; color: var(--text-muted); font-family: 'Fira Code', monospace; }

/* ══════════════════════════════════════════
   JOINT TYPES
══════════════════════════════════════════ */
.joint-types { }
.jt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.jt-grid  { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.jt-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--jt-color); border-radius: 12px;
  padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.jtc-header { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.jtc-type   { font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 700; color: var(--jt-color); background: none; padding: 0; }
.jtc-dof    { font-size: .68rem; font-weight: 800; letter-spacing: .05em; padding: 2px 7px; border-radius: 999px; white-space: nowrap; }
.jtc-desc   { font-size: .81rem; color: var(--text-secondary); line-height: 1.4; }
.jtc-use    { display: flex; align-items: center; font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 5px 9px; }

/* URDF Tree */
.urdf-tree  { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden; }
.ut-title   { display: flex; align-items: center; gap: 8px; padding: 12px 16px; border-bottom: 1px solid var(--border-subtle); font-size: .88rem; font-weight: 600; color: var(--text-secondary); background: var(--bg-surface-solid); }
.ut-body    { padding: 6px 0; }
.ut-node    { padding: 8px 14px; border-bottom: 1px solid var(--border-subtle); display: flex; align-items: flex-start; gap: 10px; transition: background .15s; }
.ut-node:last-child { border-bottom: none; }
.ut-node:hover      { background: var(--bg-surface-hover); }
.utn-main   { display: flex; align-items: center; gap: 7px; min-width: 320px; flex-shrink: 0; }
.utn-tag    { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 600; background: none; padding: 0; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.utn-desc   { font-size: .77rem; color: var(--text-muted); flex: 1; }

/* ══════════════════════════════════════════
   XACRO
══════════════════════════════════════════ */
.xacro-panel { display: flex; flex-direction: column; gap: 8px; }
.xcp-header { display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700; padding: 8px 12px; border-radius: 8px; }
.xacro-bad  .xcp-header { background: rgba(251,191,36,.1); color: #fbbf24; }
.xacro-good .xcp-header { background: rgba(74,222,128,.1);  color: #4ade80; }
.xcp-note { font-size: .81rem; color: var(--text-secondary); border-radius: 7px; padding: 8px 12px; border: 1px solid var(--border-subtle); line-height: 1.4; }
.xcp-note-bad  { background: rgba(251,191,36,.05); }
.xcp-note-good { background: rgba( 74,222,128,.05); }

.xacro-features { }
.xf-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.xf-grid  { display: grid; grid-template-columns: repeat(2,1fr); gap: 14px; }
.xf-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--xf-color); border-radius: 12px;
  padding: 1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.xfc-syntax { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; color: var(--text-muted); background: none; padding: 0; }
.xfc-name   { font-size: .9rem; font-weight: 700; }
.xfc-desc   { font-size: .81rem; color: var(--text-secondary); line-height: 1.4; }

/* ══════════════════════════════════════════
   LAUNCH ELEMENTS
══════════════════════════════════════════ */
.launch-elements { }
.le-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.le-grid  { display: grid; grid-template-columns: repeat(3,1fr); gap: 10px; }
.le-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--le-color); border-radius: 10px;
  padding: 10px 12px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.lec-tag  { font-family: 'Fira Code', monospace; font-size: .78rem; font-weight: 700; color: var(--le-color); background: none; padding: 0; word-break: break-all; }
.lec-desc { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }
.lec-ex   { font-size: .73rem; color: var(--text-muted); font-style: italic; }

/* ══════════════════════════════════════════
   VALIDATION
══════════════════════════════════════════ */
.validation-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.vt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--vt-color); border-radius: 14px;
  padding: 1.1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.vtc-header { display: flex; align-items: center; gap: 8px; }
.vtc-name   { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.vtc-desc   { font-size: .81rem; color: var(--text-secondary); line-height: 1.4; }

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
.summary-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.summary-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 4px solid var(--sc-color); border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s; }
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; }
.sc-desc    { font-size: .81rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1100px) {
  .rules-grid { grid-template-columns: repeat(3,1fr); }
  .es-grid    { grid-template-columns: repeat(3,1fr); }
  .dt-grid    { grid-template-columns: repeat(3,1fr); }
}
@media (max-width: 900px) {
  .rules-grid      { grid-template-columns: repeat(2,1fr); }
  .es-grid         { grid-template-columns: repeat(3,1fr); }
  .dt-grid         { grid-template-columns: repeat(2,1fr); }
  .jt-grid         { grid-template-columns: repeat(2,1fr); }
  .xf-grid         { grid-template-columns: 1fr; }
  .le-grid         { grid-template-columns: repeat(2,1fr); }
  .validation-grid { grid-template-columns: 1fr; }
  .summary-grid    { grid-template-columns: repeat(2,1fr); }
  .utn-main        { min-width: 0; }
  .utn-desc        { display: none; }
}
@media (max-width: 768px) {
  .rules-grid  { grid-template-columns: 1fr; }
  .tt-grid     { grid-template-columns: 1fr; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
}
@media (max-width: 480px) {
  .es-grid      { grid-template-columns: repeat(2,1fr); }
  .dt-grid      { grid-template-columns: 1fr; }
  .jt-grid      { grid-template-columns: 1fr; }
  .le-grid      { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: 1fr; }
}
</style>
