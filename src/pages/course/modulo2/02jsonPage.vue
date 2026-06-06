<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        JSON (JavaScript Object Notation) es el <strong>lenguaje de intercambio de datos</strong>
        más universal de la web. En ROS 2 lo encontrarás en <strong>rosbridge</strong>
        (para controlar robots desde el navegador), en APIs REST, en archivos de parámetros y en
        metadata de bags. Sin etiquetas, sin comentarios — solo pares clave-valor.
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
        Anatomía — Pares Clave-Valor
      </SectionTitle>

      <TextBlock>
        La unidad básica de JSON es el <strong>par clave-valor</strong>.
        Un objeto es una colección de pares entre <code>{}</code>.
        Un array es una lista ordenada entre <code>[]</code>.
        La clave es <em>siempre</em> un string entre comillas dobles.
      </TextBlock>

      <!-- Anatomy visual -->
      <div class="kv-anatomy q-mt-lg">
        <div class="kva-row">
          <div v-for="part in kvParts" :key="part.label" class="kva-part"
            :style="{ '--kva-color': part.color }">
            <div class="kvap-token">{{ part.token }}</div>
            <div class="kvap-arrow">↑</div>
            <div class="kvap-label">{{ part.label }}</div>
          </div>
        </div>
        <div class="kva-rule">
          La clave (<code>"nombre"</code>) es <strong>siempre un string</strong>.
          El valor puede ser cualquiera de los 6 tipos de dato JSON.
        </div>
      </div>

      <!-- 6 JSON Types grid -->
      <div class="types-section q-mt-xl">
        <div class="ts-title">
          <q-icon name="category" size="16px" color="primary" />
          Los 6 tipos de dato en JSON
        </div>
        <div class="types-grid">
          <div v-for="t in jsonTypes" :key="t.type" class="type-card"
            :style="{ '--tc-color': t.color }">
            <div class="tc-header">
              <q-icon :name="t.icon" size="18px" :style="{ color: t.color }" />
              <span class="tc-type">{{ t.type }}</span>
            </div>
            <CodeBlock :hide-header="true" lang="json" :content="t.example" />
            <div class="tc-use">{{ t.use }}</div>
          </div>
        </div>
      </div>

      <!-- Full nested example -->
      <CodeBlock title="robot_config.json — Objeto completo con anidación"
        lang="json" :content="robotConfigCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         02 REGLAS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        5 Reglas que Diferencian JSON de Python Dict
      </SectionTitle>

      <TextBlock>
        Python y JSON se ven similares pero <strong>no son lo mismo</strong>.
        Estas diferencias son la fuente de la mayoría de los errores al integrar Python con rosbridge:
      </TextBlock>

      <div class="rules-grid q-mt-lg">
        <div v-for="rule in jsonRules" :key="rule.num" class="rule-card"
          :style="{ '--rule-color': rule.color }">
          <div class="rc-num" :style="{ background: rule.color }">{{ rule.num }}</div>
          <q-icon :name="rule.icon" size="22px" :style="{ color: rule.color }" />
          <div class="rc-title">{{ rule.title }}</div>
          <div class="rc-bad">
            <q-icon name="cancel" size="13px" color="negative" class="q-mr-xs" />
            <code>{{ rule.bad }}</code>
          </div>
          <div class="rc-good">
            <q-icon name="check_circle" size="13px" color="positive" class="q-mr-xs" />
            <code>{{ rule.good }}</code>
          </div>
          <div class="rc-note">{{ rule.note }}</div>
        </div>
      </div>

      <!-- Python vs JSON comparison table -->
      <div class="compare-table q-mt-xl">
        <div class="ct-title">
          <q-icon name="compare" size="16px" color="primary" />
          Tabla de diferencias clave — Python Dict vs JSON
        </div>
        <div class="ct-body">
          <div class="ct-row ct-header">
            <div class="ct-cell">Característica</div>
            <div class="ct-cell ct-python">Python Dict</div>
            <div class="ct-cell ct-json">JSON</div>
          </div>
          <div v-for="row in comparisonRows" :key="row.feature" class="ct-row">
            <div class="ct-cell ct-feature">{{ row.feature }}</div>
            <div class="ct-cell ct-python-val">
              <code>{{ row.python }}</code>
            </div>
            <div class="ct-cell ct-json-val">
              <code>{{ row.json }}</code>
              <q-icon v-if="row.warn" name="warning" size="13px" color="warning" class="q-ml-xs" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 ROSBRIDGE
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        rosbridge — Controlar Robots desde el Navegador
      </SectionTitle>

      <TextBlock>
        <strong>rosbridge</strong> expone todos los topics y servicios de ROS 2 como
        mensajes JSON a través de un WebSocket. Puedes controlar tu robot desde
        cualquier página web, app móvil o script Python externo.
      </TextBlock>

      <!-- Architecture visual -->
      <div class="arch-visual q-mt-lg">
        <div v-for="(node, i) in archNodes" :key="node.label" class="arch-node-wrap">
          <div class="arch-node" :style="{ '--arch-color': node.color }">
            <q-icon :name="node.icon" size="22px" :style="{ color: node.color }" />
            <div class="an-label">{{ node.label }}</div>
            <div class="an-sub">{{ node.sub }}</div>
          </div>
          <div v-if="i < archNodes.length - 1" class="arch-arrow">
            <q-icon name="arrow_forward" size="18px" style="color:var(--text-muted)" />
            <div class="aa-proto">{{ archNodes[i]?.proto }}</div>
          </div>
        </div>
      </div>

      <!-- Install -->
      <CodeBlock title="Instalación de rosbridge" lang="bash"
        :content="rosbridgeInstallCode" :copyable="true" class="q-mt-xl" />

      <!-- Publish + Subscribe -->
      <SplitBlock class="q-mt-xl">
        <template #left>
          <div class="rb-panel">
            <div class="rbp-header rbp-pub">
              <q-icon name="upload" size="15px" />
              Publicar — Web → Robot
            </div>
            <CodeBlock :hide-header="true" lang="json"
              :content="rosbridgePublishCode" :copyable="true" />
            <div class="rbp-note">
              Envía velocidad lineal 0.5 m/s + angular 1.2 rad/s al robot
            </div>
          </div>
        </template>
        <template #right>
          <div class="rb-panel">
            <div class="rbp-header rbp-sub">
              <q-icon name="download" size="15px" />
              Suscribirse — Robot → Web
            </div>
            <CodeBlock :hide-header="true" lang="json"
              :content="rosbridgeSubscribeCode" :copyable="true" />
            <div class="rbp-note">
              Recibe datos del LiDAR a máximo 100 ms de intervalo (10 Hz)
            </div>
          </div>
        </template>
      </SplitBlock>

      <!-- Service call -->
      <div class="service-card q-mt-xl">
        <div class="sc-header">
          <q-icon name="settings_ethernet" size="16px" color="primary" />
          Llamar servicio — op: "call_service"
        </div>
        <SplitBlock>
          <template #left>
            <CodeBlock :hide-header="true" lang="json"
              :content="rosbridgeServiceReqCode" :copyable="true" />
          </template>
          <template #right>
            <CodeBlock :hide-header="true" lang="json"
              :content="rosbridgeServiceResCode" />
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 PYTHON + JSON en NODOS ROS 2
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        Python + JSON en Nodos ROS 2
      </SectionTitle>

      <TextBlock>
        El módulo estándar <code>json</code> de Python convierte entre strings JSON y
        diccionarios Python. En un nodo ROS 2 lo usarás para leer configuraciones al
        iniciar y para parsear mensajes String provenientes de rosbridge.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="py-panel">
            <div class="pyp-header">
              <q-icon name="swap_horiz" size="14px" color="positive" />
              Módulo <code>json</code> — operaciones básicas
            </div>
            <CodeBlock :hide-header="true" lang="python"
              :content="pythonJsonCode" :copyable="true" />
          </div>
        </template>
        <template #right>
          <div class="py-panel">
            <div class="pyp-header">
              <q-icon name="smart_toy" size="14px" color="primary" />
              Nodo ROS 2 que lee JSON al arrancar
            </div>
            <CodeBlock :hide-header="true" lang="python"
              :content="rclpyNodeCode" :copyable="true" />
          </div>
        </template>
      </SplitBlock>

      <AlertBlock type="info" title="Patrón: JSON como String topic para rosbridge" class="q-mt-lg">
        rosbridge puede enviar comandos complejos como un <code>std_msgs/String</code> con JSON
        dentro. El nodo parsea el JSON con <code>json.loads(msg.data)</code> y actúa.
        Siempre envuelve el parse en <code>try/except json.JSONDecodeError</code> para
        evitar que un mensaje malformado tire el nodo.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         05 JQ — CLI PARA JSON
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        jq — Procesar JSON en la Terminal
      </SectionTitle>

      <TextBlock>
        <code>jq</code> es el <strong>grep de JSON</strong>: filtra, transforma y valida
        JSON desde la terminal. Es esencial para depurar respuestas de rosbridge y para
        procesar logs de robots.
      </TextBlock>

      <CodeBlock title="Instalar y usar jq" lang="bash"
        :content="jqCode" :copyable="true" />

      <!-- jq filters cheatsheet -->
      <div class="jq-filters q-mt-xl">
        <div class="jf-title">
          <q-icon name="filter_list" size="16px" color="primary" />
          Filtros jq más usados
        </div>
        <div class="jf-grid">
          <div v-for="filt in jqFilters" :key="filt.filter" class="jf-card"
            :style="{ '--jf-color': filt.color }">
            <code class="jfc-filter">{{ filt.filter }}</code>
            <div class="jfc-desc">{{ filt.desc }}</div>
            <code class="jfc-ex">{{ filt.example }}</code>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Diccionario de Errores JSON</SectionTitle>

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
              <CodeBlock :hide-header="true" lang="json" :content="err.bad" />
              <CodeBlock :hide-header="true" lang="json" :content="err.good" />
            </div>
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive" />
              <strong>Solución rápida:</strong> {{ err.fix }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Config JSON + Nodo Python</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea el config JSON de tu robot y léelo en ROS 2</div>
            <div class="challenge-subtitle">
              Combina JSON estructurado con un nodo Python rclpy real
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
      <TextBlock>JSON en ROS 2 — rosbridge y manejo de datos:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="JSON en ROS 2" frameborder="0"
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
  { icon: '🌐', label: 'rosbridge — WebSocket JSON ↔ ROS 2 topics en tiempo real' },
  { icon: '📦', label: 'json.loads() / json.dumps() — conversión inmediata Python ↔ JSON' },
  { icon: '🔧', label: 'jq — herramienta CLI para filtrar y validar JSON' },
];

// ── KV anatomy ─────────────────────────────────────────────────
const kvParts = [
  { token: '{',          label: 'apertura objeto', color: '#64748b' },
  { token: '"nombre"',   label: 'clave — string',  color: '#fbbf24' },
  { token: ':',          label: 'separador',        color: '#64748b' },
  { token: '"TurtleBot"',label: 'valor — string',   color: '#4ade80' },
  { token: ',',          label: 'más pares',        color: '#94a3b8' },
  { token: '"vel"',      label: 'clave 2',          color: '#fbbf24' },
  { token: ':',          label: 'separador',        color: '#64748b' },
  { token: '2.5',        label: 'valor — number',   color: '#60a5fa' },
  { token: '}',          label: 'cierre objeto',    color: '#64748b' },
];

// ── JSON Types ─────────────────────────────────────────────────
const jsonTypes = [
  { type: 'String',  icon: 'text_fields',  color: '#4ade80', use: 'Nombres, IDs, rutas, mensajes de texto',
    example: '"nombre": "TurtleBot4"\n"modelo": "Differential"' },
  { type: 'Number',  icon: 'numbers',      color: '#60a5fa', use: 'Velocidades, posiciones, contadores, tiempos',
    example: '"velocidad": 2.5\n"ticks_encoder": 1024' },
  { type: 'Boolean', icon: 'toggle_on',    color: '#c084fc', use: 'Flags de estado, activar/desactivar sensores',
    example: '"activo": true\n"emergency_stop": false' },
  { type: 'Null',    icon: 'block',        color: '#94a3b8', use: 'Ausencia de valor, campos opcionales sin dato',
    example: '"ultimo_error": null\n"destino": null' },
  { type: 'Array',   icon: 'list',         color: '#f97316', use: 'Listas, coordenadas [x,y,z], múltiples sensores',
    example: '"posicion": [1.0, 2.5, 0.0]\n"sensores": ["lidar", "cam"]' },
  { type: 'Object',  icon: 'data_object',  color: '#fbbf24', use: 'Estructuras anidadas, sub-componentes del robot',
    example: '"bateria": {\n  "voltaje": 12.4,\n  "nivel": 85\n}' },
];

// ── JSON Rules ─────────────────────────────────────────────────
const jsonRules = [
  { num: 1, icon: 'format_quote', color: '#fbbf24', title: 'Comillas dobles obligatorias',
    bad: "{'key': 'val'}", good: '{"key": "val"}', note: 'Comillas simples no son JSON válido' },
  { num: 2, icon: 'remove',       color: '#f87171', title: 'Sin coma final (trailing comma)',
    bad: '{"a": 1, "b": 2,}',   good: '{"a": 1, "b": 2}', note: 'La coma final rompe el parser JSON' },
  { num: 3, icon: 'abc',          color: '#c084fc', title: 'Booleanos y null en minúscula',
    bad: '{"ok": True, "err": None}', good: '{"ok": true, "err": null}', note: 'Python usa True/False/None — JSON usa true/false/null' },
  { num: 4, icon: 'vpn_key',      color: '#60a5fa', title: 'Claves siempre entre comillas',
    bad: '{nombre: "Robot"}',   good: '{"nombre": "Robot"}', note: 'En JSON toda clave es un string — sin excepción' },
  { num: 5, icon: 'code_off',     color: '#f97316', title: 'Sin comentarios',
    bad: '{"vel": 2.5 // max}', good: '{"vel": 2.5}', note: 'JSON puro no soporta // ni /* */. Usa YAML para configs comentadas' },
];

// ── Comparison table ───────────────────────────────────────────
const comparisonRows = [
  { feature: 'Comillas en strings', python: "'simple' o \"doble\"", json: '"doble" solo', warn: true },
  { feature: 'Booleanos',           python: 'True / False',         json: 'true / false',  warn: true },
  { feature: 'Valor nulo',          python: 'None',                  json: 'null',           warn: true },
  { feature: 'Coma final',          python: '{"a": 1,} ✅',         json: '{"a": 1,} ❌',  warn: true },
  { feature: 'Comentarios',         python: '# esto es válido',     json: 'no soportado',   warn: true },
  { feature: 'Claves sin comillas', python: '{nombre: "x"} ✅',    json: '{nombre: "x"} ❌',warn: true },
];

// ── Rosbridge architecture ─────────────────────────────────────
const archNodes = [
  { label: 'Navegador / App', sub: 'JavaScript, React, etc.', icon: 'web',          color: '#60a5fa', proto: 'WebSocket ws://' },
  { label: 'rosbridge_server', sub: 'Puerto 9090',            icon: 'hub',          color: '#fbbf24', proto: 'JSON ↔ ROS 2' },
  { label: 'ROS 2 Topics',    sub: 'Jazzy topics/services',  icon: 'account_tree', color: '#4ade80', proto: '' },
];

// ── jq filters ────────────────────────────────────────────────
const jqFilters = [
  { filter: '. ',                 color: '#4ade80', desc: 'Identity — pretty print completo',         example: 'jq . config.json' },
  { filter: '.clave',             color: '#60a5fa', desc: 'Acceder a una clave del objeto raíz',      example: 'jq .nombre config.json' },
  { filter: '.obj.clave',         color: '#fbbf24', desc: 'Acceder a clave anidada',                  example: 'jq .robot.velocidad_max' },
  { filter: '.array[0]',          color: '#c084fc', desc: 'Primer elemento de un array',              example: 'jq .sensores[0]' },
  { filter: '.array[]',           color: '#f97316', desc: 'Iterar todos los elementos del array',     example: 'jq .sensores[]' },
  { filter: 'length',             color: '#f87171', desc: 'Largo de un array u objeto',               example: "jq '.sensores | length'" },
  { filter: 'keys',               color: '#22d3ee', desc: 'Lista de claves del objeto',               example: 'jq keys config.json' },
  { filter: 'select(cond)',       color: '#94a3b8', desc: 'Filtrar elementos que cumplan condición',   example: "jq '.[] | select(.activo)'" },
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    type: 'SyntaxError en Python',
    msg: "json.JSONDecodeError: Expecting property name enclosed in double quotes",
    summary: 'El JSON usa comillas simples o claves sin comillas',
    color: '#f87171',
    cause: 'Python acepta comillas simples en dicts, pero JSON requiere dobles. Al copiar de un print() de Python obtienes JSON inválido.',
    bad: "// ❌ Desde Python repr()\n{'nombre': 'TurtleBot', 'activo': True}",
    good: '// ✅ JSON válido\n{"nombre": "TurtleBot", "activo": true}',
    fix: 'Usa json.dumps() en Python para generar JSON válido, nunca str() ni repr().',
    open: false,
  },
  {
    type: 'Trailing comma',
    msg: "json.JSONDecodeError: Expecting value",
    summary: 'Coma después del último elemento — prohibida en JSON',
    color: '#fbbf24',
    cause: 'En Python los trailing commas están permitidos (y recomendados). En JSON rompen el parser. Es el error más común al escribir JSON a mano.',
    bad: '// ❌ Trailing comma\n{\n  "vel": 2.5,\n  "nombre": "Robot",\n}',
    good: '// ✅ Sin trailing comma\n{\n  "vel": 2.5,\n  "nombre": "Robot"\n}',
    fix: 'Borra la coma después del último campo. VS Code con extensión JSON resalta este error antes de guardar.',
    open: false,
  },
  {
    type: 'Booleanos y null de Python',
    msg: "json.JSONDecodeError: Expecting value at line 3 column 11",
    summary: 'True, False o None en lugar de true, false o null',
    color: '#c084fc',
    cause: 'Python imprime True/False/None con mayúscula. Al copiar o formatear manualmente JSON puedes mezclar estilos. json.loads() rechaza la mayúscula.',
    bad: '// ❌ Estilo Python\n{\n  "activo": True,\n  "bateria": None,\n  "stop": False\n}',
    good: '// ✅ Estilo JSON\n{\n  "activo": true,\n  "bateria": null,\n  "stop": false\n}',
    fix: 'Siempre genera JSON con json.dumps() — jamás construyas JSON con f-strings o concatenación.',
    open: false,
  },
  {
    type: 'Error en rosbridge',
    msg: 'rosbridge: received message with missing field "op"',
    summary: 'El mensaje JSON a rosbridge no tiene el campo "op" obligatorio',
    color: '#f97316',
    cause: 'Todos los mensajes de rosbridge requieren el campo "op" (publish, subscribe, call_service, etc.). Sin él el servidor rechaza el mensaje silenciosamente.',
    bad: '// ❌ Sin campo "op"\n{\n  "topic": "/cmd_vel",\n  "msg": {"linear": {"x": 0.5}}\n}',
    good: '// ✅ Con "op" correcto\n{\n  "op": "publish",\n  "topic": "/cmd_vel",\n  "msg": {"linear": {"x": 0.5}}\n}',
    fix: 'Revisa la documentación de rosbridge: cada operación tiene campos obligatorios (op, topic, type según la operación).',
    open: false,
  },
  {
    type: 'JSON anidado como string',
    msg: 'TypeError: string indices must be integers (inesperado al acceder .robot.nombre)',
    summary: 'El JSON fue cargado como string en lugar de dict — doble serialización',
    color: '#4ade80',
    cause: 'Si usas json.loads() en datos que ya son un dict (no string), o si el JSON está serializado dos veces (string dentro de string), el resultado es un string, no un dict.',
    bad: '// ❌ JSON dentro de JSON (doble serialización)\ndata = json.loads(json_string)  # data es un string con JSON\ndata["robot"]["nombre"]  # TypeError!',
    good: '// ✅ Una sola deserialización\ndata = json.loads(json_string)  # data es dict Python\ndata["robot"]["nombre"]  # "TurtleBot"',
    fix: 'Imprime type(data) para verificar que es dict, no str. Si es str, necesitas un json.loads() adicional.',
    open: false,
  },
]);

// ── Challenge ──────────────────────────────────────────────────
const challengeSteps = [
  { num: 1, color: '#60a5fa', text: 'Crea robot_config.json con campos: nombre, tipo, velocidad_max, sensores[] (≥2), bateria{}' },
  { num: 2, color: '#fbbf24', text: 'Valida con jq: jq . robot_config.json — debe imprimir formato bonito sin errores' },
  { num: 3, color: '#4ade80', text: 'Completa el nodo Python: lee el JSON, loggea los campos, itera los sensores activos' },
  { num: 4, color: '#c084fc', text: 'Ejecuta el nodo: python3 config_node.py — verifica la salida del logger' },
  { num: 5, color: '#f97316', text: '(Bonus) Instala rosbridge y envía un publish de cmd_vel desde wscat o curl' },
];

const challengeHints = [
  'Estructura mínima del JSON: {"robot": {"nombre": "...", "sensores": [{...}, {...}]}}',
  'En Python: with open("robot_config.json") as f: config = json.load(f)',
  'json.load() lee archivo directo — json.loads() parsea un string. Son funciones distintas',
  'Para iterar sensores activos: [s for s in config["robot"]["sensores"] if s["activo"]]',
  'Instalar rosbridge: sudo apt install ros-jazzy-rosbridge-suite',
  'wscat -c ws://localhost:9090 permite enviar JSON a rosbridge desde la terminal',
];

const summaryItems = [
  { cmd: '{"clave": valor}', desc: 'Objeto JSON — pares clave-valor',             example: 'Clave siempre entre comillas dobles', color: '#4ade80' },
  { cmd: 'true/false/null',  desc: 'Literales JSON — en minúscula',               example: 'NO True/False/None (eso es Python)',  color: '#fbbf24' },
  { cmd: '[1, 2, 3]',        desc: 'Array — lista ordenada de valores',           example: '"posicion": [x, y, z]',               color: '#60a5fa' },
  { cmd: 'json.loads(s)',    desc: 'String JSON → dict Python',                   example: 'para parsear msg.data en ROS 2',      color: '#c084fc' },
  { cmd: 'json.dumps(d)',    desc: 'Dict Python → string JSON',                   example: 'json.dumps(d, indent=2)',             color: '#f97316' },
  { cmd: 'json.load(f)',     desc: 'Leer JSON directamente desde un archivo',     example: 'with open("cfg.json") as f:',         color: '#f87171' },
  { cmd: 'jq . file.json',  desc: 'Validar y pretty-print en terminal',           example: 'sin output = JSON inválido',          color: '#22d3ee' },
  { cmd: '"op": "publish"', desc: 'rosbridge — publicar en un topic desde web',   example: '"topic": "/cmd_vel"',                 color: '#fbbf24' },
  { cmd: 'rosbridge',       desc: 'WebSocket que expone ROS 2 como JSON',         example: 'puerto 9090 por defecto',             color: '#4ade80' },
];

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════════════════

const robotConfigCode = [
  '{',
  '  "robot": {',
  '    "id": "robot_001",',
  '    "nombre": "TurtleBot Diferencial",',
  '    "tipo": "movil",',
  '    "velocidad_max": 2.5,',
  '    "sensores": [',
  '      {',
  '        "nombre": "lidar",',
  '        "modelo": "RPLidar A1",',
  '        "rango_max": 12.0,',
  '        "activo": true',
  '      },',
  '      {',
  '        "nombre": "camara",',
  '        "modelo": "Intel RealSense D435",',
  '        "resolucion": [1920, 1080],',
  '        "fps": 30,',
  '        "activo": true',
  '      }',
  '    ],',
  '    "motores": {',
  '      "izquierdo": { "vel_max": 3.0, "encoder_ticks": 1024 },',
  '      "derecho":   { "vel_max": 3.0, "encoder_ticks": 1024 }',
  '    },',
  '    "bateria": {',
  '      "voltaje_nominal": 12.0,',
  '      "capacidad_mah": 5200,',
  '      "ultimo_error": null',
  '    }',
  '  }',
  '}',
].join('\n');

const rosbridgeInstallCode = [
  '# Instalar rosbridge en ROS 2 Jazzy',
  'sudo apt install ros-jazzy-rosbridge-suite',
  '',
  '# Lanzar servidor WebSocket (puerto 9090)',
  'ros2 launch rosbridge_server rosbridge_websocket_launch.xml',
  '',
  '# Conectar desde JavaScript en el navegador:',
  "const ws = new WebSocket('ws://localhost:9090');",
  'ws.onopen = () => ws.send(JSON.stringify(msg));',
  '',
  '# Conectar desde terminal con wscat:',
  'sudo npm install -g wscat',
  'wscat -c ws://localhost:9090',
].join('\n');

const rosbridgePublishCode = [
  '{',
  '  "op": "publish",',
  '  "topic": "/cmd_vel",',
  '  "msg": {',
  '    "linear":  { "x": 0.5, "y": 0.0, "z": 0.0 },',
  '    "angular": { "x": 0.0, "y": 0.0, "z": 1.2 }',
  '  }',
  '}',
].join('\n');

const rosbridgeSubscribeCode = [
  '{',
  '  "op": "subscribe",',
  '  "topic": "/scan",',
  '  "type": "sensor_msgs/LaserScan",',
  '  "throttle_rate": 100',
  '}',
  '// throttle_rate: ms mínimos entre mensajes',
  '// 100 ms = 10 Hz (evita saturar WebSocket)',
].join('\n');

const rosbridgeServiceReqCode = [
  '// Petición (Web → rosbridge)',
  '{',
  '  "op": "call_service",',
  '  "service": "/mi_robot/get_battery",',
  '  "args": {}',
  '}',
].join('\n');

const rosbridgeServiceResCode = [
  '// Respuesta (rosbridge → Web)',
  '{',
  '  "op": "service_response",',
  '  "service": "/mi_robot/get_battery",',
  '  "values": {',
  '    "voltaje": 11.8,',
  '    "nivel_pct": 78',
  '  },',
  '  "result": true',
  '}',
].join('\n');

const pythonJsonCode = [
  'import json',
  '',
  '# Dict Python → string JSON',
  'robot = {',
  "    'nombre': 'TurtleBot',  # comillas simples OK en Python",
  "    'activo': True,          # booleano Python",
  "    'bateria': None",
  '}',
  'json_str = json.dumps(robot)',
  '# {"nombre": "TurtleBot", "activo": true, "bateria": null}',
  '',
  '# Con formato legible',
  'json_bonito = json.dumps(robot, indent=2, ensure_ascii=False)',
  '',
  '# String JSON → dict Python',
  "json_data = '{\"velocidad\": 2.5, \"activo\": true}'",
  'data = json.loads(json_data)',
  "print(data['velocidad'])  # 2.5  (float Python)",
  '',
  '# Leer/escribir archivos',
  "with open('config.json', 'r') as f:",
  '    config = json.load(f)          # leer',
  '',
  "with open('output.json', 'w') as f:",
  '    json.dump(config, f, indent=2)  # escribir',
].join('\n');

const rclpyNodeCode = [
  '#!/usr/bin/env python3',
  'import json, os',
  'import rclpy',
  'from rclpy.node import Node',
  'from std_msgs.msg import String',
  '',
  '',
  'class ConfigNode(Node):',
  '    def __init__(self):',
  "        super().__init__('config_node')",
  '',
  '        # Leer JSON al arrancar',
  "        cfg_path = os.path.join(os.path.dirname(__file__), 'robot_config.json')",
  "        with open(cfg_path, 'r') as f:",
  '            self.cfg = json.load(f)',
  '',
  "        nombre = self.cfg['robot']['nombre']",
  "        vel = self.cfg['robot']['velocidad_max']",
  "        self.get_logger().info(f'Robot: {nombre}, vel_max={vel} m/s')",
  '',
  '        # Suscribirse a comandos JSON (ej. desde rosbridge)',
  '        self.create_subscription(',
  '            String, "/robot/cmd_json", self.cmd_cb, 10',
  '        )',
  '',
  '    def cmd_cb(self, msg: String) -> None:',
  '        try:',
  '            cmd = json.loads(msg.data)',
  "            accion = cmd.get('accion', 'desconocida')",
  "            self.get_logger().info(f'Comando: {accion}')",
  '        except json.JSONDecodeError as e:',
  "            self.get_logger().error(f'JSON inválido: {e}')",
  '',
  '',
  'def main():',
  '    rclpy.init()',
  '    rclpy.spin(ConfigNode())',
  '    rclpy.shutdown()',
].join('\n');

const jqCode = [
  '# Instalar jq',
  'sudo apt install jq',
  '',
  '# Pretty-print (valida formato al mismo tiempo)',
  'jq . robot_config.json',
  '',
  '# Extraer campo específico',
  "jq '.robot.nombre' robot_config.json",
  '',
  '# Acceder a array por índice',
  "jq '.robot.sensores[0]' robot_config.json",
  '',
  '# Iterar array y extraer campo',
  "jq '.robot.sensores[].nombre' robot_config.json",
  '',
  '# Filtrar elementos (solo sensores activos)',
  "jq '.robot.sensores[] | select(.activo == true)' robot_config.json",
  '',
  '# Contar sensores',
  "jq '.robot.sensores | length' robot_config.json",
  '',
  '# Modificar un valor y guardar',
  "jq '.robot.velocidad_max = 3.0' robot_config.json > config_nuevo.json",
  '',
  '# Validar JSON de una API (ej. rosbridge output)',
  "curl -s http://localhost:8080/api/robot | jq .",
].join('\n');

const challengeCode = [
  '#!/usr/bin/env python3',
  '"""',
  'RETO: Completa este nodo para que lea robot_config.json',
  'y procese los sensores activos.',
  '"""',
  'import json',
  'import os',
  'import rclpy',
  'from rclpy.node import Node',
  '',
  '',
  'class RobotConfigNode(Node):',
  '    def __init__(self):',
  "        super().__init__('robot_config')",
  '',
  '        # TODO 1: Construir ruta al archivo',
  '        # cfg_path = os.path.join(..., "robot_config.json")',
  '',
  '        # TODO 2: Abrir y parsear el JSON',
  '        # with open(cfg_path) as f:',
  '        #     config = ...',
  '',
  '        # TODO 3: Extraer y loggear nombre y velocidad_max',
  '        # nombre = config[...][...]',
  '        # ...',
  '',
  '        # TODO 4: Iterar sensores activos y loggearlos',
  '        # for sensor in config["robot"]["sensores"]:',
  '        #     if sensor[...]:',
  '        #         ...',
  '',
  '',
  'def main():',
  '    rclpy.init()',
  '    node = RobotConfigNode()',
  '    rclpy.spin_once(node)',
  '    node.destroy_node()',
  '    rclpy.shutdown()',
  '',
  '',
  "if __name__ == '__main__':",
  '    main()',
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
   KV ANATOMY
══════════════════════════════════════════ */
.kv-anatomy {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 2rem 1.5rem;
}
.kva-row { display: flex; align-items: flex-end; gap: 2px; justify-content: center; flex-wrap: wrap; margin-bottom: 1.5rem; }
.kva-part { display: flex; flex-direction: column; align-items: center; gap: 4px; }
.kvap-token { font-family: 'Fira Code', monospace; font-size: 1rem; font-weight: 700; color: var(--kva-color); background: color-mix(in srgb, var(--kva-color) 10%, transparent); border-radius: 6px; padding: 4px 8px; white-space: nowrap; }
.kvap-arrow { font-size: .9rem; color: var(--kva-color); opacity: .7; }
.kvap-label { font-size: .68rem; font-weight: 700; color: var(--kva-color); text-align: center; }
.kva-rule { text-align: center; font-size: .82rem; color: var(--text-muted); border-top: 1px dashed var(--border-subtle); padding-top: 1rem; }

/* ══════════════════════════════════════════
   JSON TYPES
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
.tc-header { display: flex; align-items: center; gap: 9px; padding: 10px 14px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); flex-shrink: 0; }
.tc-type { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.tc-use { padding: 9px 14px; font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); margin-top: auto; }

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
.rc-num {
  width: 28px; height: 28px; border-radius: 50%;
  font-size: .85rem; font-weight: 800; color: #1e1e1e;
  display: flex; align-items: center; justify-content: center;
}
.rc-title { font-size: .85rem; font-weight: 700; color: var(--text-primary); }
.rc-bad   { display: flex; align-items: center; font-size: .74rem; color: var(--text-muted); }
.rc-good  { display: flex; align-items: center; font-size: .74rem; color: var(--text-secondary); }
.rc-bad code, .rc-good code { font-size: .72rem; background: none; padding: 0; }
.rc-note  { font-size: .72rem; color: var(--text-muted); font-style: italic; line-height: 1.3; }

/* Comparison table */
.compare-table { }
.ct-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.ct-body  { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.ct-row   { display: grid; grid-template-columns: 1.2fr 1fr 1fr; border-bottom: 1px solid var(--border-subtle); }
.ct-row:last-child { border-bottom: none; }
.ct-header { background: var(--bg-surface-solid); font-size: .82rem; font-weight: 700; color: var(--text-primary); }
.ct-cell  { padding: 9px 14px; border-right: 1px solid var(--border-subtle); font-size: .82rem; color: var(--text-secondary); display: flex; align-items: center; }
.ct-cell:last-child { border-right: none; }
.ct-feature { font-weight: 600; color: var(--text-primary); }
.ct-python-val { background: rgba(96,165,250,.04); }
.ct-json-val   { background: rgba(74,222,128,.04); }
.ct-python, .ct-json { color: var(--text-primary); }

/* ══════════════════════════════════════════
   ROSBRIDGE ARCH
══════════════════════════════════════════ */
.arch-visual {
  display: flex; align-items: center; justify-content: center;
  flex-wrap: wrap; gap: 0;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; padding: 1.5rem;
}
.arch-node-wrap { display: flex; align-items: center; }
.arch-node {
  background: color-mix(in srgb, var(--arch-color) 8%, var(--bg-surface));
  border: 1px solid color-mix(in srgb, var(--arch-color) 30%, transparent);
  border-radius: 12px; padding: 1rem 1.5rem;
  display: flex; flex-direction: column; align-items: center; gap: 5px; text-align: center;
}
.an-label { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.an-sub   { font-size: .72rem; color: var(--text-muted); }
.arch-arrow { display: flex; flex-direction: column; align-items: center; gap: 3px; padding: 0 8px; }
.aa-proto   { font-size: .7rem; color: var(--text-muted); white-space: nowrap; }

/* rosbridge panels */
.rb-panel { display: flex; flex-direction: column; gap: 8px; }
.rbp-header {
  display: flex; align-items: center; gap: 7px; font-size: .84rem;
  font-weight: 700; padding: 8px 12px; border-radius: 8px;
}
.rbp-pub { background: rgba(96,165,250,.1);  color: #60a5fa; }
.rbp-sub { background: rgba(74,222,128,.1);   color: #4ade80; }
.rbp-note { font-size: .8rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 7px; padding: 7px 11px; }

.service-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden; }
.sc-header { display: flex; align-items: center; gap: 8px; padding: 11px 16px; border-bottom: 1px solid var(--border-subtle); font-size: .88rem; font-weight: 600; color: var(--text-secondary); background: var(--bg-surface-solid); }

/* ══════════════════════════════════════════
   PYTHON PANELS
══════════════════════════════════════════ */
.py-panel { display: flex; flex-direction: column; gap: 8px; }
.pyp-header { display: flex; align-items: center; gap: 8px; font-size: .84rem; font-weight: 700; padding: 8px 12px; border-radius: 8px; background: var(--bg-surface-hover); color: var(--text-secondary); }

/* ══════════════════════════════════════════
   JQ FILTERS
══════════════════════════════════════════ */
.jq-filters { }
.jf-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.jf-grid  { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; }
.jf-card  {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--jf-color); border-radius: 10px;
  padding: 10px 12px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.jfc-filter { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; color: var(--jf-color); background: none; padding: 0; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.jfc-desc   { font-size: .78rem; color: var(--text-secondary); line-height: 1.3; }
.jfc-ex     { font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); background: none; padding: 0; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }

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
  .jf-grid    { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 900px) {
  .types-grid { grid-template-columns: repeat(2, 1fr); }
  .rules-grid { grid-template-columns: repeat(2, 1fr); }
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
  .err-pair   { grid-template-columns: 1fr; }
}
@media (max-width: 768px) {
  .rules-grid      { grid-template-columns: 1fr; }
  .ct-row          { grid-template-columns: 1fr; }
  .ct-cell         { border-right: none; border-bottom: 1px solid var(--border-subtle); }
  .arch-visual     { flex-direction: column; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
}
@media (max-width: 480px) {
  .types-grid   { grid-template-columns: 1fr; }
  .jf-grid      { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: 1fr; }
}
</style>
