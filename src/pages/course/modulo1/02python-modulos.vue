<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        En robótica el código crece rápido. Un nodo con 2000 líneas es imposible de mantener.
        Los <strong>módulos y paquetes</strong> son la forma de dividir tu sistema en piezas
        reutilizables. Esta lección cubre el sistema de imports de Python, la estructura
        oficial de un paquete ROS 2, y — lo más importante — <strong>cómo conecta
        <code>ros2 run</code> con tu función <code>main()</code></strong>.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 IMPORT FLOW
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        ¿Qué Pasa Internamente con import?
      </SectionTitle>

      <TextBlock>
        Cada vez que escribes <code>import algo</code>, Python ejecuta una búsqueda de 4 pasos.
        Entender este proceso te evitará horas de depuración con <code>ModuleNotFoundError</code>.
      </TextBlock>

      <div class="import-flow q-mt-lg">
        <div v-for="(step, i) in importSteps" :key="i" class="if-step-wrap">
          <div class="if-step" :style="{ '--if-color': step.color }">
            <div class="if-num-wrap">
              <div class="if-num" :style="{ background: step.color }">{{ i + 1 }}</div>
            </div>
            <div class="if-icon-wrap" :style="{ background: step.color + '18' }">
              <q-icon :name="step.icon" size="22px" :style="{ color: step.color }" />
            </div>
            <div class="if-body">
              <div class="if-title">{{ step.title }}</div>
              <div class="if-desc">{{ step.desc }}</div>
            </div>
          </div>
          <div v-if="i < importSteps.length - 1" class="if-connector"></div>
        </div>
      </div>

      <AlertBlock type="warning" title="Import solo ejecuta una vez" class="q-mt-lg">
        Si importas el mismo módulo en 10 archivos distintos, Python solo ejecuta su código
        <strong>una vez</strong>. Después reutiliza la versión en caché
        (<code>sys.modules</code>). Esto es crucial para singletons y estado compartido.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         02 TRES ESTILOS DE IMPORT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Tres Estilos de Import
      </SectionTitle>

      <div class="import-styles-grid q-mt-lg">
        <div v-for="style in importStyles" :key="style.title" class="is-card"
          :style="{ '--isc-color': style.color }">
          <div class="isc-header">
            <q-icon :name="style.icon" size="18px" :style="{ color: style.color }" />
            <span class="isc-status" :style="{ color: style.color }">{{ style.status }}</span>
          </div>
          <div class="isc-title">{{ style.title }}</div>
          <CodeBlock :hide-header="true" lang="python" :content="style.code" />
          <div class="isc-notes">
            <div v-if="style.pros" class="isc-note isc-pro">
              <q-icon name="thumb_up" size="11px" color="positive" />
              {{ style.pros }}
            </div>
            <div v-if="style.cons" class="isc-note isc-con">
              <q-icon name="thumb_down" size="11px" color="negative" />
              {{ style.cons }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 PEP 8 — ORDEN DE IMPORTS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Orden de Imports — PEP 8
      </SectionTitle>

      <TextBlock>
        PEP 8 (la guía de estilo oficial de Python) establece que los imports deben ir
        agrupados en tres zonas separadas por una línea en blanco.
        Seguirlo permite entender de un vistazo las dependencias de cada módulo.
      </TextBlock>

      <div class="pep8-demo q-mt-lg">
        <div class="pd-chrome">
          <span class="pd-filename">control_node.py — orden correcto</span>
        </div>
        <div class="pd-body">
          <div v-for="zone in importZones" :key="zone.label" class="pd-zone"
            :style="{ '--pdz-color': zone.color }">
            <div class="pdz-label" :style="{ color: zone.color }">
              <q-icon name="label" size="11px" />
              {{ zone.label }}
            </div>
            <div class="pdz-lines">
              <div v-for="line in zone.lines" :key="line" class="pdz-line">
                <span v-if="line" class="pdz-code">{{ line }}</span>
                <span v-else class="pdz-empty"></span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="Herramienta: isort" class="q-mt-md">
        <code>isort</code> ordena y agrupa imports automáticamente:
        <code>sudo apt install python3-isort</code> → <code>isort control_node.py</code>.
        El VS Code con la extensión Python lo hace en cada guardado.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         04 __INIT__.PY
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        El Archivo Mágico: __init__.py
      </SectionTitle>

      <TextBlock>
        Python distingue una carpeta normal de un <strong>paquete importable</strong>
        por la presencia de un archivo llamado <code>__init__.py</code>.
        Sin él, los imports fallan aunque el archivo exista.
      </TextBlock>

      <!-- Before / After -->
      <div class="init-comparison q-mt-lg">
        <div class="ic-panel ic-bad">
          <div class="icp-header">
            <q-icon name="cancel" size="16px" color="negative" />
            Sin __init__.py
          </div>
          <div class="icp-tree">
            <div class="icp-item">
              <q-icon name="folder" size="16px" style="color:var(--text-muted)" />
              mi_robot/
            </div>
            <div class="icp-item icp-indent">
              <q-icon name="description" size="14px" style="color:var(--text-muted)" />
              motor.py
            </div>
            <div class="icp-item icp-indent">
              <q-icon name="description" size="14px" style="color:var(--text-muted)" />
              sensor.py
            </div>
          </div>
          <div class="icp-result icp-result-bad">
            <code>from mi_robot import motor</code>
            <span class="icp-error">ModuleNotFoundError</span>
          </div>
        </div>

        <div class="ic-arrow">
          <q-icon name="add" size="20px" style="color:var(--text-muted)" />
          <code style="color:var(--text-code)">__init__.py</code>
          <q-icon name="east" size="24px" color="positive" />
        </div>

        <div class="ic-panel ic-good">
          <div class="icp-header">
            <q-icon name="check_circle" size="16px" color="positive" />
            Con __init__.py
          </div>
          <div class="icp-tree">
            <div class="icp-item">
              <q-icon name="folder" size="16px" color="primary" />
              mi_robot/
            </div>
            <div class="icp-item icp-indent icp-highlight">
              <q-icon name="star" size="14px" color="warning" />
              __init__.py
            </div>
            <div class="icp-item icp-indent">
              <q-icon name="description" size="14px" color="positive" />
              motor.py
            </div>
            <div class="icp-item icp-indent">
              <q-icon name="description" size="14px" color="positive" />
              sensor.py
            </div>
          </div>
          <div class="icp-result icp-result-good">
            <code>from mi_robot import motor</code>
            <span class="icp-ok">✓ funciona</span>
          </div>
        </div>
      </div>

      <!-- __init__.py options -->
      <div class="init-options q-mt-xl">
        <div class="io-title">¿Qué poner en __init__.py?</div>
        <div class="io-grid">
          <div v-for="opt in initOptions" :key="opt.title" class="io-card"
            :style="{ '--io-color': opt.color }">
            <div class="io-header">
              <q-icon :name="opt.icon" size="18px" :style="{ color: opt.color }" />
              <div>
                <div class="io-level" :style="{ color: opt.color }">{{ opt.level }}</div>
                <div class="io-title-text">{{ opt.title }}</div>
              </div>
            </div>
            <CodeBlock :hide-header="true" lang="python" :content="opt.code" />
            <div class="io-note">{{ opt.note }}</div>
          </div>
        </div>
      </div>

      <!-- __all__ -->
      <div class="all-section q-mt-xl">
        <div class="all-title">
          <q-icon name="filter_list" size="18px" color="primary" />
          <code>__all__</code> — Control de API pública
        </div>
        <TextBlock>
          Cuando alguien hace <code>from mi_robot import *</code>,
          Python exporta solo los nombres definidos en <code>__all__</code>.
          También sirve de documentación: dice exactamente qué es la API pública del módulo.
        </TextBlock>
        <CodeBlock title="__init__.py con control de exports" lang="python"
          :content="allExportCode" :copyable="true" />
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 ESTRUCTURA ROS 2 — ÁRBOL COMPLETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Estructura Oficial de un Paquete ROS 2 Python
      </SectionTitle>

      <TextBlock>
        Esta es la estructura exacta que espera ROS 2. Cada archivo tiene un rol específico.
        Puedes generarla automáticamente con un comando.
      </TextBlock>

      <!-- ros2 pkg create command -->
      <div class="create-cmd q-mt-lg">
        <div class="cc-header">
          <q-icon name="auto_fix_high" size="16px" color="positive" />
          Generar automáticamente con ros2 pkg create
        </div>
        <CodeBlock :hide-header="true" lang="bash" :content="pkgCreateCode" :copyable="true" />
      </div>

      <!-- File tree -->
      <div class="file-tree q-mt-xl">
        <div class="ft-header">
          <q-icon name="account_tree" size="16px" color="primary" />
          Estructura generada
        </div>
        <div class="ft-body">
          <div v-for="(item, i) in fileTree" :key="i" class="ft-item"
            :class="{ 'ft-folder': item.type === 'folder', 'ft-critical': item.critical, 'ft-important': item.important }"
            :style="{ '--ft-color': item.color, paddingLeft: (item.level * 24 + 14) + 'px' }">
            <div class="ft-item-main">
              <q-icon :name="item.type === 'folder' ? 'folder' : 'description'"
                size="16px" :style="{ color: item.color }" />
              <span class="ft-name" :style="{ color: item.type === 'folder' ? item.color : 'var(--text-primary)' }">
                {{ item.name }}
              </span>
              <span v-if="item.critical" class="ft-badge-critical">CRÍTICO</span>
              <span v-if="item.important" class="ft-badge-important">IMPORTANTE</span>
            </div>
            <div class="ft-desc">{{ item.desc }}</div>
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="El nombre duplicado no es un error" class="q-mt-lg">
        La carpeta raíz <code>mi_robot_pkg/</code> y la carpeta de código fuente
        <code>mi_robot_pkg/mi_robot_pkg/</code> tienen el mismo nombre a propósito.
        ROS 2 requiere que el nombre del paquete Python coincida con el nombre del paquete ROS 2.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         06 SETUP.PY / SETUP.CFG + ENTRY POINTS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">06</span>
        setup.py y Entry Points — El Puente con ros2 run
      </SectionTitle>

      <TextBlock>
        Este es el concepto más importante de esta lección y el que más confunde.
        Los <strong>entry points</strong> son la configuración que le dice a ROS 2:
        "cuando alguien ejecute <code>ros2 run mi_pkg mi_nodo</code>,
        llama a la función <code>main()</code> de este archivo".
      </TextBlock>

      <!-- ros2 run → entry_points flow -->
      <div class="ep-flow q-mt-lg">
        <div v-for="(step, i) in epFlow" :key="i" class="ep-step-wrap">
          <div class="ep-step" :style="{ '--ep-color': step.color }">
            <div class="eps-icon" :style="{ background: step.color + '15' }">
              <q-icon :name="step.icon" size="20px" :style="{ color: step.color }" />
            </div>
            <div class="eps-body">
              <div class="eps-label" :style="{ color: step.color }">{{ step.label }}</div>
              <code class="eps-code">{{ step.code }}</code>
              <div class="eps-desc">{{ step.desc }}</div>
            </div>
          </div>
          <div v-if="i < epFlow.length - 1" class="ep-connector">
            <q-icon name="south" size="16px" style="color:var(--text-muted)" />
          </div>
        </div>
      </div>

      <!-- setup.py vs setup.cfg -->
      <div class="setup-comparison q-mt-xl">
        <div class="sc-title">
          <q-icon name="compare" size="16px" color="primary" />
          setup.py vs setup.cfg — Los dos archivos de configuración
        </div>
        <div class="sc-panels">
          <div class="sc-panel">
            <div class="scp-header scp-py">
              <q-icon name="code" size="16px" />
              setup.py — Dinámico (Python)
            </div>
            <CodeBlock :hide-header="true" lang="python" :content="setupPyCode" :copyable="true" />
            <div class="scp-note">Permite lógica en Python. Más flexible.</div>
          </div>
          <div class="sc-panel">
            <div class="scp-header scp-cfg">
              <q-icon name="settings" size="16px" />
              setup.cfg — Declarativo (INI) ⭐ Moderno
            </div>
            <CodeBlock :hide-header="true" lang="ini" :content="setupCfgCode" :copyable="true" />
            <div class="scp-note">
              <strong>Preferido en ROS 2 Jazzy.</strong> Más legible, menos código.
            </div>
          </div>
        </div>
      </div>

      <!-- Colcon workflow -->
      <div class="colcon-workflow q-mt-xl">
        <div class="cw-title">
          <q-icon name="loop" size="18px" color="primary" />
          El Ciclo Completo: Editar → Compilar → Ejecutar
        </div>
        <div class="cw-steps">
          <div v-for="(step, i) in colconSteps" :key="i" class="cw-step"
            :style="{ '--cw-color': step.color }">
            <div class="cws-num" :style="{ background: step.color }">{{ i + 1 }}</div>
            <div class="cws-body">
              <div class="cws-title">{{ step.title }}</div>
              <CodeBlock :hide-header="true" lang="bash" :content="step.cmd" :copyable="true" />
              <div class="cws-note">{{ step.note }}</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         07 IMPORTS ABSOLUTOS VS RELATIVOS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">07</span>
        Absolutos vs Relativos — ¿Cuál Usar?
      </SectionTitle>

      <TextBlock>
        Python soporta dos formas de referenciar módulos dentro de un paquete.
        En ROS 2 hay una regla clara al respecto.
      </TextBlock>

      <SplitBlock class="q-mt-lg">
        <template #left>
          <div class="ir-panel ir-absolute">
            <div class="irp-header">
              <q-icon name="public" size="16px" color="primary" />
              Import Absoluto
            </div>
            <CodeBlock :hide-header="true" lang="python" :content="absoluteCode" :copyable="true" />
            <div class="irp-pros">
              <div class="irp-pro"><q-icon name="check" size="12px" color="positive" />Claro y explícito</div>
              <div class="irp-pro"><q-icon name="check" size="12px" color="positive" />Funciona desde cualquier contexto</div>
              <div class="irp-pro"><q-icon name="check" size="12px" color="positive" />Compatible con entry points</div>
            </div>
            <div class="irp-badge irp-badge-ok">✅ Usar en ROS 2</div>
          </div>
        </template>
        <template #right>
          <div class="ir-panel ir-relative">
            <div class="irp-header">
              <q-icon name="share" size="16px" style="color:#c084fc" />
              Import Relativo
            </div>
            <CodeBlock :hide-header="true" lang="python" :content="relativeCode" :copyable="true" />
            <div class="irp-pros">
              <div class="irp-pro"><q-icon name="check" size="12px" color="positive" />Más corto</div>
              <div class="irp-con"><q-icon name="close" size="12px" color="negative" />Falla al ejecutar directamente</div>
              <div class="irp-con"><q-icon name="close" size="12px" color="negative" />Problemas con entry points</div>
            </div>
            <div class="irp-badge irp-badge-warn">⚠️ Evitar en ROS 2</div>
          </div>
        </template>
      </SplitBlock>

      <AlertBlock type="warning" title="La regla de oro en ROS 2" class="q-mt-lg">
        Usa siempre <strong>imports absolutos</strong> en paquetes ROS 2.
        Los imports relativos solo funcionan cuando el módulo es importado (no ejecutado directamente),
        y los entry points en <code>setup.py</code> pueden fallar de formas inesperadas con imports relativos.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num">{{ i + 1 }}</div>
              <div>
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
         RETO PRÁCTICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Crea un Paquete ROS 2 Desde Cero</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Mi primer paquete ROS 2 Python</div>
            <div class="challenge-subtitle">
              Estructura, imports, entry points y colcon build
            </div>
          </div>
          <div class="challenge-badge">30 min</div>
        </div>

        <div class="challenge-reqs q-mt-md">
          <div class="cr-title">Pasos del reto:</div>
          <div class="cr-list">
            <div v-for="req in challengeSteps" :key="req.step" class="cr-step">
              <div class="cr-step-num" :style="{ background: req.color }">{{ req.step }}</div>
              <div class="cr-step-text">{{ req.text }}</div>
            </div>
          </div>
        </div>

        <CodeBlock title="Comandos de referencia" lang="bash"
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
      <TextBlock>Módulos, paquetes y estructura de proyectos ROS 2:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Módulos Python ROS 2" frameborder="0"
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
      <SectionTitle>Resumen — Conceptos y Comandos Clave</SectionTitle>
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
  { icon: '📄', label: 'Módulo = un archivo .py reutilizable' },
  { icon: '📦', label: 'Paquete = carpeta con __init__.py' },
  { icon: '🔌', label: 'Entry points = el puente entre ros2 run y main()' },
];

// ── Import flow steps ──────────────────────────────────────────
const importSteps = [
  { title: 'Escribes import mi_robot', desc: 'Python recibe la petición de importación', icon: 'edit', color: '#60a5fa' },
  { title: 'Busca en sys.path', desc: 'Recorre los directorios en orden hasta encontrar mi_robot.py o mi_robot/__init__.py', icon: 'search', color: '#c084fc' },
  { title: 'Ejecuta el módulo (una vez)', desc: 'Lee y ejecuta todo el código del archivo. El resultado queda en sys.modules como caché', icon: 'play_circle', color: '#fbbf24' },
  { title: 'Crea el namespace', desc: 'Ahora puedes acceder a mi_robot.funcion(), mi_robot.CONSTANTE, etc.', icon: 'check_circle', color: '#4ade80' },
];

// ── Import styles ──────────────────────────────────────────────
const importStyles = [
  {
    status: '✅ Recomendado — siempre', icon: 'check_circle', color: '#4ade80',
    title: 'Namespace completo',
    code: [
      'import numpy as np',
      'import rclpy',
      '',
      '# Uso: el origen es explícito',
      'array = np.array([1.0, 2.0, 3.0])',
      'rclpy.init()',
    ].join('\n'),
    pros: 'Legible, sin conflictos de nombres, estándar en proyectos ROS 2',
    cons: null,
  },
  {
    status: '⚠️ Con cuidado', icon: 'warning', color: '#fbbf24',
    title: 'Import específico',
    code: [
      'from math import sqrt, pi',
      'from rclpy.node import Node',
      '',
      '# Uso directo — pero puede haber conflictos',
      'raiz = sqrt(16)',
      'nodo = Node("mi_nodo")',
    ].join('\n'),
    pros: 'Código más corto',
    cons: 'Riesgo de conflicto si ya tienes una función con el mismo nombre',
  },
  {
    status: '❌ NUNCA — prohibido', icon: 'cancel', color: '#f87171',
    title: 'Wildcard import',
    code: [
      'from rclpy.node import *',
      'from std_msgs.msg import *',
      '',
      '# ¿De dónde viene create_publisher?',
      '# ¿Qué más importé sin saberlo?',
      '# Debugging imposible.',
    ].join('\n'),
    pros: null,
    cons: 'Cientos de nombres invisibles, conflictos impredecibles, debugging imposible',
  },
];

// ── PEP 8 import zones ─────────────────────────────────────────
const importZones = [
  {
    label: 'Zona 1 — Librería estándar de Python',
    color: '#4ade80',
    lines: ['import os', 'import sys', 'import argparse', ''],
  },
  {
    label: 'Zona 2 — Terceros (instalados vía apt/pip)',
    color: '#60a5fa',
    lines: ['import numpy as np', 'import rclpy', 'from rclpy.node import Node', 'from std_msgs.msg import String', ''],
  },
  {
    label: 'Zona 3 — Locales (tu propio código)',
    color: '#c084fc',
    lines: ['from mi_robot_pkg.utils import calcular_distancia', 'from mi_robot_pkg.config import ROBOT_NAME'],
  },
];

// ── __init__.py options ────────────────────────────────────────
const initOptions = [
  {
    level: 'Opción 1', title: 'Vacío — Solo marca el paquete', icon: 'circle', color: '#94a3b8',
    code: '# __init__.py\n# Vacío — solo su existencia es suficiente',
    note: 'La opción más simple. ROS 2 solo necesita que el archivo exista.',
  },
  {
    level: 'Opción 2', title: 'Re-exportar clases clave', icon: 'output', color: '#60a5fa',
    code: [
      '# __init__.py',
      'from mi_robot_pkg.motor import MotorController',
      'from mi_robot_pkg.sensor import LidarSensor',
      '',
      '__all__ = ["MotorController", "LidarSensor"]',
    ].join('\n'),
    note: 'Permite: from mi_robot_pkg import MotorController directamente.',
  },
  {
    level: 'Opción 3', title: 'Inicialización del paquete', icon: 'settings', color: '#c084fc',
    code: [
      '# __init__.py',
      'import logging',
      '',
      '# Configura logging al importar el paquete',
      'logging.getLogger(__name__).addHandler(',
      '    logging.NullHandler()',
      ')',
    ].join('\n'),
    note: 'Para paquetes de librería que no deben imprimir logs por defecto.',
  },
];

// ── File tree ──────────────────────────────────────────────────
const fileTree = [
  { type: 'folder', name: 'mi_robot_pkg/', level: 0, color: '#60a5fa', desc: 'Directorio raíz — nombre del paquete ROS 2' },
  { type: 'file',   name: 'package.xml',  level: 1, color: '#f97316', desc: 'Metadatos: nombre, versión, dependencias ROS 2', important: true },
  { type: 'file',   name: 'setup.py',     level: 1, color: '#f97316', desc: 'Entry points: conecta ros2 run con tu main()', important: true },
  { type: 'file',   name: 'setup.cfg',    level: 1, color: '#fbbf24', desc: 'Configuración del paquete Python (nombre, licencia)' },
  { type: 'file',   name: 'resource/mi_robot_pkg', level: 1, color: '#94a3b8', desc: 'Archivo de registro requerido por ament' },
  { type: 'folder', name: 'mi_robot_pkg/', level: 1, color: '#4ade80', desc: 'Código fuente Python — MISMO NOMBRE que raíz' },
  { type: 'file',   name: '__init__.py',   level: 2, color: '#fbbf24', desc: 'Marca la carpeta como paquete importable', critical: true },
  { type: 'file',   name: 'control_node.py', level: 2, color: '#4ade80', desc: 'Nodo de control — contiene def main()' },
  { type: 'file',   name: 'vision_node.py',  level: 2, color: '#4ade80', desc: 'Nodo de visión/cámara — contiene def main()' },
  { type: 'file',   name: 'utils.py',        level: 2, color: '#22d3ee', desc: 'Funciones de utilidad compartidas entre nodos' },
  { type: 'folder', name: 'launch/', level: 1, color: '#c084fc', desc: 'Launch files — inicia múltiples nodos a la vez' },
  { type: 'file',   name: 'robot.launch.py', level: 2, color: '#c084fc', desc: 'Lanza control + vision + sensores juntos' },
  { type: 'folder', name: 'config/', level: 1, color: '#22d3ee', desc: 'Archivos YAML de configuración' },
  { type: 'folder', name: 'test/',   level: 1, color: '#94a3b8', desc: 'Tests unitarios e integración' },
];

// ── Entry points flow ──────────────────────────────────────────
const epFlow = [
  {
    label: 'Paso 1 — El usuario ejecuta', icon: 'terminal', color: '#60a5fa',
    code: 'ros2 run mi_robot_pkg control_node',
    desc: 'ROS 2 busca el entry point "control_node" en el paquete "mi_robot_pkg"',
  },
  {
    label: 'Paso 2 — ROS 2 consulta setup.py', icon: 'settings', color: '#fbbf24',
    code: "'control_node = mi_robot_pkg.control_node:main'",
    desc: 'El entry point define: módulo Python = mi_robot_pkg.control_node, función = main',
  },
  {
    label: 'Paso 3 — Python llama main()', icon: 'play_circle', color: '#4ade80',
    code: 'mi_robot_pkg/control_node.py → def main() → rclpy.spin()',
    desc: 'La función main() se ejecuta — el nodo ROS 2 arranca',
  },
];

// ── Colcon steps ───────────────────────────────────────────────
const colconSteps = [
  {
    title: 'Compilar el paquete', color: '#f97316',
    cmd: 'cd ~/ros2_ws\ncolcon build --packages-select mi_robot_pkg\n# Solo recompila mi_robot_pkg, no todo el workspace',
    note: 'Crea la carpeta install/ con el paquete listo para usar',
  },
  {
    title: 'Activar el workspace (¡obligatorio!)', color: '#fbbf24',
    cmd: 'source install/setup.bash\n# Agrega mi_robot_pkg a PYTHONPATH y PATH',
    note: 'Sin este source, ros2 run no encuentra tu paquete. Hazlo en CADA nueva terminal.',
  },
  {
    title: 'Ejecutar el nodo', color: '#4ade80',
    cmd: 'ros2 run mi_robot_pkg control_node\n# O con argumentos:\nros2 run mi_robot_pkg control_node --robot Atlas --rate 5.0',
    note: 'ROS 2 usa el entry point para encontrar y ejecutar tu main()',
  },
];

// ── Summary items ──────────────────────────────────────────────
const summaryItems = [
  { cmd: '__init__.py', desc: 'Marca carpeta como paquete Python importable', example: 'touch mi_pkg/__init__.py', color: '#fbbf24' },
  { cmd: 'entry_points', desc: 'Conecta ros2 run con tu función main()', example: "'nodo = mi_pkg.nodo:main'", color: '#f97316' },
  { cmd: 'import absoluto', desc: 'Forma segura de importar en paquetes ROS 2', example: 'from mi_pkg.utils import fn', color: '#4ade80' },
  { cmd: 'colcon build', desc: 'Compila el workspace ROS 2', example: 'colcon build --symlink-install', color: '#60a5fa' },
  { cmd: 'source setup.bash', desc: 'Activa el workspace compilado', example: 'source install/setup.bash', color: '#c084fc' },
  { cmd: 'sys.path', desc: 'Lista de directorios donde Python busca módulos', example: 'python3 -c "import sys; print(sys.path)"', color: '#22d3ee' },
];

// ── Challenge ──────────────────────────────────────────────────
const challengeSteps = [
  { step: 1, color: '#60a5fa', text: 'Crea el paquete con: ros2 pkg create --build-type ament_python mi_robot_pkg' },
  { step: 2, color: '#4ade80', text: 'Crea utils.py con calcular_distancia(x1, y1, x2, y2) con type hints → float' },
  { step: 3, color: '#fbbf24', text: 'Crea control_node.py que importa calcular_distancia con import absoluto' },
  { step: 4, color: '#c084fc', text: 'Agrega control_node al entry_points de setup.py' },
  { step: 5, color: '#f87171', text: 'Compila con colcon build, source install/setup.bash, y lanza con ros2 run' },
];

const challengeHints = [
  'La función en utils.py: def calcular_distancia(x1: float, y1: float, x2: float, y2: float) -> float: return ((x2-x1)**2 + (y2-y1)**2)**0.5',
  'En el entry_point: "control_node = mi_robot_pkg.control_node:main"',
  'Si colcon falla con "package not found", verifica que package.xml exista y tenga el nombre correcto',
  'Después de colcon build, el source install/setup.bash es OBLIGATORIO en cada terminal nueva',
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    msg: "ModuleNotFoundError: No module named 'mi_paquete'",
    summary: 'Python no encuentra el paquete — típicamente falta source o colcon build',
    cause: 'El paquete no está en sys.path. Puede que no hayas compilado, o no hayas hecho source del workspace.',
    steps: [
      'Compila: cd ~/ros2_ws && colcon build --packages-select mi_paquete',
      'Activa: source install/setup.bash (en CADA terminal nueva)',
      'Verifica que existe __init__.py en la carpeta del paquete',
    ],
    code: 'cd ~/ros2_ws\ncolcon build --packages-select mi_paquete\nsource install/setup.bash',
    open: false,
  },
  {
    msg: "ImportError: attempted relative import with no known parent package",
    summary: 'Import relativo usado al ejecutar el archivo directamente',
    cause: 'Los imports relativos (from .utils import fn) solo funcionan cuando el módulo es importado como parte de un paquete, no cuando se ejecuta directamente con python3.',
    steps: [
      'Cambia el import relativo por absoluto: from mi_paquete.utils import fn',
      'O ejecuta como módulo: python3 -m mi_paquete.nodo (no python3 mi_paquete/nodo.py)',
      'En ROS 2: usa siempre imports absolutos para evitar este problema',
    ],
    code: '# ❌ Problema\nfrom .utils import fn\n\n# ✅ Solución\nfrom mi_paquete.utils import fn',
    open: false,
  },
  {
    msg: "AttributeError: module 'mi_modulo' has no attribute 'mi_funcion'",
    summary: 'El módulo existe pero no tiene la función/clase que buscas',
    cause: 'La función no existe, tiene otro nombre, o hay un error de tipeo.',
    steps: [
      'Inspecciona el módulo: python3 -c "import mi_modulo; print(dir(mi_modulo))"',
      'Verifica que guardaste el archivo correctamente',
      'Verifica que no hay typos en el nombre de la función',
    ],
    code: 'python3 -c "import mi_modulo; print(dir(mi_modulo))"',
    open: false,
  },
  {
    msg: 'No executable found matching command "mi_nodo"',
    summary: 'El entry point no está registrado o el paquete no está compilado',
    cause: 'ROS 2 no puede encontrar el entry_point "mi_nodo" en el paquete. Falta colcon build después de editar setup.py.',
    steps: [
      'Verifica el entry_point en setup.py: "mi_nodo = mi_pkg.mi_nodo:main"',
      'Recompila SIEMPRE que modifiques setup.py: colcon build --packages-select mi_pkg',
      'Haz source install/setup.bash en la terminal donde ejecutas ros2 run',
    ],
    code: 'cat setup.py  # Verifica entry_points\ncolcon build --packages-select mi_pkg\nsource install/setup.bash',
    open: false,
  },
  {
    msg: 'colcon build — CMake Error: Could not find package.xml',
    summary: 'La estructura del paquete es incorrecta',
    cause: 'colcon busca package.xml en la carpeta. Si no lo encuentra, el paquete no es válido.',
    steps: [
      'Verifica que estás en ~/ros2_ws/src/ y que package.xml existe en la raíz del paquete',
      'El comando correcto: cd ~/ros2_ws && colcon build (no desde dentro del paquete)',
      'Si creaste la carpeta manualmente, usa ros2 pkg create para generar la estructura correcta',
    ],
    code: 'cd ~/ros2_ws\nls src/mi_paquete/  # Debe tener package.xml, setup.py, setup.cfg',
    open: false,
  },
]);

// ═══════════════════════════════════════════════════════════════
// CODE STRING CONSTANTS
// ═══════════════════════════════════════════════════════════════

const allExportCode = [
  '# mi_robot_pkg/__init__.py',
  'from mi_robot_pkg.motor import MotorController',
  'from mi_robot_pkg.sensor import LidarSensor',
  'from mi_robot_pkg.utils import calcular_distancia',
  '',
  '# Define la API pública del paquete',
  '__all__ = [',
  '    "MotorController",',
  '    "LidarSensor",',
  '    "calcular_distancia",',
  ']',
  '',
  '# Ahora funciona:',
  '# from mi_robot_pkg import MotorController  ✅',
  '# from mi_robot_pkg import SecretoInterno   ❌ (no está en __all__)',
].join('\n');

const pkgCreateCode = [
  '# Ir a la carpeta src del workspace',
  'cd ~/ros2_ws/src',
  '',
  '# Crear paquete Python con un nodo inicial',
  'ros2 pkg create \\',
  '  --build-type ament_python \\',
  '  --node-name control_node \\',
  '  mi_robot_pkg',
  '',
  '# Genera la estructura completa automáticamente',
].join('\n');

const setupPyCode = [
  'from setuptools import setup',
  '',
  "package_name = 'mi_robot_pkg'",
  '',
  'setup(',
  '    name=package_name,',
  "    version='0.1.0',",
  '    packages=[package_name],',
  "    install_requires=['setuptools'],",
  '    entry_points={',
  "        'console_scripts': [",
  "            'control_node = mi_robot_pkg.control_node:main',",
  "            'vision_node  = mi_robot_pkg.vision_node:main',",
  '        ],',
  '    },',
  ')',
].join('\n');

const setupCfgCode = [
  '[metadata]',
  'name = mi_robot_pkg',
  'version = 0.1.0',
  'description = Mi paquete ROS 2',
  'maintainer = Tu Nombre',
  'license = Apache-2.0',
  '',
  '[options]',
  'packages =',
  '    mi_robot_pkg',
  'install_requires =',
  '    setuptools',
  '',
  '[options.entry_points]',
  'console_scripts =',
  '    control_node = mi_robot_pkg.control_node:main',
  '    vision_node  = mi_robot_pkg.vision_node:main',
].join('\n');

const absoluteCode = [
  '# En mi_robot_pkg/vision_node.py',
  '',
  '# Ruta completa desde la raíz del paquete',
  'from mi_robot_pkg.utils import calcular_distancia',
  'from mi_robot_pkg.config import VELOCIDAD_MAX',
  '',
  '# ✅ Funciona desde cualquier contexto:',
  '# - python3 vision_node.py',
  '# - ros2 run mi_robot_pkg vision_node',
  '# - import mi_robot_pkg.vision_node',
].join('\n');

const relativeCode = [
  '# En mi_robot_pkg/vision_node.py',
  '',
  '# El punto (.) = "mismo paquete"',
  'from .utils import calcular_distancia',
  'from .config import VELOCIDAD_MAX',
  '',
  '# ❌ Falla al ejecutar directamente:',
  '# python3 vision_node.py → ImportError',
  '# ✅ Funciona solo al importar:',
  '# import mi_robot_pkg.vision_node',
].join('\n');

const challengeCode = [
  '# 1. Crear el paquete',
  'cd ~/ros2_ws/src',
  'ros2 pkg create --build-type ament_python mi_robot_pkg',
  '',
  '# 2. Crear utils.py',
  'touch mi_robot_pkg/mi_robot_pkg/utils.py',
  '',
  '# 3. Compilar y lanzar',
  'cd ~/ros2_ws',
  'colcon build --packages-select mi_robot_pkg',
  'source install/setup.bash',
  'ros2 run mi_robot_pkg control_node',
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
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
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
   IMPORT FLOW
══════════════════════════════════════════ */
.import-flow { display: flex; flex-direction: column; }
.if-step-wrap { display: flex; flex-direction: column; }
.if-connector {
  width: 2px; height: 16px; background: var(--border-medium); margin-left: 20px;
}
.if-step {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--if-color, var(--border-medium));
  border-radius: 12px; padding: 1rem 1.25rem;
  display: flex; align-items: center; gap: 14px; transition: all .2s;
}
.if-step:hover { box-shadow: var(--shadow-sm); transform: translateX(4px); }
.if-num-wrap { flex-shrink: 0; }
.if-num {
  width: 34px; height: 34px; border-radius: 50%;
  display: flex; align-items: center; justify-content: center;
  font-size: .95rem; font-weight: 800; color: #1e1e1e;
}
.if-icon-wrap {
  width: 42px; height: 42px; border-radius: 10px; flex-shrink: 0;
  display: flex; align-items: center; justify-content: center;
}
.if-body { min-width: 0; }
.if-title { font-size: .95rem; font-weight: 700; color: var(--text-primary); margin-bottom: 3px; }
.if-desc  { font-size: .84rem; color: var(--text-secondary); line-height: 1.4; }

/* ══════════════════════════════════════════
   IMPORT STYLES
══════════════════════════════════════════ */
.import-styles-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 16px; }
.is-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--isc-color, var(--border-medium));
  border-radius: 14px; padding: 1.25rem;
  display: flex; flex-direction: column; gap: 10px; min-width: 0;
  transition: all .25s;
}
.is-card:hover { transform: translateY(-4px); box-shadow: var(--shadow-md); }
.isc-header { display: flex; align-items: center; gap: 8px; }
.isc-status { font-size: .82rem; font-weight: 700; }
.isc-title  { font-size: .95rem; font-weight: 700; color: var(--text-primary); }
.isc-notes  { display: flex; flex-direction: column; gap: 5px; }
.isc-note   { display: flex; align-items: flex-start; gap: 5px; font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }

/* ══════════════════════════════════════════
   PEP 8 IMPORT ORDER
══════════════════════════════════════════ */
.pep8-demo {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; overflow: hidden;
}
.pd-chrome {
  background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle);
  padding: 10px 16px;
}
.pd-filename { color: var(--text-muted); font-size: .88rem; font-family: 'Fira Code', monospace; }
.pd-body { padding: 0; }
.pd-zone { border-bottom: 1px solid var(--border-subtle); padding: 12px 16px 12px 0; }
.pd-zone:last-child { border-bottom: none; }
.pdz-label {
  display: flex; align-items: center; gap: 5px; font-size: .72rem; font-weight: 700;
  letter-spacing: .06em; text-transform: uppercase; margin-bottom: 8px; padding-left: 16px;
  opacity: .85;
}
.pdz-lines { padding-left: 16px; }
.pdz-line { padding: 2px 0; }
.pdz-code { font-family: 'Fira Code', monospace; font-size: .88rem; color: var(--text-secondary); }
.pdz-empty { display: block; height: 12px; }

/* ══════════════════════════════════════════
   __INIT__.PY COMPARISON
══════════════════════════════════════════ */
.init-comparison {
  display: grid; grid-template-columns: 1fr auto 1fr; gap: 14px; align-items: start;
}
.ic-panel {
  background: var(--bg-surface); border-radius: 14px; overflow: hidden;
  border: 1px solid var(--border-subtle);
}
.ic-bad  { border-top: 3px solid #f87171; }
.ic-good { border-top: 3px solid #4ade80; }
.icp-header {
  display: flex; align-items: center; gap: 8px; font-size: .86rem; font-weight: 700;
  padding: 10px 14px; border-bottom: 1px solid var(--border-subtle);
}
.ic-bad  .icp-header { background: rgba(248,113,113,.08); color: #f87171; }
.ic-good .icp-header { background: rgba( 74,222,128,.08); color: #4ade80; }
.icp-tree { padding: 14px; display: flex; flex-direction: column; gap: 4px; }
.icp-item {
  display: flex; align-items: center; gap: 7px;
  font-family: 'Fira Code', monospace; font-size: .84rem; color: var(--text-secondary);
  padding: 3px 6px; border-radius: 5px;
}
.icp-indent { padding-left: 22px; }
.icp-highlight {
  background: rgba(251,191,36,.12); border: 1px solid rgba(251,191,36,.3);
  color: #fbbf24 !important;
}
.icp-result {
  margin: 0 12px 12px; border-radius: 8px; padding: 8px 12px;
  font-family: 'Fira Code', monospace; font-size: .8rem;
  display: flex; flex-direction: column; gap: 4px;
}
.icp-result-bad  { background: rgba(248,113,113,.08); border: 1px solid rgba(248,113,113,.2); }
.icp-result-good { background: rgba( 74,222,128,.08); border: 1px solid rgba( 74,222,128,.2); }
.icp-error { color: #f87171; font-weight: 700; font-family: sans-serif; font-size: .8rem; }
.icp-ok    { color: #4ade80; font-weight: 700; font-family: sans-serif; font-size: .8rem; }
.ic-arrow {
  display: flex; flex-direction: column; align-items: center; gap: 6px;
  padding-top: 3rem; flex-shrink: 0; font-size: .78rem; color: var(--text-muted);
}

/* __init__ options */
.init-options { }
.io-title { font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.io-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.io-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--io-color, var(--border-medium));
  border-radius: 14px; padding: 1.1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.io-header { display: flex; align-items: center; gap: 10px; }
.io-level { font-size: .72rem; font-weight: 800; letter-spacing: .06em; }
.io-title-text { font-size: .9rem; font-weight: 700; color: var(--text-primary); }
.io-note { font-size: .8rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 6px 10px; }

/* __all__ section */
.all-section {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid #c084fc; border-radius: 14px; padding: 1.5rem;
}
.all-title {
  display: flex; align-items: center; gap: 8px; font-size: 1rem; font-weight: 700;
  color: var(--text-primary); margin-bottom: .75rem;
}

/* ══════════════════════════════════════════
   FILE TREE
══════════════════════════════════════════ */
.create-cmd {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 12px; overflow: hidden;
}
.cc-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  background: rgba(74,222,128,.08); border-bottom: 1px solid var(--border-subtle);
  font-size: .86rem; font-weight: 700; color: #4ade80;
}
.file-tree {
  background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden;
}
.ft-header {
  display: flex; align-items: center; gap: 8px; padding: 12px 16px;
  border-bottom: 1px solid var(--border-subtle); font-size: .88rem; font-weight: 600;
  color: var(--text-secondary); background: var(--bg-surface-solid);
}
.ft-body { padding: 8px 0; }
.ft-item {
  display: flex; align-items: center; padding: 6px 14px 6px;
  border-bottom: 1px solid var(--border-subtle); transition: background .15s;
  gap: 8px; flex-wrap: wrap;
}
.ft-item:last-child { border-bottom: none; }
.ft-item:hover { background: var(--bg-surface-hover); }
.ft-folder { font-weight: 700; }
.ft-critical { background: rgba(251,191,36,.05); }
.ft-important { background: rgba(249,115,22,.05); }
.ft-item-main { display: flex; align-items: center; gap: 7px; min-width: 220px; }
.ft-name { font-family: 'Fira Code', monospace; font-size: .86rem; }
.ft-badge-critical {
  font-size: .65rem; font-weight: 800; letter-spacing: .07em;
  background: rgba(251,191,36,.18); color: #fbbf24; border: 1px solid rgba(251,191,36,.35);
  border-radius: 999px; padding: 1px 7px;
}
.ft-badge-important {
  font-size: .65rem; font-weight: 800; letter-spacing: .07em;
  background: rgba(249,115,22,.15); color: #f97316; border: 1px solid rgba(249,115,22,.3);
  border-radius: 999px; padding: 1px 7px;
}
.ft-desc { font-size: .78rem; color: var(--text-muted); margin-left: auto; }

/* ══════════════════════════════════════════
   ENTRY POINTS FLOW
══════════════════════════════════════════ */
.ep-flow { display: flex; flex-direction: column; align-items: center; }
.ep-step-wrap { display: flex; flex-direction: column; align-items: center; width: 100%; }
.ep-step {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--ep-color, var(--border-medium));
  border-radius: 14px; padding: 1.25rem 1.5rem;
  display: flex; align-items: flex-start; gap: 14px; width: 100%; min-width: 0;
  transition: box-shadow .2s;
}
.ep-step:hover { box-shadow: var(--shadow-md); }
.eps-icon {
  width: 44px; height: 44px; border-radius: 12px; flex-shrink: 0;
  display: flex; align-items: center; justify-content: center;
}
.eps-body { flex: 1; min-width: 0; }
.eps-label { font-size: .75rem; font-weight: 800; letter-spacing: .06em; text-transform: uppercase; margin-bottom: 4px; }
.eps-code  { font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 700; color: var(--text-code); background: none; padding: 0; display: block; margin-bottom: 6px; word-break: break-all; }
.eps-desc  { font-size: .84rem; color: var(--text-muted); }
.ep-connector { padding: 6px 0; opacity: .6; }

/* setup comparison */
.setup-comparison { }
.sc-title {
  display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600;
  color: var(--text-secondary); margin-bottom: 12px;
}
.sc-panels { display: grid; grid-template-columns: repeat(2,1fr); gap: 14px; }
.sc-panel { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden; min-width: 0; }
.scp-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  border-bottom: 1px solid var(--border-subtle); font-size: .85rem; font-weight: 700;
}
.scp-py  { background: rgba( 96,165,250,.1); color: #60a5fa; }
.scp-cfg { background: rgba( 74,222,128,.1); color: #4ade80; }
.scp-note {
  padding: 8px 14px; font-size: .82rem; color: var(--text-muted);
  background: var(--bg-surface-hover); border-top: 1px solid var(--border-subtle);
}

/* colcon workflow */
.colcon-workflow { }
.cw-title {
  display: flex; align-items: center; gap: 8px; font-size: .95rem; font-weight: 700;
  color: var(--text-primary); margin-bottom: 14px;
}
.cw-steps { display: flex; flex-direction: column; }
.cw-step {
  display: flex; align-items: flex-start; gap: 14px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--cw-color, var(--border-medium));
  border-radius: 12px; padding: 1.1rem; margin-bottom: 8px; min-width: 0;
}
.cws-num {
  min-width: 34px; width: 34px; height: 34px; border-radius: 50%; flex-shrink: 0;
  display: flex; align-items: center; justify-content: center;
  font-size: .95rem; font-weight: 800; color: #1e1e1e;
}
.cws-body { flex: 1; min-width: 0; display: flex; flex-direction: column; gap: 6px; }
.cws-title { font-size: .9rem; font-weight: 700; color: var(--text-primary); }
.cws-note  { font-size: .8rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 5px 10px; }

/* ══════════════════════════════════════════
   ABSOLUTE VS RELATIVE
══════════════════════════════════════════ */
.ir-panel { height: 100%; display: flex; flex-direction: column; gap: 8px; }
.irp-header {
  display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700;
  color: var(--text-primary); padding: 8px 12px; border-radius: 8px;
  background: var(--bg-surface-hover);
}
.irp-pros { display: flex; flex-direction: column; gap: 5px; }
.irp-pro, .irp-con {
  display: flex; align-items: center; gap: 5px; font-size: .82rem; color: var(--text-secondary);
}
.irp-badge {
  font-size: .78rem; font-weight: 800; border-radius: 8px; padding: 5px 12px; text-align: center;
}
.irp-badge-ok   { background: rgba( 74,222,128,.12); color: #4ade80; border: 1px solid rgba( 74,222,128,.3); }
.irp-badge-warn { background: rgba(251,191, 36,.12); color: #fbbf24; border: 1px solid rgba(251,191, 36,.3); }

/* ══════════════════════════════════════════
   ERROR LIST
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid #ef4444; border-radius: 12px; overflow: hidden;
}
.err-header {
  display: flex; align-items: center; justify-content: space-between;
  padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s;
}
.err-header:hover { background: var(--bg-surface-hover); }
.err-left { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num {
  min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0;
  background: rgba(239,68,68,.15); color: #ef4444;
  font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center;
}
.err-msg     { font-size: .84rem; display: block; margin-bottom: 2px; color: #ef4444; background: none; padding: 0; word-break: break-all; }
.err-summary { font-size: .8rem; color: var(--text-muted); }
.err-body {
  padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle);
  display: flex; flex-direction: column; gap: 10px;
}
.err-cause, .err-fix {
  font-size: .87rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px;
}
.err-fix ol { margin: 4px 0 0 16px; }
.err-fix li { margin-bottom: 4px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX
══════════════════════════════════════════ */
.challenge-box {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b;
}
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon {
  width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px;
  display: flex; align-items: center; justify-content: center; flex-shrink: 0;
}
.challenge-title    { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge {
  margin-left: auto; font-size: .72rem; font-weight: 800; letter-spacing: .07em;
  padding: 4px 12px; border-radius: 999px;
  background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3); white-space: nowrap;
}
.challenge-reqs { background: var(--bg-surface-hover); border-radius: 12px; padding: 1rem 1.25rem; }
.cr-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 10px; }
.cr-list { display: flex; flex-direction: column; gap: 8px; }
.cr-step { display: flex; align-items: flex-start; gap: 10px; }
.cr-step-num {
  min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0;
  font-size: .82rem; font-weight: 800; color: #1e1e1e;
  display: flex; align-items: center; justify-content: center;
}
.cr-step-text { font-size: .87rem; color: var(--text-secondary); padding-top: 3px; }
:deep(.answer-header) {
  background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25);
  border-radius: 10px; color: #22c55e;
}
.answer-body {
  background: var(--bg-surface-hover); padding: 1.1rem 1.25rem;
  border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px;
}
.answer-row { display: flex; align-items: baseline; gap: 8px; font-size: .88rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   VIDEO CARD
══════════════════════════════════════════ */
.video-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 1.25rem; overflow: hidden;
}
.video-wrapper {
  position: relative; padding-bottom: 56.25%; height: 0;
  overflow: hidden; border-radius: 10px; background: #000;
}
.video-wrapper iframe { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.video-caption {
  display: flex; align-items: center; margin-top: 12px; font-size: .82rem;
  color: var(--text-muted); padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px;
}

/* ══════════════════════════════════════════
   SUMMARY GRID
══════════════════════════════════════════ */
.summary-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.summary-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sc-color, var(--border-medium));
  border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s;
}
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .92rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; }
.sc-desc    { font-size: .83rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .73rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .ft-desc { display: none; }
  .ft-item { flex-wrap: nowrap; }
}

@media (max-width: 900px) {
  .import-styles-grid { grid-template-columns: 1fr; }
  .io-grid            { grid-template-columns: 1fr 1fr; }
  .sc-panels          { grid-template-columns: 1fr; }
  .summary-grid       { grid-template-columns: repeat(2,1fr); }
  .init-comparison    { grid-template-columns: 1fr; }
  .ic-arrow           { flex-direction: row; padding-top: 0; justify-content: center; }
}

@media (max-width: 768px) {
  .import-styles-grid { grid-template-columns: 1fr; }
  .io-grid            { grid-template-columns: 1fr; }
  .sc-panels          { grid-template-columns: 1fr; }
  .challenge-header   { flex-direction: column; }
  .challenge-badge    { margin-left: 0; }
  .fact-pills         { flex-direction: column; gap: 8px; }
}

@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
  .ft-item-main { min-width: unset; }
}
</style>
