<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         SECCIÓN 1: CONTEXTO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Ya sabes moverte por el sistema. Ahora aprenderás a
        <strong>manipular la materia digital</strong>: crear archivos, duplicarlos, moverlos y
        destruirlos.<br /><br />
        En robótica, esto es el día a día: organizar logs de sensores, hacer backups de
        configuraciones, limpiar archivos temporales de simulaciones, y estructurar tu workspace de
        ROS 2. <strong>Un sistema desorganizado es un robot que falla.</strong>
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>

      <AlertBlock type="info" title="La Filosofía Unix: Todo es un Archivo">
        En Linux, <strong>todo</strong> es un archivo. Tu código Python es un archivo. La
        configuración de tu robot es un archivo. Incluso los dispositivos de hardware (cámara,
        lidar) aparecen como archivos en <code>/dev</code>.<br /><br />
        Dominar la gestión de archivos es dominar el sistema operativo.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 2: MKDIR
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Crear Directorios — <code class="cmd-inline">mkdir</code>
      </SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Antes de crear archivos necesitas un lugar donde guardarlos.
            <code>mkdir</code> crea carpetas al instante.<br /><br />
            <strong>Regla de oro:</strong> Organiza tu código en carpetas lógicas desde el día 1.
            Un proyecto robótico puede tener cientos de archivos.
          </TextBlock>

          <CodeBlock title="Uso básico" lang="bash"
            content="mkdir mi_robot
ls -l" :copyable="true" />

          <AlertBlock type="success" title="Bandera -p: Crea toda la jerarquía de una vez">
            Sin <code>-p</code> tendrías que crear cada carpeta una por una.
            Con <code>-p</code> creas todo el árbol en un comando.
          </AlertBlock>
        </template>

        <template #right>
          <div class="tree-demo-card">
            <div class="tree-demo-header">
              <q-icon name="account_tree" size="16px" color="primary" />
              <span>Workspace ROS 2 con <code>-p</code></span>
            </div>
            <div class="tree-demo-body">
              <CodeBlock :hide-header="true" lang="bash"
                content="mkdir -p ros2_ws/src/mi_robot/launch
mkdir -p ros2_ws/src/mi_robot/config
mkdir -p ros2_ws/src/mi_robot/scripts" />
            </div>
            <div class="tree-result">
              <div class="tr-node root"><q-icon name="folder_open" size="14px" style="color:#eab308" /> ros2_ws/</div>
              <div class="tr-children">
                <div class="tr-node"><q-icon name="folder" size="13px" style="color:#60a5fa" /> src/</div>
                <div class="tr-children">
                  <div class="tr-node"><q-icon name="folder" size="13px" style="color:#4ade80" /> mi_robot/</div>
                  <div class="tr-children">
                    <div class="tr-node"><q-icon name="folder" size="12px" style="color:#c084fc" /> launch/</div>
                    <div class="tr-node"><q-icon name="folder" size="12px" style="color:#fb923c" /> config/</div>
                    <div class="tr-node"><q-icon name="folder" size="12px" style="color:#f472b6" /> scripts/</div>
                  </div>
                </div>
              </div>
            </div>
            <div class="tree-note">
              <q-icon name="lightbulb" size="13px" color="warning" class="q-mr-xs" />
              Crea todos los niveles aunque los intermedios no existan
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 3: TOUCH
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">02</span>
        Crear Archivos Vacíos — <code class="cmd-inline">touch</code>
      </SectionTitle>

      <div class="two-col-grid q-mt-lg">
        <div class="info-card" style="--ic-color: #4ade80">
          <div class="ic-header">
            <q-icon name="note_add" size="22px" style="color:#4ade80" />
            <span class="ic-title">Uso Principal — Crear</span>
          </div>
          <p class="ic-desc">
            <code>touch</code> crea un archivo totalmente vacío (0 bytes).
            Es como sacar una hoja de papel en blanco.<br /><br />
            <strong>¿Para qué crear archivos vacíos?</strong> Para reservar el nombre antes de
            escribir código, o crear placeholders en tu estructura de proyecto.
          </p>
          <CodeBlock lang="bash"
            content="touch nodo_control.py
touch config.yaml
ls -lh" :copyable="true" />
        </div>

        <div class="info-card" style="--ic-color: #60a5fa">
          <div class="ic-header">
            <q-icon name="schedule" size="22px" style="color:#60a5fa" />
            <span class="ic-title">Uso Secundario — Actualizar Timestamp</span>
          </div>
          <p class="ic-desc">
            Si el archivo <strong>ya existe</strong>, <code>touch</code> solo actualiza su
            fecha de modificación sin borrar el contenido.<br /><br />
            Útil en sistemas de compilación (Make) donde la fecha determina qué archivos recompilar.
          </p>
          <CodeBlock lang="bash"
            content="# Archivo existe con contenido
touch archivo_viejo.txt

# Solo cambia la fecha, contenido intacto
ls -l archivo_viejo.txt" :copyable="true" />
        </div>
      </div>

      <AlertBlock type="info" title="Crear múltiples archivos de una vez" class="q-mt-md">
        Separa los nombres con espacios en un solo comando:
        <br /><code>touch archivo1.txt archivo2.txt archivo3.txt</code>
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 4: CP
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">03</span>
        Copiar Archivos y Directorios — <code class="cmd-inline">cp</code>
      </SectionTitle>

      <TextBlock>
        <code>cp</code> duplica archivos. Siempre necesita dos argumentos:
        <strong>origen</strong> (qué copiar) y <strong>destino</strong> (a dónde).
      </TextBlock>

      <!-- Syntax display -->
      <div class="syntax-card q-mt-md">
        <span class="syntax-eyebrow">SINTAXIS</span>
        <div class="syntax-line">
          <span class="syn-cmd">cp</span>
          <span class="syn-arg muted">[opciones]</span>
          <span class="syn-arg blue">origen</span>
          <span class="syn-arrow">→</span>
          <span class="syn-arg purple">destino</span>
        </div>
      </div>

      <div class="op-cards-grid q-mt-lg">
        <div v-for="op in cpOps" :key="op.title" class="op-card" :style="{ '--op-color': op.color }">
          <div class="op-icon-wrap" :style="{ background: op.color + '18' }">
            <q-icon :name="op.icon" size="24px" :style="{ color: op.color }" />
          </div>
          <div class="op-title">{{ op.title }}</div>
          <div class="op-flag"><code>{{ op.flag }}</code></div>
          <p class="op-desc">{{ op.desc }}</p>
          <CodeBlock :hide-header="true" lang="bash" :content="op.code" />
          <div class="op-note">
            <q-icon name="info" size="12px" class="q-mr-xs" />{{ op.note }}
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="Caso de uso real: Backup antes de modificar">
        Antes de tocar un archivo crítico (launch file, YAML de Nav2), siempre haz copia:
        <br /><br />
        <code>cp nav2_params.yaml nav2_params.yaml.backup</code>
        <br /><br />
        Si algo sale mal: <code>cp nav2_params.yaml.backup nav2_params.yaml</code>
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 5: MV
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">04</span>
        Mover y Renombrar — <code class="cmd-inline">mv</code>
      </SectionTitle>

      <TextBlock>
        En Linux <strong>mover</strong> y <strong>renombrar</strong> son la misma operación: <code>mv</code>.
        La diferencia está en el destino que indiques.
      </TextBlock>

      <div class="mv-grid q-mt-lg">
        <div v-for="m in mvCases" :key="m.title" class="mv-card" :style="{ '--mv-color': m.color }">
          <div class="mv-card-header">
            <div class="mv-letter" :style="{ background: m.color + '22', color: m.color }">{{ m.letter }}</div>
            <div>
              <div class="mv-card-title">{{ m.title }}</div>
              <div class="mv-card-sub">{{ m.sub }}</div>
            </div>
          </div>
          <CodeBlock :hide-header="true" lang="bash" :content="m.code" />
        </div>
      </div>

      <AlertBlock type="success" title="Pro tip: Renombrar múltiples archivos con un loop">
        <code>for f in *.txt; do mv "$f" "${f%.txt}.log"; done</code><br /><br />
        Convierte todos los <code>.txt</code> a <code>.log</code> en un solo comando.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 6: RM — ZONA DE PELIGRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Eliminar — <code class="cmd-inline">rm</code>
        <span class="danger-tag">PELIGRO</span>
      </SectionTitle>

      <!-- Danger banner -->
      <div class="danger-banner q-mt-lg">
        <div class="danger-stripe"></div>
        <div class="danger-content">
          <q-icon name="warning" size="2.5rem" color="negative" />
          <div class="danger-text">
            <div class="danger-title">EN LA TERMINAL NO EXISTE LA PAPELERA</div>
            <div class="danger-sub">
              Cuando borras con <code>rm</code>, desaparece para siempre.
              No hay <kbd>Ctrl+Z</kbd>. No hay "Restaurar". <strong>Verifica dos veces antes de Enter.</strong>
            </div>
          </div>
        </div>
      </div>

      <!-- rm variants -->
      <div class="rm-grid q-mt-lg">
        <div v-for="r in rmVariants" :key="r.title" class="rm-card" :style="{ '--rm-color': r.color }">
          <div class="rm-top">
            <q-icon :name="r.icon" size="20px" :style="{ color: r.color }" />
            <code class="rm-flag">{{ r.flag }}</code>
          </div>
          <div class="rm-title">{{ r.title }}</div>
          <CodeBlock :hide-header="true" lang="bash" :content="r.code" />
          <div class="rm-note">{{ r.note }}</div>
        </div>
      </div>

      <AlertBlock type="danger" title="El comando más destructivo del universo" class="q-mt-lg">
        <code>rm -rf /</code> con permisos de root borraría <strong>TODO</strong> tu sistema operativo.<br /><br />
        <strong>Nunca</strong> copies comandos de internet sin entenderlos. Un espacio mal puesto puede destruirlo todo:<br />
        <code>rm -rf carpeta /</code> — nota el espacio antes de <code>/</code>: borraría la carpeta <em>y</em> la raíz.
      </AlertBlock>

      <div class="q-mt-md">
        <SplitBlock>
          <template #left>
            <AlertBlock type="info" title="Alternativa segura: Mover a Papelera">
              <code>gio trash archivo.txt</code><br /><br />
              Mueve el archivo a la papelera del sistema en lugar de borrarlo permanentemente.
            </AlertBlock>
          </template>
          <template #right>
            <AlertBlock type="success" title="Protección: Alias en ~/.bashrc">
              <code>alias rm='rm -i'</code><br /><br />
              Así <code>rm</code> siempre pedirá confirmación. Aplica con <code>source ~/.bashrc</code>.
            </AlertBlock>
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 7: WILDCARDS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Wildcards — Operaciones en Masa</SectionTitle>
      <TextBlock>
        Los <strong>wildcards</strong> (comodines) representan múltiples archivos a la vez.
        Son esenciales para trabajar eficientemente con docenas de archivos.
      </TextBlock>

      <div class="wc-grid q-mt-lg">
        <div v-for="w in wildcards" :key="w.symbol" class="wc-card">
          <div class="wc-symbol" :style="{ color: w.color }">{{ w.symbol }}</div>
          <div class="wc-name">{{ w.name }}</div>
          <div class="wc-desc">{{ w.desc }}</div>
          <div class="wc-examples">
            <div v-for="ex in w.examples" :key="ex.pattern" class="wc-example">
              <code class="wc-pattern" :style="{ color: w.color }">{{ ex.pattern }}</code>
              <span class="wc-result">→ {{ ex.result }}</span>
            </div>
          </div>
        </div>
      </div>

      <!-- Wildcard practice table -->
      <div class="wc-practice q-mt-xl">
        <div class="wcp-header">
          <q-icon name="terminal" size="16px" color="primary" />
          Ejemplos Prácticos en ROS 2
        </div>
        <div class="wcp-rows">
          <div v-for="p in wcPractice" :key="p.cmd" class="wcp-row">
            <code class="wcp-cmd">{{ p.cmd }}</code>
            <span class="wcp-desc">{{ p.desc }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 8: ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Cómo Resolverlos</SectionTitle>

      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-number">{{ i + 1 }}</div>
              <div>
                <code class="err-msg">{{ err.msg }}</code>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'" size="20px" style="color: var(--text-muted)" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Causa:</strong> {{ err.cause }}
            </div>
            <div class="err-solution">
              <q-icon name="check_circle" size="14px" class="q-mr-xs" color="positive" />
              <strong>Solución:</strong>
              <ol><li v-for="s in err.steps" :key="s">{{ s }}</li></ol>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 9: RETO PRÁCTICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto de Ingeniería — Organizar un Workspace</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Construye un workspace ROS 2 desde cero</div>
            <div class="challenge-subtitle">
              Ejecuta los comandos línea por línea y observa cómo se construye la estructura.
            </div>
          </div>
          <div class="challenge-badge">NIVEL BÁSICO</div>
        </div>

        <CodeBlock title="Construcción de Workspace Completo" lang="bash"
          content="# 1. Crear estructura base
mkdir -p ~/ros2_ws/src/mi_robot/{launch,config,scripts,msg}

# 2. Crear archivos de configuración
cd ~/ros2_ws/src/mi_robot
touch package.xml setup.py
touch config/nav2_params.yaml
touch launch/robot_bringup.launch.py
touch scripts/control_node.py
touch msg/SensorData.msg

# 3. Verificar estructura
cd ~/ros2_ws
find src/ -type f

# 4. Hacer backup de configuración
cp config/nav2_params.yaml config/nav2_params.yaml.backup

# 5. Simular limpieza de temporales
cd ~/ros2_ws && touch build.log install.log
rm *.log

# 6. Verificar resultado final
ls -lah" :copyable="true" />

        <q-expansion-item
          icon="lightbulb"
          label="Ver resultado esperado (no hagas trampa)"
          header-class="answer-header"
          class="q-mt-md"
        >
          <div class="answer-body">
            <div class="answer-row">
              <span class="answer-key">📂 Estructura creada:</span>
              <span>ros2_ws/src/mi_robot/ con launch/, config/, scripts/, msg/</span>
            </div>
            <div class="answer-row">
              <span class="answer-key">📄 Archivos creados:</span>
              <span>package.xml, setup.py, nav2_params.yaml, robot_bringup.launch.py, control_node.py, SensorData.msg</span>
            </div>
            <div class="answer-row">
              <span class="answer-key">💾 Backup:</span>
              <code>config/nav2_params.yaml.backup</code>
            </div>
            <div class="answer-row">
              <span class="answer-key">🧹 Limpieza:</span>
              <span>Los archivos build.log e install.log fueron eliminados con el wildcard <code>*.log</code></span>
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 10: VIDEO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>Mira estas operaciones de gestión de archivos en acción:</TextBlock>

      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Gestión de Archivos en Linux" frameborder="0"
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
         SECCIÓN 11: RESUMEN
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Comandos Esenciales</SectionTitle>

      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryCommands" :key="s.cmd" class="summary-card" :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="terminal" size="12px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="¡Siguiente paso!" class="q-mt-xl">
        Ahora que dominas la gestión de archivos, el siguiente módulo te enseñará sobre
        <strong>permisos y usuarios</strong>: controlar quién puede leer, escribir o ejecutar tus archivos.
        ¡Esencial para la seguridad de tu robot!
      </AlertBlock>
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

// ── Fact pills ────────────────────────────────────────────────────
const facts = [
  { icon: '📁', label: 'mkdir — crea directorios y jerarquías' },
  { icon: '📄', label: 'touch — crea archivos vacíos al instante' },
  { icon: '📋', label: 'cp/mv/rm — copia, mueve y elimina' },
];

// ── cp operations ─────────────────────────────────────────────────
const cpOps = [
  {
    title: 'Copiar Archivo', flag: '(sin flags)', color: '#22d3ee',
    icon: 'content_copy',
    desc: 'Duplica un archivo en el mismo directorio o en otro. El original permanece intacto.',
    code: '# Mismo directorio\ncp robot.py robot_backup.py\n\n# A otro directorio\ncp robot.py ~/backups/',
    note: 'El archivo origen permanece intacto',
  },
  {
    title: 'Copiar Directorio', flag: '-r (recursive)', color: '#c084fc',
    icon: 'folder_copy',
    desc: 'Copia carpetas completas incluyendo todos sus archivos y subdirectorios.',
    code: '# Copiar carpeta completa\ncp -r mi_robot/ mi_robot_backup/\n\n# Copia TODO el árbol',
    note: 'Sin -r obtendrás error: "omitting directory"',
  },
  {
    title: 'Copiar con Confirmación', flag: '-i (interactive)', color: '#fb923c',
    icon: 'help_outline',
    desc: 'Pregunta antes de sobrescribir un archivo existente en el destino.',
    code: '# Pregunta si el destino existe\ncp -i robot.py robot_backup.py\n# overwrite \'robot_backup.py\'? (y/n)',
    note: 'Evita pérdidas accidentales de datos',
  },
];

// ── mv cases ──────────────────────────────────────────────────────
const mvCases = [
  {
    letter: 'A', title: 'Mover a otra carpeta', color: '#22d3ee',
    sub: 'El destino es una carpeta existente',
    code: '# Mueve robot.py dentro de scripts/\nmv robot.py scripts/\n\n# Antes: ./robot.py\n# Después: scripts/robot.py',
  },
  {
    letter: 'B', title: 'Renombrar en el mismo lugar', color: '#a78bfa',
    sub: 'El destino es un nombre nuevo (no una carpeta)',
    code: '# Renombrar archivo\nmv robot_viejo.py robot_nuevo.py\n\n# Renombrar carpeta\nmv carpeta_vieja/ carpeta_nueva/',
  },
  {
    letter: 'C', title: 'Mover Y renombrar', color: '#4ade80',
    sub: 'Mueve a otra carpeta y cambia el nombre a la vez',
    code: '# Mover y renombrar simultáneamente\nmv robot.py scripts/control_node.py\n\n# Cambia de nombre Y de ubicación',
  },
];

// ── rm variants ───────────────────────────────────────────────────
const rmVariants = [
  {
    title: 'Borrar archivo', flag: 'rm archivo', color: '#94a3b8',
    icon: 'description',
    code: 'rm archivo.txt',
    note: 'Simple y directo. El archivo desaparece permanentemente.',
  },
  {
    title: 'Borrar con confirmación', flag: 'rm -i', color: '#fbbf24',
    icon: 'help_outline',
    code: 'rm -i archivo.txt\n# remove archivo.txt? (y/n)',
    note: 'Pregunta antes de borrar. Usa -i siempre que tengas dudas.',
  },
  {
    title: 'Borrar directorio', flag: 'rm -r', color: '#f87171',
    icon: 'folder_delete',
    code: 'rm -r carpeta/\n\n# -ri: borra con confirmación\nrm -ri carpeta/',
    note: '-r (recursive) borra la carpeta y todo su contenido',
  },
];

// ── Wildcards ─────────────────────────────────────────────────────
const wildcards = [
  {
    symbol: '*', name: 'Asterisco', color: '#f59e0b',
    desc: 'Cualquier cantidad de caracteres (incluso ninguno)',
    examples: [
      { pattern: '*.py', result: 'Todos los archivos Python' },
      { pattern: 'robot_*', result: 'robot_v1.py, robot_v2.py…' },
      { pattern: 'log_*_2024*', result: 'Archivos de log de 2024' },
    ],
  },
  {
    symbol: '?', name: 'Interrogación', color: '#a78bfa',
    desc: 'Exactamente UN carácter (el que sea)',
    examples: [
      { pattern: 'log?.txt', result: 'log1.txt, log2.txt, logA.txt' },
      { pattern: 'node_?.py', result: 'node_1.py, node_2.py' },
      { pattern: 'v?.?.?', result: 'v1.0.0, v2.3.1' },
    ],
  },
  {
    symbol: '[]', name: 'Corchetes', color: '#22d3ee',
    desc: 'UNO de los caracteres dentro de los corchetes',
    examples: [
      { pattern: 'log[123].txt', result: 'log1.txt, log2.txt, log3.txt' },
      { pattern: '[A-Z]*.py', result: 'Archivos que empiezan con mayúscula' },
      { pattern: '*.[ch]', result: 'Archivos .c y .h (C/C++)' },
    ],
  },
];

const wcPractice = [
  { cmd: 'cp *.py backup/', desc: 'Copia todos los scripts Python a la carpeta de backup' },
  { cmd: 'rm temp_*.log', desc: 'Elimina todos los logs temporales de simulación' },
  { cmd: 'mv *.yaml config/', desc: 'Mueve todos los archivos YAML a la carpeta config' },
  { cmd: 'ls *.db3', desc: 'Lista todos los rosbags grabados' },
  { cmd: 'touch test_{1,2,3}.py', desc: 'Crea test_1.py, test_2.py y test_3.py a la vez' },
];

// ── Common errors ─────────────────────────────────────────────────
const commonErrors = reactive([
  {
    msg: "mkdir: cannot create directory 'carpeta': File exists",
    summary: 'Intentaste crear una carpeta que ya existe',
    cause: 'El directorio ya existe en esa ruta.',
    steps: [
      'Usa ls para verificar qué carpetas ya existen.',
      'Usa mkdir -p que ignora el error si la carpeta ya existe.',
      'Elige otro nombre para el directorio.',
    ],
    open: false,
  },
  {
    msg: "cp: cannot stat 'archivo.txt': No such file or directory",
    summary: 'El archivo origen no existe o el nombre está mal escrito',
    cause: 'El archivo que intentas copiar no existe en el directorio actual.',
    steps: [
      'Verifica que estás en el directorio correcto con pwd.',
      'Lista los archivos disponibles con ls.',
      'Usa TAB para autocompletar nombres y evitar errores tipográficos.',
      'Recuerda: Linux es case-sensitive (Archivo.txt ≠ archivo.txt).',
    ],
    open: false,
  },
  {
    msg: "rm: cannot remove 'carpeta': Is a directory",
    summary: 'Intentaste borrar una carpeta sin la bandera -r',
    cause: 'rm sin flags solo borra archivos. Para carpetas necesitas -r.',
    steps: [
      'Usa rm -r carpeta/ para borrar la carpeta y todo su contenido.',
      'Usa rm -ri carpeta/ si quieres confirmar cada archivo.',
      'Si la carpeta está vacía, puedes usar rmdir carpeta/.',
    ],
    open: false,
  },
  {
    msg: 'Borré un archivo importante por accidente',
    summary: 'rm eliminó algo que necesitabas',
    cause: 'rm es permanente — no existe papelera de reciclaje en la terminal.',
    steps: [
      'Si usas Git, puedes recuperar versiones con git checkout -- archivo.',
      'Haz backups regulares con cp o rsync antes de borrar.',
      'Configura alias rm=\'rm -i\' en ~/.bashrc para pedir confirmación siempre.',
      'Considera instalar trash-cli para tener papelera: gio trash archivo.',
    ],
    open: false,
  },
]);

// ── Summary ───────────────────────────────────────────────────────
const summaryCommands = [
  { cmd: 'mkdir -p',  desc: 'Crear directorio (toda la jerarquía de una vez)', example: 'mkdir -p a/b/c', color: '#4ade80' },
  { cmd: 'touch',     desc: 'Crear archivo vacío o actualizar timestamp',       example: 'touch robot.py', color: '#22d3ee' },
  { cmd: 'cp -r',     desc: 'Copiar archivos o directorios',                    example: 'cp -r src/ bak/', color: '#c084fc' },
  { cmd: 'mv',        desc: 'Mover o renombrar archivos y carpetas',            example: 'mv old.txt new.txt', color: '#f59e0b' },
  { cmd: 'rm -r',     desc: 'Eliminar archivos o directorios (¡irreversible!)', example: 'rm -r temp/', color: '#f87171' },
  { cmd: '* ? []',    desc: 'Wildcards para operar sobre múltiples archivos',   example: 'rm *.log', color: '#fb923c' },
];
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background: var(--bg-code);
  color: var(--text-code);
  padding: 2px 7px;
  border-radius: 5px;
  font-family: 'Fira Code', monospace;
  font-size: 0.9em;
}
kbd {
  background: var(--bg-surface-hover);
  border: 1px solid var(--border-medium);
  border-bottom-width: 3px;
  color: var(--text-primary);
  padding: 2px 7px;
  border-radius: 5px;
  font-size: 0.85em;
}

/* ── Section title enhancements */
.cmd-badge {
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: 0.75rem; font-weight: 800;
  margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }

.cmd-inline { font-family:'Fira Code',monospace; font-size:.95em; background:none; color:var(--text-code); padding:0; }

.danger-tag {
  display: inline-flex; align-items: center;
  font-size: 0.65rem; font-weight: 800; letter-spacing: .1em;
  background: rgba(239,68,68,.15); color: #ef4444;
  border: 1px solid rgba(239,68,68,.3);
  padding: 2px 8px; border-radius: 999px;
  margin-left: 10px; vertical-align: middle;
}

/* ── Fact pills */
.fact-pills { display:flex; gap:10px; flex-wrap:wrap; margin-bottom:1.5rem; }
.fact-pill {
  display:flex; align-items:center; gap:8px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 999px; padding: 7px 16px;
  font-size: 0.84rem; color: var(--text-secondary);
  transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fact-icon { font-size: 1rem; }

/* ══════════════════════════════════════════
   MKDIR TREE DEMO
══════════════════════════════════════════ */
.tree-demo-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; overflow: hidden; height: 100%;
  display: flex; flex-direction: column;
}
.tree-demo-header {
  background: var(--bg-deep); border-bottom: 1px solid var(--border-subtle);
  padding: 10px 16px; display: flex; align-items: center; gap: 8px;
  font-size: 0.85rem; font-weight: 600; color: var(--text-secondary);
}
.tree-demo-body { padding: 0 12px; }
.tree-result {
  padding: 1rem 1.25rem;
  font-family: 'Fira Code', monospace; font-size: 0.88rem;
}
.tr-node {
  display: flex; align-items: center; gap: 6px;
  padding: 3px 8px; border-radius: 5px; color: var(--text-secondary);
  transition: background .2s;
}
.tr-node:hover { background: var(--bg-surface-hover); }
.tr-children {
  margin-left: 16px;
  border-left: 2px dashed var(--border-medium);
  padding-left: 10px;
}
.tree-note {
  margin: 0 1rem 1rem 1rem;
  display: flex; align-items: center;
  font-size: 0.82rem; color: var(--text-muted);
  background: var(--bg-surface-hover); border-radius: 8px; padding: 8px 12px;
}

/* ══════════════════════════════════════════
   TOUCH — INFO CARDS (2 col)
══════════════════════════════════════════ */
.two-col-grid {
  display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px;
}
.info-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ic-color, var(--border-medium));
  border-radius: 14px; padding: 1.25rem;
  display: flex; flex-direction: column; gap: 12px;
  min-width: 0;
}
.ic-header { display:flex; align-items:center; gap:10px; }
.ic-title { font-size:1rem; font-weight:700; color:var(--text-primary); }
.ic-desc { font-size:0.9rem; color:var(--text-secondary); line-height:1.55; margin:0; }

/* ══════════════════════════════════════════
   CP — SYNTAX + OP CARDS
══════════════════════════════════════════ */
.syntax-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 12px; padding: 1.25rem 1.5rem;
}
.syntax-eyebrow {
  font-size: 0.7rem; font-weight: 700; letter-spacing: .1em;
  color: var(--text-muted); display: block; margin-bottom: 10px;
}
.syntax-line {
  font-family: 'Fira Code', monospace; font-size: 1.15rem;
  display: flex; align-items: center; gap: 10px; flex-wrap: wrap;
}
.syn-cmd   { color: var(--text-code); font-weight: 800; }
.syn-arg   { padding: 3px 10px; border-radius: 6px; font-size: 0.95rem; }
.syn-arg.muted  { background: var(--bg-surface-hover); color: var(--text-muted); }
.syn-arg.blue   { background: rgba(96,165,250,.15); color: #60a5fa; }
.syn-arg.purple { background: rgba(167,139,250,.15); color: #a78bfa; }
.syn-arrow { color: var(--text-muted); font-size: 1.1rem; }

.op-cards-grid {
  display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px;
}
.op-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--op-color, var(--border-medium));
  border-radius: 14px; padding: 1.25rem;
  display: flex; flex-direction: column; gap: 10px;
  min-width: 0; transition: all .25s;
}
.op-card:hover { transform: translateY(-4px); box-shadow: var(--shadow-md); }
.op-icon-wrap {
  width:48px; height:48px; border-radius:12px;
  display:flex; align-items:center; justify-content:center;
}
.op-title { font-size: 1rem; font-weight: 700; color: var(--text-primary); }
.op-flag code { font-family:'Fira Code',monospace; font-size:0.82rem; color:var(--op-color); background:none; padding:0; }
.op-desc { font-size:0.88rem; color:var(--text-secondary); line-height:1.5; margin:0; flex:1; }
.op-note {
  display:flex; align-items:center; font-size:0.8rem; color:var(--text-muted);
  background:var(--bg-surface-hover); border-radius:7px; padding:7px 10px;
}

/* ══════════════════════════════════════════
   MV — GRID
══════════════════════════════════════════ */
.mv-grid {
  display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px;
}
.mv-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--mv-color, var(--border-medium));
  border-radius: 12px; padding: 1.25rem;
  display: flex; flex-direction: column; gap: 10px;
  min-width: 0; transition: all .25s;
}
.mv-card:hover { transform: translateY(-4px); box-shadow: var(--shadow-md); }
.mv-card-header { display:flex; align-items:flex-start; gap:12px; }
.mv-letter {
  width:36px; height:36px; border-radius:10px;
  display:flex; align-items:center; justify-content:center;
  font-size:1rem; font-weight:900; flex-shrink:0;
}
.mv-card-title { font-size:0.95rem; font-weight:700; color:var(--text-primary); line-height:1.3; }
.mv-card-sub   { font-size:0.78rem; color:var(--text-muted); margin-top:2px; }
.mv-card :deep(pre), .mv-card :deep(code) { overflow-x:auto; max-width:100%; }

/* ══════════════════════════════════════════
   RM — DANGER BANNER + GRID
══════════════════════════════════════════ */
.danger-banner {
  background: linear-gradient(135deg, rgba(127,29,29,.18), rgba(153,27,27,.18));
  border: 2px solid rgba(239,68,68,.45);
  border-radius: 16px; overflow: hidden;
}
.danger-stripe {
  height: 6px;
  background: repeating-linear-gradient(
    45deg, #ef4444 0, #ef4444 8px, transparent 8px, transparent 16px
  );
  opacity: .7;
}
.danger-content {
  display: flex; align-items: flex-start; gap: 1.25rem;
  padding: 1.5rem;
}
.danger-text { flex: 1; }
.danger-title {
  font-size: 1.15rem; font-weight: 900; color: #ef4444;
  text-transform: uppercase; letter-spacing: .05em; margin-bottom: 8px;
}
.danger-sub { font-size:0.9rem; color:var(--text-secondary); line-height:1.6; }
.danger-sub code { color: #f87171; }

.rm-grid {
  display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px;
}
.rm-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--rm-color, var(--border-medium));
  border-radius: 12px; padding: 1.1rem;
  display: flex; flex-direction: column; gap: 8px;
  min-width: 0;
}
.rm-top { display:flex; align-items:center; gap:8px; }
.rm-flag { font-family:'Fira Code',monospace; font-size:0.85rem; color:var(--rm-color); background:none; padding:0; }
.rm-title { font-size:0.95rem; font-weight:700; color:var(--text-primary); }
.rm-note { font-size:0.8rem; color:var(--text-muted); background:var(--bg-surface-hover); border-radius:6px; padding:7px 10px; }

/* ══════════════════════════════════════════
   WILDCARDS
══════════════════════════════════════════ */
.wc-grid {
  display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px;
}
.wc-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 1.5rem;
  text-align: center; transition: all .25s;
}
.wc-card:hover { transform: translateY(-4px); box-shadow: var(--shadow-md); }
.wc-symbol {
  font-family:'Fira Code',monospace; font-size:2.8rem; font-weight:900;
  margin-bottom: 6px; line-height:1;
}
.wc-name { font-size:1rem; font-weight:700; color:var(--text-primary); margin-bottom:6px; }
.wc-desc { font-size:0.85rem; color:var(--text-secondary); margin-bottom:1rem; line-height:1.5; }
.wc-examples { display:flex; flex-direction:column; gap:8px; text-align:left; }
.wc-example {
  background: var(--bg-surface-hover); border-radius:7px; padding:8px 10px;
  display:flex; flex-direction:column; gap:3px;
}
.wc-pattern { font-family:'Fira Code',monospace; font-size:0.88rem; font-weight:700; background:none; padding:0; }
.wc-result  { font-size:0.78rem; color:var(--text-muted); }

.wc-practice {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; overflow: hidden;
}
.wcp-header {
  background: var(--bg-deep); border-bottom: 1px solid var(--border-subtle);
  padding: 10px 16px; display:flex; align-items:center; gap:8px;
  font-size:0.85rem; font-weight:600; color:var(--text-secondary);
}
.wcp-rows { display:flex; flex-direction:column; }
.wcp-row {
  display:flex; align-items:baseline; gap:16px; padding:10px 16px;
  border-bottom:1px solid var(--border-subtle);
  transition: background .15s;
}
.wcp-row:last-child { border-bottom:none; }
.wcp-row:hover { background: var(--bg-surface-hover); }
.wcp-cmd { font-family:'Fira Code',monospace; font-size:0.9rem; color:var(--text-code); flex-shrink:0; background:none; padding:0; }
.wcp-desc { font-size:0.88rem; color:var(--text-secondary); }

/* ══════════════════════════════════════════
   ERROR LIST (same as lesson 01)
══════════════════════════════════════════ */
.error-list { display:flex; flex-direction:column; gap:10px; }
.error-item {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:3px solid #ef4444; border-radius:12px; overflow:hidden;
}
.err-header {
  display:flex; align-items:center; justify-content:space-between;
  padding:1rem 1.25rem; cursor:pointer; gap:12px; transition:background .2s;
}
.err-header:hover { background:var(--bg-surface-hover); }
.err-left { display:flex; align-items:flex-start; gap:12px; min-width:0; }
.err-number {
  min-width:28px; width:28px; height:28px; border-radius:50%;
  background:rgba(239,68,68,.15); color:#ef4444;
  font-size:.8rem; font-weight:800;
  display:flex; align-items:center; justify-content:center; flex-shrink:0;
}
.err-msg { font-size:.88rem; display:block; margin-bottom:4px; color:#ef4444; background:none; padding:0; word-break:break-all; }
.err-summary { font-size:.82rem; color:var(--text-muted); }
.err-body {
  padding:1rem 1.5rem 1.25rem 1.5rem; border-top:1px solid var(--border-subtle);
  display:flex; flex-direction:column; gap:10px;
}
.err-cause, .err-solution {
  font-size:.9rem; color:var(--text-secondary);
  display:flex; align-items:flex-start; gap:6px; line-height:1.5;
}
.err-solution ol { margin:4px 0 0 16px; padding:0; }
.err-solution li { margin-bottom:4px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX (same as lesson 01)
══════════════════════════════════════════ */
.challenge-box {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:20px; padding:1.75rem; border-top:3px solid #f59e0b;
}
.challenge-header { display:flex; align-items:flex-start; gap:1rem; margin-bottom:1.25rem; flex-wrap:wrap; }
.challenge-icon {
  width:52px; height:52px; background:rgba(245,158,11,.15); border-radius:14px;
  display:flex; align-items:center; justify-content:center; flex-shrink:0;
}
.challenge-title { font-size:1.05rem; font-weight:700; color:var(--text-primary); margin-bottom:4px; }
.challenge-subtitle { font-size:.9rem; color:var(--text-secondary); }
.challenge-badge {
  margin-left:auto; font-size:.72rem; font-weight:800; letter-spacing:.07em;
  padding:4px 12px; border-radius:999px;
  background:rgba(34,197,94,.12); color:#22c55e; border:1px solid rgba(34,197,94,.3); white-space:nowrap;
}
:deep(.answer-header) {
  background:rgba(34,197,94,.08); border:1px solid rgba(34,197,94,.25);
  border-radius:10px; color:#22c55e;
}
.answer-body {
  background:var(--bg-surface-hover); padding:1.25rem 1.5rem;
  border-radius:0 0 10px 10px; display:flex; flex-direction:column; gap:10px;
}
.answer-row { display:flex; align-items:baseline; flex-wrap:wrap; gap:8px; font-size:.9rem; color:var(--text-secondary); }
.answer-key { font-weight:600; color:var(--text-primary); white-space:nowrap; }

/* ══════════════════════════════════════════
   VIDEO CARD
══════════════════════════════════════════ */
.video-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:16px; padding:1.25rem; overflow:hidden;
}
.video-wrapper {
  position:relative; padding-bottom:56.25%; height:0;
  overflow:hidden; border-radius:10px; background:#000;
}
.video-wrapper iframe { position:absolute; top:0; left:0; width:100%; height:100%; }
.video-caption {
  display:flex; align-items:center; margin-top:12px; font-size:.82rem;
  color:var(--text-muted); padding:8px 12px; background:var(--bg-surface-hover); border-radius:8px;
}

/* ══════════════════════════════════════════
   SUMMARY GRID
══════════════════════════════════════════ */
.summary-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:14px; }
.summary-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid var(--sc-color,var(--border-medium));
  border-radius:12px; padding:1rem 1.25rem; transition:all .25s;
}
.summary-card:hover { transform:translateY(-3px); box-shadow:var(--shadow-sm); }
.sc-cmd { display:block; font-family:'Fira Code',monospace; font-size:1.05rem; font-weight:700; color:var(--sc-color,var(--text-code)); background:none; padding:0; margin-bottom:6px; }
.sc-desc { font-size:.88rem; color:var(--text-secondary); margin-bottom:8px; line-height:1.45; }
.sc-example { display:flex; align-items:center; font-family:'Fira Code',monospace; font-size:.78rem; color:var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 900px) {
  .op-cards-grid { grid-template-columns: 1fr 1fr; }
  .mv-grid        { grid-template-columns: 1fr 1fr; }
  .rm-grid        { grid-template-columns: 1fr 1fr; }
  .wc-grid        { grid-template-columns: 1fr 1fr; }
  .summary-grid   { grid-template-columns: 1fr 1fr; }
}

@media (max-width: 768px) {
  .two-col-grid   { grid-template-columns: 1fr; }
  .op-cards-grid  { grid-template-columns: 1fr; }
  .mv-grid        { grid-template-columns: 1fr; }
  .rm-grid        { grid-template-columns: 1fr; }
  .wc-grid        { grid-template-columns: 1fr; }
  .summary-grid   { grid-template-columns: 1fr 1fr; }
  .fact-pills     { flex-direction: column; gap: 8px; }
  .fact-pill      { border-radius: 12px; }
  .danger-content { flex-direction: column; }
  .wcp-row        { flex-direction: column; gap: 4px; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
}

@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
  .op-card :deep(pre), .mv-card :deep(pre), .rm-card :deep(pre) { overflow-x: auto; }
  .mv-card-header { flex-direction: column; gap: 8px; }
}
</style>
