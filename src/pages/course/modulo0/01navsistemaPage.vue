<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         SECCIÓN 1: CONTEXTO E IMPORTANCIA
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Como ingeniero en robótica, el 90% de tu tiempo estarás conectado a robots o servidores que
        <strong>no tienen monitor ni mouse</strong>. Solo tendrás una pantalla negra y tu teclado.
        <br /><br />
        La terminal no es una reliquia del pasado. Es el <strong>lenguaje nativo</strong> de Linux, y
        Linux es el sistema operativo del 96% de los servidores del mundo y del 100% de los robots
        profesionales. Dominarla es la diferencia entre un usuario y un desarrollador.
      </TextBlock>

      <!-- Pill stats -->
      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-num" :style="{ color: f.color }">{{ f.num }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>

      <AlertBlock type="info" title="¿Por qué no usar interfaz gráfica?">
        Las interfaces gráficas (GUI) consumen recursos (RAM, CPU) que en un robot son críticos.
        Además, cuando trabajas remotamente vía SSH (conexión a distancia), solo tienes acceso a la terminal.
        <br /><br />
        <strong>Regla de oro:</strong> Si puedes hacerlo en la terminal, hazlo en la terminal.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 2: ANATOMÍA DEL PROMPT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Anatomía del Prompt (La Línea de Comandos)</SectionTitle>
      <TextBlock>
        Antes de ejecutar comandos, debes entender qué significa cada símbolo que ves en la terminal.
        Pasa el cursor sobre cada parte para ver su significado.
      </TextBlock>

      <div class="terminal-anatomy-card q-mt-lg">
        <!-- Terminal chrome -->
        <div class="term-chrome">
          <div class="term-dots">
            <span class="term-dot red"></span>
            <span class="term-dot yellow"></span>
            <span class="term-dot green"></span>
          </div>
          <span class="term-title">bash — 80×24</span>
        </div>

        <!-- Prompt display -->
        <div class="term-body">
          <div class="prompt-line">
            <div class="prompt-part" data-tooltip="Nombre de usuario">
              <span class="pp-user">alexander</span>
              <div class="pp-tooltip">👤 Usuario actual</div>
            </div>
            <span class="pp-sep">@</span>
            <div class="prompt-part" data-tooltip="Hostname">
              <span class="pp-host">robot</span>
              <div class="pp-tooltip">🖥 Nombre del equipo</div>
            </div>
            <span class="pp-sep">:</span>
            <div class="prompt-part" data-tooltip="Directorio actual">
              <span class="pp-path">~/ros2_ws</span>
              <div class="pp-tooltip">📁 Directorio actual<br/><small>~ = /home/alexander</small></div>
            </div>
            <div class="prompt-part" data-tooltip="Símbolo de usuario">
              <span class="pp-symbol">$</span>
              <div class="pp-tooltip">⚡ Usuario normal<br/><small># = superusuario (root)</small></div>
            </div>
            <span class="pp-cursor">█</span>
          </div>
        </div>

        <!-- Legend -->
        <div class="prompt-legend">
          <div class="leg-item" v-for="p in promptParts" :key="p.code">
            <span class="leg-dot" :style="{ background: p.color }"></span>
            <div>
              <div class="leg-code" :style="{ color: p.color }">{{ p.code }}</div>
              <div class="leg-desc">{{ p.desc }}</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 3: FILESYSTEM HIERARCHY
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>El Árbol de Directorios de Linux</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            A diferencia de Windows (<code>C:\</code>, <code>D:\</code>), Linux tiene una sola raíz: <code>/</code>
            <br /><br />
            Todo en el sistema es un archivo o directorio — incluso los dispositivos de hardware
            (como tu cámara USB) aparecen como archivos en <code>/dev</code>.
          </TextBlock>

          <AlertBlock type="warning" title="Directorios Críticos para ROS 2">
            <ul class="dir-list">
              <li>
                <code>/home</code>
                <span>Tu espacio personal. Aquí vive tu workspace de ROS.</span>
              </li>
              <li>
                <code>/opt/ros</code>
                <span>Instalación oficial de ROS 2 (Jazzy, Humble…)</span>
              </li>
              <li>
                <code>/usr/bin</code>
                <span>Binarios ejecutables del sistema (python3, gcc)</span>
              </li>
              <li>
                <code>/tmp</code>
                <span>Archivos temporales — se borran al reiniciar</span>
              </li>
            </ul>
          </AlertBlock>
        </template>

        <template #right>
          <div class="fs-tree-card">
            <div class="fs-tree-header">
              <q-icon name="account_tree" color="primary" size="sm" />
              <span>Estructura del Filesystem</span>
            </div>
            <div class="fs-tree">
              <div class="fs-node root-node">
                <q-icon name="folder_open" size="18px" style="color: #eab308" />
                <span class="fn-name root">/ <small class="fn-hint">raíz</small></span>
              </div>
              <div class="fs-children">
                <div v-for="dir in fsTree" :key="dir.name" class="fs-branch">
                  <div class="fs-node" :class="{ highlight: dir.highlight }">
                    <q-icon name="folder" size="16px" :style="{ color: dir.color }" />
                    <span class="fn-name">{{ dir.name }}</span>
                    <span v-if="dir.badge" class="fn-badge">{{ dir.badge }}</span>
                  </div>
                  <div v-if="dir.children" class="fs-children nested">
                    <div v-for="child in dir.children" :key="child.name" class="fs-node child">
                      <q-icon name="folder" size="14px" :style="{ color: child.color }" />
                      <span class="fn-name">{{ child.name }}</span>
                      <span v-if="child.badge" class="fn-badge">{{ child.badge }}</span>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 4: COMANDO PWD
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">01</span>
        Ubicación Actual — <code class="cmd-inline">pwd</code>
      </SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El comando <code>pwd</code> (<em>Print Working Directory</em>) es tu GPS.
            Te devuelve la ruta <strong>absoluta</strong> desde la raíz (<code>/</code>) hasta donde estás.
            <br /><br />
            <strong>¿Cuándo usarlo?</strong> Siempre que te sientas perdido después de varios
            <code>cd</code>, o cuando necesites copiar la ruta exacta para un script.
          </TextBlock>

          <CodeBlock
            title="pwd en acción"
            lang="bash"
            content="alexander@robot:~$ pwd
/home/alexander

alexander@robot:~$ cd /opt/ros/jazzy
alexander@robot:/opt/ros/jazzy$ pwd
/opt/ros/jazzy"
            :copyable="true"
          />
        </template>

        <template #right>
          <div class="concept-card">
            <div class="concept-header">
              <q-icon name="fork_right" color="positive" size="md" />
              <span class="concept-title">Rutas: Absoluta vs Relativa</span>
            </div>
            <div class="concept-item accent-green">
              <div class="ci-label" style="color: #4ade80">ABSOLUTA</div>
              <p class="ci-desc">Empieza desde <code>/</code>. Funciona desde cualquier ubicación.</p>
              <div class="ci-example">/home/alexander/ros2_ws/src</div>
            </div>
            <div class="concept-item accent-blue">
              <div class="ci-label" style="color: #60a5fa">RELATIVA</div>
              <p class="ci-desc">Empieza desde donde estás. Más corta pero depende del contexto.</p>
              <div class="ci-example">../install/setup.bash</div>
            </div>
            <div class="concept-tip">
              <q-icon name="tips_and_updates" size="14px" color="warning" class="q-mr-xs" />
              Usa absolutas en scripts, relativas en la terminal.
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 5: COMANDO LS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">02</span>
        Explorar el Directorio — <code class="cmd-inline">ls</code>
      </SectionTitle>
      <TextBlock>
        <code>ls</code> lista el contenido del directorio actual. Los <strong>flags</strong> lo potencian.
        Aquí están las variantes que usarás a diario:
      </TextBlock>

      <div class="cmd-cards-grid q-mt-lg">
        <div v-for="cmd in lsCommands" :key="cmd.name" class="cmd-card" :style="{ '--cmd-color': cmd.color }">
          <div class="cmd-card-top">
            <q-icon :name="cmd.icon" size="20px" :style="{ color: cmd.color }" />
            <code class="cmd-name">{{ cmd.name }}</code>
          </div>
          <p class="cmd-desc">{{ cmd.desc }}</p>
          <CodeBlock :hide-header="true" lang="bash" :content="cmd.example" />
          <div class="cmd-use-case">
            <q-icon name="lightbulb" size="12px" color="warning" class="q-mr-xs" />
            {{ cmd.useCase }}
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="Combinación Pro: ls -lah">
        Combina las tres banderas más útiles: <code>-l</code> (detallado) +
        <code>-a</code> (archivos ocultos) + <code>-h</code> (tamaños legibles).
        <br />
        <strong>Este es el comando que usarás el 80% del tiempo.</strong>
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 6: COMANDO CD
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">03</span>
        Moverse por el Sistema — <code class="cmd-inline">cd</code>
      </SectionTitle>
      <TextBlock>
        La terminal tiene atajos de <em>"teletransportación"</em> que te ahorrarán horas.
        Memoriza estos seis y tu velocidad se multiplicará:
      </TextBlock>

      <div class="shortcuts-grid q-mt-lg">
        <div v-for="s in cdShortcuts" :key="s.cmd" class="shortcut-card">
          <div class="sc-cmd">
            <span class="sc-dollar">$</span>
            <code class="sc-code">{{ s.cmd }}</code>
          </div>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">{{ s.example }}</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SplitBlock>
          <template #left>
            <AlertBlock type="success" title="Autocompletado con TAB">
              Escribe parte del nombre y pulsa <kbd>TAB</kbd>:
              <br /><code>cd rob</code> + TAB → <code>cd robot_ws/</code>
              <br /><br />
              Dos TABs seguidos → lista todas las coincidencias posibles.
            </AlertBlock>
          </template>
          <template #right>
            <AlertBlock type="info" title="Historial de Comandos">
              Navega el historial con <kbd>↑</kbd> <kbd>↓</kbd>.
              <br /><br />
              Presiona <kbd>Ctrl+R</kbd> y escribe parte de un comando para buscarlo — es la forma más rápida.
            </AlertBlock>
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 7: ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Cómo Resolverlos</SectionTitle>
      <TextBlock>
        Estos son los tres errores que verás con más frecuencia al empezar.
        Conocerlos de antemano te ahorra frustración.
      </TextBlock>

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
              <q-icon name="search" size="14px" class="q-mr-xs" /> <strong>Causa:</strong> {{ err.cause }}
            </div>
            <div class="err-solution">
              <q-icon name="check_circle" size="14px" class="q-mr-xs" color="positive" />
              <strong>Solución:</strong>
              <ol>
                <li v-for="step in err.steps" :key="step">{{ step }}</li>
              </ol>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 8: RETO PRÁCTICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto de Ingeniería</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Predice el resultado antes de ejecutar</div>
            <div class="challenge-subtitle">¿En qué carpeta terminarás? ¿Qué archivos se habrán creado?</div>
          </div>
          <div class="challenge-badge">NIVEL BÁSICO</div>
        </div>

        <CodeBlock
          title="Simulación de Navegación"
          lang="bash"
          content="cd ~
mkdir -p proyecto_xyz/logs
cd proyecto_xyz/logs
touch error.log debug.log
ls -lh
cd ../..
pwd
ls"
          :copyable="true"
        />

        <q-expansion-item
          icon="lightbulb"
          label="Ver respuesta (no hagas trampa)"
          header-class="answer-header"
          class="q-mt-md"
        >
          <div class="answer-body">
            <div class="answer-row">
              <span class="answer-key">📍 Directorio final:</span>
              <code>/home/alexander</code>
            </div>
            <div class="answer-row">
              <span class="answer-key">📄 Archivos creados:</span>
              <code>~/proyecto_xyz/logs/error.log</code> y <code>~/proyecto_xyz/logs/debug.log</code>
            </div>
            <div class="answer-row">
              <span class="answer-key">💡 Explicación:</span>
              <span><code>cd ../..</code> sube dos niveles: <code>logs/</code> → <code>proyecto_xyz/</code> → <code>~</code></span>
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 9: VIDEO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>
        Refuerza lo aprendido viendo estos comandos en acción en una terminal real:
      </TextBlock>

      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Navegación en Terminal Linux"
            frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" size="16px" color="info" class="q-mr-sm" />
          Video en progreso — será reemplazado con contenido del curso.
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 10: RESUMEN
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Comandos Esenciales</SectionTitle>

      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryCommands" :key="s.cmd" class="summary-card" :style="{ '--sc-color': s.color }">
          <code class="sc-cmd-code">{{ s.cmd }}</code>
          <div class="sc-cmd-desc">{{ s.desc }}</div>
          <div class="sc-cmd-example">
            <q-icon name="terminal" size="12px" class="q-mr-xs" />
            {{ s.example }}
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="¡Siguiente paso!" class="q-mt-xl">
        Ahora que dominas la navegación, el siguiente módulo te enseñará a
        <strong>crear, copiar, mover y eliminar</strong> archivos y directorios.
        ¡Prepárate para construir tu primer workspace de ROS 2!
      </AlertBlock>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import { reactive } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// ── Fact pills ──────────────────────────────────────────────────
const facts = [
  { num: '96%', label: 'de los servidores usan Linux', color: '#4ade80' },
  { num: '100%', label: 'de los robots profesionales usan Linux', color: '#60a5fa' },
  { num: '90%', label: 'del trabajo de robótica es en terminal', color: '#f59e0b' },
];

// ── Prompt legend ────────────────────────────────────────────────
const promptParts = [
  { code: 'alexander', desc: 'Nombre de usuario actual', color: '#22d3ee' },
  { code: '@', desc: 'Separador "at"', color: '#64748b' },
  { code: 'robot', desc: 'Hostname (nombre del equipo)', color: '#4ade80' },
  { code: '~/ros2_ws', desc: 'Directorio actual (~ = /home/alexander)', color: '#60a5fa' },
  { code: '$', desc: 'Usuario normal (# = root/superusuario)', color: '#facc15' },
];

// ── Filesystem tree ──────────────────────────────────────────────
const fsTree = [
  { name: '/home', color: '#60a5fa', highlight: true, badge: 'Tu workspace ROS',
    children: [{ name: '/alexander', color: '#22d3ee', badge: '~' }] },
  { name: '/opt', color: '#4ade80', highlight: true,
    children: [{ name: '/ros/jazzy', color: '#86efac', badge: 'ROS 2' }] },
  { name: '/usr/bin', color: '#f59e0b', badge: 'python3, gcc' },
  { name: '/dev', color: '#a78bfa', badge: 'dispositivos' },
  { name: '/tmp', color: '#f87171', badge: 'temporales' },
  { name: '/etc', color: '#94a3b8' },
];

// ── ls commands ──────────────────────────────────────────────────
const lsCommands = [
  {
    name: 'ls -l', icon: 'list', color: '#22d3ee',
    desc: 'Formato de lista detallada: permisos (rwx), propietario, tamaño y fecha.',
    example: '$ ls -l\n-rw-r--r-- 1 user user 1024 Jan 10 robot.py\ndrwxr-xr-x 2 user user 4096 Jan 10 src/',
    useCase: 'Verificar si un script tiene permisos de ejecución antes de lanzarlo.',
  },
  {
    name: 'ls -a', icon: 'visibility', color: '#c084fc',
    desc: 'Muestra archivos ocultos (los que empiezan con .). Aquí viven configuraciones críticas.',
    example: '$ ls -a\n.  ..  .bashrc  .gitignore  robot.py  src/',
    useCase: 'Editar .bashrc para configurar variables de entorno de ROS 2.',
  },
  {
    name: 'ls -lh', icon: 'data_usage', color: '#4ade80',
    desc: 'Tamaños en formato legible (KB, MB, GB) en lugar de bytes crudos.',
    example: '$ ls -lh\n-rw-r--r-- 1 user user 1.0K robot.py\n-rw-r--r-- 1 user user 2.3M rosbag.db3',
    useCase: 'Verificar el tamaño de un rosbag antes de transferirlo al host.',
  },
  {
    name: 'ls *.py', icon: 'filter_alt', color: '#fb923c',
    desc: 'Wildcards (*) para filtrar por extensión o patrón de nombre.',
    example: '$ ls *.py\nrobot.py  sensor_node.py  controller.py',
    useCase: 'Listar solo archivos Python en un directorio con múltiples tipos de archivo.',
  },
];

// ── cd shortcuts ─────────────────────────────────────────────────
const cdShortcuts = [
  { cmd: 'cd ..', desc: 'Subir un nivel (carpeta padre)', example: '/home/user/ws/src → /home/user/ws' },
  { cmd: 'cd ~', desc: 'Ir directamente a tu Home', example: 'Desde cualquier lugar → /home/alexander' },
  { cmd: 'cd -', desc: 'Volver al directorio anterior (como ← del navegador)', example: 'Alterna entre /opt/ros ↔ ~/ros2_ws' },
  { cmd: 'cd', desc: 'Sin argumentos: también va a Home', example: 'Equivalente a cd ~' },
  { cmd: 'cd /', desc: 'Ir a la raíz del sistema', example: 'El punto de partida de todo el filesystem' },
  { cmd: 'cd ../..', desc: 'Subir dos niveles de una sola vez', example: '/home/user/ws/src/pkg → /home/user/ws' },
];

// ── Common errors ────────────────────────────────────────────────
const commonErrors = reactive([
  {
    msg: 'bash: cd: directorio: No such file or directory',
    summary: 'El directorio no existe o está mal escrito',
    cause: 'Escribiste mal el nombre del directorio o no existe en la ruta indicada.',
    steps: [
      'Usa ls para ver qué directorios existen en el directorio actual.',
      'Usa TAB para autocompletar y evitar errores de tipeo.',
      'Recuerda que Linux es case-sensitive: /Home ≠ /home.',
    ],
    open: false,
  },
  {
    msg: 'ls: cannot access "archivo": Permission denied',
    summary: 'No tienes permisos para acceder a ese archivo o directorio',
    cause: 'El archivo pertenece a otro usuario o root, y no tienes permisos de lectura.',
    steps: [
      'Verifica permisos con ls -l — busca los símbolos rwx.',
      'Si es tu archivo, otorga permisos con chmod +r archivo.',
      'Si es del sistema, usa sudo ls archivo (solo cuando sea necesario).',
    ],
    open: false,
  },
  {
    msg: 'Me perdí en el filesystem después de varios cd',
    summary: 'No sabes en qué directorio estás',
    cause: 'Navegaste con varias rutas relativas y perdiste el hilo de dónde estás.',
    steps: [
      'Ejecuta pwd para ver tu ubicación exacta.',
      'Ejecuta cd ~ para regresar inmediatamente a tu Home.',
      'Usa cd - para volver al directorio en el que estabas antes.',
    ],
    open: false,
  },
]);

// ── Summary commands ─────────────────────────────────────────────
const summaryCommands = [
  { cmd: 'pwd', desc: 'Muestra tu ubicación actual en el filesystem', example: 'pwd → /home/alexander', color: '#22d3ee' },
  { cmd: 'ls -lah', desc: 'Lista archivos: detallado + ocultos + legible', example: 'ls -lah → detallado', color: '#c084fc' },
  { cmd: 'cd ~', desc: 'Ir a tu carpeta Home desde cualquier lugar', example: 'cd ~ → /home/alexander', color: '#4ade80' },
  { cmd: 'cd ..', desc: 'Subir un nivel al directorio padre', example: 'cd .. → nivel anterior', color: '#fb923c' },
  { cmd: 'cd -', desc: 'Volver al directorio anterior (toggle)', example: 'cd - → alterna dirs', color: '#f59e0b' },
  { cmd: 'TAB', desc: 'Autocompletado — usa siempre para evitar errores', example: 'cd rob + TAB', color: '#f87171' },
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
  font-family: 'Fira Code', monospace;
}

/* ══════════════════════════════════════════
   SECTION TITLE ENHANCEMENTS
══════════════════════════════════════════ */
.cmd-badge {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 28px; height: 28px;
  border-radius: 8px;
  font-size: 0.75rem;
  font-weight: 800;
  margin-right: 8px;
  vertical-align: middle;
}
.cmd-badge.cyan   { background: rgba(34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }

.cmd-inline {
  font-family: 'Fira Code', monospace;
  font-size: 0.95em;
  background: none;
  color: var(--text-code);
  padding: 0;
}

/* ══════════════════════════════════════════
   FACT PILLS
══════════════════════════════════════════ */
.fact-pills {
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
  margin-bottom: 1.5rem;
}
.fact-pill {
  display: flex;
  align-items: center;
  gap: 10px;
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 999px;
  padding: 8px 18px;
  transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fact-num {
  font-weight: 900;
  font-size: 1.1rem;
  font-variant-numeric: tabular-nums;
}
.fact-label {
  font-size: 0.82rem;
  color: var(--text-secondary);
}

/* ══════════════════════════════════════════
   TERMINAL ANATOMY
══════════════════════════════════════════ */
.terminal-anatomy-card {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-medium);
  border-radius: 16px;
  overflow: hidden;
  box-shadow: var(--shadow-md);
}

.term-chrome {
  background: var(--bg-deep);
  padding: 10px 16px;
  display: flex;
  align-items: center;
  gap: 12px;
  border-bottom: 1px solid var(--border-subtle);
}
.term-dots { display: flex; gap: 6px; }
.term-dot {
  width: 12px; height: 12px;
  border-radius: 50%;
}
.term-dot.red    { background: #ef4444; }
.term-dot.yellow { background: #f59e0b; }
.term-dot.green  { background: #22c55e; }
.term-title {
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  color: var(--text-muted);
}

.term-body {
  padding: 1.75rem 2rem;
  background: var(--bg-surface-solid);
}

.prompt-line {
  display: flex;
  align-items: center;
  gap: 2px;
  font-family: 'Fira Code', monospace;
  font-size: 1.3rem;
  flex-wrap: wrap;
}

.prompt-part {
  position: relative;
  cursor: default;
  padding: 2px 4px;
  border-radius: 4px;
  transition: background .2s;
}
.prompt-part:hover { background: rgba(255,255,255,0.06); }
.prompt-part:hover .pp-tooltip { opacity: 1; pointer-events: auto; transform: translateY(0); }

.pp-user   { color: #22d3ee; font-weight: 700; }
.pp-host   { color: #4ade80; font-weight: 700; }
.pp-path   { color: #60a5fa; }
.pp-symbol { color: #facc15; padding: 0 8px; }
.pp-sep    { color: #64748b; }
.pp-cursor { color: var(--text-primary); animation: blink 1s step-end infinite; }

@keyframes blink { 50% { opacity: 0; } }

.pp-tooltip {
  position: absolute;
  top: calc(100% + 8px);
  left: 50%;
  transform: translateX(-50%) translateY(4px);
  background: var(--bg-deep);
  border: 1px solid var(--border-medium);
  border-radius: 8px;
  padding: 8px 12px;
  font-size: 0.78rem;
  color: var(--text-primary);
  white-space: nowrap;
  z-index: 10;
  opacity: 0;
  pointer-events: none;
  transition: all .2s;
  box-shadow: var(--shadow-md);
  font-family: -apple-system, sans-serif;
  line-height: 1.4;
}

.prompt-legend {
  background: var(--bg-deep);
  border-top: 1px solid var(--border-subtle);
  padding: 1.25rem 2rem;
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
  gap: 1rem;
}
.leg-item {
  display: flex;
  align-items: flex-start;
  gap: 10px;
}
.leg-dot {
  width: 10px; height: 10px;
  border-radius: 50%;
  margin-top: 4px;
  flex-shrink: 0;
}
.leg-code {
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  font-weight: 700;
  margin-bottom: 2px;
}
.leg-desc {
  font-size: 0.8rem;
  color: var(--text-secondary);
}

/* ══════════════════════════════════════════
   FILESYSTEM TREE
══════════════════════════════════════════ */
.fs-tree-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  overflow: hidden;
}
.fs-tree-header {
  background: var(--bg-deep);
  border-bottom: 1px solid var(--border-subtle);
  padding: 12px 16px;
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 0.85rem;
  font-weight: 600;
  color: var(--text-secondary);
}
.fs-tree {
  padding: 1.25rem 1rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}
.fs-node {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 5px 10px;
  border-radius: 7px;
  transition: background .2s;
  cursor: default;
}
.fs-node:hover { background: var(--bg-surface-hover); }
.fs-node.root-node {
  background: rgba(234,179,8,.1);
  border-left: 3px solid #eab308;
  margin-bottom: 4px;
}
.fs-node.highlight { background: rgba(34,211,238,.07); }
.fn-name { color: var(--text-secondary); }
.fn-name.root { color: var(--text-primary); font-weight: 700; }
.fn-hint { font-size: 0.75em; color: var(--text-muted); }
.fn-badge {
  margin-left: auto;
  font-size: 0.7rem;
  background: var(--bg-surface-hover);
  border: 1px solid var(--border-subtle);
  color: var(--text-muted);
  padding: 1px 7px;
  border-radius: 999px;
  white-space: nowrap;
}
.fs-children {
  margin-left: 20px;
  border-left: 2px dashed var(--border-medium);
  padding-left: 12px;
}
.fs-children.nested { margin-top: 2px; }
.fs-branch { margin: 4px 0; }
.fs-node.child { padding: 3px 8px; }

/* Alert list */
.dir-list {
  list-style: none;
  padding: 0; margin: 0;
  display: flex; flex-direction: column; gap: 8px;
}
.dir-list li {
  display: flex;
  align-items: baseline;
  gap: 8px;
  font-size: 0.9rem;
  color: var(--text-secondary);
}
.dir-list code { flex-shrink: 0; }
.dir-list span { line-height: 1.4; }

/* ══════════════════════════════════════════
   CONCEPT CARD (pwd)
══════════════════════════════════════════ */
.concept-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 14px;
  height: 100%;
}
.concept-header {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 4px;
}
.concept-title {
  font-size: 1.05rem;
  font-weight: 700;
  color: var(--text-primary);
}
.concept-item {
  background: var(--bg-surface-hover);
  border-radius: 10px;
  padding: 1rem;
  border-left: 3px solid var(--border-medium);
}
.concept-item.accent-green { border-left-color: #4ade80; }
.concept-item.accent-blue  { border-left-color: #60a5fa; }
.ci-label {
  font-size: 0.75rem;
  font-weight: 800;
  letter-spacing: .08em;
  margin-bottom: 6px;
}
.ci-desc {
  font-size: 0.88rem;
  color: var(--text-secondary);
  margin: 0 0 8px 0;
  line-height: 1.5;
}
.ci-example {
  font-family: 'Fira Code', monospace;
  font-size: 0.82rem;
  color: var(--text-code);
  background: var(--bg-code);
  padding: 6px 10px;
  border-radius: 6px;
  word-break: break-all;
}
.concept-tip {
  display: flex;
  align-items: center;
  font-size: 0.82rem;
  color: var(--text-muted);
  background: var(--bg-surface-hover);
  border-radius: 8px;
  padding: 8px 12px;
}

/* ══════════════════════════════════════════
   COMMAND CARDS (ls)
══════════════════════════════════════════ */
.cmd-cards-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 16px;
}
.cmd-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--cmd-color, var(--border-medium));
  border-radius: 14px;
  padding: 1.25rem;
  display: flex;
  flex-direction: column;
  gap: 12px;
  transition: all .25s;
  min-width: 0; /* prevent grid blowout */
  overflow: hidden;
}
/* Evita que el código dentro del CodeBlock desborde la card */
.cmd-card :deep(pre),
.cmd-card :deep(code),
.cmd-card :deep(.q-markup-table) {
  overflow-x: auto;
  max-width: 100%;
  white-space: pre;
}
.cmd-card:hover {
  transform: translateY(-4px);
  box-shadow: var(--shadow-md);
  border-color: var(--cmd-color, var(--border-hover));
}
.cmd-card-top {
  display: flex;
  align-items: center;
  gap: 10px;
}
.cmd-name {
  font-family: 'Fira Code', monospace;
  font-size: 1.05rem;
  font-weight: 700;
  color: var(--cmd-color, var(--text-code));
  background: none;
  padding: 0;
}
.cmd-desc {
  font-size: 0.9rem;
  color: var(--text-secondary);
  line-height: 1.55;
  margin: 0;
  flex: 1;
}
.cmd-use-case {
  display: flex;
  align-items: flex-start;
  gap: 4px;
  font-size: 0.82rem;
  color: var(--text-muted);
  background: var(--bg-surface-hover);
  border-radius: 7px;
  padding: 8px 10px;
  line-height: 1.4;
}

/* ══════════════════════════════════════════
   SHORTCUTS GRID (cd)
══════════════════════════════════════════ */
.shortcuts-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
  gap: 12px;
}
.shortcut-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 12px;
  padding: 1.1rem 1.25rem;
  transition: all .25s;
}
.shortcut-card:hover {
  transform: translateY(-2px);
  box-shadow: var(--shadow-sm);
  border-color: var(--border-hover);
}
.sc-cmd {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 8px;
}
.sc-dollar {
  font-family: 'Fira Code', monospace;
  color: var(--text-muted);
  font-size: 1rem;
}
.sc-code {
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  font-weight: 700;
  color: var(--text-code);
  background: none;
  padding: 0;
}
.sc-desc {
  font-size: 0.88rem;
  color: var(--text-secondary);
  margin-bottom: 8px;
  line-height: 1.4;
}
.sc-example {
  font-family: 'Fira Code', monospace;
  font-size: 0.78rem;
  color: var(--text-muted);
  background: var(--bg-surface-hover);
  padding: 5px 10px;
  border-radius: 6px;
}

/* ══════════════════════════════════════════
   ERROR LIST
══════════════════════════════════════════ */
.error-list {
  display: flex;
  flex-direction: column;
  gap: 10px;
}
.error-item {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-left: 3px solid #ef4444;
  border-radius: 12px;
  overflow: hidden;
}
.err-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 1rem 1.25rem;
  cursor: pointer;
  gap: 12px;
  transition: background .2s;
}
.err-header:hover { background: var(--bg-surface-hover); }
.err-left {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  min-width: 0;
}
.err-number {
  min-width: 28px; min-height: 28px;
  width: 28px; height: 28px;
  border-radius: 50%;
  background: rgba(239,68,68,.15);
  color: #ef4444;
  font-size: 0.8rem;
  font-weight: 800;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}
.err-msg {
  font-size: 0.88rem;
  display: block;
  margin-bottom: 4px;
  color: #ef4444;
  background: none;
  padding: 0;
  word-break: break-all;
}
.err-summary {
  font-size: 0.82rem;
  color: var(--text-muted);
}
.err-body {
  padding: 1rem 1.5rem 1.25rem 1.5rem;
  border-top: 1px solid var(--border-subtle);
  display: flex;
  flex-direction: column;
  gap: 10px;
}
.err-cause, .err-solution {
  font-size: 0.9rem;
  color: var(--text-secondary);
  display: flex;
  align-items: flex-start;
  gap: 6px;
  line-height: 1.5;
}
.err-solution ol {
  margin: 4px 0 0 16px;
  padding: 0;
}
.err-solution li { margin-bottom: 4px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX
══════════════════════════════════════════ */
.challenge-box {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 20px;
  padding: 1.75rem;
  border-top: 3px solid #f59e0b;
}
.challenge-header {
  display: flex;
  align-items: flex-start;
  gap: 1rem;
  margin-bottom: 1.25rem;
  flex-wrap: wrap;
}
.challenge-icon {
  width: 52px; height: 52px;
  background: rgba(245,158,11,.15);
  border-radius: 14px;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}
.challenge-title {
  font-size: 1.05rem;
  font-weight: 700;
  color: var(--text-primary);
  margin-bottom: 4px;
}
.challenge-subtitle {
  font-size: 0.9rem;
  color: var(--text-secondary);
}
.challenge-badge {
  margin-left: auto;
  font-size: 0.72rem;
  font-weight: 800;
  letter-spacing: .07em;
  padding: 4px 12px;
  border-radius: 999px;
  background: rgba(34,197,94,.12);
  color: #22c55e;
  border: 1px solid rgba(34,197,94,.3);
  white-space: nowrap;
}

:deep(.answer-header) {
  background: rgba(34,197,94,.08);
  border: 1px solid rgba(34,197,94,.25);
  border-radius: 10px;
  color: #22c55e;
}

.answer-body {
  background: var(--bg-surface-hover);
  padding: 1.25rem 1.5rem;
  border-radius: 0 0 10px 10px;
  display: flex;
  flex-direction: column;
  gap: 10px;
}
.answer-row {
  display: flex;
  align-items: baseline;
  flex-wrap: wrap;
  gap: 8px;
  font-size: 0.9rem;
  color: var(--text-secondary);
}
.answer-key {
  font-weight: 600;
  color: var(--text-primary);
  white-space: nowrap;
}

/* ══════════════════════════════════════════
   VIDEO CARD
══════════════════════════════════════════ */
.video-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  padding: 1.25rem;
  overflow: hidden;
}
.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 10px;
  background: #000;
}
.video-wrapper iframe {
  position: absolute;
  top: 0; left: 0;
  width: 100%; height: 100%;
}
.video-caption {
  display: flex;
  align-items: center;
  margin-top: 12px;
  font-size: 0.82rem;
  color: var(--text-muted);
  padding: 8px 12px;
  background: var(--bg-surface-hover);
  border-radius: 8px;
}

/* ══════════════════════════════════════════
   SUMMARY GRID
══════════════════════════════════════════ */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 14px;
}
.summary-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sc-color, var(--border-medium));
  border-radius: 12px;
  padding: 1rem 1.25rem;
  transition: all .25s;
}
.summary-card:hover {
  transform: translateY(-3px);
  box-shadow: var(--shadow-sm);
}
.sc-cmd-code {
  display: block;
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  font-weight: 700;
  color: var(--sc-color, var(--text-code));
  background: none;
  padding: 0;
  margin-bottom: 6px;
}
.sc-cmd-desc {
  font-size: 0.88rem;
  color: var(--text-secondary);
  margin-bottom: 8px;
  line-height: 1.45;
}
.sc-cmd-example {
  display: flex;
  align-items: center;
  font-family: 'Fira Code', monospace;
  font-size: 0.78rem;
  color: var(--text-muted);
}

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */

/* Tablet: grids a 2 columnas */
@media (max-width: 900px) {
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
}

/* Móvil: todo a 1 columna */
@media (max-width: 768px) {
  .fact-pills { flex-direction: column; gap: 8px; }
  .fact-pill  { border-radius: 12px; }

  .prompt-line   { font-size: 0.9rem; flex-wrap: wrap; gap: 2px; }
  .prompt-legend { grid-template-columns: 1fr 1fr; padding: 1rem; }

  .cmd-cards-grid { grid-template-columns: 1fr; }
  .shortcuts-grid { grid-template-columns: 1fr 1fr; }
  .summary-grid   { grid-template-columns: 1fr 1fr; }

  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
  .terminal-anatomy-card { overflow-x: auto; }

  .fs-tree-card { font-size: 0.85rem; }
  .fn-badge { display: none; } /* libera espacio en móvil */
}

/* Móvil pequeño */
@media (max-width: 480px) {
  .prompt-line   { font-size: 0.78rem; gap: 1px; }
  .prompt-legend { grid-template-columns: 1fr; }
  .shortcuts-grid { grid-template-columns: 1fr; }
  .summary-grid   { grid-template-columns: 1fr; }
  .err-left { flex-direction: column; }
  .challenge-header { gap: 0.75rem; }
}
</style>
