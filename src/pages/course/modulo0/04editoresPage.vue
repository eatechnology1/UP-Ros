<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         SECCIÓN 1: CONTEXTO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        El código no se escribe en el aire. Necesitas un editor que entienda Python, C++, YAML
        — y que funcione tanto en tu laptop como en un robot sin pantalla conectado por SSH
        a 10,000 km de distancia.<br /><br />
        Esta lección cubre <strong>tres herramientas</strong> para contextos distintos:
        <strong>nano</strong> para emergencias, <strong>VS Code</strong> para desarrollo diario,
        y <strong>vim</strong> para los valientes.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 2: LA TRINIDAD
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        La Trinidad de Editores
      </SectionTitle>

      <div class="editors-grid q-mt-lg">
        <div v-for="ed in editors" :key="ed.name" class="editor-card"
          :style="{ '--ed-color': ed.color }">
          <div class="ed-top">
            <div class="ed-emoji">{{ ed.emoji }}</div>
            <div class="ed-badge" :style="{ background: ed.color + '18', color: ed.color }">
              {{ ed.badge }}
            </div>
          </div>
          <div class="ed-name">{{ ed.name }}</div>
          <div class="ed-tagline">{{ ed.tagline }}</div>
          <div class="ed-stats">
            <div class="ed-stat">
              <span class="es-label">Curva aprendizaje</span>
              <span class="es-value">{{ ed.curve }}</span>
            </div>
            <div class="ed-stat">
              <span class="es-label">Uso principal</span>
              <span class="es-value">{{ ed.use }}</span>
            </div>
          </div>
          <div class="ed-when">
            <q-icon name="lightbulb" size="14px" :style="{ color: ed.color }" class="q-mr-xs" />
            {{ ed.when }}
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="Regla del 95/5" class="q-mt-lg">
        <strong>95% del tiempo:</strong> VS Code — desarrollo, debugging, Git, IntelliSense.<br />
        <strong>5% del tiempo:</strong> nano — SSH a robots remotos, configs del sistema, emergencias.<br />
        <strong>vim:</strong> opcional, para cuando quieras impresionar a alguien.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 3: NANO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">02</span>
        GNU nano — Tu Navaja Suiza
      </SectionTitle>

      <TextBlock>
        <code>nano</code> es el editor que <strong>siempre funciona</strong>. No importa si estás
        en una Raspberry Pi sin GUI, en un contenedor Docker, o conectado por SSH a un robot.
        Si hay terminal, hay nano.
      </TextBlock>

      <!-- Mock terminal nano -->
      <div class="nano-terminal q-mt-lg">
        <div class="nt-chrome">
          <div class="nt-dots">
            <span class="dot red"></span>
            <span class="dot yellow"></span>
            <span class="dot green"></span>
          </div>
          <span class="nt-title">nano 6.2 — robot_config.yaml</span>
        </div>
        <div class="nt-body">
          <div class="nt-line" v-for="l in nanoLines" :key="l.num">
            <span class="nl-num">{{ l.num }}</span>
            <span class="nl-code" v-html="l.code"></span>
          </div>
        </div>
        <div class="nt-statusbar">
          <span class="ns-left">Modified</span>
          <span class="ns-center">robot_config.yaml</span>
          <span class="ns-right">ln 3, col 14</span>
        </div>
        <div class="nt-shortcuts">
          <div class="ns-item" v-for="s in nanoShortcuts" :key="s.key">
            <kbd>{{ s.key }}</kbd>
            <span>{{ s.label }}</span>
          </div>
        </div>
      </div>

      <!-- Workflow steps -->
      <div class="workflow q-mt-xl">
        <div class="wf-title">Flujo de Trabajo con nano</div>
        <div class="wf-steps">
          <div v-for="(step, i) in nanoWorkflow" :key="i" class="wf-step">
            <div class="wfs-num">{{ i + 1 }}</div>
            <div class="wfs-body">
              <div class="wfs-title">{{ step.title }}</div>
              <div class="wfs-desc" v-html="step.desc"></div>
              <CodeBlock v-if="step.code" :hide-header="true" lang="bash" :content="step.code" />
            </div>
            <div v-if="i < nanoWorkflow.length - 1" class="wfs-arrow">↓</div>
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="Pro tip: crear y editar en un comando" class="q-mt-lg">
        Si el archivo no existe, nano lo crea automáticamente al guardarlo:<br />
        <code>nano nuevo_nodo.py</code> → escribe código → <kbd>Ctrl+O</kbd> → <kbd>Enter</kbd>
        → <kbd>Ctrl+X</kbd>
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 4: VS CODE
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        VS Code — Tu IDE de Batalla para ROS 2
      </SectionTitle>

      <TextBlock>
        Visual Studio Code es <strong>el estándar de facto</strong> para desarrollo ROS 2.
        Gratis, potente, y con extensiones para todo. Instálalo y configúralo una vez — te
        acompaña durante toda la carrera.
      </TextBlock>

      <!-- Extensions -->
      <div class="vsc-section q-mt-lg">
        <div class="vsc-section-title">
          <q-icon name="extension" size="18px" color="primary" />
          Extensiones Esenciales
        </div>
        <div class="ext-grid">
          <div v-for="ext in vscExtensions" :key="ext.name" class="ext-card"
            :style="{ '--ext-color': ext.color }">
            <div class="ext-icon-wrap" :style="{ background: ext.color + '18' }">
              <q-icon :name="ext.icon" size="20px" :style="{ color: ext.color }" />
            </div>
            <div class="ext-info">
              <div class="ext-name">{{ ext.name }}</div>
              <div class="ext-author">{{ ext.author }}</div>
              <div class="ext-desc">{{ ext.desc }}</div>
            </div>
          </div>
        </div>
      </div>

      <!-- Settings JSON -->
      <div class="vsc-section q-mt-xl">
        <div class="vsc-section-title">
          <q-icon name="settings" size="18px" color="warning" />
          Configuración Recomendada para ROS 2
        </div>
        <CodeBlock title="settings.json — Ctrl+Shift+P → 'Open Settings JSON'" lang="json"
          content='{
  "python.defaultInterpreterPath": "/usr/bin/python3",
  "python.autoComplete.extraPaths": [
    "/opt/ros/jazzy/lib/python3.12/site-packages"
  ],
  "files.associations": {
    "*.launch": "xml",
    "*.world": "xml",
    "*.urdf": "xml",
    "*.xacro": "xml"
  },
  "editor.formatOnSave": true,
  "editor.rulers": [100],
  "editor.tabSize": 4
}' :copyable="true" />
      </div>

      <!-- Keyboard shortcuts -->
      <div class="vsc-section q-mt-xl">
        <div class="vsc-section-title">
          <q-icon name="keyboard" size="18px" color="positive" />
          Shortcuts Esenciales
        </div>
        <div class="kbd-grid">
          <div v-for="k in vscShortcuts" :key="k.keys" class="kbd-card">
            <div class="kc-keys">
              <kbd v-for="(key, ki) in k.keys.split('+')" :key="ki">{{ key }}</kbd>
            </div>
            <div class="kc-action">{{ k.action }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 5: REMOTE SSH
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        VS Code Remote SSH — El Puente Mágico
      </SectionTitle>

      <TextBlock>
        Imagina editar código en un robot físico (Raspberry Pi, Jetson) con toda la comodidad
        de VS Code en tu laptop. <strong>Remote SSH</strong> hace exactamente eso: tu editor
        local, el código en el robot.
      </TextBlock>

      <!-- SSH Diagram -->
      <div class="ssh-diagram q-mt-lg">
        <div class="ssh-node laptop">
          <div class="ssh-node-icon">💻</div>
          <div class="ssh-node-title">Tu Laptop</div>
          <div class="ssh-node-sub">VS Code con GUI</div>
          <div class="ssh-node-tag" style="--nt-color:#60a5fa">edición</div>
        </div>

        <div class="ssh-tunnel">
          <div class="ssh-line"></div>
          <div class="ssh-label">SSH Puerto 22</div>
          <div class="ssh-line"></div>
        </div>

        <div class="ssh-node robot">
          <div class="ssh-node-icon">🤖</div>
          <div class="ssh-node-title">Robot (RPi / Jetson)</div>
          <div class="ssh-node-sub">Código corriendo aquí</div>
          <div class="ssh-node-tag" style="--nt-color:#4ade80">ejecución</div>
        </div>
      </div>

      <!-- Steps -->
      <div class="connect-steps q-mt-lg">
        <div class="cs-title">
          <q-icon name="cable" size="16px" color="primary" />
          Cómo Conectarte en 4 Pasos
        </div>
        <div class="cs-list">
          <div v-for="(step, i) in sshSteps" :key="i" class="cs-step">
            <div class="cs-num">{{ i + 1 }}</div>
            <div class="cs-body" v-html="step"></div>
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="Ventaja clave" class="q-mt-md">
        Con Remote SSH usas <strong>debugging, IntelliSense, terminal integrada y Git</strong>
        directamente en el robot. Todo el poder de VS Code, el código vive en el hardware.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 6: VIM (OPCIONAL)
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Vim — El Camino del Guerrero
        <span class="optional-tag">OPCIONAL</span>
      </SectionTitle>

      <TextBlock>
        <code>vim</code> es legendario por su eficiencia… y por su curva de aprendizaje vertical.
        <strong>No es obligatorio para este curso</strong>, pero si entras por accidente,
        necesitas saber cómo salir.
      </TextBlock>

      <!-- Modes diagram -->
      <div class="vim-modes q-mt-lg">
        <div class="vm-title">Los 3 Modos de Vim</div>
        <div class="vm-flow">
          <div v-for="mode in vimModes" :key="mode.name" class="vm-mode"
            :style="{ '--vm-color': mode.color }">
            <div class="vmm-name" :style="{ color: mode.color }">{{ mode.name }}</div>
            <div class="vmm-desc">{{ mode.desc }}</div>
            <div class="vmm-key">
              <kbd>{{ mode.enter }}</kbd>
              <span>para entrar</span>
            </div>
          </div>
        </div>
      </div>

      <!-- Survival kit -->
      <div class="survival-kit q-mt-xl">
        <div class="sk-header">
          <q-icon name="sos" size="20px" color="negative" />
          Kit de Supervivencia Vim
        </div>
        <div class="sk-grid">
          <div v-for="s in vimSurvival" :key="s.cmd" class="sk-card"
            :class="{ 'sk-escape': s.escape }">
            <div class="sk-cmd">{{ s.cmd }}</div>
            <div class="sk-desc">{{ s.desc }}</div>
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="La leyenda urbana" class="q-mt-lg">
        Se dice que hay programadores atrapados en Vim desde 1995 porque no saben cómo salir.
        La respuesta es <code>:q!</code> (dos puntos, q, exclamación, Enter).<br /><br />
        O más elegante: <code>:wq</code> para guardar y salir.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 7: CASOS DE USO REALES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Casos de Uso Reales en ROS 2</SectionTitle>

      <div class="scenarios-grid q-mt-lg">
        <div v-for="sc in scenarios" :key="sc.title" class="scenario-card"
          :style="{ '--sc-color': sc.color }">
          <div class="scc-header">
            <q-icon :name="sc.icon" size="22px" :style="{ color: sc.color }" />
            <div class="scc-title">{{ sc.title }}</div>
          </div>
          <div class="scc-situation">{{ sc.situation }}</div>
          <div class="scc-solution">{{ sc.solution }}</div>
          <div class="scc-tool" :style="{ background: sc.color + '18', color: sc.color }">
            <q-icon name="build" size="13px" class="q-mr-xs" />
            {{ sc.tool }}
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 8: RETO PRÁCTICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto de Práctica — Crear un Nodo con nano</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Escribe tu primer nodo Python desde la terminal</div>
            <div class="challenge-subtitle">
              Sin VS Code. Solo la terminal y nano. Así trabajo un ingeniero de campo.
            </div>
          </div>
          <div class="challenge-badge">NIVEL BÁSICO</div>
        </div>

        <CodeBlock title="Reto: Nodo Mínimo con nano" lang="bash"
          content="# 1. Crea el archivo (nano lo crea al guardar)
nano ~/hola_robot.py

# 2. Escribe este código en nano:
# #!/usr/bin/env python3
# print('Hola desde ROS 2 Academy!')
# print('Editor: nano desde la terminal')

# 3. Guarda: Ctrl+O → Enter
# 4. Sale: Ctrl+X

# 5. Hazlo ejecutable
chmod +x ~/hola_robot.py

# 6. Ejecútalo
python3 ~/hola_robot.py

# 7. BONUS: ábrelo en VS Code y compáralo
code ~/hola_robot.py" :copyable="true" />

        <q-expansion-item icon="lightbulb" label="Ver resultado esperado"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div class="answer-row">
              <span class="answer-key">Terminal muestra:</span>
              <code>Hola desde ROS 2 Academy!</code>
            </div>
            <div class="answer-row">
              <span class="answer-key">ls -l muestra:</span>
              <code>-rwxr-xr-x ... hola_robot.py</code>
            </div>
            <div class="answer-row">
              <span class="answer-key">VS Code abre:</span>
              <span>el mismo archivo con syntax highlighting Python</span>
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
      <TextBlock>Mira nano, VS Code y la conexión SSH en acción:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Editores de Texto en Linux" frameborder="0"
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
         SECCIÓN 10: RESUMEN
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Cuándo Usar Cada Editor</SectionTitle>

      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.editor" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.editor }}</code>
          <div class="sc-desc">{{ s.when }}</div>
          <div class="sc-example">
            <q-icon name="terminal" size="12px" class="q-mr-xs" />{{ s.cmd }}
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="¡Siguiente paso!" class="q-mt-xl">
        Ya tienes las herramientas de edición. El siguiente módulo cubre
        <strong>Variables de Entorno</strong> — cómo ROS 2 usa variables como
        <code>ROS_DOMAIN_ID</code> y <code>RMW_IMPLEMENTATION</code> para configurar
        el sistema sin tocar código.
      </AlertBlock>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';

// ── Fact pills ──────────────────────────────────────────────────
const facts = [
  { icon: '🛡️', label: 'nano — siempre disponible en cualquier sistema' },
  { icon: '💻', label: 'VS Code — 95% del tiempo de desarrollo' },
  { icon: '💀', label: 'vim — para los valientes y los atrapados' },
];

// ── Editors ─────────────────────────────────────────────────────
const editors = [
  {
    name: 'GNU nano', emoji: '🛡️', tagline: '"El salvavidas"',
    badge: 'EMERGENCIAS', color: '#4ade80',
    curve: '30 segundos', use: 'Ediciones rápidas por SSH',
    when: 'Editar .bashrc, configurar redes, arreglar archivos en robots remotos o sin GUI.',
  },
  {
    name: 'VS Code', emoji: '💻', tagline: '"El taller completo"',
    badge: 'DESARROLLO', color: '#60a5fa',
    curve: '1 semana', use: 'Desarrollo de paquetes ROS 2',
    when: 'Escribir nodos Python/C++, debugging, Git, trabajo local. El 95% del tiempo.',
  },
  {
    name: 'Vim', emoji: '💀', tagline: '"El legendario"',
    badge: 'OPCIONAL', color: '#f87171',
    curve: '6 meses — ∞', use: 'Edición ultra-rápida (expertos)',
    when: 'Cuando quieras impresionar (o torturarte). No obligatorio para este curso.',
  },
];

// ── Nano mock ──────────────────────────────────────────────────
const nanoLines = [
  { num: 1, code: '<span style="color:#94a3b8">ros_domain_id</span><span style="color:#94a3b8">: </span><span style="color:#fbbf24">30</span>' },
  { num: 2, code: '<span style="color:#94a3b8">node_name</span><span style="color:#94a3b8">: </span><span style="color:#4ade80">"control_node"</span>' },
  { num: 3, code: '<span style="color:#94a3b8">max_speed</span><span style="color:#94a3b8">: </span><span style="color:#fbbf24">1.5</span><span class="cursor">█</span>' },
  { num: 4, code: '<span style="color:#94a3b8">max_angular_vel</span><span style="color:#94a3b8">: </span><span style="color:#fbbf24">0.8</span>' },
];

const nanoShortcuts = [
  { key: '^G', label: 'Ayuda' },
  { key: '^O', label: 'Guardar' },
  { key: '^X', label: 'Salir' },
  { key: '^K', label: 'Cortar línea' },
  { key: '^U', label: 'Pegar' },
  { key: '^W', label: 'Buscar' },
];

const nanoWorkflow = [
  {
    title: 'Abrir (o crear) archivo',
    desc: 'Si no existe, nano lo crea al guardar.',
    code: 'nano robot_config.yaml',
  },
  {
    title: 'Editar',
    desc: 'Escribe normalmente. Las flechas funcionan como esperas. El cursor <span style="color:#4ade80">█</span> indica posición.',
    code: undefined,
  },
  {
    title: 'Guardar',
    desc: '<kbd>Ctrl+O</kbd> → aparece "File Name to Write" → <kbd>Enter</kbd> para confirmar.',
    code: undefined,
  },
  {
    title: 'Salir',
    desc: '<kbd>Ctrl+X</kbd>. Si no guardaste, te pregunta si quieres hacerlo.',
    code: undefined,
  },
];

// ── VS Code ────────────────────────────────────────────────────
const vscExtensions = [
  { name: 'Python',       author: 'Microsoft',  color: '#fbbf24', icon: 'code',
    desc: 'IntelliSense, debugging, linting para nodos Python' },
  { name: 'C/C++',        author: 'Microsoft',  color: '#60a5fa', icon: 'memory',
    desc: 'Autocompletado para nodos C++, integración CMake' },
  { name: 'ROS',          author: 'Microsoft',  color: '#4ade80', icon: 'precision_manufacturing',
    desc: 'Syntax para .msg, .srv, .action, launch files' },
  { name: 'Remote - SSH', author: 'Microsoft',  color: '#c084fc', icon: 'wifi',
    desc: 'Editar código en robots remotos como si fueran locales' },
  { name: 'YAML',         author: 'Red Hat',    color: '#fb923c', icon: 'description',
    desc: 'Validación de sintaxis para archivos de configuración' },
  { name: 'GitLens',      author: 'GitKraken',  color: '#f472b6', icon: 'merge',
    desc: 'Superpoderes para Git: blame, history, comparaciones' },
];

const vscShortcuts = [
  { keys: 'Ctrl+Shift+P', action: 'Command Palette — comando más importante' },
  { keys: 'Ctrl+P',       action: 'Abrir archivo por nombre rápidamente' },
  { keys: 'Ctrl+`',       action: 'Abrir terminal integrada' },
  { keys: 'Ctrl+Shift+E', action: 'Explorador de archivos' },
  { keys: 'F5',           action: 'Ejecutar debugger' },
  { keys: 'Ctrl+Shift+G', action: 'Git panel' },
  { keys: 'Alt+Shift+F',  action: 'Formatear documento' },
  { keys: 'Ctrl+/',       action: 'Comentar / descomentar línea' },
];

// ── Remote SSH ─────────────────────────────────────────────────
const sshSteps = [
  'Instala la extensión <strong>"Remote - SSH"</strong> en VS Code',
  'Presiona <kbd>F1</kbd> → escribe <strong>"Remote-SSH: Connect to Host"</strong>',
  'Escribe la dirección: <code>usuario@192.168.1.100</code> (IP de tu robot)',
  'Ingresa contraseña → ¡Listo! Editas archivos en el robot como si fueran locales',
];

// ── Vim ────────────────────────────────────────────────────────
const vimModes = [
  { name: 'NORMAL', color: '#60a5fa',  enter: 'ESC',
    desc: 'Modo por defecto. Navegas y das comandos. No escribes texto.' },
  { name: 'INSERT', color: '#4ade80',  enter: 'i',
    desc: 'Modo de escritura. El texto que escribes aparece en el archivo.' },
  { name: 'COMMAND', color: '#fbbf24', enter: ':',
    desc: 'Modo de comandos. Guardar (:w), salir (:q), buscar (/pattern).' },
];

const vimSurvival = [
  { cmd: 'vim archivo.txt', desc: 'Abrir archivo', escape: false },
  { cmd: 'i',               desc: 'Modo INSERT — ahora puedes escribir', escape: false },
  { cmd: 'ESC',             desc: 'Volver a modo NORMAL', escape: true },
  { cmd: ':w',              desc: 'Guardar (write)', escape: false },
  { cmd: ':q',              desc: 'Salir (quit)', escape: false },
  { cmd: ':wq',             desc: 'Guardar y salir (recomendado)', escape: false },
  { cmd: ':q!',             desc: 'Salir SIN guardar (forzar)', escape: true },
  { cmd: 'u',               desc: 'Deshacer (undo) — modo NORMAL', escape: false },
];

// ── Scenarios ──────────────────────────────────────────────────
const scenarios = [
  {
    icon: 'bug_report', color: '#f87171', tool: 'nano',
    title: 'Robot falla en campo',
    situation: 'El robot deja de responder en una competencia. Sin monitor, sin red WiFi.',
    solution: 'SSH al robot → nano /var/log/ros.log → revisar error → editar config → reiniciar.',
  },
  {
    icon: 'code', color: '#60a5fa', tool: 'VS Code',
    title: 'Desarrollar nodo de navegación',
    situation: 'Escribir un nodo de control con lógica compleja de Nav2.',
    solution: 'VS Code → IntelliSense para ROS 2 types → debugging integrado → Git commit.',
  },
  {
    icon: 'wifi_off', color: '#fbbf24', tool: 'nano',
    title: 'Configurar WiFi sin monitor',
    situation: 'El robot no se conecta a la red en el nuevo laboratorio.',
    solution: 'sudo nano /etc/netplan/01-netcfg.yaml → editar SSID/password → sudo netplan apply.',
  },
  {
    icon: 'cloud_sync', color: '#c084fc', tool: 'VS Code Remote SSH',
    title: 'Editar código en robot remoto',
    situation: 'El robot está corriendo y necesitas ajustar parámetros en vivo.',
    solution: 'VS Code → Remote SSH → editar launch file → recargar sin recompilar.',
  },
];

// ── Summary ─────────────────────────────────────────────────────
const summaryItems = [
  { editor: 'nano',             when: 'Ediciones rápidas, SSH, sin GUI disponible',    cmd: 'nano archivo.txt', color: '#4ade80' },
  { editor: 'VS Code',          when: 'Desarrollo diario, debugging, proyectos grandes', cmd: 'code .', color: '#60a5fa' },
  { editor: 'Remote SSH',       when: 'Editar código en robots físicos remotamente',   cmd: 'F1 → Remote-SSH', color: '#c084fc' },
  { editor: 'vim',              when: 'Opcional: cuando vim ya está en el servidor',    cmd: 'vim archivo.txt', color: '#f87171' },
];
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background: var(--bg-code); color: var(--text-code);
  padding: 2px 7px; border-radius: 5px;
  font-family: 'Fira Code', monospace; font-size: 0.9em;
}
kbd {
  background: var(--bg-surface-hover); border: 1px solid var(--border-medium);
  border-bottom-width: 3px; color: var(--text-primary);
  padding: 2px 7px; border-radius: 5px; font-size: 0.85em;
  font-family: inherit;
}
.cmd-badge {
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: 0.75rem; font-weight: 800; margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }
.cmd-inline { font-family:'Fira Code',monospace; font-size:.95em; background:none; color:var(--text-code); padding:0; }
.optional-tag {
  display:inline-flex; align-items:center; font-size:.65rem; font-weight:800;
  letter-spacing:.1em; background:rgba(248,113,113,.12); color:#f87171;
  border:1px solid rgba(248,113,113,.3); padding:2px 8px; border-radius:999px;
  margin-left:10px; vertical-align:middle;
}

/* ── Fact pills */
.fact-pills { display:flex; gap:10px; flex-wrap:wrap; margin-bottom:1.5rem; }
.fact-pill {
  display:flex; align-items:center; gap:8px;
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:999px; padding:7px 16px;
  font-size:.84rem; color:var(--text-secondary); transition:transform .2s;
}
.fact-pill:hover { transform:translateY(-2px); }
.fact-icon { font-size:1rem; }

/* ══════════════════════════════════════════
   EDITOR CARDS (3-col)
══════════════════════════════════════════ */
.editors-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:16px; }
.editor-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-top:4px solid var(--ed-color, var(--border-medium));
  border-radius:16px; padding:1.5rem;
  display:flex; flex-direction:column; gap:12px;
  text-align:center; transition:all .25s;
}
.editor-card:hover { transform:translateY(-5px); box-shadow:var(--shadow-md); }
.ed-top { display:flex; align-items:flex-start; justify-content:space-between; }
.ed-emoji { font-size:2.5rem; line-height:1; }
.ed-badge {
  font-size:.65rem; font-weight:800; letter-spacing:.08em;
  padding:3px 10px; border-radius:999px;
}
.ed-name    { font-size:1.35rem; font-weight:800; color:var(--text-primary); }
.ed-tagline { font-size:.9rem; color:var(--text-muted); font-style:italic; }
.ed-stats   { display:flex; flex-direction:column; gap:8px; }
.ed-stat {
  background:var(--bg-surface-hover); border-radius:8px; padding:8px 10px;
  display:flex; flex-direction:column; gap:2px; text-align:left;
}
.es-label { font-size:.7rem; text-transform:uppercase; letter-spacing:.06em; color:var(--text-muted); }
.es-value { font-size:.88rem; font-weight:700; color:var(--text-secondary); }
.ed-when {
  display:flex; align-items:flex-start; gap:4px;
  font-size:.85rem; color:var(--text-secondary); line-height:1.5;
  background:var(--bg-surface-hover); border-radius:8px; padding:10px;
  text-align:left;
}

/* ══════════════════════════════════════════
   NANO TERMINAL MOCK
══════════════════════════════════════════ */
.nano-terminal {
  background:var(--bg-deep); border:1px solid var(--border-medium);
  border-radius:14px; overflow:hidden; font-family:'Fira Code',monospace;
}
.nt-chrome {
  display:flex; align-items:center; gap:12px; padding:10px 14px;
  background:var(--bg-surface); border-bottom:1px solid var(--border-medium);
}
.nt-dots { display:flex; gap:6px; }
.dot { width:12px; height:12px; border-radius:50%; display:block; }
.dot.red    { background:#ef4444; }
.dot.yellow { background:#f59e0b; }
.dot.green  { background:#22c55e; }
.nt-title { font-size:.82rem; color:var(--text-muted); flex:1; text-align:center; }
.nt-body { padding:1rem 1.25rem; min-height:100px; }
.nt-line { display:flex; gap:12px; margin-bottom:4px; }
.nl-num  { color:#475569; min-width:2ch; text-align:right; user-select:none; font-size:.9rem; }
.nl-code { font-size:.9rem; color:var(--text-secondary); }
.cursor  { background:#22c55e; color:#1e1e1e; animation:blink 1s step-end infinite; border-radius:1px; }
@keyframes blink { 50% { opacity:0; } }
.nt-statusbar {
  background:var(--bg-surface); border-top:1px solid var(--border-medium);
  display:flex; justify-content:space-between; padding:5px 14px;
  font-size:.78rem; color:var(--text-muted);
}
.nt-shortcuts {
  background:var(--bg-surface-solid); border-top:1px solid var(--border-medium);
  display:grid; grid-template-columns:repeat(6,1fr); gap:8px;
  padding:10px 14px; font-size:.8rem;
}
.ns-item { display:flex; align-items:center; gap:6px; color:var(--text-secondary); }
.ns-item kbd {
  background:#1e293b; color:#60a5fa; border-color:#334155;
  font-size:.75rem; padding:1px 5px; min-width:32px; text-align:center;
}

/* ── Workflow steps */
.workflow { }
.wf-title { font-size:1rem; font-weight:700; color:var(--text-primary); margin-bottom:1rem; }
.wf-steps { display:flex; flex-direction:column; gap:0; }
.wf-step  { display:flex; align-items:flex-start; gap:14px; position:relative; }
.wfs-num {
  min-width:36px; width:36px; height:36px; border-radius:50%;
  background:linear-gradient(135deg, #4ade80, #22c55e);
  display:flex; align-items:center; justify-content:center;
  font-size:1rem; font-weight:800; color:#1e1e1e; flex-shrink:0; margin-top:2px;
}
.wfs-body { flex:1; padding-bottom:1.5rem; }
.wfs-title { font-size:1rem; font-weight:700; color:var(--text-primary); margin-bottom:4px; }
.wfs-desc  { font-size:.88rem; color:var(--text-secondary); line-height:1.5; }
.wfs-desc :deep(kbd) {
  background:var(--bg-surface-hover); border:1px solid var(--border-medium);
  border-bottom-width:3px; padding:1px 6px; border-radius:4px; font-size:.82em;
}
.wfs-arrow { color:var(--text-muted); font-size:1.2rem; align-self:flex-end; padding-bottom:4px; }

/* ══════════════════════════════════════════
   VS CODE
══════════════════════════════════════════ */
.vsc-section-title {
  display:flex; align-items:center; gap:8px;
  font-size:.95rem; font-weight:700; color:var(--text-primary); margin-bottom:14px;
}
.ext-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }
.ext-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:3px solid var(--ext-color, var(--border-medium));
  border-radius:10px; padding:1rem;
  display:flex; align-items:flex-start; gap:10px; transition:all .2s;
}
.ext-card:hover { transform:translateX(4px); }
.ext-icon-wrap {
  width:36px; height:36px; border-radius:8px; flex-shrink:0;
  display:flex; align-items:center; justify-content:center;
}
.ext-info { flex:1; }
.ext-name   { font-size:.9rem; font-weight:700; color:var(--text-primary); margin-bottom:1px; }
.ext-author { font-size:.72rem; color:var(--text-muted); margin-bottom:4px; }
.ext-desc   { font-size:.82rem; color:var(--text-secondary); line-height:1.4; }

.kbd-grid { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; }
.kbd-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:10px; padding:.9rem; text-align:center; transition:all .2s;
}
.kbd-card:hover { background:var(--bg-surface-hover); }
.kc-keys { display:flex; align-items:center; justify-content:center; gap:3px; flex-wrap:wrap; margin-bottom:6px; }
.kc-keys kbd { font-size:.72rem; padding:2px 6px; }
.kc-action { font-size:.78rem; color:var(--text-secondary); line-height:1.3; }

/* ══════════════════════════════════════════
   REMOTE SSH DIAGRAM
══════════════════════════════════════════ */
.ssh-diagram {
  display:flex; align-items:center; justify-content:center; gap:0;
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:16px; padding:2rem; flex-wrap:wrap; gap:12px;
}
.ssh-node {
  background:var(--bg-surface-hover); border:1px solid var(--border-medium);
  border-radius:14px; padding:1.5rem 2rem; text-align:center;
  display:flex; flex-direction:column; align-items:center; gap:6px;
}
.ssh-node-icon  { font-size:2.5rem; line-height:1; }
.ssh-node-title { font-size:.95rem; font-weight:700; color:var(--text-primary); }
.ssh-node-sub   { font-size:.8rem; color:var(--text-muted); }
.ssh-node-tag {
  font-size:.7rem; font-weight:700; letter-spacing:.07em;
  background:rgba(var(--nt-color-rgb, 96,165,250),.12); color:var(--nt-color,#60a5fa);
  border:1px solid rgba(var(--nt-color-rgb, 96,165,250),.25);
  padding:2px 10px; border-radius:999px; margin-top:4px;
  background: color-mix(in srgb, var(--nt-color) 12%, transparent);
  border-color: color-mix(in srgb, var(--nt-color) 30%, transparent);
}
.ssh-tunnel {
  display:flex; align-items:center; gap:8px; flex-direction:column;
  padding:0 12px;
}
.ssh-line {
  width:60px; height:2px;
  background:linear-gradient(90deg, #22d3ee, #818cf8);
  border-radius:2px;
}
.ssh-label {
  font-family:'Fira Code',monospace; font-size:.75rem; color:#22d3ee;
  background:rgba(34,211,238,.12); padding:3px 10px; border-radius:6px; white-space:nowrap;
}

.connect-steps {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:14px; overflow:hidden;
}
.cs-title {
  display:flex; align-items:center; gap:8px; padding:12px 16px;
  background:var(--bg-deep); border-bottom:1px solid var(--border-subtle);
  font-size:.9rem; font-weight:600; color:var(--text-secondary);
}
.cs-list { padding:1rem 1.25rem; display:flex; flex-direction:column; gap:12px; }
.cs-step { display:flex; align-items:flex-start; gap:12px; font-size:.9rem; color:var(--text-secondary); }
.cs-num {
  min-width:26px; height:26px; border-radius:50%;
  background:rgba(192,132,252,.15); color:#c084fc;
  font-size:.8rem; font-weight:800;
  display:flex; align-items:center; justify-content:center; flex-shrink:0;
}
.cs-body :deep(kbd)  { font-size:.82em; }
.cs-body :deep(code) { background:var(--bg-code); padding:2px 6px; border-radius:4px; font-family:'Fira Code',monospace; font-size:.88em; color:var(--text-code); }
.cs-body :deep(strong) { color:var(--text-primary); }

/* ══════════════════════════════════════════
   VIM
══════════════════════════════════════════ */
.vim-modes {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:16px; padding:1.5rem;
}
.vm-title { font-size:.9rem; font-weight:700; color:var(--text-primary); margin-bottom:14px; text-align:center; }
.vm-flow  { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }
.vm-mode {
  background:var(--bg-surface-hover); border:1px solid var(--border-subtle);
  border-top:3px solid var(--vm-color,var(--border-medium));
  border-radius:12px; padding:1.1rem; text-align:center;
}
.vmm-name { font-family:'Fira Code',monospace; font-size:1.1rem; font-weight:900; margin-bottom:6px; }
.vmm-desc { font-size:.82rem; color:var(--text-secondary); margin-bottom:10px; line-height:1.4; }
.vmm-key  { display:flex; align-items:center; justify-content:center; gap:6px; font-size:.8rem; color:var(--text-muted); }

.survival-kit {
  background:var(--bg-surface); border:2px solid rgba(248,113,113,.35);
  border-radius:16px; overflow:hidden;
}
.sk-header {
  display:flex; align-items:center; gap:8px; padding:1rem 1.5rem;
  background:rgba(248,113,113,.1); border-bottom:1px solid rgba(248,113,113,.25);
  font-size:1rem; font-weight:700; color:#f87171;
}
.sk-grid { display:grid; grid-template-columns:repeat(4,1fr); gap:1px; background:var(--border-subtle); }
.sk-card {
  background:var(--bg-surface); padding:1rem; text-align:center;
  transition:background .15s;
}
.sk-card:hover { background:var(--bg-surface-hover); }
.sk-card.sk-escape { background:rgba(248,113,113,.08); }
.sk-cmd { font-family:'Fira Code',monospace; font-size:1.1rem; font-weight:800; color:#fbbf24; margin-bottom:6px; }
.sk-card.sk-escape .sk-cmd { color:#f87171; }
.sk-desc { font-size:.8rem; color:var(--text-secondary); line-height:1.4; }

/* ══════════════════════════════════════════
   SCENARIOS GRID
══════════════════════════════════════════ */
.scenarios-grid { display:grid; grid-template-columns:repeat(2,1fr); gap:16px; }
.scenario-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid var(--sc-color, var(--border-medium));
  border-radius:14px; padding:1.25rem;
  display:flex; flex-direction:column; gap:10px; transition:all .25s;
}
.scenario-card:hover { transform:translateY(-4px); box-shadow:var(--shadow-md); }
.scc-header { display:flex; align-items:center; gap:10px; }
.scc-title    { font-size:1rem; font-weight:700; color:var(--text-primary); }
.scc-situation{ font-size:.88rem; color:var(--text-muted); font-style:italic; }
.scc-solution { font-size:.88rem; color:var(--text-secondary); line-height:1.5; flex:1; }
.scc-tool {
  display:flex; align-items:center; font-size:.8rem; font-weight:700;
  padding:5px 12px; border-radius:8px; align-self:flex-start;
}

/* ══════════════════════════════════════════
   CHALLENGE BOX
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
.challenge-title    { font-size:1.05rem; font-weight:700; color:var(--text-primary); margin-bottom:4px; }
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
.summary-grid { display:grid; grid-template-columns:repeat(2,1fr); gap:14px; }
.summary-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid var(--sc-color,var(--border-medium));
  border-radius:12px; padding:1rem 1.25rem; transition:all .25s;
}
.summary-card:hover { transform:translateY(-3px); box-shadow:var(--shadow-sm); }
.sc-cmd   { display:block; font-family:'Fira Code',monospace; font-size:1.05rem; font-weight:700; color:var(--sc-color,var(--text-code)); background:none; padding:0; margin-bottom:6px; }
.sc-desc  { font-size:.88rem; color:var(--text-secondary); margin-bottom:8px; line-height:1.45; }
.sc-example { display:flex; align-items:center; font-family:'Fira Code',monospace; font-size:.78rem; color:var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 900px) {
  .editors-grid { grid-template-columns: 1fr 1fr; }
  .ext-grid     { grid-template-columns: 1fr 1fr; }
  .kbd-grid     { grid-template-columns: repeat(2,1fr); }
  .sk-grid      { grid-template-columns: repeat(2,1fr); }
  .vm-flow      { grid-template-columns: 1fr 1fr; }
}

@media (max-width: 768px) {
  .editors-grid   { grid-template-columns: 1fr; }
  .ext-grid       { grid-template-columns: 1fr; }
  .kbd-grid       { grid-template-columns: 1fr 1fr; }
  .sk-grid        { grid-template-columns: 1fr 1fr; }
  .vm-flow        { grid-template-columns: 1fr; }
  .scenarios-grid { grid-template-columns: 1fr; }
  .nt-shortcuts   { grid-template-columns: repeat(3,1fr); }
  .fact-pills     { flex-direction:column; gap:8px; }
  .fact-pill      { border-radius:12px; }
  .ssh-diagram    { flex-direction:column; align-items:center; }
  .ssh-tunnel     { flex-direction:row; }
  .ssh-line       { width:30px; height:2px; }
  .challenge-header { flex-direction:column; }
  .challenge-badge  { margin-left:0; }
}

@media (max-width: 480px) {
  .kbd-grid     { grid-template-columns: 1fr; }
  .sk-grid      { grid-template-columns: 1fr 1fr; }
  .summary-grid { grid-template-columns: 1fr; }
  .nt-shortcuts { grid-template-columns: repeat(2,1fr); }
  .wf-step      { flex-direction:column; align-items:flex-start; }
  .wfs-arrow    { display:none; }
}
</style>
