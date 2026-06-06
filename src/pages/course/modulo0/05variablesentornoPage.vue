<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         SECCIÓN 1: CONTEXTO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        ¿Por qué tu terminal sabe dónde está <code>python3</code> pero lanza error con
        <code>ros2</code>? La respuesta está en las <strong>Variables de Entorno</strong>:
        la memoria invisible que determina qué comandos funcionan y cuáles no.<br /><br />
        Esta lección te enseña a manipular el "ADN" de tu sesión para que ROS 2 funcione
        automáticamente en <strong>cada terminal que abras</strong>.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 2: ¿QUÉ SON?
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        ¿Qué son las Variables de Entorno?
      </SectionTitle>

      <TextBlock>
        Son <strong>pares clave=valor</strong> que almacenan información sobre tu sesión.
        Cada terminal que abres hereda estas variables — como una mochila invisible que
        contiene la configuración de tu sistema.
      </TextBlock>

      <!-- Anatomy card -->
      <div class="anatomy-card q-mt-lg">
        <div class="ac-title">Anatomía de una Variable</div>
        <div class="ac-formula">
          <span class="acf-name">ROS_DOMAIN_ID</span>
          <span class="acf-eq">=</span>
          <span class="acf-val">"42"</span>
        </div>
        <div class="ac-parts">
          <div class="acp-item">
            <div class="acp-arrow">↑</div>
            <div class="acp-label">NOMBRE<br /><span>siempre MAYÚSCULAS</span></div>
          </div>
          <div class="acp-item" style="margin-left:7.5rem">
            <div class="acp-arrow">↑</div>
            <div class="acp-label">VALOR<br /><span>string, número, ruta…</span></div>
          </div>
        </div>
      </div>

      <!-- Common variables table -->
      <div class="vars-table q-mt-xl">
        <div class="vt-header">
          <q-icon name="table_chart" size="14px" color="primary" />
          Variables del Sistema Más Importantes
        </div>
        <div class="vt-rows">
          <div v-for="v in commonVars" :key="v.name" class="vt-row">
            <code class="vt-name" :style="{ color: v.color }">{{ v.name }}</code>
            <span class="vt-desc">{{ v.desc }}</span>
            <code class="vt-example">{{ v.example }}</code>
          </div>
        </div>
      </div>

      <!-- Commands grid -->
      <div class="cmd-grid q-mt-xl">
        <div v-for="c in varCommands" :key="c.title" class="cmd-card"
          :style="{ '--cc-color': c.color }">
          <div class="cc-top">
            <code class="cc-title">{{ c.title }}</code>
          </div>
          <CodeBlock :hide-header="true" lang="bash" :content="c.code" />
          <div class="cc-note">{{ c.note }}</div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 3: $PATH
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">02</span>
        La Variable Reina — <code class="cmd-inline">$PATH</code>
      </SectionTitle>

      <TextBlock>
        Cuando escribes <code>ros2</code>, Linux <strong>NO busca en todo el disco</strong>.
        Solo busca en las carpetas listadas en <code>$PATH</code>, separadas por <code>:</code>.
      </TextBlock>

      <!-- PATH flow -->
      <div class="path-flow q-mt-lg">
        <div class="pf-step" style="--pf-color:#60a5fa">
          <div class="pfs-num" style="background:#60a5fa">1</div>
          <div class="pfs-title">Escribes</div>
          <code class="pfs-code">ros2 topic list</code>
        </div>
        <div class="pf-arrow">→</div>
        <div class="pf-step" style="--pf-color:#fbbf24">
          <div class="pfs-num" style="background:#fbbf24">2</div>
          <div class="pfs-title">Linux busca <code>ros2</code> en cada carpeta de PATH</div>
          <code class="pfs-code">/usr/bin → /usr/local/bin → /opt/ros/jazzy/bin → …</code>
        </div>
        <div class="pf-arrow">→</div>
        <div class="pf-step" style="--pf-color:#4ade80">
          <div class="pfs-num" style="background:#4ade80">3</div>
          <div class="pfs-title">Resultado</div>
          <div class="pfs-results">
            <div class="pfs-ok">✅ Encontrado → Ejecuta</div>
            <div class="pfs-err">❌ No encontrado → "command not found"</div>
          </div>
        </div>
      </div>

      <CodeBlock title="Ver tu PATH actual" lang="bash"
        content="echo $PATH
# /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/opt/ros/jazzy/bin

# Ver cada directorio en líneas separadas
echo $PATH | tr ':' '\n'" :copyable="true" class="q-mt-lg" />

      <AlertBlock type="warning" title="El problema de ROS 2" class="q-mt-md">
        ROS 2 se instala en <code>/opt/ros/jazzy</code>, una carpeta que
        <strong>NO está en PATH por defecto</strong>.<br />
        Por eso obtienes <code>bash: ros2: command not found</code> hasta que lo agregas.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 4: SOURCE
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">03</span>
        El Ritual del <code class="cmd-inline">source</code>
      </SectionTitle>

      <TextBlock>
        <code>source</code> ejecuta un script que modifica tus variables de entorno en la
        sesión actual. En ROS 2, el archivo <code>setup.bash</code> agrega las rutas
        necesarias a PATH y define variables críticas.
      </TextBlock>

      <!-- Before / After -->
      <div class="source-comparison q-mt-lg">
        <div class="sc-panel before">
          <div class="scp-header">
            <q-icon name="cancel" size="18px" color="negative" />
            Antes de source
          </div>
          <CodeBlock :hide-header="true" lang="bash"
            content="ros2 topic list
# bash: ros2: command not found

echo $ROS_DISTRO
# (vacío)" />
        </div>

        <div class="sc-action">
          <q-icon name="bolt" size="28px" color="warning" />
          <CodeBlock :hide-header="true" lang="bash"
            content="source /opt/ros/jazzy/setup.bash" :copyable="true" />
          <q-icon name="arrow_downward" size="20px" style="color:var(--text-muted)" />
        </div>

        <div class="sc-panel after">
          <div class="scp-header">
            <q-icon name="check_circle" size="18px" color="positive" />
            Después de source
          </div>
          <CodeBlock :hide-header="true" lang="bash"
            content="ros2 topic list
# /parameter_events
# /rosout

echo $ROS_DISTRO
# jazzy" />
        </div>
      </div>

      <AlertBlock type="info" title="¿Qué hace setup.bash exactamente?" class="q-mt-lg">
        <ol class="setup-list">
          <li>Agrega <code>/opt/ros/jazzy/bin</code> a tu <code>$PATH</code></li>
          <li>Define <code>ROS_DISTRO=jazzy</code> y otras variables de ROS</li>
          <li>Configura rutas de Python para encontrar paquetes ROS 2</li>
          <li>Establece configuración de DDS (middleware de comunicación)</li>
        </ol>
      </AlertBlock>

      <!-- Amnesia card -->
      <div class="amnesia-card q-mt-lg">
        <div class="am-stripe"></div>
        <div class="am-content">
          <q-icon name="psychology" size="2.2rem" color="warning" />
          <div class="am-text">
            <div class="am-title">Amnesia de Terminal</div>
            <div class="am-desc">
              Las variables son <strong>temporales</strong>. Si cierras la terminal,
              desaparecen — como si nunca hubieras hecho source.<br />
              Tendrías que repetir el proceso en cada nueva ventana... a menos que lo
              <strong>automatices en .bashrc</strong>.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 5: .BASHRC
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">04</span>
        La Solución Permanente — <code class="cmd-inline">~/.bashrc</code>
      </SectionTitle>

      <TextBlock>
        <code>~/.bashrc</code> es un script que se ejecuta <strong>automáticamente</strong>
        cada vez que abres una terminal nueva. Es el lugar perfecto para
        "tatuar" tus configuraciones de ROS 2.
      </TextBlock>

      <!-- Workflow -->
      <div class="bashrc-steps q-mt-lg">
        <div v-for="(step, i) in bashrcSteps" :key="i" class="bs-step">
          <div class="bs-connector" v-if="i > 0"></div>
          <div class="bs-row">
            <div class="bs-num">{{ i + 1 }}</div>
            <div class="bs-body">
              <div class="bs-title">{{ step.title }}</div>
              <div v-if="step.desc" class="bs-desc">{{ step.desc }}</div>
              <CodeBlock v-if="step.code" :hide-header="true" lang="bash"
                :content="step.code" :copyable="true" />
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="¡Ahora es permanente!" class="q-mt-lg">
        Cada nueva terminal que abras tendrá ROS 2 configurado automáticamente.
        No más <code>source</code> manual en cada ventana.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 6: ROS_DOMAIN_ID
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Variable Crítica — <code class="cmd-inline">ROS_DOMAIN_ID</code>
      </SectionTitle>

      <TextBlock>
        ROS 2 usa DDS para comunicación. Por defecto, <strong>todos los robots en la misma
        red WiFi se ven entre sí</strong>. Imagina 20 estudiantes en el mismo laboratorio:
        caos total. <code>ROS_DOMAIN_ID</code> crea canales privados aislados.
      </TextBlock>

      <!-- Domain scenarios -->
      <div class="domain-scenarios q-mt-lg">
        <div class="ds-panel bad">
          <div class="ds-header">
            <q-icon name="warning" size="18px" color="negative" />
            Sin ROS_DOMAIN_ID — Dominio 0 (defecto)
          </div>
          <div class="ds-body">
            <div class="ds-desc">
              20 estudiantes en la misma WiFi → todos en dominio 0 →
              los comandos de un robot <strong>interfieren con los demás</strong>.
            </div>
            <div class="ds-visual chaos">
              <div v-for="n in 3" :key="n" class="dv-bot">🤖</div>
              <div class="dv-chaos">← señales cruzadas →</div>
              <div v-for="n in 3" :key="n+3" class="dv-bot">🤖</div>
            </div>
          </div>
        </div>

        <div class="ds-panel good">
          <div class="ds-header">
            <q-icon name="check_circle" size="18px" color="positive" />
            Con ROS_DOMAIN_ID único — Canales aislados
          </div>
          <div class="ds-body">
            <div class="ds-desc">
              Cada estudiante tiene su propio dominio (0–101) →
              aislamiento total → solo tu laptop controla tu robot.
            </div>
            <div class="ds-visual domains">
              <div v-for="d in domainBoxes" :key="d.id" class="dv-domain"
                :style="{ '--dv-color': d.color }">
                <div class="dvd-label">Dominio {{ d.id }}</div>
                <div class="dvd-bot">🤖</div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <CodeBlock title="Configurar tu dominio único" lang="bash"
        content="# En ~/.bashrc — elige un número entre 0 y 101
export ROS_DOMAIN_ID=42

# Verificar
echo $ROS_DOMAIN_ID
# 42" :copyable="true" class="q-mt-lg" />
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 7: OVERLAY
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">06</span>
        Overlay de Workspace
      </SectionTitle>

      <TextBlock>
        Cuando compilas tu propio código con <code>colcon build</code>, se genera un
        <code>install/setup.bash</code> en tu workspace. Debes hacer source de
        <strong>ambos en orden</strong>: primero el underlay (ROS base), luego el overlay
        (tu código).
      </TextBlock>

      <!-- Layer diagram -->
      <div class="overlay-diagram q-mt-lg">
        <div class="od-layer underlay">
          <div class="odl-badge">Underlay</div>
          <div class="odl-title">ROS 2 Base</div>
          <code class="odl-cmd">source /opt/ros/jazzy/setup.bash</code>
          <div class="odl-desc">Paquetes oficiales: nav2, tf2, rclpy, etc.</div>
        </div>
        <div class="od-arrow">
          <q-icon name="add" size="20px" style="color:var(--text-muted)" />
          <span>Overlay</span>
        </div>
        <div class="od-layer overlay">
          <div class="odl-badge">Overlay</div>
          <div class="odl-title">Tu Workspace</div>
          <code class="odl-cmd">source ~/ros2_ws/install/setup.bash</code>
          <div class="odl-desc">Tus paquetes personalizados: mi_robot, sensores, etc.</div>
        </div>
        <div class="od-arrow">
          <q-icon name="check" size="20px" color="positive" />
          <span>Resultado</span>
        </div>
        <div class="od-result">
          <code>ros2 run mi_robot control_node</code>
          <span>¡Funciona! ROS encuentra tus paquetes Y los oficiales.</span>
        </div>
      </div>

      <CodeBlock title="Orden correcto en ~/.bashrc" lang="bash"
        content="# 1. PRIMERO el ROS base (underlay)
source /opt/ros/jazzy/setup.bash

# 2. LUEGO tu workspace (overlay) — solo si existe
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi" :copyable="true" class="q-mt-lg" />

      <AlertBlock type="warning" title="El orden importa" class="q-mt-md">
        Si haces source del overlay antes del underlay, el overlay no encontrará las
        dependencias de ROS base y tu workspace no funcionará correctamente.
      </AlertBlock>
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
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'" size="20px"
              style="color:var(--text-muted)" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Causa:</strong> {{ err.cause }}
            </div>
            <div class="err-solution">
              <q-icon name="check_circle" size="14px" class="q-mr-xs" color="positive" />
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
         SECCIÓN 9: RETO PRÁCTICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Configurar tu .bashrc Profesional</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea tu .bashrc ideal para ROS 2 Jazzy</div>
            <div class="challenge-subtitle">
              Después de esto, cada terminal nueva tendrá todo configurado automáticamente.
            </div>
          </div>
          <div class="challenge-badge">NIVEL BÁSICO</div>
        </div>

        <CodeBlock title="Tu .bashrc Profesional para ROS 2" lang="bash"
          content="# 1. Abrir .bashrc
nano ~/.bashrc

# 2. Navega al final del archivo y agrega:
# ─────────────────────────────────────────
# ROS 2 Jazzy — Configuración Base
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42    # Elige TU número (0-101)

# Workspace personal (se activa cuando exista)
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi

# Alias para flujo de trabajo ROS 2
alias cb='colcon build --symlink-install'
alias sb='source install/setup.bash'
alias rt='ros2 topic list'
alias rn='ros2 node list'
# ─────────────────────────────────────────

# 3. Guardar: Ctrl+O → Enter → Ctrl+X
# 4. Aplicar ahora sin cerrar terminal:
source ~/.bashrc

# 5. Verificar que todo funciona
echo $ROS_DISTRO    # jazzy
echo $ROS_DOMAIN_ID # 42
ros2 --version" :copyable="true" />

        <q-expansion-item icon="lightbulb" label="Ver resultado esperado"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div class="answer-row">
              <span class="answer-key">echo $ROS_DISTRO:</span>
              <code>jazzy</code>
            </div>
            <div class="answer-row">
              <span class="answer-key">echo $ROS_DOMAIN_ID:</span>
              <code>42</code> (tu número elegido)
            </div>
            <div class="answer-row">
              <span class="answer-key">ros2 --version:</span>
              <code>ros2 cli version 0.x.x (jazzy)</code>
            </div>
            <div class="answer-row">
              <span class="answer-key">Nuevas terminales:</span>
              <span>ya tienen ROS 2 activo sin hacer source manualmente</span>
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
      <TextBlock>Variables de entorno y configuración de ROS 2 en acción:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Variables de Entorno en Linux" frameborder="0"
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
        <div v-for="s in summaryItems" :key="s.cmd" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="terminal" size="12px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="¡Siguiente paso!" class="q-mt-xl">
        Con las variables de entorno dominadas, el siguiente módulo cubre la
        <strong>instalación de ROS 2 Jazzy</strong> en Ubuntu 24.04 — el proceso completo
        paso a paso, incluyendo la configuración automática del .bashrc.
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

// ── Fact pills ─────────────────────────────────────────────────
const facts = [
  { icon: '🔑', label: 'NOMBRE=valor — pares clave-valor del sistema' },
  { icon: '🛤️', label: '$PATH — define qué comandos existen' },
  { icon: '📄', label: '~/.bashrc — la memoria permanente de tu terminal' },
];

// ── Common variables ───────────────────────────────────────────
const commonVars = [
  { name: 'HOME',          desc: 'Tu carpeta personal',                          example: '/home/alexander', color: '#4ade80' },
  { name: 'USER',          desc: 'Tu nombre de usuario',                          example: 'alexander',       color: '#22d3ee' },
  { name: 'PATH',          desc: 'Carpetas donde Linux busca ejecutables',         example: '/usr/bin:/opt/ros/jazzy/bin', color: '#fbbf24' },
  { name: 'ROS_DISTRO',    desc: 'Versión de ROS 2 activa',                       example: 'jazzy',           color: '#c084fc' },
  { name: 'ROS_DOMAIN_ID', desc: 'Canal de comunicación DDS (0-101)',              example: '42',              color: '#f87171' },
  { name: 'SHELL',         desc: 'Intérprete de comandos en uso',                  example: '/bin/bash',       color: '#fb923c' },
];

// ── Var commands ───────────────────────────────────────────────
const varCommands = [
  {
    title: 'printenv', color: '#4ade80',
    code: 'printenv\n# Lista TODAS las variables\n\nprintenv | grep ROS\n# Solo variables de ROS 2',
    note: 'Ver todas las variables activas',
  },
  {
    title: 'echo $VAR', color: '#60a5fa',
    code: 'echo $HOME\n# /home/alexander\n\necho $ROS_DISTRO\n# jazzy',
    note: 'Leer el valor de una variable',
  },
  {
    title: 'export', color: '#c084fc',
    code: 'export MI_ROBOT="TurtleBot3"\nexport ROS_DOMAIN_ID=42\n\necho $MI_ROBOT',
    note: 'Crear / modificar variable (temporal)',
  },
];

// ── .bashrc workflow ───────────────────────────────────────────
const bashrcSteps = [
  {
    title: 'Abrir .bashrc con nano',
    desc: 'El archivo está en tu home. Si no existe, nano lo crea.',
    code: 'nano ~/.bashrc',
  },
  {
    title: 'Agregar al final del archivo',
    desc: 'Usa las flechas para ir al final. Agrega estas líneas:',
    code: '# ROS 2 Jazzy\nsource /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=42\n\n# Alias útiles\nalias cb=\'colcon build --symlink-install\'\nalias sb=\'source install/setup.bash\'',
  },
  {
    title: 'Guardar y salir de nano',
    desc: undefined,
    code: '# Ctrl+O → Enter (guardar)\n# Ctrl+X (salir)',
  },
  {
    title: 'Aplicar los cambios ahora',
    desc: 'Sin cerrar la terminal:',
    code: 'source ~/.bashrc\n\n# Verificar:\necho $ROS_DISTRO   # jazzy\nros2 --version',
  },
];

// ── Domain boxes ───────────────────────────────────────────────
const domainBoxes = [
  { id: 42, color: '#4ade80' },
  { id: 43, color: '#60a5fa' },
  { id: 44, color: '#c084fc' },
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    msg: "bash: ros2: command not found",
    summary: 'No has activado el entorno de ROS 2',
    cause: 'No se ha ejecutado source del setup.bash de ROS 2 en esta terminal.',
    steps: [
      'Ejecuta temporalmente: source /opt/ros/jazzy/setup.bash',
      'Para que sea permanente, agrega esa línea a ~/.bashrc',
      'Recarga: source ~/.bashrc',
    ],
    code: 'source /opt/ros/jazzy/setup.bash\n# O agrega a ~/.bashrc y recarga:\nsource ~/.bashrc',
    open: false,
  },
  {
    msg: "Package 'mi_paquete' not found",
    summary: 'Compilaste pero no activaste tu workspace',
    cause: 'Hiciste colcon build pero no hiciste source del install/setup.bash del workspace.',
    steps: [
      'Ve a tu workspace: cd ~/ros2_ws',
      'Activa el workspace: source install/setup.bash',
      'O agrega la línea condicional a ~/.bashrc',
    ],
    code: 'cd ~/ros2_ws\nsource install/setup.bash',
    open: false,
  },
  {
    msg: "Veo topics de otros robots en mi red",
    summary: 'Estás usando el dominio 0 (por defecto)',
    cause: 'Sin ROS_DOMAIN_ID configurado, todos los nodos en la misma red WiFi se ven entre sí.',
    steps: [
      'Elige un número único entre 0 y 101',
      'Agrégalo a ~/.bashrc: export ROS_DOMAIN_ID=42',
      'Recarga la configuración: source ~/.bashrc',
      'Todos los dispositivos de tu setup deben usar el mismo número',
    ],
    code: 'echo \'export ROS_DOMAIN_ID=42\' >> ~/.bashrc\nsource ~/.bashrc',
    open: false,
  },
  {
    msg: "source: install/setup.bash: No such file or directory",
    summary: 'El workspace no ha sido compilado aún',
    cause: 'El archivo install/setup.bash solo existe después de hacer colcon build.',
    steps: [
      'Compila primero: cd ~/ros2_ws && colcon build',
      'Luego activa: source install/setup.bash',
      'La condición en .bashrc (if [ -f ... ]) evita este error automáticamente',
    ],
    code: 'cd ~/ros2_ws\ncolcon build --symlink-install\nsource install/setup.bash',
    open: false,
  },
]);

// ── Summary ────────────────────────────────────────────────────
const summaryItems = [
  { cmd: 'printenv',         desc: 'Ver todas las variables de entorno activas',    example: 'printenv | grep ROS',         color: '#4ade80' },
  { cmd: 'echo $VAR',        desc: 'Leer el valor de una variable específica',      example: 'echo $ROS_DISTRO',            color: '#22d3ee' },
  { cmd: 'export VAR=val',   desc: 'Crear o modificar variable (temporal)',          example: 'export ROS_DOMAIN_ID=42',     color: '#c084fc' },
  { cmd: 'source file.bash', desc: 'Ejecutar script y aplicar vars al entorno',     example: 'source /opt/ros/jazzy/setup.bash', color: '#fbbf24' },
  { cmd: 'nano ~/.bashrc',   desc: 'Editar configuración permanente del shell',      example: 'nano ~/.bashrc',              color: '#fb923c' },
  { cmd: 'source ~/.bashrc', desc: 'Recargar .bashrc sin cerrar la terminal',        example: 'source ~/.bashrc',            color: '#60a5fa' },
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
.cmd-badge {
  display:inline-flex; align-items:center; justify-content:center;
  width:28px; height:28px; border-radius:8px;
  font-size:.75rem; font-weight:800; margin-right:8px; vertical-align:middle;
}
.cmd-badge.green  { background:rgba( 74,222,128,.15); color:#4ade80; }
.cmd-badge.cyan   { background:rgba( 34,211,238,.15); color:#22d3ee; }
.cmd-badge.purple { background:rgba(192,132,252,.15); color:#c084fc; }
.cmd-badge.amber  { background:rgba(251,191, 36,.15); color:#fbbf24; }
.cmd-badge.red    { background:rgba(248,113,113,.15); color:#f87171; }
.cmd-inline { font-family:'Fira Code',monospace; font-size:.95em; background:none; color:var(--text-code); padding:0; }

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
   ANATOMY CARD
══════════════════════════════════════════ */
.anatomy-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:16px; padding:2rem; text-align:center;
}
.ac-title { font-size:.9rem; font-weight:700; color:var(--text-muted); text-transform:uppercase; letter-spacing:.08em; margin-bottom:1.25rem; }
.ac-formula {
  font-family:'Fira Code',monospace; font-size:1.8rem; font-weight:900; margin-bottom:1rem;
}
.acf-name { color:var(--text-primary); }
.acf-eq   { color:var(--text-muted); margin:0 .75rem; }
.acf-val  { color:#4ade80; }
.ac-parts { display:flex; justify-content:center; gap:0; position:relative; }
.acp-item { display:flex; flex-direction:column; align-items:center; gap:4px; }
.acp-arrow { font-size:1.1rem; color:var(--text-muted); }
.acp-label { font-size:.78rem; font-weight:700; color:var(--text-secondary); text-align:center; }
.acp-label span { font-size:.72rem; font-weight:400; color:var(--text-muted); display:block; }

/* ── Vars table */
.vars-table {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:14px; overflow:hidden;
}
.vt-header {
  background:var(--bg-deep); border-bottom:1px solid var(--border-subtle);
  padding:10px 16px; display:flex; align-items:center; gap:8px;
  font-size:.85rem; font-weight:600; color:var(--text-secondary);
}
.vt-rows { display:flex; flex-direction:column; }
.vt-row {
  display:grid; grid-template-columns:160px 1fr 220px; gap:12px;
  padding:10px 16px; border-bottom:1px solid var(--border-subtle);
  align-items:center; transition:background .15s;
}
.vt-row:last-child { border-bottom:none; }
.vt-row:hover { background:var(--bg-surface-hover); }
.vt-name    { font-family:'Fira Code',monospace; font-size:.9rem; font-weight:700; background:none; padding:0; }
.vt-desc    { font-size:.85rem; color:var(--text-secondary); }
.vt-example { font-family:'Fira Code',monospace; font-size:.78rem; color:var(--text-muted); background:none; padding:0; }

/* ── Commands grid (3 cards) */
.cmd-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:14px; }
.cmd-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-top:3px solid var(--cc-color,var(--border-medium));
  border-radius:12px; padding:1.1rem;
  display:flex; flex-direction:column; gap:8px; min-width:0;
}
.cc-top { display:flex; }
.cc-title { font-family:'Fira Code',monospace; font-size:1rem; font-weight:800; color:var(--cc-color); background:none; padding:0; }
.cc-note  { font-size:.8rem; color:var(--text-muted); font-style:italic; }

/* ══════════════════════════════════════════
   PATH FLOW
══════════════════════════════════════════ */
.path-flow {
  display:grid; grid-template-columns:1fr auto 1fr auto 1fr; gap:8px;
  align-items:center; background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:16px; padding:1.5rem;
}
.pf-step {
  background:var(--bg-surface-hover);
  border:1px solid var(--border-medium);
  border-top:3px solid var(--pf-color, var(--border-medium));
  border-radius:12px; padding:1.1rem; text-align:center;
  display:flex; flex-direction:column; align-items:center; gap:8px;
}
.pfs-num {
  width:28px; height:28px; border-radius:50%;
  display:flex; align-items:center; justify-content:center;
  font-size:.85rem; font-weight:800; color:#1e1e1e;
}
.pfs-title { font-size:.85rem; font-weight:600; color:var(--text-primary); line-height:1.3; }
.pfs-code  { font-family:'Fira Code',monospace; font-size:.75rem; color:var(--text-code); background:var(--bg-code); padding:3px 8px; border-radius:5px; word-break:break-all; }
.pfs-results { display:flex; flex-direction:column; gap:4px; font-size:.82rem; }
.pfs-ok  { color:#4ade80; }
.pfs-err { color:#f87171; }
.pf-arrow { font-size:1.5rem; color:var(--text-muted); text-align:center; }

/* ══════════════════════════════════════════
   SOURCE COMPARISON
══════════════════════════════════════════ */
.source-comparison {
  display:grid; grid-template-columns:1fr auto 1fr; gap:16px; align-items:start;
}
.sc-panel {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:14px; overflow:hidden;
}
.sc-panel.before { border-top:3px solid #f87171; }
.sc-panel.after  { border-top:3px solid #4ade80; }
.scp-header {
  display:flex; align-items:center; gap:8px; padding:10px 14px;
  border-bottom:1px solid var(--border-subtle);
  font-size:.88rem; font-weight:700; color:var(--text-primary);
}
.sc-panel.before .scp-header { background:rgba(248,113,113,.08); }
.sc-panel.after  .scp-header { background:rgba( 74,222,128,.08); }
.sc-action {
  display:flex; flex-direction:column; align-items:center; gap:10px;
  padding-top:2rem;
}

/* ── Amnesia card */
.amnesia-card {
  background:linear-gradient(135deg, rgba(120,53,15,.15), rgba(146,64,14,.15));
  border:2px solid rgba(251,146,60,.4); border-radius:16px; overflow:hidden;
}
.am-stripe {
  height:5px;
  background:repeating-linear-gradient(45deg,#f59e0b 0,#f59e0b 8px,transparent 8px,transparent 16px);
  opacity:.6;
}
.am-content {
  display:flex; align-items:flex-start; gap:1.25rem; padding:1.5rem;
}
.am-title { font-size:1.1rem; font-weight:800; color:#f59e0b; margin-bottom:6px; }
.am-desc  { font-size:.9rem; color:var(--text-secondary); line-height:1.6; }
.am-desc strong { color:var(--text-primary); }

.setup-list { margin:0; padding-left:18px; }
.setup-list li { margin-bottom:6px; font-size:.9rem; color:var(--text-secondary); }

/* ══════════════════════════════════════════
   .BASHRC WORKFLOW
══════════════════════════════════════════ */
.bashrc-steps { display:flex; flex-direction:column; gap:0; }
.bs-step { display:flex; flex-direction:column; }
.bs-connector { width:2px; height:20px; background:var(--border-medium); margin-left:17px; }
.bs-row { display:flex; align-items:flex-start; gap:14px; }
.bs-num {
  min-width:36px; width:36px; height:36px; border-radius:50%; flex-shrink:0;
  background:linear-gradient(135deg,#fbbf24,#f59e0b);
  display:flex; align-items:center; justify-content:center;
  font-size:1rem; font-weight:800; color:#1e1e1e; margin-top:2px;
}
.bs-body { flex:1; padding-bottom:1rem; }
.bs-title { font-size:1rem; font-weight:700; color:var(--text-primary); margin-bottom:6px; }
.bs-desc  { font-size:.88rem; color:var(--text-secondary); margin-bottom:8px; line-height:1.5; }

/* ══════════════════════════════════════════
   DOMAIN SCENARIOS
══════════════════════════════════════════ */
.domain-scenarios { display:flex; flex-direction:column; gap:16px; }
.ds-panel {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:14px; overflow:hidden;
}
.ds-panel.bad  { border-top:3px solid #f87171; }
.ds-panel.good { border-top:3px solid #4ade80; }
.ds-header {
  display:flex; align-items:center; gap:8px; padding:12px 16px;
  font-size:.95rem; font-weight:700; border-bottom:1px solid var(--border-subtle);
}
.ds-panel.bad  .ds-header { background:rgba(248,113,113,.08); color:#f87171; }
.ds-panel.good .ds-header { background:rgba( 74,222,128,.08); color:#4ade80; }
.ds-body { padding:1.25rem 1.5rem; }
.ds-desc { font-size:.9rem; color:var(--text-secondary); line-height:1.6; margin-bottom:1rem; }
.ds-desc strong { color:var(--text-primary); }
.ds-visual {
  background:var(--bg-surface-hover); border-radius:10px; padding:1rem;
  display:flex; align-items:center; justify-content:center; flex-wrap:wrap; gap:8px;
}
.ds-visual.chaos { gap:4px; }
.dv-bot { font-size:1.8rem; line-height:1; }
.dv-chaos { font-size:.8rem; color:#f87171; font-style:italic; width:100%; text-align:center; }
.ds-visual.domains { gap:12px; }
.dv-domain {
  display:flex; flex-direction:column; align-items:center; gap:6px;
  background:color-mix(in srgb, var(--dv-color) 10%, var(--bg-surface));
  border:1px solid color-mix(in srgb, var(--dv-color) 30%, transparent);
  border-radius:10px; padding:.75rem 1.25rem;
}
.dvd-label { font-family:'Fira Code',monospace; font-size:.78rem; color:var(--dv-color); font-weight:700; }
.dvd-bot   { font-size:1.8rem; line-height:1; }

/* ══════════════════════════════════════════
   OVERLAY DIAGRAM
══════════════════════════════════════════ */
.overlay-diagram {
  display:flex; flex-direction:column; align-items:center; gap:0;
}
.od-layer {
  width:100%; max-width:620px; border-radius:14px; padding:1.25rem 1.5rem; text-align:center;
  display:flex; flex-direction:column; gap:8px; align-items:center;
}
.od-layer.underlay {
  background:rgba( 96,165,250,.1); border:2px solid rgba( 96,165,250,.4);
}
.od-layer.overlay {
  background:rgba(192,132,252,.1); border:2px solid rgba(192,132,252,.4);
}
.odl-badge {
  font-size:.7rem; font-weight:800; letter-spacing:.08em;
  padding:2px 10px; border-radius:999px; margin-bottom:4px;
}
.underlay .odl-badge { background:rgba(96,165,250,.15); color:#60a5fa; }
.overlay  .odl-badge { background:rgba(192,132,252,.15); color:#c084fc; }
.odl-title { font-size:1rem; font-weight:700; color:var(--text-primary); }
.odl-cmd   { font-family:'Fira Code',monospace; font-size:.85rem; color:var(--text-code); background:var(--bg-code); padding:4px 12px; border-radius:6px; }
.odl-desc  { font-size:.82rem; color:var(--text-muted); }
.od-arrow {
  display:flex; align-items:center; gap:6px; padding:8px 0;
  font-size:.82rem; font-weight:700; color:var(--text-muted);
}
.od-result {
  width:100%; max-width:620px; border-radius:12px;
  background:rgba(74,222,128,.08); border:2px solid rgba(74,222,128,.3);
  padding:1rem 1.5rem; text-align:center; display:flex; flex-direction:column; gap:6px;
}
.od-result code { color:#4ade80; font-size:.9rem; background:none; padding:0; }
.od-result span { font-size:.85rem; color:var(--text-secondary); }

/* ══════════════════════════════════════════
   ERROR LIST
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
.err-msg     { font-size:.88rem; display:block; margin-bottom:4px; color:#ef4444; background:none; padding:0; word-break:break-all; }
.err-summary { font-size:.82rem; color:var(--text-muted); }
.err-body {
  padding:1rem 1.5rem 1.25rem; border-top:1px solid var(--border-subtle);
  display:flex; flex-direction:column; gap:10px;
}
.err-cause, .err-solution {
  font-size:.9rem; color:var(--text-secondary);
  display:flex; align-items:flex-start; gap:6px; line-height:1.5;
}
.err-solution ol { margin:4px 0 0 16px; padding:0; }
.err-solution li { margin-bottom:4px; }

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
.summary-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:14px; }
.summary-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid var(--sc-color,var(--border-medium));
  border-radius:12px; padding:1rem 1.25rem; transition:all .25s;
}
.summary-card:hover { transform:translateY(-3px); box-shadow:var(--shadow-sm); }
.sc-cmd   { display:block; font-family:'Fira Code',monospace; font-size:1rem; font-weight:700; color:var(--sc-color,var(--text-code)); background:none; padding:0; margin-bottom:6px; }
.sc-desc  { font-size:.85rem; color:var(--text-secondary); margin-bottom:8px; line-height:1.4; }
.sc-example { display:flex; align-items:center; font-family:'Fira Code',monospace; font-size:.75rem; color:var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 900px) {
  .cmd-grid     { grid-template-columns: 1fr 1fr; }
  .summary-grid { grid-template-columns: 1fr 1fr; }
  .vt-row       { grid-template-columns: 140px 1fr; }
  .vt-row .vt-example { display: none; }
  .path-flow    { grid-template-columns: 1fr; }
  .pf-arrow     { text-align:center; transform:rotate(90deg); }
  .source-comparison { grid-template-columns: 1fr; }
  .sc-action    { flex-direction: row; padding:0; }
}

@media (max-width: 768px) {
  .cmd-grid     { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: 1fr 1fr; }
  .fact-pills   { flex-direction:column; gap:8px; }
  .fact-pill    { border-radius:12px; }
  .am-content   { flex-direction:column; }
  .challenge-header { flex-direction:column; }
  .challenge-badge  { margin-left:0; }
  .vt-row { grid-template-columns: 1fr 1fr; }
  .vt-row .vt-example { display:none; }
  .ds-visual.domains { gap:8px; }
}

@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
  .vt-row { grid-template-columns: 1fr; }
  .dv-domain { padding:.6rem 1rem; }
  .ac-formula { font-size:1.2rem; }
}
</style>
