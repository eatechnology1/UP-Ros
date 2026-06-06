<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         SECCIÓN 1: CONTEXTO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        En Linux el software no se descarga de páginas web — viaja por
        <strong>repositorios seguros</strong> verificados por la distribución.
        Esta lección cubre los tres gestores que usarás en robótica, y culmina con
        <strong>la instalación completa de ROS 2 Jazzy</strong> en Ubuntu 24.04,
        paso a paso, sin saltarse nada.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 2: LAS TRES VÍAS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Las Tres Vías de Instalación
      </SectionTitle>

      <div class="methods-grid q-mt-lg">
        <div v-for="m in methods" :key="m.name" class="method-card"
          :style="{ '--mc-color': m.color }">
          <div class="mc-top">
            <div class="mc-icon-wrap" :style="{ background: m.color + '18' }">
              <q-icon :name="m.icon" size="26px" :style="{ color: m.color }" />
            </div>
            <div class="mc-badge" :style="{ background: m.color + '18', color: m.color }">{{ m.badge }}</div>
          </div>
          <div class="mc-name">{{ m.name }}</div>
          <div class="mc-sub">{{ m.sub }}</div>
          <div class="mc-desc">{{ m.desc }}</div>
          <div class="mc-examples">
            <span class="mce-label">Ejemplos:</span>
            <code v-for="ex in m.examples" :key="ex">{{ ex }}</code>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 3: APT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        APT — El Gestor Oficial de Ubuntu
      </SectionTitle>

      <TextBlock>
        APT (Advanced Package Tool) es el gestor principal de Ubuntu.
        Resuelve dependencias automáticamente y garantiza software verificado y seguro.
      </TextBlock>

      <!-- APT workflow — vertical timeline -->
      <div class="apt-timeline q-mt-lg">
        <div v-for="(step, i) in aptSteps" :key="i" class="at-step">
          <div class="at-connector" v-if="i > 0"></div>
          <div class="at-row">
            <div class="at-num" :style="{ background: step.color }">{{ i + 1 }}</div>
            <div class="at-body">
              <div class="at-title">{{ step.title }}</div>
              <CodeBlock :hide-header="true" lang="bash" :content="step.code" :copyable="true" />
              <div class="at-note">
                <q-icon name="info" size="12px" class="q-mr-xs" />{{ step.note }}
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- APT commands grid -->
      <div class="apt-cmds-grid q-mt-xl">
        <div class="acg-title">
          <q-icon name="grid_view" size="15px" color="primary" />
          Comandos APT de Referencia Rápida
        </div>
        <div class="acg-cards">
          <div v-for="c in aptCmds" :key="c.cmd" class="acg-card"
            :style="{ '--acg-color': c.color }">
            <code class="acg-cmd">{{ c.cmd }}</code>
            <div class="acg-desc">{{ c.desc }}</div>
            <CodeBlock :hide-header="true" lang="bash" :content="c.example" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 4: PYTHON PIP / VENV
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Python — PIP, APT y Entornos Virtuales
      </SectionTitle>

      <AlertBlock type="danger" title="Ubuntu 24.04+ bloquea pip install global (PEP 668)">
        Si intentas <code>pip install numpy</code> directamente, Ubuntu responde:
        <strong>"error: externally-managed-environment"</strong>.
        El sistema te protege de romper dependencias del OS. Tienes dos caminos correctos:
      </AlertBlock>

      <div class="pip-options q-mt-lg">
        <div v-for="opt in pipOptions" :key="opt.title" class="pip-option"
          :style="{ '--po-color': opt.color }">
          <div class="po-header">
            <div class="po-icon-wrap" :style="{ background: opt.color + '18' }">
              <q-icon :name="opt.icon" size="22px" :style="{ color: opt.color }" />
            </div>
            <div>
              <div class="po-status" :style="{ color: opt.color }">{{ opt.status }}</div>
              <div class="po-title">{{ opt.title }}</div>
            </div>
          </div>
          <div class="po-desc">{{ opt.desc }}</div>
          <CodeBlock :hide-header="true" lang="bash" :content="opt.code" :copyable="true" />
          <div class="po-note" :style="{ background: opt.color + '10', borderColor: opt.color + '30' }">
            <strong>{{ opt.noteLabel }}</strong> {{ opt.note }}
          </div>
        </div>
      </div>

      <!-- Never do this -->
      <div class="never-card q-mt-lg">
        <div class="nc-header">
          <q-icon name="block" size="18px" color="negative" />
          Nunca hagas esto
        </div>
        <CodeBlock :hide-header="true" lang="bash" :content="neverCode" />
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 5: INSTALACIÓN ROS 2 JAZZY ⭐
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">04</span>
        Instalación de ROS 2 Jazzy — Ubuntu 24.04
        <span class="star-tag">⭐ PASO A PASO</span>
      </SectionTitle>

      <TextBlock>
        ROS 2 Jazzy Jalisco es la versión LTS (Long-Term Support) lanzada en mayo 2024,
        con soporte hasta mayo 2029. Esta es la distribución que usarás durante todo el curso.
        Sigue los pasos en orden — cada uno es necesario.
      </TextBlock>

      <!-- Prerequisites banner -->
      <div class="prereq-banner q-mt-lg">
        <div class="pb-title">
          <q-icon name="checklist" size="18px" color="primary" />
          Requisitos previos
        </div>
        <div class="pb-items">
          <div v-for="r in prereqs" :key="r" class="pb-item">
            <q-icon name="check_circle" size="14px" color="positive" />
            {{ r }}
          </div>
        </div>
      </div>

      <!-- Installation steps — the hero block -->
      <div class="install-steps q-mt-xl">
        <div v-for="(step, i) in installSteps" :key="i" class="is-step">
          <div class="is-connector" v-if="i > 0"></div>
          <div class="is-card" :style="{ '--is-color': step.color }">
            <div class="is-header">
              <div class="is-num-wrap">
                <div class="is-num" :style="{ background: step.color }">{{ i + 1 }}</div>
                <div class="is-total">/ {{ installSteps.length }}</div>
              </div>
              <div class="is-meta">
                <div class="is-title">{{ step.title }}</div>
                <div class="is-sub">{{ step.sub }}</div>
              </div>
              <div class="is-duration">
                <q-icon name="schedule" size="14px" class="q-mr-xs" />{{ step.time }}
              </div>
            </div>
            <div v-if="step.note" class="is-note">
              <q-icon name="lightbulb" size="14px" color="warning" class="q-mr-xs" />{{ step.note }}
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="step.code" :copyable="true" />
            <div v-if="step.verify" class="is-verify">
              <q-icon name="verified" size="14px" color="positive" class="q-mr-xs" />
              <strong>Verificación:</strong> {{ step.verify }}
            </div>
          </div>
        </div>
      </div>

      <!-- Variants comparison -->
      <div class="ros-variants q-mt-xl">
        <div class="rv-title">
          <q-icon name="layers" size="16px" color="primary" />
          ¿Qué variante instalar?
        </div>
        <div class="rv-grid">
          <div v-for="v in rosVariants" :key="v.name" class="rv-card"
            :class="{ 'rv-recommended': v.recommended }"
            :style="{ '--rv-color': v.color }">
            <div class="rvc-name">{{ v.name }}</div>
            <code class="rvc-pkg">{{ v.pkg }}</code>
            <div class="rvc-includes">
              <div v-for="inc in v.includes" :key="inc" class="rvc-item">
                <q-icon name="check" size="11px" :style="{ color: v.color }" />
                {{ inc }}
              </div>
            </div>
            <div v-if="v.recommended" class="rvc-badge">Recomendado para este curso</div>
          </div>
        </div>
      </div>

      <!-- Verification section -->
      <div class="verify-section q-mt-xl">
        <div class="vs-title">
          <q-icon name="fact_check" size="18px" color="positive" />
          Verificación Final — El Test del Talker/Listener
        </div>
        <TextBlock>
          Este es el "Hola Mundo" de ROS 2. Si funciona, tu instalación es correcta.
          Necesitas <strong>dos terminales simultáneas</strong>.
        </TextBlock>
        <div class="vs-terminals">
          <div class="vst-panel">
            <div class="vst-header" style="background:rgba(74,222,128,.12); color:#4ade80">
              <q-icon name="terminal" size="16px" />
              Terminal 1 — Publicador (Talker)
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="talkerCode" :copyable="true" />
          </div>
          <div class="vst-arrow">
            <q-icon name="swap_horiz" size="28px" style="color:var(--text-muted)" />
            <span>DDS</span>
          </div>
          <div class="vst-panel">
            <div class="vst-header" style="background:rgba(96,165,250,.12); color:#60a5fa">
              <q-icon name="terminal" size="16px" />
              Terminal 2 — Suscriptor (Listener)
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="listenerCode" :copyable="true" />
          </div>
        </div>
        <AlertBlock type="success" title="¡Instalación exitosa! ✓" class="q-mt-md">
          Si ves los mensajes cruzándose entre terminales, ROS 2 Jazzy está perfectamente
          instalado y funcionando. El middleware DDS está comunicando nodos entre sí.
        </AlertBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 6: ROSDEP
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">05</span>
        rosdep — El Gestor de Dependencias ROS
      </SectionTitle>

      <TextBlock>
        <code>rosdep</code> lee los archivos <code>package.xml</code> de tus paquetes
        e instala <strong>todas las dependencias automáticamente</strong> usando APT.
        Indispensable al clonar paquetes de GitHub.
      </TextBlock>

      <SplitBlock class="q-mt-lg">
        <template #left>
          <div class="rosdep-scenario">
            <div class="rs-title">El problema sin rosdep</div>
            <div class="rs-steps">
              <div class="rs-step bad">
                <q-icon name="download" size="14px" color="negative" />
                Clonas paquete de GitHub
              </div>
              <div class="rs-arrow">↓</div>
              <div class="rs-step bad">
                <q-icon name="search" size="14px" color="negative" />
                Buscas dependencias en el README
              </div>
              <div class="rs-arrow">↓</div>
              <div class="rs-step bad">
                <q-icon name="error" size="14px" color="negative" />
                Instalas una por una manualmente
              </div>
              <div class="rs-arrow">↓</div>
              <div class="rs-step bad">
                <q-icon name="bug_report" size="14px" color="negative" />
                Se te olvida alguna → error al compilar
              </div>
            </div>
          </div>
        </template>
        <template #right>
          <div class="rosdep-solution">
            <div class="rs-title good">La solución con rosdep</div>
            <CodeBlock title="Inicializar (solo una vez)" lang="bash"
              content="sudo rosdep init
rosdep update" :copyable="true" />
            <CodeBlock title="Instalar dependencias del workspace" lang="bash"
              content="cd ~/ros2_ws
rosdep install --from-paths src \
  --ignore-src -r -y
# Instala TODO automáticamente" :copyable="true" />
          </div>
        </template>
      </SplitBlock>

      <AlertBlock type="success" title="Pro tip" class="q-mt-md">
        Ejecuta <code>rosdep install</code> cada vez que clones un paquete nuevo de GitHub.
        Te ahorrará horas de depuración por dependencias faltantes.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 7: ERRORES COMUNES
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
         SECCIÓN 8: RETO PRÁCTICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Verifica Tu Instalación Completa</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Checklist de instalación de ROS 2 Jazzy</div>
            <div class="challenge-subtitle">
              Ejecuta cada comando y verifica que el output sea el esperado.
            </div>
          </div>
          <div class="challenge-badge">VERIFICACIÓN</div>
        </div>

        <CodeBlock title="Checklist Completo" lang="bash" :content="checklistCode" :copyable="true" />

        <q-expansion-item icon="lightbulb" label="Ver output esperado"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div class="answer-row"><code class="answer-key">ros2 --version</code><span>muestra la versión de jazzy</span></div>
            <div class="answer-row"><code class="answer-key">$ROS_DISTRO</code><span>imprime <code>jazzy</code></span></div>
            <div class="answer-row"><code class="answer-key">pkg list | grep demo</code><span>muestra demo_nodes_cpp y demo_nodes_py</span></div>
            <div class="answer-row"><code class="answer-key">talker</code><span>publica "Hello World: N" cada segundo</span></div>
            <div class="answer-row"><code class="answer-key">rclpy import</code><span>imprime "rclpy OK" sin errores</span></div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 9: VIDEO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>Instalación de ROS 2 Jazzy en Ubuntu 24.04 en tiempo real:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Instalación ROS 2 Jazzy" frameborder="0"
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
    <div class="section-group">
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
    </div>

    <!-- ══════════════════════════════════════════
         CTA FINAL — FIN DE MÓDULO 0
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <div class="module-complete-card">
        <div class="mcc-bg"></div>
        <div class="mcc-content">
          <div class="mcc-badge">MÓDULO 0 COMPLETADO</div>
          <div class="mcc-title">🎉 ¡Eres un usuario de Linux!</div>
          <div class="mcc-sub">
            Dominaste el sistema de archivos, permisos, editores, variables de entorno
            y tienes ROS 2 Jazzy instalado y funcionando. Estás listo para lo siguiente.
          </div>

          <div class="mcc-stats">
            <div v-for="st in moduleStats" :key="st.label" class="mcc-stat">
              <div class="mcc-stat-num">{{ st.num }}</div>
              <div class="mcc-stat-label">{{ st.label }}</div>
            </div>
          </div>

          <div class="mcc-next">
            <div class="mcc-next-label">Siguiente módulo</div>
            <div class="mcc-next-title">Módulo 1 — Fundamentos de Python para ROS 2</div>
            <div class="mcc-next-desc">
              Aprenderás Python desde cero con enfoque en robótica: estructuras de datos,
              funciones, clases, y cómo escribir tu primer nodo ROS 2.
            </div>
          </div>

          <q-btn
            unelevated
            rounded
            size="lg"
            padding="14px 40px"
            to="/modulo-1/01python-scripts"
            icon="rocket_launch"
            label="Comenzar Módulo 1 — Python para ROS 2"
            class="mcc-btn text-weight-bold q-mt-lg"
            color="positive"
          />
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
  { icon: '📦', label: 'APT — gestor oficial verificado por Ubuntu' },
  { icon: '🐍', label: 'venv — entornos aislados para Python' },
  { icon: '🤖', label: 'ROS 2 Jazzy — LTS hasta mayo 2029' },
];

// ── Install methods ────────────────────────────────────────────
const methods = [
  {
    name: 'APT', sub: 'Advanced Package Tool', badge: 'SISTEMA',
    icon: 'verified', color: '#f97316',
    desc: 'Software verificado por Ubuntu. Estable, con actualizaciones automáticas y gestión de dependencias. Siempre tu primera opción.',
    examples: ['ros-jazzy-desktop', 'git', 'python3-numpy'],
  },
  {
    name: 'PIP + venv', sub: 'Python Package Index', badge: 'PYTHON',
    icon: 'code', color: '#60a5fa',
    desc: 'Librerías Python puras en entornos aislados. Ideal para versiones específicas, IA y proyectos que no entran en APT.',
    examples: ['tensorflow', 'torch', 'opencv-python'],
  },
  {
    name: 'Source', sub: 'Git + colcon', badge: 'AVANZADO',
    icon: 'build', color: '#94a3b8',
    desc: 'Clonas el código fuente y compilas con colcon. Máximo control. Para drivers de robots y paquetes experimentales.',
    examples: ['tu workspace ROS 2', 'drivers de sensores'],
  },
];

// ── APT steps ──────────────────────────────────────────────────
const aptSteps = [
  {
    title: 'Actualizar catálogo de paquetes',
    code: 'sudo apt update\n# Descarga la lista de paquetes disponibles',
    note: 'Hazlo SIEMPRE antes de instalar. Si no, pedirás versiones fantasma.',
    color: '#f97316',
  },
  {
    title: 'Instalar paquete',
    code: 'sudo apt install git\n# APT resuelve dependencias automáticamente',
    note: 'Agrega -y para confirmar sin preguntar: apt install -y git',
    color: '#fbbf24',
  },
  {
    title: 'Actualizar sistema',
    code: 'sudo apt upgrade -y\n# Actualiza TODOS los paquetes instalados',
    note: 'Hazlo semanalmente. Los parches de seguridad son importantes.',
    color: '#4ade80',
  },
];

// ── APT commands ───────────────────────────────────────────────
const aptCmds = [
  { cmd: 'apt search',           color: '#60a5fa', desc: 'Buscar paquetes por nombre o descripción', example: 'apt search opencv' },
  { cmd: 'apt show',             color: '#c084fc', desc: 'Ver info detallada: versión, dependencias, tamaño', example: 'apt show python3-numpy' },
  { cmd: 'apt list --installed', color: '#4ade80', desc: 'Listar todos los paquetes instalados', example: 'apt list --installed | grep ros' },
  { cmd: 'apt remove',           color: '#f87171', desc: 'Desinstalar paquete (no elimina configs)', example: 'sudo apt remove paquete' },
  { cmd: 'apt autoremove',       color: '#fb923c', desc: 'Eliminar dependencias huérfanas', example: 'sudo apt autoremove' },
  { cmd: 'apt-cache policy',     color: '#fbbf24', desc: 'Ver versión instalada vs disponible', example: 'apt-cache policy ros-jazzy-desktop' },
];

// ── PIP options ────────────────────────────────────────────────
const pipOptions = [
  {
    title: 'APT — Librerías Python del sistema', status: '✅ Recomendado',
    icon: 'verified', color: '#4ade80',
    desc: 'Para librerías comunes que Ubuntu distribuye oficialmente. Estable e integrado.',
    code: 'sudo apt install python3-numpy python3-pandas python3-matplotlib\n# Disponible para todo el sistema, sin conflictos',
    noteLabel: 'Ventaja:', note: 'Estable, actualizado, sin riesgo de conflictos',
  },
  {
    title: 'VENV + PIP — Entorno virtual aislado', status: '✅ Flexible',
    icon: 'science', color: '#60a5fa',
    desc: 'Para versiones específicas o librerías que no están en APT (IA, ML).',
    code: '# Crear entorno virtual\npython3 -m venv ~/ai_env\n\n# Activar\nsource ~/ai_env/bin/activate\n\n# Instalar libremente\npip install tensorflow==2.16.0 torch opencv-python\n\n# Desactivar\ndeactivate',
    noteLabel: 'Ventaja:', note: 'Aislado, versiones exactas, sin afectar el sistema',
  },
];

// ── Prerequisites ──────────────────────────────────────────────
const prereqs = [
  'Ubuntu 24.04 LTS (Noble Numbat) instalado',
  'Acceso a internet estable',
  'Cuenta con permisos sudo',
  'Aproximadamente 3 GB de espacio libre',
];

// ── ROS 2 Installation steps ───────────────────────────────────
const installSteps = [
  {
    title: 'Configurar Locale',
    sub: 'ROS 2 requiere UTF-8',
    time: '~1 min',
    color: '#4ade80',
    note: 'Necesario para que los mensajes de ROS 2 se muestren correctamente.',
    code: `sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8`,
    verify: 'locale | grep "LANG=en_US.UTF-8" debe aparecer en la salida',
  },
  {
    title: 'Habilitar el Repositorio Universe',
    sub: 'Acceso a paquetes de la comunidad',
    time: '~30 seg',
    color: '#22d3ee',
    note: undefined,
    code: `sudo apt install software-properties-common -y
sudo add-apt-repository universe`,
    verify: undefined,
  },
  {
    title: 'Agregar la Clave GPG de ROS 2',
    sub: 'Firma de seguridad para verificar paquetes',
    time: '~1 min',
    color: '#fbbf24',
    note: 'Esta clave garantiza que los paquetes descargados son oficiales y no han sido manipulados.',
    code: `sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \\
  -o /usr/share/keyrings/ros-archive-keyring.gpg`,
    verify: 'ls /usr/share/keyrings/ros-archive-keyring.gpg debe mostrar el archivo',
  },
  {
    title: 'Agregar el Repositorio Oficial de ROS 2',
    sub: 'Fuente de paquetes ROS 2',
    time: '~30 seg',
    color: '#fb923c',
    note: undefined,
    code: `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \\
  http://packages.ros.org/ros2/ubuntu \\
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \\
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`,
    verify: 'cat /etc/apt/sources.list.d/ros2.list debe mostrar la URL del repositorio',
  },
  {
    title: 'Actualizar e Instalar ROS 2 Jazzy Desktop',
    sub: 'Instalación principal — el paso más largo',
    time: '~15-30 min',
    color: '#c084fc',
    note: 'ros-jazzy-desktop incluye rviz2, rqt, demos, y todas las librerías de comunicación. Tarda según tu conexión.',
    code: `sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y
sudo apt install ros-dev-tools -y`,
    verify: 'ls /opt/ros/jazzy/ debe mostrar bin/, lib/, share/ y más',
  },
  {
    title: 'Instalar Herramientas de Desarrollo',
    sub: 'colcon, rosdep, y utilidades CLI',
    time: '~3 min',
    color: '#f87171',
    note: undefined,
    code: `sudo apt install -y \\
  python3-colcon-common-extensions \\
  python3-rosdep \\
  python3-vcstool \\
  python3-argcomplete

sudo rosdep init
rosdep update`,
    verify: 'colcon --version y rosdep --version deben mostrar su versión',
  },
  {
    title: 'Configurar .bashrc para Activación Automática',
    sub: 'La configuración permanente del entorno',
    time: '~1 min',
    color: '#4ade80',
    note: 'Este paso hace que cada nueva terminal tenga ROS 2 activo automáticamente sin hacer source manual.',
    code: `echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Aplicar inmediatamente
source ~/.bashrc

# Verificar
echo $ROS_DISTRO`,
    verify: 'echo $ROS_DISTRO debe mostrar: jazzy',
  },
];

// ── ROS 2 variants ─────────────────────────────────────────────
const rosVariants = [
  {
    name: 'Desktop Full', pkg: 'ros-jazzy-desktop-full', color: '#4ade80', recommended: false,
    includes: ['Todo lo de Desktop', 'Gazebo (simulador)', 'ROS 1 bridge', 'Paquetes de percepción'],
  },
  {
    name: 'Desktop', pkg: 'ros-jazzy-desktop', color: '#60a5fa', recommended: true,
    includes: ['ROS 2 core', 'rviz2', 'rqt tools', 'demos y tutoriales', 'nav2 compatible'],
  },
  {
    name: 'Base', pkg: 'ros-jazzy-ros-base', color: '#94a3b8', recommended: false,
    includes: ['ROS 2 core mínimo', 'rclpy / rclcpp', 'Sin herramientas GUI', 'Para robots embebidos'],
  },
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    msg: 'E: Unable to locate package ros-jazzy-desktop',
    summary: 'El repositorio de ROS 2 no está configurado o no se actualizó',
    cause: 'No completaste el paso de agregar el repositorio, o falta sudo apt update después.',
    steps: [
      'Verifica que el archivo existe: cat /etc/apt/sources.list.d/ros2.list',
      'Ejecuta: sudo apt update',
      'Si el archivo no existe, repite el paso 4 de la instalación',
    ],
    code: 'cat /etc/apt/sources.list.d/ros2.list\nsudo apt update\nsudo apt install ros-jazzy-desktop',
    open: false,
  },
  {
    msg: 'error: externally-managed-environment',
    summary: 'pip install global bloqueado en Ubuntu 24.04+',
    cause: 'Ubuntu 24.04 implementa PEP 668 que protege el entorno del sistema operativo.',
    steps: [
      'Opción 1 (recomendada): usa APT — sudo apt install python3-nombre-paquete',
      'Opción 2: crea un entorno virtual — python3 -m venv mi_env && source mi_env/bin/activate',
      'Nunca uses pip install sin un venv activo ni sudo pip install',
    ],
    code: '# APT (preferido)\nsudo apt install python3-numpy\n\n# O VENV\npython3 -m venv ~/mi_env\nsource ~/mi_env/bin/activate\npip install numpy',
    open: false,
  },
  {
    msg: "bash: ros2: command not found",
    summary: 'ROS 2 instalado pero no activado en la terminal',
    cause: 'No has hecho source del setup.bash o no está en .bashrc.',
    steps: [
      'Temporal: source /opt/ros/jazzy/setup.bash',
      'Permanente: echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc',
      'Aplica: source ~/.bashrc',
    ],
    code: 'source /opt/ros/jazzy/setup.bash\necho $ROS_DISTRO  # jazzy',
    open: false,
  },
  {
    msg: 'rosdep: command not found',
    summary: 'Las herramientas de desarrollo de ROS no están instaladas',
    cause: 'Falta instalar python3-rosdep o ros-dev-tools.',
    steps: [
      'sudo apt install python3-rosdep ros-dev-tools',
      'sudo rosdep init',
      'rosdep update',
    ],
    code: 'sudo apt install python3-rosdep ros-dev-tools -y\nsudo rosdep init\nrosdep update',
    open: false,
  },
  {
    msg: 'GPG error: EXPKEYSIG — repository signature invalid',
    summary: 'La clave GPG del repositorio expiró',
    cause: 'Las claves GPG tienen fecha de expiración. Necesitas renovarla.',
    steps: [
      'Elimina la clave antigua: sudo rm /usr/share/keyrings/ros-archive-keyring.gpg',
      'Descarga la clave actualizada (paso 3 de la instalación)',
      'sudo apt update',
    ],
    code: 'sudo rm /usr/share/keyrings/ros-archive-keyring.gpg\nsudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \\\n  -o /usr/share/keyrings/ros-archive-keyring.gpg\nsudo apt update',
    open: false,
  },
]);

// ── Summary ────────────────────────────────────────────────────
const summaryItems = [
  { cmd: 'sudo apt update',    desc: 'Actualizar catálogo de paquetes',          example: 'siempre antes de instalar',              color: '#f97316' },
  { cmd: 'sudo apt install',   desc: 'Instalar software verificado del sistema',  example: 'sudo apt install ros-jazzy-desktop',     color: '#fbbf24' },
  { cmd: 'python3 -m venv',    desc: 'Crear entorno virtual Python aislado',      example: 'python3 -m venv ~/mi_env',               color: '#60a5fa' },
  { cmd: 'rosdep install',     desc: 'Instalar todas las dependencias de ROS 2', example: 'rosdep install --from-paths src -y',      color: '#c084fc' },
  { cmd: 'source setup.bash',  desc: 'Activar ROS 2 en la terminal actual',      example: 'source /opt/ros/jazzy/setup.bash',        color: '#4ade80' },
  { cmd: 'colcon build',       desc: 'Compilar paquetes de tu workspace',         example: 'colcon build --symlink-install',          color: '#22d3ee' },
];

// ── Inline code strings (pulled out of template to avoid Vue parser issues) ──
const neverCode = [
  'pip install numpy        # ❌ Bloqueado en Ubuntu 24.04+',
  'sudo pip install numpy   # ❌ PELIGROSO: rompe dependencias del sistema',
].join('\n');

const talkerCode = [
  'source /opt/ros/jazzy/setup.bash',
  'ros2 run demo_nodes_cpp talker',
  '# Resultado esperado:',
  '# [INFO] Publishing: \'Hello World: 1\'',
  '# [INFO] Publishing: \'Hello World: 2\'',
  '# ...',
].join('\n');

const listenerCode = [
  'source /opt/ros/jazzy/setup.bash',
  'ros2 run demo_nodes_py listener',
  '# Resultado esperado:',
  '# [INFO] I heard: [Hello World: 1]',
  '# [INFO] I heard: [Hello World: 2]',
  '# ...',
].join('\n');

const checklistCode = [
  '# ── 1. Verificar versión de ROS 2 ──────────────',
  'source /opt/ros/jazzy/setup.bash',
  'ros2 --version',
  '# ros2 cli version 0.x.x (jazzy)',
  '',
  '# ── 2. Verificar variable de entorno ────────────',
  'echo $ROS_DISTRO',
  '# jazzy',
  '',
  '# ── 3. Listar paquetes de demo ──────────────────',
  'ros2 pkg list | grep demo',
  '# demo_nodes_cpp',
  '# demo_nodes_py',
  '',
  '# ── 4. Test talker (Ctrl+C después de 3 msgs) ───',
  'ros2 run demo_nodes_cpp talker',
  '',
  '# ── 5. Verificar tools de desarrollo ────────────',
  'colcon version-check',
  "python3 -c 'import rclpy; print(\"rclpy OK\")'",
  '',
  '# ── 6. Verificar rosdep ─────────────────────────',
  'rosdep --version',
  '',
  '# ── 7. Verificar que .bashrc tiene el source ────',
  'grep ros ~/.bashrc',
  '# source /opt/ros/jazzy/setup.bash  <- debe aparecer',
].join('\n');

// ── Module completion stats ────────────────────────────────────
const moduleStats = [
  { num: '6', label: 'Lecciones\ncompletadas' },
  { num: '30+', label: 'Comandos\naprendidos' },
  { num: '1', label: 'ROS 2\ninstalado' },
  { num: '∞', label: 'Potencial\ndesbloqueado' },
];
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background:var(--bg-code); color:var(--text-code);
  padding:2px 7px; border-radius:5px;
  font-family:'Fira Code',monospace; font-size:.9em;
}
.cmd-badge {
  display:inline-flex; align-items:center; justify-content:center;
  width:28px; height:28px; border-radius:8px;
  font-size:.75rem; font-weight:800; margin-right:8px; vertical-align:middle;
}
.cmd-badge.green  { background:rgba( 74,222,128,.15); color:#4ade80; }
.cmd-badge.cyan   { background:rgba( 34,211,238,.15); color:#22d3ee; }
.cmd-badge.amber  { background:rgba(251,191, 36,.15); color:#fbbf24; }
.cmd-badge.purple { background:rgba(192,132,252,.15); color:#c084fc; }
.cmd-badge.red    { background:rgba(248,113,113,.15); color:#f87171; }
.star-tag {
  display:inline-flex; align-items:center; font-size:.65rem; font-weight:800;
  letter-spacing:.08em; background:rgba(251,191,36,.15); color:#fbbf24;
  border:1px solid rgba(251,191,36,.3); padding:2px 8px; border-radius:999px;
  margin-left:10px; vertical-align:middle;
}

/* ── Fact pills */
.fact-pills { display:flex; gap:10px; flex-wrap:wrap; }
.fact-pill {
  display:flex; align-items:center; gap:8px;
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:999px; padding:7px 16px; font-size:.84rem; color:var(--text-secondary);
  transition:transform .2s;
}
.fact-pill:hover { transform:translateY(-2px); }
.fact-icon { font-size:1rem; }

/* ══════════════════════════════════════════
   METHODS GRID (3-col)
══════════════════════════════════════════ */
.methods-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:16px; }
.method-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-top:4px solid var(--mc-color,var(--border-medium));
  border-radius:16px; padding:1.5rem;
  display:flex; flex-direction:column; gap:10px; transition:all .25s;
}
.method-card:hover { transform:translateY(-5px); box-shadow:var(--shadow-md); }
.mc-top { display:flex; align-items:flex-start; justify-content:space-between; }
.mc-icon-wrap {
  width:48px; height:48px; border-radius:12px;
  display:flex; align-items:center; justify-content:center;
}
.mc-badge {
  font-size:.65rem; font-weight:800; letter-spacing:.08em;
  padding:3px 10px; border-radius:999px;
}
.mc-name { font-size:1.35rem; font-weight:800; color:var(--text-primary); }
.mc-sub  { font-size:.82rem; color:var(--text-muted); font-style:italic; }
.mc-desc { font-size:.88rem; color:var(--text-secondary); line-height:1.55; flex:1; }
.mc-examples {
  display:flex; flex-wrap:wrap; align-items:center; gap:6px;
  background:var(--bg-surface-hover); border-radius:8px; padding:8px 10px;
  font-size:.8rem;
}
.mce-label { color:var(--text-muted); font-size:.78rem; }
.mc-examples code { background:none; padding:0; color:var(--mc-color); font-size:.82rem; }

/* ══════════════════════════════════════════
   APT TIMELINE
══════════════════════════════════════════ */
.apt-timeline { display:flex; flex-direction:column; }
.at-step { display:flex; flex-direction:column; }
.at-connector { width:2px; height:18px; background:var(--border-medium); margin-left:17px; }
.at-row { display:flex; align-items:flex-start; gap:14px; }
.at-num {
  min-width:36px; width:36px; height:36px; border-radius:50%; flex-shrink:0;
  display:flex; align-items:center; justify-content:center;
  font-size:1rem; font-weight:800; color:#1e1e1e; margin-top:2px;
}
.at-body {
  flex:1; background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:12px; padding:1.1rem; margin-bottom:0; display:flex; flex-direction:column; gap:8px;
}
.at-title { font-size:1rem; font-weight:700; color:var(--text-primary); }
.at-note {
  display:flex; align-items:center; font-size:.8rem; color:var(--text-muted);
  background:var(--bg-surface-hover); border-radius:7px; padding:6px 10px;
}

/* ── APT commands grid */
.apt-cmds-grid { }
.acg-title {
  display:flex; align-items:center; gap:8px; font-size:.9rem; font-weight:600;
  color:var(--text-secondary); margin-bottom:12px;
}
.acg-cards { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }
.acg-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:3px solid var(--acg-color,var(--border-medium));
  border-radius:10px; padding:1rem; display:flex; flex-direction:column; gap:6px; min-width:0;
}
.acg-cmd  { font-family:'Fira Code',monospace; font-size:.9rem; font-weight:700; color:var(--acg-color); background:none; padding:0; }
.acg-desc { font-size:.8rem; color:var(--text-secondary); line-height:1.4; }

/* ══════════════════════════════════════════
   PIP OPTIONS
══════════════════════════════════════════ */
.pip-options { display:grid; grid-template-columns:repeat(2,1fr); gap:16px; }
.pip-option {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-top:3px solid var(--po-color,var(--border-medium));
  border-radius:14px; padding:1.25rem; display:flex; flex-direction:column; gap:10px; min-width:0;
}
.po-header { display:flex; align-items:center; gap:12px; }
.po-icon-wrap { width:40px; height:40px; border-radius:10px; display:flex; align-items:center; justify-content:center; flex-shrink:0; }
.po-status { font-size:.75rem; font-weight:800; }
.po-title  { font-size:.95rem; font-weight:700; color:var(--text-primary); }
.po-desc   { font-size:.87rem; color:var(--text-secondary); line-height:1.5; }
.po-note {
  font-size:.82rem; border:1px solid; border-radius:8px; padding:8px 12px;
  display:flex; gap:4px; align-items:baseline;
}

.never-card {
  background:var(--bg-surface); border:2px solid rgba(248,113,113,.35);
  border-radius:14px; overflow:hidden;
}
.nc-header {
  display:flex; align-items:center; gap:8px; padding:.75rem 1rem;
  background:rgba(248,113,113,.1); border-bottom:1px solid rgba(248,113,113,.25);
  font-size:.9rem; font-weight:700; color:#f87171;
}

/* ══════════════════════════════════════════
   PREREQUISITES BANNER
══════════════════════════════════════════ */
.prereq-banner {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid #60a5fa; border-radius:12px; padding:1.1rem 1.25rem;
}
.pb-title {
  display:flex; align-items:center; gap:8px; font-size:.9rem; font-weight:700;
  color:var(--text-primary); margin-bottom:10px;
}
.pb-items { display:flex; flex-wrap:wrap; gap:8px; }
.pb-item {
  display:flex; align-items:center; gap:6px; font-size:.85rem; color:var(--text-secondary);
  background:var(--bg-surface-hover); border-radius:999px; padding:4px 14px;
}

/* ══════════════════════════════════════════
   INSTALLATION STEPS — HERO BLOCK
══════════════════════════════════════════ */
.install-steps { display:flex; flex-direction:column; }
.is-step { display:flex; flex-direction:column; }
.is-connector {
  width:3px; height:24px; margin-left:19px;
  background:linear-gradient(180deg, var(--border-medium), var(--border-subtle));
}
.is-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid var(--is-color,var(--border-medium));
  border-radius:16px; padding:1.5rem; display:flex; flex-direction:column; gap:10px; min-width:0;
  transition:box-shadow .25s;
}
.is-card:hover { box-shadow:var(--shadow-md); }
.is-header {
  display:flex; align-items:flex-start; gap:14px; flex-wrap:wrap;
}
.is-num-wrap { display:flex; align-items:center; gap:4px; flex-shrink:0; }
.is-num {
  min-width:42px; width:42px; height:42px; border-radius:50%;
  display:flex; align-items:center; justify-content:center;
  font-size:1.2rem; font-weight:900; color:#1e1e1e; flex-shrink:0;
}
.is-total { font-size:.75rem; color:var(--text-muted); margin-top:auto; padding-bottom:2px; }
.is-meta { flex:1; min-width:0; }
.is-title { font-size:1.05rem; font-weight:800; color:var(--text-primary); margin-bottom:2px; }
.is-sub   { font-size:.82rem; color:var(--text-muted); }
.is-duration {
  display:flex; align-items:center; font-size:.78rem; color:var(--text-muted);
  background:var(--bg-surface-hover); border-radius:999px; padding:3px 10px; white-space:nowrap;
  margin-left:auto;
}
.is-note {
  display:flex; align-items:flex-start; gap:6px;
  background:rgba(251,191,36,.08); border:1px solid rgba(251,191,36,.2);
  border-radius:8px; padding:8px 12px; font-size:.84rem; color:var(--text-secondary);
}
.is-verify {
  display:flex; align-items:center; gap:6px;
  background:rgba(74,222,128,.08); border-radius:8px; padding:8px 12px;
  font-size:.84rem; color:var(--text-secondary);
}

/* ROS 2 variants */
.ros-variants { }
.rv-title {
  display:flex; align-items:center; gap:8px; font-size:.9rem; font-weight:600;
  color:var(--text-secondary); margin-bottom:12px;
}
.rv-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }
.rv-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-top:3px solid var(--rv-color,var(--border-medium));
  border-radius:12px; padding:1.1rem; display:flex; flex-direction:column; gap:8px;
  position:relative; transition:all .25s;
}
.rv-card:hover { transform:translateY(-3px); box-shadow:var(--shadow-sm); }
.rv-card.rv-recommended {
  border-color: var(--rv-color); box-shadow: 0 0 0 2px color-mix(in srgb, var(--rv-color) 20%, transparent);
}
.rvc-name { font-size:1rem; font-weight:700; color:var(--text-primary); }
.rvc-pkg  { font-family:'Fira Code',monospace; font-size:.78rem; color:var(--rv-color); background:none; padding:0; }
.rvc-includes { display:flex; flex-direction:column; gap:4px; margin-top:4px; }
.rvc-item { display:flex; align-items:center; gap:5px; font-size:.8rem; color:var(--text-secondary); }
.rvc-badge {
  font-size:.68rem; font-weight:700; letter-spacing:.06em;
  background:color-mix(in srgb,var(--rv-color) 15%,transparent);
  color:var(--rv-color); border-radius:999px; padding:2px 10px; text-align:center;
}

/* Verification terminals */
.verify-section { }
.vs-title {
  display:flex; align-items:center; gap:8px; font-size:1.05rem; font-weight:700;
  color:var(--text-primary); margin-bottom:1rem;
}
.vs-terminals { display:grid; grid-template-columns:1fr auto 1fr; gap:12px; align-items:start; }
.vst-panel { background:var(--bg-surface); border:1px solid var(--border-subtle); border-radius:12px; overflow:hidden; }
.vst-header {
  display:flex; align-items:center; gap:8px; padding:10px 14px;
  border-bottom:1px solid var(--border-subtle); font-size:.85rem; font-weight:700;
}
.vst-arrow {
  display:flex; flex-direction:column; align-items:center; gap:4px;
  padding-top:2.5rem; font-size:.75rem; color:var(--text-muted);
}

/* ══════════════════════════════════════════
   ROSDEP
══════════════════════════════════════════ */
.rosdep-scenario, .rosdep-solution { display:flex; flex-direction:column; gap:10px; height:100%; }
.rs-title { font-size:.95rem; font-weight:700; color:var(--text-primary); margin-bottom:4px; }
.rs-title.good { color:#4ade80; }
.rs-steps { display:flex; flex-direction:column; gap:4px; }
.rs-step {
  display:flex; align-items:center; gap:8px; padding:8px 12px;
  background:var(--bg-surface); border:1px solid var(--border-subtle); border-radius:8px;
  font-size:.85rem; color:var(--text-secondary);
}
.rs-step.bad { border-color:rgba(248,113,113,.25); }
.rs-arrow { color:var(--text-muted); text-align:center; font-size:1rem; padding:2px 0; }

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
  background:rgba(96,165,250,.12); color:#60a5fa; border:1px solid rgba(96,165,250,.3); white-space:nowrap;
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
.answer-key { font-family:'Fira Code',monospace; font-weight:700; color:#4ade80; background:none; padding:0; }

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
   MODULE COMPLETE CARD — CTA FINAL
══════════════════════════════════════════ */
.module-complete-card {
  position:relative; border-radius:24px; overflow:hidden;
  border:1px solid rgba(74,222,128,.25);
  box-shadow: 0 0 60px rgba(74,222,128,.1), 0 0 120px rgba(34,211,238,.05);
}
.mcc-bg {
  position:absolute; inset:0; pointer-events:none;
  background: radial-gradient(circle at 20% 50%, rgba(74,222,128,.12) 0%, transparent 60%),
              radial-gradient(circle at 80% 50%, rgba(34,211,238,.08) 0%, transparent 60%);
}
.mcc-content {
  position:relative; padding:3rem 2.5rem; display:flex; flex-direction:column; align-items:center; gap:1.25rem;
  text-align:center; background:var(--bg-surface);
}
.mcc-badge {
  font-size:.72rem; font-weight:900; letter-spacing:.12em;
  background:rgba(74,222,128,.15); color:#4ade80;
  border:1px solid rgba(74,222,128,.3); padding:4px 16px; border-radius:999px;
}
.mcc-title { font-size:2rem; font-weight:900; color:var(--text-primary); line-height:1.2; }
.mcc-sub   { font-size:1rem; color:var(--text-secondary); line-height:1.6; max-width:600px; }
.mcc-stats {
  display:flex; gap:2rem; flex-wrap:wrap; justify-content:center; margin:1rem 0;
}
.mcc-stat { display:flex; flex-direction:column; align-items:center; gap:4px; }
.mcc-stat-num {
  font-size:2.5rem; font-weight:900;
  background:linear-gradient(135deg,#4ade80,#22d3ee); -webkit-background-clip:text;
  -webkit-text-fill-color:transparent; background-clip:text; line-height:1;
}
.mcc-stat-label { font-size:.75rem; color:var(--text-muted); white-space:pre-line; text-align:center; }
.mcc-next {
  background:var(--bg-surface-hover); border:1px solid var(--border-subtle);
  border-radius:16px; padding:1.25rem 1.5rem; max-width:580px; text-align:left; width:100%;
}
.mcc-next-label { font-size:.72rem; font-weight:700; letter-spacing:.08em; color:var(--text-muted); text-transform:uppercase; margin-bottom:4px; }
.mcc-next-title { font-size:1.05rem; font-weight:800; color:var(--text-primary); margin-bottom:6px; }
.mcc-next-desc  { font-size:.88rem; color:var(--text-secondary); line-height:1.5; }
.mcc-btn { background:linear-gradient(135deg,#4ade80,#22c55e) !important; }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .acg-cards { grid-template-columns:repeat(2,1fr); }
}

@media (max-width: 900px) {
  .methods-grid  { grid-template-columns:1fr 1fr; }
  .rv-grid       { grid-template-columns:1fr 1fr; }
  .pip-options   { grid-template-columns:1fr; }
  .acg-cards     { grid-template-columns:repeat(2,1fr); }
  .summary-grid  { grid-template-columns:1fr 1fr; }
  .vs-terminals  { grid-template-columns:1fr; }
  .vst-arrow     { flex-direction:row; padding:0; justify-content:center; }
}

@media (max-width: 768px) {
  .methods-grid  { grid-template-columns:1fr; }
  .rv-grid       { grid-template-columns:1fr; }
  .acg-cards     { grid-template-columns:1fr; }
  .fact-pills    { flex-direction:column; gap:8px; }
  .fact-pill     { border-radius:12px; }
  .is-header     { flex-direction:column; align-items:flex-start; }
  .is-duration   { margin-left:0; }
  .challenge-header { flex-direction:column; }
  .challenge-badge  { margin-left:0; }
  .mcc-content   { padding:2rem 1.25rem; }
  .mcc-title     { font-size:1.5rem; }
  .mcc-stats     { gap:1.25rem; }
  .mcc-stat-num  { font-size:2rem; }
}

@media (max-width: 480px) {
  .summary-grid  { grid-template-columns:1fr; }
  .prereq-banner .pb-items { flex-direction:column; }
  .pb-item       { border-radius:8px; }
}
</style>
