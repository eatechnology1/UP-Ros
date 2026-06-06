<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         SECCIÓN 1: CONTEXTO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Linux no es una democracia — es una <strong>meritocracia militarizada</strong>.
        Cada archivo tiene un guardia que verifica tu credencial antes de dejarte pasar.
        Aprende quién tiene la llave nuclear (Root) y cómo autorizar a tu robot para acceder
        al hardware sin comprometer la seguridad del sistema.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>

      <AlertBlock type="info" title="¿Por qué importa en Robótica?">
        Tu robot necesita acceso a puertos USB (lidar, cámara, GPS), archivos de configuración
        críticos, y ejecutar scripts Python automáticamente.<br /><br />
        <strong>Sin permisos correctos, tu código compilará perfectamente pero no correrá.</strong>
        Este es uno de los errores más comunes en robots físicos.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 2: SISTEMA DE TRES NIVELES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        El Sistema de Tres Niveles
      </SectionTitle>

      <TextBlock>
        Cada archivo en Linux tiene permisos para <strong>tres categorías</strong> de usuarios,
        definidas en orden estricto.
      </TextBlock>

      <div class="levels-grid q-mt-lg">
        <div v-for="l in levels" :key="l.title" class="level-card" :style="{ '--lc-color': l.color }">
          <div class="lc-icon-wrap" :style="{ background: l.color + '18' }">
            <q-icon :name="l.icon" size="32px" :style="{ color: l.color }" />
          </div>
          <div class="lc-title">{{ l.title }}</div>
          <div class="lc-sub">{{ l.sub }}</div>
          <div class="lc-example">
            <code>{{ l.example }}</code>
          </div>
        </div>
      </div>

      <!-- Permission symbols -->
      <div class="q-mt-xl">
        <div class="sub-title">Tres permisos por nivel:</div>
        <div class="perms-row q-mt-md">
          <div v-for="p in permSymbols" :key="p.sym" class="perm-pill" :style="{ '--pp-color': p.color }">
            <div class="perm-sym">{{ p.sym }}</div>
            <div class="perm-right">
              <div class="perm-name">{{ p.name }}</div>
              <div class="perm-val">valor: <strong>{{ p.val }}</strong></div>
              <div class="perm-file-dir">
                <span><q-icon name="description" size="11px" /> {{ p.forFile }}</span>
                <span><q-icon name="folder" size="11px" /> {{ p.forDir }}</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 3: DECODIFICADOR ls -l
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">02</span>
        Decodificando <code class="cmd-inline">ls -l</code>
      </SectionTitle>

      <TextBlock>
        Cuando ejecutas <code>ls -l</code> ves una cadena de 10 caracteres al inicio de cada línea.
        Vamos a diseccionarla segmento por segmento.
      </TextBlock>

      <!-- Visual decoder -->
      <div class="perm-decoder q-mt-lg">
        <div class="pd-terminal-line">
          <span class="pd-seg file-type">-</span>
          <span class="pd-seg owner-seg">rwx</span>
          <span class="pd-seg group-seg">r-x</span>
          <span class="pd-seg other-seg">r--</span>
          <span class="pd-rest"> 1 alexander dialout 2048 Jan 10 14:30 <strong>robot.py</strong></span>
        </div>

        <div class="pd-legend">
          <div class="pdl-item">
            <div class="pdl-bracket file-type-br">
              <div class="pdl-brace">└</div>
              <div class="pdl-label">Tipo<br /><span>- archivo<br />d directorio<br />l symlink</span></div>
            </div>
          </div>
          <div class="pdl-item">
            <div class="pdl-bracket owner-br">
              <div class="pdl-brace">└──</div>
              <div class="pdl-label">Propietario<br /><span>r w x<br />Leer+Escribir+Ejecutar</span></div>
            </div>
          </div>
          <div class="pdl-item">
            <div class="pdl-bracket group-br">
              <div class="pdl-brace">└──</div>
              <div class="pdl-label">Grupo<br /><span>r - x<br />Leer+Ejecutar</span></div>
            </div>
          </div>
          <div class="pdl-item">
            <div class="pdl-bracket other-br">
              <div class="pdl-brace">└──</div>
              <div class="pdl-label">Otros<br /><span>r - -<br />Solo leer</span></div>
            </div>
          </div>
        </div>
      </div>

      <!-- Quick-decode table -->
      <div class="decode-table q-mt-xl">
        <div class="dt-header">
          <q-icon name="table_chart" size="15px" color="primary" />
          Tabla de Referencia Rápida
        </div>
        <div class="dt-rows">
          <div v-for="d in decodeTable" :key="d.str" class="dt-row">
            <code class="dt-str" :style="{ color: d.color }">{{ d.str }}</code>
            <span class="dt-type">{{ d.type }}</span>
            <span class="dt-who">{{ d.who }}</span>
            <span class="dt-desc">{{ d.desc }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 4: CHMOD
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">03</span>
        Cambiar Permisos — <code class="cmd-inline">chmod</code>
      </SectionTitle>

      <TextBlock>
        <code>chmod</code> tiene dos formas de especificar permisos. La numérica (octal) es más
        compacta; la simbólica es más legible.
      </TextBlock>

      <div class="chmod-grid q-mt-lg">
        <!-- Numeric method -->
        <div class="chmod-card" style="--ck-color: #c084fc">
          <div class="ck-header">
            <q-icon name="tag" size="20px" style="color:#c084fc" />
            <span class="ck-title">Método Numérico (Octal)</span>
          </div>
          <p class="ck-desc">Suma los valores de cada permiso para cada nivel (Owner, Group, Others).</p>

          <div class="octal-table">
            <div class="ot-row header">
              <span>Nº</span><span>Permisos</span><span>Cálculo</span><span>Descripción</span>
            </div>
            <div v-for="o in octalRef" :key="o.num" class="ot-row">
              <code :style="{ color: o.color }">{{ o.num }}</code>
              <code class="ot-perm" :style="{ color: o.color }">{{ o.perm }}</code>
              <span class="ot-calc">{{ o.calc }}</span>
              <span class="ot-desc">{{ o.desc }}</span>
            </div>
          </div>

          <CodeBlock lang="bash"
            content="# Ejemplo: 755 = owner(7) group(5) others(5)
chmod 755 script.py
chmod 644 config.yaml
chmod 600 secrets.key" :copyable="true" />
        </div>

        <!-- Symbolic method -->
        <div class="chmod-card" style="--ck-color: #22d3ee">
          <div class="ck-header">
            <q-icon name="abc" size="20px" style="color:#22d3ee" />
            <span class="ck-title">Método Simbólico</span>
          </div>
          <p class="ck-desc">Más legible. Usa letras para el target y operadores para agregar/quitar.</p>

          <div class="sym-table">
            <div class="sym-row"><code style="color:#4ade80">u</code><span>user (propietario)</span></div>
            <div class="sym-row"><code style="color:#60a5fa">g</code><span>group (grupo)</span></div>
            <div class="sym-row"><code style="color:#94a3b8">o</code><span>others (resto)</span></div>
            <div class="sym-row"><code style="color:#f59e0b">a</code><span>all (todos)</span></div>
            <div class="sym-divider"></div>
            <div class="sym-row"><code style="color:#4ade80">+</code><span>agregar permiso</span></div>
            <div class="sym-row"><code style="color:#f87171">-</code><span>quitar permiso</span></div>
            <div class="sym-row"><code style="color:#fbbf24">=</code><span>establecer exacto</span></div>
          </div>

          <CodeBlock lang="bash"
            content="# Agregar ejecución al propietario
chmod u+x mi_nodo.py

# Quitar escritura al grupo
chmod g-w config.yaml

# Dar lectura a todos
chmod a+r readme.txt

# Establecer exactamente
chmod u=rwx,go=rx script.sh" :copyable="true" />
        </div>
      </div>

      <!-- Common chmod values -->
      <div class="chmod-common q-mt-xl">
        <div class="cc-title">
          <q-icon name="bookmark" size="15px" color="warning" />
          Valores Más Usados en Robótica
        </div>
        <div class="cc-grid">
          <div v-for="c in commonChmod" :key="c.val" class="cc-card" :style="{ '--cc-color': c.color }">
            <code class="cc-val">{{ c.val }}</code>
            <div class="cc-perm">{{ c.perm }}</div>
            <div class="cc-use">{{ c.use }}</div>
          </div>
        </div>
      </div>

      <AlertBlock type="warning" title="Nunca uses chmod 777" class="q-mt-lg">
        <code>chmod 777</code> da permisos <strong>totales a TODO EL MUNDO</strong>.
        Es como dejar tu robot sin seguridad en una red pública.<br /><br />
        Usa <code>755</code> para scripts ejecutables y <code>644</code> para archivos de datos.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 5: CASOS DE USO ROS 2
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">04</span>
        Casos de Uso en ROS 2
      </SectionTitle>

      <div class="usecase-grid q-mt-lg">
        <div v-for="u in useCases" :key="u.title" class="usecase-card" :style="{ '--uc-color': u.color }">
          <div class="uc-header">
            <div class="uc-icon-wrap" :style="{ background: u.color + '18' }">
              <q-icon :name="u.icon" size="22px" :style="{ color: u.color }" />
            </div>
            <div class="uc-title">{{ u.title }}</div>
          </div>
          <p class="uc-desc">{{ u.desc }}</p>
          <CodeBlock :hide-header="true" lang="bash" :content="u.code" />
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 6: SUDO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        SUDO — Superpoderes Temporales
      </SectionTitle>

      <TextBlock>
        <code>sudo</code> (SuperUser DO) ejecuta un comando con permisos de administrador (root).
        No lo uses por default — úsalo solo cuando el sistema lo requiera.
      </TextBlock>

      <SplitBlock class="q-mt-lg">
        <template #left>
          <AlertBlock type="success" title="Cuándo SÍ usar sudo">
            <div class="sudo-list">
              <div class="sudo-item" v-for="s in sudoDo" :key="s.cmd">
                <code>{{ s.cmd }}</code>
                <span>{{ s.desc }}</span>
              </div>
            </div>
          </AlertBlock>
        </template>
        <template #right>
          <AlertBlock type="danger" title="Cuándo NUNCA usar sudo">
            <div class="sudo-list">
              <div class="sudo-item" v-for="s in sudoDont" :key="s.cmd">
                <code>{{ s.cmd }}</code>
                <span>{{ s.desc }}</span>
              </div>
            </div>
          </AlertBlock>
        </template>
      </SplitBlock>

      <div class="sudo-explain q-mt-lg">
        <div class="se-header">
          <q-icon name="help_outline" size="18px" color="warning" />
          ¿Por qué nunca <code>sudo colcon build</code>?
        </div>
        <div class="se-body">
          <div class="se-step">
            <div class="se-num">1</div>
            <div>Compilas con <code>sudo colcon build</code></div>
          </div>
          <div class="se-arrow">→</div>
          <div class="se-step">
            <div class="se-num">2</div>
            <div>Los archivos en <code>build/</code> e <code>install/</code> pertenecen a <strong>root</strong></div>
          </div>
          <div class="se-arrow">→</div>
          <div class="se-step warn">
            <div class="se-num">3</div>
            <div>Tu usuario normal <strong>no puede modificarlos</strong> — workspace roto</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         SECCIÓN 7: GRUPOS DE HARDWARE
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">06</span>
        Grupos de Linux — Acceso a Hardware
      </SectionTitle>

      <TextBlock>
        Los dispositivos de hardware (USB, cámaras, GPU) pertenecen a grupos del sistema.
        Para acceder a ellos <strong>sin sudo</strong>, tu usuario debe estar en el grupo correcto.
      </TextBlock>

      <div class="groups-grid q-mt-lg">
        <div v-for="g in groups" :key="g.name" class="group-card" :style="{ '--gc-color': g.color }">
          <div class="gc-top">
            <q-icon :name="g.icon" size="22px" :style="{ color: g.color }" />
            <code class="gc-name">{{ g.name }}</code>
          </div>
          <div class="gc-devices">{{ g.devices }}</div>
          <div class="gc-note">{{ g.note }}</div>
        </div>
      </div>

      <!-- How to add to group -->
      <div class="group-howto q-mt-xl">
        <div class="gh-title">
          <q-icon name="person_add" size="16px" color="primary" />
          Cómo agregar tu usuario a un grupo
        </div>
        <CodeBlock title="Ejemplo: acceso a puertos seriales (Lidar/Arduino)" lang="bash"
          content="# 1. Verificar tus grupos actuales
groups

# 2. Agregar al grupo dialout
sudo usermod -aG dialout $USER

# 3. También puedes necesitar el grupo tty
sudo usermod -aG tty $USER

# 4. Verificar que se agregó
grep dialout /etc/group

# 5. REINICIA la sesión (obligatorio!)
# Los grupos solo se aplican en una nueva sesión" :copyable="true" />
      </div>

      <AlertBlock type="warning" title="Reinicio obligatorio" class="q-mt-md">
        Después de agregar tu usuario a un grupo, <strong>debes reiniciar el PC</strong>
        (o al menos cerrar y abrir sesión) para que los cambios surtan efecto.
        Verifica con el comando <code>groups</code>.
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
              style="color: var(--text-muted)" />
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
      <SectionTitle>Reto de Ingeniería — Configurar un Workspace Seguro</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Configura permisos correctos para un proyecto ROS 2</div>
            <div class="challenge-subtitle">
              Aplica lo aprendido para proteger archivos críticos y hacer ejecutables los scripts.
            </div>
          </div>
          <div class="challenge-badge">NIVEL BÁSICO</div>
        </div>

        <CodeBlock title="Reto de Permisos" lang="bash"
          content="# 1. Crea la estructura del proyecto
mkdir -p ~/perm_lab/{scripts,config,logs}
touch ~/perm_lab/scripts/control_node.py
touch ~/perm_lab/config/nav2_params.yaml
touch ~/perm_lab/config/secrets.key
touch ~/perm_lab/logs/debug.log

# 2. Verifica los permisos actuales
ls -la ~/perm_lab/scripts/
ls -la ~/perm_lab/config/

# 3. Haz ejecutable el script de Python
chmod +x ~/perm_lab/scripts/control_node.py

# 4. Protege el archivo de configuración (solo lectura)
chmod 444 ~/perm_lab/config/nav2_params.yaml

# 5. Protege el archivo de secretos (solo el owner)
chmod 600 ~/perm_lab/config/secrets.key

# 6. Limita logs (no ejecutable, no escritura pública)
chmod 644 ~/perm_lab/logs/debug.log

# 7. Verifica todos los permisos resultantes
ls -la ~/perm_lab/scripts/
ls -la ~/perm_lab/config/

# ¿Puedes 'ejecutar' el control_node.py?
# ./~/perm_lab/scripts/control_node.py" :copyable="true" />

        <q-expansion-item icon="lightbulb" label="Ver permisos esperados (no hagas trampa)"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div class="answer-row">
              <code class="answer-key">-rwxr-xr-x</code>
              <span>control_node.py — todos pueden leer y ejecutar, solo owner escribe</span>
            </div>
            <div class="answer-row">
              <code class="answer-key">-r--r--r--</code>
              <span>nav2_params.yaml — todos solo leen, nadie escribe</span>
            </div>
            <div class="answer-row">
              <code class="answer-key">-rw-------</code>
              <span>secrets.key — solo el propietario lee y escribe</span>
            </div>
            <div class="answer-row">
              <code class="answer-key">-rw-r--r--</code>
              <span>debug.log — owner escribe, todos leen</span>
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
      <TextBlock>Observa el sistema de permisos en acción en un entorno real:</TextBlock>

      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Permisos en Linux" frameborder="0"
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
        <div v-for="s in summaryCommands" :key="s.cmd" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="terminal" size="12px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="¡Siguiente paso!" class="q-mt-xl">
        Ahora que controlas los permisos, el siguiente módulo te enseñará
        <strong>editores de texto en terminal</strong>: Nano y Vim.
        Necesarios para editar archivos de configuración sin interfaz gráfica.
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

// ── Fact pills ─────────────────────────────────────────────────
const facts = [
  { icon: '👤', label: 'Owner / Group / Others — 3 niveles' },
  { icon: '🔑', label: 'r=4  w=2  x=1 — tres permisos' },
  { icon: '⚡', label: 'chmod — modifica permisos al instante' },
];

// ── Levels ─────────────────────────────────────────────────────
const levels = [
  {
    title: 'Propietario (User)', sub: 'El creador del archivo', color: '#4ade80',
    icon: 'person', example: 'alexander',
  },
  {
    title: 'Grupo (Group)', sub: 'Equipo con acceso compartido', color: '#60a5fa',
    icon: 'group', example: 'dialout',
  },
  {
    title: 'Otros (Others)', sub: 'El resto del mundo', color: '#94a3b8',
    icon: 'public', example: 'world',
  },
];

// ── Permission symbols ─────────────────────────────────────────
const permSymbols = [
  {
    sym: 'r', name: 'Read (Leer)', val: 4, color: '#4ade80',
    forFile: 'abrir y ver contenido',
    forDir: 'listar archivos (ls)',
  },
  {
    sym: 'w', name: 'Write (Escribir)', val: 2, color: '#fbbf24',
    forFile: 'modificar o borrar',
    forDir: 'crear/borrar archivos dentro',
  },
  {
    sym: 'x', name: 'Execute (Ejecutar)', val: 1, color: '#f87171',
    forFile: 'ejecutar como programa',
    forDir: 'entrar con cd',
  },
];

// ── Decode table ───────────────────────────────────────────────
const decodeTable = [
  { str: '-rwxr-xr--', type: 'archivo', who: 'script Python', desc: 'Owner lee/escribe/ejecuta, grupo lee/ejecuta, otros solo leen', color: '#4ade80' },
  { str: 'drwxr-xr-x', type: 'directorio', who: 'carpeta paquete', desc: 'Todos pueden entrar y listar, solo owner puede crear/borrar dentro', color: '#60a5fa' },
  { str: '-rw-r--r--', type: 'archivo', who: 'config YAML', desc: 'Owner lee y escribe, todos los demás solo leen', color: '#fbbf24' },
  { str: '-rw-------', type: 'archivo', who: 'secrets.key', desc: 'Solo el owner tiene acceso total — nadie más puede verlo', color: '#f87171' },
];

// ── Octal reference ────────────────────────────────────────────
const octalRef = [
  { num: '7', perm: 'rwx', calc: '4+2+1', desc: 'Todo', color: '#4ade80' },
  { num: '6', perm: 'rw-', calc: '4+2+0', desc: 'Leer y escribir', color: '#60a5fa' },
  { num: '5', perm: 'r-x', calc: '4+0+1', desc: 'Leer y ejecutar', color: '#fbbf24' },
  { num: '4', perm: 'r--', calc: '4+0+0', desc: 'Solo leer', color: '#fb923c' },
  { num: '0', perm: '---', calc: '0+0+0', desc: 'Sin acceso', color: '#f87171' },
];

// ── Common chmod values ────────────────────────────────────────
const commonChmod = [
  { val: '755', perm: 'rwxr-xr-x', use: 'Scripts ejecutables, binarios', color: '#4ade80' },
  { val: '644', perm: 'rw-r--r--', use: 'Archivos de datos, configs', color: '#60a5fa' },
  { val: '600', perm: 'rw-------', use: 'Claves privadas, secrets', color: '#fbbf24' },
  { val: '444', perm: 'r--r--r--', use: 'Read-only para todos', color: '#fb923c' },
  { val: '777', perm: 'rwxrwxrwx', use: '⛔ Nunca — inseguro', color: '#f87171' },
  { val: '+x',  perm: '+ejecución', use: 'Hacer ejecutable rápido', color: '#c084fc' },
];

// ── Use cases ──────────────────────────────────────────────────
const useCases = [
  {
    title: 'Hacer un Nodo Ejecutable', color: '#4ade80', icon: 'play_circle',
    desc: 'Antes de ejecutar directamente un script Python como ROS 2 node, necesita el bit +x.',
    code: '# Dar permisos de ejecución\nchmod +x mi_nodo.py\n\n# Verificar\nls -l mi_nodo.py\n# -rwxr-xr-x  alexander  mi_nodo.py\n\n# Ahora puedes ejecutarlo\n./mi_nodo.py',
  },
  {
    title: 'Proteger Configuraciones', color: '#f59e0b', icon: 'lock',
    desc: 'Los archivos de Nav2, URDF o parámetros críticos no deberían modificarse accidentalmente.',
    code: '# Bloquear nav2 params (solo lectura)\nchmod 444 nav2_params.yaml\n\n# Si necesitas editarlo:\nchmod 644 nav2_params.yaml\n# edita...\nchmod 444 nav2_params.yaml  # vuelve a proteger',
  },
  {
    title: 'Permisos Recursivos', color: '#c084fc', icon: 'folder_special',
    desc: 'Para aplicar permisos a toda una carpeta de scripts o a tu paquete completo.',
    code: '# Aplicar 755 a toda la carpeta scripts/\nchmod -R 755 scripts/\n\n# x en directorios = permiso de entrar (cd)\n# x en archivos = permiso de ejecutar\n\n# Solo carpetas (sin tocar archivos)\nfind . -type d -exec chmod 755 {} \\;',
  },
];

// ── Sudo do/don't ──────────────────────────────────────────────
const sudoDo = [
  { cmd: 'sudo apt install', desc: 'Instalar paquetes del sistema' },
  { cmd: 'sudo systemctl', desc: 'Controlar servicios del sistema' },
  { cmd: 'sudo usermod -aG', desc: 'Agregar usuario a grupos' },
  { cmd: 'sudo nano /etc/hosts', desc: 'Editar archivos del sistema' },
];
const sudoDont = [
  { cmd: 'sudo colcon build', desc: 'Rompe los permisos del workspace' },
  { cmd: 'sudo chmod 777 -R /', desc: 'Elimina toda seguridad del sistema' },
  { cmd: 'sudo rm -rf /*', desc: 'Destruye el sistema operativo' },
  { cmd: 'sudo python3 script.py', desc: 'Innecesario y peligroso' },
];

// ── Hardware groups ────────────────────────────────────────────
const groups = [
  {
    name: 'dialout', color: '#4ade80', icon: 'usb',
    devices: 'Puertos seriales: Arduino, Lidar, GPS, módems',
    note: 'El más importante para robótica física',
  },
  {
    name: 'video', color: '#60a5fa', icon: 'videocam',
    devices: 'Cámaras USB, webcams, capturadoras',
    note: 'Necesario para ROS 2 camera drivers',
  },
  {
    name: 'audio', color: '#a78bfa', icon: 'mic',
    devices: 'Micrófonos, bocinas, interfaces de audio',
    note: 'Para robots con procesamiento de voz',
  },
  {
    name: 'docker', color: '#22d3ee', icon: 'sailing',
    devices: 'Docker daemon sin sudo',
    note: 'docker run sin necesitar sudo',
  },
  {
    name: 'render', color: '#fb923c', icon: 'memory',
    devices: 'GPU para aceleración gráfica',
    note: 'Para simulación en Gazebo con GPU',
  },
  {
    name: 'tty', color: '#f59e0b', icon: 'terminal',
    devices: 'Terminales y dispositivos TTY',
    note: 'Backup de dialout en algunos sistemas',
  },
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    msg: 'bash: ./script.py: Permission denied',
    summary: 'El archivo no tiene permisos de ejecución',
    cause: 'Falta el bit de ejecución (x). Los archivos nuevos no son ejecutables por defecto.',
    steps: [
      'Agrega el bit de ejecución: chmod +x script.py',
      'Verifica el resultado: ls -l script.py',
      'Ahora debería aparecer -rwxr-xr-x',
    ],
    code: 'chmod +x script.py\nls -l script.py',
    open: false,
  },
  {
    msg: 'Error: cannot open /dev/ttyUSB0: Permission denied',
    summary: 'Tu usuario no tiene acceso al puerto serial',
    cause: 'El dispositivo /dev/ttyUSB0 pertenece al grupo dialout y tu usuario no es miembro.',
    steps: [
      'Verifica el grupo del dispositivo: ls -la /dev/ttyUSB0',
      'Agrega tu usuario al grupo dialout: sudo usermod -aG dialout $USER',
      'Reinicia la sesión o el PC para aplicar el cambio',
      'Verifica con: groups (debería aparecer dialout)',
    ],
    code: 'sudo usermod -aG dialout $USER\n# Reinicia la sesión y verifica:\ngroups',
    open: false,
  },
  {
    msg: 'colcon build: Permission denied to build/',
    summary: 'Los archivos del workspace pertenecen a root',
    cause: 'Compilaste previamente con "sudo colcon build" y los archivos quedaron con dueño root.',
    steps: [
      'Cambia el dueño de vuelta a tu usuario: sudo chown -R $USER:$USER ~/ros2_ws/',
      'Nunca vuelvas a usar sudo con colcon.',
      'Para limpiar completamente: rm -rf build/ install/ log/',
    ],
    code: 'sudo chown -R $USER:$USER ~/ros2_ws/\n# O limpiar completamente:\nrm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log',
    open: false,
  },
  {
    msg: 'touch: cannot touch \'archivo\': Permission denied',
    summary: 'No tienes permisos de escritura en ese directorio',
    cause: 'El directorio tiene permisos que no permiten escritura a tu usuario.',
    steps: [
      'Verifica los permisos del directorio: ls -ld directorio/',
      'Si eres el dueño: chmod u+w directorio/',
      'Si no eres el dueño, opera en tu directorio home /home/$USER/',
    ],
    code: 'ls -ld .\nchmod u+w .',
    open: false,
  },
]);

// ── Summary ────────────────────────────────────────────────────
const summaryCommands = [
  { cmd: 'ls -l',          desc: 'Ver permisos de archivos en detalle',               example: 'ls -la scripts/', color: '#4ade80' },
  { cmd: 'chmod 755',      desc: 'Cambiar permisos método numérico',                   example: 'chmod 644 config.yaml', color: '#c084fc' },
  { cmd: 'chmod +x',       desc: 'Hacer ejecutable (método simbólico)',                example: 'chmod +x nodo.py', color: '#22d3ee' },
  { cmd: 'sudo',           desc: 'Ejecutar como superusuario (usar con cuidado)',       example: 'sudo apt install', color: '#f59e0b' },
  { cmd: 'usermod -aG',    desc: 'Agregar usuario a un grupo del sistema',             example: 'sudo usermod -aG dialout $USER', color: '#60a5fa' },
  { cmd: 'groups',         desc: 'Ver a qué grupos pertenece tu usuario actual',       example: 'groups', color: '#fb923c' },
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
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: 0.75rem; font-weight: 800; margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }
.cmd-inline { font-family:'Fira Code',monospace; font-size:.95em; background:none; color:var(--text-code); padding:0; }
.sub-title { font-size:1rem; font-weight:600; color:var(--text-primary); margin-bottom:4px; }

/* ── Fact pills */
.fact-pills { display:flex; gap:10px; flex-wrap:wrap; margin-bottom:1.5rem; }
.fact-pill {
  display:flex; align-items:center; gap:8px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 999px; padding: 7px 16px;
  font-size: 0.84rem; color: var(--text-secondary); transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fact-icon { font-size: 1rem; }

/* ══════════════════════════════════════════
   LEVELS + PERM SYMBOLS
══════════════════════════════════════════ */
.levels-grid {
  display: grid; grid-template-columns: repeat(3, 1fr); gap: 16px;
}
.level-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--lc-color, var(--border-medium));
  border-radius: 14px; padding: 1.5rem;
  text-align: center; transition: all .25s;
}
.level-card:hover { transform: translateY(-4px); box-shadow: var(--shadow-md); }
.lc-icon-wrap {
  width: 54px; height: 54px; border-radius: 14px;
  display: flex; align-items: center; justify-content: center;
  margin: 0 auto 12px;
}
.lc-title { font-size:1rem; font-weight:700; color:var(--text-primary); margin-bottom:4px; }
.lc-sub   { font-size:.84rem; color:var(--text-secondary); margin-bottom:8px; }
.lc-example code { background:none; padding:0; color:var(--lc-color); font-size:.9rem; }

.perms-row {
  display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px;
}
.perm-pill {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--pp-color, var(--border-medium));
  border-radius: 12px; padding: 1rem 1.25rem;
  display: flex; align-items: flex-start; gap: 14px; transition: all .25s;
}
.perm-pill:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.perm-sym {
  font-family: 'Fira Code', monospace; font-size: 2.2rem; font-weight: 900;
  color: var(--pp-color); line-height: 1; flex-shrink: 0;
}
.perm-right { flex: 1; }
.perm-name { font-size:.9rem; font-weight:700; color:var(--text-primary); margin-bottom:2px; }
.perm-val  { font-family:'Fira Code',monospace; font-size:.82rem; color:var(--pp-color); margin-bottom:6px; }
.perm-file-dir { display:flex; flex-direction:column; gap:2px; font-size:.78rem; color:var(--text-muted); }

/* ══════════════════════════════════════════
   PERMISSION DECODER
══════════════════════════════════════════ */
.perm-decoder {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 1.5rem; overflow: hidden;
}
.pd-terminal-line {
  font-family: 'Fira Code', monospace; font-size: 1.15rem;
  display: flex; align-items: center; flex-wrap: wrap; gap: 0;
  background: var(--bg-deep); border-radius: 10px; padding: 1rem 1.25rem;
  margin-bottom: 1.25rem; overflow-x: auto;
}
.pd-seg {
  padding: 4px 8px; border-radius: 6px; font-weight: 800;
  margin-right: 2px;
}
.file-type   { color: #94a3b8; background: rgba(148,163,184,.12); }
.owner-seg   { color: #4ade80; background: rgba( 74,222,128,.15); }
.group-seg   { color: #60a5fa; background: rgba( 96,165,250,.15); }
.other-seg   { color: #94a3b8; background: rgba(148,163,184,.12); }
.pd-rest { color: var(--text-secondary); font-size:.95rem; margin-left:8px; white-space:nowrap; }

.pd-legend {
  display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px;
}
.pdl-item { text-align: center; }
.pdl-brace { font-size:1.2rem; color:var(--text-muted); margin-bottom:4px; }
.pdl-label {
  font-size:.8rem; font-weight:700; color:var(--text-secondary);
  span { font-size:.75rem; font-weight:400; color:var(--text-muted); font-family:'Fira Code',monospace; white-space:pre; }
}
.file-type-br .pdl-label { color: #94a3b8; }
.owner-br    .pdl-label  { color: #4ade80; }
.group-br    .pdl-label  { color: #60a5fa; }
.other-br    .pdl-label  { color: #94a3b8; }

.decode-table {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; overflow: hidden;
}
.dt-header {
  background: var(--bg-deep); border-bottom: 1px solid var(--border-subtle);
  padding: 10px 16px; display:flex; align-items:center; gap:8px;
  font-size:.85rem; font-weight:600; color:var(--text-secondary);
}
.dt-rows { display:flex; flex-direction:column; }
.dt-row {
  display:grid; grid-template-columns: 140px 80px 140px 1fr; gap:12px;
  padding:10px 16px; border-bottom:1px solid var(--border-subtle);
  align-items:center; transition:background .15s;
}
.dt-row:last-child { border-bottom:none; }
.dt-row:hover { background:var(--bg-surface-hover); }
.dt-str  { font-family:'Fira Code',monospace; font-size:.9rem; font-weight:700; background:none; padding:0; }
.dt-type { font-size:.82rem; color:var(--text-muted); font-style:italic; }
.dt-who  { font-size:.82rem; color:var(--text-secondary); }
.dt-desc { font-size:.82rem; color:var(--text-secondary); }

/* ══════════════════════════════════════════
   CHMOD CARDS
══════════════════════════════════════════ */
.chmod-grid {
  display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px;
}
.chmod-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ck-color, var(--border-medium));
  border-radius: 14px; padding: 1.5rem;
  display: flex; flex-direction: column; gap: 14px; min-width: 0;
}
.ck-header { display:flex; align-items:center; gap:10px; }
.ck-title  { font-size:1.05rem; font-weight:700; color:var(--text-primary); }
.ck-desc   { font-size:.88rem; color:var(--text-secondary); line-height:1.5; margin:0; }

.octal-table { background:var(--bg-surface-hover); border-radius:10px; overflow:hidden; }
.ot-row {
  display:grid; grid-template-columns:40px 60px 80px 1fr; gap:8px;
  padding:8px 12px; border-bottom:1px solid var(--border-subtle);
  font-size:.85rem; align-items:center;
}
.ot-row.header { font-weight:700; color:var(--text-muted); font-size:.78rem; text-transform:uppercase; letter-spacing:.06em; }
.ot-row:last-child { border-bottom:none; }
.ot-perm { font-family:'Fira Code',monospace; background:none; padding:0; }
.ot-calc { color:var(--text-muted); font-family:'Fira Code',monospace; font-size:.8rem; }
.ot-desc { color:var(--text-secondary); }

.sym-table { background:var(--bg-surface-hover); border-radius:10px; padding:.75rem 1rem; display:flex; flex-direction:column; gap:6px; }
.sym-row { display:flex; align-items:center; gap:12px; font-size:.88rem; }
.sym-row code { width:24px; text-align:center; font-size:1rem; font-weight:800; background:none; padding:0; }
.sym-row span { color:var(--text-secondary); }
.sym-divider { border-top:1px dashed var(--border-medium); margin:4px 0; }

.chmod-common { }
.cc-title {
  display:flex; align-items:center; gap:8px;
  font-size:.9rem; font-weight:600; color:var(--text-secondary); margin-bottom:12px;
}
.cc-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }
.cc-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid var(--cc-color, var(--border-medium));
  border-radius:10px; padding:1rem; transition:all .2s;
}
.cc-card:hover { transform:translateY(-3px); box-shadow:var(--shadow-sm); }
.cc-val  { display:block; font-family:'Fira Code',monospace; font-size:1.2rem; font-weight:900; color:var(--cc-color); background:none; padding:0; margin-bottom:4px; }
.cc-perm { font-family:'Fira Code',monospace; font-size:.78rem; color:var(--text-muted); margin-bottom:4px; }
.cc-use  { font-size:.82rem; color:var(--text-secondary); }

/* ══════════════════════════════════════════
   USE CASES
══════════════════════════════════════════ */
.usecase-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:16px; }
.usecase-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-top:3px solid var(--uc-color, var(--border-medium));
  border-radius:14px; padding:1.25rem;
  display:flex; flex-direction:column; gap:10px; min-width:0; transition:all .25s;
}
.usecase-card:hover { transform:translateY(-4px); box-shadow:var(--shadow-md); }
.uc-header { display:flex; align-items:center; gap:12px; }
.uc-icon-wrap { width:40px; height:40px; border-radius:10px; display:flex; align-items:center; justify-content:center; flex-shrink:0; }
.uc-title { font-size:.95rem; font-weight:700; color:var(--text-primary); }
.uc-desc  { font-size:.88rem; color:var(--text-secondary); line-height:1.5; margin:0; flex:1; }

/* ══════════════════════════════════════════
   SUDO
══════════════════════════════════════════ */
.sudo-list { display:flex; flex-direction:column; gap:8px; }
.sudo-item { display:flex; flex-direction:column; gap:2px; }
.sudo-item code { background:none; padding:0; color:var(--text-code); font-size:.88rem; }
.sudo-item span { font-size:.8rem; color:var(--text-muted); }

.sudo-explain {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-left:4px solid #f59e0b; border-radius:14px; overflow:hidden;
}
.se-header {
  display:flex; align-items:center; gap:8px; padding:1rem 1.25rem;
  font-size:.9rem; font-weight:700; color:var(--text-primary);
  border-bottom:1px solid var(--border-subtle);
  code { background:none; padding:0; color:#f59e0b; }
}
.se-body {
  display:flex; align-items:center; flex-wrap:wrap; gap:0;
  padding:1rem 1.25rem;
}
.se-step {
  display:flex; align-items:flex-start; gap:10px; flex:1; min-width:180px;
  font-size:.88rem; color:var(--text-secondary); padding:8px;
  code { background:none; padding:0; color:var(--text-code); }
}
.se-step.warn { background:rgba(239,68,68,.07); border-radius:8px; }
.se-step.warn strong { color:#f87171; }
.se-num {
  min-width:24px; width:24px; height:24px; border-radius:50%;
  background:var(--bg-surface-hover); border:1px solid var(--border-medium);
  font-size:.75rem; font-weight:800; color:var(--text-muted);
  display:flex; align-items:center; justify-content:center; flex-shrink:0;
}
.se-arrow { font-size:1.3rem; color:var(--text-muted); padding:0 6px; flex-shrink:0; }

/* ══════════════════════════════════════════
   GROUPS
══════════════════════════════════════════ */
.groups-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:14px; }
.group-card {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-top:3px solid var(--gc-color, var(--border-medium));
  border-radius:12px; padding:1.1rem;
  display:flex; flex-direction:column; gap:6px; transition:all .25s;
}
.group-card:hover { transform:translateY(-3px); box-shadow:var(--shadow-sm); }
.gc-top { display:flex; align-items:center; gap:8px; }
.gc-name { font-family:'Fira Code',monospace; font-size:1rem; font-weight:800; color:var(--gc-color); background:none; padding:0; }
.gc-devices { font-size:.85rem; color:var(--text-secondary); line-height:1.4; }
.gc-note { font-size:.78rem; color:var(--text-muted); font-style:italic; }

.group-howto {
  background:var(--bg-surface); border:1px solid var(--border-subtle);
  border-radius:14px; overflow:hidden;
}
.gh-title {
  display:flex; align-items:center; gap:8px; padding:12px 16px;
  background:var(--bg-deep); border-bottom:1px solid var(--border-subtle);
  font-size:.9rem; font-weight:600; color:var(--text-secondary);
}

/* ══════════════════════════════════════════
   ERROR LIST (same pattern as lesson 01/02)
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
.answer-key { font-family:'Fira Code',monospace; font-weight:700; color:#4ade80; background:none; padding:0; white-space:nowrap; }

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
.sc-cmd   { display:block; font-family:'Fira Code',monospace; font-size:1.05rem; font-weight:700; color:var(--sc-color,var(--text-code)); background:none; padding:0; margin-bottom:6px; }
.sc-desc  { font-size:.88rem; color:var(--text-secondary); margin-bottom:8px; line-height:1.45; }
.sc-example { display:flex; align-items:center; font-family:'Fira Code',monospace; font-size:.78rem; color:var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 900px) {
  .levels-grid   { grid-template-columns: 1fr 1fr 1fr; }
  .perms-row     { grid-template-columns: 1fr 1fr; }
  .chmod-grid    { grid-template-columns: 1fr; }
  .cc-grid       { grid-template-columns: repeat(3,1fr); }
  .usecase-grid  { grid-template-columns: 1fr 1fr; }
  .groups-grid   { grid-template-columns: 1fr 1fr; }
  .summary-grid  { grid-template-columns: 1fr 1fr; }
  .pd-legend     { grid-template-columns: repeat(2,1fr); }
  .dt-row        { grid-template-columns: 130px 70px 1fr; }
  .dt-row .dt-desc { display: none; }
}

@media (max-width: 768px) {
  .levels-grid   { grid-template-columns: 1fr; }
  .perms-row     { grid-template-columns: 1fr; }
  .usecase-grid  { grid-template-columns: 1fr; }
  .groups-grid   { grid-template-columns: 1fr 1fr; }
  .cc-grid       { grid-template-columns: repeat(2,1fr); }
  .summary-grid  { grid-template-columns: 1fr 1fr; }
  .fact-pills    { flex-direction:column; gap:8px; }
  .fact-pill     { border-radius:12px; }
  .pd-legend     { grid-template-columns: repeat(2,1fr); }
  .dt-row        { grid-template-columns: 120px 1fr; }
  .dt-row .dt-type, .dt-row .dt-who { display: none; }
  .se-body       { flex-direction:column; }
  .se-arrow      { transform:rotate(90deg); align-self:center; }
  .challenge-header { flex-direction:column; }
  .challenge-badge  { margin-left:0; }
}

@media (max-width: 480px) {
  .groups-grid   { grid-template-columns: 1fr; }
  .cc-grid       { grid-template-columns: 1fr 1fr; }
  .summary-grid  { grid-template-columns: 1fr; }
  .pd-terminal-line { font-size:.95rem; }
  .pd-legend     { grid-template-columns: 1fr 1fr; }
  .ot-row        { grid-template-columns: 35px 55px 1fr; }
  .ot-row .ot-desc { display:none; }
}
</style>
