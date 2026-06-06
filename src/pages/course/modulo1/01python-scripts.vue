<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Python es el lenguaje de facto de ROS 2: no por ser "fácil",
        sino porque permite <strong>iterar rápido sin compilar</strong>.
        Un bug en un driver de C++ puede costar horas de recompilación;
        en Python, corriges y relanzas en segundos.
        Esta lección te enseña a escribir scripts Python que funcionen como
        <strong>procesos autónomos de robótica</strong> — con shebang, permisos,
        type hints, logging y argumentos de CLI — no como scripts web.
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
        Anatomía de un Script Python para ROS 2
      </SectionTitle>

      <TextBlock>
        Un script de robótica no termina. Es un <strong>proceso continuo</strong>
        que escucha sensores, toma decisiones y envía comandos en un bucle de eventos.
        Cada parte de la estructura tiene una razón de ser.
      </TextBlock>

      <!-- Code demo with annotations -->
      <div class="annotated-code q-mt-lg">
        <div class="ac-chrome">
          <div class="ac-dots">
            <span class="acd red"></span>
            <span class="acd yellow"></span>
            <span class="acd green"></span>
          </div>
          <span class="ac-filename">robot_node.py</span>
        </div>
        <div class="ac-body">
          <div v-for="(ln, i) in codeLines" :key="i"
            class="ac-line"
            :class="{ 'ac-line-hl': ln.highlight, 'ac-line-empty': ln.empty }">
            <span class="ac-num">{{ i + 1 }}</span>
            <span class="ac-code" v-html="ln.html"></span>
            <div v-if="ln.annotation" class="ac-annotation">
              <q-icon name="arrow_left" size="12px" />
              {{ ln.annotation }}
            </div>
          </div>
        </div>
      </div>

      <!-- Node lifecycle flow -->
      <div class="lifecycle-flow q-mt-xl">
        <div class="lf-title">
          <q-icon name="account_tree" size="16px" color="primary" />
          Ciclo de vida de un nodo ROS 2
        </div>
        <div class="lf-steps">
          <div v-for="(step, i) in lifecycle" :key="i" class="lf-step-wrap">
            <div class="lf-step" :style="{ '--lf-color': step.color }">
              <q-icon :name="step.icon" size="18px" :style="{ color: step.color }" />
              <code class="lf-fn">{{ step.fn }}</code>
              <div class="lf-desc">{{ step.desc }}</div>
            </div>
            <div v-if="i < lifecycle.length - 1" class="lf-arrow">→</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         02 SHEBANG
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        El Shebang — La Primera Línea
      </SectionTitle>

      <TextBlock>
        En Linux la extensión <code>.py</code> es decorativa.
        El sistema operativo lee los primeros dos bytes del archivo (<code>#!</code>)
        para saber qué intérprete usar. Sin shebang, <code>./script.py</code> falla
        aunque tengas chmod +x.
      </TextBlock>

      <div class="shebang-grid q-mt-lg">
        <div v-for="opt in shebangOptions" :key="opt.title" class="shebang-card"
          :class="opt.type"
          :style="{ '--shb-color': opt.color }">
          <div class="shb-header">
            <q-icon :name="opt.icon" size="20px" :style="{ color: opt.color }" />
            <span class="shb-status" :style="{ color: opt.color }">{{ opt.status }}</span>
          </div>
          <div class="shb-title">{{ opt.title }}</div>
          <CodeBlock :hide-header="true" lang="python" :content="opt.code" />
          <div class="shb-reason">
            <strong>{{ opt.reasonLabel }}:</strong> {{ opt.reason }}
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="¿Cómo funciona env?" class="q-mt-lg">
        <code>/usr/bin/env python3</code> le dice al OS:
        "ejecuta <code>python3</code>, buscándolo en el <strong>PATH actual</strong>".
        Funciona con entornos virtuales, pyenv y cualquier instalación no estándar.
        La ruta absoluta <code>/usr/bin/python3</code> falla si Python está en otro lugar.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         03 CHMOD + EJECUCIÓN
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        chmod +x y las Tres Formas de Ejecutar
      </SectionTitle>

      <!-- chmod demo -->
      <div class="chmod-visual q-mt-lg">
        <div class="cv-panel">
          <div class="cvp-label error-label">Sin permisos</div>
          <div class="cvp-file">
            <q-icon name="description" size="40px" style="color:var(--text-muted)" />
            <code class="cvp-name">robot.py</code>
            <div class="cvp-perms"><span class="perm-rw">rw-</span><span class="perm-r">r--</span><span class="perm-r">r--</span></div>
            <div class="cvp-status cvp-status-bad">
              <q-icon name="block" size="14px" />
              Ejecutar: denegado
            </div>
          </div>
        </div>
        <div class="cv-transform">
          <div class="cvt-cmd">
            <CodeBlock :hide-header="true" lang="bash" content="chmod +x robot.py" :copyable="true" />
          </div>
          <div class="cvt-arrow">
            <q-icon name="east" size="24px" color="positive" />
          </div>
        </div>
        <div class="cv-panel">
          <div class="cvp-label success-label">Con permisos</div>
          <div class="cvp-file">
            <q-icon name="terminal" size="40px" style="color:#4ade80" />
            <code class="cvp-name">robot.py</code>
            <div class="cvp-perms"><span class="perm-rwx">rwx</span><span class="perm-rx">r-x</span><span class="perm-rx">r-x</span></div>
            <div class="cvp-status cvp-status-ok">
              <q-icon name="check_circle" size="14px" />
              ./robot.py → funciona
            </div>
          </div>
        </div>
      </div>

      <!-- 3 execution methods grid -->
      <div class="exec-grid q-mt-xl">
        <div v-for="m in execMethods" :key="m.title" class="exec-card"
          :class="{ 'exec-recommended': m.recommended }"
          :style="{ '--em-color': m.color }">
          <div class="ec-num" :style="{ background: m.color }">{{ m.num }}</div>
          <div class="ec-title">{{ m.title }}</div>
          <CodeBlock :hide-header="true" lang="bash" :content="m.cmd" :copyable="true" />
          <div class="ec-meta">
            <div class="ec-pros">
              <div v-for="p in m.pros" :key="p" class="ec-item ec-pro">
                <q-icon name="check" size="11px" style="color:#4ade80" />{{ p }}
              </div>
            </div>
            <div class="ec-note">
              <q-icon name="info" size="11px" class="q-mr-xs" />
              {{ m.use }}
            </div>
          </div>
          <div v-if="m.recommended" class="ec-recommended-badge">Producción</div>
        </div>
      </div>

      <AlertBlock type="warning" title='¿Por qué "./script.py" y no "script.py"?' class="q-mt-lg">
        Por <strong>seguridad</strong>. Linux no ejecuta archivos del directorio actual por defecto
        (a diferencia de Windows). El <code>./</code> dice explícitamente:
        "ejecuta este archivo de AQUÍ, no del PATH". Protege contra comandos maliciosos con el mismo nombre.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         04 PYTHON MODERNO — TYPE HINTS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        Python Moderno — Type Hints y __main__
      </SectionTitle>

      <TextBlock>
        Los <strong>type hints</strong> son anotaciones de tipo que no afectan la ejecución
        pero hacen que tu editor detecte errores antes de correr el código.
        En ROS 2, donde un tipo incorrecto en un mensaje puede hacer que el robot actúe mal,
        son indispensables.
      </TextBlock>

      <!-- Type hints comparison -->
      <div class="types-comparison q-mt-lg">
        <div class="tc-panel tc-before">
          <div class="tc-header tc-header-bad">
            <q-icon name="cancel" size="16px" color="negative" />
            Sin Type Hints — ambiguo
          </div>
          <CodeBlock :hide-header="true" lang="python" :content="noTypesCode" />
        </div>
        <div class="tc-arrow">
          <q-icon name="trending_flat" size="28px" style="color:var(--text-muted)" />
          <span>mejor con</span>
        </div>
        <div class="tc-panel tc-after">
          <div class="tc-header tc-header-good">
            <q-icon name="check_circle" size="16px" color="positive" />
            Con Type Hints — claro y seguro
          </div>
          <CodeBlock :hide-header="true" lang="python" :content="withTypesCode" />
        </div>
      </div>

      <!-- Type reference -->
      <div class="type-ref-grid q-mt-lg">
        <div v-for="t in typeHints" :key="t.hint" class="type-ref-card"
          :style="{ '--tr-color': t.color }">
          <code class="tr-hint">{{ t.hint }}</code>
          <div class="tr-example">{{ t.example }}</div>
          <div class="tr-ros">{{ t.ros }}</div>
        </div>
      </div>

      <!-- __main__ guard -->
      <div class="guard-section q-mt-xl">
        <div class="guard-title">
          <q-icon name="security" size="18px" color="primary" />
          El Guard <code>if __name__ == '__main__'</code> — ¿Por qué existe?
        </div>
        <TextBlock>
          Cuando Python carga un archivo, asigna a la variable especial
          <code>__name__</code> un valor según cómo se use:
        </TextBlock>
        <div class="guard-cases q-mt-md">
          <div v-for="gc in guardCases" :key="gc.how" class="guard-case"
            :style="{ '--gc-color': gc.color }">
            <div class="gc-how">{{ gc.how }}</div>
            <div class="gc-arrow">→</div>
            <code class="gc-value" :style="{ color: gc.color }">{{ gc.value }}</code>
            <div class="gc-result">{{ gc.result }}</div>
          </div>
        </div>
        <CodeBlock title="Demostración práctica" lang="python" :content="mainGuardCode" class="q-mt-md" />
        <AlertBlock type="success" title="Por qué importa en ROS 2" class="q-mt-md">
          Un paquete ROS 2 puede <code>import</code> funciones de otro módulo.
          Sin el guard, cada <code>import</code> ejecutaría el código de inicio —
          lanzando nodos duplicados y causando caos en el sistema de comunicación.
        </AlertBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 ARGUMENTOS CLI
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Argumentos de Línea de Comandos
      </SectionTitle>

      <TextBlock>
        Un nodo robótico configurable no hardcodea valores como la frecuencia de publicación
        o el nombre del robot. Los acepta como <strong>argumentos al ejecutar</strong>,
        permitiendo reutilizar el mismo script para diferentes robots o situaciones.
      </TextBlock>

      <!-- sys.argv vs argparse split -->
      <SplitBlock class="q-mt-lg">
        <template #left>
          <div class="arg-panel">
            <div class="ap-header ap-header-basic">
              <q-icon name="data_array" size="18px" />
              sys.argv — Básico
            </div>
            <CodeBlock :hide-header="true" lang="python" :content="sysArgvCode" />
            <div class="ap-note ap-note-bad">
              <q-icon name="warning" size="14px" color="warning" />
              Sin validación, sin ayuda, sin tipos. Solo para casos simples.
            </div>
          </div>
        </template>
        <template #right>
          <div class="arg-panel">
            <div class="ap-header ap-header-pro">
              <q-icon name="settings" size="18px" />
              argparse — Profesional
            </div>
            <CodeBlock :hide-header="true" lang="python" :content="argparseCode" />
            <div class="ap-note ap-note-good">
              <q-icon name="check_circle" size="14px" color="positive" />
              Tipos automáticos, valores default, mensaje --help generado.
            </div>
          </div>
        </template>
      </SplitBlock>

      <!-- argparse features -->
      <div class="argparse-features q-mt-lg">
        <div class="af-title">Beneficios de argparse</div>
        <div class="af-grid">
          <div v-for="feat in argparseFeatures" :key="feat.title" class="af-card"
            :style="{ '--af-color': feat.color }">
            <q-icon :name="feat.icon" size="18px" :style="{ color: feat.color }" />
            <div class="af-title-text">{{ feat.title }}</div>
            <div class="af-desc">{{ feat.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         06 LOGGING ROS 2
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">06</span>
        Logging en ROS 2 — Nunca más print()
      </SectionTitle>

      <TextBlock>
        <code>print()</code> no existe en producción robótica. El sistema de logging de ROS 2
        añade <strong>timestamp, nombre del nodo y nivel de severidad</strong> a cada mensaje,
        y puede filtrarse o redirigirse a archivos sin cambiar el código.
      </TextBlock>

      <!-- Log levels visual -->
      <div class="log-levels q-mt-lg">
        <div v-for="lvl in logLevels" :key="lvl.level" class="log-level-row">
          <div class="ll-badge" :style="{ background: lvl.color + '18', color: lvl.color, borderColor: lvl.color + '40' }">
            {{ lvl.level }}
          </div>
          <code class="ll-api">self.{{ lvl.api }}</code>
          <div class="ll-terminal" :style="{ borderLeftColor: lvl.color }">
            <span class="ll-bracket" :style="{ color: lvl.color }">[{{ lvl.level }}]</span>
            <span class="ll-time">[1.2345]</span>
            <span class="ll-node">[mi_nodo]:</span>
            <span class="ll-msg">{{ lvl.example }}</span>
          </div>
          <div class="ll-use">{{ lvl.use }}</div>
        </div>
      </div>

      <!-- Print vs Logger comparison -->
      <div class="print-vs-log q-mt-xl">
        <div class="pvl-panel pvl-bad">
          <div class="pvl-header">
            <q-icon name="cancel" size="16px" color="negative" />
            ❌ No hagas esto — print()
          </div>
          <CodeBlock :hide-header="true" lang="python" :content="printCode" />
        </div>
        <div class="pvl-panel pvl-good">
          <div class="pvl-header">
            <q-icon name="check_circle" size="16px" color="positive" />
            ✅ Haz esto — get_logger()
          </div>
          <CodeBlock :hide-header="true" lang="python" :content="loggerCode" />
        </div>
      </div>

      <AlertBlock type="success" title="Bonus: throttle y once" class="q-mt-lg">
        Para evitar spam en el log cuando algo ocurre continuamente:
        <CodeBlock :hide-header="true" lang="python" :content="throttleCode" />
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         07 DEBUGGING
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">07</span>
        Debugging — El Arte de Cazar Bugs
      </SectionTitle>

      <TextBlock>
        En robótica el debugging es difícil: el robot se mueve, los sensores llegan a 100 Hz,
        y no puedes pausar el tiempo real. Cada técnica tiene su momento.
      </TextBlock>

      <div class="debug-cards q-mt-lg">
        <div v-for="d in debugTechniques" :key="d.title" class="debug-card"
          :style="{ '--dc-color': d.color }">
          <div class="dc-header">
            <div class="dc-icon-wrap" :style="{ background: d.color + '18' }">
              <q-icon :name="d.icon" size="22px" :style="{ color: d.color }" />
            </div>
            <div>
              <div class="dc-level" :style="{ color: d.color }">{{ d.level }}</div>
              <div class="dc-title">{{ d.title }}</div>
            </div>
          </div>
          <CodeBlock :hide-header="true" lang="python" :content="d.code" />
          <div class="dc-meta">
            <div class="dc-pro">
              <q-icon name="thumb_up" size="11px" color="positive" />{{ d.pro }}
            </div>
            <div class="dc-con">
              <q-icon name="thumb_down" size="11px" color="negative" />{{ d.con }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         08 PYTHONPATH
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">08</span>
        PYTHONPATH — Dónde Busca Python
      </SectionTitle>

      <TextBlock>
        Cuando haces <code>import mi_modulo</code>, Python recorre una lista de directorios
        en orden hasta encontrar el módulo. Si no lo encuentra en ninguno: <code>ModuleNotFoundError</code>.
      </TextBlock>

      <div class="pythonpath-flow q-mt-lg">
        <div v-for="(item, i) in pythonpathItems" :key="i" class="pp-item"
          :style="{ '--pp-color': item.color }">
          <div class="pp-num" :style="{ background: item.color }">{{ i + 1 }}</div>
          <div class="pp-body">
            <div class="pp-title">{{ item.title }}</div>
            <code class="pp-path">{{ item.path }}</code>
          </div>
          <div class="pp-ros" v-if="item.rosNote">
            <q-icon name="smart_toy" size="12px" color="positive" />
            {{ item.rosNote }}
          </div>
        </div>
      </div>

      <CodeBlock title="Ver tu PYTHONPATH actual" lang="python" :content="sysPathCode" :copyable="true" class="q-mt-lg" />

      <AlertBlock type="warning" title="Trampa frecuente: ModuleNotFoundError 'rclpy'" class="q-mt-md">
        Si Python no encuentra <code>rclpy</code>, ROS 2 no está activo en la terminal actual.
        Debes hacer <code>source /opt/ros/jazzy/setup.bash</code> antes de ejecutar el script
        — o verificar que está en tu <code>~/.bashrc</code>.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         SCRIPT COMPLETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Script ROS 2 Profesional Completo</SectionTitle>

      <TextBlock>
        Este nodo integra todo lo aprendido: shebang, type hints, argparse, logging,
        manejo de excepciones y el guard <code>__main__</code>.
        Es la plantilla que usarás en cada nodo ROS 2 del curso.
      </TextBlock>

      <div class="complete-script-header q-mt-md">
        <q-icon name="star" size="16px" color="warning" />
        <span>La plantilla dorada — úsala como base en todos tus nodos</span>
      </div>

      <CodeBlock title="talker_node.py — Template de nodo ROS 2" lang="python"
        :content="talkerNodeCode" :copyable="true" />

      <div class="run-steps q-mt-lg">
        <div class="rs-title">
          <q-icon name="play_circle" size="18px" color="positive" />
          Cómo ejecutar
        </div>
        <div class="rs-grid">
          <div v-for="(step, i) in runSteps" :key="i" class="rs-step">
            <div class="rs-num">{{ i + 1 }}</div>
            <div class="rs-body">
              <div class="rs-label">{{ step.label }}</div>
              <CodeBlock :hide-header="true" lang="bash" :content="step.cmd" :copyable="true" />
            </div>
          </div>
        </div>
      </div>
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
      <SectionTitle>Reto — Tu Primer Nodo ROS 2 Completo</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea un nodo publicador configurable</div>
            <div class="challenge-subtitle">
              Aplicando todo lo aprendido en esta lección
            </div>
          </div>
          <div class="challenge-badge">15 min</div>
        </div>

        <div class="challenge-requirements q-mt-md">
          <div class="cr-title">Tu script debe tener:</div>
          <div class="cr-list">
            <div v-for="req in challengeReqs" :key="req.text" class="cr-item">
              <q-icon :name="req.icon" size="14px" :style="{ color: req.color }" />
              {{ req.text }}
            </div>
          </div>
        </div>

        <CodeBlock title="Specificationes" lang="python" :content="challengeSpec" class="q-mt-md" />

        <q-expansion-item icon="lightbulb" label="Ver pistas si estás atascado"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div v-for="hint in challengeHints" :key="hint" class="answer-row">
              <q-icon name="chevron_right" size="14px" style="color:#4ade80" />
              {{ hint }}
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
      <TextBlock>Scripts Python para ROS 2 — estructura y buenas prácticas:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Python Scripts ROS 2" frameborder="0"
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
      <SectionTitle>Resumen — Comandos y Conceptos Esenciales</SectionTitle>
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
  { icon: '🐍', label: 'Python — lenguaje #1 de ROS 2' },
  { icon: '⚡', label: 'Sin compilar — itera en segundos' },
  { icon: '🤖', label: 'Nodos, topics, servicios — todo en Python' },
];

// ── Annotated code lines ───────────────────────────────────────
const codeLines = [
  { html: '<span class="tk-shebang">#!/usr/bin/env python3</span>', annotation: 'Shebang — indica el intérprete' },
  { html: '<span class="tk-str">"""mi_nodo.py — Nodo publicador básico."""</span>' },
  { html: '', empty: true },
  { html: '<span class="tk-kw">import</span> rclpy', annotation: 'API central de ROS 2 en Python' },
  { html: '<span class="tk-kw">from</span> rclpy.node <span class="tk-kw">import</span> Node' },
  { html: '<span class="tk-kw">from</span> std_msgs.msg <span class="tk-kw">import</span> String' },
  { html: '', empty: true },
  { html: '<span class="tk-kw">class</span> <span class="tk-class">MiNodo</span>(Node):', highlight: true, annotation: 'Hereda de Node — es un ciudadano ROS 2' },
  { html: '    <span class="tk-kw">def</span> <span class="tk-fn">__init__</span>(self) -> <span class="tk-kw">None</span>:' },
  { html: '        <span class="tk-fn">super</span>().__init__(<span class="tk-str">\'mi_nodo\'</span>)', annotation: 'Registra el nodo con este nombre' },
  { html: '        self.publisher = self.<span class="tk-fn">create_publisher</span>(String, <span class="tk-str">\'topic\'</span>, 10)' },
  { html: '        self.<span class="tk-fn">create_timer</span>(1.0, self.callback)' },
  { html: '', empty: true },
  { html: '    <span class="tk-kw">def</span> <span class="tk-fn">callback</span>(self) -> <span class="tk-kw">None</span>:', annotation: 'ROS 2 llama esta función cada 1s' },
  { html: '        self.<span class="tk-fn">get_logger</span>().<span class="tk-fn">info</span>(<span class="tk-str">\'Hola ROS 2\'</span>)' },
  { html: '', empty: true },
  { html: '<span class="tk-kw">def</span> <span class="tk-fn">main</span>() -> <span class="tk-kw">None</span>:', highlight: true, annotation: 'Punto de entrada — llamado por ROS 2' },
  { html: '    rclpy.<span class="tk-fn">init</span>()' },
  { html: '    node = <span class="tk-class">MiNodo</span>()' },
  { html: '    rclpy.<span class="tk-fn">spin</span>(node)', annotation: 'Event loop — corre hasta Ctrl+C' },
  { html: '', empty: true },
  { html: '<span class="tk-kw">if</span> __name__ == <span class="tk-str">\'__main__\'</span>:', highlight: true, annotation: 'Guard — evita ejecución al importar' },
  { html: '    <span class="tk-fn">main</span>()' },
];

// ── Node lifecycle ─────────────────────────────────────────────
const lifecycle = [
  { fn: 'rclpy.init()', desc: 'Inicializa DDS', icon: 'power_settings_new', color: '#4ade80' },
  { fn: 'Node()', desc: 'Registra el nodo', icon: 'account_tree', color: '#22d3ee' },
  { fn: 'rclpy.spin()', desc: 'Event loop', icon: 'loop', color: '#fbbf24' },
  { fn: 'destroy_node()', desc: 'Limpieza', icon: 'delete', color: '#f87171' },
  { fn: 'rclpy.shutdown()', desc: 'Cierra DDS', icon: 'power_off', color: '#94a3b8' },
];

// ── Shebang options ────────────────────────────────────────────
const shebangOptions = [
  {
    type: 'shb-correct', status: '✅ Correcto — usa esto siempre',
    icon: 'check_circle', color: '#4ade80', title: 'Portable (env)',
    code: '#!/usr/bin/env python3',
    reasonLabel: 'Por qué',
    reason: 'env busca python3 en el PATH actual. Funciona con venv, pyenv y cualquier instalación.',
  },
  {
    type: 'shb-wrong', status: '❌ Evitar — ruta fija',
    icon: 'cancel', color: '#f87171', title: 'Rígido (ruta absoluta)',
    code: '#!/usr/bin/python3',
    reasonLabel: 'Problema',
    reason: 'Si Python está en /usr/local/bin, /opt, o en un venv, este path no existe y el script falla.',
  },
];

// ── Execution methods ──────────────────────────────────────────
const execMethods = [
  {
    num: 1, title: 'Manual', cmd: 'python3 script.py', color: '#94a3b8',
    pros: ['No requiere shebang', 'No requiere chmod'],
    use: 'Pruebas y debugging rápido',
  },
  {
    num: 2, title: 'Directo Linux', cmd: './script.py', color: '#4ade80',
    pros: ['Estilo Unix nativo', 'No expone el intérprete'],
    use: 'Scripts standalone y herramientas CLI',
  },
  {
    num: 3, title: 'ros2 run', cmd: 'ros2 run mi_pkg mi_nodo', color: '#60a5fa',
    pros: ['ROS 2 encuentra el ejecutable', 'Integración con launcher'],
    use: 'Producción — nodos ROS 2', recommended: true,
  },
];

// ── Type hints ─────────────────────────────────────────────────
const typeHints = [
  { hint: 'str', example: 'robot_name: str', ros: 'Nombre de nodo, topic', color: '#4ade80' },
  { hint: 'int', example: 'queue_size: int', ros: 'Tamaño de buffer de mensajes', color: '#60a5fa' },
  { hint: 'float', example: 'rate_hz: float', ros: 'Frecuencia de timer', color: '#fbbf24' },
  { hint: 'bool', example: 'verbose: bool', ros: 'Flags de configuración', color: '#c084fc' },
  { hint: 'list[str]', example: 'topics: list[str]', ros: 'Lista de topics a suscribir', color: '#f97316' },
  { hint: 'None', example: '-> None', ros: 'Funciones sin retorno (callbacks)', color: '#94a3b8' },
];

// ── __main__ guard cases ───────────────────────────────────────
const guardCases = [
  { how: 'python3 script.py', value: "'__main__'", result: 'main() SE ejecuta', color: '#4ade80' },
  { how: 'import script', value: "'script'", result: 'main() NO se ejecuta', color: '#f87171' },
];

// ── argparse features ──────────────────────────────────────────
const argparseFeatures = [
  { icon: 'help', title: '--help automático', desc: 'Genera documentación del CLI sin escribir nada extra', color: '#4ade80' },
  { icon: 'verified', title: 'Tipos automáticos', desc: 'Convierte "10.5" → 10.5 float y valida el formato', color: '#60a5fa' },
  { icon: 'tune', title: 'Defaults', desc: 'Valores por defecto cuando el usuario no especifica', color: '#fbbf24' },
  { icon: 'flag', title: 'Flags booleanos', desc: 'action="store_true" para --verbose sin valor', color: '#c084fc' },
];

// ── Logging levels ─────────────────────────────────────────────
const logLevels = [
  { level: 'DEBUG', api: 'get_logger().debug()', example: 'velocidad_actual = 0.5 m/s', use: 'Diagnóstico detallado en desarrollo', color: '#94a3b8' },
  { level: 'INFO', api: 'get_logger().info()', example: 'Nodo iniciado correctamente', use: 'Progreso normal — el default', color: '#60a5fa' },
  { level: 'WARN', api: 'get_logger().warn()', example: 'Batería baja: 15%', use: 'Algo inesperado pero recuperable', color: '#fbbf24' },
  { level: 'ERROR', api: 'get_logger().error()', example: 'Conexión perdida con sensor', use: 'Error — el sistema sigue corriendo', color: '#f87171' },
  { level: 'FATAL', api: 'get_logger().fatal()', example: 'Hardware dañado — parando robot', use: 'Error crítico — el robot debe parar', color: '#dc2626' },
];

// ── Debug techniques ───────────────────────────────────────────
const debugTechniques = [
  {
    level: 'Básico', title: 'f-strings de diagnóstico', icon: 'print', color: '#fbbf24',
    code: [
      'def callback(self, msg):',
      '    # Agrega f-string para ver el estado',
      '    self.get_logger().debug(',
      "        f'[DEBUG] recibido={msg.data!r} tipo={type(msg.data).__name__}'",
      '    )',
    ].join('\n'),
    pro: 'Simple, instantáneo, no requiere setup',
    con: 'Eliminar manualmente al terminar — usa DEBUG level',
  },
  {
    level: 'Intermedio', title: 'rqt_console — GUI de logs', icon: 'terminal', color: '#60a5fa',
    code: [
      '# En una terminal: lanza el nodo',
      'ros2 run mi_pkg mi_nodo',
      '',
      '# En otra terminal: abre la GUI de logs',
      'ros2 run rqt_console rqt_console',
      '',
      '# Filtra por nodo, nivel o texto',
      '# Guarda logs en archivo desde la GUI',
    ].join('\n'),
    pro: 'Filtra, colorea y guarda logs sin tocar código',
    con: 'Solo para sistemas con GUI (no robots embebidos)',
  },
  {
    level: 'Avanzado', title: 'VS Code Debugger', icon: 'code', color: '#c084fc',
    code: [
      '// .vscode/launch.json',
      '{',
      '  "configurations": [{',
      '    "name": "ROS 2 Python",',
      '    "type": "python",',
      '    "request": "launch",',
      '    "program": "${file}",',
      '    "env": {"PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages"}',
      '  }]',
      '}',
    ].join('\n'),
    pro: 'Breakpoints visuales, inspección de variables en vivo',
    con: 'Requiere configuración — más difícil con nodos ROS 2 reales',
  },
];

// ── PYTHONPATH items ───────────────────────────────────────────
const pythonpathItems = [
  { title: 'Directorio del script', path: '/home/user/mi_proyecto/', color: '#4ade80', rosNote: undefined },
  { title: 'Variable PYTHONPATH', path: 'export PYTHONPATH=/mi/libreria:$PYTHONPATH', color: '#22d3ee', rosNote: undefined },
  { title: 'Librerías estándar', path: '/usr/lib/python3.12/', color: '#fbbf24', rosNote: undefined },
  { title: 'Paquetes instalados', path: '/usr/lib/python3/dist-packages/', color: '#f97316', rosNote: undefined },
  { title: 'ROS 2 packages', path: '/opt/ros/jazzy/lib/python3.12/site-packages/', color: '#60a5fa', rosNote: 'Solo si hiciste source setup.bash' },
];

// ── Challenge ──────────────────────────────────────────────────
const challengeReqs = [
  { icon: 'verified', color: '#f87171', text: 'Shebang correcto en la primera línea' },
  { icon: 'verified', color: '#f87171', text: 'Type hints en todas las funciones y métodos' },
  { icon: 'verified', color: '#4ade80', text: 'argparse con al menos 3 argumentos: --name, --rate, --verbose' },
  { icon: 'verified', color: '#4ade80', text: 'get_logger() en lugar de print() en todo el código' },
  { icon: 'verified', color: '#60a5fa', text: 'Guard if __name__ == "__main__"' },
  { icon: 'verified', color: '#60a5fa', text: 'try/except/finally para manejo limpio de Ctrl+C' },
];

const challengeHints = [
  'El parser se crea en una función parse_args() separada de main()',
  'El nombre del nodo en super().__init__() puede venir de args.name',
  'if args.verbose: self.get_logger().set_level(LoggingSeverity.DEBUG)',
  'En finally: siempre llama node.destroy_node() y rclpy.shutdown()',
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    msg: 'bash: ./script.py: Permission denied',
    summary: 'El archivo no tiene permiso de ejecución',
    cause: 'Linux crea archivos sin el bit de ejecución por seguridad. Debes otorgarlo explícitamente.',
    steps: [
      'Ejecuta: chmod +x script.py',
      'Verifica con: ls -la script.py (debe aparecer -rwxr-xr-x)',
      'Vuelve a intentar: ./script.py',
    ],
    code: 'chmod +x script.py\nls -la script.py',
    open: false,
  },
  {
    msg: "ModuleNotFoundError: No module named 'rclpy'",
    summary: 'ROS 2 no está activo en esta terminal',
    cause: 'No hiciste source del setup.bash de ROS 2, o está mal configurado el .bashrc.',
    steps: [
      'Activa ROS 2 en la terminal actual: source /opt/ros/jazzy/setup.bash',
      'Verifica: echo $ROS_DISTRO (debe mostrar "jazzy")',
      'Para hacerlo permanente: echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc',
    ],
    code: 'source /opt/ros/jazzy/setup.bash\necho $ROS_DISTRO',
    open: false,
  },
  {
    msg: 'SyntaxError: invalid syntax (línea 1, shebang)',
    summary: 'Ejecutaste el script con python2 o con el intérprete equivocado',
    cause: 'El comando "python" en muchos sistemas apunta a Python 2 que no entiende el shebang.',
    steps: [
      'Siempre usa: python3 script.py (no python)',
      'O configura un alias: echo "alias python=python3" >> ~/.bashrc',
      'O usa el método directo con shebang + chmod +x: ./script.py',
    ],
    code: 'python3 script.py\n# O bien\n./script.py',
    open: false,
  },
  {
    msg: 'TypeError: callback() takes 1 positional argument but 2 were given',
    summary: 'El callback de una suscripción no tiene el parámetro "msg"',
    cause: 'Los callbacks de suscriptores reciben el mensaje como primer argumento (además de self).',
    steps: [
      'Revisa la firma del callback: debe ser def callback(self, msg): no def callback(self):',
      'El tipo del msg depende del tipo de topic: String, Float32, etc.',
    ],
    code: [
      '# ❌ Incorrecto',
      'def mi_callback(self):',
      '    pass',
      '',
      '# ✅ Correcto',
      'def mi_callback(self, msg: String) -> None:',
      '    self.get_logger().info(msg.data)',
    ].join('\n'),
    open: false,
  },
  {
    msg: 'AttributeError: __init__() missing required positional argument',
    summary: 'Olvidaste llamar a super().__init__() en tu nodo',
    cause: 'La clase Node de rclpy requiere inicializarse con el nombre del nodo.',
    steps: [
      'Añade super().__init__("nombre_nodo") como primera línea del __init__',
      'El nombre debe ser único en el sistema ROS 2',
    ],
    code: [
      'class MiNodo(Node):',
      '    def __init__(self) -> None:',
      "        super().__init__('mi_nodo')  # ← no olvides esto",
    ].join('\n'),
    open: false,
  },
]);

// ── Summary items ──────────────────────────────────────────────
const summaryItems = [
  { cmd: 'chmod +x', desc: 'Otorgar permiso de ejecución al script', example: 'chmod +x robot.py', color: '#f97316' },
  { cmd: '#!/usr/bin/env', desc: 'Shebang portable — busca intérprete en PATH', example: '#!/usr/bin/env python3', color: '#4ade80' },
  { cmd: 'type hints', desc: 'Anotaciones de tipo para seguridad y autocompletado', example: 'def fn(x: float) -> None:', color: '#c084fc' },
  { cmd: 'argparse', desc: 'Argumentos de CLI con tipos, defaults y --help', example: 'parser.add_argument("--rate", type=float)', color: '#60a5fa' },
  { cmd: 'get_logger()', desc: 'Logger de ROS 2 con niveles y timestamp', example: 'self.get_logger().info("msg")', color: '#22d3ee' },
  { cmd: '__name__', desc: 'Guard para evitar ejecución al importar', example: 'if __name__ == "__main__": main()', color: '#fbbf24' },
];

// ═══════════════════════════════════════════════════════════════
// CODE STRING CONSTANTS
// ═══════════════════════════════════════════════════════════════

const noTypesCode = [
  'def calcular_velocidad(distancia, tiempo):',
  '    # ¿distancia en metros? ¿pies?',
  '    # ¿retorna m/s? ¿km/h?',
  '    return distancia / tiempo',
].join('\n');

const withTypesCode = [
  'def calcular_velocidad(',
  '    distancia: float,  # metros',
  '    tiempo: float,     # segundos',
  ') -> float:            # m/s',
  '    """Calcula velocidad en m/s."""',
  '    return distancia / tiempo',
].join('\n');

const mainGuardCode = [
  '# modulo.py',
  'print("Cargando módulo...")',
  '',
  'def helper() -> str:',
  '    return "ayuda"',
  '',
  'def main() -> None:',
  '    print(f"Ejecutando como script: {helper()}")',
  '',
  'if __name__ == "__main__":',
  '    main()  # Solo se ejecuta con: python3 modulo.py',
  '',
  '# Si alguien hace: import modulo',
  '# → imprime "Cargando módulo..." pero NO ejecuta main()',
].join('\n');

const sysArgvCode = [
  'import sys',
  '',
  '# python3 script.py robot_1 10.0',
  'robot = sys.argv[1]  # "robot_1"',
  'rate  = float(sys.argv[2])  # 10.0',
  '',
  '# ❌ Sin validación — explota si faltan args',
].join('\n');

const argparseCode = [
  'import argparse',
  '',
  'parser = argparse.ArgumentParser()',
  'parser.add_argument("--robot",',
  '    type=str, default="robot_1")',
  'parser.add_argument("--rate",',
  '    type=float, default=10.0)',
  'parser.add_argument("--verbose",',
  '    action="store_true")',
  'args = parser.parse_args()',
  '',
  '# python3 script.py --robot r2 --rate 5.0',
  '# python3 script.py --help  ← generado gratis',
].join('\n');

const printCode = [
  'def timer_callback(self):',
  '    msg = self.leer_sensor()',
  '    print(f"Sensor: {msg}")  # ❌',
  '    # Sin timestamp, sin nodo, sin nivel',
  '    # No se puede filtrar ni redirigir',
].join('\n');

const loggerCode = [
  'def timer_callback(self):',
  '    msg = self.leer_sensor()',
  '    self.get_logger().info(  # ✅',
  '        f"Sensor: {msg}"',
  '    )',
  '    # [INFO] [1234.56] [mi_nodo]: Sensor: ...',
  '    # Filtrable, redirigible, con contexto',
].join('\n');

const throttleCode = [
  '# Cada 2 segundos, aunque se llame 100x/s',
  'self.get_logger().warn_throttle(2.0, "Motor caliente")',
  '',
  '# Solo una vez en toda la vida del nodo',
  'self.get_logger().info_once("Nodo iniciado")',
].join('\n');

const sysPathCode = [
  'import sys',
  "print('\\n'.join(sys.path))",
  '',
  '# Salida esperada con ROS 2 activo:',
  '# /opt/ros/jazzy/lib/python3.12/site-packages',
  '# /usr/lib/python3/dist-packages',
  '# ...',
].join('\n');

const talkerNodeCode = [
  '#!/usr/bin/env python3',
  '"""talker_node.py — Nodo publicador ROS 2 con buenas prácticas."""',
  '',
  'import argparse',
  'import rclpy',
  'from rclpy.node import Node',
  'from std_msgs.msg import String',
  '',
  '',
  'class TalkerNode(Node):',
  '    """Publica mensajes en un topic a frecuencia configurable."""',
  '',
  '    def __init__(self, robot_name: str, rate_hz: float) -> None:',
  "        super().__init__('talker')",
  '        self.robot_name = robot_name',
  '        self.counter: int = 0',
  '',
  '        self.publisher = self.create_publisher(',
  "            String, 'chatter', 10",
  '        )',
  '        self.create_timer(1.0 / rate_hz, self.publish_message)',
  '        self.get_logger().info(',
  "            f'Talker iniciado — robot: {robot_name}, rate: {rate_hz} Hz'",
  '        )',
  '',
  '    def publish_message(self) -> None:',
  '        """Publica un mensaje con nombre de robot y contador."""',
  '        msg = String()',
  "        msg.data = f'[{self.robot_name}] Mensaje: {self.counter}'",
  '        self.publisher.publish(msg)',
  "        self.get_logger().debug(f'Publicado: {msg.data}')",
  '        self.counter += 1',
  '',
  '',
  'def parse_args() -> argparse.Namespace:',
  '    parser = argparse.ArgumentParser(',
  "        description='Nodo publicador ROS 2 configurable'",
  '    )',
  '    parser.add_argument(',
  "        '--robot', type=str, default='Robot',",
  "        help='Nombre del robot (default: Robot)'",
  '    )',
  '    parser.add_argument(',
  "        '--rate', type=float, default=1.0,",
  "        help='Frecuencia en Hz (default: 1.0)'",
  '    )',
  '    return parser.parse_args()',
  '',
  '',
  'def main() -> None:',
  '    args = parse_args()',
  '    rclpy.init()',
  '    node = TalkerNode(robot_name=args.robot, rate_hz=args.rate)',
  '    try:',
  '        rclpy.spin(node)',
  '    except KeyboardInterrupt:',
  "        node.get_logger().info('Detenido por el usuario')",
  '    finally:',
  '        node.destroy_node()',
  '        rclpy.shutdown()',
  '',
  '',
  "if __name__ == '__main__':",
  '    main()',
].join('\n');

const runSteps = [
  { label: 'Dar permisos', cmd: 'chmod +x talker_node.py' },
  { label: 'Ejecutar con nombre de robot', cmd: './talker_node.py --robot R2D2 --rate 2.0' },
  { label: 'Escuchar el topic (otra terminal)', cmd: 'ros2 topic echo /chatter' },
  { label: 'Ver ayuda', cmd: './talker_node.py --help' },
];

const challengeSpec = [
  '#!/usr/bin/env python3',
  '# Tu nodo debe hacer esto:',
  '#',
  '# 1. Aceptar: --name (str), --rate (float), --verbose (flag)',
  '# 2. Publicar en /robot_status: "[nombre] tick N"',
  '# 3. Usar get_logger().info() para mensajes normales',
  '# 4. Usar get_logger().debug() si --verbose',
  '# 5. Manejar Ctrl+C limpiamente',
  '#',
  '# Ejecución esperada:',
  '# ./mi_nodo.py --name Atlas --rate 0.5 --verbose',
  '# [INFO] [Atlas] Nodo iniciado',
  '# [DEBUG] [Atlas] Publicando tick 0',
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

/* ── Fact pills */
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
   ANNOTATED CODE
══════════════════════════════════════════ */
.annotated-code {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; overflow: hidden; font-family: 'Fira Code', monospace;
}
.ac-chrome {
  background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle);
  padding: 10px 16px; display: flex; align-items: center; gap: 12px;
}
.ac-dots { display: flex; gap: 6px; }
.acd {
  width: 12px; height: 12px; border-radius: 50%; display: block;
}
.acd.red    { background: #ff5f56; }
.acd.yellow { background: #ffbd2e; }
.acd.green  { background: #27c93f; }
.ac-filename { color: var(--text-muted); font-size: .88rem; }
.ac-body { padding: 1rem 1.5rem; }
.ac-line {
  display: grid; grid-template-columns: 3ch 1fr auto;
  align-items: center; gap: 12px;
  padding: 3px 0; min-height: 1.6rem; position: relative;
  transition: background .15s;
}
.ac-line:hover { background: var(--bg-surface-hover); border-radius: 6px; }
.ac-line-hl {
  background: rgba(96,165,250,.08);
  border-left: 3px solid #60a5fa; padding-left: 8px; border-radius: 0 6px 6px 0;
}
.ac-line-empty { min-height: 1rem; }
.ac-num { color: var(--text-muted); text-align: right; user-select: none; font-size: .82rem; }
.ac-code { font-size: .9rem; color: var(--text-secondary); }
.ac-annotation {
  display: flex; align-items: center; gap: 4px;
  font-family: sans-serif; font-size: .75rem; color: #60a5fa;
  background: rgba(96,165,250,.1); border: 1px solid rgba(96,165,250,.2);
  border-radius: 999px; padding: 1px 10px; white-space: nowrap;
}

/* token colors */
:deep(.tk-shebang) { color: #f472b6; font-style: italic; }
:deep(.tk-str)     { color: #4ade80; }
:deep(.tk-kw)      { color: #c084fc; font-weight: 700; }
:deep(.tk-class)   { color: #fbbf24; }
:deep(.tk-fn)      { color: #60a5fa; }
:deep(.tk-comment) { color: var(--text-muted); font-style: italic; }

/* ── Node lifecycle */
.lifecycle-flow {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 1.5rem;
}
.lf-title {
  display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600;
  color: var(--text-secondary); margin-bottom: 1.25rem;
}
.lf-steps {
  display: flex; align-items: center; flex-wrap: wrap; gap: 8px;
}
.lf-step-wrap { display: flex; align-items: center; gap: 8px; }
.lf-step {
  background: var(--bg-surface-hover); border: 1px solid var(--border-subtle);
  border-top: 2px solid var(--lf-color, var(--border-medium));
  border-radius: 10px; padding: 10px 14px;
  display: flex; flex-direction: column; align-items: center; gap: 4px; text-align: center;
  min-width: 100px;
}
.lf-fn { font-family: 'Fira Code', monospace; font-size: .78rem; color: var(--text-code); background: none; padding: 0; }
.lf-desc { font-size: .72rem; color: var(--text-muted); }
.lf-arrow { color: var(--text-muted); font-size: 1.2rem; flex-shrink: 0; }

/* ══════════════════════════════════════════
   SHEBANG
══════════════════════════════════════════ */
.shebang-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 16px; }
.shebang-card {
  background: var(--bg-surface); border-radius: 14px; overflow: hidden;
  border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--shb-color, var(--border-medium));
  display: flex; flex-direction: column; gap: 10px; padding: 1.25rem;
}
.shb-header { display: flex; align-items: center; gap: 8px; }
.shb-status { font-size: .82rem; font-weight: 700; }
.shb-title { font-size: 1rem; font-weight: 700; color: var(--text-primary); }
.shb-reason {
  font-size: .85rem; color: var(--text-secondary); line-height: 1.5;
  background: var(--bg-surface-hover); border-radius: 8px; padding: 10px 12px;
}

/* ══════════════════════════════════════════
   CHMOD VISUAL
══════════════════════════════════════════ */
.chmod-visual {
  display: grid; grid-template-columns: 1fr auto 1fr; gap: 16px; align-items: center;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 2rem;
}
.cv-panel { display: flex; flex-direction: column; align-items: center; gap: 8px; }
.cvp-label { font-size: .75rem; font-weight: 700; letter-spacing: .06em; color: var(--text-muted); text-transform: uppercase; }
.cvp-file {
  background: var(--bg-surface-hover); border: 1px solid var(--border-subtle);
  border-radius: 12px; padding: 1.5rem 2rem; text-align: center;
  display: flex; flex-direction: column; align-items: center; gap: 8px; width: 100%;
}
.cvp-name { font-family: 'Fira Code', monospace; font-size: 1rem; font-weight: 700; color: var(--text-primary); background: none; padding: 0; }
.cvp-perms { display: flex; gap: 0; font-family: 'Fira Code', monospace; font-size: 1rem; }
.perm-rw  { color: var(--text-muted); }
.perm-r   { color: var(--text-muted); }
.perm-rwx { color: #4ade80; }
.perm-rx  { color: #4ade80; opacity: .6; }
.cvp-status {
  display: flex; align-items: center; gap: 6px; font-size: .83rem; font-weight: 700;
  padding: 5px 14px; border-radius: 999px;
}
.cvp-status-bad { background: rgba(248,113,113,.12); color: #f87171; }
.cvp-status-ok  { background: rgba( 74,222,128,.12); color: #4ade80; }
.error-label { color: #f87171; }
.success-label { color: #4ade80; }
.cv-transform {
  display: flex; flex-direction: column; align-items: center; gap: 10px;
}
.cvt-cmd { width: 100%; }
.cvt-arrow { display: flex; align-items: center; }

/* Exec methods */
.exec-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.exec-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--em-color, var(--border-medium));
  border-radius: 14px; padding: 1.25rem; position: relative;
  display: flex; flex-direction: column; gap: 10px; transition: all .25s;
}
.exec-card:hover { transform: translateY(-4px); box-shadow: var(--shadow-md); }
.exec-recommended { box-shadow: 0 0 0 2px color-mix(in srgb, var(--em-color) 30%, transparent); }
.ec-num {
  width: 36px; height: 36px; border-radius: 50%; display: flex; align-items: center; justify-content: center;
  font-size: 1.1rem; font-weight: 800; color: #1e1e1e;
}
.ec-title { font-size: 1rem; font-weight: 700; color: var(--text-primary); }
.ec-meta { display: flex; flex-direction: column; gap: 6px; margin-top: 4px; }
.ec-pros { display: flex; flex-direction: column; gap: 4px; }
.ec-item { display: flex; align-items: center; gap: 5px; font-size: .8rem; color: var(--text-secondary); }
.ec-note {
  display: flex; align-items: center; font-size: .78rem; color: var(--text-muted);
  background: var(--bg-surface-hover); border-radius: 6px; padding: 5px 8px;
}
.ec-recommended-badge {
  position: absolute; top: 10px; right: 10px;
  font-size: .68rem; font-weight: 800; letter-spacing: .06em;
  background: rgba(96,165,250,.15); color: #60a5fa;
  border: 1px solid rgba(96,165,250,.3); border-radius: 999px; padding: 2px 8px;
}

/* ══════════════════════════════════════════
   TYPE HINTS
══════════════════════════════════════════ */
.types-comparison {
  display: grid; grid-template-columns: 1fr auto 1fr; gap: 12px; align-items: start;
}
.tc-panel { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; min-width: 0; }
.tc-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  border-bottom: 1px solid var(--border-subtle); font-size: .86rem; font-weight: 700;
}
.tc-header-bad  { background: rgba(248,113,113,.08); color: #f87171; }
.tc-header-good { background: rgba( 74,222,128,.08); color: #4ade80; }
.tc-arrow {
  display: flex; flex-direction: column; align-items: center; gap: 4px;
  padding-top: 2.5rem; font-size: .72rem; color: var(--text-muted); flex-shrink: 0;
}

.type-ref-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 10px; }
.type-ref-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--tr-color, var(--border-medium));
  border-radius: 10px; padding: 12px 14px;
  display: flex; flex-direction: column; gap: 4px; min-width: 0;
}
.tr-hint    { font-family: 'Fira Code', monospace; font-size: 1rem; font-weight: 800; color: var(--tr-color); background: none; padding: 0; }
.tr-example { font-family: 'Fira Code', monospace; font-size: .78rem; color: var(--text-muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.tr-ros     { font-size: .78rem; color: var(--text-secondary); }

/* __main__ guard */
.guard-section {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid #c084fc; border-radius: 14px; padding: 1.5rem;
}
.guard-title {
  display: flex; align-items: center; gap: 8px; font-size: 1rem; font-weight: 700;
  color: var(--text-primary); margin-bottom: 1rem;
}
.guard-cases { display: flex; flex-direction: column; gap: 8px; }
.guard-case {
  display: grid; grid-template-columns: 1fr auto 1fr 2fr; align-items: center; gap: 10px;
  background: var(--bg-surface-hover); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--gc-color, var(--border-medium));
  border-radius: 8px; padding: 10px 14px;
}
.gc-how    { font-family: 'Fira Code', monospace; font-size: .84rem; color: var(--text-secondary); }
.gc-arrow  { color: var(--text-muted); }
.gc-value  { font-family: 'Fira Code', monospace; font-size: .84rem; font-weight: 700; background: none; padding: 0; }
.gc-result { font-size: .82rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   ARGUMENTS
══════════════════════════════════════════ */
.arg-panel { height: 100%; display: flex; flex-direction: column; gap: 8px; }
.ap-header {
  display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700;
  padding: 8px 12px; border-radius: 8px; color: var(--text-primary);
}
.ap-header-basic { background: rgba(148,163,184,.1); }
.ap-header-pro   { background: rgba( 74,222,128,.1); }
.ap-note {
  display: flex; align-items: flex-start; gap: 6px;
  font-size: .82rem; color: var(--text-secondary);
  padding: 8px 12px; border-radius: 8px; border: 1px solid var(--border-subtle);
}
.ap-note-bad  { background: rgba(251,191,36,.06); }
.ap-note-good { background: rgba( 74,222,128,.06); }

.argparse-features { }
.af-title { font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 10px; }
.af-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 10px; }
.af-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--af-color, var(--border-medium));
  border-radius: 10px; padding: 12px 14px;
  display: flex; flex-direction: column; gap: 4px;
}
.af-title-text { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.af-desc { font-size: .82rem; color: var(--text-secondary); line-height: 1.4; }

/* ══════════════════════════════════════════
   LOGGING LEVELS
══════════════════════════════════════════ */
.log-levels { display: flex; flex-direction: column; gap: 8px; }
.log-level-row {
  display: grid; grid-template-columns: 90px 1fr 2fr 1fr; align-items: center; gap: 12px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 10px; padding: 10px 14px;
}
.ll-badge {
  font-size: .75rem; font-weight: 900; letter-spacing: .06em;
  border: 1px solid; border-radius: 6px; padding: 3px 8px; text-align: center;
}
.ll-api { font-family: 'Fira Code', monospace; font-size: .8rem; color: var(--text-muted); background: none; padding: 0; }
.ll-terminal {
  font-family: 'Fira Code', monospace; font-size: .78rem;
  background: var(--bg-code); border-radius: 6px; padding: 4px 10px;
  border-left: 3px solid; display: flex; align-items: center; gap: 6px;
  overflow: hidden;
}
.ll-bracket { font-weight: 800; }
.ll-time, .ll-node { color: var(--text-muted); }
.ll-msg { color: var(--text-code); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.ll-use { font-size: .8rem; color: var(--text-muted); }

.print-vs-log { display: grid; grid-template-columns: repeat(2,1fr); gap: 14px; }
.pvl-panel { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.pvl-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  border-bottom: 1px solid var(--border-subtle); font-size: .86rem; font-weight: 700;
}
.pvl-bad  .pvl-header { background: rgba(248,113,113,.08); color: #f87171; }
.pvl-good .pvl-header { background: rgba( 74,222,128,.08); color: #4ade80; }

/* ══════════════════════════════════════════
   DEBUG CARDS
══════════════════════════════════════════ */
.debug-cards { display: grid; grid-template-columns: repeat(3,1fr); gap: 16px; }
.debug-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--dc-color, var(--border-medium));
  border-radius: 14px; overflow: hidden; display: flex; flex-direction: column; min-width: 0;
}
.dc-header {
  display: flex; align-items: center; gap: 12px; padding: 1rem 1.25rem;
  border-bottom: 1px solid var(--border-subtle);
  background: color-mix(in srgb, var(--dc-color) 6%, transparent);
}
.dc-icon-wrap { width: 40px; height: 40px; border-radius: 10px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.dc-level { font-size: .72rem; font-weight: 800; letter-spacing: .06em; text-transform: uppercase; }
.dc-title { font-size: .95rem; font-weight: 700; color: var(--text-primary); }
.dc-meta { display: flex; flex-direction: column; gap: 6px; padding: 10px 14px; background: var(--bg-surface-hover); }
.dc-pro, .dc-con { display: flex; align-items: flex-start; gap: 5px; font-size: .8rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   PYTHONPATH
══════════════════════════════════════════ */
.pythonpath-flow { display: flex; flex-direction: column; gap: 6px; }
.pp-item {
  display: grid; grid-template-columns: auto 1fr auto; align-items: center; gap: 12px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--pp-color, var(--border-medium));
  border-radius: 10px; padding: 10px 14px;
}
.pp-num {
  width: 28px; height: 28px; border-radius: 50%; display: flex; align-items: center; justify-content: center;
  font-size: .82rem; font-weight: 800; color: #1e1e1e; flex-shrink: 0;
}
.pp-body { display: flex; flex-direction: column; gap: 2px; min-width: 0; }
.pp-title { font-size: .88rem; font-weight: 600; color: var(--text-primary); }
.pp-path  { font-family: 'Fira Code', monospace; font-size: .78rem; color: var(--text-muted); background: none; padding: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.pp-ros {
  display: flex; align-items: center; gap: 4px; font-size: .75rem; color: #4ade80;
  background: rgba(74,222,128,.08); border: 1px solid rgba(74,222,128,.2);
  border-radius: 999px; padding: 2px 10px; white-space: nowrap;
}

/* ══════════════════════════════════════════
   COMPLETE SCRIPT
══════════════════════════════════════════ */
.complete-script-header {
  display: flex; align-items: center; gap: 8px; font-size: .88rem; font-weight: 600;
  color: var(--text-secondary); background: rgba(251,191,36,.08);
  border: 1px solid rgba(251,191,36,.2); border-radius: 8px; padding: 8px 14px;
}
.run-steps { }
.rs-title {
  display: flex; align-items: center; gap: 8px; font-size: .95rem; font-weight: 700;
  color: var(--text-primary); margin-bottom: 12px;
}
.rs-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 10px; }
.rs-step {
  display: flex; align-items: flex-start; gap: 10px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 10px; padding: 12px 14px; min-width: 0;
}
.rs-num {
  min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0;
  background: rgba(74,222,128,.15); color: #4ade80; border: 1px solid rgba(74,222,128,.3);
  display: flex; align-items: center; justify-content: center;
  font-size: .82rem; font-weight: 800; margin-top: 2px;
}
.rs-body { flex: 1; min-width: 0; display: flex; flex-direction: column; gap: 6px; }
.rs-label { font-size: .85rem; font-weight: 600; color: var(--text-secondary); }

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
  padding: 1rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s;
}
.err-header:hover { background: var(--bg-surface-hover); }
.err-left { display: flex; align-items: flex-start; gap: 12px; min-width: 0; }
.err-num {
  min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0;
  background: rgba(239,68,68,.15); color: #ef4444;
  font-size: .8rem; font-weight: 800;
  display: flex; align-items: center; justify-content: center;
}
.err-msg     { font-size: .86rem; display: block; margin-bottom: 3px; color: #ef4444; background: none; padding: 0; word-break: break-all; }
.err-summary { font-size: .82rem; color: var(--text-muted); }
.err-body {
  padding: 1rem 1.5rem 1.25rem; border-top: 1px solid var(--border-subtle);
  display: flex; flex-direction: column; gap: 10px;
}
.err-cause, .err-fix {
  font-size: .88rem; color: var(--text-secondary);
  display: flex; align-items: flex-start; gap: 6px;
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
.challenge-requirements { background: var(--bg-surface-hover); border-radius: 10px; padding: 1rem 1.25rem; }
.cr-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 8px; }
.cr-list { display: flex; flex-direction: column; gap: 6px; }
.cr-item { display: flex; align-items: center; gap: 8px; font-size: .88rem; color: var(--text-secondary); }
:deep(.answer-header) {
  background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25);
  border-radius: 10px; color: #22c55e;
}
.answer-body {
  background: var(--bg-surface-hover); padding: 1.25rem;
  border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px;
}
.answer-row { display: flex; align-items: baseline; gap: 8px; font-size: .9rem; color: var(--text-secondary); }

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
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .95rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; }
.sc-desc    { font-size: .83rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .73rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .log-level-row { grid-template-columns: 80px 1fr 1fr; }
  .log-level-row .ll-use { display: none; }
}

@media (max-width: 900px) {
  .exec-grid         { grid-template-columns: 1fr 1fr; }
  .debug-cards       { grid-template-columns: 1fr 1fr; }
  .type-ref-grid     { grid-template-columns: repeat(2,1fr); }
  .summary-grid      { grid-template-columns: repeat(2,1fr); }
  .types-comparison  { grid-template-columns: 1fr; }
  .tc-arrow          { display: none; }
  .print-vs-log      { grid-template-columns: 1fr; }
  .log-level-row     { grid-template-columns: 70px 1fr; }
  .log-level-row .ll-terminal { display: none; }
  .guard-case        { grid-template-columns: 1fr auto 1fr; }
  .guard-case .gc-result { display: none; }
}

@media (max-width: 768px) {
  .shebang-grid   { grid-template-columns: 1fr; }
  .exec-grid      { grid-template-columns: 1fr; }
  .debug-cards    { grid-template-columns: 1fr; }
  .type-ref-grid  { grid-template-columns: repeat(2,1fr); }
  .chmod-visual   { grid-template-columns: 1fr; }
  .cv-transform   { flex-direction: row; }
  .rs-grid        { grid-template-columns: 1fr; }
  .af-grid        { grid-template-columns: 1fr; }
  .lf-steps       { flex-direction: column; align-items: flex-start; }
  .lf-arrow       { transform: rotate(90deg); }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
  .ac-annotation  { display: none; }
}

@media (max-width: 480px) {
  .summary-grid  { grid-template-columns: 1fr; }
  .type-ref-grid { grid-template-columns: 1fr; }
  .shebang-grid  { grid-template-columns: 1fr; }
  .fact-pills    { flex-direction: column; }
}
</style>
