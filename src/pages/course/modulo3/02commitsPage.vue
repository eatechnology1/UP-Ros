<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Los commits son el corazón de Git. Cada commit es una fotografía inmutable de tu proyecto
        en un momento exacto — con un ID único, autor, fecha y lista de cambios.
        Dominar cómo, cuándo y qué commitear te diferencia de un usuario amateur de un
        <strong>desarrollador profesional</strong>.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div v-for="f in facts" :key="f.label" class="fact-pill">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 EL RITUAL DE 3 PASOS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        El Ritual de los 3 Pasos
      </SectionTitle>

      <TextBlock>
        Git no adivina qué quieres guardar. Tú decides qué va en cada commit. Este ciclo se
        repite decenas de veces al día — interiorizarlo es clave:
      </TextBlock>

      <div class="ritual-steps q-mt-lg">
        <div v-for="(step, idx) in ritualSteps" :key="step.title" class="rst-step">
          <div class="rst-icon" :style="{ '--rst-color': step.color }">
            <q-icon :name="step.icon" size="26px" style="color:#1e293b" />
          </div>
          <div class="rst-content">
            <div class="rst-num" :style="{ color: step.color }">{{ idx + 1 }}. {{ step.title }}</div>
            <div class="rst-desc">{{ step.desc }}</div>
            <CodeBlock :hide-header="true" lang="bash" :content="step.code" :copyable="true" />
          </div>
          <div v-if="idx < ritualSteps.length - 1" class="rst-arrow">
            <q-icon name="arrow_downward" size="20px" style="color:var(--text-muted)" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         02 STAGING QUIRÚRGICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Staging Quirúrgico — Elige con Precisión
      </SectionTitle>

      <TextBlock>
        <code>git add .</code> agrega TODO — pero el staging real es selectivo. Elige exactamente
        qué cambios van en cada commit. Un master de Git nunca usa <code>git add .</code> ciegamente:
      </TextBlock>

      <div class="staging-grid q-mt-lg">
        <div v-for="cmd in stagingCmds" :key="cmd.cmd" class="stg-card"
          :style="{ '--stg-color': cmd.color }">
          <div class="stgc-header">
            <code class="stgc-cmd">{{ cmd.cmd }}</code>
            <span v-if="cmd.tag" class="stgc-tag" :style="{ background: cmd.color + '18', color: cmd.color, borderColor: cmd.color + '30' }">{{ cmd.tag }}</span>
          </div>
          <div class="stgc-desc">{{ cmd.desc }}</div>
          <div v-if="cmd.warning" class="stgc-warn">
            <q-icon name="warning_amber" size="13px" color="warning" />
            {{ cmd.warning }}
          </div>
        </div>
      </div>

      <!-- git add -p detail -->
      <div class="patch-detail q-mt-xl">
        <div class="pd-title">
          <code>git add -p</code> — Staging por hunks (el modo profesional)
        </div>
        <TextBlock>
          Cuando modificaste varias cosas en el mismo archivo para commits distintos,
          <code>-p</code> te permite elegir bloque por bloque:
        </TextBlock>
        <CodeBlock title="Sesión interactiva de git add -p" lang="bash"
          :content="patchCode" :copyable="false" class="q-mt-md" />
        <div class="patch-keys q-mt-md">
          <div v-for="k in patchKeys" :key="k.key" class="pk-item"
            :style="{ '--pk-color': k.color }">
            <code class="pk-key">{{ k.key }}</code>
            <span class="pk-desc">{{ k.desc }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 ARTE DEL MENSAJE
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        El Arte del Mensaje de Commit
      </SectionTitle>

      <TextBlock>
        Un buen mensaje de commit explica el <strong>QUÉ</strong> y el <strong>POR QUÉ</strong>,
        no el CÓMO (eso ya lo hace el código). El mensaje es para el "tú del futuro" y para tu equipo:
      </TextBlock>

      <!-- Bad vs Good comparison -->
      <div class="msg-comparison q-mt-lg">
        <div class="msgc-card msgc-bad">
          <div class="msgc-header">
            <q-icon name="cancel" size="20px" style="color:#f87171" />
            <span class="msgc-title" style="color:#f87171">Mensajes que dañan el proyecto</span>
          </div>
          <div class="msgc-items">
            <div v-for="m in badMessages" :key="m.msg" class="msgi-item">
              <code class="msgi-msg">{{ m.msg }}</code>
              <span class="msgi-problem">{{ m.problem }}</span>
            </div>
          </div>
        </div>

        <div class="msgc-divider">
          <q-icon name="arrow_forward" size="24px" style="color:var(--text-muted)" />
        </div>

        <div class="msgc-card msgc-good">
          <div class="msgc-header">
            <q-icon name="check_circle" size="20px" style="color:#4ade80" />
            <span class="msgc-title" style="color:#4ade80">Mensajes que construyen contexto</span>
          </div>
          <div class="msgc-items">
            <div v-for="m in goodMessages" :key="m.msg" class="msgi-item msgi-good">
              <code class="msgi-msg">{{ m.msg }}</code>
              <span class="msgi-why">{{ m.why }}</span>
            </div>
          </div>
        </div>
      </div>

      <!-- 5 Reglas de Oro -->
      <div class="rules-grid q-mt-xl">
        <div v-for="rule in messageRules" :key="rule.num" class="rule-card"
          :style="{ '--rule-color': rule.color }">
          <div class="rc-num" :style="{ background: rule.color }">{{ rule.num }}</div>
          <div class="rc-body">
            <div class="rc-title">{{ rule.title }}</div>
            <div v-for="ex in rule.examples" :key="ex.text" class="rc-example">
              <q-icon :name="ex.ok ? 'check' : 'close'" size="14px"
                :style="{ color: ex.ok ? '#4ade80' : '#f87171' }" />
              <code>{{ ex.text }}</code>
            </div>
            <p v-if="rule.note" class="rc-note">{{ rule.note }}</p>
          </div>
        </div>
      </div>

      <!-- Multi-line commit -->
      <div class="multiline-box q-mt-xl">
        <div class="mlb-title">Commits multilínea — para cambios complejos</div>
        <div class="mlb-body">
          <div class="mlb-col">
            <div class="mlb-subtitle">Formato completo</div>
            <CodeBlock :hide-header="true" lang="bash" :content="multilineCode" :copyable="true" />
          </div>
          <div class="mlb-col">
            <div class="mlb-subtitle">Estructura</div>
            <div class="mlb-structure">
              <div v-for="part in multilineParts" :key="part.label" class="mlbs-row"
                :style="{ '--mlbs-color': part.color }">
                <div class="mlbs-badge">{{ part.label }}</div>
                <div class="mlbs-desc">{{ part.desc }}</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 CONVENTIONAL COMMITS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        Conventional Commits — Estándar de la Industria
      </SectionTitle>

      <TextBlock>
        Prefijos estandarizados que categorizan el tipo de cambio. Permiten generar changelogs
        automáticos, versionar semánticamente (semver) y hacer búsquedas en el historial.
        Formato: <code>tipo(scope): descripción</code>
      </TextBlock>

      <div class="conv-grid q-mt-lg">
        <div v-for="ct in conventionalTypes" :key="ct.type" class="conv-card"
          :style="{ '--conv-color': ct.color }">
          <div class="convc-header">
            <q-icon :name="ct.icon" size="18px" :style="{ color: ct.color }" />
            <code class="convc-type">{{ ct.type }}:</code>
          </div>
          <div class="convc-desc">{{ ct.desc }}</div>
          <div class="convc-examples">
            <div v-for="ex in ct.examples" :key="ex" class="convce-item">{{ ex }}</div>
          </div>
        </div>
      </div>

      <!-- Scope + Breaking changes -->
      <div class="conv-extra q-mt-xl">
        <div class="cex-card" style="--cex-color:#fbbf24">
          <div class="cexc-title">
            <q-icon name="category" size="16px" color="warning" />
            scope — Alcance del cambio
          </div>
          <p class="cexc-desc">El scope (opcional) especifica qué parte del código afecta el cambio:</p>
          <CodeBlock :hide-header="true" lang="bash" :content="scopeCode" :copyable="true" />
        </div>
        <div class="cex-card" style="--cex-color:#f87171">
          <div class="cexc-title">
            <q-icon name="priority_high" size="16px" color="negative" />
            BREAKING CHANGE — Cambios incompatibles
          </div>
          <p class="cexc-desc">Cuando el cambio rompe compatibilidad con versiones anteriores:</p>
          <CodeBlock :hide-header="true" lang="bash" :content="breakingCode" :copyable="true" />
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 COMMITS ATÓMICOS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Commits Atómicos — Un Cambio, Un Commit
      </SectionTitle>

      <TextBlock>
        Un commit atómico hace <em>exactamente una cosa lógica</em>. Si mezclas cambios no
        relacionados en un commit, pierdes la capacidad de revertir selectivamente — el superpoder
        más valioso de Git:
      </TextBlock>

      <div class="atomic-visual q-mt-lg">
        <div class="av-side av-bad">
          <div class="avs-label avs-bad">
            <q-icon name="cancel" size="16px" style="color:#f87171" />
            Commit Monolítico
          </div>
          <div class="av-commit av-commit-bad">
            <div class="avc-hash">a1b2c3d</div>
            <div class="avc-msg">"fix stuff"</div>
            <div class="avc-changes">
              <div v-for="ch in monoChanges" :key="ch" class="avcc-item">{{ ch }}</div>
            </div>
          </div>
          <div class="avs-note avs-note-bad">
            Si la UI rompe algo, no puedes revertir solo eso sin perder el fix del motor
          </div>
        </div>

        <div class="av-arrow">
          <div class="ava-label">Divide en atómicos</div>
          <q-icon name="arrow_forward" size="28px" style="color:var(--text-muted)" />
        </div>

        <div class="av-side av-good">
          <div class="avs-label avs-good">
            <q-icon name="check_circle" size="16px" style="color:#4ade80" />
            Commits Atómicos
          </div>
          <div v-for="ac in atomicCommits" :key="ac.hash" class="av-commit av-commit-good"
            :style="{ '--avc-color': ac.color }">
            <div class="avc-hash" :style="{ color: ac.color }">{{ ac.hash }}</div>
            <div class="avc-msg">{{ ac.msg }}</div>
          </div>
          <div class="avs-note avs-note-good">
            Cada cambio es independiente — <code>git revert</code> funciona quirúrgicamente
          </div>
        </div>
      </div>

      <div class="atomic-rule q-mt-lg">
        <q-icon name="lightbulb" size="18px" color="warning" />
        <strong>Regla atómica:</strong>
        Si no puedes describir el commit en una línea sin usar la palabra "y",
        probablemente debería ser dos commits.
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         06 MODIFICAR Y DESHACER
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Modificar y Deshacer — Control Total
      </SectionTitle>

      <TextBlock>
        Git tiene múltiples formas de deshacer cambios. Cada una es para un escenario distinto —
        elegir la equivocada puede costar trabajo. Aprende la diferencia:
      </TextBlock>

      <div class="undo-grid q-mt-lg">
        <div v-for="cmd in undoCmds" :key="cmd.name" class="undo-card"
          :style="{ '--undo-color': cmd.color }">
          <div class="undoc-header">
            <q-icon :name="cmd.icon" size="20px" :style="{ color: cmd.color }" />
            <span class="undoc-name">{{ cmd.name }}</span>
            <span class="undoc-safe" :style="{ background: cmd.safe ? '#4ade8018' : '#f8717118', color: cmd.safe ? '#4ade80' : '#f87171', borderColor: cmd.safe ? '#4ade8030' : '#f8717130' }">
              {{ cmd.safe ? '✓ seguro' : '⚠ destructivo' }}
            </span>
          </div>
          <p class="undoc-desc">{{ cmd.desc }}</p>
          <CodeBlock :hide-header="true" lang="bash" :content="cmd.code" :copyable="true" />
          <div v-if="cmd.note" class="undoc-note">{{ cmd.note }}</div>
        </div>
      </div>

      <!-- git reset modes comparison -->
      <div class="reset-table q-mt-xl">
        <div class="rt-title">
          <q-icon name="compare" size="16px" color="info" />
          Comparación: modos de git reset
        </div>
        <div class="rt-body">
          <div class="rt-row rt-header">
            <div class="rt-cell">Modo</div>
            <div class="rt-cell">Commit eliminado</div>
            <div class="rt-cell">Staging</div>
            <div class="rt-cell">Working Dir</div>
            <div class="rt-cell">Cuándo usar</div>
          </div>
          <div v-for="mode in resetModes" :key="mode.flag" class="rt-row"
            :style="{ '--rt-color': mode.color }">
            <div class="rt-cell"><code>{{ mode.flag }}</code></div>
            <div class="rt-cell">{{ mode.commit }}</div>
            <div class="rt-cell">{{ mode.staging }}</div>
            <div class="rt-cell">{{ mode.working }}</div>
            <div class="rt-cell rt-when">{{ mode.when }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         07 GIT STASH
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">07</span>
        git stash — El Cajón de "Para Luego"
      </SectionTitle>

      <TextBlock>
        Necesitas cambiar de rama urgentemente pero tienes trabajo a medias y no quieres
        commitear algo incompleto. <code>git stash</code> guarda tus cambios temporalmente
        fuera del working directory y del historial:
      </TextBlock>

      <CodeBlock title="Flujo completo de git stash" lang="bash"
        :content="stashCode" :copyable="true" class="q-mt-lg" />

      <div class="stash-tips q-mt-lg">
        <div v-for="tip in stashTips" :key="tip.cmd" class="stip-item"
          :style="{ '--stip-color': tip.color }">
          <code class="stip-cmd">{{ tip.cmd }}</code>
          <span class="stip-desc">{{ tip.desc }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         08 NAVEGAR EL HISTORIAL
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">08</span>
        Navegar el Historial — Tu Máquina del Tiempo
      </SectionTitle>

      <TextBlock>
        <code>git log</code>, <code>git diff</code> y <code>git blame</code> son los comandos
        de inspección más usados. Entender el output te permite encontrar bugs introducidos
        en el pasado en minutos:
      </TextBlock>

      <!-- git log visual terminal -->
      <div class="log-terminal q-mt-lg">
        <div class="lt-topbar">
          <div class="lt-dot" style="background:#f87171"></div>
          <div class="lt-dot" style="background:#fbbf24"></div>
          <div class="lt-dot" style="background:#4ade80"></div>
          <code class="lt-cmd">git log --oneline --graph --all</code>
        </div>
        <div class="lt-body">
          <div v-for="line in logLines" :key="line.hash || line.graph" class="lt-line">
            <span class="lt-graph" :style="{ color: line.graphColor || 'var(--text-muted)' }">{{ line.graph }}</span>
            <span v-if="line.hash" class="lt-hash">{{ line.hash }}</span>
            <span v-if="line.refs" class="lt-refs">
              <span v-for="ref in line.refs" :key="ref.label" class="lt-ref"
                :style="{ background: ref.color + '18', color: ref.color, borderColor: ref.color + '30' }">{{ ref.label }}</span>
            </span>
            <span v-if="line.msg" class="lt-msg">{{ line.msg }}</span>
          </div>
        </div>
      </div>

      <div class="log-legend q-mt-md">
        <div v-for="l in logLegend" :key="l.label" class="ll-item">
          <span class="ll-badge" :style="{ background: l.color + '15', color: l.color, borderColor: l.color + '25' }">{{ l.label }}</span>
          <span class="ll-desc">{{ l.desc }}</span>
        </div>
      </div>

      <CodeBlock title="Comandos de inspección del historial" lang="bash"
        :content="inspectCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes al Commitear</SectionTitle>

      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item"
          :style="{ '--err-color': err.color }">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num" :style="{ background: err.color + '18', color: err.color }">{{ i + 1 }}</div>
              <div>
                <div class="err-type" :style="{ color: err.color }">{{ err.type }}</div>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'"
              size="20px" style="color:var(--text-muted); flex-shrink:0" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Situación:</strong> {{ err.cause }}
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="err.code" />
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive" />
              <strong>Solución:</strong> {{ err.fix }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Historial Limpio de ROS 2</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea un historial profesional para un paquete ROS 2</div>
            <div class="challenge-subtitle">
              10 commits atómicos siguiendo Conventional Commits, con staging selectivo
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

        <CodeBlock title="Commits objetivo a reproducir" lang="bash"
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
      <TextBlock>Commits y staging en profundidad:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Git Commits Best Practices" frameborder="0"
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
      <SectionTitle>Resumen — Comandos de Commits</SectionTitle>
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

    <!-- ══════════════════════════════════════════
         CTA FINAL
    ══════════════════════════════════════════ -->
    <div class="section-group q-mt-xl">
      <div class="final-cta">
        <div class="fca-icon">
          <q-icon name="history" size="40px" color="primary" />
        </div>
        <h2 class="fca-title">¡Commits dominados!</h2>
        <p class="fca-sub">
          Ahora sabes stagear con precisión, escribir mensajes profesionales y deshacer
          errores con seguridad. El siguiente paso: navegar y manipular el historial completo.
        </p>
        <div class="fca-actions">
          <q-btn color="primary" unelevated rounded size="lg" padding="14px 40px"
            to="/modulo-3/03historialPage"
            icon="arrow_forward" label="Historial y Navegación"
            class="text-weight-bold" />
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
import CodeBlock from 'components/content/CodeBlock.vue';

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════════════════

const patchCode = [
  '$ git add -p motor_controller.py',
  '',
  'diff --git a/motor_controller.py b/motor_controller.py',
  '@@ -45,6 +45,10 @@ class MotorController:',
  '+    def calibrate(self):',
  '+        """Calibrar el motor al inicio."""',
  '+        self.speed = 0',
  '',
  'Stage this hunk [y,n,q,a,d,s,?]? y   ← aceptar este bloque',
  '',
  '@@ -80,3 +84,7 @@ class MotorController:',
  '+    # TODO: fix overflow bug aquí',
  '+    if speed > MAX_SPEED:',
  '+        speed = MAX_SPEED',
  '',
  'Stage this hunk [y,n,q,a,d,s,?]? n   ← rechazar, va en otro commit',
].join('\n');

const multilineCode = [
  '# Abrir el editor configurado (nano/vim)',
  'git commit',
  '',
  '# O escribir todo en una línea',
  'git commit -m "feat(lidar): add distance filter" \\',
  '           -m "Filter rejects readings < 0.1m to avoid floor reflections." \\',
  '           -m "Closes #42"',
].join('\n');

const scopeCode = [
  '# Sin scope — afecta todo el proyecto',
  'feat: add obstacle avoidance system',
  '',
  '# Con scope — especifica qué subsistema',
  'feat(lidar): add distance filter for floor reflections',
  'fix(nav2): correct waypoint interpolation near walls',
  'docs(readme): add TurtleBot3 setup instructions',
  'test(motor): add unit tests for PID controller',
  'chore(deps): update nav2_msgs to 1.2.0',
].join('\n');

const breakingCode = [
  '# Opción 1: ! en el tipo (más visible)',
  'feat!: remove legacy motor API',
  '',
  '# Opción 2: footer BREAKING CHANGE',
  'feat(motor): replace speed() with set_velocity()',
  '',
  'BREAKING CHANGE: speed() ha sido eliminado.',
  'Usar set_velocity(linear, angular) en su lugar.',
  '',
  '# Nota: BREAKING CHANGE incrementa MAJOR en semver',
  '# 1.2.0 → 2.0.0',
].join('\n');

const stashCode = [
  '# Situación: estás a medias en feature/nav2-tuning',
  '# Llega urgencia: debes arreglar un bug en main',
  '',
  '# 1. Guardar trabajo actual',
  'git stash push -m "WIP: nav2 velocity tuning"',
  '',
  '# 2. Cambiar de rama',
  'git checkout main',
  '',
  '# 3. Arreglar el bug y commitear',
  'git commit -m "fix(motor): correct PID gains for 12V supply"',
  '',
  '# 4. Volver a tu feature',
  'git checkout feature/nav2-tuning',
  '',
  '# 5. Recuperar trabajo guardado',
  'git stash pop',
  '',
  '# Ver todos los stashes guardados',
  'git stash list',
  '# stash@{0}: On feature/nav2-tuning: WIP: nav2 velocity tuning',
  '',
  '# Recuperar un stash específico',
  'git stash apply stash@{0}',
].join('\n');

const inspectCode = [
  '# Historial compacto con gráfico',
  'git log --oneline --graph --all',
  '',
  '# Ver cambios de un archivo específico',
  'git log --oneline --follow src/motor_controller.py',
  '',
  '# Buscar en mensajes de commit',
  'git log --oneline --grep="lidar"',
  '',
  '# Ver qué cambios introdujo un commit específico',
  'git show a1b2c3d',
  '',
  '# Diferencia entre commits',
  'git diff a1b2c3d 4e5f6g7',
  '',
  '# Diferencia con main',
  'git diff main..feature/lidar',
  '',
  '# Ver quién escribió cada línea (git blame)',
  'git blame src/motor_controller.py',
  '# a1b2c3d4 (Alexander 2025-01-11) def calibrate(self):',
  '',
  '# Encontrar cuándo se introdujo un bug (bisect)',
  'git bisect start',
  'git bisect bad           # commit actual tiene el bug',
  'git bisect good v1.0.0   # esta versión funcionaba bien',
  '# Git hace búsqueda binaria automáticamente',
].join('\n');

const challengeCode = [
  '# git log --oneline objetivo al final del reto:',
  '',
  '# a1b2c3d (HEAD -> main) docs: add README with build instructions',
  '# 4e5f6g7 test(motor): add unit tests for velocity limits',
  '# 8h9i0j1 fix(motor): correct overflow at max_vel_x > 0.5',
  '# 2k3l4m5 feat(motor): add emergency stop service',
  '# 6n7o8p9 feat(lidar): add min_distance filter (floor reflections)',
  '# 1q2r3s4 feat(lidar): create LaserScan subscriber node',
  '# 5t6u7v8 chore: add nav2_params.yaml with TurtleBot3 defaults',
  '# 9w0x1y2 chore: configure package.xml with correct deps',
  '# 3z4a5b6 chore: create ROS 2 Python package mi_robot',
  '# 7c8d9e0 chore: add .gitignore for ROS 2 workspace',
].join('\n');

// ═══════════════════════════════════════════════════════════════
// DATA ARRAYS
// ═══════════════════════════════════════════════════════════════

const facts = [
  { icon: '🔍', label: 'git add -p — elige qué líneas van en el commit, no solo qué archivos' },
  { icon: '📝', label: 'Un buen mensaje de commit ahorra horas de debugging en el futuro' },
  { icon: '🔄', label: 'git stash: guarda trabajo sin commitear para cambiar de rama al instante' },
];

const ritualSteps = [
  {
    title: 'Check — Verificar estado',
    icon: 'search',
    color: '#60a5fa',
    desc: '¿Qué cambié? ¿Qué está pendiente? Siempre empieza por aquí.',
    code: [
      'git status',
      '',
      '# Output esperado:',
      '# On branch feature/add-lidar',
      '# Changes not staged for commit:',
      '#   modified:   src/motor_driver.py',
      '#   modified:   config/params.yaml',
      '# Untracked files:',
      '#   src/lidar_node.py',
    ].join('\n'),
  },
  {
    title: 'Stage — Preparar el commit',
    icon: 'add_box',
    color: '#fbbf24',
    desc: 'Elige EXACTAMENTE qué archivos (o partes de archivos) van en este commit.',
    code: [
      '# Agregar archivo específico',
      'git add src/lidar_node.py',
      '',
      '# Agregar directorio',
      'git add src/',
      '',
      '# Staging interactivo (hunk por hunk)',
      'git add -p src/motor_driver.py',
      '',
      '# Verificar lo que está staged',
      'git diff --staged',
    ].join('\n'),
  },
  {
    title: 'Commit — Confirmar y sellar',
    icon: 'save',
    color: '#4ade80',
    desc: 'El commit crea una fotografía inmutable. Escribe un mensaje que tenga sentido en 6 meses.',
    code: [
      '# Commit con mensaje corto',
      'git commit -m "feat(lidar): add LaserScan subscriber node"',
      '',
      '# Output confirmación:',
      '# [feature/add-lidar a1b2c3d] feat(lidar): add LaserScan subscriber node',
      '# 1 file changed, 45 insertions(+)',
    ].join('\n'),
  },
];

const stagingCmds = [
  { cmd: 'git add archivo.py',    color: '#4ade80', tag: 'recomendado', desc: 'Agrega un archivo específico. Control exacto sobre qué va en el commit.', warning: null },
  { cmd: 'git add src/',          color: '#60a5fa', tag: null,           desc: 'Agrega todo un directorio. Útil cuando todos los cambios en src/ son del mismo tema.', warning: null },
  { cmd: 'git add -p',            color: '#c084fc', tag: 'pro',          desc: 'Staging interactivo hunk por hunk. Perfecto cuando un archivo tiene cambios para distintos commits.', warning: null },
  { cmd: 'git add -u',            color: '#fbbf24', tag: null,           desc: 'Agrega todos los archivos ya rastreados que cambiaron. No incluye archivos nuevos.', warning: null },
  { cmd: 'git add .',             color: '#f87171', tag: '⚠ cuidado',    desc: 'Agrega TODO lo que cambió en el directorio actual, incluyendo archivos nuevos no deseados.', warning: 'Siempre revisa con git status antes. Puede incluir archivos que no deben ir.' },
  { cmd: 'git restore --staged f',color: '#94a3b8', tag: null,           desc: 'Quita un archivo del staging sin perder los cambios en working directory.', warning: null },
];

const patchKeys = [
  { key: 'y', color: '#4ade80', desc: 'Stage este hunk (incluir en el commit)' },
  { key: 'n', color: '#f87171', desc: 'No stage este hunk (omitir)' },
  { key: 's', color: '#fbbf24', desc: 'Split: dividir el hunk en partes más pequeñas' },
  { key: 'q', color: '#94a3b8', desc: 'Quit: salir sin stagear más hunks' },
  { key: '?', color: '#60a5fa', desc: 'Help: ver todos los comandos disponibles' },
];

const badMessages = [
  { msg: 'update',    problem: '¿Qué actualizaste? No dice nada' },
  { msg: 'fix stuff', problem: '¿Qué arreglaste? Igual de inútil' },
  { msg: 'WIP',       problem: 'Work In Progress no es un commit completo' },
  { msg: 'cambios',   problem: 'Obvio — todos los commits tienen cambios' },
  { msg: 'asdfgh',    problem: 'Ni siquiera intentaste' },
];

const goodMessages = [
  { msg: 'fix(motor): correct speed overflow at max_vel_x > 0.5',       why: 'Dice qué, dónde y condición exacta' },
  { msg: 'feat(lidar): add min_distance filter for floor reflections',    why: 'Describe la feature y su propósito' },
  { msg: 'refactor(nav2): extract PID gains to config file',             why: 'Explica el cambio y la motivación' },
  { msg: 'docs: add TurtleBot3 Jazzy installation guide to README',      why: 'Qué documentación y para quién' },
];

const messageRules = [
  {
    num: 1,
    title: 'Modo imperativo',
    color: '#4ade80',
    examples: [
      { ok: true,  text: 'Add LIDAR driver' },
      { ok: false, text: 'Added LIDAR driver' },
      { ok: false, text: 'Adding LIDAR driver' },
    ],
    note: 'Lee el mensaje como "Este commit va a [mensaje]"',
  },
  {
    num: 2,
    title: 'Máximo 50 caracteres',
    color: '#60a5fa',
    examples: [
      { ok: true,  text: 'feat: add obstacle avoidance' },
      { ok: false, text: 'feat: add obstacle avoidance system with path replanning and safety margins...' },
    ],
    note: 'Si necesitas más detalle, usa el cuerpo del commit (línea en blanco, luego el cuerpo).',
  },
  {
    num: 3,
    title: 'Sin punto al final',
    color: '#fbbf24',
    examples: [
      { ok: true,  text: 'Fix sensor timeout' },
      { ok: false, text: 'Fix sensor timeout.' },
    ],
    note: 'Es un título, no una oración completa.',
  },
  {
    num: 4,
    title: 'En inglés técnico',
    color: '#c084fc',
    examples: [
      { ok: true,  text: 'fix: correct PID gains for 12V motors' },
      { ok: false, text: 'fix: corrección del PID para motores' },
    ],
    note: 'Inglés = más búsquedas, más comprensión, más estándar en ROS 2.',
  },
  {
    num: 5,
    title: 'QUÉ y POR QUÉ, no CÓMO',
    color: '#f87171',
    examples: [
      { ok: true,  text: 'fix: prevent crash when LIDAR returns empty scan' },
      { ok: false, text: 'fix: add if len(msg.ranges) > 0 check in callback' },
    ],
    note: 'El código ya explica cómo. El mensaje explica la intención.',
  },
];

const multilineParts = [
  { label: 'Subject',        color: '#4ade80', desc: 'Primera línea: tipo(scope): descripción (≤50 chars)' },
  { label: 'Blank line',     color: '#94a3b8', desc: 'Línea en blanco obligatoria — separa subject del body' },
  { label: 'Body',           color: '#60a5fa', desc: 'Contexto adicional, motivación, trade-offs. Máx 72 chars/línea' },
  { label: 'Footer',         color: '#c084fc', desc: 'Refs a issues (Closes #42), BREAKING CHANGE, co-autores' },
];

const conventionalTypes = [
  { type: 'feat',     color: '#4ade80', icon: 'new_releases', desc: 'Nueva funcionalidad para el usuario', examples: ['feat(lidar): add distance filter', 'feat(nav2): implement waypoint replanning'] },
  { type: 'fix',      color: '#f87171', icon: 'bug_report',   desc: 'Corrección de bug que afecta al usuario', examples: ['fix(motor): correct speed overflow', 'fix(sensor): handle empty LIDAR scan'] },
  { type: 'docs',     color: '#60a5fa', icon: 'description',  desc: 'Solo cambios en documentación', examples: ['docs: add Nav2 setup guide', 'docs(readme): update TurtleBot3 steps'] },
  { type: 'refactor', color: '#c084fc', icon: 'build',        desc: 'Restructuración sin cambio funcional', examples: ['refactor(controller): extract PID to class', 'refactor: simplify scan callback logic'] },
  { type: 'test',     color: '#22d3ee', icon: 'science',      desc: 'Agregar o corregir tests', examples: ['test(motor): add velocity limit tests', 'test: fix failing integration test'] },
  { type: 'chore',    color: '#94a3b8', icon: 'settings',     desc: 'Build, deps, herramientas (sin código de usuario)', examples: ['chore: update nav2_msgs dep', 'chore: add .gitignore for ROS 2'] },
  { type: 'perf',     color: '#f97316', icon: 'speed',        desc: 'Mejoras de rendimiento medibles', examples: ['perf(lidar): cache filtered scan results', 'perf: reduce path planning frequency'] },
  { type: 'style',    color: '#a78bfa', icon: 'palette',      desc: 'Formato, espacios, puntos y comas — sin lógica', examples: ['style: fix indentation in controller.py', 'style: remove trailing whitespace'] },
];

const monoChanges = [
  '🛠️ Fix motor speed overflow bug',
  '🎨 Update dashboard UI colors',
  '📄 Add Nav2 documentation',
  '🔧 Refactor config loading',
];

const atomicCommits = [
  { hash: 'a1b2c3d', msg: 'fix(motor): correct speed overflow bug', color: '#f87171' },
  { hash: '4e5f6g7', msg: 'style(dashboard): update UI color scheme', color: '#a78bfa' },
  { hash: '8h9i0j1', msg: 'docs(nav2): add parameter reference guide', color: '#60a5fa' },
  { hash: '2k3l4m5', msg: 'refactor: simplify config file loading', color: '#c084fc' },
];

const undoCmds = [
  {
    name: 'git commit --amend',
    icon: 'edit',
    color: '#60a5fa',
    safe: true,
    desc: 'Modifica el último commit: cambia el mensaje o agrega archivos olvidados. SOLO antes de push.',
    note: '⚠️ Después de push, --amend reescribe el historial — usa git revert en cambio.',
    code: [
      '# Olvidaste agregar un archivo al commit',
      'git add forgotten_sensor.py',
      'git commit --amend --no-edit  # mismo mensaje',
      '',
      '# Cambiar solo el mensaje',
      'git commit --amend -m "feat(lidar): add obstacle detection with distance filter"',
    ].join('\n'),
  },
  {
    name: 'git reset HEAD~1',
    icon: 'undo',
    color: '#fbbf24',
    safe: true,
    desc: 'Deshace el último commit pero mantiene los cambios. --mixed (default) los devuelve al working directory.',
    note: 'Solo para commits NO pusheados. No pierde trabajo.',
    code: [
      '# Deshacer commit, mantener cambios sin stagear (default)',
      'git reset HEAD~1',
      '',
      '# Deshacer commit, mantener cambios STAGED',
      'git reset --soft HEAD~1',
      '',
      '# Deshacer commit y BORRAR cambios (¡destructivo!)',
      'git reset --hard HEAD~1',
    ].join('\n'),
  },
  {
    name: 'git revert',
    icon: 'history',
    color: '#4ade80',
    safe: true,
    desc: 'Crea un NUEVO commit que invierte los cambios de un commit anterior. Seguro en cualquier situación.',
    note: '✅ La forma correcta de deshacer commits ya pusheados. No reescribe historial.',
    code: [
      '# Revertir el último commit',
      'git revert HEAD',
      '',
      '# Revertir un commit específico',
      'git revert a1b2c3d',
      '',
      '# Revertir sin hacer commit automático',
      'git revert --no-commit a1b2c3d',
      'git commit -m "revert: undo lidar filter (caused false positives)"',
    ].join('\n'),
  },
  {
    name: 'git restore',
    icon: 'restore',
    color: '#c084fc',
    safe: true,
    desc: 'Descarta cambios en working directory (antes de git add). No afecta commits.',
    note: '⚠️ Los cambios descartados NO van a ningún lado — se pierden permanentemente.',
    code: [
      '# Descartar cambios en un archivo (working directory)',
      'git restore motor_controller.py',
      '',
      '# Descartar todos los cambios no staged',
      'git restore .',
      '',
      '# Sacar un archivo del staging (volver a working)',
      'git restore --staged motor_controller.py',
    ].join('\n'),
  },
];

const resetModes = [
  { flag: '--soft',  color: '#4ade80', commit: '❌ deshecho', staging: '✅ intacto',  working: '✅ intacto', when: 'Reescribir el mensaje del commit' },
  { flag: '--mixed', color: '#fbbf24', commit: '❌ deshecho', staging: '❌ limpiado', working: '✅ intacto', when: 'Dividir un commit en varios (default)' },
  { flag: '--hard',  color: '#f87171', commit: '❌ deshecho', staging: '❌ limpiado', working: '❌ borrado', when: 'Descartar cambios malos completamente' },
];

const stashTips = [
  { cmd: 'git stash list',         color: '#4ade80', desc: 'Ver todos los stashes guardados' },
  { cmd: 'git stash pop',          color: '#60a5fa', desc: 'Aplicar el último stash y eliminarlo de la lista' },
  { cmd: 'git stash apply stash@{1}', color: '#fbbf24', desc: 'Aplicar stash específico sin eliminarlo' },
  { cmd: 'git stash drop stash@{0}', color: '#f87171', desc: 'Eliminar un stash sin aplicarlo' },
  { cmd: 'git stash branch feat/x', color: '#c084fc', desc: 'Crear rama nueva desde el stash' },
];

const logLines = [
  { graph: '*',  graphColor: '#4ade80', hash: 'a1b2c3d', refs: [{ label: 'HEAD → main', color: '#fbbf24' }], msg: 'feat: add obstacle avoidance' },
  { graph: '*',  graphColor: '#4ade80', hash: '4e5f6g7', refs: [{ label: 'origin/main', color: '#60a5fa' }], msg: 'fix(motor): correct speed calibration' },
  { graph: '|\\', graphColor: '#94a3b8', hash: '', refs: [], msg: '' },
  { graph: '| *', graphColor: '#c084fc', hash: '8h9i0j1', refs: [{ label: 'feature/lidar', color: '#c084fc' }], msg: 'feat(lidar): add distance filter' },
  { graph: '| *', graphColor: '#c084fc', hash: '2k3l4m5', refs: [], msg: 'feat(lidar): create subscriber node' },
  { graph: '|/', graphColor: '#94a3b8', hash: '', refs: [], msg: '' },
  { graph: '*',  graphColor: '#4ade80', hash: '6n7o8p9', refs: [{ label: 'tag: v1.0.0', color: '#f97316' }], msg: 'chore: initial ROS 2 workspace setup' },
];

const logLegend = [
  { label: 'HEAD → main',     color: '#fbbf24', desc: 'Dónde estás parado y la rama actual' },
  { label: 'origin/main',     color: '#60a5fa', desc: 'Última versión subida a GitHub' },
  { label: 'feature/lidar',   color: '#c084fc', desc: 'Rama de feature (apunta al último commit de esa rama)' },
  { label: 'tag: v1.0.0',     color: '#f97316', desc: 'Tag de versión — marca un release' },
];

const commonErrors = reactive([
  {
    type: 'git add . antes de revisar — commiteaste basura',
    summary: 'Archivos de build, logs, credenciales o archivos temporales entraron al commit',
    color: '#f87171',
    cause: 'Hiciste git add . sin verificar con git status y git diff --staged qué estabas agregando',
    code: [
      '# ❌ El commit incluye build/ y *.pyc',
      '# git log --stat muestra:',
      '# a1b2c3d feat: add lidar driver',
      '#  build/lidar_node/lidar_node  | Bin 0 -> 156323 bytes',
      '#  src/lidar_node.pyc           | Bin 0 -> 4521 bytes',
      '',
      '# ✅ Solución si NO hiciste push aún',
      '# 1. Deshacer el commit',
      'git reset HEAD~1',
      '',
      '# 2. Agregar .gitignore PRIMERO',
      'echo "build/" >> .gitignore',
      'echo "*.pyc" >> .gitignore',
      'git add .gitignore',
      'git commit -m "chore: add .gitignore for ROS 2 build artifacts"',
      '',
      '# 3. Ahora sí agregar los archivos correctos',
      'git add src/lidar_node.py',
      'git commit -m "feat(lidar): add subscriber node"',
    ].join('\n'),
    fix: 'Siempre git status + git diff --staged antes de git commit. Crea .gitignore antes del primer commit.',
    open: false,
  },
  {
    type: '--amend después de push → historial divergente',
    summary: 'Modificaste un commit ya pusheado y ahora git push falla con "non-fast-forward"',
    color: '#fbbf24',
    cause: 'git commit --amend reescribe el commit con un hash nuevo. Si ya está en GitHub, las versiones son incompatibles.',
    code: [
      '# ❌ Situación: hiciste --amend de un commit ya pusheado',
      '# git push falla:',
      '# ! [rejected] main → main (non-fast-forward)',
      '# hint: Updates were rejected because the tip of your current branch is behind',
      '',
      '# ✅ La solución correcta: git revert (no reescribe historial)',
      'git revert HEAD',
      '# Esto crea un nuevo commit que deshace el anterior',
      'git push   # funciona normalmente',
      '',
      '# ❌ NUNCA hacer esto en ramas compartidas:',
      '# git push --force  ← destruye el historial de tus compañeros',
    ].join('\n'),
    fix: 'Después de push, usa git revert para deshacer. Guarda --amend solo para commits locales que aún no subiste.',
    open: false,
  },
  {
    type: 'git reset --hard — perdiste trabajo',
    summary: 'Pensabas que solo deshacías el commit, pero --hard también borró tus cambios en disco',
    color: '#c084fc',
    cause: 'git reset --hard borra los cambios del working directory y del staging. No hay papelera.',
    code: [
      '# ❌ Ejecutaste reset --hard por error',
      '# git reset --hard HEAD~1',
      '# HEAD is now at 9f8e7d6 Previous commit',
      '# ... tu código desapareció',
      '',
      '# ✅ Opción 1: el commit a veces es recuperable vía reflog',
      'git reflog',
      '# a1b2c3d HEAD@{1}: commit: feat(lidar): add filter',
      '',
      '# Restaurar desde reflog',
      'git checkout a1b2c3d',
      'git checkout -b recovery/lidar-filter',
      '',
      '# ✅ Opción 2: si nunca hiciste commit, no hay recuperación',
      '# Por eso: commitea frecuente aunque sea con WIP',
    ].join('\n'),
    fix: 'Usa git reset --mixed (default) que mantiene los cambios. El --hard solo cuando estás 100% seguro. En duda, revisa git reflog para recuperar commits.',
    open: false,
  },
  {
    type: 'Commit monolítico — 50 archivos en un commit',
    summary: 'Trabajaste días sin commitear y ahora todo está mezclado en un solo commit gigante',
    color: '#f97316',
    cause: 'git diff muestra cambios de features distintas, bugs distintos y docs todo mezclado — imposible revertir selectivamente',
    code: [
      '# ❌ Situación: git status muestra 20 archivos modificados',
      '# de temas completamente distintos',
      '',
      '# ✅ Separar con staging selectivo',
      '# Commit 1: solo el fix del motor',
      'git add src/motor_controller.py',
      'git add config/motor_params.yaml',
      'git commit -m "fix(motor): correct PID gains for 12V supply"',
      '',
      '# Commit 2: solo la nueva feature de LIDAR',
      'git add src/lidar_node.py',
      'git add src/lidar_filter.py',
      'git commit -m "feat(lidar): add obstacle detection with distance filter"',
      '',
      '# Commit 3: documentación',
      'git add docs/',
      'git commit -m "docs: add Nav2 parameter reference guide"',
    ].join('\n'),
    fix: 'Usa git add -p para staging interactivo y separa lógicamente. En el futuro, commitea cada vez que completes una unidad lógica — no esperes días.',
    open: false,
  },
  {
    type: 'Commiteaste directamente en main',
    summary: 'Olvidaste crear una feature branch y commiteaste directo en main',
    color: '#60a5fa',
    cause: 'git log --oneline muestra el nuevo commit en main — debería estar en una feature branch',
    code: [
      '# ❌ Situación: git log --oneline',
      '# a1b2c3d (HEAD -> main) feat: add camera driver  ← debería ser en feature/camera',
      '# 9f8e7d6 fix: motor calibration',
      '',
      '# ✅ Solución: mover el commit a una rama nueva',
      '# 1. Crear rama donde está el commit',
      'git branch feature/camera',
      '',
      '# 2. Regresar main al commit anterior',
      'git reset HEAD~1',
      '# ← main vuelve a 9f8e7d6',
      '',
      '# 3. El commit ahora solo existe en feature/camera',
      'git checkout feature/camera',
      '# git log --oneline: a1b2c3d feat: add camera driver ✓',
    ].join('\n'),
    fix: 'git branch para crear rama con el commit actual, git reset HEAD~1 para quitarlo de main. En el futuro: git checkout -b feature/x antes de empezar a trabajar.',
    open: false,
  },
]);

const challengeSteps = [
  { num: 1, color: '#4ade80', text: 'Crea un paquete ROS 2 vacío: ros2 pkg create --build-type ament_python mi_robot' },
  { num: 2, color: '#60a5fa', text: 'Primer commit: "chore: add .gitignore for ROS 2 workspace" (solo el .gitignore)' },
  { num: 3, color: '#fbbf24', text: 'Siguiente commit: "chore: create ROS 2 Python package mi_robot"' },
  { num: 4, color: '#c084fc', text: 'Crea src/lidar_node.py con un subscriber básico y commitea: "feat(lidar): create LaserScan subscriber node"' },
  { num: 5, color: '#f87171', text: 'Agrega filtro de distancia a lidar_node.py (misma clase) y commitea con git add -p para separar en 2 commits' },
  { num: 6, color: '#f97316', text: 'Completa hasta llegar a 10 commits como el historial objetivo — verificar con git log --oneline' },
];

const challengeHints = [
  'git log --oneline al final debe ser idéntico al objetivo mostrado arriba (mismo formato de mensajes)',
  'Para los 2 commits de lidar: primero commitea el subscriber, después usa git add -p para stagear solo las líneas del filtro',
  'git diff --staged antes de cada commit para verificar que solo está lo correcto',
  'El scope entre paréntesis es opcional pero añade mucho contexto: feat(lidar): vs feat:',
  'Para el commit de "emergency stop": crea una función separate y un servicio ROS 2 en un archivo nuevo',
];

const summaryItems = [
  { cmd: 'git add -p',           desc: 'Staging interactivo hunk por hunk',      example: 'y/n/s para cada bloque', color: '#4ade80' },
  { cmd: 'git diff --staged',    desc: 'Ver qué está en staging antes de commit', example: 'siempre antes de commit', color: '#60a5fa' },
  { cmd: 'git commit -m',        desc: 'Crear commit con mensaje',                example: '"feat(x): descripción"', color: '#fbbf24' },
  { cmd: 'git commit --amend',   desc: 'Modificar último commit',                 example: 'solo antes de push',     color: '#c084fc' },
  { cmd: 'git revert HEAD',      desc: 'Deshacer commit creando uno nuevo',       example: 'seguro después de push', color: '#4ade80' },
  { cmd: 'git reset --soft',     desc: 'Deshacer commit, mantener staged',        example: 'reescribir mensaje',     color: '#60a5fa' },
  { cmd: 'git stash',            desc: 'Guardar trabajo sin commitear',           example: 'cambiar de rama',        color: '#fbbf24' },
  { cmd: 'git log --oneline',    desc: 'Ver historial compacto',                  example: '--graph --all',          color: '#f87171' },
  { cmd: 'git blame',            desc: 'Ver quién escribió cada línea',           example: 'src/motor.py',           color: '#94a3b8' },
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
   RITUAL STEPS
══════════════════════════════════════════ */
.ritual-steps { display: flex; flex-direction: column; gap: 0; }
.rst-step {
  display: grid; grid-template-columns: 54px 1fr; gap: 16px; align-items: start;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; padding: 1.25rem 1.5rem; min-width: 0;
}
.rst-step + .rst-step { margin-top: 8px; }
.rst-icon {
  width: 48px; height: 48px; border-radius: 14px; flex-shrink: 0;
  background: var(--rst-color); display: flex; align-items: center; justify-content: center;
}
.rst-num  { font-size: .92rem; font-weight: 800; margin-bottom: 4px; }
.rst-desc { font-size: .84rem; color: var(--text-muted); margin-bottom: 10px; }
.rst-arrow { grid-column: 1 / -1; display: flex; justify-content: flex-start; padding-left: 70px; margin: 4px 0; }
.rst-content { min-width: 0; }

/* ══════════════════════════════════════════
   STAGING GRID
══════════════════════════════════════════ */
.staging-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
.stg-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--stg-color); border-radius: 12px;
  padding: 12px 14px; display: flex; flex-direction: column; gap: 6px; min-width: 0;
}
.stgc-header { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.stgc-cmd { font-family: 'Fira Code', monospace; font-size: .84rem; font-weight: 700; color: var(--stg-color); background: none; padding: 0; word-break: break-all; }
.stgc-tag { font-size: .68rem; font-weight: 800; padding: 2px 7px; border-radius: 999px; border: 1px solid; }
.stgc-desc { font-size: .78rem; color: var(--text-secondary); line-height: 1.4; flex: 1; }
.stgc-warn { font-size: .75rem; color: #fbbf24; display: flex; align-items: flex-start; gap: 5px; line-height: 1.4; }
.patch-detail { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.pd-title { font-size: .92rem; font-weight: 700; color: var(--text-primary); margin-bottom: 8px; }
.pd-title code { background: none; padding: 0; }
.patch-keys { display: flex; flex-wrap: wrap; gap: 8px; }
.pk-item { display: flex; align-items: center; gap: 8px; background: var(--bg-surface-hover); border-radius: 8px; padding: 6px 12px; }
.pk-key  { font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 900; color: var(--pk-color); background: none; padding: 0; }
.pk-desc { font-size: .8rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   MESSAGE COMPARISON
══════════════════════════════════════════ */
.msg-comparison { display: grid; grid-template-columns: 1fr auto 1fr; gap: 16px; align-items: start; }
.msgc-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; padding: 1.25rem; display: flex; flex-direction: column; gap: 10px;
}
.msgc-bad  { border-top: 3px solid #f87171; }
.msgc-good { border-top: 3px solid #4ade80; }
.msgc-header { display: flex; align-items: center; gap: 8px; }
.msgc-title  { font-size: .88rem; font-weight: 700; }
.msgc-items  { display: flex; flex-direction: column; gap: 8px; }
.msgi-item { background: var(--bg-surface-hover); border-radius: 8px; padding: 8px 12px; display: flex; flex-direction: column; gap: 3px; }
.msgi-msg    { font-family: 'Fira Code', monospace; font-size: .82rem; color: var(--text-secondary); background: none; padding: 0; }
.msgi-problem { font-size: .76rem; color: #f87171; font-style: italic; }
.msgi-good .msgi-msg { color: var(--text-primary); }
.msgi-why    { font-size: .76rem; color: #4ade80; font-style: italic; }
.msgc-divider { display: flex; align-items: center; justify-content: center; padding-top: 2rem; }

/* Rules grid */
.rules-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 10px; }
.rule-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--rule-color); border-radius: 12px; padding: 12px 14px;
  display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.rc-num   { width: 28px; height: 28px; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-size: .88rem; font-weight: 800; color: #0d1117; flex-shrink: 0; }
.rc-title { font-size: .84rem; font-weight: 700; color: var(--text-primary); }
.rc-example { display: flex; align-items: baseline; gap: 5px; font-size: .78rem; }
.rc-example code { background: none; padding: 0; font-size: .77rem; color: var(--text-secondary); word-break: break-all; }
.rc-note  { font-size: .74rem; color: var(--text-muted); line-height: 1.4; margin: 2px 0 0; border-top: 1px solid var(--border-subtle); padding-top: 6px; }

/* Multi-line */
.multiline-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.25rem; }
.mlb-title    { font-size: .92rem; font-weight: 700; color: var(--text-primary); margin-bottom: 12px; }
.mlb-body     { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
.mlb-subtitle { font-size: .82rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 8px; }
.mlb-structure { display: flex; flex-direction: column; gap: 8px; }
.mlbs-row  { display: flex; align-items: flex-start; gap: 10px; }
.mlbs-badge { font-size: .72rem; font-weight: 800; color: var(--mlbs-color); background: color-mix(in srgb, var(--mlbs-color) 10%, var(--bg-surface)); border: 1px solid color-mix(in srgb, var(--mlbs-color) 25%, transparent); border-radius: 6px; padding: 3px 8px; white-space: nowrap; flex-shrink: 0; }
.mlbs-desc { font-size: .78rem; color: var(--text-secondary); line-height: 1.4; padding-top: 2px; }

/* ══════════════════════════════════════════
   CONVENTIONAL COMMITS
══════════════════════════════════════════ */
.conv-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; }
.conv-card {
  background: color-mix(in srgb, var(--conv-color) 6%, var(--bg-surface));
  border: 1px solid color-mix(in srgb, var(--conv-color) 20%, transparent);
  border-radius: 12px; padding: 12px 14px; display: flex; flex-direction: column; gap: 6px; min-width: 0;
}
.convc-header { display: flex; align-items: center; gap: 8px; }
.convc-type   { font-family: 'Fira Code', monospace; font-size: .95rem; font-weight: 900; color: var(--conv-color); background: none; padding: 0; }
.convc-desc   { font-size: .78rem; color: var(--text-secondary); }
.convc-examples { display: flex; flex-direction: column; gap: 3px; margin-top: 2px; border-top: 1px solid color-mix(in srgb, var(--conv-color) 15%, transparent); padding-top: 6px; }
.convce-item  { font-family: 'Fira Code', monospace; font-size: .7rem; color: var(--text-muted); }
.conv-extra   { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.cex-card     { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-top: 3px solid var(--cex-color, var(--border-subtle)); border-radius: 14px; padding: 1.25rem; min-width: 0; }
.cexc-title   { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700; color: var(--text-primary); margin-bottom: 8px; }
.cexc-desc    { font-size: .84rem; color: var(--text-secondary); margin: 0 0 12px; }

/* ══════════════════════════════════════════
   ATOMIC VISUAL
══════════════════════════════════════════ */
.atomic-visual { display: grid; grid-template-columns: 1fr auto 1fr; gap: 20px; align-items: center; }
.av-side { display: flex; flex-direction: column; gap: 8px; }
.avs-label { display: flex; align-items: center; gap: 7px; font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.avs-bad  { }
.avs-good { }
.av-commit { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 10px; padding: 10px 14px; min-width: 0; }
.av-commit-bad { border-left: 3px solid #f87171; }
.av-commit-good { border-left: 3px solid var(--avc-color, var(--border-subtle)); }
.avc-hash { font-family: 'Fira Code', monospace; font-size: .76rem; color: var(--avc-color, var(--text-muted)); margin-bottom: 3px; }
.av-commit-bad .avc-hash { color: #f87171; }
.avc-msg  { font-size: .82rem; color: var(--text-secondary); }
.avc-changes { display: flex; flex-direction: column; gap: 4px; margin-top: 8px; border-top: 1px solid var(--border-subtle); padding-top: 8px; }
.avcc-item { font-size: .78rem; color: var(--text-muted); padding: 3px 0; }
.avs-note { font-size: .78rem; font-style: italic; padding: 6px 10px; border-radius: 8px; }
.avs-note-bad  { color: #f87171; background: rgba(248,113,113,.08); }
.avs-note-good { color: #4ade80; background: rgba(74,222,128,.08); }
.avs-note-good code { background: none; padding: 0; color: #4ade80; font-size: .78rem; }
.av-arrow { display: flex; flex-direction: column; align-items: center; gap: 6px; }
.ava-label { font-size: .75rem; font-weight: 700; color: var(--text-muted); text-align: center; }
.atomic-rule {
  display: flex; align-items: flex-start; gap: 10px; font-size: .9rem; color: var(--text-secondary);
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid #fbbf24; border-radius: 12px; padding: 14px 16px; line-height: 1.5;
}

/* ══════════════════════════════════════════
   UNDO GRID
══════════════════════════════════════════ */
.undo-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.undo-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--undo-color); border-radius: 14px;
  display: flex; flex-direction: column; gap: 10px; padding: 1.25rem; min-width: 0;
}
.undoc-header { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.undoc-name   { font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 800; color: var(--undo-color); }
.undoc-safe   { font-size: .7rem; font-weight: 800; padding: 2px 8px; border-radius: 999px; border: 1px solid; margin-left: auto; white-space: nowrap; }
.undoc-desc   { font-size: .84rem; color: var(--text-secondary); margin: 0; line-height: 1.5; }
.undoc-note   { font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 8px; padding: 8px 12px; }

/* Reset table */
.reset-table { }
.rt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 10px; }
.rt-body  { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; overflow-x: auto; }
.rt-row   { display: grid; grid-template-columns: 1fr 1fr 1fr 1fr 1.5fr; border-bottom: 1px solid var(--border-subtle); border-left: 3px solid var(--rt-color, transparent); }
.rt-row:last-child { border-bottom: none; }
.rt-header { background: var(--bg-surface-solid); font-size: .8rem; font-weight: 700; color: var(--text-primary); border-left: none; }
.rt-cell  { padding: 9px 12px; font-size: .8rem; color: var(--text-secondary); display: flex; align-items: center; border-right: 1px solid var(--border-subtle); }
.rt-cell:last-child { border-right: none; }
.rt-cell code { background: none; padding: 0; font-size: .8rem; }
.rt-when  { font-size: .76rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   STASH
══════════════════════════════════════════ */
.stash-tips { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
.stip-item  { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--stip-color); border-radius: 10px; padding: 10px 12px; display: flex; flex-direction: column; gap: 4px; }
.stip-cmd   { font-family: 'Fira Code', monospace; font-size: .78rem; font-weight: 700; color: var(--stip-color); background: none; padding: 0; word-break: break-all; }
.stip-desc  { font-size: .76rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   LOG TERMINAL
══════════════════════════════════════════ */
.log-terminal { background: var(--bg-deep, #0d1117); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden; }
.lt-topbar { display: flex; align-items: center; gap: 8px; padding: 10px 16px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); }
.lt-dot    { width: 10px; height: 10px; border-radius: 50%; }
.lt-cmd    { font-family: 'Fira Code', monospace; font-size: .82rem; color: var(--text-muted); background: none; padding: 0; margin-left: 8px; }
.lt-body   { padding: 1.25rem; font-family: 'Fira Code', monospace; font-size: .85rem; display: flex; flex-direction: column; gap: 5px; }
.lt-line   { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.lt-graph  { font-weight: 700; min-width: 20px; }
.lt-hash   { color: #f97316; font-weight: 700; }
.lt-refs   { display: flex; gap: 5px; flex-wrap: wrap; }
.lt-ref    { font-size: .72rem; font-weight: 700; padding: 2px 7px; border-radius: 5px; border: 1px solid; }
.lt-msg    { color: var(--text-secondary, #94a3b8); }
.log-legend { display: flex; flex-wrap: wrap; gap: 10px; }
.ll-item   { display: flex; align-items: center; gap: 8px; font-size: .8rem; color: var(--text-muted); }
.ll-badge  { font-family: 'Fira Code', monospace; font-size: .72rem; font-weight: 700; padding: 2px 8px; border-radius: 5px; border: 1px solid; }

/* ══════════════════════════════════════════
   ERROR ACCORDION
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--err-color); border-radius: 12px; overflow: hidden; }
.err-header { display: flex; align-items: center; justify-content: space-between; padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s; }
.err-header:hover { background: var(--bg-surface-hover); }
.err-left   { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num    { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center; }
.err-type   { font-size: .82rem; font-weight: 700; color: var(--text-primary); margin-bottom: 2px; }
.err-summary{ font-size: .78rem; color: var(--text-muted); }
.err-body   { padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 10px; }
.err-cause  { font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }
.err-fix    { font-size: .85rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }

/* ══════════════════════════════════════════
   CHALLENGE
══════════════════════════════════════════ */
.challenge-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b; }
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon   { width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.challenge-title  { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge  { margin-left: auto; font-size: .72rem; font-weight: 800; padding: 4px 12px; border-radius: 999px; white-space: nowrap; background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3); }
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
.summary-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
.summary-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 4px solid var(--sc-color); border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s; }
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; word-break: break-all; }
.sc-desc    { font-size: .81rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   CTA FINAL
══════════════════════════════════════════ */
.final-cta {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 24px; padding: 3rem 2rem;
  display: flex; flex-direction: column; align-items: center; gap: 1rem; text-align: center;
}
.fca-icon  { width: 72px; height: 72px; background: rgba(96,165,250,.1); border-radius: 20px; display: flex; align-items: center; justify-content: center; }
.fca-title { font-size: 1.5rem; font-weight: 800; color: var(--text-primary); margin: 0; }
.fca-sub   { font-size: .95rem; color: var(--text-secondary); max-width: 520px; line-height: 1.6; margin: 0; }
.fca-actions { margin-top: .5rem; }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1100px) {
  .rules-grid  { grid-template-columns: repeat(3, 1fr); }
  .conv-grid   { grid-template-columns: repeat(4, 1fr); }
}
@media (max-width: 900px) {
  .staging-grid { grid-template-columns: repeat(2, 1fr); }
  .rules-grid   { grid-template-columns: repeat(2, 1fr); }
  .conv-grid    { grid-template-columns: repeat(2, 1fr); }
  .conv-extra   { grid-template-columns: 1fr; }
  .undo-grid    { grid-template-columns: 1fr; }
  .stash-tips   { grid-template-columns: repeat(2, 1fr); }
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
  .mlb-body     { grid-template-columns: 1fr; }
}
@media (max-width: 768px) {
  .msg-comparison  { grid-template-columns: 1fr; }
  .msgc-divider    { transform: rotate(90deg); }
  .atomic-visual   { grid-template-columns: 1fr; }
  .av-arrow        { transform: rotate(90deg); }
  .staging-grid    { grid-template-columns: 1fr; }
  .rules-grid      { grid-template-columns: 1fr; }
  .stash-tips      { grid-template-columns: 1fr; }
  .rt-row          { grid-template-columns: 1fr 1fr; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
}
@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
  .conv-grid    { grid-template-columns: 1fr; }
}
</style>
