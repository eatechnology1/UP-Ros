<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        El historial de Git es tu máquina del tiempo. Cada commit es un punto fijo en la evolución
        de tu proyecto — puedes volver a cualquiera, comparar dos versiones, o descubrir exactamente
        qué commit introdujo un bug. Dominar la navegación del historial convierte el debugging
        de <strong>horas</strong> en <strong>minutos</strong>.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div v-for="f in facts" :key="f.label" class="fact-pill">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 GIT LOG
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        git log — Tu Mapa del Historial
      </SectionTitle>

      <TextBlock>
        <code>git log</code> por defecto es difícil de leer. Con los flags correctos se convierte
        en un árbol visual del proyecto. Esta combinación es la que usan los profesionales
        a diario:
      </TextBlock>

      <div class="hero-cmd q-mt-lg">
        <div class="hcmd-label">El comando que necesitas memorizar</div>
        <CodeBlock :hide-header="true" lang="bash" :content="heroLogCode" :copyable="true" />
      </div>

      <!-- git log visual terminal -->
      <div class="log-terminal q-mt-xl">
        <div class="lt-topbar">
          <div class="lt-dot" style="background:#f87171"></div>
          <div class="lt-dot" style="background:#fbbf24"></div>
          <div class="lt-dot" style="background:#4ade80"></div>
          <code class="lt-cmd">git log --oneline --graph --all --decorate</code>
        </div>
        <div class="lt-body">
          <div v-for="line in logLines" :key="line.id" class="lt-line">
            <span class="lt-graph" :style="{ color: line.graphColor }">{{ line.graph }}</span>
            <span v-if="line.hash" class="lt-hash">{{ line.hash }}</span>
            <span v-if="line.refs && line.refs.length" class="lt-refs">
              <span v-for="ref in line.refs" :key="ref.label" class="lt-ref"
                :style="{ background: ref.color + '18', color: ref.color, borderColor: ref.color + '30' }">
                {{ ref.label }}
              </span>
            </span>
            <span v-if="line.msg" class="lt-msg">{{ line.msg }}</span>
          </div>
        </div>
      </div>

      <!-- log flag options -->
      <div class="flags-grid q-mt-xl">
        <div v-for="flag in logFlags" :key="flag.flag" class="flag-card"
          :style="{ '--flag-color': flag.color }">
          <div class="flagc-header">
            <q-icon :name="flag.icon" size="18px" :style="{ color: flag.color }" />
            <code class="flagc-flag">{{ flag.flag }}</code>
          </div>
          <div class="flagc-desc">{{ flag.desc }}</div>
          <div class="flagc-example">{{ flag.example }}</div>
        </div>
      </div>

      <CodeBlock title="Combinaciones avanzadas de git log" lang="bash"
        :content="logAdvancedCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         02 GIT DIFF
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        git diff — Ver Exactamente Qué Cambió
      </SectionTitle>

      <TextBlock>
        <code>git diff</code> muestra las diferencias línea por línea entre dos versiones.
        Es esencial antes de commitear — siempre deberías saber exactamente qué estás confirmando:
      </TextBlock>

      <div class="diff-cards q-mt-lg">
        <div v-for="dc in diffCmds" :key="dc.cmd" class="diffc-item"
          :style="{ '--dc-color': dc.color }">
          <div class="diffc-header">
            <q-icon :name="dc.icon" size="16px" :style="{ color: dc.color }" />
            <code class="diffc-cmd">{{ dc.cmd }}</code>
          </div>
          <div class="diffc-desc">{{ dc.desc }}</div>
        </div>
      </div>

      <!-- diff visual output -->
      <div class="diff-terminal q-mt-xl"
        :style="{ '--dt-add': '#4ade80', '--dt-rem': '#f87171', '--dt-hunk': '#60a5fa' }">
        <div class="dt-topbar">
          <div class="lt-dot" style="background:#f87171"></div>
          <div class="lt-dot" style="background:#fbbf24"></div>
          <div class="lt-dot" style="background:#4ade80"></div>
          <code class="lt-cmd">git diff src/motor_controller.py</code>
        </div>
        <div class="dt-body">
          <div v-for="line in diffLines" :key="line.id" class="dt-line"
            :class="{
              'dt-file':    line.type === 'file',
              'dt-meta':    line.type === 'meta',
              'dt-hunk':    line.type === 'hunk',
              'dt-context': line.type === 'context',
              'dt-removed': line.type === 'removed',
              'dt-added':   line.type === 'added',
            }">
            <span class="dt-prefix">{{ line.prefix }}</span>
            <span class="dt-content">{{ line.content }}</span>
          </div>
        </div>
      </div>

      <div class="diff-legend q-mt-md">
        <div class="dleg-item" :style="{ '--dl-color': '#f87171' }">
          <div class="dleg-dot"></div>
          <span>Líneas eliminadas (rojo)</span>
        </div>
        <div class="dleg-item" :style="{ '--dl-color': '#4ade80' }">
          <div class="dleg-dot"></div>
          <span>Líneas añadidas (verde)</span>
        </div>
        <div class="dleg-item" :style="{ '--dl-color': '#60a5fa' }">
          <div class="dleg-dot"></div>
          <span>Hunk header — contexto del cambio</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 GIT SHOW + BLAME
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        git show + git blame — Inspeccionar en Profundidad
      </SectionTitle>

      <TextBlock>
        <code>git show</code> revela el contenido completo de un commit. <code>git blame</code>
        muestra quién escribió cada línea del código — esencial para entender decisiones de diseño
        y para el code review:
      </TextBlock>

      <div class="inspect-grid q-mt-lg">
        <div v-for="cmd in inspectCmds" :key="cmd.name" class="inspectc-card"
          :style="{ '--ic-color': cmd.color }">
          <div class="icc-header">
            <q-icon :name="cmd.icon" size="20px" :style="{ color: cmd.color }" />
            <code class="icc-name">{{ cmd.name }}</code>
          </div>
          <p class="icc-desc">{{ cmd.desc }}</p>
          <CodeBlock :hide-header="true" lang="bash" :content="cmd.code" :copyable="true" />
        </div>
      </div>

      <!-- blame output visual -->
      <div class="blame-terminal q-mt-xl">
        <div class="lt-topbar">
          <div class="lt-dot" style="background:#f87171"></div>
          <div class="lt-dot" style="background:#fbbf24"></div>
          <div class="lt-dot" style="background:#4ade80"></div>
          <code class="lt-cmd">git blame src/motor_controller.py</code>
        </div>
        <div class="lt-body blame-body">
          <div v-for="bl in blameLines" :key="bl.hash" class="bl-row">
            <span class="bl-hash" :style="{ color: bl.color }">{{ bl.hash }}</span>
            <span class="bl-author">{{ bl.author }}</span>
            <span class="bl-date">{{ bl.date }}</span>
            <span class="bl-num">{{ bl.lineNum }}</span>
            <span class="bl-code">{{ bl.code }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 NAVEGACIÓN EN EL HISTORIAL
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        Viajar en el Tiempo — Navegar Commits
      </SectionTitle>

      <TextBlock>
        Puedes visitar cualquier commit del pasado. Git cambia todos los archivos del proyecto
        al estado exacto de ese momento. Útil para reproducir bugs o recuperar código antiguo:
      </TextBlock>

      <!-- Timeline visual (data-driven, no pseudo-elements) -->
      <div class="timeline-visual q-mt-lg">
        <div class="tv-label">Línea de tiempo del proyecto</div>
        <div class="tv-track">
          <div v-for="(point, idx) in timelinePoints" :key="point.hash" class="tv-item">
            <div class="tv-node" :style="{ '--tvn-color': point.color }"
              :class="{ 'tv-node-current': point.current }">
              <div class="tvn-dot"></div>
              <div class="tvn-info">
                <div class="tvn-hash">{{ point.hash }}</div>
                <div class="tvn-label">{{ point.label }}</div>
                <div class="tvn-date">{{ point.date }}</div>
              </div>
            </div>
            <div v-if="idx < timelinePoints.length - 1" class="tv-connector"></div>
          </div>
        </div>
      </div>

      <CodeBlock title="Comandos de navegación" lang="bash"
        :content="navCode" :copyable="true" class="q-mt-xl" />

      <div class="detached-head q-mt-lg">
        <div class="dh-header">
          <q-icon name="link_off" size="20px" style="color:#f87171" />
          <span class="dh-title">Detached HEAD — Estado de solo lectura</span>
        </div>
        <p class="dh-desc">
          Al hacer <code>git checkout &lt;hash&gt;</code> entras en "Detached HEAD":
          puedes ver, compilar y ejecutar el código, pero cualquier commit que hagas se
          <strong>perderá</strong> al volver a una rama.
        </p>
        <div class="dh-options">
          <div v-for="opt in detachedOptions" :key="opt.title" class="dho-item"
            :style="{ '--dho-color': opt.color }">
            <div class="dhoi-header">
              <q-icon :name="opt.icon" size="16px" :style="{ color: opt.color }" />
              {{ opt.title }}
            </div>
            <code class="dhoi-cmd">{{ opt.cmd }}</code>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 LOS 3 NIVELES DE DESHACER
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Los 3 Niveles de Deshacer
      </SectionTitle>

      <TextBlock>
        El comando para deshacer depende de <em>hasta dónde llegaste</em>.
        Nivel 1 = más seguro, Nivel 3 = más potente (y más peligroso):
      </TextBlock>

      <div class="undo-levels q-mt-lg">
        <div v-for="lvl in undoLevels" :key="lvl.level" class="ulvl-card"
          :style="{ '--ulvl-color': lvl.color }">
          <div class="ulvl-header">
            <div class="ulvl-badge" :style="{ background: lvl.color }">Nivel {{ lvl.level }}</div>
            <div class="ulvl-title">{{ lvl.title }}</div>
          </div>
          <div class="ulvl-scenario">
            <strong>Situación:</strong> {{ lvl.scenario }}
          </div>
          <CodeBlock :hide-header="true" lang="bash" :content="lvl.code" :copyable="true" />
          <div class="ulvl-note" :class="lvl.safe ? 'ulvl-safe' : 'ulvl-warn'">
            {{ lvl.note }}
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         06 RESET vs REVERT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        reset vs revert — La Diferencia Crucial
      </SectionTitle>

      <TextBlock>
        Ambos "deshacen" commits, pero de formas completamente diferentes. Elegir mal puede
        romper el trabajo de tu equipo. La regla es simple:
      </TextBlock>

      <div class="rvr-grid q-mt-lg">
        <div v-for="cmd in rvCmds" :key="cmd.name" class="rvr-card"
          :style="{ '--rvr-color': cmd.color }">
          <div class="rvrc-header">
            <q-icon :name="cmd.icon" size="24px" :style="{ color: cmd.color }" />
            <code class="rvrc-name">{{ cmd.name }}</code>
            <span class="rvrc-safety" :style="{ background: cmd.safe ? '#4ade8018' : '#f8717118', color: cmd.safe ? '#4ade80' : '#f87171', borderColor: cmd.safe ? '#4ade8030' : '#f8717130' }">
              {{ cmd.safe ? '✓ seguro siempre' : '⚠ solo local' }}
            </span>
          </div>
          <p class="rvrc-desc">{{ cmd.desc }}</p>

          <!-- Commit chain before/after -->
          <div class="rvrc-chain-label">Antes:</div>
          <div class="rvrc-chain">
            <div v-for="(c, idx) in cmd.before" :key="c.label + 'b'" class="chain-node"
              :class="{ 'cn-bad': c.bad, 'cn-current': c.current }"
              :style="{ '--cn-color': c.color }">
              {{ c.label }}
              <span v-if="idx < cmd.before.length - 1" class="chain-arrow">→</span>
            </div>
          </div>
          <div class="rvrc-chain-label">Después de <code>{{ cmd.action }}</code>:</div>
          <div class="rvrc-chain">
            <div v-for="(c, idx) in cmd.after" :key="c.label + 'a'" class="chain-node"
              :class="{ 'cn-deleted': c.deleted, 'cn-new': c.new, 'cn-current': c.current }"
              :style="{ '--cn-color': c.color }">
              {{ c.label }}
              <span v-if="!c.deleted && idx < cmd.after.length - 1" class="chain-arrow">→</span>
            </div>
          </div>

          <CodeBlock :hide-header="true" lang="bash" :content="cmd.code" :copyable="true" class="q-mt-sm" />
        </div>
      </div>

      <div class="rvr-rule q-mt-lg">
        <div class="rvr-rule-item" :style="{ '--rri-color': '#f87171' }">
          <q-icon name="computer" size="18px" style="color:#f87171" />
          <strong>Solo local (no push):</strong> puedes usar <code>reset</code> libremente
        </div>
        <div class="rvr-rule-item" :style="{ '--rri-color': '#4ade80' }">
          <q-icon name="cloud" size="18px" style="color:#4ade80" />
          <strong>Ya pusheado (compartido):</strong> usa siempre <code>revert</code>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         07 GIT BISECT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">07</span>
        git bisect — Búsqueda Binaria de Bugs
      </SectionTitle>

      <TextBlock>
        Sabes que en la versión 1.0.0 funcionaba. Ahora en el commit más reciente está roto.
        Hay 50 commits en el medio. <code>git bisect</code> hace una búsqueda binaria automática
        — en 6 pasos encuentra el commit culpable sin importar cuántos commits haya:
      </TextBlock>

      <!-- Bisect visual -->
      <div class="bisect-visual q-mt-lg">
        <div class="bv-label">Git divide el historial a la mitad en cada paso</div>
        <div class="bv-track">
          <div v-for="(seg, idx) in bisectSegments" :key="seg.label" class="bvt-seg"
            :style="{ '--bvs-color': seg.color, '--bvs-width': seg.width }">
            <div class="bvts-bar">
              <div class="bvts-label">{{ seg.label }}</div>
            </div>
            <div v-if="idx < bisectSegments.length - 1" class="bvts-arrow">→</div>
          </div>
        </div>
        <div class="bv-commits">
          <div v-for="bc in bisectCommits" :key="bc.num" class="bvc-dot"
            :style="{ '--bvc-color': bc.color }">
            <div class="bvcd-circle" :class="{ 'bvcd-target': bc.target }"></div>
            <div class="bvcd-num">{{ bc.num }}</div>
          </div>
        </div>
        <div class="bv-note">← good (funciona) ········· bad (roto) →</div>
      </div>

      <CodeBlock title="Flujo completo de git bisect" lang="bash"
        :content="bisectCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         08 GIT REFLOG
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">08</span>
        git reflog — La Red de Seguridad Definitiva
      </SectionTitle>

      <TextBlock>
        <code>git reflog</code> registra <em>todos los movimientos de HEAD</em>, incluyendo
        commits que "borraste" con <code>reset --hard</code>. Siempre que hayas commiteado algo,
        puedes recuperarlo aunque haya desaparecido del historial visible:
      </TextBlock>

      <div class="reflog-terminal q-mt-lg">
        <div class="lt-topbar">
          <div class="lt-dot" style="background:#f87171"></div>
          <div class="lt-dot" style="background:#fbbf24"></div>
          <div class="lt-dot" style="background:#4ade80"></div>
          <code class="lt-cmd">git reflog</code>
        </div>
        <div class="lt-body">
          <div v-for="rl in reflogLines" :key="rl.ref" class="rl-row">
            <span class="rl-hash" :style="{ color: rl.color }">{{ rl.hash }}</span>
            <span class="rl-ref">{{ rl.ref }}</span>
            <span class="rl-action" :style="{ color: rl.actionColor }">{{ rl.action }}</span>
            <span class="rl-msg">{{ rl.msg }}</span>
          </div>
        </div>
      </div>

      <CodeBlock title="Recuperar trabajo con git reflog" lang="bash"
        :content="reflogCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         09 GIT TAG
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">09</span>
        git tag — Marcar Versiones del Proyecto
      </SectionTitle>

      <TextBlock>
        Los tags son punteros permanentes a commits específicos. Se usan para marcar releases
        (v1.0.0, v2.1.3). A diferencia de las ramas, los tags no se mueven cuando haces nuevos
        commits:
      </TextBlock>

      <div class="tag-types q-mt-lg">
        <div v-for="tt in tagTypes" :key="tt.name" class="tt-card"
          :style="{ '--tt-color': tt.color }">
          <div class="ttc-header">
            <q-icon :name="tt.icon" size="18px" :style="{ color: tt.color }" />
            <span class="ttc-name">{{ tt.name }}</span>
          </div>
          <p class="ttc-desc">{{ tt.desc }}</p>
          <CodeBlock :hide-header="true" lang="bash" :content="tt.code" :copyable="true" />
        </div>
      </div>

      <div class="semver-box q-mt-xl">
        <div class="sv-title">
          <q-icon name="tag" size="16px" color="primary" />
          Semantic Versioning (semver) — El estándar para versiones
        </div>
        <div class="sv-format">
          <div v-for="part in semverParts" :key="part.name" class="svf-item"
            :style="{ '--svf-color': part.color }">
            <div class="svfi-num">{{ part.example }}</div>
            <div class="svfi-name">{{ part.name }}</div>
            <div class="svfi-desc">{{ part.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes al Navegar el Historial</SectionTitle>

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
      <SectionTitle>Reto — Detective del Historial</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Encuentra y revierte el commit que rompió el robot</div>
            <div class="challenge-subtitle">
              Usa git bisect, git blame y git revert para resolver el caso
            </div>
          </div>
          <div class="challenge-badge">50 min</div>
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

        <CodeBlock title="Comandos base del reto" lang="bash"
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
      <TextBlock>Git log, diff y navegación del historial:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Git History Navigation" frameborder="0"
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
      <SectionTitle>Resumen — Comandos de Historial</SectionTitle>
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
          <q-icon name="account_tree" size="40px" color="primary" />
        </div>
        <h2 class="fca-title">¡Historial dominado!</h2>
        <p class="fca-sub">
          Ya sabes navegar el tiempo, comparar versiones, encontrar bugs con bisect y
          recuperar trabajo con reflog. El siguiente paso: branches — la característica más
          poderosa de Git.
        </p>
        <div class="fca-actions">
          <q-btn color="primary" unelevated rounded size="lg" padding="14px 40px"
            to="/modulo-3/04ramasPage"
            icon="arrow_forward" label="Branches y Merging"
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

interface ChainNode {
  label: string;
  color: string;
  current?: boolean;
  bad?: boolean;
  deleted?: boolean;
  new?: boolean;
}

interface RvCmd {
  name: string;
  icon: string;
  color: string;
  safe: boolean;
  desc: string;
  action: string;
  before: ChainNode[];
  after: ChainNode[];
  code: string;
}

// ═══════════════════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════════════════

const heroLogCode = 'git log --oneline --graph --all --decorate';

const logAdvancedCode = [
  '# Por autor',
  'git log --oneline --author="Alexander"',
  '',
  '# Últimos 5 commits',
  'git log --oneline -5',
  '',
  '# En las últimas 2 semanas',
  'git log --oneline --since="2 weeks ago"',
  '',
  '# Buscar en mensajes de commit',
  'git log --oneline --grep="lidar"',
  '',
  '# Ver archivos que cambiaron en cada commit',
  'git log --stat --oneline',
  '',
  '# Ver solo nombres de archivos modificados',
  'git log --name-only --oneline -5',
  '',
  '# Formato personalizado: hash + autor + fecha + mensaje',
  'git log --format="%h | %an | %ar | %s" -10',
].join('\n');

const navCode = [
  '# Ver commit específico (modo solo lectura)',
  'git checkout a1b2c3d',
  '# → HEAD is now at a1b2c3d...',
  '',
  '# Ver un archivo específico de un commit pasado',
  'git show a1b2c3d:src/motor_controller.py',
  '',
  '# Restaurar un archivo específico de un commit pasado',
  'git restore --source=a1b2c3d src/motor_controller.py',
  '',
  '# Comparar archivo entre dos commits',
  'git diff 6n7o8p9 a1b2c3d -- src/motor_controller.py',
  '',
  '# Volver al presente',
  'git checkout main',
  '# o en Git moderno:',
  'git switch main',
].join('\n');

const bisectCode = [
  '# Escenario: Nav2 no arranca. Funcionaba en v1.0.0.',
  '# Hay 50 commits entre v1.0.0 y HEAD.',
  '',
  '# 1. Iniciar bisect',
  'git bisect start',
  '',
  '# 2. Marcar el commit actual como roto (bad)',
  'git bisect bad',
  '',
  '# 3. Marcar la versión que funcionaba como buena',
  'git bisect good v1.0.0',
  '# Git hace checkout al commit 25 (mitad)',
  '',
  '# 4. Probar si funciona, luego marcar',
  'ros2 launch mi_robot nav2.launch.py',
  'git bisect good   # si funciona',
  '# Git hace checkout al commit 37',
  '',
  'ros2 launch mi_robot nav2.launch.py',
  'git bisect bad    # si falla',
  '# Git sigue dividiendo...',
  '',
  '# Después de ~6 iteraciones:',
  '# a1b2c3d is the first bad commit',
  '# feat(nav2): update velocity limits  ← CULPABLE',
  '',
  '# 5. Volver al estado normal',
  'git bisect reset',
  '',
  '# 6. Ver qué cambió en ese commit',
  'git show a1b2c3d',
].join('\n');

const reflogCode = [
  '# Ver el reflog (todos los movimientos de HEAD)',
  'git reflog',
  '',
  '# Situación: hiciste git reset --hard y perdiste un commit',
  '# git reflog muestra:',
  '# a1b2c3d HEAD@{1}: commit: feat(lidar): add filter ← el commit "perdido"',
  '# 9f8e7d6 HEAD@{2}: reset: moving to HEAD~1',
  '',
  '# Recuperar: crear rama desde el commit perdido',
  'git checkout -b recovery/lidar-filter a1b2c3d',
  '',
  '# O restaurarlo directamente a main',
  'git checkout main',
  'git merge recovery/lidar-filter',
  '',
  '# Reflog también guarda operaciones de merge, rebase, etc.',
  'git reflog --all',
].join('\n');

const challengeCode = [
  '# Setup: clona un repositorio con un bug',
  'git clone https://github.com/tu-usuario/ros2-bug-hunt.git',
  'cd ros2-bug-hunt',
  '',
  '# El robot funcionaba en v1.0.0 pero falla en main',
  '# Tu misión: encontrar y revertir el commit culpable',
  '',
  '# Paso 1: Ver el historial',
  'git log --oneline --graph',
  '',
  '# Paso 2: Iniciar bisect',
  'git bisect start',
  'git bisect bad HEAD',
  'git bisect good v1.0.0',
  '',
  '# Paso 3: En cada checkout de bisect, probar:',
  'python3 tests/test_motor.py',
  '# Si pasa: git bisect good',
  '# Si falla: git bisect bad',
  '',
  '# Paso 4: Ver qué cambió en el commit culpable',
  'git show <hash-del-culpable>',
  'git blame src/motor_controller.py',
  '',
  '# Paso 5: Revertir',
  'git bisect reset',
  'git revert <hash-del-culpable>',
].join('\n');

// ═══════════════════════════════════════════════════════════════
// DATA ARRAYS
// ═══════════════════════════════════════════════════════════════

const facts = [
  { icon: '🔎', label: 'git bisect: con 1000 commits entre "funciona" y "roto", encuentra el culpable en 10 pasos' },
  { icon: '🛡️', label: 'git reflog: incluso después de git reset --hard, los commits son recuperables durante 90 días' },
  { icon: '📍', label: 'git tag: marca versiones permanentes — un tag nunca se mueve aunque hagas nuevos commits' },
];

const logLines = [
  { id: 1,  graph: '*',   graphColor: '#4ade80', hash: 'a1b2c3d', refs: [{ label: 'HEAD → main', color: '#fbbf24' }, { label: 'origin/main', color: '#60a5fa' }], msg: 'feat: add obstacle avoidance' },
  { id: 2,  graph: '*',   graphColor: '#4ade80', hash: '4e5f6g7', refs: [], msg: 'fix(nav2): correct waypoint tolerance' },
  { id: 3,  graph: '|\\', graphColor: '#94a3b8', hash: '', refs: [], msg: '' },
  { id: 4,  graph: '| *', graphColor: '#c084fc', hash: '8h9i0j1', refs: [{ label: 'feature/lidar', color: '#c084fc' }], msg: 'feat(lidar): add min_distance filter' },
  { id: 5,  graph: '| *', graphColor: '#c084fc', hash: '2k3l4m5', refs: [], msg: 'feat(lidar): create LaserScan subscriber' },
  { id: 6,  graph: '|/', graphColor: '#94a3b8', hash: '', refs: [], msg: '' },
  { id: 7,  graph: '*',   graphColor: '#4ade80', hash: '6n7o8p9', refs: [{ label: 'tag: v1.0.0', color: '#f97316' }], msg: 'chore: initial ROS 2 workspace setup' },
];

const logFlags = [
  { flag: '--oneline',       color: '#4ade80', icon: 'compress',      desc: 'Una línea por commit: hash corto + mensaje', example: 'a1b2c3d feat: add LIDAR' },
  { flag: '--graph',         color: '#60a5fa', icon: 'account_tree',  desc: 'Gráfico ASCII de branches y merges', example: 'Muestra * y |\\|/' },
  { flag: '--all',           color: '#c084fc', icon: 'all_inclusive',  desc: 'Todas las branches, no solo la actual', example: 'Ve feature/* y main juntos' },
  { flag: '--decorate',      color: '#fbbf24', icon: 'label',         desc: 'Muestra HEAD, branches y tags en el log', example: '(HEAD → main, tag: v1.0)' },
  { flag: '-n 5',            color: '#f97316', icon: 'filter_5',      desc: 'Limita a los últimos N commits', example: 'git log --oneline -10' },
  { flag: '--since="2 weeks"',color: '#f87171', icon: 'calendar_today', desc: 'Commits desde una fecha', example: '"yesterday", "2025-01-01"' },
];

const diffCmds = [
  { cmd: 'git diff',               color: '#f87171', icon: 'difference',    desc: 'Working Directory — cambios sin stagear' },
  { cmd: 'git diff --staged',      color: '#fbbf24', icon: 'check_box',     desc: 'Staging Area — lo que se va a commitear' },
  { cmd: 'git diff main..feature', color: '#60a5fa', icon: 'compare_arrows',desc: 'Entre dos branches — todos los cambios' },
  { cmd: 'git diff HEAD~2 HEAD',   color: '#c084fc', icon: 'history',       desc: 'Entre commits — 2 atrás vs ahora' },
  { cmd: 'git diff -- archivo.py', color: '#4ade80', icon: 'description',   desc: 'Solo un archivo específico' },
  { cmd: 'git diff --stat',        color: '#94a3b8', icon: 'bar_chart',     desc: 'Resumen: archivos y líneas cambiadas' },
];

const diffLines = [
  { id: 1, type: 'file',    prefix: '',   content: 'diff --git a/src/motor_controller.py b/src/motor_controller.py' },
  { id: 2, type: 'meta',    prefix: '',   content: 'index a1b2c3d..4e5f6g7 100644' },
  { id: 3, type: 'meta',    prefix: '',   content: '--- a/src/motor_controller.py' },
  { id: 4, type: 'meta',    prefix: '',   content: '+++ b/src/motor_controller.py' },
  { id: 5, type: 'hunk',    prefix: '',   content: '@@ -42,7 +42,9 @@ class MotorController:' },
  { id: 6, type: 'context', prefix: ' ',  content: '    max_rpm = 1000' },
  { id: 7, type: 'removed', prefix: '-',  content: '    speed = rpm / 100' },
  { id: 8, type: 'added',   prefix: '+',  content: '    if rpm > max_rpm:' },
  { id: 9, type: 'added',   prefix: '+',  content: '        rpm = max_rpm' },
  { id: 10,type: 'added',   prefix: '+',  content: '    speed = rpm / max_rpm' },
  { id: 11,type: 'context', prefix: ' ',  content: '    return speed' },
];

const inspectCmds = [
  {
    name: 'git show',
    icon: 'preview',
    color: '#60a5fa',
    desc: 'Muestra el contenido completo de un commit: metadatos, archivos cambiados y el diff.',
    code: [
      '# Ver el commit más reciente',
      'git show HEAD',
      '',
      '# Ver un commit específico',
      'git show a1b2c3d',
      '',
      '# Ver solo los archivos que cambiaron',
      'git show a1b2c3d --name-only',
      '',
      '# Ver el contenido de un archivo en un commit',
      'git show a1b2c3d:src/motor_controller.py',
    ].join('\n'),
  },
  {
    name: 'git blame',
    icon: 'person_search',
    color: '#c084fc',
    desc: 'Anota cada línea de un archivo con quién la escribió, cuándo y en qué commit.',
    code: [
      '# Ver quién escribió cada línea',
      'git blame src/motor_controller.py',
      '',
      '# Mostrar solo las líneas 40-60',
      'git blame -L 40,60 src/motor_controller.py',
      '',
      '# Ignorar espacios y reformateos',
      'git blame -w src/motor_controller.py',
      '',
      '# Ver en VS Code (más cómodo):',
      '# Extensión: GitLens → click en línea',
    ].join('\n'),
  },
];

const blameLines = [
  { hash: 'a1b2c3d', color: '#4ade80', author: 'Alexander', date: '2025-01-11', lineNum: '42)', code: '    max_rpm = 1000' },
  { hash: 'f87g8h9', color: '#f87171', author: 'Alexander', date: '2025-01-08', lineNum: '43)', code: '    if rpm > max_rpm:' },
  { hash: 'f87g8h9', color: '#f87171', author: 'Alexander', date: '2025-01-08', lineNum: '44)', code: '        rpm = max_rpm' },
  { hash: 'a1b2c3d', color: '#4ade80', author: 'Alexander', date: '2025-01-11', lineNum: '45)', code: '    speed = rpm / max_rpm' },
  { hash: '2k3l4m5', color: '#fbbf24', author: 'Alexander', date: '2024-12-15', lineNum: '46)', code: '    return speed' },
];

const timelinePoints = [
  { hash: '6n7o8p9', label: 'v1.0.0 — Release', date: 'Hace 2 meses', color: '#f97316', current: false },
  { hash: '2k3l4m5', label: 'feat: add LIDAR',  date: 'Hace 1 mes',   color: '#60a5fa', current: false },
  { hash: '8h9i0j1', label: 'fix: motor bug',   date: 'Hace 1 semana',color: '#4ade80', current: false },
  { hash: 'a1b2c3d', label: 'HEAD → main',      date: 'Ahora',        color: '#fbbf24', current: true },
];

const detachedOptions = [
  { title: 'Solo mirar — volver sin guardar', color: '#94a3b8', icon: 'visibility',  cmd: 'git checkout main' },
  { title: 'Quiero hacer cambios desde aquí', color: '#4ade80', icon: 'add_road',    cmd: 'git checkout -b nueva-rama' },
  { title: 'Restaurar solo un archivo',        color: '#60a5fa', icon: 'file_copy',   cmd: 'git restore --source=<hash> archivo.py' },
];

const undoLevels = [
  {
    level: 1,
    title: 'Deshacer cambios sin stagear',
    color: '#fbbf24',
    scenario: 'Modificaste archivos pero NO hiciste git add — quieres descartar los cambios',
    safe: false,
    note: '⚠️ Los cambios se pierden permanentemente — no hay recuperación posible',
    code: [
      '# Descartar cambios en un archivo',
      'git restore motor_controller.py',
      '',
      '# Descartar TODOS los cambios locales',
      'git restore .',
    ].join('\n'),
  },
  {
    level: 2,
    title: 'Sacar archivos del staging',
    color: '#f97316',
    scenario: 'Hiciste git add pero NO git commit — quieres quitar algo del staging',
    safe: true,
    note: '✅ Los cambios permanecen en Working Directory — no se pierde nada',
    code: [
      '# Sacar archivo del staging (conserva cambios)',
      'git restore --staged motor_controller.py',
      '',
      '# Sacar todo del staging',
      'git restore --staged .',
    ].join('\n'),
  },
  {
    level: 3,
    title: 'Deshacer commits',
    color: '#f87171',
    scenario: 'Hiciste commit pero el commit es incorrecto — quieres deshacerlo',
    safe: false,
    note: '⚠️ reset --hard borra los cambios en disco. reset sin --hard los conserva. revert es siempre seguro.',
    code: [
      '# Deshacer último commit (mantiene cambios)',
      'git reset HEAD~1',
      '',
      '# Deshacer y borrar los cambios (¡destructivo!)',
      'git reset --hard HEAD~1',
      '',
      '# Revertir commit ya pusheado (seguro)',
      'git revert HEAD',
    ].join('\n'),
  },
];

const rvCmds: RvCmd[] = [
  {
    name: 'git reset',
    icon: 'delete_sweep',
    color: '#f87171',
    safe: false,
    desc: 'BORRA commits del historial. Como si no hubieran existido. Peligroso en ramas compartidas.',
    action: 'git reset --hard B',
    before: [
      { label: 'A', color: '#4ade80', current: false },
      { label: 'B', color: '#4ade80', current: false },
      { label: 'C', color: '#f87171', bad: true },
      { label: 'D', color: '#60a5fa', current: true },
    ],
    after: [
      { label: 'A', color: '#4ade80', current: false },
      { label: 'B', color: '#4ade80', current: true },
      { label: 'C', color: '#94a3b8', deleted: true },
      { label: 'D', color: '#94a3b8', deleted: true },
    ],
    code: [
      '# Volver 2 commits atrás (PELIGROSO si ya hiciste push)',
      'git reset --hard HEAD~2',
      '',
      '# Volver a commit específico',
      'git reset --hard a1b2c3d',
    ].join('\n'),
  },
  {
    name: 'git revert',
    icon: 'undo',
    color: '#4ade80',
    safe: true,
    desc: 'CREA un nuevo commit que deshace los cambios. El historial queda intacto.',
    action: 'git revert C',
    before: [
      { label: 'A', color: '#4ade80', current: false },
      { label: 'B', color: '#4ade80', current: false },
      { label: 'C', color: '#f87171', bad: true },
      { label: 'D', color: '#60a5fa', current: true },
    ],
    after: [
      { label: 'A', color: '#4ade80', current: false },
      { label: 'B', color: '#4ade80', current: false },
      { label: 'C', color: '#f87171', bad: true },
      { label: 'D', color: '#60a5fa', current: false },
      { label: "C'", color: '#4ade80', new: true, current: true },
    ],
    code: [
      '# Revertir commit específico',
      'git revert a1b2c3d',
      '',
      '# Revertir sin abrir editor (usa mensaje por defecto)',
      'git revert a1b2c3d --no-edit',
    ].join('\n'),
  },
];

const bisectSegments = [
  { label: 'good ✓', color: '#4ade80', width: '20%' },
  { label: 'bisect 1', color: '#fbbf24', width: '20%' },
  { label: 'bisect 2', color: '#f97316', width: '20%' },
  { label: 'bisect 3', color: '#f87171', width: '20%' },
  { label: '💥 bad', color: '#f87171', width: '20%' },
];

const bisectCommits = [
  { num: '1', color: '#4ade80', target: false },
  { num: '2', color: '#4ade80', target: false },
  { num: '3', color: '#fbbf24', target: true },
  { num: '4', color: '#f87171', target: false },
  { num: '5', color: '#f87171', target: false },
  { num: '6', color: '#f87171', target: false },
  { num: '7', color: '#f87171', target: false },
  { num: '8', color: '#f87171', target: false },
];

const reflogLines = [
  { hash: 'a1b2c3d', color: '#4ade80', ref: 'HEAD@{0}', action: 'commit:', actionColor: '#4ade80', msg: 'feat: add obstacle avoidance' },
  { hash: '9f8e7d6', color: '#f87171', ref: 'HEAD@{1}', action: 'reset:', actionColor: '#f87171',  msg: 'moving to HEAD~1' },
  { hash: 'b2c3d4e', color: '#fbbf24', ref: 'HEAD@{2}', action: 'commit:', actionColor: '#4ade80', msg: 'feat(lidar): add distance filter  ← "perdido"' },
  { hash: '3d4e5f6', color: '#60a5fa', ref: 'HEAD@{3}', action: 'checkout:', actionColor: '#60a5fa', msg: 'moving from feature/lidar to main' },
  { hash: 'c4d5e6f', color: '#fbbf24', ref: 'HEAD@{4}', action: 'commit:', actionColor: '#4ade80', msg: 'fix(motor): correct speed overflow' },
];

const tagTypes = [
  {
    name: 'Tag ligero',
    icon: 'label',
    color: '#60a5fa',
    desc: 'Solo un puntero a un commit, sin metadatos adicionales. Uso rápido e informal.',
    code: [
      '# Crear tag en el commit actual',
      'git tag v1.0.0',
      '',
      '# Crear tag en un commit específico',
      'git tag v1.0.0 a1b2c3d',
      '',
      '# Ver todos los tags',
      'git tag -l',
    ].join('\n'),
  },
  {
    name: 'Tag anotado',
    icon: 'bookmark',
    color: '#4ade80',
    desc: 'Incluye autor, fecha, mensaje y firma GPG opcional. Recomendado para releases oficiales.',
    code: [
      '# Crear tag anotado (abre editor)',
      'git tag -a v1.0.0 -m "Release v1.0.0 — TurtleBot3 Nav2 integration"',
      '',
      '# Ver detalles del tag',
      'git show v1.0.0',
      '',
      '# Subir tags a GitHub',
      'git push origin v1.0.0',
      'git push origin --tags  # todos los tags',
    ].join('\n'),
  },
];

const semverParts = [
  { name: 'MAJOR', example: '2', color: '#f87171', desc: 'Cambio que rompe compatibilidad (BREAKING CHANGE)' },
  { name: 'MINOR', example: '.1', color: '#fbbf24', desc: 'Nueva feature sin romper compatibilidad' },
  { name: 'PATCH', example: '.3', color: '#4ade80', desc: 'Bugfix compatible con versiones anteriores' },
];

const commonErrors = reactive([
  {
    type: 'git reset --hard y los cambios desaparecieron',
    summary: 'Ejecutaste reset --hard para limpiar y borraste trabajo no commiteado',
    color: '#f87171',
    cause: 'git reset --hard borra los cambios del working directory sin posibilidad de recuperación con git — solo funciona si el trabajo estaba commiteado',
    code: [
      '# ❌ Perdiste cambios no commiteados con reset --hard',
      '# No hay recuperación si nunca commiteaste',
      '',
      '# ✅ Si SÍ hiciste commit antes del reset, usa reflog',
      'git reflog',
      '# HEAD@{1}: commit: feat(lidar): add filter ← el commit "perdido"',
      '',
      '# Recuperar creando rama desde ese commit',
      'git checkout -b recovery/lidar a1b2c3d',
      '',
      '# Prevención: commitea frecuentemente aunque sea WIP',
      'git commit -m "WIP: lidar filter in progress"',
    ].join('\n'),
    fix: 'Si el trabajo estaba commiteado: usa git reflog para encontrarlo. Si nunca commiteaste: no hay recuperación. Por eso: commitea frecuente.',
    open: false,
  },
  {
    type: 'Detached HEAD — commits perdidos al volver a una rama',
    summary: 'Hiciste commits en Detached HEAD y al hacer git checkout main desaparecieron',
    color: '#fbbf24',
    cause: 'En Detached HEAD los commits no pertenecen a ninguna rama — si cambias de branch sin crear una rama primero, esos commits quedan "flotando" y Git los limpia eventualmente',
    code: [
      '# ❌ Situación: hiciste commits en Detached HEAD',
      '# git checkout main → ahora no ves esos commits',
      '',
      '# ✅ Si es reciente, reflog los guarda',
      'git reflog',
      '# HEAD@{2}: commit: feat: add experimental algo  ← el commit',
      '',
      '# Crear rama desde ese commit',
      'git checkout -b feature/experimental a1b2c3d',
      '',
      '# Prevención: SIEMPRE crear rama antes de commitear en Detached HEAD',
      'git checkout -b mi-experimento',
    ].join('\n'),
    fix: 'Usa git reflog para encontrar el hash del commit perdido y crear una rama desde él. En el futuro: git checkout -b nueva-rama antes de hacer cambios en Detached HEAD.',
    open: false,
  },
  {
    type: 'git reset en main después de push — historial divergente',
    summary: 'Usaste git reset en un commit que ya habías subido a GitHub',
    color: '#c084fc',
    cause: 'git push falla con "rejected non-fast-forward" porque el historial local y remoto divergieron',
    code: [
      '# ❌ Situación: reset en commit ya pusheado',
      '# git push → rejected non-fast-forward',
      '',
      '# ✅ La forma correcta: git revert',
      'git revert HEAD~2..HEAD  # revertir los últimos 2 commits',
      'git push                 # funciona — no reescribe historial',
      '',
      '# Si es tu rama personal y estás seguro:',
      'git push --force-with-lease  # más seguro que --force',
      '# Avisa al equipo antes de hacer esto',
      '',
      '# ❌ NUNCA en main o ramas compartidas:',
      '# git push --force',
    ].join('\n'),
    fix: 'Usa git revert para deshacer commits ya pusheados — crea un nuevo commit que invierte los cambios sin tocar el historial.',
    open: false,
  },
  {
    type: 'git diff no muestra nada — pero sí hay cambios',
    summary: 'Modificaste archivos, pero git diff muestra vacío',
    color: '#60a5fa',
    cause: 'git diff sin argumentos solo muestra cambios NO staged. Si ya hiciste git add, esos cambios están en staging y git diff los oculta',
    code: [
      '# ❌ git diff no muestra nada',
      '# Posible causa: los cambios ya están staged',
      '',
      '# Verificar qué está staged',
      'git diff --staged',
      '# ← aquí aparecen los cambios',
      '',
      '# Ver TODO: staged + unstaged',
      'git diff HEAD',
      '',
      '# Ver estado general',
      'git status',
      '# Changes to be committed: ← estos ya no aparecen en git diff',
      '# Changes not staged: ← estos sí aparecen en git diff',
    ].join('\n'),
    fix: 'git diff (sin args) = cambios sin stagear. git diff --staged = cambios staged. git diff HEAD = todos. Siempre usa git status primero para entender el estado.',
    open: false,
  },
  {
    type: 'Tag creado en el commit equivocado',
    summary: 'Pusheaste el tag v1.0.0 pero apunta al commit incorrecto',
    color: '#f97316',
    cause: 'git tag sin especificar commit usa el HEAD actual. Si estabas en la rama equivocada, el tag quedó en el commit incorrecto.',
    code: [
      '# ❌ Tag v1.0.0 apunta al commit equivocado',
      '',
      '# Si NO has hecho push del tag:',
      '# 1. Borrar el tag local',
      'git tag -d v1.0.0',
      '',
      '# 2. Recrear en el commit correcto',
      'git tag -a v1.0.0 a1b2c3d -m "Release v1.0.0"',
      '',
      '# Si YA hiciste push del tag:',
      '# 1. Borrar el tag remoto',
      'git push origin --delete v1.0.0',
      '',
      '# 2. Borrar el tag local y recrear',
      'git tag -d v1.0.0',
      'git tag -a v1.0.0 a1b2c3d -m "Release v1.0.0"',
      'git push origin v1.0.0',
    ].join('\n'),
    fix: 'git tag -d para borrar el tag local, luego recrear con el hash correcto. Si ya está en GitHub, primero borrarlo remoto con git push origin --delete.',
    open: false,
  },
]);

const challengeSteps = [
  { num: 1, color: '#4ade80', text: 'Crea un repositorio con 10 commits, uno de los cuales "rompe" una función (devuelve valor incorrecto)' },
  { num: 2, color: '#60a5fa', text: 'Usa git bisect start, git bisect bad y git bisect good para encontrar el commit culpable en ≤ 4 pasos' },
  { num: 3, color: '#fbbf24', text: 'Usa git blame en el archivo afectado para confirmar quién y cuándo hizo el cambio' },
  { num: 4, color: '#c084fc', text: 'Usa git show <hash> para ver exactamente qué cambió en ese commit' },
  { num: 5, color: '#f87171', text: 'Usa git revert <hash> para deshacer el commit culpable sin perder los commits posteriores' },
  { num: 6, color: '#f97316', text: '(Bonus) Crea un tag v1.0.0 en el commit anterior al bug y v1.1.0 en el revert' },
];

const challengeHints = [
  'Para crear el bug: cambia speed = rpm / max_rpm a speed = rpm / 100 en algún commit intermedio',
  'git bisect good v1.0.0 necesita que hayas creado el tag v1.0.0 antes del bug',
  'Después de git bisect reset, el hash del commit culpable sigue visible en git reflog',
  'git revert no borra commits — el historial queda intacto, solo agrega un commit nuevo',
  'Para verificar que el revert funcionó: git log --oneline debe mostrar el revert commit',
];

const summaryItems = [
  { cmd: 'git log --oneline --graph --all', desc: 'Historial visual completo', example: 'ver branches y merges', color: '#4ade80' },
  { cmd: 'git diff --staged',               desc: 'Ver qué está en staging',   example: 'antes de cada commit',   color: '#fbbf24' },
  { cmd: 'git show <hash>',                 desc: 'Contenido completo de commit', example: 'diff + metadatos',   color: '#60a5fa' },
  { cmd: 'git blame archivo.py',            desc: 'Quién escribió cada línea', example: '-L 40,60 para rango',   color: '#c084fc' },
  { cmd: 'git checkout <hash>',             desc: 'Viajar al pasado (solo lectura)', example: 'git checkout main para volver', color: '#f97316' },
  { cmd: 'git revert HEAD',                 desc: 'Deshacer commit (seguro)',   example: 'crear commit inverso',  color: '#4ade80' },
  { cmd: 'git bisect start/bad/good',       desc: 'Encontrar commit del bug',   example: 'búsqueda binaria',     color: '#f87171' },
  { cmd: 'git reflog',                      desc: 'Red de seguridad total',     example: 'recuperar lo "perdido"', color: '#fbbf24' },
  { cmd: 'git tag -a v1.0.0 -m "msg"',     desc: 'Marcar versión de release',  example: 'git push origin --tags', color: '#60a5fa' },
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
   HERO CMD
══════════════════════════════════════════ */
.hero-cmd { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-top: 3px solid #c084fc; border-radius: 14px; padding: 1.25rem; }
.hcmd-label { font-size: .82rem; font-weight: 600; color: var(--text-muted); margin-bottom: 10px; }

/* ══════════════════════════════════════════
   LOG TERMINAL (shared style)
══════════════════════════════════════════ */
.log-terminal, .diff-terminal, .blame-terminal, .reflog-terminal {
  background: var(--bg-deep, #0d1117); border: 1px solid var(--border-subtle);
  border-radius: 14px; overflow: hidden;
}
.lt-topbar {
  display: flex; align-items: center; gap: 8px;
  padding: 10px 16px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle);
}
.lt-dot  { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
.lt-cmd  { font-family: 'Fira Code', monospace; font-size: .82rem; color: var(--text-muted); background: none; padding: 0; margin-left: 8px; }
.lt-body { padding: 1.25rem; font-family: 'Fira Code', monospace; font-size: .85rem; display: flex; flex-direction: column; gap: 5px; }
.lt-line { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.lt-graph { font-weight: 700; min-width: 20px; }
.lt-hash  { color: #f97316; font-weight: 700; }
.lt-refs  { display: flex; gap: 5px; flex-wrap: wrap; }
.lt-ref   { font-size: .72rem; font-weight: 700; padding: 2px 7px; border-radius: 5px; border: 1px solid; }
.lt-msg   { color: var(--text-secondary, #94a3b8); }

/* ══════════════════════════════════════════
   FLAGS GRID
══════════════════════════════════════════ */
.flags-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
.flag-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--flag-color); border-radius: 12px;
  padding: 12px 14px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.flagc-header { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.flagc-flag   { font-family: 'Fira Code', monospace; font-size: .84rem; font-weight: 700; color: var(--flag-color); background: none; padding: 0; }
.flagc-desc   { font-size: .78rem; color: var(--text-secondary); line-height: 1.4; flex: 1; }
.flagc-example { font-family: 'Fira Code', monospace; font-size: .7rem; color: var(--text-muted); border-top: 1px solid var(--border-subtle); padding-top: 6px; }

/* ══════════════════════════════════════════
   DIFF
══════════════════════════════════════════ */
.diff-cards { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
.diffc-item {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--dc-color); border-radius: 10px;
  padding: 10px 12px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.diffc-header { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.diffc-cmd { font-family: 'Fira Code', monospace; font-size: .8rem; font-weight: 700; color: var(--dc-color); background: none; padding: 0; word-break: break-all; }
.diffc-desc { font-size: .76rem; color: var(--text-muted); line-height: 1.4; }
/* diff terminal */
.dt-topbar { display: flex; align-items: center; gap: 8px; padding: 10px 16px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); }
.dt-body { padding: 1.25rem; font-family: 'Fira Code', monospace; font-size: .85rem; display: flex; flex-direction: column; gap: 1px; }
.dt-line { display: flex; gap: 8px; min-width: 0; }
.dt-prefix { width: 14px; flex-shrink: 0; font-weight: 700; }
.dt-content { flex: 1; word-break: break-all; }
.dt-file    { color: var(--text-primary); font-weight: 700; margin-bottom: 4px; }
.dt-meta    { color: var(--text-muted); opacity: .7; }
.dt-hunk    { color: var(--dt-hunk, #60a5fa); margin: 4px 0; }
.dt-context { color: var(--text-secondary); }
.dt-removed { background: color-mix(in srgb, var(--dt-rem, #f87171) 12%, transparent); color: var(--dt-rem, #f87171); padding-left: 4px; border-radius: 3px; }
.dt-removed .dt-prefix { color: var(--dt-rem, #f87171); }
.dt-added   { background: color-mix(in srgb, var(--dt-add, #4ade80) 12%, transparent); color: var(--dt-add, #4ade80); padding-left: 4px; border-radius: 3px; }
.dt-added   .dt-prefix { color: var(--dt-add, #4ade80); }
/* diff legend */
.diff-legend { display: flex; flex-wrap: wrap; gap: 14px; }
.dleg-item { display: flex; align-items: center; gap: 7px; font-size: .8rem; color: var(--text-muted); }
.dleg-dot  { width: 10px; height: 10px; border-radius: 3px; background: var(--dl-color); flex-shrink: 0; }

/* ══════════════════════════════════════════
   INSPECT
══════════════════════════════════════════ */
.inspect-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.inspectc-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ic-color); border-radius: 14px;
  display: flex; flex-direction: column; gap: 10px; padding: 1.25rem; min-width: 0;
}
.icc-header { display: flex; align-items: center; gap: 10px; }
.icc-name   { font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 800; color: var(--ic-color); background: none; padding: 0; }
.icc-desc   { font-size: .84rem; color: var(--text-secondary); line-height: 1.5; margin: 0; }
/* blame terminal */
.blame-body { gap: 3px; }
.bl-row  { display: grid; grid-template-columns: 60px 80px 80px 30px 1fr; gap: 8px; align-items: center; font-size: .78rem; padding: 2px 0; }
.bl-hash { font-weight: 700; }
.bl-author, .bl-date { color: var(--text-muted); }
.bl-num  { color: var(--text-muted); text-align: right; }
.bl-code { color: var(--text-secondary); }

/* ══════════════════════════════════════════
   TIMELINE VISUAL
══════════════════════════════════════════ */
.timeline-visual { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.tv-label { font-size: .88rem; font-weight: 600; color: var(--text-secondary); text-align: center; margin-bottom: 1.5rem; }
.tv-track { display: flex; align-items: center; justify-content: center; flex-wrap: nowrap; gap: 0; overflow-x: auto; }
.tv-item  { display: flex; align-items: center; }
.tv-connector { flex: 0 0 40px; height: 2px; background: var(--border-medium); }
.tv-node {
  display: flex; flex-direction: column; align-items: center; gap: 6px;
  cursor: default;
}
.tvn-dot {
  width: 18px; height: 18px; border-radius: 50%;
  background: var(--tvn-color); border: 3px solid var(--bg-surface);
  box-shadow: 0 0 0 2px var(--tvn-color);
  flex-shrink: 0;
}
.tv-node-current .tvn-dot {
  width: 22px; height: 22px;
  box-shadow: 0 0 0 2px var(--tvn-color), 0 0 12px color-mix(in srgb, var(--tvn-color) 50%, transparent);
}
.tvn-info { text-align: center; min-width: 80px; }
.tvn-hash  { font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--tvn-color); font-weight: 700; }
.tvn-label { font-size: .76rem; color: var(--text-secondary); margin-top: 2px; }
.tvn-date  { font-size: .68rem; color: var(--text-muted); }
/* detached HEAD */
.detached-head { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid #f87171; border-radius: 12px; padding: 1.25rem; }
.dh-header { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
.dh-title  { font-size: .9rem; font-weight: 700; color: var(--text-primary); }
.dh-desc   { font-size: .84rem; color: var(--text-secondary); line-height: 1.5; margin: 0 0 12px; }
.dh-desc code { background: none; padding: 0; }
.dh-options { display: flex; gap: 10px; flex-wrap: wrap; }
.dho-item  { flex: 1; min-width: 160px; background: var(--bg-surface-hover); border-radius: 10px; padding: 10px 12px; border-left: 3px solid var(--dho-color); }
.dhoi-header { display: flex; align-items: center; gap: 6px; font-size: .8rem; font-weight: 700; color: var(--text-primary); margin-bottom: 6px; }
.dhoi-cmd { font-family: 'Fira Code', monospace; font-size: .76rem; color: var(--dho-color); display: block; word-break: break-all; }

/* ══════════════════════════════════════════
   UNDO LEVELS
══════════════════════════════════════════ */
.undo-levels { display: flex; flex-direction: column; gap: 10px; }
.ulvl-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--ulvl-color); border-radius: 12px;
  padding: 1.25rem; display: flex; flex-direction: column; gap: 10px;
}
.ulvl-header { display: flex; align-items: center; gap: 12px; }
.ulvl-badge { font-size: .78rem; font-weight: 800; color: #0d1117; padding: 4px 10px; border-radius: 8px; }
.ulvl-title { font-size: .95rem; font-weight: 700; color: var(--text-primary); }
.ulvl-scenario { font-size: .84rem; color: var(--text-secondary); }
.ulvl-note { font-size: .8rem; padding: 8px 12px; border-radius: 8px; }
.ulvl-safe { background: color-mix(in srgb, #4ade80 10%, transparent); color: #4ade80; }
.ulvl-warn { background: color-mix(in srgb, #f87171 10%, transparent); color: #f87171; }

/* ══════════════════════════════════════════
   RESET VS REVERT
══════════════════════════════════════════ */
.rvr-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.rvr-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--rvr-color); border-radius: 14px;
  padding: 1.25rem; display: flex; flex-direction: column; gap: 10px; min-width: 0;
}
.rvrc-header { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.rvrc-name   { font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 800; color: var(--rvr-color); background: none; padding: 0; }
.rvrc-safety { font-size: .7rem; font-weight: 800; padding: 2px 8px; border-radius: 999px; border: 1px solid; margin-left: auto; white-space: nowrap; }
.rvrc-desc   { font-size: .84rem; color: var(--text-secondary); margin: 0; line-height: 1.5; }
.rvrc-chain-label { font-size: .78rem; font-weight: 600; color: var(--text-muted); }
.rvrc-chain-label code { background: none; padding: 0; }
.rvrc-chain { display: flex; align-items: center; flex-wrap: wrap; gap: 4px; background: var(--bg-surface-hover); padding: 8px 12px; border-radius: 8px; }
.chain-node {
  width: 40px; height: 40px; border-radius: 8px;
  background: color-mix(in srgb, var(--cn-color) 25%, var(--bg-surface));
  border: 2px solid var(--cn-color);
  display: flex; align-items: center; justify-content: center;
  font-size: .82rem; font-weight: 800; color: var(--cn-color);
  flex-shrink: 0; position: relative;
}
.cn-bad  { border-style: dashed; }
.cn-deleted { opacity: .3; border-color: var(--border-subtle); background: var(--bg-surface-hover); color: var(--text-muted); }
.cn-new  { border-color: #4ade80; color: #4ade80; background: color-mix(in srgb, #4ade80 15%, transparent); }
.cn-current { box-shadow: 0 0 0 2px color-mix(in srgb, var(--cn-color) 30%, transparent); }
.chain-arrow { color: var(--text-muted); font-size: .9rem; margin-left: 4px; }
.rvr-rule { display: flex; flex-direction: column; gap: 10px; }
.rvr-rule-item { display: flex; align-items: center; gap: 10px; font-size: .88rem; color: var(--text-secondary); background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--rri-color); border-radius: 10px; padding: 10px 14px; }
.rvr-rule-item code { background: none; padding: 0; }

/* ══════════════════════════════════════════
   BISECT
══════════════════════════════════════════ */
.bisect-visual { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.bv-label { font-size: .84rem; color: var(--text-muted); text-align: center; margin-bottom: 14px; }
.bv-track { display: flex; align-items: center; justify-content: center; gap: 4px; margin-bottom: 10px; }
.bvt-seg  { display: flex; align-items: center; gap: 4px; }
.bvts-bar {
  background: color-mix(in srgb, var(--bvs-color) 15%, transparent);
  border: 1px solid color-mix(in srgb, var(--bvs-color) 30%, transparent);
  border-radius: 6px; padding: 4px 10px;
}
.bvts-label { font-size: .72rem; font-weight: 700; color: var(--bvs-color); white-space: nowrap; }
.bvts-arrow { color: var(--text-muted); font-size: .9rem; }
.bv-commits { display: flex; justify-content: center; gap: 8px; flex-wrap: wrap; }
.bvc-dot { display: flex; flex-direction: column; align-items: center; gap: 3px; }
.bvcd-circle {
  width: 28px; height: 28px; border-radius: 50%;
  background: color-mix(in srgb, var(--bvc-color) 20%, transparent);
  border: 2px solid var(--bvc-color);
}
.bvcd-target { box-shadow: 0 0 0 3px color-mix(in srgb, var(--bvc-color) 40%, transparent); }
.bvcd-num { font-size: .72rem; color: var(--text-muted); font-family: 'Fira Code', monospace; }
.bv-note { text-align: center; font-size: .74rem; color: var(--text-muted); margin-top: 8px; font-family: 'Fira Code', monospace; }

/* ══════════════════════════════════════════
   REFLOG
══════════════════════════════════════════ */
.rl-row { display: grid; grid-template-columns: 70px 90px 70px 1fr; gap: 10px; padding: 3px 0; font-size: .78rem; align-items: center; }
.rl-hash   { font-weight: 700; }
.rl-ref    { color: var(--text-muted); }
.rl-action { font-weight: 700; }
.rl-msg    { color: var(--text-secondary); }

/* ══════════════════════════════════════════
   TAGS
══════════════════════════════════════════ */
.tag-types { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.tt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--tt-color); border-radius: 14px;
  padding: 1.25rem; display: flex; flex-direction: column; gap: 10px; min-width: 0;
}
.ttc-header { display: flex; align-items: center; gap: 8px; }
.ttc-name   { font-size: .92rem; font-weight: 700; color: var(--text-primary); }
.ttc-desc   { font-size: .84rem; color: var(--text-secondary); margin: 0; line-height: 1.5; }
.semver-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.sv-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700; color: var(--text-primary); margin-bottom: 16px; }
.sv-format { display: flex; gap: 0; border: 1px solid var(--border-subtle); border-radius: 10px; overflow: hidden; }
.svf-item  { flex: 1; padding: 14px 16px; border-right: 1px solid var(--border-subtle); }
.svf-item:last-child { border-right: none; }
.svfi-num  { font-family: 'Fira Code', monospace; font-size: 1.6rem; font-weight: 900; color: var(--svf-color); }
.svfi-name { font-size: .82rem; font-weight: 700; color: var(--text-primary); margin: 4px 0 2px; }
.svfi-desc { font-size: .74rem; color: var(--text-muted); line-height: 1.4; }

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
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; word-break: break-all; }
.sc-desc    { font-size: .81rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   CTA FINAL
══════════════════════════════════════════ */
.final-cta { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 24px; padding: 3rem 2rem; display: flex; flex-direction: column; align-items: center; gap: 1rem; text-align: center; }
.fca-icon  { width: 72px; height: 72px; background: rgba(96,165,250,.1); border-radius: 20px; display: flex; align-items: center; justify-content: center; }
.fca-title { font-size: 1.5rem; font-weight: 800; color: var(--text-primary); margin: 0; }
.fca-sub   { font-size: .95rem; color: var(--text-secondary); max-width: 520px; line-height: 1.6; margin: 0; }
.fca-actions { margin-top: .5rem; }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .flags-grid  { grid-template-columns: repeat(3, 1fr); }
  .diff-cards  { grid-template-columns: repeat(3, 1fr); }
}
@media (max-width: 900px) {
  .flags-grid  { grid-template-columns: repeat(2, 1fr); }
  .diff-cards  { grid-template-columns: repeat(2, 1fr); }
  .inspect-grid { grid-template-columns: 1fr; }
  .rvr-grid    { grid-template-columns: 1fr; }
  .tag-types   { grid-template-columns: 1fr; }
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
  .sv-format   { flex-direction: column; }
  .svf-item    { border-right: none; border-bottom: 1px solid var(--border-subtle); }
  .bl-row      { grid-template-columns: 60px 70px 1fr; }
  .bl-date     { display: none; }
}
@media (max-width: 768px) {
  .flags-grid  { grid-template-columns: 1fr; }
  .diff-cards  { grid-template-columns: 1fr; }
  .rl-row      { grid-template-columns: 60px 1fr; }
  .rl-action   { display: none; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
  .tv-track    { overflow-x: auto; }
  .dh-options  { flex-direction: column; }
}
@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
}
</style>
