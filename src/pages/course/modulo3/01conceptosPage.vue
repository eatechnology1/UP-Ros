<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        Git es el sistema de control de versiones más usado en el mundo. En robótica, donde trabajas
        con archivos de código, configuración YAML y parámetros de Nav2 simultáneamente,
        Git te permite <strong>experimentar sin miedo</strong>, colaborar con tu equipo y
        viajar en el tiempo cuando algo se rompe. No es solo "guardar archivos".
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div v-for="f in facts" :key="f.label" class="fact-pill">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 CONFIGURACIÓN INICIAL
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Configuración Inicial — Antes de Todo
      </SectionTitle>

      <TextBlock>
        Antes de hacer tu primer commit, Git necesita saber quién eres.
        Esta configuración se guarda globalmente y aparecerá en cada commit:
      </TextBlock>

      <CodeBlock title="Configura tu identidad (solo una vez)" lang="bash"
        :content="gitConfigCode" :copyable="true" class="q-mt-lg" />

      <div class="config-cards q-mt-lg">
        <div v-for="cfg in configItems" :key="cfg.key" class="config-card"
          :style="{ '--cfg-color': cfg.color }">
          <code class="cfc-key">{{ cfg.key }}</code>
          <div class="cfc-desc">{{ cfg.desc }}</div>
          <div class="cfc-example">Ej: <em>{{ cfg.example }}</em></div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         02 LAS 3 ZONAS
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Las Tres Zonas de Git
      </SectionTitle>

      <TextBlock>
        Git no guarda automáticamente. Tus archivos viajan conscientemente a través de tres zonas.
        Es como preparar un paquete para envío: primero lo empacas (add), luego lo cierras (commit):
      </TextBlock>

      <!-- Zone cards -->
      <div class="zones-grid q-mt-lg">
        <div v-for="(zone, idx) in zones" :key="zone.name" class="zone-card"
          :style="{ '--zone-color': zone.color }">
          <div class="zc-header">
            <q-icon :name="zone.icon" size="28px" :style="{ color: zone.color }" />
            <span class="zc-name">{{ zone.name }}</span>
          </div>
          <div class="zc-desc">{{ zone.desc }}</div>
          <div class="zc-files">
            <div v-for="file in zone.files" :key="file.name" class="zf-item">
              <q-icon name="description" size="14px" :style="{ color: zone.color, opacity: .7 }" />
              <span class="zf-name">{{ file.name }}</span>
              <span class="zf-status" :style="{ color: zone.color }">{{ file.status }}</span>
            </div>
          </div>
          <div v-if="zone.action" class="zc-action" :style="{ background: zone.color + '14', borderColor: zone.color + '30' }">
            <code :style="{ color: zone.color }">{{ zone.action }}</code>
            <q-icon v-if="idx < zones.length - 1" name="arrow_downward" size="14px" :style="{ color: zone.color }" />
          </div>
        </div>
      </div>

      <!-- git status visual -->
      <div class="status-visual q-mt-xl">
        <div class="sv-title">
          <code>git status</code> — Así se ve en la terminal:
        </div>
        <div class="status-terminal">
          <div class="st-line st-branch">On branch main</div>
          <div class="st-line st-head">Your branch is up to date with 'origin/main'.</div>
          <div class="st-line st-section">Changes to be committed (Staging):</div>
          <div class="st-line st-staged">  &nbsp;&nbsp;new file: &nbsp;&nbsp;src/lidar_driver.py</div>
          <div class="st-line st-section">Changes not staged for commit (Working):</div>
          <div class="st-line st-modified">  &nbsp;&nbsp;modified: config/params.yaml</div>
          <div class="st-line st-section">Untracked files (sin rastrear):</div>
          <div class="st-line st-untracked">  &nbsp;&nbsp;notes.txt</div>
        </div>
        <div class="sv-legend">
          <div v-for="leg in statusLegend" :key="leg.label" class="svl-item"
            :style="{ '--svl-color': leg.color }">
            <div class="svl-dot"></div>
            <span>{{ leg.label }}</span>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         03 ANATOMÍA DE UN COMMIT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Anatomía de un Commit
      </SectionTitle>

      <TextBlock>
        Un commit no es solo "guardar". Es una captura completa del proyecto con un ID único
        (hash SHA-1) que incluye quién hizo el cambio, cuándo y qué archivos se modificaron:
      </TextBlock>

      <!-- Commit card visual -->
      <div class="commit-visual q-mt-lg">
        <div class="cv-card">
          <div class="cvc-topbar">
            <span class="cvc-hash">commit a1b2c3d4e5f6789a</span>
            <span class="cvc-ref main-ref">main</span>
            <span class="cvc-ref head-ref">HEAD</span>
          </div>
          <div class="cvc-msg">✨ feat: Add LIDAR sensor integration for TurtleBot3</div>
          <div class="cvc-meta">
            <div class="cvc-meta-row">
              <q-icon name="person" size="14px" style="color:var(--text-muted)" />
              <span>Author: Alexander &lt;alex@robot.com&gt;</span>
            </div>
            <div class="cvc-meta-row">
              <q-icon name="calendar_today" size="14px" style="color:var(--text-muted)" />
              <span>Date: Fri Jan 11 03:40:00 2025</span>
            </div>
          </div>
          <div class="cvc-diff">
            <div class="cvd-item cvd-added">
              <q-icon name="add" size="13px" color="positive" />
              <code>src/lidar_driver.py</code>
              <span class="cvd-stat">+45 líneas</span>
            </div>
            <div class="cvd-item cvd-modified">
              <q-icon name="edit" size="13px" color="warning" />
              <code>config/params.yaml</code>
              <span class="cvd-stat">+3 −1</span>
            </div>
          </div>
          <div class="cvc-parent">
            <q-icon name="arrow_upward" size="13px" style="color:var(--text-muted)" />
            parent: <code>9f8e7d6c5b4a</code>
          </div>
        </div>

        <div class="cv-fields">
          <div v-for="field in commitFields" :key="field.name" class="cvf-item"
            :style="{ '--cvf-color': field.color }">
            <div class="cvf-header">
              <q-icon :name="field.icon" size="18px" :style="{ color: field.color }" />
              <span class="cvf-name">{{ field.name }}</span>
            </div>
            <p class="cvf-desc">{{ field.desc }}</p>
          </div>
        </div>
      </div>

      <!-- Conventional Commits -->
      <div class="conv-commits q-mt-xl">
        <div class="cc-title">
          <q-icon name="verified" size="18px" color="positive" />
          Conventional Commits — El estándar de la industria
        </div>
        <TextBlock>
          Usar un prefijo estandarizado en tus mensajes hace el historial legible y permite
          generar changelogs automáticos. En proyectos de robótica profesional es obligatorio:
        </TextBlock>
        <div class="cc-types q-mt-md">
          <div v-for="type in conventionalTypes" :key="type.prefix" class="cct-item"
            :style="{ '--cct-color': type.color }">
            <code class="cct-prefix">{{ type.prefix }}</code>
            <div>
              <div class="cct-label">{{ type.label }}</div>
              <div class="cct-example">{{ type.example }}</div>
            </div>
          </div>
        </div>
        <div class="cc-rule q-mt-lg">
          <q-icon name="lightbulb" size="16px" color="warning" />
          <strong>Formato: </strong>
          <code>tipo(scope): descripción imperativa corta (&lt;50 chars)</code>
          <span class="ccr-ex">Ej: <code>feat(lidar): add distance filter for obstacle detection</code></span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 HEAD
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        HEAD — El Puntero Mágico
      </SectionTitle>

      <TextBlock>
        <code>HEAD</code> es simplemente un archivo que le dice a Git <em>"en qué commit estás parado
        ahora mismo"</em>. Normalmente apunta a una rama, y la rama apunta al último commit:
      </TextBlock>

      <div class="head-visual q-mt-lg">
        <div class="hv-chain">
          <div v-for="(item, idx) in headChain" :key="item.label" class="hvc-item">
            <div class="hvc-node" :style="{ '--hvc-color': item.color }">
              <code class="hvc-label">{{ item.label }}</code>
              <span class="hvc-sub">{{ item.sub }}</span>
            </div>
            <div v-if="idx < headChain.length - 1" class="hvc-arrow">
              <q-icon name="arrow_forward" size="18px" style="color:var(--text-muted)" />
            </div>
          </div>
        </div>
        <div class="hv-note">
          <q-icon name="info" size="14px" color="info" />
          Con <code>git checkout feature/lidar</code>, HEAD pasa a apuntar a esa rama.
        </div>
      </div>

      <div class="head-states q-mt-lg">
        <div v-for="state in headStates" :key="state.name" class="hs-card"
          :style="{ '--hs-color': state.color }">
          <div class="hsc-header">
            <q-icon :name="state.icon" size="18px" :style="{ color: state.color }" />
            <span class="hsc-name">{{ state.name }}</span>
          </div>
          <p class="hsc-desc">{{ state.desc }}</p>
          <code class="hsc-cmd">{{ state.cmd }}</code>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 BRANCHES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Branches — Universos Paralelos
      </SectionTitle>

      <TextBlock>
        Una rama es simplemente un puntero a un commit. Crear una rama es instantáneo —
        Git solo crea un archivo con el nombre. Puedes tener decenas de ramas sin overhead.
      </TextBlock>

      <!-- SVG branch diagram -->
      <div class="branch-diagram q-mt-lg">
        <svg viewBox="0 0 680 220" class="branch-svg" aria-label="Diagrama de branches Git">
          <!-- main branch line -->
          <path d="M 40 160 L 640 160" stroke="#4ade80" stroke-width="3.5" fill="none" />
          <text x="48" y="148" fill="#4ade80" font-family="monospace" font-weight="bold" font-size="14">main</text>

          <!-- Main commits -->
          <circle cx="80"  cy="160" r="9" fill="#4ade80" />
          <circle cx="180" cy="160" r="9" fill="#4ade80" />
          <circle cx="580" cy="160" r="9" fill="#4ade80" />

          <!-- feature/lidar branch -->
          <path d="M 180 160 C 220 160 220 80 260 80 L 440 80 C 480 80 480 130 520 140"
            stroke="#c084fc" stroke-width="3.5" fill="none" stroke-dasharray="9,5" />
          <text x="268" y="67" fill="#c084fc" font-family="monospace" font-weight="bold" font-size="13">feature/lidar</text>
          <circle cx="300" cy="80" r="8" fill="#c084fc" />
          <circle cx="380" cy="80" r="8" fill="#c084fc" />

          <!-- Merge point -->
          <circle cx="580" cy="160" r="14" fill="none" stroke="#fbbf24" stroke-width="3" />
          <text x="540" y="195" fill="#fbbf24" font-family="monospace" font-size="12">merge</text>

          <!-- bugfix/motor branch (below) -->
          <path d="M 180 160 C 200 160 200 200 220 200 L 340 200"
            stroke="#f87171" stroke-width="3.5" fill="none" stroke-dasharray="9,5" />
          <text x="225" y="218" fill="#f87171" font-family="monospace" font-weight="bold" font-size="13">bugfix/motor</text>
          <circle cx="280" cy="200" r="8" fill="#f87171" />

          <!-- HEAD label -->
          <rect x="600" y="140" width="50" height="22" rx="6" fill="none" stroke="#fbbf24" stroke-width="2" />
          <text x="606" y="156" fill="#fbbf24" font-family="monospace" font-size="12" font-weight="bold">HEAD</text>
        </svg>
      </div>

      <div class="branch-types q-mt-lg">
        <div v-for="bt in branchTypes" :key="bt.name" class="bt-card"
          :style="{ '--bt-color': bt.color }">
          <div class="btc-header">
            <q-icon :name="bt.icon" size="20px" :style="{ color: bt.color }" />
            <code class="btc-name">{{ bt.name }}</code>
          </div>
          <p class="btc-desc">{{ bt.desc }}</p>
          <div class="btc-rule">{{ bt.rule }}</div>
        </div>
      </div>

      <CodeBlock title="Comandos de branches" lang="bash"
        :content="branchCmdsCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         06 GIT vs GITHUB
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Git ≠ GitHub
      </SectionTitle>

      <TextBlock>
        El error más común: confundir Git con GitHub. Son herramientas completamente distintas
        que se complementan. Puedes usar Git sin GitHub (offline), pero no GitHub sin Git:
      </TextBlock>

      <div class="vs-grid q-mt-lg">
        <div v-for="tool in vsTools" :key="tool.name" class="vs-card"
          :style="{ '--vs-color': tool.color }">
          <div class="vsc-header">
            <div class="vsc-icon">
              <q-icon :name="tool.icon" size="32px" :style="{ color: tool.color }" />
            </div>
            <div class="vsc-name">{{ tool.name }}</div>
            <div class="vsc-sub">{{ tool.sub }}</div>
          </div>
          <div class="vsc-points">
            <div v-for="p in tool.points" :key="p" class="vscp-item">
              <q-icon name="check_circle" size="14px" :style="{ color: tool.color }" />
              <span>{{ p }}</span>
            </div>
          </div>
          <div v-if="tool.alts" class="vsc-alts">
            <strong>Alternativas:</strong> {{ tool.alts }}
          </div>
        </div>
      </div>

      <!-- Sync visual -->
      <div class="sync-visual q-mt-xl">
        <div class="sync-node sync-local">
          <q-icon name="computer" size="36px" style="color:var(--text-secondary)" />
          <div class="sn-label">Tu PC</div>
          <div class="sn-sub">Repositorio local (Git)</div>
        </div>
        <div class="sync-arrows">
          <div v-for="cmd in syncCmds" :key="cmd.cmd" class="sya-item"
            :style="{ '--sya-color': cmd.color }">
            <code class="sya-cmd">{{ cmd.cmd }}</code>
            <q-icon :name="cmd.arrow" size="20px" :style="{ color: cmd.color }" />
            <span class="sya-desc">{{ cmd.desc }}</span>
          </div>
        </div>
        <div class="sync-node sync-remote">
          <q-icon name="cloud_circle" size="36px" style="color:var(--text-muted)" />
          <div class="sn-label">GitHub</div>
          <div class="sn-sub">Repositorio remoto</div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         07 FLUJO DE TRABAJO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">07</span>
        Flujo de Trabajo Diario
      </SectionTitle>

      <TextBlock>
        Este es el flujo que repetirás cientos de veces en proyectos de robótica.
        Memorízalo como músculo — con el tiempo será automático:
      </TextBlock>

      <div class="workflow-list q-mt-lg">
        <div v-for="step in workflowSteps" :key="step.num" class="wf-step">
          <div class="wfs-num" :style="{ background: step.color }">{{ step.num }}</div>
          <div class="wfs-content">
            <div class="wfs-title" :style="{ color: step.color }">{{ step.title }}</div>
            <p class="wfs-desc">{{ step.desc }}</p>
            <CodeBlock :hide-header="true" lang="bash" :content="step.code" :copyable="true" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         08 COMANDOS ESENCIALES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">08</span>
        Comandos Esenciales de Consulta
      </SectionTitle>

      <TextBlock>
        Más allá de <code>add/commit/push</code>, estos comandos te permiten
        inspeccionar el estado de tu repositorio y deshacer cambios con seguridad:
      </TextBlock>

      <div class="cmd-table q-mt-lg">
        <div class="cmt-header">
          <div class="cmt-cell">Comando</div>
          <div class="cmt-cell">Qué hace</div>
          <div class="cmt-cell">Cuándo usarlo</div>
        </div>
        <div v-for="cmd in essentialCmds" :key="cmd.cmd" class="cmt-row"
          :style="{ '--cmt-color': cmd.color }">
          <div class="cmt-cell">
            <code class="cmt-cmd">{{ cmd.cmd }}</code>
          </div>
          <div class="cmt-cell cmt-desc">{{ cmd.desc }}</div>
          <div class="cmt-cell cmt-when">{{ cmd.when }}</div>
        </div>
      </div>

      <CodeBlock title="git log — visualizar el historial" lang="bash"
        :content="gitLogCode" :copyable="true" class="q-mt-xl" />
    </div>

    <!-- ══════════════════════════════════════════
         09 BEST PRACTICES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">09</span>
        Best Practices — Código Profesional
      </SectionTitle>

      <div class="practices-grid q-mt-lg">
        <div v-for="p in bestPractices" :key="p.title" class="prc-card"
          :style="{ '--prc-color': p.color }">
          <div class="prcc-icon">
            <q-icon :name="p.icon" size="26px" :style="{ color: p.color }" />
          </div>
          <div class="prcc-title">{{ p.title }}</div>
          <div class="prcc-body">
            <div v-for="item in p.items" :key="item.text" class="prci-item">
              <q-icon :name="item.ok ? 'check_circle' : 'cancel'" size="14px"
                :style="{ color: item.ok ? '#4ade80' : '#f87171' }" />
              <code>{{ item.text }}</code>
            </div>
            <p v-if="p.note" class="prci-note">{{ p.note }}</p>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes en Git</SectionTitle>

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
              <strong>Cómo se ve:</strong> {{ err.cause }}
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
      <SectionTitle>Reto — Tu Primer Repositorio ROS 2</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Inicializa y estructura un workspace ROS 2 con Git</div>
            <div class="challenge-subtitle">
              Desde cero hasta un historial limpio con branches correctas
            </div>
          </div>
          <div class="challenge-badge">60 min</div>
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

        <CodeBlock title="Comandos base para empezar" lang="bash"
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
      <TextBlock>Git para principiantes — conceptos fundamentales:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="Git Fundamentals para Robótica" frameborder="0"
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
      <SectionTitle>Resumen — Conceptos Clave</SectionTitle>
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
          <q-icon name="commit" size="40px" color="primary" />
        </div>
        <h2 class="fca-title">¡Conceptos dominados!</h2>
        <p class="fca-sub">
          Entiendes las 3 zonas, commits, HEAD, branches y el flujo de trabajo.
          Ahora vamos a profundizar en cómo escribir commits perfectos con buenas prácticas.
        </p>
        <div class="fca-actions">
          <q-btn color="primary" unelevated rounded size="lg" padding="14px 40px"
            to="/modulo-3/02commitsPage"
            icon="arrow_forward" label="Commits y Staging Area"
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

const gitConfigCode = [
  '# Tu nombre (aparece en cada commit)',
  'git config --global user.name "Alexander García"',
  '',
  '# Tu email (debe coincidir con GitHub)',
  'git config --global user.email "alex@robot.com"',
  '',
  '# Rama por defecto se llama "main" (no "master")',
  'git config --global init.defaultBranch main',
  '',
  '# Editor para mensajes de commit',
  'git config --global core.editor nano',
  '',
  '# Ver toda la configuración',
  'git config --global --list',
].join('\n');

const branchCmdsCode = [
  '# Crear rama nueva',
  'git branch feature/lidar-driver',
  '',
  '# Cambiar a la rama (modo clásico)',
  'git checkout feature/lidar-driver',
  '',
  '# Crear Y cambiar en un solo comando (preferido)',
  'git checkout -b feature/lidar-driver',
  '# Equivalente moderno (Git 2.23+)',
  'git switch -c feature/lidar-driver',
  '',
  '# Ver todas las ramas (local + remota)',
  'git branch -a',
  '',
  '# Fusionar rama a main',
  'git checkout main',
  'git merge feature/lidar-driver',
  '',
  '# Eliminar rama (tras merge exitoso)',
  'git branch -d feature/lidar-driver',
  '',
  '# Eliminar rama remota',
  'git push origin --delete feature/lidar-driver',
].join('\n');

const gitLogCode = [
  '# Historial completo',
  'git log',
  '',
  '# Vista compacta — una línea por commit',
  'git log --oneline',
  '',
  '# Vista compacta con gráfico de branches',
  'git log --oneline --graph --all',
  '',
  '# Filtrar por autor',
  'git log --author="Alexander" --oneline',
  '',
  '# Ver qué cambió en el último commit',
  'git show HEAD',
  '',
  '# Ver diferencias sin stagear',
  'git diff',
  '',
  '# Ver diferencias en staging (lo que se va a commitear)',
  'git diff --staged',
].join('\n');

const challengeCode = [
  '# 1. Configura tu identidad',
  'git config --global user.name "Tu Nombre"',
  'git config --global user.email "tu@email.com"',
  '',
  '# 2. Crea el workspace ROS 2',
  'mkdir ~/ros2_ws && cd ~/ros2_ws',
  'mkdir -p src',
  '',
  '# 3. Inicializa Git',
  'git init',
  '',
  '# 4. Crea .gitignore (pegar el contenido adecuado)',
  'nano .gitignore',
  '',
  '# 5. Primer commit',
  'git add .gitignore',
  'git commit -m "chore: add .gitignore for ROS 2 workspace"',
  '',
  '# 6. Crea tu primer paquete ROS 2',
  'cd src',
  'ros2 pkg create --build-type ament_python mi_robot',
  'cd ..',
  '',
  '# 7. Comittea con un buen mensaje',
  'git add src/mi_robot/',
  'git commit -m "feat(mi_robot): create initial ROS 2 Python package"',
  '',
  '# 8. Crea una rama para una nueva feature',
  'git checkout -b feature/add-lidar',
  '# ... hacer cambios ...',
  'git add .',
  'git commit -m "feat(lidar): add LIDAR subscriber node"',
  '',
  '# 9. Fusiona a main',
  'git checkout main',
  'git merge feature/add-lidar',
].join('\n');

// ═══════════════════════════════════════════════════════════════
// DATA ARRAYS
// ═══════════════════════════════════════════════════════════════

const facts = [
  { icon: '🕐', label: 'Git guarda SNAPSHOTS completos del proyecto — no solo los cambios' },
  { icon: '⚡', label: 'Crear un branch es instantáneo — solo crea un puntero de 41 bytes' },
  { icon: '🌍', label: '+96% de los proyectos ROS 2 en GitHub usan Git como control de versiones' },
];

const configItems = [
  { key: 'user.name',  color: '#60a5fa', desc: 'Nombre que aparece en cada commit',              example: '"Alexander García"' },
  { key: 'user.email', color: '#4ade80', desc: 'Email (debe coincidir con tu cuenta de GitHub)', example: '"alex@robot.com"' },
  { key: 'init.defaultBranch', color: '#c084fc', desc: 'Nombre de la rama principal', example: '"main"' },
  { key: 'core.editor', color: '#fbbf24', desc: 'Editor para mensajes de commit largos', example: '"nano"' },
];

const zones = [
  {
    name: 'Working Directory',
    icon: 'edit_document',
    desc: 'Tu mesa de trabajo — aquí editas archivos',
    color: '#f87171',
    files: [
      { name: 'robot_controller.py', status: 'M' },
      { name: 'lidar_driver.py',     status: '?' },
    ],
    action: 'git add <archivo>',
  },
  {
    name: 'Staging Area',
    icon: 'inventory_2',
    desc: 'Lista de cambios preparados para el commit',
    color: '#fbbf24',
    files: [
      { name: 'robot_controller.py', status: 'staged' },
      { name: 'lidar_driver.py',     status: 'staged' },
    ],
    action: 'git commit -m "msg"',
  },
  {
    name: 'Repository',
    icon: 'history',
    desc: 'Historial permanente — los commits guardados',
    color: '#4ade80',
    files: [
      { name: 'a1b2c3d — Add LIDAR',  status: '✓' },
      { name: '9f8e7d6 — Fix motor',  status: '✓' },
    ],
    action: null,
  },
];

const statusLegend = [
  { label: 'Verde (staged): listo para commitear',        color: '#4ade80' },
  { label: 'Amarillo (modified): cambiado, sin stage',   color: '#fbbf24' },
  { label: 'Rojo (untracked): nuevo, Git no lo conoce',  color: '#f87171' },
];

const commitFields = [
  { name: 'Hash SHA-1',      icon: 'fingerprint',    color: '#c084fc', desc: 'ID único e irrepetible de 40 hexadecimales. Los primeros 7 son suficientes para identificarlo.' },
  { name: 'Mensaje',         icon: 'message',         color: '#60a5fa', desc: 'Imperativo, máx 50 chars en línea 1. Explica QUÉ y POR QUÉ, no cómo. Usa Conventional Commits.' },
  { name: 'Autor + Fecha',   icon: 'person',          color: '#fbbf24', desc: 'Quién hizo el commit y cuándo. Vienen de tu git config user.name y user.email.' },
  { name: 'Parent commit',   icon: 'account_tree',    color: '#4ade80', desc: 'Puntero al commit anterior. Esta cadena de punteros es lo que permite viajar en el tiempo.' },
];

const conventionalTypes = [
  { prefix: 'feat:',     color: '#4ade80', label: 'Nueva funcionalidad',       example: 'feat(lidar): add distance filter for obstacle detection' },
  { prefix: 'fix:',      color: '#f87171', label: 'Corrección de bug',          example: 'fix(motor): correct speed calculation for ramp-up' },
  { prefix: 'docs:',     color: '#60a5fa', label: 'Solo documentación',         example: 'docs(readme): add Nav2 configuration guide' },
  { prefix: 'refactor:', color: '#c084fc', label: 'Refactorización sin cambio funcional', example: 'refactor(controller): extract PID logic to class' },
  { prefix: 'chore:',    color: '#94a3b8', label: 'Mantenimiento/configuración', example: 'chore: update Nav2 params for TurtleBot3' },
  { prefix: 'test:',     color: '#fbbf24', label: 'Agregar o corregir tests',    example: 'test(lidar): add unit tests for filter function' },
];

const headChain = [
  { label: 'HEAD',       sub: 'archivo: .git/HEAD', color: '#fbbf24' },
  { label: 'main',       sub: 'apunta al último commit', color: '#4ade80' },
  { label: 'a1b2c3d',   sub: 'el commit actual', color: '#60a5fa' },
];

const headStates = [
  {
    name: 'HEAD normal (attached)',
    icon: 'link',
    color: '#4ade80',
    desc: 'HEAD apunta a una rama, la rama apunta a un commit. Todo nuevo commit avanza la rama.',
    cmd: '$ git status → On branch main',
  },
  {
    name: 'Detached HEAD',
    icon: 'link_off',
    color: '#f87171',
    desc: 'HEAD apunta directamente a un commit, no a una rama. Los commits nuevos se pierden si no creas una rama.',
    cmd: '$ git checkout a1b2c3d → HEAD detached at a1b2c3d',
  },
];

const branchTypes = [
  { name: 'main',      icon: 'verified',    color: '#4ade80', desc: 'Código estable y funcional. Siempre debe compilar.',   rule: 'Nunca commits directos — solo vía merge/PR' },
  { name: 'feature/*', icon: 'science',     color: '#c084fc', desc: 'Nuevas funcionalidades. Puedes romper todo aquí.',     rule: 'Sale de main, vuelve a main via merge' },
  { name: 'bugfix/*',  icon: 'bug_report',  color: '#fbbf24', desc: 'Correcciones de bugs. Se arregla y se fusiona.',       rule: 'Sale de main, pequeña y corta vida' },
  { name: 'hotfix/*',  icon: 'emergency',   color: '#f87171', desc: 'Emergencias en producción. Máxima prioridad.',         rule: 'Sale de main, merge inmediato tras fix' },
];

const vsTools = [
  {
    name: 'Git',
    sub: 'El motor (local)',
    icon: 'terminal',
    color: '#f97316',
    points: [
      'Software instalado en tu PC',
      'Funciona 100% sin internet',
      'Open source — creado por Linus Torvalds (2005)',
      'Guarda el historial en .git/ localmente',
    ],
    alts: null,
  },
  {
    name: 'GitHub',
    sub: 'La nube (remoto)',
    icon: 'cloud',
    color: '#60a5fa',
    points: [
      'Servicio web — el repositorio vive en sus servidores',
      'Requiere internet para push/pull/clone',
      'Agrega: Pull Requests, Issues, Actions (CI/CD)',
      'Propiedad de Microsoft desde 2018',
    ],
    alts: 'GitLab · Bitbucket · Gitea (self-hosted)',
  },
];

const syncCmds = [
  { cmd: 'git push', arrow: 'arrow_forward', color: '#60a5fa', desc: 'Sube commits locales → GitHub' },
  { cmd: 'git pull', arrow: 'arrow_back',    color: '#4ade80', desc: 'Baja commits remotos → local' },
  { cmd: 'git clone', arrow: 'arrow_back',   color: '#c084fc', desc: 'Descarga repositorio completo' },
  { cmd: 'git fetch', arrow: 'arrow_back',   color: '#fbbf24', desc: 'Baja cambios sin fusionarlos' },
];

const workflowSteps = [
  {
    num: 1,
    title: 'Clonar o Inicializar',
    desc: 'Si el proyecto ya existe en GitHub, clónalo. Si es nuevo, inicializa desde cero.',
    color: '#4ade80',
    code: [
      '# Proyecto nuevo',
      'mkdir ~/ros2_ws && cd ~/ros2_ws',
      'git init',
      '',
      '# Proyecto existente en GitHub',
      'git clone https://github.com/user/robot_ws.git',
      'cd robot_ws',
    ].join('\n'),
  },
  {
    num: 2,
    title: 'Crear Rama para la Feature',
    desc: 'Nunca trabajes en main directamente. Siempre en una rama con nombre descriptivo.',
    color: '#60a5fa',
    code: [
      '# Asegúrate de estar en main actualizado',
      'git checkout main',
      'git pull',
      '',
      '# Crear y cambiar a la nueva rama',
      'git checkout -b feature/add-camera-driver',
    ].join('\n'),
  },
  {
    num: 3,
    title: 'Hacer Cambios y Commits',
    desc: 'Edita archivos, agrega solo lo necesario al staging, y commitea con mensaje claro.',
    color: '#fbbf24',
    code: [
      '# Ver qué cambió',
      'git status',
      'git diff',
      '',
      '# Agregar archivos específicos (no git add -A)',
      'git add src/camera_driver.py config/camera_params.yaml',
      '',
      '# Commit con Conventional Commits',
      'git commit -m "feat(camera): add USB camera driver with ROS 2 publisher"',
    ].join('\n'),
  },
  {
    num: 4,
    title: 'Subir a GitHub',
    desc: 'Primera vez crea la rama remota. Después solo git push.',
    color: '#c084fc',
    code: [
      '# Primera vez (crea la rama en GitHub)',
      'git push -u origin feature/add-camera-driver',
      '',
      '# Siguientes veces',
      'git push',
    ].join('\n'),
  },
  {
    num: 5,
    title: 'Merge a Main',
    desc: 'Cuando la feature está lista, vuelve a main y fusiona.',
    color: '#f87171',
    code: [
      '# Volver a main y actualizar',
      'git checkout main',
      'git pull',
      '',
      '# Fusionar la feature',
      'git merge feature/add-camera-driver',
      '',
      '# Subir a GitHub y limpiar',
      'git push',
      'git branch -d feature/add-camera-driver',
    ].join('\n'),
  },
];

const essentialCmds = [
  { cmd: 'git status',       color: '#4ade80', desc: 'Estado actual: qué cambió, qué está staged, qué es nuevo',     when: 'Antes de cada add/commit' },
  { cmd: 'git diff',         color: '#60a5fa', desc: 'Diferencias en Working Directory (no staged)',                   when: 'Ver qué cambiaste antes de stagear' },
  { cmd: 'git diff --staged',color: '#60a5fa', desc: 'Diferencias en Staging Area (lo que se va a commitear)',        when: 'Revisar antes de hacer commit' },
  { cmd: 'git log --oneline',color: '#fbbf24', desc: 'Historial compacto: hash + mensaje, una línea por commit',      when: 'Ver el historial rápidamente' },
  { cmd: 'git show HEAD',    color: '#c084fc', desc: 'Detalle completo del último commit (cambios incluidos)',         when: 'Ver qué hizo el commit anterior' },
  { cmd: 'git stash',        color: '#f97316', desc: 'Guarda cambios sin commitear para cambiar de rama',             when: 'Cambiar de rama con trabajo en progreso' },
  { cmd: 'git stash pop',    color: '#f97316', desc: 'Recupera cambios del stash',                                     when: 'Volver a la feature después del cambio' },
  { cmd: 'git reset HEAD~1', color: '#f87171', desc: 'Deshace el último commit (mantiene cambios en working)',        when: 'Deshacer commit sin perder el código' },
];

const bestPractices = [
  {
    title: 'Mensajes de Commit Claros',
    icon: 'message',
    color: '#60a5fa',
    note: 'El mensaje es para el "tú del futuro" — ponlo en inglés técnico.',
    items: [
      { ok: true,  text: 'feat(nav2): add waypoint interpolation for smooth paths' },
      { ok: true,  text: 'fix(motor): correct PID gains for 12V motors' },
      { ok: false, text: 'update' },
      { ok: false, text: 'fix stuff' },
      { ok: false, text: 'asdfasdf' },
    ],
  },
  {
    title: 'Commits Pequeños y Frecuentes',
    icon: 'commit',
    color: '#4ade80',
    note: 'Si no puedes describir el commit en una línea, es demasiado grande.',
    items: [
      { ok: true,  text: '1 commit = 1 cambio lógico atómico' },
      { ok: true,  text: 'git add archivos específicos' },
      { ok: false, text: 'git add -A (todo de golpe)' },
      { ok: false, text: '1 commit con 50 archivos cambiados' },
    ],
  },
  {
    title: 'Nunca Commitear a Main Directamente',
    icon: 'account_tree',
    color: '#c084fc',
    note: 'Main es sagrado: solo código probado y revisado llega ahí.',
    items: [
      { ok: true,  text: 'feature/* → PR → review → merge main' },
      { ok: true,  text: 'bugfix/* → fix → test → merge main' },
      { ok: false, text: 'git checkout main && git commit' },
      { ok: false, text: 'git push origin main (cambios sin revisar)' },
    ],
  },
  {
    title: 'Pull Antes de Push',
    icon: 'sync',
    color: '#fbbf24',
    note: 'Evita la mayoría de los conflictos con este hábito.',
    items: [
      { ok: true,  text: 'git pull → resolver conflictos → push' },
      { ok: true,  text: 'git fetch → git status → git merge' },
      { ok: false, text: 'git push sin pull previo' },
      { ok: false, text: 'git push --force (destruye historial remoto)' },
    ],
  },
];

const commonErrors = reactive([
  {
    type: 'Commiteaste en main por error',
    summary: 'Hiciste cambios directamente en main en lugar de en una feature branch',
    color: '#f87171',
    cause: 'git log --oneline muestra el commit en main, pero debería estar en feature/*',
    code: [
      '# ❌ Situación: cometiste en main por error',
      '# git log --oneline',
      '# a1b2c3d (HEAD -> main) feat: mi nueva feature',
      '',
      '# ✅ Solución: crea la rama y mueve el commit',
      '# 1. Crea la rama desde el commit actual',
      'git branch feature/mi-feature',
      '',
      '# 2. Regresa main al commit anterior',
      'git reset HEAD~1',
      '',
      '# 3. El commit ahora solo está en feature/mi-feature',
      'git checkout feature/mi-feature',
    ].join('\n'),
    fix: 'git branch para crear la rama con el commit, git reset HEAD~1 para quitar el commit de main.',
    open: false,
  },
  {
    type: 'Mensaje de commit terrible',
    summary: '"update", "fix", "wip", "asdfasdf" — imposible saber qué cambió',
    color: '#fbbf24',
    cause: 'git log muestra mensajes crípticos — en 3 meses nadie sabe qué hizo ese commit',
    code: [
      '# ❌ Mensajes típicos de principiante',
      '# a1b2c3d update',
      '# 9f8e7d6 fix',
      '# 4c3d2e1 wip',
      '# 7f6e5d4 asdf',
      '',
      '# Si el commit NO fue pusheado, puedes editar el mensaje:',
      'git commit --amend -m "feat(lidar): add obstacle detection threshold parameter"',
      '',
      '# Si ya fue pusheado, crea un commit de corrección:',
      'git commit -m "docs: clarify previous commit — was lidar threshold config"',
    ].join('\n'),
    fix: 'git commit --amend para cambiar el último mensaje (antes de push). Sigue Conventional Commits: tipo(scope): descripción.',
    open: false,
  },
  {
    type: 'Subiste credenciales o archivos sensibles',
    summary: 'passwords, API keys, .env files, archivos de 100MB+ en el repositorio',
    color: '#c084fc',
    cause: 'git log y GitHub muestran las credenciales — cualquier persona con acceso puede verlas',
    code: [
      '# Prevención — .gitignore ANTES del primer commit',
      'echo ".env" >> .gitignore',
      'echo "*.key" >> .gitignore',
      'echo "build/" >> .gitignore',
      'echo "install/" >> .gitignore',
      'echo "log/" >> .gitignore',
      'git add .gitignore',
      'git commit -m "chore: add gitignore for ROS 2 workspace"',
      '',
      '# Si ya subiste algo sensible → cambia las credenciales INMEDIATAMENTE',
      '# Luego usa BFG Repo Cleaner o git filter-repo para limpiar el historial',
      '# Ver: https://docs.github.com/en/authentication/keeping-your-account-and-data-secure',
    ].join('\n'),
    fix: 'Crea .gitignore antes del primer commit. Si ya lo subiste: cambia las credenciales ahora, luego limpia el historial con git filter-repo.',
    open: false,
  },
  {
    type: 'Merge conflict sin resolver correctamente',
    summary: 'El archivo tiene marcadores <<<<<<< sin resolver y lo commiteaste así',
    color: '#f97316',
    cause: 'El código tiene <<<<<<< HEAD, =======, >>>>>>> literalmente en el archivo Python/YAML',
    code: [
      '# ❌ El archivo quedó así (sin resolver)',
      '# <<<<<<< HEAD',
      '# max_vel_x: 0.26',
      '# =======',
      '# max_vel_x: 0.50',
      '# >>>>>>> feature/fast-mode',
      '',
      '# ✅ Proceso correcto para resolver conflict',
      '# 1. Ver qué archivos tienen conflictos',
      'git status',
      '',
      '# 2. Editar el archivo — eliminar marcadores y dejar el código correcto',
      'nano config/nav2_params.yaml  # ← editar manualmente',
      '',
      '# 3. Marcar como resuelto',
      'git add config/nav2_params.yaml',
      '',
      '# 4. Completar el merge',
      'git commit -m "merge: resolve conflict in nav2_params max_vel_x"',
    ].join('\n'),
    fix: 'Edita el archivo eliminando <<<<<<< / ======= / >>>>>>>, deja el código correcto, git add, git commit. Usa VS Code que colorea los conflictos.',
    open: false,
  },
  {
    type: 'git push --force destruyó el historial remoto',
    summary: '--force reescribe el historial en GitHub — borra commits de tus compañeros',
    color: '#ef4444',
    cause: 'git push --force funcionó pero ahora tus compañeros tienen conflictos o perdieron commits',
    code: [
      '# ❌ NUNCA en main o ramas compartidas',
      'git push --force  # ← destruye el historial remoto',
      '',
      '# ✅ Si necesitas deshacer un push (solo en TU rama personal)',
      '# Opción 1: Revert (crea commit nuevo que deshace)',
      'git revert HEAD',
      'git push',
      '',
      '# Opción 2: --force-with-lease (más seguro que --force)',
      '# Solo funciona si nadie más pushó después de ti',
      'git push --force-with-lease',
      '',
      '# Regla: si dudas, usa git revert — es siempre seguro',
    ].join('\n'),
    fix: 'Usa git revert en lugar de git push --force. Si necesitas reescribir historial, usa --force-with-lease y avisa al equipo primero.',
    open: false,
  },
]);

const challengeSteps = [
  { num: 1, color: '#4ade80', text: 'Instala y configura Git con tu nombre y email real' },
  { num: 2, color: '#60a5fa', text: 'Crea ~/ros2_ws/, inicializa Git y agrega un .gitignore para ROS 2 (build/, install/, log/)' },
  { num: 3, color: '#fbbf24', text: 'Haz el primer commit: "chore: add .gitignore for ROS 2 workspace"' },
  { num: 4, color: '#c084fc', text: 'Crea un paquete ROS 2 con ros2 pkg create y comittéalo con feat(paquete): ...' },
  { num: 5, color: '#f87171', text: 'Crea rama feature/add-params, agrega un params.yaml, commit y merge a main' },
  { num: 6, color: '#f97316', text: '(Bonus) Crea repo en GitHub, conéctalo como remote y haz push' },
];

const challengeHints = [
  'git log --oneline --graph al final debe mostrar 3+ commits con mensajes claros',
  'Para el .gitignore de ROS 2: build/, install/, log/, *.pyc, __pycache__/',
  'git config --global --list muestra toda la configuración actual',
  'git branch -a muestra ramas locales y remotas juntas',
  'Si haces algo mal, git reset HEAD~1 deshace el último commit (sin perder cambios)',
];

const summaryItems = [
  { cmd: 'git config',       desc: 'Configurar nombre/email/editor',   example: '--global user.name "Yo"', color: '#60a5fa' },
  { cmd: 'git init',         desc: 'Inicializar repositorio',           example: 'Crea carpeta .git/',       color: '#4ade80' },
  { cmd: 'git status',       desc: 'Ver estado actual',                 example: 'Working vs Staging',       color: '#4ade80' },
  { cmd: 'git add',          desc: 'Working → Staging',                 example: 'git add archivo.py',       color: '#fbbf24' },
  { cmd: 'git commit',       desc: 'Staging → Repository',             example: '-m "feat: descripción"',   color: '#fbbf24' },
  { cmd: 'git branch',       desc: 'Crear/listar ramas',               example: '-b feature/nueva',         color: '#c084fc' },
  { cmd: 'git merge',        desc: 'Fusionar ramas',                   example: 'git merge feature/x',      color: '#c084fc' },
  { cmd: 'git push/pull',    desc: 'Sincronizar con GitHub',            example: 'push sube, pull baja',     color: '#f87171' },
  { cmd: 'git log --oneline',desc: 'Ver historial compacto',           example: 'hash + mensaje',           color: '#94a3b8' },
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
   CONFIG CARDS
══════════════════════════════════════════ */
.config-cards { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; }
.config-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--cfg-color); border-radius: 12px;
  padding: 12px 14px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.cfc-key     { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--cfg-color); background: none; padding: 0; word-break: break-all; }
.cfc-desc    { font-size: .78rem; color: var(--text-secondary); line-height: 1.4; }
.cfc-example { font-size: .74rem; color: var(--text-muted); font-style: italic; }

/* ══════════════════════════════════════════
   ZONES GRID
══════════════════════════════════════════ */
.zones-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
.zone-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--zone-color); border-radius: 14px;
  display: flex; flex-direction: column; gap: 10px; padding: 1.25rem; min-width: 0;
  transition: transform .2s;
}
.zone-card:hover { transform: translateY(-3px); }
.zc-header { display: flex; align-items: center; gap: 10px; }
.zc-name   { font-size: .95rem; font-weight: 800; color: var(--text-primary); }
.zc-desc   { font-size: .82rem; color: var(--text-muted); }
.zc-files  { display: flex; flex-direction: column; gap: 6px; flex: 1; }
.zf-item   { display: flex; align-items: center; gap: 7px; background: var(--bg-surface-hover); border-radius: 7px; padding: 6px 10px; }
.zf-name   { font-family: 'Fira Code', monospace; font-size: .78rem; color: var(--text-secondary); flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.zf-status { font-size: .78rem; font-weight: 800; flex-shrink: 0; }
.zc-action { border: 1px solid; border-radius: 8px; padding: 8px 12px; text-align: center; }
.zc-action code { background: none; padding: 0; font-size: .82rem; font-weight: 700; }

/* git status terminal */
.status-visual { }
.sv-title { font-size: .88rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 8px; }
.sv-title code { background: none; padding: 0; }
.status-terminal {
  background: var(--bg-deep, #0d1117); border: 1px solid var(--border-subtle);
  border-radius: 12px; padding: 1.25rem; font-family: 'Fira Code', monospace;
  font-size: .84rem; display: flex; flex-direction: column; gap: 4px;
}
.st-line    { white-space: pre-wrap; word-break: break-all; }
.st-branch  { color: #c084fc; }
.st-head    { color: var(--text-muted); }
.st-section { color: #fbbf24; margin-top: 6px; }
.st-staged  { color: #4ade80; }
.st-modified{ color: #fbbf24; }
.st-untracked{ color: #f87171; }
.sv-legend { display: flex; flex-wrap: wrap; gap: 14px; margin-top: 10px; }
.svl-item  { display: flex; align-items: center; gap: 7px; font-size: .8rem; color: var(--text-muted); }
.svl-dot   { width: 10px; height: 10px; border-radius: 50%; background: var(--svl-color); flex-shrink: 0; }

/* ══════════════════════════════════════════
   COMMIT VISUAL
══════════════════════════════════════════ */
.commit-visual { display: grid; grid-template-columns: 1fr 1fr; gap: 1.5rem; }
.cv-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; overflow: hidden;
}
.cvc-topbar {
  background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle);
  padding: 10px 14px; display: flex; align-items: center; gap: 8px; flex-wrap: wrap;
}
.cvc-hash { font-family: 'Fira Code', monospace; font-size: .8rem; color: #f97316; font-weight: 700; flex: 1; }
.cvc-ref  { font-size: .72rem; font-weight: 800; padding: 2px 8px; border-radius: 4px; }
.main-ref { background: rgba(74,222,128,.15); color: #4ade80; }
.head-ref { background: rgba(251,191,36,.15);  color: #fbbf24; }
.cvc-msg  { font-size: .9rem; font-weight: 700; color: var(--text-primary); padding: 12px 14px; border-bottom: 1px solid var(--border-subtle); }
.cvc-meta { padding: 10px 14px; display: flex; flex-direction: column; gap: 4px; border-bottom: 1px solid var(--border-subtle); }
.cvc-meta-row { display: flex; align-items: center; gap: 7px; font-size: .8rem; color: var(--text-muted); }
.cvc-diff { padding: 10px 14px; display: flex; flex-direction: column; gap: 6px; border-bottom: 1px solid var(--border-subtle); }
.cvd-item { display: flex; align-items: center; gap: 7px; font-size: .82rem; }
.cvd-item code { background: none; padding: 0; color: var(--text-secondary); }
.cvd-stat { margin-left: auto; font-size: .75rem; font-weight: 700; }
.cvd-added   .cvd-stat { color: #4ade80; }
.cvd-modified .cvd-stat { color: #fbbf24; }
.cvc-parent { padding: 8px 14px; font-size: .78rem; color: var(--text-muted); display: flex; align-items: center; gap: 5px; }
.cvc-parent code { background: none; padding: 0; }
.cv-fields { display: flex; flex-direction: column; gap: 10px; }
.cvf-item {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--cvf-color); border-radius: 10px; padding: 10px 14px;
}
.cvf-header { display: flex; align-items: center; gap: 8px; margin-bottom: 5px; }
.cvf-name   { font-size: .88rem; font-weight: 700; color: var(--text-primary); }
.cvf-desc   { font-size: .8rem; color: var(--text-secondary); line-height: 1.5; margin: 0; }

/* Conventional Commits */
.conv-commits { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.5rem; }
.cc-title { display: flex; align-items: center; gap: 8px; font-size: .95rem; font-weight: 700; color: var(--text-primary); margin-bottom: 12px; }
.cc-types { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
.cct-item {
  background: color-mix(in srgb, var(--cct-color) 6%, var(--bg-surface));
  border: 1px solid color-mix(in srgb, var(--cct-color) 20%, transparent);
  border-radius: 10px; padding: 10px 12px; display: flex; align-items: flex-start; gap: 10px;
}
.cct-prefix { font-family: 'Fira Code', monospace; font-size: .85rem; font-weight: 900; color: var(--cct-color); background: none; padding: 0; white-space: nowrap; flex-shrink: 0; padding-top: 2px; }
.cct-label  { font-size: .8rem; font-weight: 600; color: var(--text-secondary); }
.cct-example { font-family: 'Fira Code', monospace; font-size: .7rem; color: var(--text-muted); margin-top: 3px; word-break: break-all; }
.cc-rule { display: flex; align-items: center; gap: 8px; font-size: .85rem; color: var(--text-secondary); flex-wrap: wrap; margin-top: 4px; }
.cc-rule code { background: none; padding: 0; }
.ccr-ex { color: var(--text-muted); font-size: .8rem; }

/* ══════════════════════════════════════════
   HEAD VISUAL
══════════════════════════════════════════ */
.head-visual { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.hv-chain { display: flex; align-items: center; justify-content: center; gap: 8px; flex-wrap: wrap; margin-bottom: 14px; }
.hvc-item { display: flex; align-items: center; gap: 8px; }
.hvc-node {
  background: color-mix(in srgb, var(--hvc-color) 10%, var(--bg-surface));
  border: 1px solid color-mix(in srgb, var(--hvc-color) 30%, transparent);
  border-radius: 10px; padding: 8px 16px; text-align: center;
}
.hvc-label { font-family: 'Fira Code', monospace; font-size: .95rem; font-weight: 800; color: var(--hvc-color); background: none; padding: 0; display: block; }
.hvc-sub   { font-size: .7rem; color: var(--text-muted); display: block; margin-top: 2px; }
.hv-note { font-size: .82rem; color: var(--text-muted); display: flex; align-items: center; gap: 8px; }
.hv-note code { background: none; padding: 0; }
.head-states { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; margin-top: 14px; }
.hs-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--hs-color); border-radius: 12px; padding: 14px;
}
.hsc-header { display: flex; align-items: center; gap: 8px; margin-bottom: 7px; }
.hsc-name   { font-size: .9rem; font-weight: 700; color: var(--text-primary); }
.hsc-desc   { font-size: .82rem; color: var(--text-secondary); line-height: 1.5; margin: 0 0 8px; }
.hsc-cmd    { font-family: 'Fira Code', monospace; font-size: .75rem; color: var(--text-muted); background: var(--bg-surface-hover); padding: 5px 10px; border-radius: 6px; display: block; }

/* ══════════════════════════════════════════
   BRANCHES
══════════════════════════════════════════ */
.branch-diagram {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 14px; padding: 1.5rem;
}
.branch-svg { width: 100%; height: auto; }
.branch-types { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; }
.bt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--bt-color); border-radius: 12px; padding: 14px; min-width: 0;
}
.btc-header { display: flex; align-items: center; gap: 8px; margin-bottom: 6px; flex-wrap: wrap; }
.btc-name   { font-family: 'Fira Code', monospace; font-size: .85rem; font-weight: 800; color: var(--bt-color); background: none; padding: 0; }
.btc-desc   { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; margin: 0 0 8px; }
.btc-rule   { font-size: .74rem; color: var(--text-muted); border-top: 1px solid var(--border-subtle); padding-top: 7px; }

/* ══════════════════════════════════════════
   GIT vs GITHUB
══════════════════════════════════════════ */
.vs-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.vs-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--vs-color); border-radius: 14px;
  display: flex; flex-direction: column; gap: 14px; padding: 1.5rem; min-width: 0;
}
.vsc-header { display: flex; flex-direction: column; align-items: center; gap: 8px; text-align: center; }
.vsc-icon   { width: 56px; height: 56px; border-radius: 16px; display: flex; align-items: center; justify-content: center; background: color-mix(in srgb, var(--vs-color) 12%, var(--bg-surface)); }
.vsc-name   { font-size: 1.4rem; font-weight: 800; color: var(--text-primary); }
.vsc-sub    { font-size: .82rem; color: var(--text-muted); }
.vsc-points { display: flex; flex-direction: column; gap: 8px; flex: 1; }
.vscp-item  { display: flex; align-items: flex-start; gap: 8px; font-size: .88rem; color: var(--text-secondary); }
.vsc-alts   { font-size: .82rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 8px; padding: 8px 12px; }

/* Sync visual */
.sync-visual {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 2rem;
  display: grid; grid-template-columns: 1fr auto 1fr; gap: 2rem; align-items: center;
}
.sync-node { display: flex; flex-direction: column; align-items: center; gap: 8px; text-align: center; }
.sn-label  { font-size: 1rem; font-weight: 700; color: var(--text-primary); }
.sn-sub    { font-size: .78rem; color: var(--text-muted); }
.sync-arrows { display: flex; flex-direction: column; gap: 12px; }
.sya-item  { display: flex; align-items: center; gap: 10px; }
.sya-cmd   { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; color: var(--sya-color); background: color-mix(in srgb, var(--sya-color) 8%, var(--bg-surface)); border: 1px solid color-mix(in srgb, var(--sya-color) 20%, transparent); border-radius: 6px; padding: 3px 8px; min-width: 70px; text-align: center; }
.sya-desc  { font-size: .78rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   WORKFLOW
══════════════════════════════════════════ */
.workflow-list { display: flex; flex-direction: column; gap: 1.25rem; }
.wf-step { display: grid; grid-template-columns: 44px 1fr; gap: 14px; align-items: start; }
.wfs-num {
  width: 44px; height: 44px; border-radius: 50%; flex-shrink: 0;
  display: flex; align-items: center; justify-content: center;
  font-size: 1.2rem; font-weight: 800; color: #0d1117;
}
.wfs-content {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 12px; padding: 1.25rem; display: flex; flex-direction: column; gap: 10px; min-width: 0;
}
.wfs-title { font-size: 1rem; font-weight: 700; }
.wfs-desc  { font-size: .86rem; color: var(--text-secondary); margin: 0; }

/* ══════════════════════════════════════════
   CMD TABLE
══════════════════════════════════════════ */
.cmd-table { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.cmt-header { display: grid; grid-template-columns: 1fr 1.5fr 1.5fr; background: var(--bg-surface-solid); padding: 10px 14px; font-size: .82rem; font-weight: 700; color: var(--text-primary); border-bottom: 1px solid var(--border-subtle); }
.cmt-row { display: grid; grid-template-columns: 1fr 1.5fr 1.5fr; border-bottom: 1px solid var(--border-subtle); border-left: 3px solid var(--cmt-color); transition: background .15s; }
.cmt-row:last-child { border-bottom: none; }
.cmt-row:hover { background: var(--bg-surface-hover); }
.cmt-cell { padding: 9px 14px; font-size: .82rem; color: var(--text-secondary); display: flex; align-items: center; }
.cmt-cmd  { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; color: var(--cmt-color); background: none; padding: 0; }
.cmt-desc { color: var(--text-secondary); }
.cmt-when { color: var(--text-muted); font-size: .78rem; }

/* ══════════════════════════════════════════
   PRACTICES GRID
══════════════════════════════════════════ */
.practices-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.prc-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--prc-color); border-radius: 14px;
  display: flex; flex-direction: column; gap: 10px; padding: 1.25rem; min-width: 0;
}
.prcc-icon  { display: flex; justify-content: center; margin-bottom: 2px; }
.prcc-title { font-size: .95rem; font-weight: 700; color: var(--text-primary); text-align: center; }
.prcc-body  { display: flex; flex-direction: column; gap: 6px; }
.prci-item  { display: flex; align-items: baseline; gap: 8px; font-size: .82rem; }
.prci-item code { background: none; padding: 0; font-size: .8rem; color: var(--text-secondary); overflow-wrap: break-word; }
.prci-note  { font-size: .78rem; color: var(--text-muted); line-height: 1.5; border-top: 1px solid var(--border-subtle); padding-top: 8px; margin: 2px 0 0; }

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
.summary-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sc-color); border-radius: 12px; padding: 1rem 1.25rem;
  transition: all .25s;
}
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
.fca-icon   { width: 72px; height: 72px; background: rgba(96,165,250,.1); border-radius: 20px; display: flex; align-items: center; justify-content: center; }
.fca-title  { font-size: 1.5rem; font-weight: 800; color: var(--text-primary); margin: 0; }
.fca-sub    { font-size: .95rem; color: var(--text-secondary); max-width: 520px; line-height: 1.6; margin: 0; }
.fca-actions{ margin-top: .5rem; }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1100px) {
  .config-cards  { grid-template-columns: repeat(2, 1fr); }
  .branch-types  { grid-template-columns: repeat(2, 1fr); }
  .cc-types      { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 900px) {
  .zones-grid    { grid-template-columns: 1fr; }
  .commit-visual { grid-template-columns: 1fr; }
  .head-states   { grid-template-columns: 1fr; }
  .summary-grid  { grid-template-columns: repeat(2, 1fr); }
  .vs-grid       { grid-template-columns: 1fr; }
  .sync-visual   { grid-template-columns: 1fr; text-align: center; }
  .sya-item      { justify-content: center; }
  .cmt-header    { grid-template-columns: 1fr 1fr; }
  .cmt-row       { grid-template-columns: 1fr 1fr; }
  .cmt-when      { display: none; }
}
@media (max-width: 768px) {
  .practices-grid { grid-template-columns: 1fr; }
  .config-cards  { grid-template-columns: 1fr; }
  .branch-types  { grid-template-columns: 1fr; }
  .cc-types      { grid-template-columns: 1fr; }
  .hv-chain      { flex-direction: column; gap: 4px; }
  .hvc-arrow     { transform: rotate(90deg); }
  .wf-step       { grid-template-columns: 36px 1fr; }
  .cmt-header    { grid-template-columns: 1fr; }
  .cmt-row       { grid-template-columns: 1fr; }
  .cmt-cell:nth-child(2) { display: none; }
}
@media (max-width: 480px) {
  .summary-grid  { grid-template-columns: 1fr; }
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
}
</style>
