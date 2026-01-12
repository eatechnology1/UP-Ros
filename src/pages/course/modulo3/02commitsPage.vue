<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Los commits son el corazón de Git. Cada commit es una fotografía inmutable de tu proyecto en
      un momento específico. Dominar el arte de hacer buenos commits te convertirá en un
      desarrollador profesional.
      <br /><br />
      No se trata solo de "guardar". Se trata de crear un historial legible, navegable y útil.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué son importantes los commits?">
      <strong>Historial claro:</strong> Entender qué cambió y cuándo
      <br />
      <strong>Debugging:</strong> Encontrar cuándo se introdujo un bug
      <br />
      <strong>Colaboración:</strong> Que tu equipo entienda tus cambios
      <br />
      <strong>Reversión:</strong> Deshacer cambios específicos sin afectar otros
    </AlertBlock>

    <!-- EL RITUAL -->
    <div class="section-group">
      <SectionTitle>1. El Ritual de los 3 Pasos</SectionTitle>
      <TextBlock>
        Git no adivina qué quieres guardar. Tienes que decírselo explícitamente. Este ciclo se
        repite docenas de veces al día:
      </TextBlock>

      <div class="ritual-visual q-mt-md">
        <div class="ritual-step">
          <div class="step-icon check">
            <q-icon name="search" size="2rem" />
          </div>
          <div class="step-content">
            <div class="step-title">1. Check (Verificar)</div>
            <div class="step-desc">¿Qué he modificado?</div>
            <CodeBlock
              lang="bash"
              content="git status

# Output:
# modified:   src/motor_driver.py
# modified:   config/params.yaml
# untracked:  src/new_sensor.py"
              :copyable="true"
            />
          </div>
        </div>

        <div class="ritual-arrow">
          <q-icon name="arrow_downward" size="2rem" color="orange-4" />
        </div>

        <div class="ritual-step">
          <div class="step-icon stage">
            <q-icon name="add_box" size="2rem" />
          </div>
          <div class="step-content">
            <div class="step-title">2. Stage (Preparar)</div>
            <div class="step-desc">Elegir qué archivos van al commit</div>
            <CodeBlock
              lang="bash"
              content="# Agregar archivo específico
git add src/motor_driver.py

# Agregar todos los cambios
git add .

# Agregar interactivamente
git add -p"
              :copyable="true"
            />
          </div>
        </div>

        <div class="ritual-arrow">
          <q-icon name="arrow_downward" size="2rem" color="orange-4" />
        </div>

        <div class="ritual-step">
          <div class="step-icon commit">
            <q-icon name="save" size="2rem" />
          </div>
          <div class="step-content">
            <div class="step-title">3. Commit (Confirmar)</div>
            <div class="step-desc">Sellar y etiquetar la caja</div>
            <CodeBlock
              lang="bash"
              content="git commit -m 'Fix motor speed calculation'

# Output:
# [main a1b2c3d] Fix motor speed calculation
# 1 file changed, 12 insertions(+), 3 deletions(-)"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- MENSAJES DE COMMIT -->
    <div class="section-group">
      <SectionTitle>2. El Arte del Mensaje</SectionTitle>
      <TextBlock>
        Un buen mensaje de commit es como un buen comentario de código: explica el
        <strong>QUÉ</strong> y el <strong>POR QUÉ</strong>, no el CÓMO (eso ya lo hace el código).
      </TextBlock>

      <div class="message-comparison q-mt-md">
        <div class="message-card bad">
          <div class="message-header">
            <q-icon name="cancel" size="2rem" />
            <span>❌ Mensajes Vagos</span>
          </div>
          <div class="message-examples">
            <div class="message-item">
              <code>update</code>
              <span class="message-problem">¿Qué actualizaste?</span>
            </div>
            <div class="message-item">
              <code>fix stuff</code>
              <span class="message-problem">¿Qué arreglaste?</span>
            </div>
            <div class="message-item">
              <code>cambios</code>
              <span class="message-problem">Obvio que son cambios</span>
            </div>
            <div class="message-item">
              <code>asdfgh</code>
              <span class="message-problem">Ni siquiera intentaste</span>
            </div>
            <div class="message-item">
              <code>WIP</code>
              <span class="message-problem">Work In Progress no es un mensaje</span>
            </div>
          </div>
        </div>

        <div class="message-card good">
          <div class="message-header">
            <q-icon name="check_circle" size="2rem" />
            <span>✅ Mensajes Profesionales</span>
          </div>
          <div class="message-examples">
            <div class="message-item">
              <code>Fix motor speed calculation overflow</code>
              <span class="message-why">Específico y claro</span>
            </div>
            <div class="message-item">
              <code>Add LIDAR driver for TurtleBot3</code>
              <span class="message-why">Dice qué y para qué</span>
            </div>
            <div class="message-item">
              <code>Refactor navigation to use A* algorithm</code>
              <span class="message-why">Explica el cambio técnico</span>
            </div>
            <div class="message-item">
              <code>Update README with installation steps</code>
              <span class="message-why">Documenta la mejora</span>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Reglas de Oro para Mensajes</SectionTitle>
        <div class="rules-grid">
          <div class="rule-card">
            <div class="rule-number">1</div>
            <div class="rule-content">
              <div class="rule-title">Imperativo</div>
              <div class="rule-desc">
                ✅ "Add feature"
                <br />
                ❌ "Added feature"
                <br />
                ❌ "Adding feature"
              </div>
            </div>
          </div>

          <div class="rule-card">
            <div class="rule-number">2</div>
            <div class="rule-content">
              <div class="rule-title">Máximo 50 caracteres</div>
              <div class="rule-desc">
                Primera línea concisa. Detalles en líneas siguientes si es necesario.
              </div>
            </div>
          </div>

          <div class="rule-card">
            <div class="rule-number">3</div>
            <div class="rule-content">
              <div class="rule-title">Comienza con mayúscula</div>
              <div class="rule-desc">
                ✅ "Fix bug"
                <br />
                ❌ "fix bug"
              </div>
            </div>
          </div>

          <div class="rule-card">
            <div class="rule-number">4</div>
            <div class="rule-content">
              <div class="rule-title">Sin punto final</div>
              <div class="rule-desc">
                ✅ "Add sensor"
                <br />
                ❌ "Add sensor."
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- CONVENTIONAL COMMITS -->
    <div class="section-group">
      <SectionTitle>3. Conventional Commits</SectionTitle>
      <TextBlock>
        Es un estándar de la industria para estructurar mensajes de commit. Usa prefijos para
        categorizar cambios automáticamente.
      </TextBlock>

      <div class="conventional-grid q-mt-md">
        <div class="conventional-card feat">
          <div class="conventional-header">
            <q-icon name="new_releases" />
            <span>feat:</span>
          </div>
          <div class="conventional-desc">Nueva funcionalidad</div>
          <div class="conventional-example">
            <code>feat: add camera driver</code>
            <br />
            <code>feat: implement path planning</code>
          </div>
        </div>

        <div class="conventional-card fix">
          <div class="conventional-header">
            <q-icon name="bug_report" />
            <span>fix:</span>
          </div>
          <div class="conventional-desc">Corrección de bug</div>
          <div class="conventional-example">
            <code>fix: motor speed overflow</code>
            <br />
            <code>fix: sensor timeout issue</code>
          </div>
        </div>

        <div class="conventional-card docs">
          <div class="conventional-header">
            <q-icon name="description" />
            <span>docs:</span>
          </div>
          <div class="conventional-desc">Documentación</div>
          <div class="conventional-example">
            <code>docs: update README</code>
            <br />
            <code>docs: add API reference</code>
          </div>
        </div>

        <div class="conventional-card style">
          <div class="conventional-header">
            <q-icon name="palette" />
            <span>style:</span>
          </div>
          <div class="conventional-desc">Formato (sin cambio de lógica)</div>
          <div class="conventional-example">
            <code>style: fix indentation</code>
            <br />
            <code>style: remove trailing spaces</code>
          </div>
        </div>

        <div class="conventional-card refactor">
          <div class="conventional-header">
            <q-icon name="build" />
            <span>refactor:</span>
          </div>
          <div class="conventional-desc">Reestructuración de código</div>
          <div class="conventional-example">
            <code>refactor: simplify loop logic</code>
            <br />
            <code>refactor: extract function</code>
          </div>
        </div>

        <div class="conventional-card test">
          <div class="conventional-header">
            <q-icon name="science" />
            <span>test:</span>
          </div>
          <div class="conventional-desc">Agregar o modificar tests</div>
          <div class="conventional-example">
            <code>test: add motor unit tests</code>
            <br />
            <code>test: fix failing integration test</code>
          </div>
        </div>

        <div class="conventional-card chore">
          <div class="conventional-header">
            <q-icon name="settings" />
            <span>chore:</span>
          </div>
          <div class="conventional-desc">Tareas de mantenimiento</div>
          <div class="conventional-example">
            <code>chore: update dependencies</code>
            <br />
            <code>chore: configure linter</code>
          </div>
        </div>

        <div class="conventional-card perf">
          <div class="conventional-header">
            <q-icon name="speed" />
            <span>perf:</span>
          </div>
          <div class="conventional-desc">Mejoras de rendimiento</div>
          <div class="conventional-example">
            <code>perf: optimize path calculation</code>
            <br />
            <code>perf: cache sensor data</code>
          </div>
        </div>
      </div>
    </div>

    <!-- ATOMIC COMMITS -->
    <div class="section-group">
      <SectionTitle>4. Commits Atómicos</SectionTitle>
      <TextBlock>
        Un commit atómico es una unidad lógica indivisible de cambio. Si mezclas múltiples cambios
        no relacionados en un commit, pierdes la capacidad de revertir selectivamente.
      </TextBlock>

      <div class="atomic-visual q-mt-md">
        <div class="atomic-comparison">
          <div class="atomic-bad">
            <div class="atomic-label bad">❌ Commit Monolítico</div>
            <div class="monolithic-commit">
              <div class="commit-content">
                <div class="change-item">🛠️ Fix motor bug</div>
                <div class="change-item">🎨 Update UI colors</div>
                <div class="change-item">📄 Add documentation</div>
                <div class="change-item">🔧 Refactor config</div>
              </div>
              <div class="problem-note">
                Si el cambio de UI rompe algo, no puedes revertirlo sin perder el fix del motor
              </div>
            </div>
          </div>

          <div class="atomic-arrow">
            <q-icon name="arrow_forward" size="3rem" color="green-4" />
            <div class="arrow-label">Divide</div>
          </div>

          <div class="atomic-good">
            <div class="atomic-label good">✅ Commits Atómicos</div>
            <div class="atomic-commits">
              <div class="atomic-commit">
                <div class="commit-hash">a1b2c3d</div>
                <div class="commit-msg">fix: motor speed calculation</div>
              </div>
              <div class="atomic-commit">
                <div class="commit-hash">4e5f6g7</div>
                <div class="commit-msg">style: update UI color scheme</div>
              </div>
              <div class="atomic-commit">
                <div class="commit-hash">8h9i0j1</div>
                <div class="commit-msg">docs: add setup instructions</div>
              </div>
              <div class="atomic-commit">
                <div class="commit-hash">2k3l4m5</div>
                <div class="commit-msg">refactor: simplify config loading</div>
              </div>
            </div>
            <div class="benefit-note">Cada cambio es independiente y reversible</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Regla Atómica">
          Si no puedes describir el commit en una línea sin usar "y", probablemente deberías
          dividirlo.
        </AlertBlock>
      </div>
    </div>

    <!-- COMANDOS AVANZADOS -->
    <div class="section-group">
      <SectionTitle>5. Comandos Útiles</SectionTitle>

      <div class="commands-grid">
        <div class="command-card">
          <div class="command-header">
            <q-icon name="edit" color="blue-4" />
            <span>Modificar Último Commit</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Olvidaste agregar un archivo
git add forgotten_file.py
git commit --amend --no-edit

# Cambiar el mensaje del último commit
git commit --amend -m 'New message'"
            :copyable="true"
          />
          <div class="command-note">
            ⚠️ Solo usa <code>--amend</code> si NO has hecho push todavía
          </div>
        </div>

        <div class="command-card">
          <div class="command-header">
            <q-icon name="undo" color="orange-4" />
            <span>Deshacer Staging</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Sacar archivo de staging (mantener cambios)
git reset HEAD file.py

# Sacar todos los archivos de staging
git reset HEAD ."
            :copyable="true"
          />
          <div class="command-note">Los cambios permanecen en Working Directory</div>
        </div>

        <div class="command-card">
          <div class="command-header">
            <q-icon name="history" color="green-4" />
            <span>Ver Historial</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Historial compacto
git log --oneline

# Con gráfico de branches
git log --oneline --graph --all

# Últimos 5 commits
git log -5

# Cambios de un archivo
git log --follow file.py"
            :copyable="true"
          />
        </div>

        <div class="command-card">
          <div class="command-header">
            <q-icon name="difference" color="purple-4" />
            <span>Ver Diferencias</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Cambios no staged
git diff

# Cambios staged
git diff --staged

# Diferencia entre commits
git diff a1b2c3d 4e5f6g7"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- GIT LOG VISUAL -->
    <div class="section-group">
      <SectionTitle>6. Navegando el Historial</SectionTitle>
      <TextBlock>
        <code>git log</code> es tu máquina del tiempo. Aprende a usarla para entender la evolución
        de tu proyecto.
      </TextBlock>

      <div class="log-visual q-mt-md">
        <div class="log-header">
          <q-icon name="terminal" />
          <span>git log --oneline --graph --all</span>
        </div>
        <div class="log-content">
          <div class="log-line">
            <span class="log-graph">*</span>
            <span class="log-hash">a1b2c3d</span>
            <span class="log-ref">(HEAD -> main)</span>
            <span class="log-message">feat: add obstacle avoidance</span>
          </div>
          <div class="log-line">
            <span class="log-graph">*</span>
            <span class="log-hash">4e5f6g7</span>
            <span class="log-ref">(origin/main)</span>
            <span class="log-message">fix: motor calibration</span>
          </div>
          <div class="log-line">
            <span class="log-graph">|\</span>
          </div>
          <div class="log-line">
            <span class="log-graph">| *</span>
            <span class="log-hash">8h9i0j1</span>
            <span class="log-ref">(feature/camera)</span>
            <span class="log-message">feat: add camera driver</span>
          </div>
          <div class="log-line">
            <span class="log-graph">|/</span>
          </div>
          <div class="log-line">
            <span class="log-graph">*</span>
            <span class="log-hash">2k3l4m5</span>
            <span class="log-message">docs: update README</span>
          </div>
          <div class="log-line">
            <span class="log-graph">*</span>
            <span class="log-hash">6n7o8p9</span>
            <span class="log-ref">(tag: v1.0.0)</span>
            <span class="log-message">chore: initial release</span>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="info" title="Leyendo el Log">
          <strong>HEAD:</strong> Dónde estás ahora
          <br />
          <strong>origin/main:</strong> Última versión en GitHub
          <br />
          <strong>tag:</strong> Versión marcada (release)
          <br />
          <strong>Gráfico (*, |, /):</strong> Visualización de branches y merges
        </AlertBlock>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Git Commits Best Practices"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" color="blue-4" size="sm" />
          Video En progreso
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>git status</code>
          <span>Ver cambios</span>
        </div>
        <div class="summary-item">
          <code>git add</code>
          <span>Preparar archivos</span>
        </div>
        <div class="summary-item">
          <code>git commit -m</code>
          <span>Crear commit</span>
        </div>
        <div class="summary-item">
          <code>git commit --amend</code>
          <span>Modificar último commit</span>
        </div>
        <div class="summary-item">
          <code>git log</code>
          <span>Ver historial</span>
        </div>
        <div class="summary-item">
          <code>git diff</code>
          <span>Ver diferencias</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Buenos Commits" class="q-mt-lg">
        ✅ Mensaje claro y descriptivo (imperativo, &lt;50 caracteres)
        <br />
        ✅ Usa Conventional Commits (feat:, fix:, docs:, etc.)
        <br />
        ✅ Un commit = una tarea lógica (atómico)
        <br />
        ✅ Commits frecuentes (no esperes días para commitear)
        <br />
        ✅ Revisa con <code>git status</code> antes de commitear
        <br />
        ✅ Usa <code>git diff</code> para verificar cambios
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* RITUAL VISUAL */
.ritual-visual {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.ritual-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.step-icon {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  color: white;
}

.step-icon.check {
  background: linear-gradient(135deg, #3b82f6, #2563eb);
}

.step-icon.stage {
  background: linear-gradient(135deg, #fbbf24, #f59e0b);
}

.step-icon.commit {
  background: linear-gradient(135deg, #22c55e, #16a34a);
}

.step-content {
  flex: 1;
}

.step-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.step-desc {
  color: #94a3b8;
  margin-bottom: 1rem;
}

.ritual-arrow {
  display: flex;
  justify-content: center;
  padding: 0.5rem 0;
}

/* MESSAGE COMPARISON */
.message-comparison {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.message-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.message-card.bad {
  border-color: #ef4444;
}

.message-card.good {
  border-color: #22c55e;
}

.message-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.message-card.bad .message-header {
  color: #fca5a5;
}

.message-card.good .message-header {
  color: #86efac;
}

.message-examples {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.message-item {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.message-item code {
  font-family: 'Fira Code', monospace;
  color: #cbd5e1;
  font-size: 0.95rem;
}

.message-problem {
  color: #f87171;
  font-size: 0.85rem;
  font-style: italic;
}

.message-why {
  color: #86efac;
  font-size: 0.85rem;
  font-style: italic;
}

/* RULES GRID */
.rules-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
  margin-top: 1rem;
}

.rule-card {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  align-items: start;
}

.rule-number {
  width: 40px;
  height: 40px;
  background: linear-gradient(135deg, #f97316, #ea580c);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.3rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.rule-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.rule-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

/* CONVENTIONAL COMMITS */
.conventional-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.conventional-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid;
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.conventional-card.feat {
  border-color: #22c55e;
}

.conventional-card.fix {
  border-color: #ef4444;
}

.conventional-card.docs {
  border-color: #3b82f6;
}

.conventional-card.style {
  border-color: #a855f7;
}

.conventional-card.refactor {
  border-color: #fbbf24;
}

.conventional-card.test {
  border-color: #06b6d4;
}

.conventional-card.chore {
  border-color: #64748b;
}

.conventional-card.perf {
  border-color: #f97316;
}

.conventional-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
}

.conventional-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

.conventional-example {
  background: rgba(0, 0, 0, 0.3);
  padding: 0.75rem;
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #cbd5e1;
  line-height: 1.8;
}

/* ATOMIC VISUAL */
.atomic-visual {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.atomic-comparison {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
}

.atomic-label {
  font-size: 1.1rem;
  font-weight: 700;
  margin-bottom: 1rem;
  text-align: center;
}

.atomic-label.bad {
  color: #fca5a5;
}

.atomic-label.good {
  color: #86efac;
}

.monolithic-commit {
  background: rgba(239, 68, 68, 0.1);
  border: 2px solid #ef4444;
  border-radius: 12px;
  padding: 1.5rem;
}

.commit-content {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  margin-bottom: 1rem;
}

.change-item {
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  color: #cbd5e1;
}

.problem-note {
  color: #fca5a5;
  font-size: 0.85rem;
  font-style: italic;
  text-align: center;
}

.atomic-arrow {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.arrow-label {
  color: #86efac;
  font-weight: 700;
}

.atomic-commits {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.atomic-commit {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid #22c55e;
  border-radius: 8px;
  padding: 0.75rem;
}

.commit-hash {
  font-family: 'Fira Code', monospace;
  color: #fbbf24;
  font-size: 0.85rem;
  margin-bottom: 0.25rem;
}

.commit-msg {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.benefit-note {
  color: #86efac;
  font-size: 0.85rem;
  font-style: italic;
  text-align: center;
  margin-top: 1rem;
}

/* COMMANDS GRID */
.commands-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.command-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.command-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
}

.command-note {
  color: #94a3b8;
  font-size: 0.85rem;
  font-style: italic;
}

/* LOG VISUAL */
.log-visual {
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.log-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-family: 'Fira Code', monospace;
  color: #94a3b8;
}

.log-content {
  padding: 1.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.log-line {
  display: flex;
  gap: 0.75rem;
  margin-bottom: 0.5rem;
  align-items: center;
}

.log-graph {
  color: #ef4444;
  font-weight: 700;
  width: 30px;
}

.log-hash {
  color: #fbbf24;
  font-weight: 700;
}

.log-ref {
  color: #3b82f6;
  font-weight: 700;
}

.log-message {
  color: #cbd5e1;
}

/* VIDEO */
.video-container {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 12px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #94a3b8;
  font-size: 0.85rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: #22c55e;
  font-size: 1rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .message-comparison {
    grid-template-columns: 1fr;
  }

  .atomic-comparison {
    grid-template-columns: 1fr;
  }

  .atomic-arrow {
    transform: rotate(90deg);
  }
}
</style>
