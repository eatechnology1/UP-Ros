<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      El historial de Git es tu máquina del tiempo. Cada commit es un punto en la línea temporal de
      tu proyecto. Dominar la navegación del historial te permite entender la evolución del código,
      encontrar bugs, y volver a versiones anteriores cuando algo falla.
      <br /><br />
      No se trata solo de ver commits antiguos. Se trata de <strong>entender</strong> cómo llegaste
      aquí y <strong>controlar</strong> hacia dónde vas.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué es importante el historial?">
      <strong>Debugging:</strong> Encontrar cuándo se introdujo un bug (git bisect)
      <br />
      <strong>Auditoría:</strong> Ver quién cambió qué y cuándo
      <br />
      <strong>Recuperación:</strong> Volver a una versión que funcionaba
      <br />
      <strong>Aprendizaje:</strong> Entender decisiones de diseño pasadas
    </AlertBlock>

    <!-- GIT LOG -->
    <div class="section-group">
      <SectionTitle>1. Git Log: Tu Mapa Estelar</SectionTitle>
      <TextBlock>
        <code>git log</code> por defecto es feo y difícil de leer. Los profesionales usan flags
        (opciones) para convertir esa lista aburrida en un árbol gráfico comprensible.
      </TextBlock>

      <div class="log-commands q-mt-md">
        <div class="command-showcase">
          <div class="command-title">La Navaja Suiza</div>
          <CodeBlock
            lang="bash"
            content="git log --oneline --graph --all --decorate"
            :copyable="true"
          />
          <div class="command-desc">
            Muestra historial compacto con gráfico de branches, todos los commits, y referencias
          </div>
        </div>

        <div class="log-visual q-mt-md">
          <div class="log-header">
            <q-icon name="terminal" />
            <span>git log --oneline --graph --all</span>
          </div>
          <div class="log-content">
            <div class="log-line">
              <span class="log-graph">*</span>
              <span class="log-hash">a1b2c3d</span>
              <span class="log-ref">(HEAD -> main, origin/main)</span>
              <span class="log-message">feat: add obstacle avoidance</span>
            </div>
            <div class="log-line">
              <span class="log-graph">*</span>
              <span class="log-hash">4e5f6g7</span>
              <span class="log-message">fix: motor calibration bug</span>
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
              <span class="log-graph">| *</span>
              <span class="log-hash">2k3l4m5</span>
              <span class="log-message">test: add camera unit tests</span>
            </div>
            <div class="log-line">
              <span class="log-graph">|/</span>
            </div>
            <div class="log-line">
              <span class="log-graph">*</span>
              <span class="log-hash">6n7o8p9</span>
              <span class="log-ref">(tag: v1.0.0)</span>
              <span class="log-message">chore: initial release</span>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Opciones Útiles de Git Log</SectionTitle>
        <div class="options-grid">
          <div class="option-card">
            <div class="option-header">
              <q-icon name="compress" color="blue-4" />
              <code>--oneline</code>
            </div>
            <div class="option-desc">Una línea por commit (hash corto + mensaje)</div>
          </div>

          <div class="option-card">
            <div class="option-header">
              <q-icon name="account_tree" color="green-4" />
              <code>--graph</code>
            </div>
            <div class="option-desc">Muestra gráfico ASCII de branches y merges</div>
          </div>

          <div class="option-card">
            <div class="option-header">
              <q-icon name="all_inclusive" color="purple-4" />
              <code>--all</code>
            </div>
            <div class="option-desc">Muestra TODAS las branches (no solo la actual)</div>
          </div>

          <div class="option-card">
            <div class="option-header">
              <q-icon name="filter_5" color="orange-4" />
              <code>-n 5</code>
            </div>
            <div class="option-desc">Limita a los últimos 5 commits</div>
          </div>

          <div class="option-card">
            <div class="option-header">
              <q-icon name="person" color="yellow-6" />
              <code>--author="Alex"</code>
            </div>
            <div class="option-desc">Filtra por autor</div>
          </div>

          <div class="option-card">
            <div class="option-header">
              <q-icon name="calendar_today" color="red-4" />
              <code>--since="2 weeks"</code>
            </div>
            <div class="option-desc">Commits de las últimas 2 semanas</div>
          </div>
        </div>
      </div>
    </div>

    <!-- NAVEGACIÓN -->
    <div class="section-group">
      <SectionTitle>2. Navegando el Historial</SectionTitle>
      <TextBlock>
        Git te permite moverte libremente por el historial. Es como tener una máquina del tiempo
        para tu código.
      </TextBlock>

      <div class="navigation-demo q-mt-md">
        <div class="timeline-visual">
          <div class="timeline-header">Línea de Tiempo del Proyecto</div>
          <div class="timeline-track">
            <div class="commit-point past">
              <div class="commit-dot"></div>
              <div class="commit-info">
                <div class="commit-hash">6n7o8p9</div>
                <div class="commit-label">v1.0.0</div>
                <div class="commit-date">Hace 2 meses</div>
              </div>
            </div>

            <div class="commit-point middle">
              <div class="commit-dot"></div>
              <div class="commit-info">
                <div class="commit-hash">2k3l4m5</div>
                <div class="commit-label">Add tests</div>
                <div class="commit-date">Hace 1 mes</div>
              </div>
            </div>

            <div class="commit-point current">
              <div class="commit-dot"></div>
              <div class="commit-info">
                <div class="commit-hash">a1b2c3d</div>
                <div class="commit-label">HEAD → main</div>
                <div class="commit-date">Ahora</div>
              </div>
            </div>
          </div>
        </div>

        <div class="navigation-commands q-mt-lg">
          <CodeBlock
            title="Comandos de navegación"
            lang="bash"
            content="# Ver commit específico (modo lectura)
git checkout a1b2c3d

# Volver al presente
git checkout main

# Ver archivo específico de un commit
git show a1b2c3d:src/motor.py

# Comparar dos commits
git diff 6n7o8p9 a1b2c3d

# Ver cambios de un commit
git show a1b2c3d"
            :copyable="true"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Detached HEAD State">
          Cuando haces <code>git checkout &lt;hash&gt;</code>, entras en "Detached HEAD". Puedes
          mirar el código, compilar y ejecutar, pero si haces cambios, se perderán al volver a main.
          <br /><br />
          <strong>Solución:</strong> Si quieres hacer cambios, crea una rama:
          <code>git checkout -b nueva-rama</code>
        </AlertBlock>
      </div>
    </div>

    <!-- DESHACER CAMBIOS -->
    <div class="section-group">
      <SectionTitle>3. Deshacer Cambios: Los 3 Niveles</SectionTitle>
      <TextBlock>
        Hay diferentes formas de "deshacer" en Git, dependiendo de qué tan lejos llegaste. Elegir el
        comando correcto es crucial.
      </TextBlock>

      <div class="undo-levels q-mt-md">
        <div class="undo-card level-1">
          <div class="undo-header">
            <div class="level-badge">Nivel 1</div>
            <div class="level-title">Cambios sin Staging</div>
          </div>
          <div class="undo-scenario">
            <strong>Escenario:</strong> Modificaste archivos pero NO hiciste <code>git add</code>
          </div>
          <CodeBlock
            lang="bash"
            content="# Descartar cambios en un archivo
git restore archivo.py

# Descartar TODOS los cambios
git restore .

# Comando antiguo (también funciona)
git checkout -- archivo.py"
            :copyable="true"
          />
          <div class="undo-warning">
            ⚠️ Los cambios se pierden permanentemente. No hay forma de recuperarlos.
          </div>
        </div>

        <div class="undo-card level-2">
          <div class="undo-header">
            <div class="level-badge">Nivel 2</div>
            <div class="level-title">Cambios en Staging</div>
          </div>
          <div class="undo-scenario">
            <strong>Escenario:</strong> Hiciste <code>git add</code> pero NO hiciste
            <code>git commit</code>
          </div>
          <CodeBlock
            lang="bash"
            content="# Sacar archivo de staging (mantener cambios)
git restore --staged archivo.py

# Sacar todo de staging
git restore --staged .

# Comando antiguo (también funciona)
git reset HEAD archivo.py"
            :copyable="true"
          />
          <div class="undo-note">✅ Los cambios permanecen en Working Directory</div>
        </div>

        <div class="undo-card level-3">
          <div class="undo-header">
            <div class="level-badge">Nivel 3</div>
            <div class="level-title">Commit Erróneo</div>
          </div>
          <div class="undo-scenario">
            <strong>Escenario:</strong> Hiciste commit pero te equivocaste (olvidaste archivo,
            mensaje mal, etc.)
          </div>
          <CodeBlock
            lang="bash"
            content="# Modificar el último commit (agregar archivo olvidado)
git add archivo_olvidado.py
git commit --amend --no-edit

# Cambiar mensaje del último commit
git commit --amend -m 'Nuevo mensaje correcto'

# Modificar commit Y mensaje
git add archivo.py
git commit --amend -m 'Mensaje actualizado'"
            :copyable="true"
          />
          <div class="undo-warning">
            ⚠️ Solo usa <code>--amend</code> si NO has hecho push. Reescribe la historia.
          </div>
        </div>
      </div>
    </div>

    <!-- RESET VS REVERT -->
    <div class="section-group">
      <SectionTitle>4. Reset vs Revert: La Diferencia Crucial</SectionTitle>
      <TextBlock>
        Ambos "deshacen" commits, pero de formas completamente diferentes. Elegir mal puede causar
        problemas serios en equipos.
      </TextBlock>

      <div class="comparison-visual q-mt-md">
        <div class="comparison-card reset">
          <div class="comparison-header">
            <q-icon name="delete_forever" size="2.5rem" />
            <div class="comparison-title">git reset</div>
          </div>
          <div class="comparison-desc">
            <strong>Borra commits</strong> de la historia. Es como si nunca hubieran existido.
          </div>

          <div class="visual-demo">
            <div class="demo-label">Antes</div>
            <div class="commit-chain">
              <div class="commit-item">A</div>
              <div class="commit-item">B</div>
              <div class="commit-item bad">C (malo)</div>
              <div class="commit-item current">D (HEAD)</div>
            </div>

            <div class="demo-arrow">
              <q-icon name="arrow_downward" size="2rem" />
              <code>git reset --hard B</code>
            </div>

            <div class="demo-label">Después</div>
            <div class="commit-chain">
              <div class="commit-item">A</div>
              <div class="commit-item current">B (HEAD)</div>
              <div class="commit-item deleted">C ❌</div>
              <div class="commit-item deleted">D ❌</div>
            </div>
          </div>

          <CodeBlock
            lang="bash"
            content="# Volver 2 commits atrás (PELIGROSO)
git reset --hard HEAD~2

# Volver a commit específico
git reset --hard a1b2c3d"
            :copyable="true"
          />

          <div class="comparison-warning">
            ⚠️ <strong>NUNCA</strong> uses reset en commits que ya hiciste push. Romperás el
            historial del equipo.
          </div>
        </div>

        <div class="comparison-card revert">
          <div class="comparison-header">
            <q-icon name="undo" size="2.5rem" />
            <div class="comparison-title">git revert</div>
          </div>
          <div class="comparison-desc">
            <strong>Crea un nuevo commit</strong> que deshace los cambios. La historia permanece
            intacta.
          </div>

          <div class="visual-demo">
            <div class="demo-label">Antes</div>
            <div class="commit-chain">
              <div class="commit-item">A</div>
              <div class="commit-item">B</div>
              <div class="commit-item bad">C (malo)</div>
              <div class="commit-item current">D (HEAD)</div>
            </div>

            <div class="demo-arrow">
              <q-icon name="arrow_downward" size="2rem" />
              <code>git revert C</code>
            </div>

            <div class="demo-label">Después</div>
            <div class="commit-chain">
              <div class="commit-item">A</div>
              <div class="commit-item">B</div>
              <div class="commit-item">C</div>
              <div class="commit-item">D</div>
              <div class="commit-item current good">C' (revierte C)</div>
            </div>
          </div>

          <CodeBlock
            lang="bash"
            content="# Revertir commit específico
git revert a1b2c3d

# Revertir sin abrir editor
git revert a1b2c3d --no-edit

# Revertir múltiples commits
git revert a1b2c3d..4e5f6g7"
            :copyable="true"
          />

          <div class="comparison-safe">
            ✅ <strong>SEGURO</strong> para código ya compartido. No reescribe historia.
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Regla de Oro">
          <strong>Código local (no push):</strong> Usa <code>reset</code>
          <br />
          <strong>Código compartido (ya push):</strong> Usa <code>revert</code>
        </AlertBlock>
      </div>
    </div>

    <!-- GIT DIFF -->
    <div class="section-group">
      <SectionTitle>5. Git Diff: Comparar Cambios</SectionTitle>
      <TextBlock>
        <code>git diff</code> te muestra exactamente qué cambió. Es esencial para revisar tu trabajo
        antes de commitear.
      </TextBlock>

      <div class="diff-commands q-mt-md">
        <div class="diff-card">
          <div class="diff-header">
            <q-icon name="difference" color="blue-4" />
            <span>Cambios no staged</span>
          </div>
          <CodeBlock lang="bash" content="git diff" :copyable="true" />
          <div class="diff-desc">
            Muestra cambios en Working Directory (no agregados con git add)
          </div>
        </div>

        <div class="diff-card">
          <div class="diff-header">
            <q-icon name="check_box" color="green-4" />
            <span>Cambios staged</span>
          </div>
          <CodeBlock lang="bash" content="git diff --staged" :copyable="true" />
          <div class="diff-desc">Muestra cambios que están en staging (listos para commit)</div>
        </div>

        <div class="diff-card">
          <div class="diff-header">
            <q-icon name="compare_arrows" color="purple-4" />
            <span>Entre commits</span>
          </div>
          <CodeBlock lang="bash" content="git diff a1b2c3d 4e5f6g7" :copyable="true" />
          <div class="diff-desc">Compara dos commits específicos</div>
        </div>

        <div class="diff-card">
          <div class="diff-header">
            <q-icon name="description" color="orange-4" />
            <span>Archivo específico</span>
          </div>
          <CodeBlock lang="bash" content="git diff HEAD~2 HEAD -- archivo.py" :copyable="true" />
          <div class="diff-desc">Cambios en un archivo entre dos puntos</div>
        </div>
      </div>

      <div class="diff-visual q-mt-lg">
        <div class="diff-output-header">
          <q-icon name="terminal" />
          <span>Ejemplo de salida de git diff</span>
        </div>
        <div class="diff-output">
          <div class="diff-file">diff --git a/src/motor.py b/src/motor.py</div>
          <div class="diff-meta">index a1b2c3d..4e5f6g7 100644</div>
          <div class="diff-meta">--- a/src/motor.py</div>
          <div class="diff-meta">+++ b/src/motor.py</div>
          <div class="diff-hunk">@@ -10,7 +10,7 @@ def calculate_speed():</div>
          <div class="diff-context">max_rpm = 1000</div>
          <div class="diff-removed">- speed = rpm / 100</div>
          <div class="diff-added">+ speed = rpm / max_rpm</div>
          <div class="diff-context">return speed</div>
        </div>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="Git History and Time Travel"
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
          Reemplaza dQw4w9WgXcQ con tu video de YouTube
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>git log --oneline --graph</code>
          <span>Ver historial visual</span>
        </div>
        <div class="summary-item">
          <code>git checkout &lt;hash&gt;</code>
          <span>Viajar a commit</span>
        </div>
        <div class="summary-item">
          <code>git restore</code>
          <span>Descartar cambios</span>
        </div>
        <div class="summary-item">
          <code>git reset</code>
          <span>Borrar commits (local)</span>
        </div>
        <div class="summary-item">
          <code>git revert</code>
          <span>Deshacer commit (seguro)</span>
        </div>
        <div class="summary-item">
          <code>git diff</code>
          <span>Ver diferencias</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Historial" class="q-mt-lg">
        ✅ Usa <code>git log --oneline --graph --all</code> para ver el panorama completo
        <br />
        ✅ <code>git diff</code> antes de commitear para revisar cambios
        <br />
        ✅ <code>git restore</code> para descartar cambios locales
        <br />
        ✅ <code>git commit --amend</code> solo si NO hiciste push
        <br />
        ✅ <code>git revert</code> para código ya compartido (nunca reset)
        <br />
        ✅ Crea una rama si quieres hacer cambios en detached HEAD
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

/* LOG COMMANDS */
.command-showcase {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid #a855f7;
  border-radius: 16px;
  padding: 2rem;
  text-align: center;
}

.command-title {
  font-size: 1.3rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.command-desc {
  color: #94a3b8;
  margin-top: 1rem;
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

/* OPTIONS GRID */
.options-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1rem;
  margin-top: 1rem;
}

.option-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.option-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  color: #f1f5f9;
}

.option-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

/* TIMELINE VISUAL */
.timeline-visual {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.timeline-header {
  text-align: center;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 2rem;
}

.timeline-track {
  position: relative;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 2rem 0;
}

.timeline-track::before {
  content: '';
  position: absolute;
  top: 50%;
  left: 0;
  right: 0;
  height: 2px;
  background: linear-gradient(90deg, #64748b, #22c55e);
  transform: translateY(-50%);
}

.commit-point {
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  z-index: 2;
}

.commit-dot {
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background: #64748b;
  border: 3px solid rgba(15, 23, 42, 0.9);
  margin-bottom: 1rem;
}

.commit-point.current .commit-dot {
  background: #22c55e;
  box-shadow: 0 0 20px rgba(34, 197, 94, 0.5);
}

.commit-info {
  text-align: center;
}

.commit-hash {
  font-family: 'Fira Code', monospace;
  color: #fbbf24;
  font-weight: 700;
  font-size: 0.9rem;
}

.commit-label {
  color: #cbd5e1;
  font-size: 0.85rem;
  margin-top: 0.25rem;
}

.commit-date {
  color: #64748b;
  font-size: 0.75rem;
  margin-top: 0.25rem;
}

/* UNDO LEVELS */
.undo-levels {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.undo-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.undo-card.level-1 {
  border-color: #fbbf24;
}

.undo-card.level-2 {
  border-color: #f97316;
}

.undo-card.level-3 {
  border-color: #ef4444;
}

.undo-header {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.level-badge {
  padding: 0.5rem 1rem;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 8px;
  font-weight: 700;
  color: #f1f5f9;
}

.level-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.undo-scenario {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.undo-warning {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid #ef4444;
  padding: 0.75rem;
  border-radius: 8px;
  color: #fca5a5;
  font-size: 0.9rem;
}

.undo-note {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid #22c55e;
  padding: 0.75rem;
  border-radius: 8px;
  color: #86efac;
  font-size: 0.9rem;
}

/* COMPARISON VISUAL */
.comparison-visual {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.comparison-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.comparison-card.reset {
  border-color: #ef4444;
}

.comparison-card.revert {
  border-color: #22c55e;
}

.comparison-header {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  color: #f1f5f9;
}

.comparison-title {
  font-family: 'Fira Code', monospace;
  font-size: 1.3rem;
  font-weight: 700;
}

.comparison-desc {
  color: #cbd5e1;
  text-align: center;
}

.visual-demo {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 1.5rem;
  border-radius: 12px;
}

.demo-label {
  font-weight: 700;
  color: #94a3b8;
  text-align: center;
}

.commit-chain {
  display: flex;
  gap: 0.5rem;
  justify-content: center;
}

.commit-item {
  width: 50px;
  height: 50px;
  background: #3b82f6;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: white;
}

.commit-item.bad {
  background: #ef4444;
}

.commit-item.current {
  background: #22c55e;
  box-shadow: 0 0 15px rgba(34, 197, 94, 0.5);
}

.commit-item.deleted {
  background: #64748b;
  opacity: 0.3;
}

.commit-item.good {
  background: #22c55e;
}

.demo-arrow {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  color: #fbbf24;
  font-family: 'Fira Code', monospace;
}

.comparison-warning {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid #ef4444;
  padding: 1rem;
  border-radius: 8px;
  color: #fca5a5;
  font-size: 0.9rem;
}

.comparison-safe {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid #22c55e;
  padding: 1rem;
  border-radius: 8px;
  color: #86efac;
  font-size: 0.9rem;
}

/* DIFF COMMANDS */
.diff-commands {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.diff-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.diff-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
}

.diff-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

/* DIFF VISUAL */
.diff-visual {
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.diff-output-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-family: 'Fira Code', monospace;
  color: #94a3b8;
}

.diff-output {
  padding: 1.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.diff-file {
  color: #f1f5f9;
  font-weight: 700;
  margin-bottom: 0.5rem;
}

.diff-meta {
  color: #64748b;
  margin-bottom: 0.25rem;
}

.diff-hunk {
  color: #3b82f6;
  margin: 0.5rem 0;
}

.diff-context {
  color: #cbd5e1;
  margin-bottom: 0.25rem;
}

.diff-removed {
  background: rgba(239, 68, 68, 0.1);
  color: #fca5a5;
  margin-bottom: 0.25rem;
  padding-left: 0.5rem;
}

.diff-added {
  background: rgba(34, 197, 94, 0.1);
  color: #86efac;
  margin-bottom: 0.25rem;
  padding-left: 0.5rem;
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
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .comparison-visual {
    grid-template-columns: 1fr;
  }

  .timeline-track {
    flex-direction: column;
    gap: 2rem;
  }

  .timeline-track::before {
    width: 2px;
    height: 100%;
    left: 50%;
    top: 0;
    transform: translateX(-50%);
  }
}
</style>
