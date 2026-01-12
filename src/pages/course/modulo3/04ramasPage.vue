<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Las ramas (branches) son la característica más poderosa de Git. Te permiten crear realidades
      alternativas donde puedes experimentar sin miedo a romper el código principal. En robótica,
      donde un bug puede hacer que el robot se estrelle, las ramas son esenciales.
      <br /><br />
      Dominar las ramas te convierte de principiante a profesional.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué usar branches?">
      <strong>Experimentación segura:</strong> Prueba nuevos algoritmos sin romper main
      <br />
      <strong>Trabajo paralelo:</strong> Múltiples features en desarrollo simultáneo
      <br />
      <strong>Code review:</strong> Revisar código antes de fusionar a main
      <br />
      <strong>Rollback fácil:</strong> Si algo falla, simplemente borra la rama
    </AlertBlock>

    <!-- REGLA DE ORO -->
    <div class="section-group">
      <SectionTitle>1. La Regla de Oro: Main es Sagrado</SectionTitle>
      <TextBlock>
        Tu rama principal (llamada <code>main</code> o antiguamente <code>master</code>) es sagrada.
        <strong>El código en main siempre debe compilar y funcionar.</strong> Nunca experimentes
        directamente en main.
      </TextBlock>

      <div class="golden-rule q-mt-md">
        <div class="rule-card main-card">
          <div class="rule-icon">
            <q-icon name="verified" size="4rem" color="green-4" />
          </div>
          <div class="rule-title">main</div>
          <div class="rule-desc">
            <div class="rule-item">✅ Código estable y probado</div>
            <div class="rule-item">✅ Siempre compila</div>
            <div class="rule-item">✅ Pasa todos los tests</div>
            <div class="rule-item">✅ Listo para producción</div>
          </div>
          <div class="rule-command">
            <code>git checkout main</code>
          </div>
        </div>

        <div class="rule-arrow">
          <q-icon name="block" size="3rem" color="red-4" />
          <div class="arrow-label">NUNCA edites directamente</div>
        </div>

        <div class="rule-card feature-card">
          <div class="rule-icon">
            <q-icon name="science" size="4rem" color="yellow-6" />
          </div>
          <div class="rule-title">feature/*</div>
          <div class="rule-desc">
            <div class="rule-item">🔬 Experimentación libre</div>
            <div class="rule-item">🔬 Puede estar roto</div>
            <div class="rule-item">🔬 Work in progress</div>
            <div class="rule-item">🔬 Zona de pruebas</div>
          </div>
          <div class="rule-command">
            <code>git checkout -b feature/nueva</code>
          </div>
        </div>
      </div>
    </div>

    <!-- CICLO DE VIDA -->
    <div class="section-group">
      <SectionTitle>2. Ciclo de Vida de una Rama</SectionTitle>
      <TextBlock>
        El flujo típico de trabajo con ramas sigue 4 pasos. Usamos comandos modernos
        (<code>switch</code>) que son más claros que el antiguo <code>checkout</code>.
      </TextBlock>

      <div class="lifecycle-visual q-mt-md">
        <div class="lifecycle-step">
          <div class="step-number">1</div>
          <div class="step-content">
            <div class="step-title">Crear y Cambiar</div>
            <div class="step-desc">Creas el universo paralelo y te teletransportas a él</div>
            <CodeBlock
              lang="bash"
              content="# Crear y cambiar en un comando
git switch -c feature/add-camera

# Equivalente antiguo
git checkout -b feature/add-camera

# Ver en qué rama estás
git branch"
              :copyable="true"
            />
          </div>
        </div>

        <div class="lifecycle-arrow">
          <q-icon name="arrow_downward" size="2rem" color="orange-4" />
        </div>

        <div class="lifecycle-step">
          <div class="step-number">2</div>
          <div class="step-content">
            <div class="step-title">Hacer Cambios</div>
            <div class="step-desc">
              Modificas archivos y haces commits normales. Main no se entera
            </div>
            <CodeBlock
              lang="bash"
              content="# Editar archivos...
git add src/camera_driver.py
git commit -m 'feat: add camera driver'

# Más commits...
git commit -m 'test: add camera tests'
git commit -m 'docs: update README'"
              :copyable="true"
            />
          </div>
        </div>

        <div class="lifecycle-arrow">
          <q-icon name="arrow_downward" size="2rem" color="orange-4" />
        </div>

        <div class="lifecycle-step">
          <div class="step-number">3</div>
          <div class="step-content">
            <div class="step-title">Volver a Main</div>
            <div class="step-desc">Regresas a la realidad original</div>
            <CodeBlock
              lang="bash"
              content="# Cambiar a main
git switch main

# Equivalente antiguo
git checkout main

# Traer últimos cambios del equipo
git pull"
              :copyable="true"
            />
          </div>
        </div>

        <div class="lifecycle-arrow">
          <q-icon name="arrow_downward" size="2rem" color="orange-4" />
        </div>

        <div class="lifecycle-step">
          <div class="step-number">4</div>
          <div class="step-content">
            <div class="step-title">Fusionar (Merge)</div>
            <div class="step-desc">Traes los cambios de la rama a main</div>
            <CodeBlock
              lang="bash"
              content="# Estando en main, fusionar feature
git merge feature/add-camera

# Si todo está bien, eliminar rama
git branch -d feature/add-camera"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- COMANDOS ESENCIALES -->
    <div class="section-group">
      <SectionTitle>3. Comandos Esenciales de Branches</SectionTitle>

      <div class="commands-grid">
        <div class="command-card">
          <div class="command-header">
            <q-icon name="add_circle" color="green-4" />
            <span>Crear Rama</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Crear sin cambiar
git branch feature/nueva

# Crear y cambiar (recomendado)
git switch -c feature/nueva

# Desde commit específico
git switch -c hotfix a1b2c3d"
            :copyable="true"
          />
        </div>

        <div class="command-card">
          <div class="command-header">
            <q-icon name="swap_horiz" color="blue-4" />
            <span>Cambiar de Rama</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Cambiar a rama existente
git switch feature/nueva

# Volver a main
git switch main

# Cambiar a rama remota
git switch -c local origin/remote-branch"
            :copyable="true"
          />
        </div>

        <div class="command-card">
          <div class="command-header">
            <q-icon name="list" color="purple-4" />
            <span>Listar Ramas</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Ramas locales
git branch

# Todas las ramas (locales + remotas)
git branch -a

# Ramas con último commit
git branch -v"
            :copyable="true"
          />
        </div>

        <div class="command-card">
          <div class="command-header">
            <q-icon name="delete" color="red-4" />
            <span>Eliminar Rama</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Eliminar rama fusionada
git branch -d feature/vieja

# Forzar eliminación (no fusionada)
git branch -D feature/experimental

# Eliminar rama remota
git push origin --delete feature/vieja"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- MERGE VISUAL -->
    <div class="section-group">
      <SectionTitle>4. Visualizando el Merge</SectionTitle>
      <TextBlock>
        Cuando haces <strong>merge</strong>, Git toma la historia de tu rama experimental y la
        inyecta en main. Hay dos tipos de merge: Fast-Forward y Merge Commit.
      </TextBlock>

      <div class="merge-types q-mt-md">
        <div class="merge-type-card">
          <div class="merge-type-header">
            <q-icon name="fast_forward" color="green-4" size="2rem" />
            <span>Fast-Forward Merge</span>
          </div>
          <div class="merge-type-desc">
            Main no ha cambiado desde que creaste la rama. Git simplemente mueve el puntero de main
            hacia adelante.
          </div>

          <div class="merge-visual">
            <div class="visual-label">Antes del merge</div>
            <div class="commit-chain">
              <div class="commit-dot main">A</div>
              <div class="commit-dot main">B</div>
              <div class="commit-dot main-pointer">main</div>
              <div class="commit-dot feature">C</div>
              <div class="commit-dot feature-pointer">feature</div>
            </div>

            <div class="visual-arrow">
              <q-icon name="arrow_downward" size="2rem" />
              <code>git merge feature</code>
            </div>

            <div class="visual-label">Después del merge</div>
            <div class="commit-chain">
              <div class="commit-dot">A</div>
              <div class="commit-dot">B</div>
              <div class="commit-dot">C</div>
              <div class="commit-dot main-pointer">main, feature</div>
            </div>
          </div>

          <div class="merge-note success">
            ✅ Historial lineal y limpio. No crea commit adicional.
          </div>
        </div>

        <div class="merge-type-card">
          <div class="merge-type-header">
            <q-icon name="merge_type" color="purple-4" size="2rem" />
            <span>Merge Commit</span>
          </div>
          <div class="merge-type-desc">
            Main ha cambiado mientras trabajabas en la rama. Git crea un commit especial que une
            ambas historias.
          </div>

          <div class="merge-visual">
            <div class="visual-label">Antes del merge</div>
            <div class="branch-diagram">
              <div class="branch-line main-line">
                <div class="commit-dot">A</div>
                <div class="commit-dot">B</div>
                <div class="commit-dot">D</div>
                <div class="commit-dot main-pointer">main</div>
              </div>
              <div class="branch-line feature-line">
                <div class="commit-dot">C</div>
                <div class="commit-dot feature-pointer">feature</div>
              </div>
            </div>

            <div class="visual-arrow">
              <q-icon name="arrow_downward" size="2rem" />
              <code>git merge feature</code>
            </div>

            <div class="visual-label">Después del merge</div>
            <div class="branch-diagram merged">
              <div class="commit-dot">A</div>
              <div class="commit-dot">B</div>
              <div class="commit-dot">D</div>
              <div class="commit-dot">C</div>
              <div class="commit-dot merge-commit">M</div>
              <div class="commit-dot main-pointer">main</div>
            </div>
          </div>

          <div class="merge-note info">ℹ️ Preserva la historia completa. Crea commit de merge.</div>
        </div>
      </div>
    </div>

    <!-- CONFLICTOS -->
    <div class="section-group">
      <SectionTitle>5. Conflictos de Merge: El Villano</SectionTitle>
      <TextBlock>
        Si tú cambias la línea 10 de un archivo en tu rama, y otro compañero cambia la MISMA línea
        10 en main... Git entra en pánico. No sabe a quién creerle y te pide ayuda.
      </TextBlock>

      <div class="conflict-scenario q-mt-md">
        <div class="scenario-header">
          <q-icon name="warning" color="red-4" size="2rem" />
          <span>Escenario de Conflicto</span>
        </div>

        <div class="scenario-visual">
          <div class="scenario-branch">
            <div class="branch-label main">main</div>
            <div class="code-snippet">
              <div class="line-number">10</div>
              <div class="code-line">max_speed = 1.5</div>
            </div>
          </div>

          <div class="conflict-icon">
            <q-icon name="close" size="3rem" color="red-4" />
          </div>

          <div class="scenario-branch">
            <div class="branch-label feature">feature/speed</div>
            <div class="code-snippet">
              <div class="line-number">10</div>
              <div class="code-line">max_speed = 2.0</div>
            </div>
          </div>
        </div>

        <div class="conflict-message">
          Git no sabe cuál valor es el correcto. Necesita que TÚ decidas.
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Cómo Resolver Conflictos</SectionTitle>
        <div class="resolution-steps">
          <div class="resolution-step">
            <div class="resolution-number">1</div>
            <div class="resolution-content">
              <div class="resolution-title">Git marca el conflicto</div>
              <div class="conflict-file">
                <div class="conflict-header">
                  <q-icon name="error" color="red-4" />
                  <span>motor_driver.py</span>
                </div>
                <div class="conflict-code">
                  <div class="conflict-marker head">&lt;&lt;&lt;&lt;&lt;&lt;&lt; HEAD (main)</div>
                  <div class="conflict-content main">max_speed = 1.5</div>
                  <div class="conflict-marker separator">=======</div>
                  <div class="conflict-content feature">max_speed = 2.0</div>
                  <div class="conflict-marker feature">
                    &gt;&gt;&gt;&gt;&gt;&gt;&gt; feature/speed
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div class="resolution-step">
            <div class="resolution-number">2</div>
            <div class="resolution-content">
              <div class="resolution-title">Edita el archivo manualmente</div>
              <div class="resolution-desc">
                Elimina los marcadores (&lt;&lt;&lt;, ===, &gt;&gt;&gt;) y deja el código correcto:
              </div>
              <CodeBlock
                lang="python"
                content="# Opción 1: Quedarte con main
max_speed = 1.5

# Opción 2: Quedarte con feature
max_speed = 2.0

# Opción 3: Combinar (lo más común)
max_speed = 2.0  # Increased for better performance"
                :copyable="true"
              />
            </div>
          </div>

          <div class="resolution-step">
            <div class="resolution-number">3</div>
            <div class="resolution-content">
              <div class="resolution-title">Marcar como resuelto</div>
              <CodeBlock
                lang="bash"
                content="# Agregar archivo resuelto
git add motor_driver.py

# Completar el merge
git commit -m 'Merge feature/speed: resolved max_speed conflict'

# Ver estado
git status"
                :copyable="true"
              />
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Prevenir Conflictos">
          <strong>Comunícate con tu equipo:</strong> Evita que dos personas trabajen en el mismo
          archivo
          <br />
          <strong>Pull frecuentemente:</strong> Trae cambios de main a tu rama regularmente
          <br />
          <strong>Commits pequeños:</strong> Más fácil de resolver que cambios masivos
        </AlertBlock>
      </div>
    </div>

    <!-- ESTRATEGIAS -->
    <div class="section-group">
      <SectionTitle>6. Estrategias de Branching</SectionTitle>
      <TextBlock>
        Diferentes proyectos usan diferentes estrategias de branches. Aquí están las más comunes en
        robótica.
      </TextBlock>

      <div class="strategies-grid q-mt-md">
        <div class="strategy-card">
          <div class="strategy-header">
            <q-icon name="account_tree" color="green-4" />
            <span>Git Flow</span>
          </div>
          <div class="strategy-desc">Estrategia completa con múltiples ramas permanentes</div>
          <div class="strategy-branches">
            <div class="branch-item"><code>main</code> - Producción</div>
            <div class="branch-item"><code>develop</code> - Desarrollo</div>
            <div class="branch-item"><code>feature/*</code> - Nuevas features</div>
            <div class="branch-item"><code>hotfix/*</code> - Bugs urgentes</div>
          </div>
          <div class="strategy-use">
            <strong>Uso:</strong> Proyectos grandes con releases programados
          </div>
        </div>

        <div class="strategy-card">
          <div class="strategy-header">
            <q-icon name="linear_scale" color="blue-4" />
            <span>GitHub Flow</span>
          </div>
          <div class="strategy-desc">Estrategia simple: solo main + feature branches</div>
          <div class="strategy-branches">
            <div class="branch-item"><code>main</code> - Siempre deployable</div>
            <div class="branch-item"><code>feature/*</code> - Todo lo demás</div>
          </div>
          <div class="strategy-use"><strong>Uso:</strong> Proyectos pequeños, deploy continuo</div>
        </div>

        <div class="strategy-card">
          <div class="strategy-header">
            <q-icon name="science" color="purple-4" />
            <span>Trunk-Based</span>
          </div>
          <div class="strategy-desc">Todos trabajan en main, branches de vida muy corta</div>
          <div class="strategy-branches">
            <div class="branch-item"><code>main</code> - Todo el trabajo</div>
            <div class="branch-item"><code>feature/*</code> - Máximo 1-2 días</div>
          </div>
          <div class="strategy-use">
            <strong>Uso:</strong> Equipos experimentados, CI/CD robusto
          </div>
        </div>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://youtu.be/Romc22GgusU"
            title="Git Branches and Merging"
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
          <code>git switch -c</code>
          <span>Crear y cambiar a rama</span>
        </div>
        <div class="summary-item">
          <code>git branch</code>
          <span>Listar ramas</span>
        </div>
        <div class="summary-item">
          <code>git merge</code>
          <span>Fusionar rama a main</span>
        </div>
        <div class="summary-item">
          <code>git branch -d</code>
          <span>Eliminar rama</span>
        </div>
        <div class="summary-item">
          <code>git pull</code>
          <span>Actualizar antes de merge</span>
        </div>
        <div class="summary-item">
          <code>git add + commit</code>
          <span>Resolver conflictos</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Branches" class="q-mt-lg">
        ✅ NUNCA edites directamente en main
        <br />
        ✅ Crea una rama para cada feature/bugfix
        <br />
        ✅ Usa nombres descriptivos (<code>feature/add-camera</code>, no
        <code>test123</code>)
        <br />
        ✅ Haz <code>git pull</code> en main antes de merge
        <br />
        ✅ Elimina ramas después de fusionar
        <br />
        ✅ Resuelve conflictos con calma (lee el código, no adivines)
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

/* GOLDEN RULE */
.golden-rule {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
}

.rule-card {
  background: rgba(15, 23, 42, 0.6);
  border: 3px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1.5rem;
  text-align: center;
}

.rule-card.main-card {
  border-color: #22c55e;
}

.rule-card.feature-card {
  border-color: #fbbf24;
}

.rule-title {
  font-size: 1.5rem;
  font-weight: 700;
  font-family: 'Fira Code', monospace;
  color: #f1f5f9;
}

.rule-desc {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  width: 100%;
}

.rule-item {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.rule-command {
  padding: 0.75rem 1.5rem;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  color: #60a5fa;
  font-weight: 700;
}

.rule-arrow {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.arrow-label {
  color: #fca5a5;
  font-weight: 700;
  text-align: center;
}

/* LIFECYCLE VISUAL */
.lifecycle-visual {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.lifecycle-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.step-number {
  width: 50px;
  height: 50px;
  background: linear-gradient(135deg, #f97316, #ea580c);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.step-content {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
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

.lifecycle-arrow {
  display: flex;
  justify-content: center;
  padding: 0.5rem 0;
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

/* MERGE TYPES */
.merge-types {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.merge-type-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.merge-type-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.merge-type-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.merge-visual {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 1.5rem;
  border-radius: 12px;
}

.visual-label {
  font-weight: 700;
  color: #94a3b8;
  text-align: center;
}

.commit-chain {
  display: flex;
  gap: 0.5rem;
  justify-content: center;
  align-items: center;
}

.commit-dot {
  width: 45px;
  height: 45px;
  background: #3b82f6;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: white;
  font-size: 0.9rem;
}

.commit-dot.main {
  background: #22c55e;
}

.commit-dot.feature {
  background: #fbbf24;
}

.commit-dot.main-pointer {
  background: #22c55e;
  box-shadow: 0 0 15px rgba(34, 197, 94, 0.5);
  font-size: 0.75rem;
}

.commit-dot.feature-pointer {
  background: #fbbf24;
  box-shadow: 0 0 15px rgba(251, 191, 36, 0.5);
  font-size: 0.75rem;
}

.commit-dot.merge-commit {
  background: #a855f7;
}

.visual-arrow {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  color: #fbbf24;
  font-family: 'Fira Code', monospace;
}

.merge-note {
  padding: 1rem;
  border-radius: 8px;
  font-size: 0.9rem;
}

.merge-note.success {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid #22c55e;
  color: #86efac;
}

.merge-note.info {
  background: rgba(59, 130, 246, 0.1);
  border: 1px solid #3b82f6;
  color: #93c5fd;
}

/* CONFLICT SCENARIO */
.conflict-scenario {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid #ef4444;
  border-radius: 16px;
  padding: 2rem;
}

.scenario-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.3rem;
  font-weight: 700;
  color: #fca5a5;
  margin-bottom: 1.5rem;
}

.scenario-visual {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  margin-bottom: 1.5rem;
}

.scenario-branch {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.branch-label {
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  text-align: center;
  padding: 0.5rem;
  border-radius: 6px;
}

.branch-label.main {
  background: rgba(34, 197, 94, 0.2);
  color: #86efac;
}

.branch-label.feature {
  background: rgba(251, 191, 36, 0.2);
  color: #fde047;
}

.code-snippet {
  display: flex;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 8px;
  overflow: hidden;
}

.line-number {
  background: rgba(100, 116, 139, 0.3);
  padding: 0.75rem;
  color: #64748b;
  font-family: 'Fira Code', monospace;
}

.code-line {
  padding: 0.75rem;
  color: #cbd5e1;
  font-family: 'Fira Code', monospace;
  flex: 1;
}

.conflict-message {
  text-align: center;
  color: #fca5a5;
  font-weight: 700;
  font-size: 1.1rem;
}

/* RESOLUTION STEPS */
.resolution-steps {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.resolution-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.resolution-number {
  width: 50px;
  height: 50px;
  background: linear-gradient(135deg, #ef4444, #dc2626);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.resolution-content {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.resolution-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.resolution-desc {
  color: #94a3b8;
  margin-bottom: 1rem;
}

.conflict-file {
  background: rgba(0, 0, 0, 0.5);
  border: 1px solid #ef4444;
  border-radius: 8px;
  overflow: hidden;
}

.conflict-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem 1rem;
  background: rgba(239, 68, 68, 0.1);
  border-bottom: 1px solid #ef4444;
  font-family: 'Fira Code', monospace;
  color: #fca5a5;
}

.conflict-code {
  padding: 1rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.conflict-marker {
  color: #ef4444;
  font-weight: 700;
  margin: 0.5rem 0;
}

.conflict-content {
  padding: 0.5rem 1rem;
  margin: 0.25rem 0;
}

.conflict-content.main {
  background: rgba(34, 197, 94, 0.1);
  color: #86efac;
}

.conflict-content.feature {
  background: rgba(251, 191, 36, 0.1);
  color: #fde047;
}

/* STRATEGIES GRID */
.strategies-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.strategy-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.strategy-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.strategy-desc {
  color: #94a3b8;
  font-size: 0.95rem;
}

.strategy-branches {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.branch-item {
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  font-family: 'Fira Code', monospace;
  color: #cbd5e1;
  font-size: 0.9rem;
}

.strategy-use {
  color: #94a3b8;
  font-size: 0.9rem;
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
  .golden-rule {
    grid-template-columns: 1fr;
  }

  .rule-arrow {
    transform: rotate(90deg);
  }

  .merge-types {
    grid-template-columns: 1fr;
  }

  .scenario-visual {
    grid-template-columns: 1fr;
  }

  .conflict-icon {
    transform: rotate(90deg);
  }
}
</style>
