<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Los Pull Requests (PRs) son el corazón de la colaboración moderna en GitHub. No es solo
      "fusionar código", es un proceso de revisión, discusión y mejora continua. En proyectos de
      robótica open-source, los PRs son la forma estándar de contribuir.
    </TextBlock>

    <AlertBlock type="info" title="¿Qué es un Pull Request?">
      <strong>Propuesta de cambio:</strong> "Por favor, revisa y acepta mis cambios"
      <br />
      <strong>Code review:</strong> El equipo revisa tu código antes de fusionar
      <br />
      <strong>Discusión:</strong> Comentarios, sugerencias y mejoras
      <br />
      <strong>Calidad:</strong> Asegura que solo código revisado llegue a main
    </AlertBlock>

    <!-- METÁFORA -->
    <div class="section-group">
      <SectionTitle>1. La Metáfora: Propuesta de Ley</SectionTitle>
      <TextBlock>
        Imagina que el código principal (main) es la Constitución. Nadie, ni siquiera el líder del
        proyecto, escribe directamente en ella. Para hacer un cambio, redactas una "Propuesta de
        Ley" (Pull Request). Tus compañeros la leen, discuten, sugieren enmiendas y, solo cuando
        todos están de acuerdo, se fusiona.
      </TextBlock>

      <div class="pr-visual q-mt-md">
        <div class="pr-card">
          <div class="pr-header">
            <div class="pr-icon">
              <q-icon name="call_merge" size="2rem" />
            </div>
            <div class="pr-title">PR #45: Add autonomous navigation</div>
            <div class="pr-status open">OPEN</div>
          </div>

          <div class="pr-body">
            <div class="pr-description">
              "This PR implements the A* algorithm for obstacle avoidance. The robot can now
              navigate autonomously in complex environments..."
            </div>

            <div class="pr-meta">
              <div class="meta-item">
                <q-icon name="commit" size="sm" />
                <span>5 commits</span>
              </div>
              <div class="meta-item">
                <q-icon name="add" size="sm" color="green-4" />
                <span>+234</span>
              </div>
              <div class="meta-item">
                <q-icon name="remove" size="sm" color="red-4" />
                <span>-87</span>
              </div>
            </div>
          </div>

          <div class="pr-reviewers">
            <div class="reviewers-label">Reviewers:</div>
            <div class="reviewers-list">
              <div class="reviewer approved">
                <div class="reviewer-avatar">JS</div>
                <q-icon name="check_circle" color="green-4" />
              </div>
              <div class="reviewer pending">
                <div class="reviewer-avatar">MR</div>
                <q-icon name="schedule" color="orange-4" />
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- FLUJO DE TRABAJO -->
    <div class="section-group">
      <SectionTitle>2. Flujo de Trabajo con PRs</SectionTitle>
      <TextBlock>
        Este es el proceso completo desde que decides hacer un cambio hasta que se fusiona en main.
      </TextBlock>

      <div class="workflow-steps q-mt-md">
        <div class="workflow-step">
          <div class="step-icon" style="background: linear-gradient(135deg, #a855f7, #9333ea)">
            <q-icon name="fork_right" />
          </div>
          <div class="step-content">
            <div class="step-title">1. Fork (opcional)</div>
            <div class="step-desc">
              Si no tienes permisos de escritura, haz fork del repositorio a tu cuenta
            </div>
            <div class="step-note">Solo necesario para proyectos externos</div>
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-icon" style="background: linear-gradient(135deg, #3b82f6, #2563eb)">
            <q-icon name="call_split" />
          </div>
          <div class="step-content">
            <div class="step-title">2. Crear rama</div>
            <CodeBlock
              lang="bash"
              content="git checkout main
git pull origin main
git checkout -b feature/add-navigation"
              :copyable="true"
            />
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-icon" style="background: linear-gradient(135deg, #22c55e, #16a34a)">
            <q-icon name="code" />
          </div>
          <div class="step-content">
            <div class="step-title">3. Hacer cambios y commits</div>
            <CodeBlock
              lang="bash"
              content="# Editar archivos...
git add .
git commit -m 'feat: add A* pathfinding algorithm'
git commit -m 'test: add navigation unit tests'
git commit -m 'docs: update README with navigation info'"
              :copyable="true"
            />
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-icon" style="background: linear-gradient(135deg, #f97316, #ea580c)">
            <q-icon name="cloud_upload" />
          </div>
          <div class="step-content">
            <div class="step-title">4. Push a GitHub</div>
            <CodeBlock
              lang="bash"
              content="git push -u origin feature/add-navigation"
              :copyable="true"
            />
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-icon" style="background: linear-gradient(135deg, #ec4899, #db2777)">
            <q-icon name="publish" />
          </div>
          <div class="step-content">
            <div class="step-title">5. Crear Pull Request en GitHub</div>
            <div class="step-desc">
              Ve a GitHub → Tu repositorio → "Compare & pull request" → Completa título y
              descripción → "Create pull request"
            </div>
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-icon" style="background: linear-gradient(135deg, #fbbf24, #f59e0b)">
            <q-icon name="rate_review" />
          </div>
          <div class="step-content">
            <div class="step-title">6. Code Review</div>
            <div class="step-desc">
              El equipo revisa tu código, deja comentarios, aprueba o pide cambios
            </div>
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-icon" style="background: linear-gradient(135deg, #8b5cf6, #7c3aed)">
            <q-icon name="merge_type" />
          </div>
          <div class="step-content">
            <div class="step-title">7. Merge</div>
            <div class="step-desc">
              Una vez aprobado, el maintainer fusiona el PR a main. ¡Tu código está en producción!
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ANATOMÍA DE UN BUEN PR -->
    <div class="section-group">
      <SectionTitle>3. Anatomía de un Buen PR</SectionTitle>
      <TextBlock>
        Un PR bien hecho facilita la revisión y aumenta las probabilidades de aprobación rápida.
      </TextBlock>

      <div class="pr-anatomy q-mt-md">
        <div class="anatomy-card">
          <div class="anatomy-header">
            <q-icon name="title" color="blue-4" />
            <span>Título Descriptivo</span>
          </div>
          <div class="anatomy-example good">
            <div class="example-label">✅ Bueno</div>
            <code>feat: add A* pathfinding for autonomous navigation</code>
          </div>
          <div class="anatomy-example bad">
            <div class="example-label">❌ Malo</div>
            <code>Update code</code>
          </div>
        </div>

        <div class="anatomy-card">
          <div class="anatomy-header">
            <q-icon name="description" color="green-4" />
            <span>Descripción Clara</span>
          </div>
          <div class="anatomy-content">
            <strong>Incluye:</strong>
            <ul>
              <li>Qué cambia</li>
              <li>Por qué es necesario</li>
              <li>Cómo probarlo</li>
              <li>Screenshots/videos (si es UI)</li>
            </ul>
          </div>
        </div>

        <div class="anatomy-card">
          <div class="anatomy-header">
            <q-icon name="commit" color="purple-4" />
            <span>Commits Atómicos</span>
          </div>
          <div class="anatomy-content">
            <strong>Cada commit debe:</strong>
            <ul>
              <li>Hacer una cosa</li>
              <li>Tener mensaje claro</li>
              <li>Compilar y pasar tests</li>
            </ul>
          </div>
        </div>

        <div class="anatomy-card">
          <div class="anatomy-header">
            <q-icon name="compress" color="orange-4" />
            <span>Tamaño Razonable</span>
          </div>
          <div class="anatomy-content">
            <strong>Ideal:</strong> 200-400 líneas
            <br />
            <strong>Máximo:</strong> ~1000 líneas
            <br />
            <br />
            PRs grandes son difíciles de revisar. Divide en múltiples PRs si es necesario.
          </div>
        </div>
      </div>
    </div>

    <!-- CODE REVIEW -->
    <div class="section-group">
      <SectionTitle>4. Code Review: Feedback Constructivo</SectionTitle>
      <TextBlock>
        El code review no es para juzgarte a ti, es para mejorar el código. Es normal que te pidan
        cambios. No te defiendas; explica tus razones o acepta la mejora.
      </TextBlock>

      <div class="review-simulation q-mt-md">
        <div class="simulation-header">
          <q-icon name="chat" />
          <span>Ejemplo de conversación en PR</span>
        </div>

        <div class="review-thread">
          <div class="code-snippet">
            <div class="code-header">src/navigation.py</div>
            <div class="code-line removed">- max_speed = 2.0</div>
            <div class="code-line added">+ max_speed = 5.0</div>
          </div>

          <div class="comment reviewer">
            <div class="comment-author">
              <div class="author-avatar">MR</div>
              <span>Senior Dev</span>
            </div>
            <div class="comment-body">
              5.0 m/s seems too fast for indoor navigation. Safety regulations limit to 1.5 m/s. Can
              you update this?
            </div>
          </div>

          <div class="comment author">
            <div class="comment-author">
              <div class="author-avatar you">You</div>
              <span>You</span>
            </div>
            <div class="comment-body">
              Good catch! I was testing in simulation. Updated to 1.5 m/s and added a config
              parameter for different environments.
            </div>
          </div>

          <div class="comment reviewer">
            <div class="comment-author">
              <div class="author-avatar">MR</div>
              <span>Senior Dev</span>
            </div>
            <div class="comment-body">
              <q-icon name="check_circle" color="green-4" />
              Perfect! Approved.
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Reglas de Oro del Code Review">
          <strong>Como autor:</strong> Acepta feedback con humildad. "Tú no eres tu código"
          <br />
          <strong>Como revisor:</strong> Critica el código, no a la persona. Sé constructivo
          <br />
          <strong>Ambos:</strong> Asume buenas intenciones. Todos quieren mejorar el proyecto
        </AlertBlock>
      </div>
    </div>

    <!-- MERGE STRATEGIES -->
    <div class="section-group">
      <SectionTitle>5. Estrategias de Merge</SectionTitle>
      <TextBlock>
        GitHub ofrece 3 formas de fusionar un PR. Cada una tiene ventajas y desventajas.
      </TextBlock>

      <div class="merge-strategies q-mt-md">
        <div class="strategy-card">
          <div class="strategy-header">
            <q-icon name="merge" color="green-4" size="2rem" />
            <span>Merge Commit</span>
          </div>
          <div class="strategy-desc">
            Crea un commit de merge que preserva toda la historia de la rama
          </div>
          <div class="strategy-visual">
            <div class="commit-line">A → B → C → D → M</div>
            <div class="commit-line branch">↗ E → F ↗</div>
          </div>
          <div class="strategy-pros">
            <strong>✅ Pros:</strong> Historia completa, fácil de revertir PR completo
          </div>
          <div class="strategy-cons">
            <strong>❌ Cons:</strong> Historial puede verse "sucio" con muchos merges
          </div>
        </div>

        <div class="strategy-card">
          <div class="strategy-header">
            <q-icon name="compress" color="blue-4" size="2rem" />
            <span>Squash and Merge</span>
          </div>
          <div class="strategy-desc">
            Combina todos los commits del PR en uno solo antes de fusionar
          </div>
          <div class="strategy-visual">
            <div class="commit-line">A → B → C → D → S</div>
            <div class="commit-note">S = E + F + G (squashed)</div>
          </div>
          <div class="strategy-pros">
            <strong>✅ Pros:</strong> Historial limpio, un commit por feature
          </div>
          <div class="strategy-cons">
            <strong>❌ Cons:</strong> Pierdes historia detallada de la rama
          </div>
        </div>

        <div class="strategy-card">
          <div class="strategy-header">
            <q-icon name="fast_forward" color="purple-4" size="2rem" />
            <span>Rebase and Merge</span>
          </div>
          <div class="strategy-desc">
            Reaplica los commits de la rama sobre main, sin crear merge commit
          </div>
          <div class="strategy-visual">
            <div class="commit-line">A → B → C → D → E → F</div>
            <div class="commit-note">Historial lineal</div>
          </div>
          <div class="strategy-pros"><strong>✅ Pros:</strong> Historial perfectamente lineal</div>
          <div class="strategy-cons">
            <strong>❌ Cons:</strong> Reescribe historia, difícil de revertir PR completo
          </div>
        </div>
      </div>
    </div>

    <!-- BEST PRACTICES -->
    <div class="section-group">
      <SectionTitle>6. Best Practices</SectionTitle>

      <div class="practices-grid">
        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="speed" size="2rem" color="green-4" />
          </div>
          <div class="practice-title">PRs Pequeños</div>
          <div class="practice-desc">
            Más fácil de revisar, más rápido de aprobar. Divide features grandes en múltiples PRs.
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="science" size="2rem" color="blue-4" />
          </div>
          <div class="practice-title">Tests Incluidos</div>
          <div class="practice-desc">
            Siempre incluye tests para tu código. Los PRs sin tests son rechazados en proyectos
            serios.
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="description" size="2rem" color="purple-4" />
          </div>
          <div class="practice-title">Documentación</div>
          <div class="practice-desc">
            Actualiza README, comentarios y docs. El código sin documentación es código muerto.
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="check_circle" size="2rem" color="orange-4" />
          </div>
          <div class="practice-title">CI/CD Passing</div>
          <div class="practice-desc">
            Asegúrate que todos los checks automáticos (tests, linters) pasen antes de pedir review.
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="update" size="2rem" color="pink-4" />
          </div>
          <div class="practice-title">Mantén Actualizado</div>
          <div class="practice-desc">
            Si main avanza mientras tu PR está abierto, haz merge/rebase para evitar conflictos.
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="chat" size="2rem" color="cyan-4" />
          </div>
          <div class="practice-title">Responde Rápido</div>
          <div class="practice-desc">
            Responde a comentarios en &lt;24h. PRs abandonados se cierran automáticamente.
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
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Pull Requests and Code Review"
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
          <code>Fork → Branch → Commit</code>
          <span>Preparar cambios</span>
        </div>
        <div class="summary-item">
          <code>Push → Create PR</code>
          <span>Proponer cambios</span>
        </div>
        <div class="summary-item">
          <code>Code Review</code>
          <span>Recibir feedback</span>
        </div>
        <div class="summary-item">
          <code>Merge</code>
          <span>Fusionar a main</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Pull Requests" class="q-mt-lg">
        ✅ Título descriptivo con prefijo (feat:, fix:, docs:)
        <br />
        ✅ Descripción clara (qué, por qué, cómo probar)
        <br />
        ✅ PR pequeño (&lt;400 líneas idealmente)
        <br />
        ✅ Tests incluidos y pasando
        <br />
        ✅ Documentación actualizada
        <br />
        ✅ CI/CD checks en verde
        <br />
        ✅ Responde a comentarios en &lt;24h
        <br />
        ✅ Acepta feedback con humildad
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

/* PR VISUAL */
.pr-visual {
  display: flex;
  justify-content: center;
}

.pr-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(148, 163, 184, 0.3);
  border-radius: 16px;
  padding: 2rem;
  max-width: 700px;
  width: 100%;
}

.pr-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  margin-bottom: 1.5rem;
  padding-bottom: 1rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.pr-icon {
  color: #22c55e;
}

.pr-title {
  flex: 1;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.pr-status {
  padding: 0.5rem 1rem;
  border-radius: 6px;
  font-weight: 700;
  font-size: 0.75rem;
}

.pr-status.open {
  background: #22c55e;
  color: white;
}

.pr-body {
  margin-bottom: 1.5rem;
}

.pr-description {
  color: #cbd5e1;
  font-style: italic;
  margin-bottom: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.pr-meta {
  display: flex;
  gap: 1.5rem;
}

.meta-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  color: #94a3b8;
  font-size: 0.9rem;
}

.pr-reviewers {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding-top: 1rem;
  border-top: 1px dashed rgba(148, 163, 184, 0.2);
}

.reviewers-label {
  color: #94a3b8;
  font-size: 0.9rem;
}

.reviewers-list {
  display: flex;
  gap: 1rem;
}

.reviewer {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.reviewer-avatar {
  width: 35px;
  height: 35px;
  border-radius: 50%;
  background: #3b82f6;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: white;
  font-size: 0.85rem;
}

/* WORKFLOW STEPS */
.workflow-steps {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.workflow-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.step-icon {
  width: 55px;
  height: 55px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
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
  margin-bottom: 0.75rem;
}

.step-desc {
  color: #cbd5e1;
  margin-bottom: 1rem;
}

.step-note {
  color: #64748b;
  font-size: 0.85rem;
  font-style: italic;
}

/* PR ANATOMY */
.pr-anatomy {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.anatomy-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.anatomy-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
}

.anatomy-example {
  padding: 1rem;
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.anatomy-example.good {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid #22c55e;
  color: #86efac;
}

.anatomy-example.bad {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid #ef4444;
  color: #fca5a5;
}

.example-label {
  font-weight: 700;
  margin-bottom: 0.5rem;
}

.anatomy-content {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.anatomy-content ul {
  margin: 0.5rem 0 0 1.5rem;
  padding: 0;
}

/* REVIEW SIMULATION */
.review-simulation {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.simulation-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
  font-size: 1.1rem;
}

.review-thread {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.code-snippet {
  background: rgba(0, 0, 0, 0.5);
  border-radius: 8px;
  overflow: hidden;
}

.code-header {
  padding: 0.5rem 1rem;
  background: rgba(100, 116, 139, 0.3);
  color: #94a3b8;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

.code-line {
  padding: 0.5rem 1rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.code-line.removed {
  background: rgba(239, 68, 68, 0.1);
  color: #fca5a5;
}

.code-line.added {
  background: rgba(34, 197, 94, 0.1);
  color: #86efac;
}

.comment {
  background: rgba(30, 41, 59, 0.6);
  border-radius: 8px;
  padding: 1rem;
  border-left: 3px solid;
}

.comment.reviewer {
  border-color: #f97316;
}

.comment.author {
  border-color: #3b82f6;
  margin-left: 2rem;
}

.comment-author {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  margin-bottom: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
}

.author-avatar {
  width: 30px;
  height: 30px;
  border-radius: 50%;
  background: #f97316;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.75rem;
  color: white;
}

.author-avatar.you {
  background: #3b82f6;
}

.comment-body {
  color: #cbd5e1;
  font-size: 0.9rem;
}

/* MERGE STRATEGIES */
.merge-strategies {
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
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.strategy-desc {
  text-align: center;
  color: #cbd5e1;
  font-size: 0.95rem;
}

.strategy-visual {
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  text-align: center;
}

.commit-line {
  color: #cbd5e1;
  margin-bottom: 0.5rem;
}

.commit-line.branch {
  color: #fbbf24;
}

.commit-note {
  color: #64748b;
  font-size: 0.85rem;
  margin-top: 0.5rem;
}

.strategy-pros,
.strategy-cons {
  font-size: 0.9rem;
}

.strategy-pros {
  color: #86efac;
}

.strategy-cons {
  color: #fca5a5;
}

/* PRACTICES GRID */
.practices-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
}

.practice-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  gap: 1rem;
}

.practice-title {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
}

.practice-desc {
  color: #cbd5e1;
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
  font-size: 0.9rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}
</style>
