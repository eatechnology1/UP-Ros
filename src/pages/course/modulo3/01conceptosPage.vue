<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Git es el sistema de control de versiones más usado en el mundo. En robótica, donde trabajas
      con múltiples archivos de código, configuración y parámetros, Git te permite
      <strong>viajar en el tiempo</strong>, experimentar sin miedo, y colaborar con tu equipo sin
      pisarte los pies. <br /><br />
      No es solo "guardar archivos". Es una máquina del tiempo para tu código.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué Git en Robótica?">
      <strong>Experimentación segura:</strong> Prueba nuevos algoritmos sin romper lo que funciona
      <br />
      <strong>Colaboración:</strong> Múltiples personas trabajando en el mismo robot
      <br />
      <strong>Historial completo:</strong> Saber qué cambió, cuándo y por qué
      <br />
      <strong>Recuperación:</strong> Volver a una versión que funcionaba <br /><br />
      En ROS 2, cada paquete debería estar en Git. Es el estándar de la industria.
    </AlertBlock>

    <!-- LAS 3 ZONAS -->
    <div class="section-group">
      <SectionTitle>1. Las Tres Zonas de Git</SectionTitle>
      <TextBlock>
        Git no guarda todo automáticamente. Tienes que mover tus archivos conscientemente a través
        de tres zonas. Es como preparar un paquete para envío:
      </TextBlock>

      <div class="zones-visual q-mt-lg">
        <div class="zone-card working">
          <div class="zone-header">
            <q-icon name="edit_document" size="2.5rem" />
            <div class="zone-title">Working Directory</div>
          </div>
          <div class="zone-content">
            <div class="zone-desc">Tu mesa de trabajo</div>
            <div class="zone-files">
              <div class="file-item modified">
                <q-icon name="description" size="sm" />
                <span>robot_controller.py</span>
                <span class="file-status">M</span>
              </div>
              <div class="file-item new">
                <q-icon name="description" size="sm" />
                <span>lidar_driver.py</span>
                <span class="file-status">?</span>
              </div>
            </div>
          </div>
          <div class="zone-action">
            <code>git add .</code>
          </div>
        </div>

        <div class="zone-arrow">
          <q-icon name="arrow_forward" size="3rem" color="orange-4" />
        </div>

        <div class="zone-card staging">
          <div class="zone-header">
            <q-icon name="shopping_cart" size="2.5rem" />
            <div class="zone-title">Staging Area</div>
          </div>
          <div class="zone-content">
            <div class="zone-desc">Preparado para commit</div>
            <div class="zone-files">
              <div class="file-item staged">
                <q-icon name="check_circle" size="sm" color="green-4" />
                <span>robot_controller.py</span>
              </div>
              <div class="file-item staged">
                <q-icon name="check_circle" size="sm" color="green-4" />
                <span>lidar_driver.py</span>
              </div>
            </div>
          </div>
          <div class="zone-action">
            <code>git commit -m "mensaje"</code>
          </div>
        </div>

        <div class="zone-arrow">
          <q-icon name="arrow_forward" size="3rem" color="orange-4" />
        </div>

        <div class="zone-card repository">
          <div class="zone-header">
            <q-icon name="inventory_2" size="2.5rem" />
            <div class="zone-title">Repository</div>
          </div>
          <div class="zone-content">
            <div class="zone-desc">Historial permanente</div>
            <div class="commit-history">
              <div class="commit-item">
                <div class="commit-hash">a1b2c3d</div>
                <div class="commit-msg">Add LIDAR driver</div>
              </div>
              <div class="commit-item">
                <div class="commit-hash">9f8e7d6</div>
                <div class="commit-msg">Fix motor speed</div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="warning" title="Staging Area: ¿Por qué existe?">
          Podrías pensar "¿por qué no commit directo?". La staging area te permite
          <strong>seleccionar</strong> qué cambios incluir. Imagina que modificaste 10 archivos pero
          solo quieres commitear 3. Staging te da ese control quirúrgico.
        </AlertBlock>
      </div>
    </div>

    <!-- ANATOMÍA DE UN COMMIT -->
    <div class="section-group">
      <SectionTitle>2. Anatomía de un Commit</SectionTitle>
      <TextBlock>
        Un <strong>commit</strong> no es solo "guardar". Es una captura completa del estado de todo
        tu proyecto en ese instante, con un identificador único (hash SHA-1) y metadatos.
      </TextBlock>

      <div class="commit-anatomy q-mt-md">
        <div class="commit-visual">
          <div class="commit-node current">
            <div class="commit-circle"></div>
            <div class="commit-details">
              <div class="commit-header">
                <span class="commit-hash">commit a1b2c3d4e5f6</span>
                <span class="commit-time">Hace 2 minutos</span>
              </div>
              <div class="commit-message">✨ Add LIDAR sensor integration</div>
              <div class="commit-meta">
                <div class="meta-row">
                  <q-icon name="person" size="sm" />
                  <span>Author: Alexander &lt;alex@robot.com&gt;</span>
                </div>
                <div class="meta-row">
                  <q-icon name="calendar_today" size="sm" />
                  <span>Date: 2026-01-11 03:40:00</span>
                </div>
              </div>
              <div class="commit-changes">
                <div class="change-item added">
                  <q-icon name="add" size="sm" />
                  <span>src/lidar_driver.py</span>
                  <span class="change-stats">+45</span>
                </div>
                <div class="change-item modified">
                  <q-icon name="edit" size="sm" />
                  <span>config/params.yaml</span>
                  <span class="change-stats">+3 -1</span>
                </div>
              </div>
            </div>
          </div>

          <div class="commit-line"></div>

          <div class="commit-node previous">
            <div class="commit-circle"></div>
            <div class="commit-details">
              <div class="commit-header">
                <span class="commit-hash">commit 9f8e7d6c5b4a</span>
              </div>
              <div class="commit-message">🐛 Fix motor speed calculation</div>
            </div>
          </div>
        </div>

        <div class="commit-info">
          <div class="info-card">
            <div class="info-header">
              <q-icon name="fingerprint" color="purple-4" />
              <span>Hash (ID Único)</span>
            </div>
            <div class="info-content">
              Identificador criptográfico único. Los primeros 7 caracteres suelen ser suficientes
              (<code>a1b2c3d</code>).
            </div>
          </div>

          <div class="info-card">
            <div class="info-header">
              <q-icon name="message" color="blue-4" />
              <span>Mensaje del Commit</span>
            </div>
            <div class="info-content">
              <strong>Imperativo:</strong> "Add feature" no "Added feature"
              <br />
              <strong>Conciso:</strong> Máximo 50 caracteres en la primera línea
              <br />
              <strong>Descriptivo:</strong> Explica QUÉ y POR QUÉ, no CÓMO
            </div>
          </div>

          <div class="info-card">
            <div class="info-header">
              <q-icon name="account_tree" color="green-4" />
              <span>Parent Commit</span>
            </div>
            <div class="info-content">
              Cada commit apunta a su padre, formando una cadena. Esto es lo que permite viajar en
              el tiempo.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- BRANCHES -->
    <div class="section-group">
      <SectionTitle>3. Branches: Universos Paralelos</SectionTitle>
      <TextBlock>
        Las ramas (branches) son la característica más poderosa de Git. Te permiten crear realidades
        alternativas donde puedes experimentar sin afectar el código principal.
      </TextBlock>

      <div class="branch-visual q-mt-md">
        <div class="branch-diagram">
          <svg viewBox="0 0 600 250" class="branch-svg">
            <!-- MAIN BRANCH -->
            <path d="M 50 180 L 550 180" stroke="#22c55e" stroke-width="4" fill="none" />
            <text
              x="60"
              y="165"
              fill="#22c55e"
              font-family="monospace"
              font-weight="bold"
              font-size="16"
            >
              main
            </text>

            <!-- COMMITS ON MAIN -->
            <circle cx="100" cy="180" r="8" fill="#22c55e" />
            <circle cx="200" cy="180" r="8" fill="#22c55e" />
            <circle cx="500" cy="180" r="8" fill="#22c55e" />

            <!-- FEATURE BRANCH -->
            <path
              d="M 200 180 Q 250 180 250 100 L 400 100 Q 450 100 450 160"
              stroke="#fbbf24"
              stroke-width="4"
              fill="none"
              stroke-dasharray="10,5"
            />
            <text
              x="260"
              y="85"
              fill="#fbbf24"
              font-family="monospace"
              font-weight="bold"
              font-size="16"
            >
              feature/lidar
            </text>

            <!-- COMMITS ON FEATURE -->
            <circle cx="300" cy="100" r="8" fill="#fbbf24" />
            <circle cx="350" cy="100" r="8" fill="#fbbf24" />

            <!-- MERGE POINT -->
            <circle cx="500" cy="180" r="14" fill="none" stroke="#f97316" stroke-width="3" />
            <text x="460" y="210" fill="#f97316" font-family="monospace" font-size="14">merge</text>

            <!-- BUGFIX BRANCH -->
            <path
              d="M 200 180 Q 250 180 250 220 L 350 220"
              stroke="#ef4444"
              stroke-width="4"
              fill="none"
              stroke-dasharray="10,5"
            />
            <text
              x="260"
              y="240"
              fill="#ef4444"
              font-family="monospace"
              font-weight="bold"
              font-size="16"
            >
              bugfix/motor
            </text>
            <circle cx="300" cy="220" r="8" fill="#ef4444" />
          </svg>
        </div>

        <div class="branch-explanation">
          <div class="branch-card">
            <div class="branch-name main">
              <q-icon name="verified" />
              <span>main</span>
            </div>
            <div class="branch-desc">
              Código estable y funcional. Siempre debe compilar y pasar tests. Es lo que se
              despliega en producción.
            </div>
          </div>

          <div class="branch-card">
            <div class="branch-name feature">
              <q-icon name="science" />
              <span>feature/*</span>
            </div>
            <div class="branch-desc">
              Nuevas funcionalidades. Puedes romper todo aquí sin afectar main. Cuando funcione, se
              fusiona (merge).
            </div>
          </div>

          <div class="branch-card">
            <div class="branch-name bugfix">
              <q-icon name="bug_report" />
              <span>bugfix/*</span>
            </div>
            <div class="branch-desc">
              Correcciones de errores. Se crean desde main, se arregla el bug, y se fusiona de
              vuelta.
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="Comandos básicos de branches"
          lang="bash"
          content="# Crear nueva rama
git branch feature/lidar

# Cambiar a la rama
git checkout feature/lidar

# Crear y cambiar en un comando
git checkout -b feature/lidar

# Ver todas las ramas
git branch -a

# Fusionar rama a main
git checkout main
git merge feature/lidar

# Eliminar rama después de merge
git branch -d feature/lidar"
          :copyable="true"
        />
      </div>
    </div>

    <!-- GIT VS GITHUB -->
    <div class="section-group">
      <SectionTitle>4. Git ≠ GitHub</SectionTitle>
      <TextBlock>
        El error más común de principiantes: confundir Git con GitHub. Son cosas completamente
        diferentes que trabajan juntas.
      </TextBlock>

      <div class="comparison-grid q-mt-md">
        <div class="comparison-card git">
          <div class="comparison-header">
            <q-icon name="terminal" size="3rem" />
            <div class="comparison-title">Git</div>
          </div>
          <div class="comparison-content">
            <div class="comparison-item">
              <q-icon name="check_circle" color="green-4" />
              <span>Software instalado en tu PC</span>
            </div>
            <div class="comparison-item">
              <q-icon name="check_circle" color="green-4" />
              <span>Funciona 100% offline</span>
            </div>
            <div class="comparison-item">
              <q-icon name="check_circle" color="green-4" />
              <span>Gratis y open source</span>
            </div>
            <div class="comparison-item">
              <q-icon name="check_circle" color="green-4" />
              <span>Creado por Linus Torvalds (2005)</span>
            </div>
          </div>
          <CodeBlock
            lang="bash"
            content="# Instalar Git
sudo apt install git

# Verificar instalación
git --version"
            :copyable="true"
          />
        </div>

        <div class="comparison-card github">
          <div class="comparison-header">
            <q-icon name="cloud" size="3rem" />
            <div class="comparison-title">GitHub</div>
          </div>
          <div class="comparison-content">
            <div class="comparison-item">
              <q-icon name="check_circle" color="blue-4" />
              <span>Servicio web en la nube</span>
            </div>
            <div class="comparison-item">
              <q-icon name="check_circle" color="blue-4" />
              <span>Requiere internet</span>
            </div>
            <div class="comparison-item">
              <q-icon name="check_circle" color="blue-4" />
              <span>Propiedad de Microsoft</span>
            </div>
            <div class="comparison-item">
              <q-icon name="check_circle" color="blue-4" />
              <span>Colaboración + Pull Requests + Issues</span>
            </div>
          </div>
          <div class="github-alternatives">
            <strong>Alternativas:</strong> GitLab, Bitbucket, Gitea
          </div>
        </div>
      </div>

      <div class="sync-visual q-mt-lg">
        <div class="sync-local">
          <q-icon name="computer" size="4rem" color="orange-4" />
          <div class="sync-label">Tu PC (Git)</div>
        </div>

        <div class="sync-arrows">
          <div class="sync-arrow push">
            <code>git push</code>
            <q-icon name="arrow_forward" size="2rem" color="blue-4" />
          </div>
          <div class="sync-arrow pull">
            <q-icon name="arrow_back" size="2rem" color="green-4" />
            <code>git pull</code>
          </div>
        </div>

        <div class="sync-remote">
          <q-icon name="cloud_circle" size="4rem" color="blue-4" />
          <div class="sync-label">GitHub (Nube)</div>
        </div>
      </div>
    </div>

    <!-- FLUJO DE TRABAJO -->
    <div class="section-group">
      <SectionTitle>5. Flujo de Trabajo Típico</SectionTitle>
      <TextBlock> Este es el flujo que usarás día a día en tus proyectos de robótica: </TextBlock>

      <div class="workflow-steps q-mt-md">
        <div class="workflow-step">
          <div class="step-number">1</div>
          <div class="step-content">
            <div class="step-title">Clonar o Inicializar</div>
            <CodeBlock
              lang="bash"
              content="# Proyecto nuevo
git init
git add .
git commit -m 'Initial commit'

# Proyecto existente
git clone https://github.com/user/robot_ws.git"
              :copyable="true"
            />
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-number">2</div>
          <div class="step-content">
            <div class="step-title">Crear Rama para Nueva Feature</div>
            <CodeBlock
              lang="bash"
              content="git checkout -b feature/add-camera
# Ahora estás en la rama feature/add-camera"
              :copyable="true"
            />
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-number">3</div>
          <div class="step-content">
            <div class="step-title">Hacer Cambios y Commits</div>
            <CodeBlock
              lang="bash"
              content="# Editar archivos...
# Verificar cambios
git status

# Agregar a staging
git add src/camera_driver.py config/params.yaml

# Commit
git commit -m 'Add camera driver with ROS 2 integration'"
              :copyable="true"
            />
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-number">4</div>
          <div class="step-content">
            <div class="step-title">Subir a GitHub</div>
            <CodeBlock
              lang="bash"
              content="# Primera vez (crear rama remota)
git push -u origin feature/add-camera

# Siguientes veces
git push"
              :copyable="true"
            />
          </div>
        </div>

        <div class="workflow-step">
          <div class="step-number">5</div>
          <div class="step-content">
            <div class="step-title">Merge a Main</div>
            <CodeBlock
              lang="bash"
              content="# Volver a main
git checkout main

# Traer últimos cambios
git pull

# Fusionar feature
git merge feature/add-camera

# Subir a GitHub
git push"
              :copyable="true"
            />
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
            <q-icon name="message" color="blue-4" size="2rem" />
          </div>
          <div class="practice-title">Mensajes de Commit Claros</div>
          <div class="practice-content">
            <div class="practice-good">
              ✅ <code>Add LIDAR driver for TurtleBot3</code>
              <br />
              ✅ <code>Fix motor speed calculation bug</code>
            </div>
            <div class="practice-bad">
              ❌ <code>update</code>
              <br />
              ❌ <code>fix stuff</code>
              <br />
              ❌ <code>asdfasdf</code>
            </div>
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="commit" color="green-4" size="2rem" />
          </div>
          <div class="practice-title">Commits Pequeños y Frecuentes</div>
          <div class="practice-content">
            Mejor 10 commits pequeños que 1 commit gigante. Cada commit debe ser una unidad lógica
            de cambio.
            <br /><br />
            <strong>Regla:</strong> Si no puedes describir el commit en una línea, probablemente es
            muy grande.
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="account_tree" color="purple-4" size="2rem" />
          </div>
          <div class="practice-title">Nunca Commitear a Main Directamente</div>
          <div class="practice-content">
            Siempre trabaja en ramas. Main debe ser sagrado: solo código probado y funcional.
            <br /><br />
            <code>feature/*</code> → desarrollo
            <br />
            <code>bugfix/*</code> → correcciones
            <br />
            <code>hotfix/*</code> → emergencias
          </div>
        </div>

        <div class="practice-card">
          <div class="practice-icon">
            <q-icon name="sync" color="orange-4" size="2rem" />
          </div>
          <div class="practice-title">Pull Antes de Push</div>
          <div class="practice-content">
            Antes de subir tus cambios, siempre trae los últimos cambios del equipo:
            <br /><br />
            <code>git pull</code>
            <br />
            <code>git push</code>
            <br /><br />
            Esto evita conflictos innecesarios.
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
            title="Git Fundamentals"
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
          <code>git init</code>
          <span>Inicializar repositorio</span>
        </div>
        <div class="summary-item">
          <code>git add</code>
          <span>Working → Staging</span>
        </div>
        <div class="summary-item">
          <code>git commit</code>
          <span>Staging → Repository</span>
        </div>
        <div class="summary-item">
          <code>git branch</code>
          <span>Crear/listar ramas</span>
        </div>
        <div class="summary-item">
          <code>git checkout</code>
          <span>Cambiar de rama</span>
        </div>
        <div class="summary-item">
          <code>git merge</code>
          <span>Fusionar ramas</span>
        </div>
        <div class="summary-item">
          <code>git push</code>
          <span>Local → GitHub</span>
        </div>
        <div class="summary-item">
          <code>git pull</code>
          <span>GitHub → Local</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Git" class="q-mt-lg">
        ✅ Entiendo las 3 zonas (Working, Staging, Repository)
        <br />
        ✅ Sé qué es un commit y cómo escribir buenos mensajes
        <br />
        ✅ Puedo crear y fusionar branches
        <br />
        ✅ Entiendo la diferencia entre Git y GitHub
        <br />
        ✅ Conozco el flujo de trabajo básico
        <br />
        ✅ Sigo las best practices (commits pequeños, mensajes claros, branches)
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

/* ZONES VISUAL */
.zones-visual {
  display: grid;
  grid-template-columns: 1fr auto 1fr auto 1fr;
  gap: 1rem;
  align-items: start;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.zone-card {
  background: rgba(30, 41, 59, 0.8);
  border: 2px solid;
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
  transition: transform 0.3s;
}

.zone-card:hover {
  transform: translateY(-5px);
}

.zone-card.working {
  border-color: #ef4444;
}

.zone-card.staging {
  border-color: #fbbf24;
}

.zone-card.repository {
  border-color: #22c55e;
}

.zone-header {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  color: #f1f5f9;
}

.zone-title {
  font-weight: 700;
  font-size: 1.1rem;
}

.zone-content {
  flex: 1;
}

.zone-desc {
  text-align: center;
  color: #94a3b8;
  font-size: 0.9rem;
  margin-bottom: 1rem;
}

.zone-files {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.file-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #cbd5e1;
}

.file-status {
  margin-left: auto;
  font-weight: 700;
}

.file-item.modified .file-status {
  color: #fbbf24;
}

.file-item.new .file-status {
  color: #22c55e;
}

.commit-history {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.commit-item {
  padding: 0.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  border-left: 3px solid #22c55e;
}

.commit-hash {
  font-family: 'Fira Code', monospace;
  font-size: 0.75rem;
  color: #94a3b8;
}

.commit-msg {
  font-size: 0.85rem;
  color: #cbd5e1;
}

.zone-action {
  text-align: center;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  color: #60a5fa;
  font-weight: 700;
}

.zone-arrow {
  display: flex;
  align-items: center;
  justify-content: center;
}

/* COMMIT ANATOMY */
.commit-anatomy {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 2rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.commit-visual {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.commit-node {
  display: flex;
  gap: 1rem;
}

.commit-node.current .commit-circle {
  background: #f97316;
  box-shadow: 0 0 15px rgba(249, 115, 22, 0.5);
}

.commit-node.previous {
  opacity: 0.5;
}

.commit-node.previous .commit-circle {
  background: #64748b;
}

.commit-circle {
  width: 16px;
  height: 16px;
  border-radius: 50%;
  flex-shrink: 0;
  margin-top: 4px;
}

.commit-line {
  width: 2px;
  height: 100%;
  background: rgba(148, 163, 184, 0.3);
  margin-left: 7px;
}

.commit-details {
  flex: 1;
}

.commit-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.5rem;
}

.commit-hash {
  font-family: 'Fira Code', monospace;
  color: #f97316;
  font-weight: 700;
  font-size: 0.9rem;
}

.commit-time {
  color: #94a3b8;
  font-size: 0.85rem;
}

.commit-message {
  color: #f1f5f9;
  font-weight: 700;
  font-size: 1.1rem;
  margin-bottom: 0.75rem;
}

.commit-meta {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
  margin-bottom: 0.75rem;
}

.meta-row {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  color: #94a3b8;
  font-size: 0.85rem;
}

.commit-changes {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 0.75rem;
  border-radius: 8px;
}

.change-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

.change-item.added {
  color: #22c55e;
}

.change-item.modified {
  color: #fbbf24;
}

.change-stats {
  margin-left: auto;
  font-weight: 700;
}

.commit-info {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.info-card {
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1rem;
}

.info-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.info-content {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

/* BRANCH VISUAL */
.branch-visual {
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.branch-diagram {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 2rem;
}

.branch-svg {
  width: 100%;
  height: auto;
}

.branch-explanation {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.branch-card {
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1rem;
}

.branch-name {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  font-size: 1rem;
  margin-bottom: 0.5rem;
}

.branch-name.main {
  color: #22c55e;
}

.branch-name.feature {
  color: #fbbf24;
}

.branch-name.bugfix {
  color: #ef4444;
}

.branch-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

/* COMPARISON GRID */
.comparison-grid {
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

.comparison-card.git {
  border-color: #f97316;
}

.comparison-card.github {
  border-color: #3b82f6;
}

.comparison-header {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  color: #f1f5f9;
}

.comparison-title {
  font-size: 1.5rem;
  font-weight: 700;
}

.comparison-content {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  flex: 1;
}

.comparison-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  color: #cbd5e1;
  font-size: 0.95rem;
}

.github-alternatives {
  padding: 1rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #93c5fd;
  font-size: 0.9rem;
}

/* SYNC VISUAL */
.sync-visual {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 3rem 2rem;
}

.sync-local,
.sync-remote {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.sync-label {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.sync-arrows {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.sync-arrow {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-family: 'Fira Code', monospace;
  font-weight: 700;
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
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

/* PRACTICES GRID */
.practices-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.practice-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.practice-icon {
  display: flex;
  justify-content: center;
}

.practice-title {
  font-weight: 700;
  color: #f1f5f9;
  text-align: center;
  font-size: 1.05rem;
}

.practice-content {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

.practice-good {
  color: #22c55e;
  margin-bottom: 0.5rem;
}

.practice-bad {
  color: #ef4444;
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
  .zones-visual {
    grid-template-columns: 1fr;
  }

  .zone-arrow {
    transform: rotate(90deg);
  }

  .commit-anatomy {
    grid-template-columns: 1fr;
  }

  .comparison-grid {
    grid-template-columns: 1fr;
  }

  .sync-visual {
    grid-template-columns: 1fr;
  }

  .sync-arrow.push {
    transform: rotate(90deg);
  }

  .sync-arrow.pull {
    transform: rotate(90deg);
  }
}
</style>
