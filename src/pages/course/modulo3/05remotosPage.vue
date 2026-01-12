<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Hasta ahora, Git ha vivido solo en tu computadora. Los repositorios remotos (como GitHub)
      permiten colaboración, backup en la nube, y compartir tu código con el mundo. Es el paso de
      principiante a profesional.
    </TextBlock>

    <AlertBlock type="info" title="¿Por qué usar repositorios remotos?">
      <strong>Colaboración:</strong> Múltiples personas trabajando en el mismo proyecto
      <br />
      <strong>Backup:</strong> Tu código seguro en la nube
      <br />
      <strong>Portafolio:</strong> Muestra tu trabajo a empleadores
      <br />
      <strong>Open Source:</strong> Contribuye a proyectos de robótica
    </AlertBlock>

    <!-- ORIGIN -->
    <div class="section-group">
      <SectionTitle>1. ¿Qué es "Origin"?</SectionTitle>
      <TextBlock>
        Cuando conectas tu carpeta local con GitHub, Git necesita un apodo para esa dirección web
        larga. Por convención universal, llamamos a ese servidor principal
        <strong>origin</strong>.
      </TextBlock>

      <div class="origin-visual q-mt-md">
        <div class="connection-card local">
          <div class="connection-icon">
            <q-icon name="laptop_mac" size="4rem" />
          </div>
          <div class="connection-label">Tu PC</div>
          <div class="connection-desc">Local Repository</div>
        </div>

        <div class="connection-link">
          <div class="link-line"></div>
          <div class="link-badge">
            <span>origin</span>
          </div>
          <div class="link-url">https://github.com/user/repo.git</div>
        </div>

        <div class="connection-card remote">
          <div class="connection-icon">
            <q-icon name="cloud" size="4rem" />
          </div>
          <div class="connection-label">GitHub</div>
          <div class="connection-desc">Remote Repository</div>
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Ver y configurar remotes"
          lang="bash"
          content="# Ver remotes configurados
git remote -v

# Agregar remote
git remote add origin https://github.com/user/repo.git

# Cambiar URL del remote
git remote set-url origin https://github.com/user/new-repo.git

# Eliminar remote
git remote remove origin"
          :copyable="true"
        />
      </div>
    </div>

    <!-- PUSH Y PULL -->
    <div class="section-group">
      <SectionTitle>2. Push y Pull: El Latido del Código</SectionTitle>
      <TextBlock>
        La sincronización no es automática (como Dropbox). Tú decides cuándo ocurre.
      </TextBlock>

      <div class="sync-visual q-mt-md">
        <div class="sync-card push">
          <div class="sync-header">
            <q-icon name="cloud_upload" size="3rem" />
            <span>git push</span>
          </div>
          <div class="sync-desc">Subir tus commits locales a GitHub</div>
          <CodeBlock
            lang="bash"
            content="# Push a main
git push origin main

# Primera vez (crear rama remota)
git push -u origin feature/nueva

# Push todas las ramas
git push --all"
            :copyable="true"
          />
          <div class="sync-flow">
            <div class="flow-item local">Tu PC</div>
            <q-icon name="arrow_forward" size="2rem" color="pink-4" />
            <div class="flow-item remote">GitHub</div>
          </div>
        </div>

        <div class="sync-card pull">
          <div class="sync-header">
            <q-icon name="cloud_download" size="3rem" />
            <span>git pull</span>
          </div>
          <div class="sync-desc">Descargar cambios del equipo desde GitHub</div>
          <CodeBlock
            lang="bash"
            content="# Pull de main
git pull origin main

# Pull con rebase (historial más limpio)
git pull --rebase origin main

# Pull de rama específica
git pull origin feature/nueva"
            :copyable="true"
          />
          <div class="sync-flow">
            <div class="flow-item remote">GitHub</div>
            <q-icon name="arrow_forward" size="2rem" color="cyan-4" />
            <div class="flow-item local">Tu PC</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="warning" title="Regla de Oro">
          Siempre haz <code>git pull</code> ANTES de <code>git push</code>. Si no, chocarás con el
          trabajo de otros y Git rechazará tu push.
        </AlertBlock>
      </div>
    </div>

    <!-- FETCH VS PULL -->
    <div class="section-group">
      <SectionTitle>3. Fetch vs Pull: La Diferencia</SectionTitle>
      <TextBlock>
        <code>git pull</code> es en realidad dos comandos en uno. Entender la diferencia te da más
        control.
      </TextBlock>

      <div class="comparison-cards q-mt-md">
        <div class="comparison-card fetch">
          <div class="comparison-header">
            <q-icon name="download" color="blue-4" size="2rem" />
            <span>git fetch</span>
          </div>
          <div class="comparison-desc">
            Descarga cambios pero NO los fusiona. Solo actualiza tu copia local del remote.
          </div>
          <CodeBlock
            lang="bash"
            content="# Descargar cambios
git fetch origin

# Ver qué hay de nuevo
git log origin/main

# Decidir si fusionar
git merge origin/main"
            :copyable="true"
          />
          <div class="comparison-note safe">✅ Seguro: puedes revisar antes de fusionar</div>
        </div>

        <div class="comparison-card pull-card">
          <div class="comparison-header">
            <q-icon name="sync" color="green-4" size="2rem" />
            <span>git pull</span>
          </div>
          <div class="comparison-desc">
            Descarga Y fusiona automáticamente. Es <code>fetch + merge</code> en un comando.
          </div>
          <CodeBlock
            lang="bash"
            content="# Equivalente a:
git fetch origin
git merge origin/main

# O simplemente:
git pull origin main"
            :copyable="true"
          />
          <div class="comparison-note fast">⚡ Rápido: todo en un paso</div>
        </div>
      </div>
    </div>

    <!-- AUTENTICACIÓN -->
    <div class="section-group">
      <SectionTitle>4. Autenticación: SSH vs HTTPS</SectionTitle>
      <TextBlock>
        Desde 2021, GitHub ya no acepta contraseñas para push. Debes usar SSH keys (recomendado) o
        Personal Access Tokens.
      </TextBlock>

      <div class="auth-methods q-mt-md">
        <div class="auth-card ssh">
          <div class="auth-header">
            <q-icon name="vpn_key" size="2.5rem" />
            <span>SSH Keys (Recomendado)</span>
          </div>
          <div class="auth-desc">
            Generas un par de llaves (pública/privada). Subes la pública a GitHub. Nunca más pide
            contraseña.
          </div>

          <div class="auth-steps">
            <div class="auth-step">
              <div class="step-num">1</div>
              <div class="step-content">
                <div class="step-title">Generar llave SSH</div>
                <CodeBlock
                  lang="bash"
                  content="ssh-keygen -t ed25519 -C 'tu@email.com'
# Presiona Enter 3 veces (sin passphrase)"
                  :copyable="true"
                />
              </div>
            </div>

            <div class="auth-step">
              <div class="step-num">2</div>
              <div class="step-content">
                <div class="step-title">Copiar llave pública</div>
                <CodeBlock
                  lang="bash"
                  content="# Linux/Mac
cat ~/.ssh/id_ed25519.pub

# Windows (PowerShell)
Get-Content ~/.ssh/id_ed25519.pub"
                  :copyable="true"
                />
              </div>
            </div>

            <div class="auth-step">
              <div class="step-num">3</div>
              <div class="step-content">
                <div class="step-title">Agregar a GitHub</div>
                <div class="step-instructions">
                  GitHub → Settings → SSH and GPG keys → New SSH key → Pegar y guardar
                </div>
              </div>
            </div>

            <div class="auth-step">
              <div class="step-num">4</div>
              <div class="step-content">
                <div class="step-title">Usar URL SSH</div>
                <CodeBlock
                  lang="bash"
                  content="# Clonar con SSH
git clone git@github.com:user/repo.git

# Cambiar remote a SSH
git remote set-url origin git@github.com:user/repo.git"
                  :copyable="true"
                />
              </div>
            </div>
          </div>
        </div>

        <div class="auth-card https">
          <div class="auth-header">
            <q-icon name="lock" size="2.5rem" />
            <span>Personal Access Token</span>
          </div>
          <div class="auth-desc">
            Generas un token en GitHub y lo usas como contraseña. Más simple pero menos seguro.
          </div>

          <div class="auth-steps">
            <div class="auth-step">
              <div class="step-num">1</div>
              <div class="step-content">
                <div class="step-title">Generar token</div>
                <div class="step-instructions">
                  GitHub → Settings → Developer settings → Personal access tokens → Generate new
                  token
                </div>
              </div>
            </div>

            <div class="auth-step">
              <div class="step-num">2</div>
              <div class="step-content">
                <div class="step-title">Guardar token</div>
                <div class="step-instructions">
                  Copia el token (solo se muestra una vez). Guárdalo en un lugar seguro.
                </div>
              </div>
            </div>

            <div class="auth-step">
              <div class="step-num">3</div>
              <div class="step-content">
                <div class="step-title">Usar como contraseña</div>
                <CodeBlock
                  lang="bash"
                  content="# Al hacer push, usa el token como contraseña
git push origin main
# Username: tu-usuario
# Password: ghp_tu_token_aqui"
                  :copyable="true"
                />
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- FLUJO COLABORATIVO -->
    <div class="section-group">
      <SectionTitle>5. Flujo de Trabajo Colaborativo</SectionTitle>
      <TextBlock> Este es el flujo diario cuando trabajas en equipo: </TextBlock>

      <div class="workflow-timeline q-mt-md">
        <div class="timeline-step">
          <div class="timeline-marker">1</div>
          <div class="timeline-content">
            <div class="timeline-title">Actualizar main</div>
            <CodeBlock
              lang="bash"
              content="git checkout main
git pull origin main"
              :copyable="true"
            />
          </div>
        </div>

        <div class="timeline-step">
          <div class="timeline-marker">2</div>
          <div class="timeline-content">
            <div class="timeline-title">Crear rama de feature</div>
            <CodeBlock lang="bash" content="git checkout -b feature/add-sensor" :copyable="true" />
          </div>
        </div>

        <div class="timeline-step">
          <div class="timeline-marker">3</div>
          <div class="timeline-content">
            <div class="timeline-title">Hacer cambios y commits</div>
            <CodeBlock
              lang="bash"
              content="# Editar archivos...
git add .
git commit -m 'feat: add sensor driver'"
              :copyable="true"
            />
          </div>
        </div>

        <div class="timeline-step">
          <div class="timeline-marker">4</div>
          <div class="timeline-content">
            <div class="timeline-title">Subir rama a GitHub</div>
            <CodeBlock
              lang="bash"
              content="git push -u origin feature/add-sensor"
              :copyable="true"
            />
          </div>
        </div>

        <div class="timeline-step">
          <div class="timeline-marker">5</div>
          <div class="timeline-content">
            <div class="timeline-title">Crear Pull Request en GitHub</div>
            <div class="timeline-desc">
              Ve a GitHub → Tu repositorio → Pull Requests → New Pull Request
            </div>
          </div>
        </div>

        <div class="timeline-step">
          <div class="timeline-marker">6</div>
          <div class="timeline-content">
            <div class="timeline-title">Después del merge, limpiar</div>
            <CodeBlock
              lang="bash"
              content="git checkout main
git pull origin main
git branch -d feature/add-sensor"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- COMANDOS ÚTILES -->
    <div class="section-group">
      <SectionTitle>6. Comandos Útiles de Remotes</SectionTitle>

      <div class="commands-grid">
        <div class="command-card">
          <div class="command-title">Ver estado de branches</div>
          <CodeBlock
            lang="bash"
            content="# Ver branches locales y remotas
git branch -a

# Ver tracking branches
git branch -vv"
            :copyable="true"
          />
        </div>

        <div class="command-card">
          <div class="command-title">Eliminar rama remota</div>
          <CodeBlock
            lang="bash"
            content="# Eliminar rama en GitHub
git push origin --delete feature/vieja

# Limpiar referencias locales
git fetch --prune"
            :copyable="true"
          />
        </div>

        <div class="command-card">
          <div class="command-title">Clonar repositorio</div>
          <CodeBlock
            lang="bash"
            content="# Clonar con HTTPS
git clone https://github.com/user/repo.git

# Clonar con SSH
git clone git@github.com:user/repo.git"
            :copyable="true"
          />
        </div>

        <div class="command-card">
          <div class="command-title">Forzar push (PELIGROSO)</div>
          <CodeBlock
            lang="bash"
            content="# Solo si estás 100% seguro
git push --force origin main

# Más seguro (rechaza si hay cambios)
git push --force-with-lease origin main"
            :copyable="true"
          />
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
            title="Git Remotes and GitHub"
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
          <code>git remote add</code>
          <span>Conectar con GitHub</span>
        </div>
        <div class="summary-item">
          <code>git push</code>
          <span>Subir commits</span>
        </div>
        <div class="summary-item">
          <code>git pull</code>
          <span>Descargar cambios</span>
        </div>
        <div class="summary-item">
          <code>git fetch</code>
          <span>Descargar sin fusionar</span>
        </div>
        <div class="summary-item">
          <code>git clone</code>
          <span>Copiar repositorio</span>
        </div>
        <div class="summary-item">
          <code>ssh-keygen</code>
          <span>Generar llave SSH</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Remotes" class="q-mt-lg">
        ✅ Configura SSH keys (una sola vez)
        <br />
        ✅ Siempre <code>git pull</code> antes de <code>git push</code>
        <br />
        ✅ Usa <code>git fetch</code> para revisar cambios antes de fusionar
        <br />
        ✅ NUNCA uses <code>--force</code> en ramas compartidas
        <br />
        ✅ Elimina ramas remotas después de merge
        <br />
        ✅ Mantén tu fork actualizado con el upstream
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

/* ORIGIN VISUAL */
.origin-visual {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 3rem 2rem;
}

.connection-card {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
  padding: 2rem;
  background: rgba(30, 41, 59, 0.8);
  border: 2px solid;
  border-radius: 12px;
}

.connection-card.local {
  border-color: #f97316;
}

.connection-card.remote {
  border-color: #3b82f6;
}

.connection-icon {
  color: #f1f5f9;
}

.connection-label {
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.connection-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

.connection-link {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  position: relative;
}

.link-line {
  width: 100%;
  height: 2px;
  background: linear-gradient(90deg, #f97316, #3b82f6);
}

.link-badge {
  padding: 0.75rem 1.5rem;
  background: #ec4899;
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  color: white;
  box-shadow: 0 0 20px rgba(236, 72, 153, 0.5);
}

.link-url {
  font-family: 'Fira Code', monospace;
  font-size: 0.75rem;
  color: #64748b;
  text-align: center;
}

/* SYNC VISUAL */
.sync-visual {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.sync-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.sync-card.push {
  border-color: #ec4899;
}

.sync-card.pull {
  border-color: #06b6d4;
}

.sync-header {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  font-size: 1.3rem;
  font-weight: 700;
  font-family: 'Fira Code', monospace;
  color: #f1f5f9;
}

.sync-desc {
  text-align: center;
  color: #94a3b8;
}

.sync-flow {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.flow-item {
  padding: 0.75rem 1.5rem;
  background: rgba(100, 116, 139, 0.3);
  border-radius: 6px;
  font-weight: 700;
  color: #cbd5e1;
}

/* COMPARISON CARDS */
.comparison-cards {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.comparison-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.comparison-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  font-family: 'Fira Code', monospace;
  color: #f1f5f9;
}

.comparison-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.comparison-note {
  padding: 1rem;
  border-radius: 8px;
  font-size: 0.9rem;
  text-align: center;
}

.comparison-note.safe {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid #22c55e;
  color: #86efac;
}

.comparison-note.fast {
  background: rgba(251, 191, 36, 0.1);
  border: 1px solid #fbbf24;
  color: #fde047;
}

/* AUTH METHODS */
.auth-methods {
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.auth-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.auth-card.ssh {
  border-color: #22c55e;
}

.auth-card.https {
  border-color: #fbbf24;
}

.auth-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.3rem;
  font-weight: 700;
  color: #f1f5f9;
}

.auth-desc {
  color: #cbd5e1;
}

.auth-steps {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.auth-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.step-num {
  width: 40px;
  height: 40px;
  background: linear-gradient(135deg, #f97316, #ea580c);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.2rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.step-content {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  padding: 1rem;
}

.step-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.step-instructions {
  color: #94a3b8;
  font-size: 0.9rem;
}

/* WORKFLOW TIMELINE */
.workflow-timeline {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.timeline-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.timeline-marker {
  width: 45px;
  height: 45px;
  background: linear-gradient(135deg, #3b82f6, #2563eb);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.3rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.timeline-content {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.timeline-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.timeline-desc {
  color: #94a3b8;
  font-size: 0.9rem;
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

.command-title {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
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
  .origin-visual {
    grid-template-columns: 1fr;
  }

  .sync-visual {
    grid-template-columns: 1fr;
  }

  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
