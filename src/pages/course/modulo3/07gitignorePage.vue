<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      El archivo <code>.gitignore</code> es tu filtro de basura. Le dice a Git qué archivos NO debe
      rastrear. Sin él, tu repositorio se llenará de archivos compilados, configuraciones
      personales, y potencialmente contraseñas o claves privadas.
    </TextBlock>

    <AlertBlock type="warning" title="¿Por qué es crítico?">
      <strong>Seguridad:</strong> Evita subir contraseñas, API keys, certificados
      <br />
      <strong>Tamaño:</strong> Repositorios limpios (&lt;100MB vs GBs de basura)
      <br />
      <strong>Conflictos:</strong> Archivos compilados causan merge conflicts constantes
      <br />
      <strong>Profesionalismo:</strong> Un repo sin .gitignore grita "principiante"
    </AlertBlock>

    <!-- EL PROBLEMA -->
    <div class="section-group">
      <SectionTitle>1. El Problema de la Basura</SectionTitle>
      <TextBlock>
        Cuando compilas código, generas cientos de archivos temporales. Si subes esta basura a
        GitHub, tu repositorio pesará GBs inútilmente y tus compañeros tendrán conflictos al
        compilar.
      </TextBlock>

      <div class="trash-visual q-mt-md">
        <div class="trash-side ignored">
          <div class="side-header">
            <q-icon name="block" size="2rem" />
            <span>Ignorado (.gitignore)</span>
          </div>
          <div class="file-list">
            <div class="file-item">
              <q-icon name="folder" />
              <span>build/</span>
              <div class="file-note">Archivos compilados</div>
            </div>
            <div class="file-item">
              <q-icon name="folder" />
              <span>install/</span>
              <div class="file-note">Binarios instalados</div>
            </div>
            <div class="file-item">
              <q-icon name="folder" />
              <span>log/</span>
              <div class="file-note">Logs de ejecución</div>
            </div>
            <div class="file-item">
              <q-icon name="folder" />
              <span>__pycache__/</span>
              <div class="file-note">Python bytecode</div>
            </div>
            <div class="file-item">
              <q-icon name="description" />
              <span>*.pyc</span>
              <div class="file-note">Archivos compilados</div>
            </div>
          </div>
        </div>

        <div class="trash-side tracked">
          <div class="side-header">
            <q-icon name="check_circle" size="2rem" />
            <span>Rastreado (Git)</span>
          </div>
          <div class="file-list">
            <div class="file-item">
              <q-icon name="folder" />
              <span>src/</span>
              <div class="file-note">Código fuente</div>
            </div>
            <div class="file-item">
              <q-icon name="description" />
              <span>package.xml</span>
              <div class="file-note">Configuración ROS</div>
            </div>
            <div class="file-item">
              <q-icon name="description" />
              <span>CMakeLists.txt</span>
              <div class="file-note">Build config</div>
            </div>
            <div class="file-item">
              <q:icon name="description" />
              <span>README.md</span>
              <div class="file-note">Documentación</div>
            </div>
            <div class="file-item">
              <q-icon name="description" />
              <span>.gitignore</span>
              <div class="file-note">Este archivo!</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- SINTAXIS -->
    <div class="section-group">
      <SectionTitle>2. Sintaxis de .gitignore</SectionTitle>
      <TextBlock>
        El archivo <code>.gitignore</code> es una lista simple de patrones. Git lo lee ANTES de
        hacer <code>git status</code>. Puedes usar comodines (*) para bloquear extensiones enteras.
      </TextBlock>

      <div class="syntax-examples q-mt-md">
        <div class="syntax-card">
          <div class="syntax-header">
            <q-icon name="description" color="blue-4" />
            <span>Archivo específico</span>
          </div>
          <CodeBlock
            lang="gitignore"
            content="config.json
secrets.env
database.db"
            :copyable="true"
          />
          <div class="syntax-desc">Ignora archivos por nombre exacto</div>
        </div>

        <div class="syntax-card">
          <div class="syntax-header">
            <q-icon name="star" color="green-4" />
            <span>Extensión completa</span>
          </div>
          <CodeBlock
            lang="gitignore"
            content="*.pyc
*.o
*.exe
*.log"
            :copyable="true"
          />
          <div class="syntax-desc">Ignora todos los archivos con esa extensión</div>
        </div>

        <div class="syntax-card">
          <div class="syntax-header">
            <q-icon name="folder" color="purple-4" />
            <span>Carpeta completa</span>
          </div>
          <CodeBlock
            lang="gitignore"
            content="build/
node_modules/
__pycache__/
.vscode/"
            :copyable="true"
          />
          <div class="syntax-desc">Ignora carpeta y todo su contenido</div>
        </div>

        <div class="syntax-card">
          <div class="syntax-header">
            <q-icon name="not_interested" color="orange-4" />
            <span>Excepciones</span>
          </div>
          <CodeBlock
            lang="gitignore"
            content="# Ignorar todos los .log
*.log

# EXCEPTO este
!important.log"
            :copyable="true"
          />
          <div class="syntax-desc">El ! niega una regla anterior</div>
        </div>
      </div>
    </div>

    <!-- GITIGNORE PARA ROS 2 -->
    <div class="section-group">
      <SectionTitle>3. .gitignore para ROS 2</SectionTitle>
      <TextBlock>
        Este es el <code>.gitignore</code> estándar para proyectos de ROS 2. Copia y pega en la raíz
        de tu workspace.
      </TextBlock>

      <div class="gitignore-file q-mt-md">
        <div class="file-header">
          <q-icon name="description" />
          <span>.gitignore</span>
          <div class="file-actions">
            <q-btn flat dense icon="content_copy" size="sm" color="grey-4">
              <q-tooltip>Copiar todo</q-tooltip>
            </q-btn>
          </div>
        </div>
        <div class="file-content">
          <CodeBlock
            lang="gitignore"
            content="# ROS 2 Build Artifacts
build/
install/
log/

# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
venv/

# C++
*.o
*.a
*.so
*.exe
*.out

# IDEs
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db
*.tmp

# Secrets (CRÍTICO)
*.pem
*.key
*.env
secrets.yaml
credentials.json

# Logs
*.log
*.bag

# Documentation builds
docs/_build/
*.pdf"
            :copyable="true"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="danger" title="NUNCA subas credenciales">
          Si accidentalmente subes una API key o contraseña a GitHub, ESTÁ COMPROMETIDA. Cambiarla
          inmediatamente. Eliminarla del historial no es suficiente (ya fue indexada).
        </AlertBlock>
      </div>
    </div>

    <!-- PATRONES AVANZADOS -->
    <div class="section-group">
      <SectionTitle>4. Patrones Avanzados</SectionTitle>

      <div class="patterns-grid">
        <div class="pattern-card">
          <div class="pattern-title">Ignorar en subdirectorios</div>
          <CodeBlock
            lang="gitignore"
            content="# Solo en raíz
/config.json

# En cualquier nivel
**/config.json"
            :copyable="true"
          />
        </div>

        <div class="pattern-card">
          <div class="pattern-title">Ignorar excepto en carpeta</div>
          <CodeBlock
            lang="gitignore"
            content="# Ignorar todos los .txt
*.txt

# Excepto en docs/
!docs/*.txt"
            :copyable="true"
          />
        </div>

        <div class="pattern-card">
          <div class="pattern-title">Rango de caracteres</div>
          <CodeBlock
            lang="gitignore"
            content="# Ignorar test1.py a test9.py
test[0-9].py

# Ignorar archivos que empiezan con temp
temp*"
            :copyable="true"
          />
        </div>

        <div class="pattern-card">
          <div class="pattern-title">Comentarios</div>
          <CodeBlock
            lang="gitignore"
            content="# Este es un comentario
# Git ignora líneas que empiezan con #

# Líneas vacías también se ignoran"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- GLOBAL GITIGNORE -->
    <div class="section-group">
      <SectionTitle>5. Gitignore Global (Higiene Personal)</SectionTitle>
      <TextBlock>
        Si usas Mac, generas <code>.DS_Store</code> en todos lados. Si usas Windows,
        <code>Thumbs.db</code>. No obligues a cada proyecto a ignorar TU basura del sistema
        operativo. Configura un ignore global.
      </TextBlock>

      <div class="global-setup q-mt-md">
        <div class="setup-step">
          <div class="step-num">1</div>
          <div class="step-content">
            <div class="step-title">Crear archivo global</div>
            <CodeBlock
              lang="bash"
              content="# Linux/Mac
touch ~/.gitignore_global

# Windows (PowerShell)
New-Item -Path $HOME\.gitignore_global -ItemType File"
              :copyable="true"
            />
          </div>
        </div>

        <div class="setup-step">
          <div class="step-num">2</div>
          <div class="step-content">
            <div class="step-title">Configurar Git</div>
            <CodeBlock
              lang="bash"
              content="git config --global core.excludesfile ~/.gitignore_global"
              :copyable="true"
            />
          </div>
        </div>

        <div class="setup-step">
          <div class="step-num">3</div>
          <div class="step-content">
            <div class="step-title">Agregar patrones personales</div>
            <CodeBlock
              lang="gitignore"
              content="# macOS
.DS_Store
.AppleDouble
.LSOverride

# Windows
Thumbs.db
ehthumbs.db
Desktop.ini

# Linux
*~
.directory

# IDEs (tu preferencia)
.vscode/
.idea/
*.swp"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- GENERADORES -->
    <div class="section-group">
      <SectionTitle>6. Generadores Automáticos</SectionTitle>
      <TextBlock>
        No escribas .gitignore a mano. Usa generadores que conocen las mejores prácticas para cada
        lenguaje y framework.
      </TextBlock>

      <div class="generators-grid q-mt-md">
        <div class="generator-card">
          <div class="generator-icon">
            <q-icon name="public" size="3rem" color="blue-4" />
          </div>
          <div class="generator-title">gitignore.io</div>
          <div class="generator-desc">
            El más popular. Escribe "ROS2, Python, VSCode" y genera el archivo completo.
          </div>
          <div class="generator-link">
            <a href="https://gitignore.io" target="_blank">gitignore.io →</a>
          </div>
        </div>

        <div class="generator-card">
          <div class="generator-icon">
            <q-icon name="code" size="3rem" color="green-4" />
          </div>
          <div class="generator-title">GitHub Templates</div>
          <div class="generator-desc">
            Al crear repo en GitHub, selecciona template de .gitignore (Python, C++, ROS, etc.)
          </div>
          <div class="generator-link">
            <a href="https://github.com/github/gitignore" target="_blank">Ver templates →</a>
          </div>
        </div>

        <div class="generator-card">
          <div class="generator-icon">
            <q-icon name="terminal" size="3rem" color="purple-4" />
          </div>
          <div class="generator-title">CLI: gig</div>
          <div class="generator-desc">Genera .gitignore desde la terminal</div>
          <CodeBlock
            lang="bash"
            content="# Instalar
npm install -g gig

# Usar
gig python ros vscode > .gitignore"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>7. Errores Comunes</SectionTitle>

      <div class="errors-grid">
        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="red-4" />
            <span>Ya commiteé el archivo</span>
          </div>
          <div class="error-desc">
            Si ya hiciste commit de un archivo, agregarlo a .gitignore NO lo elimina del historial.
          </div>
          <div class="error-solution">
            <strong>Solución:</strong>
            <CodeBlock
              lang="bash"
              content="# Eliminar del tracking (mantener local)
git rm --cached archivo.log

# Eliminar carpeta del tracking
git rm -r --cached build/

# Commit el cambio
git commit -m 'Remove build artifacts from tracking'"
              :copyable="true"
            />
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="orange-4" />
            <span>.gitignore no funciona</span>
          </div>
          <div class="error-desc">Agregaste un patrón pero Git sigue rastreando el archivo.</div>
          <div class="error-solution">
            <strong>Causas:</strong>
            <ul>
              <li>El archivo ya estaba commiteado (ver error anterior)</li>
              <li>Typo en el patrón</li>
              <li>.gitignore no está en la raíz del repo</li>
            </ul>
          </div>
        </div>

        <div class="error-card">
          <div class="error-header">
            <q-icon name="error" color="yellow-6" />
            <span>Ignoré algo importante</span>
          </div>
          <div class="error-desc">Agregaste src/ a .gitignore por error y perdiste tu código.</div>
          <div class="error-solution">
            <strong>Solución:</strong>
            <CodeBlock
              lang="bash"
              content="# Ver qué está ignorado
git status --ignored

# Forzar agregar archivo ignorado
git add -f src/important.py"
              :copyable="true"
            />
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
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="Gitignore Best Practices"
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
          <code>.gitignore</code>
          <span>Filtro de archivos</span>
        </div>
        <div class="summary-item">
          <code>build/ install/ log/</code>
          <span>ROS 2 artifacts</span>
        </div>
        <div class="summary-item">
          <code>*.pyc __pycache__/</code>
          <span>Python garbage</span>
        </div>
        <div class="summary-item">
          <code>git rm --cached</code>
          <span>Dejar de rastrear</span>
        </div>
        <div class="summary-item">
          <code>gitignore.io</code>
          <span>Generador automático</span>
        </div>
        <div class="summary-item">
          <code>~/.gitignore_global</code>
          <span>Ignore personal</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de .gitignore" class="q-mt-lg">
        ✅ Crea .gitignore ANTES del primer commit
        <br />
        ✅ Usa generador (gitignore.io) para tu stack
        <br />
        ✅ Incluye: build/, install/, log/, __pycache__/, *.pyc
        <br />
        ✅ NUNCA ignores: src/, package.xml, CMakeLists.txt, README
        <br />
        ✅ Configura .gitignore_global para tu OS
        <br />
        ✅ Revisa con <code>git status --ignored</code>
        <br />
        ✅ Si subiste credenciales, CÁMBIALAS inmediatamente
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

/* TRASH VISUAL */
.trash-visual {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.trash-side {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
}

.trash-side.ignored {
  border-color: #ef4444;
}

.trash-side.tracked {
  border-color: #22c55e;
}

.side-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.1rem;
  font-weight: 700;
  margin-bottom: 1.5rem;
  padding-bottom: 1rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.trash-side.ignored .side-header {
  color: #fca5a5;
}

.trash-side.tracked .side-header {
  color: #86efac;
}

.file-list {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.file-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  font-family: 'Fira Code', monospace;
  color: #cbd5e1;
}

.file-note {
  margin-left: auto;
  font-size: 0.75rem;
  color: #64748b;
  font-family: sans-serif;
}

/* SYNTAX EXAMPLES */
.syntax-examples {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
}

.syntax-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.syntax-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
}

.syntax-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

/* GITIGNORE FILE */
.gitignore-file {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.file-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 1rem 1.5rem;
  background: rgba(30, 41, 59, 0.8);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-family: 'Fira Code', monospace;
  color: #f1f5f9;
}

.file-actions {
  margin-left: auto;
}

.file-content {
  padding: 1.5rem;
}

/* PATTERNS GRID */
.patterns-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.pattern-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.pattern-title {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
}

/* GLOBAL SETUP */
.global-setup {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.setup-step {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  align-items: start;
}

.step-num {
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

/* GENERATORS GRID */
.generators-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.generator-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  gap: 1rem;
}

.generator-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.generator-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.generator-link a {
  color: #60a5fa;
  text-decoration: none;
  font-weight: 700;
}

.generator-link a:hover {
  color: #93c5fd;
  text-decoration: underline;
}

/* ERRORS GRID */
.errors-grid {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.error-card {
  background: rgba(15, 23, 42, 0.6);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.error-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.error-desc {
  color: #cbd5e1;
}

.error-solution {
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
}

.error-solution strong {
  color: #86efac;
  display: block;
  margin-bottom: 0.75rem;
}

.error-solution ul {
  color: #cbd5e1;
  margin: 0.5rem 0 0 1.5rem;
  padding: 0;
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

@media (max-width: 768px) {
  .trash-visual {
    grid-template-columns: 1fr;
  }
}
</style>
