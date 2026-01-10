<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-grey-5 text-weight-bold q-mb-sm">
          MÓDULO 3.7: HIGIENE DEL REPOSITORIO
        </div>

        <h1 class="hero-title">El Escudo <span class="text-white">.gitignore</span></h1>

        <TextBlock>
          Git es un acaparador: intentará guardar todo lo que vea. Si no le pones límites, tu
          repositorio se llenará de basura: archivos compilados, carpetas de compilación gigantes
          (build/install) y configuraciones personales. Aprende a configurar el
          <strong>Campo de Fuerza</strong> que mantiene tu proyecto limpio y ligero.
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL CONCEPTO: ¿QUÉ IGNORAR? -->
    <div class="section-group self-stretch">
      <SectionTitle>1. ¿Qué es Basura?</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            La regla es simple: <strong>Si se puede generar automáticamente, no se guarda.</strong>
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>
              🔨 <strong>Artefactos de Build:</strong> En ROS 2, las carpetas <code>build/</code>,
              <code>install/</code> y <code>log/</code> se crean al compilar. Son gigabytes de
              basura.
            </li>
            <li>
              🐍 <strong>Entornos Virtuales:</strong> Tu carpeta <code>venv/</code> contiene miles
              de librerías que se pueden reinstalar con un simple <code>pip install</code>.
            </li>
            <li>
              🔧 <strong>Configuración de IDE:</strong> Carpetas <code>.vscode/</code> o
              <code>.idea/</code> son tuyas, no del equipo.
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL SHIELD ANIMATION -->
          <div
            class="tool-card shield-card relative-position q-pa-xl overflow-hidden bg-slate-900 shadow-2"
          >
            <!-- GIT REPO (CASTLE) -->
            <div class="repo-castle absolute-center column items-center z-top">
              <div class="bg-slate-800 q-pa-md rounded-borders shadow-1 border-light">
                <q-icon name="fort" size="4rem" color="white" />
              </div>
              <div class="text-caption text-center text-white q-mt-sm font-mono text-weight-bold">
                Repo Puro
              </div>
            </div>

            <!-- THE SHIELD -->
            <div class="force-field absolute-center"></div>

            <!-- INCOMING FILES (ANIMATED) -->
            <!-- Trash 1 -->
            <div class="file-trash file-1 absolute row items-center">
              <q-icon name="folder" color="red-4" size="sm" class="q-mr-xs" />
              <span>build/</span>
            </div>
            <!-- Trash 2 -->
            <div class="file-trash file-2 absolute row items-center">
              <q-icon name="description" color="grey-5" size="sm" class="q-mr-xs" />
              <span>.DS_Store</span>
            </div>
            <!-- Code (Passing) -->
            <div class="file-code file-3 absolute row items-center">
              <q-icon name="code" color="green-4" size="sm" class="q-mr-xs" />
              <span>main.py</span>
            </div>

            <!-- STATUS TEXT -->
            <div class="absolute-bottom text-center q-pb-md text-grey-5 text-caption z-top">
              Solo <span class="text-green-4 text-weight-bold">código fuente</span> atraviesa el
              escudo.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. SINTAXIS (CÓMO ESCRIBIRLO) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Escribiendo las Reglas</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El archivo se debe llamar exactamente <code>.gitignore</code> (con el punto al inicio) y
            vive en la raíz de tu proyecto. <br /><br />
            Es una lista de patrones. Puedes usar comodines (*) para bloquear familias enteras de
            archivos.
          </TextBlock>
          <div class="q-mt-lg">
            <AlertBlock type="warning" title="El error retroactivo">
              Si ya subiste un archivo basura (ej: <code>build/</code>) y LUEGO creas el .gitignore,
              <strong>Git lo seguirá guardando</strong>. Debes borrarlo primero de la memoria de Git
              con <code>git rm -r --cached build/</code>.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="tool-card code-card bg-slate-900 q-pa-none border-grey shadow-2">
            <div
              class="window-header row justify-between items-center q-px-md q-py-xs bg-black border-bottom-dark"
            >
              <span class="text-grey-5 font-mono text-xs">.gitignore</span>
              <q-icon name="shield" color="grey-7" size="xs" />
            </div>
            <!-- CORREGIDO: lang & content -->
            <CodeBlock
              lang="bash"
              :show-line-numbers="true"
              content="# --- ROS 2 ---
build/
install/
log/
COLCON_IGNORE

# --- Python ---
__pycache__/
*.py[cod]
venv/
.env

# --- C++ ---
*.o
*.so

# --- Sistema ---
.DS_Store
.vscode/"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. GITIGNORE GLOBAL (TU HIGIENE PERSONAL) -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Pro Tip: Gitignore Global</SectionTitle>
      <TextBlock>
        Hay archivos que genera TU computadora (como los archivos temporales de Mac
        <code>.DS_Store</code> o de Windows <code>Thumbs.db</code>) que molestan en todos los
        proyectos.
        <br />
        En lugar de ponerlos en cada proyecto, configura un escudo personal para siempre.
      </TextBlock>

      <div class="row q-mt-lg justify-center">
        <div class="col-12 col-md-9">
          <div
            class="tool-card global-card bg-slate-800 q-pa-lg text-center border-dashed shadow-glow-grey"
          >
            <div class="row items-center justify-center q-mb-md">
              <q-icon name="public" color="grey-5" size="md" class="q-mr-sm" />
              <div class="text-h6 text-white">Configuración Única</div>
            </div>

            <div
              class="bg-black text-green-4 q-pa-md rounded-borders font-mono text-xs border-light inline-block shadow-1"
            >
              git config --global core.excludesfile ~/.gitignore_global
            </div>
            <p class="text-caption text-grey-4 q-mt-md">
              Ahora crea un archivo en tu home llamado <code>.gitignore_global</code> y pon ahí tu
              basura del sistema operativo.
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. PLANTILLAS (NO REINVENTES LA RUEDA) -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. Herramientas</SectionTitle>
      <div class="row q-col-gutter-lg">
        <div class="col-12 col-md-6">
          <div
            class="tool-card resource-card bg-slate-900 q-pa-lg row items-center border-left-blue shadow-2 transition-hover"
          >
            <div class="bg-slate-800 q-pa-md rounded-borders q-mr-md">
              <q-icon name="link" color="blue-4" size="2rem" />
            </div>
            <div>
              <div class="text-subtitle1 text-white text-weight-bold">gitignore.io</div>
              <div class="text-caption text-grey-4 q-mt-xs">
                Escribe "ROS2, Python, VisualStudioCode" y te genera el archivo perfecto
                automáticamente.
              </div>
            </div>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div
            class="tool-card resource-card bg-slate-900 q-pa-lg row items-center border-left-purple shadow-2 transition-hover"
          >
            <div class="bg-slate-800 q-pa-md rounded-borders q-mr-md">
              <q-icon name="auto_fix_high" color="purple-4" size="2rem" />
            </div>
            <div>
              <div class="text-subtitle1 text-white text-weight-bold">GitHub Templates</div>
              <div class="text-caption text-grey-4 q-mt-xs">
                Al crear un repo en GitHub, selecciona el template "ROS". Ya viene con todo
                configurado.
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(156, 163, 175, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* TOOL CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* SHIELD VISUALIZATION */
.shield-card {
  border-top: 4px solid #9ca3af;
  height: 320px;
}
.z-top {
  z-index: 5;
  position: relative;
}

.force-field {
  width: 160px;
  height: 160px;
  border-radius: 50%;
  border: 2px solid rgba(156, 163, 175, 0.4);
  background: radial-gradient(circle, rgba(156, 163, 175, 0.1) 0%, transparent 70%);
  box-shadow: 0 0 30px rgba(156, 163, 175, 0.2);
  z-index: 2;
  animation: pulseShield 4s infinite ease-in-out;
}

@keyframes pulseShield {
  0% {
    transform: translate(-50%, -50%) scale(1);
    border-color: rgba(156, 163, 175, 0.3);
  }
  50% {
    transform: translate(-50%, -50%) scale(1.1);
    border-color: rgba(156, 163, 175, 0.6);
  }
  100% {
    transform: translate(-50%, -50%) scale(1);
    border-color: rgba(156, 163, 175, 0.3);
  }
}

.file-trash {
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #ef4444;
  font-weight: bold;
  animation: bounceOff 3s infinite linear;
  opacity: 0;
}
.file-code {
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #4ade80;
  font-weight: bold;
  animation: passThrough 3s infinite linear;
  opacity: 0;
}

.file-1 {
  top: 30%;
  left: 0;
  animation-delay: 0s;
}
.file-2 {
  top: 60%;
  left: 0;
  animation-delay: 1.5s;
}
.file-3 {
  top: 45%;
  left: 0;
  animation-delay: 0.8s;
}

/* Physics Animation: Start left -> Hit Shield -> Bounce Back/Fade */
@keyframes bounceOff {
  0% {
    left: 5%;
    opacity: 0;
    transform: scale(0.8);
  }
  20% {
    opacity: 1;
    transform: scale(1);
  }
  45% {
    left: 38%;
    transform: scale(1);
  } /* Impact point */
  50% {
    left: 35%;
    transform: scale(0.9) rotate(-10deg);
    opacity: 0.8;
  } /* Recoil */
  100% {
    left: 20%;
    top: 80%;
    transform: scale(0.5) rotate(-90deg);
    opacity: 0;
  } /* Fall away */
}

/* Physics Animation: Start left -> Pass Shield -> Enter Castle */
@keyframes passThrough {
  0% {
    left: 5%;
    opacity: 0;
    transform: scale(0.8);
  }
  20% {
    opacity: 1;
    transform: scale(1);
  }
  50% {
    left: 50%;
    opacity: 1;
    transform: scale(1.1);
    text-shadow: 0 0 10px #4ade80;
  } /* Center */
  100% {
    left: 50%;
    opacity: 0;
    transform: scale(0);
  } /* Absorbed by castle */
}

/* CODE CARD */
.code-card {
  overflow: hidden;
}
.border-grey {
  border-color: #9ca3af;
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

/* GLOBAL CARD */
.global-card {
  border: 1px dashed rgba(255, 255, 255, 0.2);
}
.shadow-glow-grey {
  box-shadow: 0 0 20px rgba(255, 255, 255, 0.05);
}

/* RESOURCE CARDS */
.resource-card {
  transition:
    transform 0.3s,
    background 0.3s;
  cursor: pointer;
}
.transition-hover:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.6);
}
.border-left-blue {
  border-left: 4px solid #3b82f6;
}
.border-left-purple {
  border-left: 4px solid #a855f7;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.inline-block {
  display: inline-block;
}
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 12px;
  font-size: 1rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
