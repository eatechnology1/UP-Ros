<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 0: LOGÍSTICA DE SOFTWARE
        </div>

        <h1 class="hero-title">Gestión de <span class="text-primary">Paquetes</span></h1>

        <TextBlock>
          Olvida buscar "setup.exe" en Google. En Linux, el software viaja por tuberías seguras
          llamadas
          <strong>Repositorios</strong>. Aprende a dominar APT, entender PIP y sobrevivir al
          "Infierno de Dependencias".
        </TextBlock>
      </div>
    </section>

    <!-- 2. EL TRIÁNGULO DE APROVISIONAMIENTO -->
    <div class="section-group self-stretch">
      <SectionTitle>1. ¿De dónde viene el software?</SectionTitle>
      <div class="row q-col-gutter-lg q-mt-sm items-stretch">
        <!-- APT -->
        <div class="col-12 col-md-4">
          <div class="tool-card apt">
            <div class="tool-header column flex-center text-center">
              <q-icon name="dns" size="3rem" class="text-orange-9 q-mb-sm" />
              <h3 class="text-h5 q-my-none">APT</h3>
              <div class="text-caption text-grey-5">Advanced Package Tool</div>
            </div>
            <p class="q-mt-md text-grey-4 text-center">
              <strong>La Vía Oficial.</strong> Software verificado por Ubuntu. Estable y seguro.
              <br /><br />
              <em class="text-grey-6">Uso: Todo el sistema base.</em>
            </p>
          </div>
        </div>

        <!-- PIP -->
        <div class="col-12 col-md-4">
          <div class="tool-card pip">
            <div class="tool-header column flex-center text-center">
              <q-icon name="code" size="3rem" class="text-blue-5 q-mb-sm" />
              <h3 class="text-h5 q-my-none">PIP</h3>
              <div class="text-caption text-grey-5">Python Package Index</div>
            </div>
            <p class="q-mt-md text-grey-4 text-center">
              <strong>La Vía del Desarrollador.</strong> Librerías de Python puras. Lo último en IA.
              <br /><br />
              <em class="text-grey-6">Uso: Tensorflow, NumPy, herramientas ROS.</em>
            </p>
          </div>
        </div>

        <!-- SOURCE -->
        <div class="col-12 col-md-4">
          <div class="tool-card git">
            <div class="tool-header column flex-center text-center">
              <q-icon name="source" size="3rem" class="text-white q-mb-sm" />
              <h3 class="text-h5 q-my-none">SOURCE</h3>
              <div class="text-caption text-grey-5">Git + Compile</div>
            </div>
            <p class="q-mt-md text-grey-4 text-center">
              <strong>La Vía Artesanal.</strong> Clonas el código y lo construyes tú mismo.
              <br /><br />
              <em class="text-grey-6">Uso: Drivers de robots, tu propio código.</em>
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. DOMINANDO APT -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Dominando APT (El Comandante)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            APT es tu intendente. Antes de pedirle algo, debes decirle que actualice su catálogo de
            existencias. Si no haces <code>update</code>, pedirás versiones fantasma que ya no
            existen.
          </TextBlock>
          <div class="q-mt-md">
            <!-- CORREGIDO: language -> lang, code -> content -->
            <CodeBlock
              lang="bash"
              title="El Ritual Sagrado"
              content="# 1. Actualiza el catálogo
sudo apt update

# 2. Instala el software
sudo apt install terminator

# 3. (Opcional) Actualiza el sistema
sudo apt upgrade"
            />
          </div>
        </template>

        <template #right>
          <!-- Terminal Simulation -->
          <div class="nano-terminal q-pa-md font-mono text-caption rounded-borders shadow-2">
            <div class="text-green-4">user@upros:~$ sudo apt update</div>
            <div class="text-grey-5">Hit:1 http://archive.ubuntu.com/ubuntu noble InRelease</div>
            <div class="text-grey-5">
              Get:2 http://security.ubuntu.com/ubuntu noble-security InRelease
            </div>
            <div class="text-white">Reading package lists... Done</div>
            <div class="text-yellow-8">4 packages can be upgraded.</div>
            <br />
            <div class="text-green-4">user@upros:~$ sudo apt install git</div>
            <div class="text-white">Reading package lists... Done</div>
            <div class="text-white">The following NEW packages will be installed:</div>
            <div class="text-white q-pl-md">git git-man liberror-perl</div>
            <div class="text-white">
              Do you want to continue? [Y/n] <span class="cursor-blink">_</span>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. EL PELIGRO DE PIP -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Python y el Caos (PIP)</SectionTitle>

      <AlertBlock type="danger" title="💀 Alerta Ubuntu 24.04 (PEP 668)">
        Si intentas <code>pip install numpy</code> globalmente, Ubuntu te bloqueará:
        <strong>"Externally Managed Environment"</strong>. Linux te protege de romper dependencias
        del sistema operativo.
      </AlertBlock>

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-6">
          <div class="tool-card apt-python full-height">
            <h4 class="text-h6 text-green-4 q-mt-none q-mb-sm">✅ La Forma Correcta (APT)</h4>
            <p class="text-grey-4">
              Si necesitas una librería común (numpy, pandas) para todo el sistema, usa APT.
            </p>
            <!-- CORREGIDO: language -> lang, code -> content -->
            <CodeBlock lang="bash" content="sudo apt install python3-numpy" />
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card pip-venv full-height">
            <h4 class="text-h6 text-blue-4 q-mt-none q-mb-sm">✅ La Forma Aislada (VENV)</h4>
            <p class="text-grey-4">
              Si necesitas una versión específica, crea un entorno virtual (sandbox).
            </p>
            <!-- CORREGIDO: language -> lang, code -> content -->
            <CodeBlock
              lang="bash"
              content="python3 -m venv mi_entorno
source mi_entorno/bin/activate
pip install libreria_rara"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- 5. GIT & SOURCE -->
    <div class="section-group self-stretch">
      <SectionTitle>4. Compilando desde Source</SectionTitle>
      <TextBlock>
        En ROS 2, a menudo necesitarás código que no está en APT (ej: un driver experimental en
        GitHub). Aquí te conviertes en un <strong>Artesano de Software</strong>.
      </TextBlock>

      <div class="step-process q-mt-xl justify-center">
        <!-- Paso 1 -->
        <div class="step-card">
          <div class="step-badge">1</div>
          <div class="text-weight-bold text-white">Clonar</div>
          <div class="text-caption text-grey-4">Traer el código</div>
          <div class="code-pill">git clone ...</div>
        </div>

        <div class="step-arrow">➜</div>

        <!-- Paso 2 -->
        <div class="step-card">
          <div class="step-badge">2</div>
          <div class="text-weight-bold text-white">Dependencias</div>
          <div class="text-caption text-grey-4">¿Qué necesita?</div>
          <div class="code-pill">rosdep install</div>
        </div>

        <div class="step-arrow">➜</div>

        <!-- Paso 3 -->
        <div class="step-card">
          <div class="step-badge">3</div>
          <div class="text-weight-bold text-white">Compilar</div>
          <div class="text-caption text-grey-4">Traducir a binario</div>
          <div class="code-pill">colcon build</div>
        </div>
      </div>
    </div>

    <!-- 6. PRO TIP -->
    <div class="section-group self-stretch q-mb-xl">
      <div class="tool-card no-border-top text-center border-dashed">
        <q-icon name="psychology" size="md" color="accent" class="q-mb-sm" />
        <h3 class="text-h6 text-white q-my-none">Pro Tip: Alias</h3>
        <p class="text-grey-4 q-mb-md">
          ¿Cansado de escribir <code>sudo apt install</code>? Crea atajos en tu
          <code>.bashrc</code>.
        </p>
        <div class="row justify-center">
          <div class="col-12 col-md-8">
            <!-- CORREGIDO: language -> lang, code -> content -->
            <CodeBlock
              lang="bash"
              content="alias install='sudo apt install'
alias update='sudo apt update'"
            />
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
    radial-gradient(circle at center, rgba(251, 146, 60, 0.15), transparent 60%),
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

/* TOOL CARDS (Providers) */
.tool-card {
  height: 100%;
  padding: 24px;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}
.tool-card.apt {
  border-top: 4px solid #f97316;
} /* Orange */
.tool-card.pip {
  border-top: 4px solid #3b82f6;
} /* Blue */
.tool-card.git {
  border-top: 4px solid #ffffff;
} /* White */
.tool-card.apt-python {
  border-top: 4px solid #4ade80;
} /* Green */
.tool-card.pip-venv {
  border-top: 4px solid #60a5fa;
} /* Light Blue */
.tool-card.no-border-top {
  border-top: none;
}

.border-dashed {
  border: 1px dashed rgba(255, 255, 255, 0.2) !important;
}

/* TERMINAL SIMULATION (Reutilizado estilo nano) */
.nano-terminal {
  background-color: #0f172a; /* Slate 900 */
  border: 1px solid #334155;
}
.cursor-blink {
  background-color: #4caf50;
  color: #0f172a;
  animation: blink 1s step-end infinite;
}
@keyframes blink {
  50% {
    opacity: 0;
  }
}

/* PROCESS STEPS */
.step-process {
  display: flex;
  align-items: center;
  gap: 16px;
  flex-wrap: wrap;
}

.step-card {
  background: rgba(30, 41, 59, 0.6);
  padding: 20px;
  border-radius: 12px;
  border: 1px solid rgba(255, 255, 255, 0.1);
  width: 180px;
  position: relative;
  text-align: center;
}

.step-badge {
  position: absolute;
  top: -12px;
  left: 50%;
  transform: translateX(-50%);
  width: 30px;
  height: 30px;
  background: #38bdf8;
  color: #0f172a;
  font-weight: 800;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.code-pill {
  background: rgba(0, 0, 0, 0.3);
  padding: 4px 8px;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  color: #81c784;
  margin-top: 8px;
}

.step-arrow {
  font-size: 1.5rem;
  color: #64748b;
}

.font-mono {
  font-family: 'Fira Code', monospace;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .step-process {
    flex-direction: column;
  }
  .step-arrow {
    transform: rotate(90deg);
  }
}
</style>
