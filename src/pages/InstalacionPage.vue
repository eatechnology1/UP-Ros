<template>
  <q-page class="installation-page q-pa-lg column items-center">
    <!-- ========== SECCIÓN 1: HERO ÉPICO ========== -->
    <section class="installation-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm animate-fade-in">
          Módulo 0 / Instalación
        </div>
        <h1 class="hero-title animate-slide-up">
          Instalación Completa de <span class="text-gradient">ROS 2 Jazzy</span>
        </h1>
        <p class="hero-subtitle animate-slide-up-delay">
          De cero a workspace funcional en 45 minutos.<br />
          <strong>Ubuntu 24.04 + ROS 2 Jazzy Jalisco (LTS 2029)</strong>
        </p>

        <div class="time-estimate q-mt-md animate-fade-in-delay">
          <q-icon name="schedule" size="sm" />
          <span>Tiempo estimado: 45-60 minutos</span>
        </div>
      </div>
    </section>

    <!-- ========== SECCIÓN 2: PRE-FLIGHT CHECKLIST ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>✈️ Pre-Flight Checklist</SectionTitle>

      <TextBlock class="q-mb-md text-center">
        Antes de comenzar, verifica que cumples con estos requisitos:
      </TextBlock>

      <div class="preflight-card">
        <div class="preflight-items">
          <div
            v-for="(item, idx) in preflightItems"
            :key="idx"
            class="preflight-item"
            @click="togglePreflight(idx)"
          >
            <q-checkbox
              v-model="item.checked"
              :label="item.label"
              color="primary"
              class="preflight-checkbox"
            />
            <div class="preflight-detail">{{ item.detail }}</div>
          </div>
        </div>

        <div class="preflight-progress q-mt-lg">
          <div class="progress-label">
            Progreso: {{ preflightProgress }}% ({{ checkedCount }}/{{ preflightItems.length }})
          </div>
          <q-linear-progress
            :value="preflightProgress / 100"
            color="primary"
            size="12px"
            rounded
            class="q-mt-sm"
          />
        </div>
      </div>
    </div>

    <!-- ========== SECCIÓN 3: PROGRESS TRACKER ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>🗺️ Roadmap de Instalación</SectionTitle>

      <div class="progress-stepper">
        <div
          v-for="(stage, idx) in installationStages"
          :key="idx"
          class="stepper-item"
          :class="{ active: currentStage === idx, completed: currentStage > idx }"
          @click="currentStage = idx"
        >
          <div class="stepper-marker">
            <q-icon
              :name="currentStage > idx ? 'check_circle' : stage.icon"
              size="md"
              :color="currentStage >= idx ? 'primary' : 'grey-6'"
            />
          </div>
          <div class="stepper-content">
            <div class="stepper-title">{{ stage.title }}</div>
            <div class="stepper-desc">{{ stage.description }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ========== ETAPA 1: VERIFICACIÓN DE OS ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>1️⃣ Verificación del Sistema Operativo</SectionTitle>

      <AlertBlock title="🛑 Requisito Crítico" type="warning" class="q-mb-md">
        ROS 2 Jazzy <strong>SOLO</strong> funciona en Ubuntu 24.04 (Noble Numbat). Intentar
        instalarlo en otra versión causará errores de dependencias.
      </AlertBlock>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Ejecuta este comando para verificar tu versión de Ubuntu. Debe mostrar
            <strong>"24.04"</strong> y <strong>"noble"</strong>.
          </TextBlock>

          <div class="compatibility-table q-mt-md">
            <div class="compat-row header">
              <div>Ubuntu</div>
              <div>ROS 2</div>
              <div>Estado</div>
            </div>
            <div class="compat-row recommended">
              <div>24.04 Noble</div>
              <div>Jazzy</div>
              <div>✅ LTS 2029</div>
            </div>
            <div class="compat-row">
              <div>22.04 Jammy</div>
              <div>Humble</div>
              <div>⚠️ LTS Anterior</div>
            </div>
            <div class="compat-row disabled">
              <div>20.04 Focal</div>
              <div>Foxy</div>
              <div>❌ EOL</div>
            </div>
          </div>
        </template>

        <template #right>
          <CodeBlock title="Verificar Versión de Ubuntu" lang="bash" :content="osCheckCommand" />
        </template>
      </SplitBlock>
    </div>

    <!-- ========== ETAPA 2: CONFIGURACIÓN DE LOCALE ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>2️⃣ Configuración de Locale (UTF-8)</SectionTitle>

      <TextBlock>
        ROS 2 requiere codificación UTF-8 para la comunicación entre nodos. Sin esto, los mensajes
        fallarán.
      </TextBlock>

      <CodeBlock
        title="Configurar UTF-8"
        lang="bash"
        :content="localeCommand"
        :copyable="true"
        class="q-mt-md"
      />
    </div>

    <!-- ========== ETAPA 3: REPOSITORIOS ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>3️⃣ Agregar Repositorios Oficiales de ROS 2</SectionTitle>

      <TextBlock class="q-mb-md">
        Configuraremos el repositorio oficial de ROS 2 para que Ubuntu pueda descargar los paquetes.
      </TextBlock>

      <StepsBlock
        :steps="[
          'Habilitar repositorio <strong>Universe</strong> de Ubuntu',
          'Instalar herramientas de gestión de repositorios',
          'Descargar configuración oficial de ROS 2',
          'Instalar el paquete de configuración',
        ]"
        class="q-mb-md"
      />

      <CodeBlock
        title="Configurar Repositorios"
        lang="bash"
        :content="repoCommand"
        :copyable="true"
      />
    </div>

    <!-- ========== ETAPA 4: INSTALACIÓN DE ROS 2 ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>4️⃣ Instalación de ROS 2 Jazzy</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock> Elige la variante que necesitas: </TextBlock>

          <div class="install-options q-mt-md">
            <div class="install-card desktop" @click="selectedVariant = 'desktop'">
              <div class="install-header">
                <q-icon name="monitor" size="md" />
                <h4>ros-jazzy-desktop</h4>
                <q-badge v-if="selectedVariant === 'desktop'" color="primary">Seleccionado</q-badge>
              </div>
              <p>
                Incluye ROS Core + RViz2 + Demos<br />
                <strong>Recomendado para desarrollo</strong>
              </p>
              <div class="install-meta">~800MB de descarga</div>
            </div>

            <div class="install-card base" @click="selectedVariant = 'base'">
              <div class="install-header">
                <q-icon name="memory" size="md" />
                <h4>ros-jazzy-ros-base</h4>
                <q-badge v-if="selectedVariant === 'base'" color="primary">Seleccionado</q-badge>
              </div>
              <p>
                Solo bibliotecas de comunicación<br />
                Sin herramientas gráficas
              </p>
              <div class="install-meta">~300MB de descarga</div>
            </div>
          </div>

          <AlertBlock title="💡 Consejo" type="info" class="q-mt-md">
            Ejecuta <code>sudo apt upgrade</code> antes de instalar para evitar conflictos de
            paquetes.
          </AlertBlock>
        </template>

        <template #right>
          <CodeBlock
            title="Instalar ROS 2"
            lang="bash"
            :content="getInstallCommand()"
            :copyable="true"
          />

          <div class="install-progress-info q-mt-md">
            <div class="info-item">
              <q-icon name="download" color="primary" />
              <span>La descarga puede tardar 10-15 minutos</span>
            </div>
            <div class="info-item">
              <q-icon name="build" color="primary" />
              <span>La instalación configurará ~500 paquetes</span>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ========== ETAPA 5: HERRAMIENTAS DE DESARROLLO ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>5️⃣ Herramientas de Desarrollo</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            Instala <code>colcon</code> (compilador) y <code>rosdep</code> (gestor de dependencias).
            Son <strong>obligatorios</strong> para este curso.
          </TextBlock>
        </template>

        <template #right>
          <CodeBlock
            title="Instalar ros-dev-tools"
            lang="bash"
            :content="devToolsCommand"
            :copyable="true"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- ========== ETAPA 6: CONFIGURACIÓN DE ENTORNO ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>6️⃣ Configuración Automática del Entorno</SectionTitle>

      <TextBlock class="q-mb-md">
        Configura tu <code>.bashrc</code> para que ROS 2 se cargue automáticamente en cada terminal
        nueva.
      </TextBlock>

      <CodeBlock title="Automatizar Setup" lang="bash" :content="bashrcCommand" :copyable="true" />

      <AlertBlock title="⚡ Importante" type="warning" class="q-mt-md">
        Después de ejecutar estos comandos, <strong>cierra y abre una nueva terminal</strong> o
        ejecuta <code>source ~/.bashrc</code>.
      </AlertBlock>
    </div>

    <!-- ========== ETAPA 7: CREACIÓN DE WORKSPACE ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>7️⃣ Crear tu Primer Workspace</SectionTitle>

      <TextBlock class="q-mb-md">
        Un workspace es la carpeta donde desarrollarás tus paquetes de ROS 2. Estructura estándar:
      </TextBlock>

      <div class="workspace-diagram">
        <div class="diagram-node root">
          <q-icon name="folder" />
          <span>ros2_ws/</span>
        </div>
        <div class="diagram-children">
          <div class="diagram-node">
            <q-icon name="folder_open" />
            <span>src/</span>
            <span class="node-desc">Código fuente</span>
          </div>
          <div class="diagram-node">
            <q-icon name="folder" />
            <span>build/</span>
            <span class="node-desc">Archivos compilados</span>
          </div>
          <div class="diagram-node">
            <q-icon name="folder" />
            <span>install/</span>
            <span class="node-desc">Binarios ejecutables</span>
          </div>
          <div class="diagram-node">
            <q-icon name="folder" />
            <span>log/</span>
            <span class="node-desc">Logs de compilación</span>
          </div>
        </div>
      </div>

      <CodeBlock
        title="Crear Workspace"
        lang="bash"
        :content="workspaceCommand"
        :copyable="true"
        class="q-mt-md"
      />
    </div>

    <!-- ========== ETAPA 8: VERIFICACIÓN ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>8️⃣ Verificación de la Instalación</SectionTitle>

      <TextBlock class="q-mb-md">
        Ejecuta estas pruebas para confirmar que todo funciona correctamente:
      </TextBlock>

      <div class="verification-tests">
        <div class="test-item">
          <div class="test-header">
            <q-icon name="terminal" color="primary" />
            <h4>Test 1: Verificar Versión</h4>
          </div>
          <CodeBlock lang="bash" content="ros2 --version" />
          <div class="test-expected">
            <strong>Resultado esperado:</strong> <code>ros2 cli version: jazzy</code>
          </div>
        </div>

        <div class="test-item">
          <div class="test-header">
            <q-icon name="chat" color="primary" />
            <h4>Test 2: Talker & Listener</h4>
          </div>
          <div class="row q-col-gutter-md">
            <div class="col-12 col-md-6">
              <div class="terminal-label">Terminal 1 (Publicador)</div>
              <CodeBlock lang="bash" content="ros2 run demo_nodes_cpp talker" />
            </div>
            <div class="col-12 col-md-6">
              <div class="terminal-label">Terminal 2 (Suscriptor)</div>
              <CodeBlock lang="bash" content="ros2 run demo_nodes_py listener" />
            </div>
          </div>
          <div class="test-expected">
            <strong>Resultado esperado:</strong> El listener debe mostrar los mensajes del talker
          </div>
        </div>

        <div class="test-item">
          <div class="test-header">
            <q-icon name="3d_rotation" color="primary" />
            <h4>Test 3: RViz2 (Visualización)</h4>
          </div>
          <CodeBlock lang="bash" content="rviz2" />
          <div class="test-expected">
            <strong>Resultado esperado:</strong> Debe abrirse la interfaz gráfica de RViz2
          </div>
        </div>
      </div>
    </div>

    <!-- ========== TROUBLESHOOTING ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>🔧 Solución de Problemas Comunes</SectionTitle>

      <div class="trouble-grid">
        <div class="trouble-item">
          <div class="trouble-q">❌ "ros2: command not found"</div>
          <div class="trouble-a">
            <strong>Causa:</strong> No se cargó el entorno de ROS 2<br />
            <strong>Solución:</strong> Ejecuta <code>source ~/.bashrc</code> o abre una nueva
            terminal
          </div>
        </div>

        <div class="trouble-item">
          <div class="trouble-q">❌ "Unable to locate package ros-jazzy-desktop"</div>
          <div class="trouble-a">
            <strong>Causa:</strong> Repositorios no configurados correctamente<br />
            <strong>Solución:</strong> Verifica que <code>sudo apt update</code> muestre URLs de
            "packages.ros.org"
          </div>
        </div>

        <div class="trouble-item">
          <div class="trouble-q">❌ "Unmet dependencies"</div>
          <div class="trouble-a">
            <strong>Causa:</strong> Conflictos de versiones de paquetes<br />
            <strong>Solución:</strong> <code>sudo apt --fix-broken install</code> y luego reintenta
          </div>
        </div>

        <div class="trouble-item">
          <div class="trouble-q">⚠️ "GPG error: NO_PUBKEY"</div>
          <div class="trouble-a">
            <strong>Causa:</strong> Llave GPG expirada o no instalada<br />
            <strong>Solución:</strong> Repite el paso 3 (configuración de repositorios)
          </div>
        </div>

        <div class="trouble-item">
          <div class="trouble-q">❌ RViz2 no abre (error de display)</div>
          <div class="trouble-a">
            <strong>Causa:</strong> Problema con aceleración gráfica (común en VMs)<br />
            <strong>Solución:</strong> Habilita aceleración 3D en VirtualBox/VMware
          </div>
        </div>

        <div class="trouble-item">
          <div class="trouble-q">⚠️ Instalación muy lenta</div>
          <div class="trouble-a">
            <strong>Causa:</strong> Servidor de paquetes sobrecargado<br />
            <strong>Solución:</strong> Cambia a un mirror más cercano en Software & Updates
          </div>
        </div>
      </div>
    </div>

    <!-- ========== CTA FINAL ========== -->
    <div class="section-group self-stretch column items-center q-mt-xl">
      <div class="final-cta">
        <q-icon name="celebration" size="xl" color="primary" class="q-mb-md" />
        <h2 class="text-h4 text-white text-center q-mb-md">¡Instalación Completada! 🎉</h2>
        <p class="text-body1 text-grey-4 text-center q-mb-lg">
          Has configurado exitosamente ROS 2 Jazzy en tu sistema. Ahora estás listo para comenzar a
          desarrollar robots profesionales.
        </p>

        <div class="row q-gutter-md justify-center">
          <q-btn
            color="primary"
            unelevated
            rounded
            size="lg"
            padding="14px 40px"
            to="/modulo-0/01nav-sistema"
            icon="rocket_launch"
            label="Comenzar con Linux"
            class="text-weight-bold"
          />
          <q-btn
            flat
            rounded
            size="lg"
            color="white"
            icon="arrow_back"
            label="Volver a Introducción"
            to="/introduccion"
          />
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';

// ========== PRE-FLIGHT CHECKLIST ==========
const preflightItems = ref([
  {
    label: 'Ubuntu 24.04 (Noble) instalado',
    detail: 'Verifica con: lsb_release -a',
    checked: false,
  },
  {
    label: 'Al menos 20GB de espacio libre',
    detail: 'Verifica con: df -h',
    checked: false,
  },
  {
    label: 'Conexión a internet estable',
    detail: 'Necesaria para descargar ~800MB de paquetes',
    checked: false,
  },
  {
    label: 'Acceso sudo (administrador)',
    detail: 'Verifica con: sudo -v',
    checked: false,
  },
]);

const togglePreflight = (index: number) => {
  if (preflightItems.value[index]) {
    preflightItems.value[index].checked = !preflightItems.value[index].checked;
  }
};

const checkedCount = computed(() => preflightItems.value.filter((item) => item.checked).length);

const preflightProgress = computed(() => {
  return Math.round((checkedCount.value / preflightItems.value.length) * 100);
});

// ========== INSTALLATION STAGES ==========
const currentStage = ref(0);

const installationStages = [
  { title: 'Verificación OS', description: 'Ubuntu 24.04', icon: 'verified' },
  { title: 'Configuración', description: 'Locale y repos', icon: 'settings' },
  { title: 'Instalación ROS 2', description: 'Paquetes principales', icon: 'download' },
  { title: 'Workspace', description: 'Estructura de trabajo', icon: 'folder' },
  { title: 'Verificación', description: 'Tests funcionales', icon: 'check_circle' },
];

// ========== INSTALLATION VARIANT ==========
const selectedVariant = ref('desktop');

const getInstallCommand = () => {
  const variant = selectedVariant.value === 'desktop' ? 'ros-jazzy-desktop' : 'ros-jazzy-ros-base';
  return `sudo apt update
sudo apt upgrade
sudo apt install ${variant}`;
};

// ========== COMANDOS ==========
const osCheckCommand = `lsb_release -a
# Debe mostrar: Ubuntu 24.04 LTS (Noble Numbat)`;

const localeCommand = `locale  # Verificar configuración actual

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verificar que ahora muestre UTF-8`;

const repoCommand = `# 1. Habilitar repositorio Universe
sudo apt install software-properties-common
sudo add-apt-repository universe

# 2. Instalar curl
sudo apt update && sudo apt install curl -y

# 3. Descargar configuración de ROS 2
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/\${ROS_APT_SOURCE_VERSION}/ros2-apt-source_\${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo \${UBUNTU_CODENAME:-\${VERSION_CODENAME}})_all.deb"

# 4. Instalar el paquete
sudo dpkg -i /tmp/ros2-apt-source.deb

# 5. Actualizar índices
sudo apt update`;

const devToolsCommand = `sudo apt update
sudo apt install ros-dev-tools`;

const bashrcCommand = `# Agregar al final del .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Recargar configuración
source ~/.bashrc

# Verificar
echo $ROS_DISTRO  # Debe mostrar: jazzy`;

const workspaceCommand = `# Crear estructura de workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Compilar workspace vacío (inicialización)
colcon build

# Configurar para que se cargue automáticamente
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc`;
</script>

<style scoped>
/* ========== PÁGINA BASE ========== */
.installation-page {
  background: #0f172a;
}

.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 4rem auto;
}

/* ========== HERO ÉPICO ========== */
.installation-hero {
  position: relative;
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 4rem auto;
  padding: 5rem 2rem;
  background:
    radial-gradient(circle at top right, rgba(56, 189, 248, 0.2), transparent 70%),
    radial-gradient(circle at bottom left, rgba(139, 92, 246, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.95);
  backdrop-filter: blur(30px);
  border-radius: 32px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
  overflow: hidden;
}

.hero-content {
  position: relative;
  z-index: 1;
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 900;
  margin: 0 0 1rem 0;
  line-height: 1.1;
  color: #f8fafc;
  letter-spacing: -0.02em;
}

.text-gradient {
  background: linear-gradient(135deg, #06b6d4 0%, #8b5cf6 50%, #3b82f6 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-subtitle {
  font-size: 1.25rem;
  color: #cbd5e1;
  line-height: 1.6;
  max-width: 700px;
  margin: 0 auto;
}

.time-estimate {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  background: rgba(30, 41, 59, 0.6);
  border-radius: 20px;
  color: #cbd5e1;
  font-size: 0.95rem;
}

/* Animaciones de entrada */
.animate-fade-in {
  animation: fadeIn 0.8s ease-out;
}

.animate-slide-up {
  animation: slideUp 0.8s ease-out 0.2s both;
}

.animate-slide-up-delay {
  animation: slideUp 0.8s ease-out 0.4s both;
}

.animate-fade-in-delay {
  animation: fadeIn 0.8s ease-out 0.6s both;
}

@keyframes fadeIn {
  from {
    opacity: 0;
  }
  to {
    opacity: 1;
  }
}

@keyframes slideUp {
  from {
    opacity: 0;
    transform: translateY(30px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* ========== PRE-FLIGHT CHECKLIST ========== */
.preflight-card {
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.6), rgba(15, 23, 42, 0.8));
  backdrop-filter: blur(20px);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 2rem;
  max-width: 800px;
  margin: 0 auto;
}

.preflight-items {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.preflight-item {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1rem;
  cursor: pointer;
  transition: all 0.3s ease;
}

.preflight-item:hover {
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(99, 102, 241, 0.4);
  transform: translateX(5px);
}

.preflight-checkbox {
  font-size: 1.1rem;
  font-weight: 600;
  color: #f8fafc;
}

.preflight-detail {
  margin-top: 0.5rem;
  margin-left: 2rem;
  font-size: 0.9rem;
  color: #94a3b8;
}

.preflight-progress {
  text-align: center;
  margin-top: 1.5rem;
}

.progress-label {
  font-size: 1rem;
  font-weight: 600;
  color: #cbd5e1;
  margin-bottom: 0.5rem;
}

/* ========== PROGRESS STEPPER ========== */
.progress-stepper {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  max-width: 800px;
  margin: 0 auto;
}

.stepper-item {
  display: flex;
  align-items: center;
  gap: 1.5rem;
  padding: 1.5rem;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  cursor: pointer;
  transition: all 0.3s ease;
}

.stepper-item:hover {
  background: rgba(30, 41, 59, 0.6);
  transform: translateX(5px);
}

.stepper-item.active {
  background: rgba(99, 102, 241, 0.15);
  border-color: rgba(99, 102, 241, 0.4);
  border-left: 4px solid #6366f1;
}

.stepper-item.completed {
  opacity: 0.7;
}

.stepper-marker {
  flex-shrink: 0;
}

.stepper-content {
  flex: 1;
}

.stepper-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f8fafc;
  margin-bottom: 0.25rem;
}

.stepper-desc {
  font-size: 0.9rem;
  color: #94a3b8;
}

/* ========== COMPATIBILITY TABLE ========== */
.compatibility-table {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  overflow: hidden;
}

.compat-row {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 1rem;
  padding: 1rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
  color: #cbd5e1;
}

.compat-row:last-child {
  border-bottom: none;
}

.compat-row.header {
  background: rgba(30, 41, 59, 0.6);
  font-weight: 700;
  color: #f8fafc;
}

.compat-row.recommended {
  background: rgba(56, 189, 248, 0.1);
  border-left: 4px solid #38bdf8;
  font-weight: 600;
}

.compat-row.disabled {
  opacity: 0.5;
}

/* ========== INSTALL OPTIONS ========== */
.install-options {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.install-card {
  background: rgba(30, 41, 59, 0.4);
  border: 2px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
  cursor: pointer;
  transition: all 0.3s ease;
}

.install-card:hover {
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(99, 102, 241, 0.4);
}

.install-card.desktop {
  border-left: 4px solid #4ade80;
}

.install-card.base {
  border-left: 4px solid #94a3b8;
}

.install-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 0.75rem;
}

.install-header h4 {
  margin: 0;
  font-size: 1.1rem;
  font-weight: 700;
  color: #f8fafc;
  flex: 1;
}

.install-card p {
  color: #cbd5e1;
  margin: 0 0 0.75rem 0;
  line-height: 1.6;
}

.install-meta {
  font-size: 0.85rem;
  color: #94a3b8;
  font-style: italic;
}

.install-progress-info {
  background: rgba(30, 41, 59, 0.4);
  border-radius: 12px;
  padding: 1rem;
}

.info-item {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 0.5rem 0;
  color: #cbd5e1;
  font-size: 0.95rem;
}

/* ========== WORKSPACE DIAGRAM ========== */
.workspace-diagram {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 2rem;
  max-width: 600px;
  margin: 0 auto;
}

.diagram-node {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 0.75rem 1rem;
  background: rgba(15, 23, 42, 0.6);
  border-radius: 8px;
  margin-bottom: 0.75rem;
  color: #f8fafc;
  font-family: 'Fira Code', monospace;
}

.diagram-node.root {
  background: rgba(99, 102, 241, 0.2);
  border: 1px solid rgba(99, 102, 241, 0.4);
  font-weight: 700;
  font-size: 1.1rem;
}

.diagram-children {
  margin-left: 2rem;
  margin-top: 1rem;
  border-left: 2px solid rgba(148, 163, 184, 0.2);
  padding-left: 1rem;
}

.node-desc {
  margin-left: auto;
  font-size: 0.85rem;
  color: #94a3b8;
  font-family: 'Inter', sans-serif;
}

/* ========== VERIFICATION TESTS ========== */
.verification-tests {
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.test-item {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 1.5rem;
}

.test-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 1rem;
}

.test-header h4 {
  margin: 0;
  font-size: 1.1rem;
  font-weight: 700;
  color: #f8fafc;
}

.terminal-label {
  font-size: 0.9rem;
  font-weight: 600;
  color: #cbd5e1;
  margin-bottom: 0.5rem;
}

.test-expected {
  margin-top: 1rem;
  padding: 1rem;
  background: rgba(56, 189, 248, 0.1);
  border-left: 3px solid #38bdf8;
  border-radius: 8px;
  color: #cbd5e1;
  font-size: 0.95rem;
}

.test-expected code {
  background: rgba(15, 23, 42, 0.6);
  padding: 2px 6px;
  border-radius: 4px;
  color: #06b6d4;
  font-family: 'Fira Code', monospace;
}

/* ========== TROUBLESHOOTING GRID ========== */
.trouble-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1rem;
}

.trouble-item {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(239, 68, 68, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  transition: all 0.3s ease;
}

.trouble-item:hover {
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(239, 68, 68, 0.4);
}

.trouble-q {
  font-weight: 700;
  margin-bottom: 0.75rem;
  font-size: 1rem;
  color: #fca5a5;
}

.trouble-a {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
}

.trouble-a strong {
  color: #f8fafc;
}

.trouble-a code {
  background: rgba(15, 23, 42, 0.6);
  padding: 2px 6px;
  border-radius: 4px;
  color: #06b6d4;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

/* ========== CTA FINAL ========== */
.final-cta {
  text-align: center;
  padding: 3rem 2rem;
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.4), rgba(15, 23, 42, 0.6));
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.1);
  max-width: 800px;
  margin: 0 auto;
}

/* ========== RESPONSIVE ========== */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.5rem;
  }

  .hero-subtitle {
    font-size: 1.1rem;
  }

  .progress-stepper {
    gap: 0.75rem;
  }

  .stepper-item {
    padding: 1rem;
  }

  .compat-row {
    grid-template-columns: 1fr;
    gap: 0.5rem;
  }

  .trouble-grid {
    grid-template-columns: 1fr;
  }

  .diagram-children {
    margin-left: 1rem;
  }
}
</style>
