<template>
  <q-page class="q-pa-lg column items-center">
    <!-- HERO -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <h1 class="hero-title">Instalación de <span class="accent">ROS 2 Jazzy</span></h1>
        <TextBlock>
          Configura un entorno de desarrollo profesional en <strong>Ubuntu 24.04</strong> con la
          guía más completa. Usaremos paquetes <strong>Debian oficiales</strong> para garantizar
          estabilidad y actualizaciones automáticas.
        </TextBlock>
      </div>
    </section>

    <!-- MATRIZ DE COMPATIBILIDAD -->
    <div class="section-group self-stretch">
      <SectionTitle>⚠️ Requisito Crítico: Versión de Ubuntu</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-7">
          <TextBlock>
            ROS 2 Jazzy está compilado específicamente para las bibliotecas de sistema de
            <strong>Ubuntu 24.04 (Noble Numbat)</strong>. No intentes forzar la instalación en otras
            versiones; romperás tu sistema.
          </TextBlock>

          <div class="compatibility-table q-mt-md">
            <q-markup-table flat bordered dense>
              <thead>
                <tr class="bg-primary text-white">
                  <th class="text-left">Ubuntu Version</th>
                  <th class="text-left">ROS 2 Compatible</th>
                  <th class="text-left">Soporte</th>
                </tr>
              </thead>
              <tbody>
                <tr class="bg-green-1 text-weight-bold">
                  <td>24.04 (Noble)</td>
                  <td class="text-positive">Jazzy Jalisco</td>
                  <td>✅ Activo (LTS hasta 2029)</td>
                </tr>
                <tr>
                  <td>22.04 (Jammy)</td>
                  <td>Humble Hawksbill</td>
                  <td>⚠️ LTS Anterior (Estable)</td>
                </tr>
                <tr class="text-grey-5">
                  <td>20.04 (Focal)</td>
                  <td>Foxy Fitzroy</td>
                  <td>❌ Fin de Vida (EOL)</td>
                </tr>
              </tbody>
            </q-markup-table>
          </div>
        </div>

        <div class="col-12 col-md-5">
          <AlertBlock title="🛑 Verificación Obligatoria" type="warning">
            Ejecuta este comando. Si no ves "Noble" o "24.04", <strong>DETENTE</strong>.
          </AlertBlock>
          <CodeBlock title="Verificar OS" lang="bash" content="lsb_release -a" />
        </div>
      </div>
    </div>

    <!-- PASO 1: LOCALE -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Configuración de Locale (UTF-8)</SectionTitle>
      <TextBlock>
        ROS 2 utiliza UTF-8 para la codificación de mensajes. Si tu sistema está en una
        configuración mínima (como POSIX), los nodos fallarán al iniciar o mostrarán caracteres
        corruptos.
      </TextBlock>
      <CodeBlock title="Configurar Locale" lang="bash" :content="localeCode" />
    </div>

    <!-- PASO 2: REPOSITORIOS -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Agregar Repositorios Oficiales</SectionTitle>
      <TextBlock>
        Ubuntu no incluye ROS 2 por defecto. Usaremos el paquete oficial
        <code>ros2-apt-source</code> que maneja automáticamente las llaves de seguridad GPG,
        evitando problemas futuros de "llaves expiradas".
      </TextBlock>

      <StepsBlock :steps="repoSteps" class="q-mb-md" />

      <CodeBlock title="Configurar Fuentes" lang="bash" :content="sourcesCode" />
    </div>

    <!-- PASO 3: HERRAMIENTAS DEV -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Herramientas de Desarrollo (Vital)</SectionTitle>
      <TextBlock>
        La documentación oficial marca esto como opcional, pero
        <strong>para este curso es obligatorio</strong>. Necesitas estas herramientas para compilar
        tus propios paquetes, usar simuladores y gestionar dependencias.
      </TextBlock>
      <CodeBlock
        title="Instalar ros-dev-tools"
        lang="bash"
        content="sudo apt update && sudo apt install ros-dev-tools"
      />
    </div>

    <!-- PASO 4: INSTALACIÓN CORE -->
    <div class="section-group self-stretch">
      <SectionTitle>4. Instalar ROS 2 Jazzy</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock> Procedemos a instalar el sistema completo. </TextBlock>
          <div class="q-pl-md q-mb-md">
            <div class="text-weight-bold text-primary">
              📦 ros-jazzy-desktop (Elección del Curso)
            </div>
            <div class="text-caption text-grey-7 q-mb-sm">
              Incluye: ROS Core + RViz2 (Visualización) + Demos + Tutoriales. <br />
              <em>Peso aprox: ~800MB.</em>
            </div>

            <div class="text-weight-bold text-grey-8">📦 ros-jazzy-ros-base</div>
            <div class="text-caption text-grey-7">
              Solo bibliotecas de comunicación. Sin GUI. (Para robots reales sin pantalla).
            </div>
          </div>
          <AlertBlock title="💡 Consejo Pro" type="info">
            Siempre ejecuta <code>sudo apt upgrade</code> antes de instalar para evitar errores de
            dependencias rotas.
          </AlertBlock>
        </template>
        <template #right>
          <CodeBlock title="Instalar Desktop" lang="bash" :content="installCode" />
        </template>
      </SplitBlock>
    </div>

    <!-- PASO 5: SETUP DE ENTORNO -->
    <div class="section-group self-stretch">
      <SectionTitle>5. Automatización del Entorno (.bashrc)</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-7">
          <TextBlock>
            ROS 2 no se carga automáticamente para no interferir con el sistema. Sin embargo,
            escribir <code>source /opt/ros/jazzy/setup.bash</code> cada vez que abres una terminal
            es tedioso. <br /><br />
            Configuraremos tu archivo <code>.bashrc</code> para que lo haga por ti.
          </TextBlock>
        </div>
        <div class="col-12 col-md-5">
          <CodeBlock title="Configurar Bashrc" lang="bash" :content="setupCode" />
        </div>
      </div>
    </div>

    <!-- PASO 6: PRUEBA -->
    <div class="section-group self-stretch">
      <SectionTitle>6. Verificación (Talker/Listener)</SectionTitle>
      <TextBlock>
        Prueba de fuego: dos nodos comunicándose. Abre <strong>dos terminales nuevas</strong>.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12 col-md-6">
          <div class="text-subtitle2 q-mb-xs text-primary">Terminal 1 (Emisor C++)</div>
          <CodeBlock lang="bash" content="ros2 run demo_nodes_cpp talker" />
        </div>
        <div class="col-12 col-md-6">
          <div class="text-subtitle2 q-mb-xs text-secondary">Terminal 2 (Receptor Python)</div>
          <CodeBlock lang="bash" content="ros2 run demo_nodes_py listener" />
        </div>
      </div>
    </div>

    <!-- SECCIÓN DE ERRORES COMUNES (RECUPERADA Y MEJORADA) -->
    <div class="section-group self-stretch">
      <SectionTitle>🔧 Solución de Problemas Frecuentes</SectionTitle>
      <div class="row q-col-gutter-md">
        <!-- Error 1 -->
        <div class="col-12 col-md-6">
          <q-card flat bordered class="error-card">
            <q-card-section>
              <div class="text-weight-bold text-red-5">❌ Error: "ros2: command not found"</div>
              <div class="text-caption text-grey-4 q-my-sm">
                Sucede al abrir una terminal nueva.
              </div>
              <div class="text-caption">
                <strong>Causa:</strong> No se ejecutó el Paso 5 correctamente o no reiniciaste la
                terminal.<br />
                <strong>Solución:</strong> Ejecuta <code>source ~/.bashrc</code> o cierra y abre la
                terminal.
              </div>
            </q-card-section>
          </q-card>
        </div>

        <!-- Error 2 -->
        <div class="col-12 col-md-6">
          <q-card flat bordered class="error-card">
            <q-card-section>
              <div class="text-weight-bold text-red-5">❌ Error: "Unable to locate package"</div>
              <div class="text-caption text-grey-4 q-my-sm">
                Al intentar instalar <code>ros-jazzy-desktop</code>.
              </div>
              <div class="text-caption">
                <strong>Causa:</strong> No se agregaron los repositorios (Paso 2) o no hiciste
                <code>apt update</code>.<br />
                <strong>Solución:</strong> Repite el Paso 2 y asegúrate de ver "Get: ...
                packages.ros.org" al actualizar.
              </div>
            </q-card-section>
          </q-card>
        </div>

        <!-- Error 3 -->
        <div class="col-12 col-md-6">
          <q-card flat bordered class="error-card">
            <q-card-section>
              <div class="text-weight-bold text-red-5">❌ Error: "Unmet dependencies"</div>
              <div class="text-caption text-grey-4 q-my-sm">
                Conflicto de paquetes durante la instalación.
              </div>
              <div class="text-caption">
                <strong>Solución:</strong> Ejecuta <code>sudo apt --fix-broken install</code> y
                luego intenta instalar de nuevo. A veces requiere
                <code>sudo apt upgrade</code> previo.
              </div>
            </q-card-section>
          </q-card>
        </div>

        <!-- Error 4 -->
        <div class="col-12 col-md-6">
          <q-card flat bordered class="error-card">
            <q-card-section>
              <div class="text-weight-bold text-orange-4">⚠️ Warning: "GPG error... NO_PUBKEY"</div>
              <div class="text-caption text-grey-4 q-my-sm">
                Problema con la llave de seguridad.
              </div>
              <div class="text-caption">
                <strong>Solución:</strong> La llave cambió o expiró. Repite la descarga del archivo
                <code>ros2-apt-source</code> del Paso 2.
              </div>
            </q-card-section>
          </q-card>
        </div>
      </div>
    </div>

    <!-- DESINSTALACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>🚑 Desinstalación (Emergencia)</SectionTitle>
      <TextBlock>
        Si necesitas limpiar tu sistema completamente para reintentar desde cero:
      </TextBlock>
      <CodeBlock title="Borrar todo rastro de ROS 2" lang="bash" :content="uninstallCode" />
    </div>

    <!-- NAVEGACIÓN -->
    <div class="cta-final row justify-center q-mt-xl q-mb-lg">
      <q-btn
        color="primary"
        unelevated
        size="lg"
        to="/fundamentos"
        icon="school"
        icon-after
        class="q-mr-lg"
      >
        Ir a Fundamentos
      </q-btn>
      <q-btn flat icon="arrow_back" label="Volver a Intro" to="/introduccion" />
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';

// Pasos
const repoSteps = [
  'Habilitar repositorio <strong>Universe</strong>.',
  'Instalar utilidades <code>curl</code> y <code>software-properties-common</code>.',
  'Descargar el script oficial de configuración de fuentes.',
  'Instalar el paquete de configuración (`.deb`).',
];

// Comandos Oficiales Actualizados
const localeCode = `
locale  # Verificar. Debe decir UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8`.trim();

// Versión corregida para ESLint no-useless-escape
const sourcesCode = `
# 1. Preparar dependencias
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# 2. Obtener la última versión del configurador de fuentes
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\\" '{print $4}')

# 3. Descargar el paquete .deb oficial
# Nota: \${VAR} escapa la interpolación de JS. $(...) es comando bash subshell.
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/\${ROS_APT_SOURCE_VERSION}/ros2-apt-source_\${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo \${UBUNTU_CODENAME:-\${VERSION_CODENAME}})_all.deb"

# 4. Instalar configuración
sudo dpkg -i /tmp/ros2-apt-source.deb`.trim();

const installCode = `
# 1. Actualizar caché de repositorios (¡Vital!)
sudo apt update

# 2. Actualizar sistema base
sudo apt upgrade

# 3. Instalar ROS 2 Desktop
sudo apt install ros-jazzy-desktop`.trim();

const setupCode = `
# Agregar al final del archivo de configuración personal
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Recargar configuración
source ~/.bashrc`.trim();

const uninstallCode = `
# Eliminar paquetes principales
sudo apt remove ~nros-jazzy-* && sudo apt autoremove

# Eliminar configuración de repositorios
sudo apt remove ros2-apt-source
sudo apt update`.trim();
</script>

<style scoped>
/* =========================================
   ESTRUCTURA GENERAL
========================================= */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3rem auto;
}

.section-group > * + * {
  margin-top: 1.5rem;
}

/* HERO */
.intro-hero {
  padding: 2.5rem;
  background: rgba(15, 23, 42, 0.85);
  backdrop-filter: blur(20px);
  border-radius: 20px;
  border: 1px solid rgba(148, 163, 184, 0.3);
  text-align: center;
}

.hero-title {
  font-size: 2.75rem;
  font-weight: 800;
  margin-bottom: 1.5rem;
  line-height: 1.2;
  background: linear-gradient(135deg, #6ecbff 0%, #ffffff 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.accent {
  background: linear-gradient(135deg, #6ecbff, #a5b4fc);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}

/* TABLA DE COMPATIBILIDAD */
.compatibility-table {
  border-radius: 12px;
  overflow: hidden;
  border: 1px solid rgba(148, 163, 184, 0.2);
}

.text-positive {
  color: #21ba45;
}

/* TARJETAS DE ERROR */
.error-card {
  background: rgba(30, 10, 10, 0.3);
  border-color: rgba(255, 100, 100, 0.2);
  height: 100%;
}

.cta-final {
  max-width: 600px;
  gap: 1.5rem;
  margin: 4rem auto 2rem auto !important;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2rem;
  }
  .intro-hero {
    padding: 1.5rem;
    margin-bottom: 2rem;
  }
  .section-group {
    margin-bottom: 2rem;
  }
}
</style>
