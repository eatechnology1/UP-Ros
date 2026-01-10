<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: CONFIGURACIÓN -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Módulo 1 / Setup</div>
        <h1 class="hero-title">Instalación de <span class="text-primary">ROS 2 Jazzy</span></h1>

        <TextBlock>
          Configura un entorno de desarrollo profesional en <strong>Ubuntu 24.04</strong> con la
          guía más completa. Usaremos paquetes <strong>Debian oficiales</strong> para garantizar
          estabilidad y actualizaciones automáticas.
        </TextBlock>
      </div>
    </section>

    <!-- 2. MATRIZ DE COMPATIBILIDAD -->
    <div class="section-group self-stretch">
      <SectionTitle>⚠️ Requisito Crítico: Versión de OS</SectionTitle>

      <SplitBlock>
        <template #left>
          <TextBlock>
            ROS 2 Jazzy depende de las bibliotecas de sistema (glibc, python 3.12) de
            <strong>Ubuntu 24.04 (Noble Numbat)</strong>. Forzar la instalación en otra versión
            romperá tu sistema.
          </TextBlock>

          <!-- Tabla con estilo Dark Mode integrado -->
          <div class="compatibility-container q-mt-md">
            <q-markup-table flat dense class="bg-transparent text-grey-4">
              <thead>
                <tr>
                  <th class="text-left text-white">Ubuntu Version</th>
                  <th class="text-left text-white">ROS 2 Distro</th>
                  <th class="text-left text-white">Estado</th>
                </tr>
              </thead>
              <tbody>
                <tr class="bg-primary-soft">
                  <td class="text-weight-bold text-white">24.04 (Noble)</td>
                  <td class="text-secondary text-weight-bold">Jazzy Jalisco</td>
                  <td>✅ LTS Activa (2029)</td>
                </tr>
                <tr>
                  <td>22.04 (Jammy)</td>
                  <td>Humble Hawksbill</td>
                  <td>⚠️ LTS Anterior</td>
                </tr>
                <tr class="text-grey-6">
                  <td>20.04 (Focal)</td>
                  <td>Foxy Fitzroy</td>
                  <td>❌ EOL (Obsoleto)</td>
                </tr>
              </tbody>
            </q-markup-table>
          </div>
        </template>

        <template #right>
          <AlertBlock title="🛑 Verificación Obligatoria" type="warning">
            Ejecuta el comando abajo. Si no ves "Noble" o "24.04", <strong>DETENTE</strong> ahora
            mismo.
          </AlertBlock>
          <CodeBlock title="Verificar OS" lang="bash" content="lsb_release -a" />
        </template>
      </SplitBlock>
    </div>

    <!-- 3. CONFIGURACIÓN DE LOCALE -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Configuración de Locale (UTF-8)</SectionTitle>
      <TextBlock>
        ROS 2 utiliza UTF-8 para la codificación de mensajes en el bus de datos. Sin esto, los nodos
        fallarán al iniciar.
      </TextBlock>
      <CodeBlock title="Configurar Locale" lang="bash" :content="localeCode" />
    </div>

    <!-- 4. REPOSITORIOS -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Agregar Repositorios Oficiales</SectionTitle>
      <TextBlock>
        Usaremos el paquete oficial <code>ros2-apt-source</code> que maneja automáticamente las
        llaves GPG.
      </TextBlock>
      <StepsBlock :steps="repoSteps" class="q-mb-md" />
      <CodeBlock title="Configurar Fuentes" lang="bash" :content="sourcesCode" />
    </div>

    <!-- 5. HERRAMIENTAS DEV -->
    <div class="section-group self-stretch">
      <SectionTitle>3. Herramientas de Desarrollo</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Aunque la documentación oficial lo marca como opcional,
            <strong>para este curso es obligatorio</strong>. Necesitas estas herramientas para
            compilar paquetes (Colcon) y gestionar dependencias (Rosdep).
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock
            title="Instalar ros-dev-tools"
            lang="bash"
            content="sudo apt update && sudo apt install ros-dev-tools"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 6. INSTALACIÓN CORE -->
    <div class="section-group self-stretch">
      <SectionTitle>4. Instalar ROS 2 Jazzy</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock> Selecciona la variante adecuada para tu necesidad. </TextBlock>

          <div class="row q-col-gutter-md q-mt-xs">
            <!-- Opción Desktop (Recomendada) -->
            <div class="col-12">
              <div class="tool-card desktop-install">
                <div class="tool-header">
                  <q-icon name="monitor" size="sm" />
                  <h4 class="text-subtitle1 text-white q-my-none">ros-jazzy-desktop</h4>
                  <span class="tool-badge text-primary">Recomendado</span>
                </div>
                <p class="text-caption q-my-sm text-grey-4">
                  Incluye ROS Core + RViz2 (Visualización) + Demos. Peso: ~800MB.
                </p>
              </div>
            </div>

            <!-- Opción Base -->
            <div class="col-12">
              <div class="tool-card base-install">
                <div class="tool-header">
                  <q-icon name="memory" size="sm" />
                  <h4 class="text-subtitle1 text-white q-my-none">ros-jazzy-ros-base</h4>
                </div>
                <p class="text-caption q-my-sm text-grey-4">
                  Solo bibliotecas de comunicación. Sin GUI. Para despliegue final.
                </p>
              </div>
            </div>
          </div>

          <div class="q-mt-md">
            <AlertBlock title="💡 Consejo Pro" type="info">
              Siempre ejecuta <code>sudo apt upgrade</code> antes de instalar para evitar "Broken
              Packages".
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <CodeBlock title="Instalar Desktop" lang="bash" :content="installCode" />
        </template>
      </SplitBlock>
    </div>

    <!-- 7. SETUP DE ENTORNO -->
    <div class="section-group self-stretch">
      <SectionTitle>5. Automatización (.bashrc)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Escribir <code>source /opt/ros/jazzy/setup.bash</code> cada vez que abres una terminal
            es tedioso y propenso a errores. Configuramos el <code>.bashrc</code> para que cargue
            ROS 2 automáticamente al iniciar.
          </TextBlock>
        </template>
        <template #right>
          <CodeBlock title="Configurar Bashrc" lang="bash" :content="setupCode" />
        </template>
      </SplitBlock>
    </div>

    <!-- 8. VERIFICACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>6. Verificación (Hello World)</SectionTitle>
      <TextBlock>Prueba de fuego: comunicación entre C++ y Python.</TextBlock>

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

    <!-- 9. TROUBLESHOOTING (Estilo Grid) -->
    <div class="section-group self-stretch">
      <SectionTitle>🔧 Solución de Problemas</SectionTitle>
      <TextBlock>Errores clásicos de instalación y cómo arreglarlos.</TextBlock>

      <div class="trouble-grid q-mt-md">
        <!-- Error 1 -->
        <div class="trouble-item">
          <div class="trouble-q text-red-4">❌ "ros2: command not found"</div>
          <div class="trouble-a">
            No se reinició la terminal tras configurar el bashrc. Ejecuta
            <code>source ~/.bashrc</code> o abre una nueva ventana.
          </div>
        </div>

        <!-- Error 2 -->
        <div class="trouble-item">
          <div class="trouble-q text-red-4">❌ "Unable to locate package"</div>
          <div class="trouble-a">
            Falta actualizar índices. Ejecuta <code>sudo apt update</code> y verifica que aparezcan
            URLs de "packages.ros.org".
          </div>
        </div>

        <!-- Error 3 -->
        <div class="trouble-item">
          <div class="trouble-q text-red-4">❌ "Unmet dependencies"</div>
          <div class="trouble-a">
            Conflicto de versiones. Solución: <code>sudo apt --fix-broken install</code> y luego
            reintenta.
          </div>
        </div>

        <!-- Error 4 -->
        <div class="trouble-item">
          <div class="trouble-q text-orange-4">⚠️ "GPG error... NO_PUBKEY"</div>
          <div class="trouble-a">
            La llave de seguridad expiró. Repite el <strong>Paso 2</strong> completo para bajar la
            llave nueva.
          </div>
        </div>
      </div>
    </div>

    <!-- 10. DESINSTALACIÓN -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>🚑 Zona de Peligro: Desinstalación</SectionTitle>
      <TextBlock>Solo si necesitas limpiar el sistema para empezar de cero.</TextBlock>
      <CodeBlock title="Borrar todo rastro" lang="bash" :content="uninstallCode" />
    </div>

    <!-- NAVEGACIÓN -->
    <div class="row justify-center q-mt-xl q-gutter-md">
      <q-btn
        color="primary"
        unelevated
        rounded
        padding="12px 32px"
        to="/fundamentos"
        icon="school"
        label="Ir a Fundamentos"
        class="text-weight-bold"
      />
      <q-btn flat rounded color="white" icon="arrow_back" label="Volver" to="/introduccion" />
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

// Pasos Texto
const repoSteps = [
  'Habilitar repositorio <strong>Universe</strong>.',
  'Instalar utilidades <code>curl</code> y <code>software-properties-common</code>.',
  'Descargar script oficial de fuentes.',
  'Instalar configuración de apt (.deb).',
];

// Comandos
const localeCode = `
locale  # Verificar. Debe decir UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8`.trim();

// Escapado corregido para visualización en CodeBlock
const sourcesCode = `
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/\${ROS_APT_SOURCE_VERSION}/ros2-apt-source_\${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo \${UBUNTU_CODENAME:-\${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb`.trim();

const installCode = `
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop`.trim();

const setupCode = `
# Agregar al final del .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Recargar configuración
source ~/.bashrc`.trim();

const uninstallCode = `
sudo apt remove ~nros-jazzy-* && sudo apt autoremove
sudo apt remove ros2-apt-source
sudo apt update`.trim();
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
    radial-gradient(circle at top right, rgba(56, 189, 248, 0.15), transparent 60%),
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

/* TABLE STYLING */
.compatibility-container {
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  background: rgba(30, 41, 59, 0.3);
}
.bg-primary-soft {
  background: rgba(56, 189, 248, 0.15); /* Primary color low opacity */
}

/* CARDS ESTILO TOOL (Simplificadas para esta vista) */
.tool-card {
  padding: 16px;
  border-radius: 12px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}
.tool-card.desktop-install {
  border-left: 4px solid #4ade80;
} /* Green */
.tool-card.base-install {
  border-left: 4px solid #94a3b8;
} /* Grey */

.tool-header {
  display: flex;
  align-items: center;
  gap: 10px;
}
.tool-badge {
  margin-left: auto;
  font-size: 0.75rem;
  font-weight: 700;
  text-transform: uppercase;
}

/* GRID DE TROUBLESHOOTING */
.trouble-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 16px;
}

.trouble-item {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(239, 68, 68, 0.2); /* Red tint border */
  border-radius: 12px;
  padding: 16px;
  transition: transform 0.2s;
}

.trouble-item:hover {
  background: rgba(30, 41, 59, 0.6);
}

.trouble-q {
  font-weight: 700;
  margin-bottom: 8px;
  font-size: 0.95rem;
}

.trouble-a {
  color: #cbd5e1;
  font-size: 0.85rem;
  line-height: 1.4;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
