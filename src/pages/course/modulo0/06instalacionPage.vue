<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      Olvida buscar "setup.exe" en Google. En Linux, el software viaja por tuberías seguras llamadas
      <strong>Repositorios</strong>. Aprende a dominar APT (el gestor oficial), entender PIP
      (Python), y sobrevivir al "Infierno de Dependencias". <br /><br />
      Esta lección te prepara para instalar ROS 2, librerías de IA, y cualquier herramienta que
      necesites sin romper tu sistema.
    </TextBlock>

    <!-- TRES VÍAS -->
    <div class="section-group">
      <SectionTitle>1. Las Tres Vías de Instalación</SectionTitle>

      <div class="install-methods">
        <div class="method-card apt">
          <div class="method-icon">
            <q-icon name="verified" size="4rem" color="orange-6" />
          </div>
          <div class="method-name">APT</div>
          <div class="method-subtitle">Advanced Package Tool</div>
          <div class="method-desc">
            Software <strong>verificado</strong> por Ubuntu. Estable, seguro, con actualizaciones
            automáticas. Ideal para herramientas del sistema.
          </div>
          <div class="method-example">
            <strong>Ejemplos:</strong> git, python3, ros-humble-desktop
          </div>
        </div>

        <div class="method-card pip">
          <div class="method-icon">
            <q-icon name="code" size="4rem" color="blue-5" />
          </div>
          <div class="method-name">PIP</div>
          <div class="method-subtitle">Python Package Index</div>
          <div class="method-desc">
            Librerías de Python puras. Lo último en IA y machine learning. Más flexible pero menos
            integrado con el sistema.
          </div>
          <div class="method-example">
            <strong>Ejemplos:</strong> numpy, tensorflow, opencv-python
          </div>
        </div>

        <div class="method-card source">
          <div class="method-icon">
            <q-icon name="build" size="4rem" color="grey-4" />
          </div>
          <div class="method-name">Source</div>
          <div class="method-subtitle">Git + Compile</div>
          <div class="method-desc">
            Clonas el código fuente y lo compilas tú mismo. Máximo control, máxima complejidad. Para
            drivers de robots y código experimental.
          </div>
          <div class="method-example">
            <strong>Ejemplos:</strong> Tu workspace ROS 2, drivers de sensores
          </div>
        </div>
      </div>
    </div>

    <!-- APT PROFUNDO -->
    <div class="section-group">
      <SectionTitle>2. Dominando APT: El Gestor Oficial</SectionTitle>
      <TextBlock>
        APT es tu intendente de software. Antes de pedirle algo, debes decirle que actualice su
        catálogo. Si no haces <code>update</code>, pedirás versiones fantasma que ya no existen.
      </TextBlock>

      <div class="apt-workflow q-mt-md">
        <div class="apt-step">
          <div class="apt-step-num">1</div>
          <div class="apt-step-content">
            <div class="apt-step-title">Actualizar Catálogo</div>
            <CodeBlock
              lang="bash"
              content="sudo apt update
# Descarga la lista de paquetes disponibles"
              :copyable="true"
            />
            <div class="apt-step-note">Hazlo SIEMPRE antes de instalar algo nuevo</div>
          </div>
        </div>

        <div class="apt-step">
          <div class="apt-step-num">2</div>
          <div class="apt-step-content">
            <div class="apt-step-title">Instalar Paquete</div>
            <CodeBlock
              lang="bash"
              content="sudo apt install git
# Instala git y todas sus dependencias"
              :copyable="true"
            />
            <div class="apt-step-note">APT resuelve dependencias automáticamente</div>
          </div>
        </div>

        <div class="apt-step">
          <div class="apt-step-num">3</div>
          <div class="apt-step-content">
            <div class="apt-step-title">Actualizar Sistema (Opcional)</div>
            <CodeBlock
              lang="bash"
              content="sudo apt upgrade
# Actualiza TODOS los paquetes instalados"
              :copyable="true"
            />
            <div class="apt-step-note">Hazlo semanalmente para parches de seguridad</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Comandos APT Esenciales</SectionTitle>
        <div class="row q-col-gutter-md q-mt-sm">
          <div class="col-12 col-md-6">
            <div class="cmd-card">
              <code class="cmd-name">apt search</code>
              <div class="cmd-desc">Buscar paquetes por nombre o descripción</div>
              <CodeBlock lang="bash" content="apt search opencv" :copyable="true" />
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="cmd-card">
              <code class="cmd-name">apt show</code>
              <div class="cmd-desc">Ver información detallada de un paquete</div>
              <CodeBlock lang="bash" content="apt show python3-numpy" :copyable="true" />
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="cmd-card">
              <code class="cmd-name">apt list --installed</code>
              <div class="cmd-desc">Ver todos los paquetes instalados</div>
              <CodeBlock lang="bash" content="apt list --installed | grep ros" :copyable="true" />
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="cmd-card">
              <code class="cmd-name">apt remove</code>
              <div class="cmd-desc">Desinstalar un paquete</div>
              <CodeBlock lang="bash" content="sudo apt remove paquete" :copyable="true" />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- PIP Y VENV -->
    <div class="section-group">
      <SectionTitle>3. Python y el Caos: PIP vs APT</SectionTitle>

      <AlertBlock type="danger" title="⚠️ Alerta Ubuntu 24.04+ (PEP 668)">
        Si intentas <code>pip install numpy</code> globalmente, Ubuntu te bloqueará con:
        <strong>"Externally Managed Environment"</strong>. Linux te protege de romper dependencias
        del sistema operativo.
      </AlertBlock>

      <div class="pip-comparison q-mt-md">
        <div class="comparison-card correct">
          <div class="comparison-header">
            <q-icon name="check_circle" color="green-4" size="lg" />
            <span>✅ Opción 1: APT (Sistema Global)</span>
          </div>
          <div class="comparison-content">
            <TextBlock>
              Si necesitas una librería común (numpy, pandas) para <strong>todo el sistema</strong>,
              usa APT. Está integrada y actualizada por Ubuntu.
            </TextBlock>
            <CodeBlock
              lang="bash"
              content="sudo apt install python3-numpy python3-pandas
# Disponible para todos los usuarios"
              :copyable="true"
            />
            <div class="comparison-pros">
              <strong>Ventajas:</strong> Estable, actualizado, sin conflictos
            </div>
          </div>
        </div>

        <div class="comparison-card correct">
          <div class="comparison-header">
            <q-icon name="check_circle" color="blue-4" size="lg" />
            <span>✅ Opción 2: VENV (Entorno Virtual)</span>
          </div>
          <div class="comparison-content">
            <TextBlock>
              Si necesitas una <strong>versión específica</strong> o una librería experimental, crea
              un entorno virtual (sandbox aislado).
            </TextBlock>
            <CodeBlock
              lang="bash"
              content="# Crear entorno virtual
python3 -m venv mi_proyecto_env

# Activar
source mi_proyecto_env/bin/activate

# Ahora puedes usar pip libremente
pip install tensorflow==2.15.0

# Desactivar
deactivate"
              :copyable="true"
            />
            <div class="comparison-pros">
              <strong>Ventajas:</strong> Aislado, versiones específicas, sin riesgo
            </div>
          </div>
        </div>

        <div class="comparison-card wrong">
          <div class="comparison-header">
            <q-icon name="cancel" color="red-4" size="lg" />
            <span>❌ NUNCA: pip install global</span>
          </div>
          <div class="comparison-content">
            <CodeBlock
              lang="bash"
              content="# ¡NO HAGAS ESTO!
pip install numpy  # Fallará en Ubuntu 24.04+
sudo pip install numpy  # Romperá tu sistema"
            />
            <div class="comparison-cons">
              <strong>Peligro:</strong> Conflictos con APT, sistema inestable
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ROSDEP -->
    <div class="section-group">
      <SectionTitle>4. rosdep: El Gestor de ROS 2</SectionTitle>
      <TextBlock>
        <code>rosdep</code> es una herramienta mágica que lee los archivos <code>package.xml</code>
        de tus paquetes ROS 2 e instala TODAS las dependencias automáticamente (usando APT).
      </TextBlock>

      <div class="rosdep-demo q-mt-md">
        <div class="rosdep-scenario">
          <div class="scenario-title">Escenario: Clonas un paquete de GitHub</div>
          <div class="scenario-problem">
            El paquete necesita: <code>opencv</code>, <code>pcl</code>, <code>eigen</code>... ¿Cómo
            sabes qué instalar?
          </div>
        </div>
        <div class="rosdep-solution">
          <div class="solution-title">Solución: rosdep lo hace por ti</div>
          <CodeBlock
            lang="bash"
            content="# 1. Inicializar rosdep (solo una vez)
sudo rosdep init
rosdep update

# 2. Instalar dependencias de tu workspace
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Resultado: Todas las dependencias instaladas automáticamente"
            :copyable="true"
          />
        </div>
      </div>

      <AlertBlock type="success" title="Tip Pro" class="q-mt-md">
        Ejecuta <code>rosdep install</code> cada vez que clones un paquete nuevo de GitHub. Te
        ahorrará horas de búsqueda manual de dependencias.
      </AlertBlock>
    </div>

    <!-- CASOS DE USO -->
    <div class="section-group">
      <SectionTitle>5. Casos de Uso Reales en ROS 2</SectionTitle>

      <div class="use-cases-grid">
        <div class="use-case">
          <div class="use-case-header">
            <q-icon name="download" color="orange-4" size="md" />
            <span>Instalar ROS 2 Humble</span>
          </div>
          <div class="use-case-content">
            <CodeBlock
              lang="bash"
              content="# Agregar repositorio de ROS
sudo apt install software-properties-common
sudo add-apt-repository universe

# Agregar clave GPG
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Agregar fuente
echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar
sudo apt update
sudo apt install ros-humble-desktop"
              :copyable="true"
            />
          </div>
        </div>

        <div class="use-case">
          <div class="use-case-header">
            <q-icon name="science" color="blue-4" size="md" />
            <span>Instalar Librerías de IA</span>
          </div>
          <div class="use-case-content">
            <CodeBlock
              lang="bash"
              content="# Crear entorno virtual para proyecto de IA
python3 -m venv ~/ai_env
source ~/ai_env/bin/activate

# Instalar stack de IA
pip install tensorflow torch opencv-python numpy pandas

# Usar en tu proyecto
python mi_detector.py"
              :copyable="true"
            />
          </div>
        </div>

        <div class="use-case">
          <div class="use-case-header">
            <q-icon name="build_circle" color="green-4" size="md" />
            <span>Compilar Paquete desde Source</span>
          </div>
          <div class="use-case-content">
            <CodeBlock
              lang="bash"
              content="# Clonar repositorio
cd ~/ros2_ws/src
git clone https://github.com/ros2/demos.git

# Instalar dependencias
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y

# Compilar
colcon build --packages-select demo_nodes_cpp"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <q-expansion-item
        icon="error"
        label="E: Unable to locate package ros-humble-desktop"
        header-class="error-header"
      >
        <div class="error-content">
          <strong>Causa:</strong> No agregaste el repositorio de ROS 2 o no hiciste
          <code>apt update</code>. <br /><br />
          <strong>Solución:</strong>
          <ol>
            <li>Verifica que agregaste el repositorio correctamente</li>
            <li>Ejecuta <code>sudo apt update</code></li>
            <li>Intenta de nuevo</li>
          </ol>
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="error: externally-managed-environment (PIP)"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> Intentaste usar <code>pip install</code> globalmente en Ubuntu
          24.04+. <br /><br />
          <strong>Solución:</strong> Usa APT o crea un entorno virtual:
          <CodeBlock
            lang="bash"
            content="# Opción 1: APT
sudo apt install python3-nombre-paquete

# Opción 2: VENV
python3 -m venv mi_env
source mi_env/bin/activate
pip install paquete"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="rosdep: command not found"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> No instalaste las herramientas de desarrollo de ROS 2.
          <br /><br />
          <strong>Solución:</strong>
          <CodeBlock
            lang="bash"
            content="sudo apt install python3-rosdep
sudo rosdep init
rosdep update"
            :copyable="true"
          />
        </div>
      </q-expansion-item>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Instalación de Software en Linux"
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
      <SectionTitle>📝 Resumen de Comandos Esenciales</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>sudo apt update</code>
          <span>Actualizar catálogo de paquetes</span>
        </div>
        <div class="summary-item">
          <code>sudo apt install</code>
          <span>Instalar software del sistema</span>
        </div>
        <div class="summary-item">
          <code>python3 -m venv</code>
          <span>Crear entorno virtual Python</span>
        </div>
        <div class="summary-item">
          <code>pip install</code>
          <span>Instalar librerías Python (en venv)</span>
        </div>
        <div class="summary-item">
          <code>rosdep install</code>
          <span>Instalar dependencias de ROS 2</span>
        </div>
        <div class="summary-item">
          <code>apt search</code>
          <span>Buscar paquetes disponibles</span>
        </div>
      </div>

      <AlertBlock type="success" title="Regla de Oro" class="q-mt-lg">
        <strong>Para herramientas del sistema:</strong> APT
        <br />
        <strong>Para librerías Python globales:</strong> APT (python3-nombre)
        <br />
        <strong>Para proyectos Python específicos:</strong> VENV + PIP
        <br />
        <strong>Para dependencias de ROS 2:</strong> rosdep
      </AlertBlock>
    </div>
    <!-- ========== CTA FINAL ========== -->
    <div class="section-group q-mt-xl self-stretch column items-center">
      <div class="final-cta">
        <q-icon name="celebration" size="xl" color="primary" class="q-mb-md" />
        <h2 class="text-h4 text-white text-center q-mb-md">¡Has finalizado el módulo! 🎉</h2>
        <p class="text-body1 text-grey-4 text-center q-mb-lg">
          Has finalizado el módulo de fundamentos de Linux.
        </p>

        <div class="row q-gutter-md justify-center">
          <q-btn
            color="primary"
            unelevated
            rounded
            size="lg"
            padding="14px 40px"
            to="/modulo-1/01python-scripts"
            icon="rocket_launch"
            label="Comenzar con Fundamentos de Programación"
            class="text-weight-bold"
          />
        </div>
      </div>
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

.install-methods {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.method-card {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  text-align: center;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.method-card.apt {
  border-top: 4px solid #f97316;
}
.method-card.pip {
  border-top: 4px solid #3b82f6;
}
.method-card.source {
  border-top: 4px solid #94a3b8;
}

.method-icon {
  margin-bottom: 0.5rem;
}

.method-name {
  font-size: 1.75rem;
  font-weight: 700;
  color: #f1f5f9;
}

.method-subtitle {
  font-size: 0.85rem;
  color: #94a3b8;
  font-style: italic;
}

.method-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
  line-height: 1.6;
  flex-grow: 1;
}

.method-example {
  background: rgba(0, 0, 0, 0.3);
  padding: 0.75rem;
  border-radius: 8px;
  font-size: 0.85rem;
  color: #94a3b8;
}

.apt-workflow {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.apt-step {
  display: flex;
  gap: 1.5rem;
  align-items: flex-start;
}

.apt-step-num {
  min-width: 3rem;
  height: 3rem;
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

.apt-step-content {
  flex-grow: 1;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.apt-step-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.apt-step-note {
  margin-top: 0.75rem;
  padding: 0.5rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  font-size: 0.85rem;
  color: #94a3b8;
}

.cmd-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.25rem;
  height: 100%;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.cmd-name {
  font-family: 'Fira Code', monospace;
  font-size: 1.1rem;
  color: #fbbf24;
  font-weight: 700;
}

.cmd-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  flex-grow: 1;
}

.pip-comparison {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.comparison-card {
  border-radius: 16px;
  overflow: hidden;
}

.comparison-card.correct {
  border: 2px solid rgba(34, 197, 94, 0.5);
}

.comparison-card.wrong {
  border: 2px solid rgba(239, 68, 68, 0.5);
}

.comparison-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  font-weight: 700;
  font-size: 1.05rem;
}

.comparison-card.correct .comparison-header {
  background: rgba(20, 83, 45, 0.3);
  color: #86efac;
}

.comparison-card.wrong .comparison-header {
  background: rgba(127, 29, 29, 0.3);
  color: #fca5a5;
}

.comparison-content {
  padding: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
}

.comparison-pros,
.comparison-cons {
  margin-top: 1rem;
  padding: 0.75rem;
  border-radius: 8px;
  font-size: 0.9rem;
}

.comparison-pros {
  background: rgba(34, 197, 94, 0.1);
  color: #86efac;
}

.comparison-cons {
  background: rgba(239, 68, 68, 0.1);
  color: #fca5a5;
}

.rosdep-demo {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.rosdep-scenario {
  margin-bottom: 1.5rem;
}

.scenario-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #60a5fa;
  margin-bottom: 0.75rem;
}

.scenario-problem {
  color: #cbd5e1;
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
  border-left: 3px solid #f97316;
}

.solution-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #22c55e;
  margin-bottom: 0.75rem;
}

.use-cases-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.use-case {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
}

.use-case-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  background: rgba(59, 130, 246, 0.1);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  font-weight: 700;
  color: #f1f5f9;
}

.use-case-content {
  padding: 1.5rem;
}

:deep(.error-header) {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
  border-radius: 8px;
  color: #fca5a5;
}

.error-content {
  background: rgba(15, 23, 42, 0.6);
  padding: 1.5rem;
  color: #cbd5e1;
}

.error-content ol,
.error-content ul {
  margin-top: 0.5rem;
  padding-left: 1.5rem;
}

.error-content li {
  margin-bottom: 0.5rem;
}

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

.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
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
  .use-cases-grid {
    grid-template-columns: 1fr;
  }
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
</style>
