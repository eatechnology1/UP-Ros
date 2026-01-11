<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Un <strong>Workspace</strong> en ROS 2 es el entorno de desarrollo donde se organizan,
      compilan y ejecutan paquetes. Implementa el concepto de <strong>overlay/underlay</strong>,
      permitiendo extender instalaciones base sin modificarlas, fundamental para desarrollo modular
      y testing aislado.
    </TextBlock>

    <AlertBlock type="info" title="Estructura Canónica">
      <strong>src/</strong> - Código fuente (repositorios Git)
      <br />
      <strong>build/</strong> - Artefactos de compilación (CMake cache, objetos)
      <br />
      <strong>install/</strong> - Binarios ejecutables y bibliotecas
      <br />
      <strong>log/</strong> - Logs de compilación y ejecución
    </AlertBlock>

    <!-- WORKSPACE ANATOMY -->
    <div class="section-group">
      <SectionTitle>1. Anatomía del Workspace</SectionTitle>

      <div class="workspace-visual q-mt-md">
        <div class="workspace-header">
          <q-icon name="folder" size="2rem" />
          <span>~/ros2_ws/</span>
        </div>

        <div class="workspace-structure">
          <div class="folder-item src">
            <div class="folder-icon">
              <q-icon name="folder_open" size="1.5rem" />
            </div>
            <div class="folder-content">
              <div class="folder-name">src/</div>
              <div class="folder-desc">Zona de desarrollo - Tu código fuente</div>
              <div class="folder-details">
                <div class="detail-item">
                  <q-icon name="edit" size="sm" />
                  <span>Editable</span>
                </div>
                <div class="detail-item">
                  <q-icon name="git_hub" size="sm" />
                  <span>Versionado (Git)</span>
                </div>
              </div>
            </div>
          </div>

          <div class="folder-item build">
            <div class="folder-icon">
              <q-icon name="handyman" size="1.5rem" />
            </div>
            <div class="folder-content">
              <div class="folder-name">build/</div>
              <div class="folder-desc">Artefactos temporales de compilación</div>
              <div class="folder-details">
                <div class="detail-item">
                  <q-icon name="block" size="sm" />
                  <span>No editar</span>
                </div>
                <div class="detail-item">
                  <q-icon name="delete" size="sm" />
                  <span>Regenerable</span>
                </div>
              </div>
            </div>
          </div>

          <div class="folder-item install">
            <div class="folder-icon">
              <q-icon name="system_update_alt" size="1.5rem" />
            </div>
            <div class="folder-content">
              <div class="folder-name">install/</div>
              <div class="folder-desc">Binarios finales y setup scripts</div>
              <div class="folder-details">
                <div class="detail-item">
                  <q-icon name="play_arrow" size="sm" />
                  <span>Ejecutable</span>
                </div>
                <div class="detail-item">
                  <q-icon name="settings" size="sm" />
                  <span>setup.bash</span>
                </div>
              </div>
            </div>
          </div>

          <div class="folder-item log">
            <div class="folder-icon">
              <q-icon name="history" size="1.5rem" />
            </div>
            <div class="folder-content">
              <div class="folder-name">log/</div>
              <div class="folder-desc">Logs de colcon y runtime</div>
              <div class="folder-details">
                <div class="detail-item">
                  <q-icon name="bug_report" size="sm" />
                  <span>Debugging</span>
                </div>
                <div class="detail-item">
                  <q-icon name="search" size="sm" />
                  <span>Análisis</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="Crear workspace desde cero"
          lang="bash"
          content="# Crear estructura
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Inicializar con paquetes de ejemplo
git clone https://github.com/ros2/examples src/examples

# Compilar
colcon build

# Activar workspace
source install/setup.bash"
          :copyable="true"
        />
      </div>
    </div>

    <!-- BUILD SYSTEM -->
    <div class="section-group">
      <SectionTitle>2. Sistema de Build: colcon</SectionTitle>
      <TextBlock>
        <strong>colcon</strong> (collective construction) es el sistema de build de ROS 2. Orquesta
        CMake/ament_cmake para C++ y setuptools para Python, permitiendo compilación paralela y
        gestión de dependencias entre paquetes.
      </TextBlock>

      <div class="build-pipeline q-mt-md">
        <div class="pipeline-stage">
          <div class="stage-number">1</div>
          <div class="stage-content">
            <div class="stage-title">Dependency Resolution</div>
            <div class="stage-desc">
              colcon analiza package.xml de todos los paquetes y construye un grafo de dependencias
              topológicamente ordenado
            </div>
            <CodeBlock lang="bash" content="colcon graph" :copyable="true" />
          </div>
        </div>

        <div class="pipeline-arrow">↓</div>

        <div class="pipeline-stage">
          <div class="stage-number">2</div>
          <div class="stage-content">
            <div class="stage-title">CMake Configuration</div>
            <div class="stage-desc">
              Para paquetes C++, ejecuta cmake con ament_cmake macros. Genera Makefiles en build/
            </div>
            <CodeBlock
              lang="bash"
              content="# Equivalente a:
cmake -DCMAKE_INSTALL_PREFIX=install/pkg_name \
      -DCMAKE_BUILD_TYPE=Release \
      src/pkg_name"
              :copyable="true"
            />
          </div>
        </div>

        <div class="pipeline-arrow">↓</div>

        <div class="pipeline-stage">
          <div class="stage-number">3</div>
          <div class="stage-content">
            <div class="stage-title">Parallel Compilation</div>
            <div class="stage-desc">
              Compila paquetes en paralelo respetando dependencias. Usa todos los cores disponibles
              por defecto
            </div>
            <CodeBlock
              lang="bash"
              content="# Controlar paralelismo
colcon build --parallel-workers 4

# Ver progreso detallado
colcon build --event-handlers console_direct+"
              :copyable="true"
            />
          </div>
        </div>

        <div class="pipeline-arrow">↓</div>

        <div class="pipeline-stage">
          <div class="stage-number">4</div>
          <div class="stage-content">
            <div class="stage-title">Installation</div>
            <div class="stage-desc">
              Copia binarios, bibliotecas y recursos a install/. Genera setup scripts para sourcing
            </div>
            <CodeBlock
              lang="bash"
              content="# Symlink install (desarrollo rápido)
colcon build --symlink-install

# Evita reinstalar archivos sin cambios"
              :copyable="true"
            />
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Opciones Avanzadas de colcon</SectionTitle>
        <div class="colcon-options q-mt-md">
          <div class="option-card">
            <div class="option-header">
              <q-icon name="speed" color="green-4" />
              <span>Performance</span>
            </div>
            <CodeBlock
              lang="bash"
              content="# Compilación incremental
colcon build --packages-select my_pkg

# Solo paquetes modificados
colcon build --packages-up-to my_pkg

# Build types
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# Debug, RelWithDebInfo, MinSizeRel"
              :copyable="true"
            />
          </div>

          <div class="option-card">
            <div class="option-header">
              <q-icon name="cleaning_services" color="orange-4" />
              <span>Limpieza</span>
            </div>
            <CodeBlock
              lang="bash"
              content="# Limpiar build artifacts
rm -rf build/ install/ log/

# Limpiar paquete específico
colcon build --packages-select my_pkg --cmake-clean-cache"
              :copyable="true"
            />
          </div>

          <div class="option-card">
            <div class="option-header">
              <q-icon name="bug_report" color="purple-4" />
              <span>Debugging</span>
            </div>
            <CodeBlock
              lang="bash"
              content="# Verbose output
colcon build --event-handlers console_cohesion+

# Continuar en errores
colcon build --continue-on-error

# Ver comandos ejecutados
colcon build --cmake-args --trace"
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- PACKAGE STRUCTURE -->
    <div class="section-group">
      <SectionTitle>3. Estructura de Paquetes</SectionTitle>
      <TextBlock>
        Un paquete es la unidad atómica de software en ROS 2. Contiene código, configuración,
        dependencias y metadatos necesarios para compilación y distribución.
      </TextBlock>

      <div class="package-types q-mt-md">
        <div class="package-card cpp">
          <div class="package-header">
            <q-icon name="code" size="2rem" />
            <span>Paquete C++ (ament_cmake)</span>
          </div>
          <div class="package-structure">
            <div class="structure-item">
              <q-icon name="description" />
              <span>package.xml</span>
              <div class="item-desc">Metadatos y dependencias</div>
            </div>
            <div class="structure-item">
              <q-icon name="description" />
              <span>CMakeLists.txt</span>
              <div class="item-desc">Build configuration</div>
            </div>
            <div class="structure-item">
              <q-icon name="folder" />
              <span>include/pkg_name/</span>
              <div class="item-desc">Headers públicos (.hpp)</div>
            </div>
            <div class="structure-item">
              <q-icon name="folder" />
              <span>src/</span>
              <div class="item-desc">Implementación (.cpp)</div>
            </div>
          </div>
          <CodeBlock
            title="CMakeLists.txt mínimo"
            lang="cmake"
            content="cmake_minimum_required(VERSION 3.8)
project(my_robot_pkg)

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Ejecutable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# Instalación
install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()"
            :copyable="true"
          />
        </div>

        <div class="package-card python">
          <div class="package-header">
            <q-icon name="code" size="2rem" />
            <span>Paquete Python (ament_python)</span>
          </div>
          <div class="package-structure">
            <div class="structure-item">
              <q-icon name="description" />
              <span>package.xml</span>
              <div class="item-desc">Metadatos y dependencias</div>
            </div>
            <div class="structure-item">
              <q-icon name="description" />
              <span>setup.py</span>
              <div class="item-desc">Setuptools configuration</div>
            </div>
            <div class="structure-item">
              <q-icon name="description" />
              <span>setup.cfg</span>
              <div class="item-desc">Entry points</div>
            </div>
            <div class="structure-item">
              <q-icon name="folder" />
              <span>pkg_name/</span>
              <div class="item-desc">Módulos Python (.py)</div>
            </div>
          </div>
          <CodeBlock
            title="setup.py mínimo"
            lang="python"
            content="from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My robot package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
        ],
    },
)"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- DEPENDENCY MANAGEMENT -->
    <div class="section-group">
      <SectionTitle>4. Gestión de Dependencias: package.xml</SectionTitle>
      <TextBlock>
        El archivo <code>package.xml</code> (formato REP-149) declara metadatos y dependencias del
        paquete. Soporta tres tipos de dependencias con semántica específica.
      </TextBlock>

      <div class="dependency-types q-mt-md">
        <div class="dep-card">
          <div class="dep-header">
            <q-icon name="build" color="blue-4" />
            <span>&lt;build_depend&gt;</span>
          </div>
          <div class="dep-desc">
            Requerido durante compilación. Incluye headers, bibliotecas de desarrollo.
          </div>
          <div class="dep-example">
            <code>&lt;build_depend&gt;rclcpp&lt;/build_depend&gt;</code>
          </div>
        </div>

        <div class="dep-card">
          <div class="dep-header">
            <q-icon name="play_arrow" color="green-4" />
            <span>&lt;exec_depend&gt;</span>
          </div>
          <div class="dep-desc">
            Requerido en runtime. Bibliotecas compartidas, paquetes de datos.
          </div>
          <div class="dep-example">
            <code>&lt;exec_depend&gt;std_msgs&lt;/exec_depend&gt;</code>
          </div>
        </div>

        <div class="dep-card">
          <div class="dep-header">
            <q-icon name="science" color="purple-4" />
            <span>&lt;test_depend&gt;</span>
          </div>
          <div class="dep-desc">Requerido solo para tests. Frameworks de testing, mocks.</div>
          <div class="dep-example">
            <code>&lt;test_depend&gt;ament_cmake_gtest&lt;/test_depend&gt;</code>
          </div>
        </div>

        <div class="dep-card">
          <div class="dep-header">
            <q-icon name="all_inclusive" color="orange-4" />
            <span>&lt;depend&gt;</span>
          </div>
          <div class="dep-desc">
            Shorthand para build + exec. Usado cuando ambos son necesarios.
          </div>
          <div class="dep-example">
            <code>&lt;depend&gt;sensor_msgs&lt;/depend&gt;</code>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="package.xml completo"
          lang="xml"
          content='<?xml version="1.0"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>1.0.0</version>
  <description>Advanced robot control package</description>
  <maintainer email="dev@robot.com">Robot Team</maintainer>
  <license>Apache License 2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Dependencies -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <build_depend>geometry_msgs</build_depend>
  <exec_depend>nav2_msgs</exec_depend>

  <!-- Testing -->
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>'
          :copyable="true"
        />
      </div>

      <div class="q-mt-md">
        <SectionTitle>rosdep: Resolución Automática</SectionTitle>
        <TextBlock>
          <strong>rosdep</strong> resuelve dependencias de sistema operativo automáticamente,
          mapeando nombres de paquetes ROS a paquetes del sistema (apt, pip, etc.).
        </TextBlock>

        <CodeBlock
          lang="bash"
          content="# Inicializar rosdep (una vez)
sudo rosdep init
rosdep update

# Instalar dependencias del workspace
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Ver qué se instalará (dry-run)
rosdep install --from-paths src --ignore-src -r -y --simulate"
          :copyable="true"
        />
      </div>
    </div>

    <!-- OVERLAY/UNDERLAY -->
    <div class="section-group">
      <SectionTitle>5. Overlay/Underlay: Extensión Modular</SectionTitle>
      <TextBlock>
        ROS 2 permite <strong>encadenar workspaces</strong> mediante el mecanismo de
        overlay/underlay. Un workspace puede extender otro sin modificarlo, permitiendo desarrollo
        aislado y testing de cambios.
      </TextBlock>

      <div class="overlay-visual q-mt-md">
        <div class="layer underlay">
          <div class="layer-label">Underlay (Base)</div>
          <div class="layer-content">
            <div class="layer-item">/opt/ros/humble</div>
            <div class="layer-desc">Instalación oficial de ROS 2</div>
          </div>
        </div>

        <div class="layer-arrow">↑ extends</div>

        <div class="layer overlay1">
          <div class="layer-label">Overlay 1</div>
          <div class="layer-content">
            <div class="layer-item">~/ros2_ws</div>
            <div class="layer-desc">Tu workspace de desarrollo</div>
          </div>
        </div>

        <div class="layer-arrow">↑ extends</div>

        <div class="layer overlay2">
          <div class="layer-label">Overlay 2</div>
          <div class="layer-content">
            <div class="layer-item">~/test_ws</div>
            <div class="layer-desc">Workspace de testing</div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="Crear overlay"
          lang="bash"
          content="# 1. Source underlay
source /opt/ros/humble/setup.bash

# 2. Crear nuevo workspace
mkdir -p ~/overlay_ws/src
cd ~/overlay_ws

# 3. Clonar paquete a modificar
git clone https://github.com/ros2/examples src/examples

# 4. Compilar overlay
colcon build

# 5. Source overlay (sobrescribe underlay)
source install/setup.bash

# Ahora 'examples' viene de ~/overlay_ws, no de /opt/ros/humble"
          :copyable="true"
        />
      </div>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Orden de Sourcing">
          El orden de sourcing es crítico. El último workspace sourced tiene prioridad:
          <br />
          <br />
          <code>source /opt/ros/humble/setup.bash</code>
          <br />
          <code>source ~/ros2_ws/install/setup.bash</code>
          <br />
          <br />
          Paquetes en <code>~/ros2_ws</code> sobrescriben los de <code>/opt/ros/humble</code>
        </AlertBlock>
      </div>
    </div>

    <!-- OPTIMIZATION -->
    <div class="section-group">
      <SectionTitle>6. Optimización de Build</SectionTitle>

      <div class="optimization-grid q-mt-md">
        <div class="opt-card">
          <div class="opt-header">
            <q-icon name="link" color="blue-4" size="2rem" />
            <span>Symlink Install</span>
          </div>
          <div class="opt-desc">
            Crea enlaces simbólicos en lugar de copiar archivos. Cambios en src/ se reflejan
            inmediatamente sin recompilar.
          </div>
          <CodeBlock
            lang="bash"
            content="colcon build --symlink-install

# Útil para:
# - Desarrollo de Python (sin recompilación)
# - Launch files
# - Archivos de configuración"
            :copyable="true"
          />
          <div class="opt-benefit">⚡ 10x más rápido para iteración</div>
        </div>

        <div class="opt-card">
          <div class="opt-header">
            <q-icon name="memory" color="green-4" size="2rem" />
            <span>Compilación Paralela</span>
          </div>
          <div class="opt-desc">
            Usa múltiples cores para compilar paquetes independientes simultáneamente.
          </div>
          <CodeBlock
            lang="bash"
            content="# Auto-detect cores
colcon build

# Especificar workers
colcon build --parallel-workers 8

# Limitar memoria por worker
colcon build --executor sequential"
            :copyable="true"
          />
          <div class="opt-benefit">⚡ Speedup lineal con # cores</div>
        </div>

        <div class="opt-card">
          <div class="opt-header">
            <q-icon name="compress" color="purple-4" size="2rem" />
            <span>Build Types</span>
          </div>
          <div class="opt-desc">Diferentes niveles de optimización vs debugging.</div>
          <CodeBlock
            lang="bash"
            content="# Release (optimizado, sin debug symbols)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Debug (sin optimización, debug symbols)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# RelWithDebInfo (optimizado + debug symbols)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
            :copyable="true"
          />
          <div class="opt-benefit">⚡ Release: 3-5x más rápido que Debug</div>
        </div>

        <div class="opt-card">
          <div class="opt-header">
            <q-icon name="select_all" color="orange-4" size="2rem" />
            <span>Compilación Selectiva</span>
          </div>
          <div class="opt-desc">Compila solo paquetes modificados o específicos.</div>
          <CodeBlock
            lang="bash"
            content="# Solo un paquete
colcon build --packages-select my_pkg

# Paquete y sus dependencias
colcon build --packages-up-to my_pkg

# Excluir paquetes
colcon build --packages-skip heavy_pkg"
            :copyable="true"
          />
          <div class="opt-benefit">⚡ Evita recompilar todo el workspace</div>
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
            title="ROS 2 Workspace and Build System"
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
          Reemplaza con video técnico sobre workspaces y colcon
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>colcon build</code>
          <span>Sistema de build paralelo</span>
        </div>
        <div class="summary-item">
          <code>--symlink-install</code>
          <span>Desarrollo rápido</span>
        </div>
        <div class="summary-item">
          <code>package.xml</code>
          <span>Metadatos y dependencias</span>
        </div>
        <div class="summary-item">
          <code>rosdep</code>
          <span>Resolución automática</span>
        </div>
        <div class="summary-item">
          <code>Overlay/Underlay</code>
          <span>Extensión modular</span>
        </div>
        <div class="summary-item">
          <code>CMAKE_BUILD_TYPE</code>
          <span>Debug/Release/RelWithDebInfo</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Workspace" class="q-mt-lg">
        ✅ Usar <code>--symlink-install</code> para desarrollo iterativo
        <br />
        ✅ Compilar en Release para producción (<code>-DCMAKE_BUILD_TYPE=Release</code>)
        <br />
        ✅ Declarar todas las dependencias en <code>package.xml</code>
        <br />
        ✅ Usar <code>rosdep</code> para instalar dependencias de sistema
        <br />
        ✅ Crear overlays para testing sin modificar base
        <br />
        ✅ Limpiar build/ periódicamente para evitar cache corrupto
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

/* WORKSPACE VISUAL */
.workspace-visual {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(0, 217, 255, 0.3);
  border-radius: 16px;
  overflow: hidden;
}

.workspace-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  padding: 1.5rem;
  background: rgba(0, 217, 255, 0.1);
  border-bottom: 1px solid rgba(0, 217, 255, 0.2);
  font-size: 1.3rem;
  font-weight: 700;
  font-family: 'Fira Code', monospace;
  color: #00d9ff;
}

.workspace-structure {
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.folder-item {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  padding: 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  transition: all 0.3s ease;
}

.folder-item:hover {
  transform: translateX(8px);
  border-color: rgba(0, 217, 255, 0.5);
}

.folder-item.src {
  border-left: 4px solid #00ff88;
}

.folder-item.build {
  border-left: 4px solid #ff6b35;
}

.folder-item.install {
  border-left: 4px solid #00d9ff;
}

.folder-item.log {
  border-left: 4px solid #8b9dc3;
}

.folder-icon {
  color: #00d9ff;
}

.folder-name {
  font-size: 1.2rem;
  font-weight: 700;
  font-family: 'Fira Code', monospace;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.folder-desc {
  color: #cbd5e1;
  margin-bottom: 1rem;
}

.folder-details {
  display: flex;
  gap: 1rem;
}

.detail-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  color: #94a3b8;
  font-size: 0.85rem;
}

/* BUILD PIPELINE */
.build-pipeline {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.pipeline-stage {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 2rem;
}

.stage-number {
  width: 50px;
  height: 50px;
  background: linear-gradient(135deg, #667eea, #764ba2);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.stage-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #00d9ff;
  margin-bottom: 0.75rem;
}

.stage-desc {
  color: #cbd5e1;
  margin-bottom: 1rem;
}

.pipeline-arrow {
  text-align: center;
  color: #00ff88;
  font-size: 2rem;
  font-weight: 700;
}

/* COLCON OPTIONS */
.colcon-options {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.option-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.option-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
}

/* PACKAGE TYPES */
.package-types {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 2rem;
}

.package-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.package-card.cpp {
  border-color: #00d9ff;
}

.package-card.python {
  border-color: #fbbf24;
}

.package-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #f1f5f9;
}

.package-structure {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.structure-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  color: #cbd5e1;
  font-family: 'Fira Code', monospace;
}

.item-desc {
  margin-left: auto;
  font-size: 0.75rem;
  color: #64748b;
  font-family: sans-serif;
}

/* DEPENDENCY TYPES */
.dependency-types {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.dep-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.dep-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  font-family: 'Fira Code', monospace;
  color: #f1f5f9;
}

.dep-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.dep-example {
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.5);
  border-radius: 6px;
  font-family: 'Fira Code', monospace;
  color: #00ff88;
  font-size: 0.85rem;
}

/* OVERLAY VISUAL */
.overlay-visual {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.layer {
  padding: 1.5rem;
  border-radius: 12px;
  border: 2px solid;
}

.layer.underlay {
  border-color: #8b9dc3;
  background: rgba(139, 157, 195, 0.1);
}

.layer.overlay1 {
  border-color: #00d9ff;
  background: rgba(0, 217, 255, 0.1);
}

.layer.overlay2 {
  border-color: #00ff88;
  background: rgba(0, 255, 136, 0.1);
}

.layer-label {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
  font-size: 1.1rem;
}

.layer-item {
  font-family: 'Fira Code', monospace;
  color: #00d9ff;
  font-size: 1.05rem;
  margin-bottom: 0.5rem;
}

.layer-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

.layer-arrow {
  text-align: center;
  color: #00ff88;
  font-weight: 700;
  font-size: 1.2rem;
}

/* OPTIMIZATION GRID */
.optimization-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.opt-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.opt-header {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
}

.opt-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.opt-benefit {
  padding: 1rem;
  background: rgba(0, 255, 136, 0.1);
  border: 1px solid #00ff88;
  border-radius: 8px;
  color: #00ff88;
  font-weight: 700;
  text-align: center;
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
  color: #00d9ff;
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 1024px) {
  .package-types,
  .dependency-types,
  .optimization-grid {
    grid-template-columns: 1fr;
  }
}
</style>
