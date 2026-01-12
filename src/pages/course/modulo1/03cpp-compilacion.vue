<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      C++ es el lenguaje de <strong>alto rendimiento</strong> en ROS 2. Mientras Python ejecuta
      código línea por línea, C++ lo traduce a <strong>código de máquina</strong> antes de ejecutar.
      Esto lo hace 10-100x más rápido, crucial para control en tiempo real. <br /><br />
      Esta lección te enseña el proceso de compilación, CMake, y cómo depurar los temidos errores de
      linker que atormentan a todo desarrollador ROS 2.
    </TextBlock>

    <AlertBlock type="info" title="¿Cuándo usar C++ en ROS 2?">
      <strong>Usa C++ para:</strong> Drivers de sensores, control de motores, procesamiento de
      imágenes en tiempo real, algoritmos de navegación críticos
      <br />
      <strong>Usa Python para:</strong> Lógica de alto nivel, prototipado rápido, scripts de
      configuración, inteligencia artificial
    </AlertBlock>

    <!-- INTERPRETADO VS COMPILADO -->
    <div class="section-group">
      <SectionTitle>1. Interpretado vs Compilado: La Diferencia Clave</SectionTitle>

      <div class="comparison-grid">
        <div class="comparison-card python">
          <div class="comparison-header">
            <q-icon name="code" size="3rem" color="yellow-6" />
            <div class="comparison-title">Python (Interpretado)</div>
          </div>
          <div class="comparison-content">
            <div class="comparison-flow">
              <div class="flow-item">
                <q-icon name="description" color="yellow-6" />
                <span>script.py</span>
              </div>
              <div class="flow-arrow">→</div>
              <div class="flow-item">
                <q-icon name="play_arrow" color="yellow-6" />
                <span>Ejecutar</span>
              </div>
            </div>
            <div class="comparison-pros">
              <strong>Ventajas:</strong>
              <ul>
                <li>Desarrollo rápido (sin compilación)</li>
                <li>Fácil de depurar</li>
                <li>Multiplataforma automático</li>
              </ul>
            </div>
            <div class="comparison-cons">
              <strong>Desventajas:</strong>
              <ul>
                <li>10-100x más lento</li>
                <li>Errores en tiempo de ejecución</li>
                <li>No apto para tiempo real</li>
              </ul>
            </div>
          </div>
        </div>

        <div class="comparison-card cpp">
          <div class="comparison-header">
            <q-icon name="memory" size="3rem" color="blue-4" />
            <div class="comparison-title">C++ (Compilado)</div>
          </div>
          <div class="comparison-content">
            <div class="comparison-flow">
              <div class="flow-item">
                <q-icon name="description" color="blue-4" />
                <span>source.cpp</span>
              </div>
              <div class="flow-arrow">→</div>
              <div class="flow-item">
                <q-icon name="build" color="blue-4" />
                <span>Compilar</span>
              </div>
              <div class="flow-arrow">→</div>
              <div class="flow-item">
                <q-icon name="terminal" color="green-4" />
                <span>ejecutable</span>
              </div>
            </div>
            <div class="comparison-pros">
              <strong>Ventajas:</strong>
              <ul>
                <li>Máximo rendimiento</li>
                <li>Errores detectados antes de ejecutar</li>
                <li>Control total de memoria</li>
              </ul>
            </div>
            <div class="comparison-cons">
              <strong>Desventajas:</strong>
              <ul>
                <li>Ciclo de desarrollo más lento</li>
                <li>Curva de aprendizaje empinada</li>
                <li>Errores de compilación crípticos</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- PIPELINE DE COMPILACIÓN -->
    <div class="section-group">
      <SectionTitle>2. El Pipeline de Compilación: 3 Pasos Invisibles</SectionTitle>
      <TextBlock>
        Cuando ejecutas <code>colcon build</code>, ocurren 3 pasos detrás de escena. Entender esto
        es crucial para depurar errores en ROS 2.
      </TextBlock>

      <div class="pipeline-visual q-mt-md">
        <div class="pipeline-step">
          <div class="step-number">1</div>
          <div class="step-content">
            <div class="step-title">Preprocesador</div>
            <div class="step-icon">
              <q-icon name="content_cut" size="3rem" color="yellow-6" />
            </div>
            <div class="step-desc">
              Expande todos los <code>#include</code> y <code>#define</code>. Copia y pega el
              contenido de los headers dentro de tu archivo.
            </div>
            <div class="step-output"><strong>Salida:</strong> Archivo de texto gigante (.i)</div>
            <div class="step-errors">
              <strong>Errores típicos:</strong> "No such file or directory" (header no encontrado)
            </div>
          </div>
        </div>

        <div class="pipeline-arrow">
          <q-icon name="arrow_forward" size="2rem" />
        </div>

        <div class="pipeline-step">
          <div class="step-number">2</div>
          <div class="step-content">
            <div class="step-title">Compilador</div>
            <div class="step-icon">
              <q-icon name="translate" size="3rem" color="blue-4" />
            </div>
            <div class="step-desc">
              Traduce C++ a código de máquina (assembly). Verifica sintaxis y tipos. Cada archivo
              <code>.cpp</code> se compila <strong>por separado</strong>.
            </div>
            <div class="step-output"><strong>Salida:</strong> Archivos objeto (.o)</div>
            <div class="step-errors">
              <strong>Errores típicos:</strong> "Syntax error", "Undeclared identifier"
            </div>
          </div>
        </div>

        <div class="pipeline-arrow">
          <q-icon name="arrow_forward" size="2rem" />
        </div>

        <div class="pipeline-step">
          <div class="step-number">3</div>
          <div class="step-content">
            <div class="step-title">Linker (Enlazador)</div>
            <div class="step-icon">
              <q-icon name="link" size="3rem" color="green-4" />
            </div>
            <div class="step-desc">
              Une todos los archivos objeto (.o) y las librerías externas (rclcpp, etc.) en un solo
              ejecutable. <strong>Aquí ocurren el 90% de los errores feos.</strong>
            </div>
            <div class="step-output"><strong>Salida:</strong> Ejecutable binario</div>
            <div class="step-errors">
              <strong>Errores típicos:</strong> "Undefined reference to...", "Symbol not found"
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="warning" title="El 90% de los Errores: Linker">
          Los errores de linker son los más frustrantes porque el código está sintácticamente
          correcto, pero falta una <strong>pieza</strong> (librería no enlazada, función no
          implementada, etc.). Aprende a identificarlos.
        </AlertBlock>
      </div>
    </div>

    <!-- CMAKE -->
    <div class="section-group">
      <SectionTitle>3. CMake: El Arquitecto de la Compilación</SectionTitle>
      <TextBlock>
        En un proyecto ROS 2, tienes docenas de archivos <code>.cpp</code>, cada uno con sus
        dependencias. Compilarlos manualmente sería imposible. <strong>CMake</strong> automatiza
        todo el proceso.
      </TextBlock>

      <div class="cmake-explanation q-mt-md">
        <div class="cmake-header">
          <q-icon name="architecture" color="purple-4" size="lg" />
          <span>¿Qué hace CMake?</span>
        </div>
        <div class="cmake-steps">
          <div class="cmake-step">
            <div class="cmake-step-num">1</div>
            <div class="cmake-step-desc">
              Lee <code>CMakeLists.txt</code> (tu "receta de compilación")
            </div>
          </div>
          <div class="cmake-step">
            <div class="cmake-step-num">2</div>
            <div class="cmake-step-desc">
              Encuentra todas las dependencias (rclcpp, std_msgs, etc.)
            </div>
          </div>
          <div class="cmake-step">
            <div class="cmake-step-num">3</div>
            <div class="cmake-step-desc">
              Genera un <code>Makefile</code> (instrucciones para el compilador)
            </div>
          </div>
          <div class="cmake-step">
            <div class="cmake-step-num">4</div>
            <div class="cmake-step-desc">Ejecuta el compilador con los parámetros correctos</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Anatomía de CMakeLists.txt para ROS 2</SectionTitle>
        <CodeBlock
          title="CMakeLists.txt (Mínimo para ROS 2)"
          lang="cmake"
          content="cmake_minimum_required(VERSION 3.8)
project(mi_robot_cpp)

# Encuentra dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Define el ejecutable
add_executable(nodo_control
  src/control_node.cpp
  src/motor_driver.cpp
)

# Enlaza las librerías
ament_target_dependencies(nodo_control
  rclcpp
  std_msgs
)

# Instala el ejecutable
install(TARGETS nodo_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()"
          :copyable="true"
        />
      </div>

      <div class="q-mt-md">
        <div class="cmake-breakdown">
          <div class="breakdown-item">
            <div class="breakdown-label">find_package</div>
            <div class="breakdown-desc">
              Busca las librerías de ROS 2 en tu sistema. Si falta alguna, el build fallará aquí.
            </div>
          </div>
          <div class="breakdown-item">
            <div class="breakdown-label">add_executable</div>
            <div class="breakdown-desc">
              Define qué archivos <code>.cpp</code> compilar y cómo se llamará el ejecutable final.
            </div>
          </div>
          <div class="breakdown-item">
            <div class="breakdown-label">ament_target_dependencies</div>
            <div class="breakdown-desc">
              Enlaza las librerías de ROS 2 con tu ejecutable. Aquí es donde ocurren los errores de
              linker si falta algo.
            </div>
          </div>
          <div class="breakdown-item">
            <div class="breakdown-label">install</div>
            <div class="breakdown-desc">
              Copia el ejecutable a <code>install/lib/</code> para que ROS 2 pueda encontrarlo con
              <code>ros2 run</code>.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- PROCESO DE BUILD -->
    <div class="section-group">
      <SectionTitle>4. El Proceso de Build con Colcon</SectionTitle>
      <TextBlock>
        En ROS 2, usamos <code>colcon</code> (no CMake directamente) porque maneja múltiples
        paquetes a la vez y genera el entorno de ROS 2 automáticamente.
      </TextBlock>

      <div class="build-workflow q-mt-md">
        <div class="workflow-step">
          <div class="workflow-num">1</div>
          <div class="workflow-content">
            <div class="workflow-title">Navegar al Workspace</div>
            <CodeBlock lang="bash" content="cd ~/ros2_ws" :copyable="true" />
          </div>
        </div>

        <div class="workflow-step">
          <div class="workflow-num">2</div>
          <div class="workflow-content">
            <div class="workflow-title">Compilar (Build)</div>
            <CodeBlock
              lang="bash"
              content="colcon build --packages-select mi_robot_cpp
# O compilar todo:
# colcon build"
              :copyable="true"
            />
            <div class="workflow-note">
              Esto ejecuta CMake + compilador + linker para cada paquete
            </div>
          </div>
        </div>

        <div class="workflow-step">
          <div class="workflow-num">3</div>
          <div class="workflow-content">
            <div class="workflow-title">Cargar el Entorno</div>
            <CodeBlock lang="bash" content="source install/setup.bash" :copyable="true" />
            <div class="workflow-note">
              Agrega tus ejecutables al PATH para que ROS 2 los encuentre
            </div>
          </div>
        </div>

        <div class="workflow-step">
          <div class="workflow-num">4</div>
          <div class="workflow-content">
            <div class="workflow-title">Ejecutar</div>
            <CodeBlock lang="bash" content="ros2 run mi_robot_cpp nodo_control" :copyable="true" />
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="success" title="Flags Útiles de Colcon">
          <CodeBlock
            lang="bash"
            content="# Compilación más rápida (paralelo)
colcon build --parallel-workers 4

# Ver output detallado
colcon build --event-handlers console_direct+

# Compilar con símbolos de debug
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Limpiar y recompilar
rm -rf build install log
colcon build"
            :copyable="true"
          />
        </AlertBlock>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>5. Diccionario de Errores de Compilación</SectionTitle>

      <div class="error-cards">
        <div class="error-card compilation">
          <div class="error-header">
            <q-icon name="cancel" size="lg" />
            <span>Error de Compilación</span>
          </div>
          <div class="error-example">
            <code>error: 'cout' was not declared in this scope</code>
          </div>
          <div class="error-cause">
            <strong>Causa:</strong> Sintaxis incorrecta, variable no declarada, falta un
            <code>#include</code>
          </div>
          <div class="error-solution">
            <strong>Solución:</strong> Revisa el código en el archivo <code>.cpp</code> indicado.
            Agrega <code>#include &lt;iostream&gt;</code> si usas <code>cout</code>.
          </div>
        </div>

        <div class="error-card linker">
          <div class="error-header">
            <q-icon name="link_off" size="lg" />
            <span>Error de Linker</span>
          </div>
          <div class="error-example">
            <code>undefined reference to `rclcpp::Node::Node(...)'</code>
          </div>
          <div class="error-cause">
            <strong>Causa:</strong> El código está bien, pero falta enlazar una librería en
            <code>CMakeLists.txt</code>
          </div>
          <div class="error-solution">
            <strong>Solución:</strong> Agrega la librería faltante a
            <code>ament_target_dependencies</code>:
            <CodeBlock
              lang="cmake"
              content="ament_target_dependencies(nodo_control
  rclcpp  # ← Asegúrate de que esté aquí
)"
              :copyable="true"
            />
          </div>
        </div>

        <div class="error-card cmake">
          <div class="error-header">
            <q-icon name="build_circle" size="lg" />
            <span>Error de CMake</span>
          </div>
          <div class="error-example">
            <code>Could not find a package configuration file provided by "rclcpp"</code>
          </div>
          <div class="error-cause">
            <strong>Causa:</strong> CMake no encuentra la librería de ROS 2. No hiciste
            <code>source</code> o la librería no está instalada.
          </div>
          <div class="error-solution">
            <strong>Solución:</strong>
            <CodeBlock
              lang="bash"
              content="# 1. Hacer source del ROS 2 base
source /opt/ros/humble/setup.bash

# 2. Instalar la dependencia faltante
sudo apt install ros-humble-rclcpp"
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
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="Compilación de C++ en ROS 2"
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
          <code>colcon build</code>
          <span>Compilar todos los paquetes</span>
        </div>
        <div class="summary-item">
          <code>colcon build --packages-select pkg</code>
          <span>Compilar solo un paquete</span>
        </div>
        <div class="summary-item">
          <code>source install/setup.bash</code>
          <span>Cargar entorno compilado</span>
        </div>
        <div class="summary-item">
          <code>rm -rf build install</code>
          <span>Limpiar compilación</span>
        </div>
        <div class="summary-item">
          <code>colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug</code>
          <span>Compilar con símbolos de debug</span>
        </div>
        <div class="summary-item">
          <code>ros2 run pkg executable</code>
          <span>Ejecutar nodo compilado</span>
        </div>
      </div>

      <AlertBlock type="success" title="Flujo de Trabajo Típico" class="q-mt-lg">
        <CodeBlock
          lang="bash"
          content="# 1. Editar código C++
nano ~/ros2_ws/src/mi_pkg/src/nodo.cpp

# 2. Compilar
cd ~/ros2_ws
colcon build --packages-select mi_pkg

# 3. Cargar entorno
source install/setup.bash

# 4. Ejecutar
ros2 run mi_pkg nodo

# 5. Si hay errores, volver al paso 1"
          :copyable="true"
        />
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

/* COMPARISON GRID */
.comparison-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
  gap: 2rem;
  margin-top: 1.5rem;
}

.comparison-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.comparison-card.python {
  border-top: 4px solid #eab308;
}

.comparison-card.cpp {
  border-top: 4px solid #3b82f6;
}

.comparison-header {
  padding: 2rem;
  text-align: center;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.comparison-title {
  font-size: 1.5rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-top: 1rem;
}

.comparison-content {
  padding: 2rem;
}

.comparison-flow {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  margin-bottom: 2rem;
  padding: 1.5rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.flow-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  color: #cbd5e1;
  font-size: 0.85rem;
}

.flow-arrow {
  font-size: 1.5rem;
  color: #64748b;
}

.comparison-pros,
.comparison-cons {
  margin-bottom: 1rem;
  padding: 1rem;
  border-radius: 8px;
  font-size: 0.9rem;
}

.comparison-pros {
  background: rgba(34, 197, 94, 0.1);
  border-left: 3px solid #22c55e;
  color: #86efac;
}

.comparison-cons {
  background: rgba(239, 68, 68, 0.1);
  border-left: 3px solid #ef4444;
  color: #fca5a5;
}

.comparison-pros ul,
.comparison-cons ul {
  margin: 0.5rem 0 0 1.5rem;
  padding: 0;
}

/* PIPELINE VISUAL */
.pipeline-visual {
  display: flex;
  align-items: center;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.pipeline-step {
  flex: 1;
  text-align: center;
}

.step-number {
  width: 3rem;
  height: 3rem;
  background: linear-gradient(135deg, #8b5cf6, #7c3aed);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  margin: 0 auto 1rem;
}

.step-content {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 12px;
  padding: 1.5rem;
}

.step-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.step-icon {
  margin: 1rem 0;
}

.step-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.6;
  margin-bottom: 1rem;
}

.step-output,
.step-errors {
  margin-top: 1rem;
  padding: 0.75rem;
  border-radius: 6px;
  font-size: 0.85rem;
  text-align: left;
}

.step-output {
  background: rgba(59, 130, 246, 0.1);
  color: #93c5fd;
}

.step-errors {
  background: rgba(239, 68, 68, 0.1);
  color: #fca5a5;
}

.pipeline-arrow {
  color: #64748b;
}

/* CMAKE EXPLANATION */
.cmake-explanation {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.cmake-header {
  display: flex;
  align-items: center;
  gap: 12px;
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
}

.cmake-steps {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.cmake-step {
  display: flex;
  gap: 1rem;
  align-items: center;
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
}

.cmake-step-num {
  min-width: 2rem;
  height: 2rem;
  background: #a855f7;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.cmake-step-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

/* CMAKE BREAKDOWN */
.cmake-breakdown {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  margin-top: 1rem;
}

.breakdown-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
}

.breakdown-label {
  font-family: 'Fira Code', monospace;
  font-size: 1rem;
  font-weight: 700;
  color: #fbbf24;
  margin-bottom: 0.5rem;
}

.breakdown-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
}

/* BUILD WORKFLOW */
.build-workflow {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.workflow-step {
  display: flex;
  gap: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.workflow-num {
  min-width: 3rem;
  height: 3rem;
  background: linear-gradient(135deg, #3b82f6, #2563eb);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.workflow-content {
  flex-grow: 1;
}

.workflow-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.workflow-note {
  margin-top: 0.75rem;
  padding: 0.5rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  font-size: 0.85rem;
  color: #94a3b8;
}

/* ERROR CARDS */
.error-cards {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.error-card {
  background: rgba(15, 23, 42, 0.6);
  border-radius: 12px;
  overflow: hidden;
}

.error-card.compilation {
  border: 2px solid rgba(239, 68, 68, 0.5);
}

.error-card.linker {
  border: 2px solid rgba(249, 115, 22, 0.5);
}

.error-card.cmake {
  border: 2px solid rgba(168, 85, 247, 0.5);
}

.error-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  font-weight: 700;
  font-size: 1.05rem;
}

.error-card.compilation .error-header {
  background: rgba(127, 29, 29, 0.3);
  color: #fca5a5;
}

.error-card.linker .error-header {
  background: rgba(124, 45, 18, 0.3);
  color: #fdba74;
}

.error-card.cmake .error-header {
  background: rgba(88, 28, 135, 0.3);
  color: #e9d5ff;
}

.error-example,
.error-cause,
.error-solution {
  padding: 1rem 1.5rem;
}

.error-example {
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
}

.error-example code {
  font-family: 'Fira Code', monospace;
  color: #fca5a5;
  font-size: 0.9rem;
}

.error-cause,
.error-solution {
  color: #cbd5e1;
  font-size: 0.9rem;
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
  .pipeline-visual {
    flex-direction: column;
  }

  .pipeline-arrow {
    transform: rotate(90deg);
  }

  .comparison-grid {
    grid-template-columns: 1fr;
  }
}
</style>
