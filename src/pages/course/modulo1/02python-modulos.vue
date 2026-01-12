<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      En robótica, tu código crece rápido. Un solo archivo de 2000 líneas es un infierno de
      mantener. Los <strong>módulos y paquetes</strong> son la forma de organizar código en Python,
      dividiéndolo en archivos lógicos y reutilizables. <br /><br />
      Esta lección te enseña cómo estructurar proyectos ROS 2, entender el sistema de imports, y
      evitar el temido <code>ModuleNotFoundError</code>.
    </TextBlock>

    <AlertBlock type="info" title="Módulo vs Paquete">
      <strong>Módulo:</strong> Un solo archivo <code>.py</code> (ej: <code>motor.py</code>)
      <br />
      <strong>Paquete:</strong> Una carpeta con múltiples módulos + archivo
      <code>__init__.py</code>
      <br /><br />
      En ROS 2, cada "paquete" que creas es un paquete Python con estructura específica.
    </AlertBlock>

    <!-- LA MAGIA DEL IMPORT -->
    <div class="section-group">
      <SectionTitle>1. ¿Qué Pasa Cuando Haces import?</SectionTitle>
      <TextBlock>
        Cuando escribes <code>import math</code>, Python busca un archivo llamado
        <code>math.py</code> en una lista de directorios predefinidos. Si lo encuentra, lo lee y
        ejecuta su código.
      </TextBlock>

      <div class="import-flow q-mt-md">
        <div class="flow-step">
          <div class="step-icon">
            <q-icon name="edit" color="blue-4" size="xl" />
          </div>
          <div class="step-content">
            <div class="step-title">1. Escribes</div>
            <CodeBlock :hide-header="true" lang="python" content="import mi_robot" />
          </div>
        </div>

        <div class="flow-arrow">↓</div>

        <div class="flow-step">
          <div class="step-icon">
            <q-icon name="search" color="purple-4" size="xl" />
          </div>
          <div class="step-content">
            <div class="step-title">2. Python busca</div>
            <div class="step-desc">
              Busca <code>mi_robot.py</code> en cada directorio de <code>sys.path</code>
            </div>
          </div>
        </div>

        <div class="flow-arrow">↓</div>

        <div class="flow-step">
          <div class="step-icon">
            <q-icon name="play_arrow" color="green-4" size="xl" />
          </div>
          <div class="step-content">
            <div class="step-title">3. Ejecuta el archivo</div>
            <div class="step-desc">
              Lee y ejecuta todo el código del módulo (solo la primera vez)
            </div>
          </div>
        </div>

        <div class="flow-arrow">↓</div>

        <div class="flow-step">
          <div class="step-icon">
            <q-icon name="check_circle" color="yellow-6" size="xl" />
          </div>
          <div class="step-content">
            <div class="step-title">4. Crea namespace</div>
            <div class="step-desc">Ahora puedes usar <code>mi_robot.funcion()</code></div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="warning" title="Importante: Import Solo Ejecuta Una Vez">
          Si haces <code>import mi_robot</code> 10 veces en diferentes archivos, Python solo ejecuta
          el código de <code>mi_robot.py</code> la primera vez. Después, reutiliza el módulo ya
          cargado en memoria.
        </AlertBlock>
      </div>
    </div>

    <!-- ESTILOS DE IMPORT -->
    <div class="section-group">
      <SectionTitle>2. Tres Estilos de Import</SectionTitle>

      <div class="import-styles">
        <div class="style-card correct">
          <div class="style-header">
            <q-icon name="check_circle" color="green-4" size="lg" />
            <span>✅ Estilo 1: Namespace Completo (Recomendado)</span>
          </div>
          <div class="style-content">
            <CodeBlock
              lang="python"
              content="import numpy as np
import rclpy

# Uso
array = np.array([1, 2, 3])
rclpy.init()"
              :copyable="true"
            />
            <div class="style-pros">
              <strong>Ventajas:</strong>
              <ul>
                <li>Código legible: sabes de dónde viene cada función</li>
                <li>Sin conflictos de nombres</li>
                <li>Estándar en proyectos grandes</li>
              </ul>
            </div>
          </div>
        </div>

        <div class="style-card warning">
          <div class="style-header">
            <q-icon name="warning" color="yellow-6" size="lg" />
            <span>⚠️ Estilo 2: Import Específico (Usar con Cuidado)</span>
          </div>
          <div class="style-content">
            <CodeBlock
              lang="python"
              content="from math import sqrt, pi
from rclpy.node import Node

# Uso directo
raiz = sqrt(16)
nodo = Node('mi_nodo')"
              :copyable="true"
            />
            <div class="style-pros">
              <strong>Ventajas:</strong> Código más corto
              <br />
              <strong>Desventajas:</strong> Riesgo de conflictos si ya tienes una función
              <code>sqrt</code>
            </div>
          </div>
        </div>

        <div class="style-card danger">
          <div class="style-header">
            <q-icon name="cancel" color="red-4" size="lg" />
            <span>❌ Estilo 3: Wildcard (NUNCA Usar)</span>
          </div>
          <div class="style-content">
            <CodeBlock
              lang="python"
              content="from rclpy.node import *

# ¿De dónde salió create_publisher?
# ¿Qué más importé sin darme cuenta?"
            />
            <div class="style-cons">
              <strong>Peligros:</strong>
              <ul>
                <li>Importas cientos de nombres invisibles</li>
                <li>Conflictos impredecibles</li>
                <li>Debugging imposible</li>
                <li><strong>Prohibido en código profesional</strong></li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- __INIT__.PY -->
    <div class="section-group">
      <SectionTitle>3. El Archivo Mágico: __init__.py</SectionTitle>
      <TextBlock>
        Para que Python reconozca una carpeta como un <strong>paquete importable</strong>, debe
        contener un archivo llamado <code>__init__.py</code>. Puede estar vacío, pero debe existir.
      </TextBlock>

      <div class="init-demo q-mt-md">
        <div class="init-before">
          <div class="demo-label">❌ Sin __init__.py</div>
          <div class="folder-structure">
            <div class="folder-item">
              <q-icon name="folder" color="grey-5" />
              <span>mi_robot/</span>
            </div>
            <div class="folder-item indent">
              <q-icon name="description" color="grey-5" />
              <span>motor.py</span>
            </div>
            <div class="folder-item indent">
              <q-icon name="description" color="grey-5" />
              <span>sensor.py</span>
            </div>
          </div>
          <div class="demo-result error">
            <CodeBlock
              :hide-header="true"
              lang="python"
              content="import mi_robot.motor
# ModuleNotFoundError!"
            />
          </div>
        </div>

        <div class="init-arrow">
          <q-icon name="arrow_forward" size="3rem" color="yellow-6" />
          <div>Agregar __init__.py</div>
        </div>

        <div class="init-after">
          <div class="demo-label">✅ Con __init__.py</div>
          <div class="folder-structure">
            <div class="folder-item">
              <q-icon name="folder" color="blue-4" />
              <span>mi_robot/</span>
            </div>
            <div class="folder-item indent highlight">
              <q-icon name="description" color="yellow-6" />
              <span>__init__.py</span>
            </div>
            <div class="folder-item indent">
              <q-icon name="description" color="green-4" />
              <span>motor.py</span>
            </div>
            <div class="folder-item indent">
              <q-icon name="description" color="green-4" />
              <span>sensor.py</span>
            </div>
          </div>
          <div class="demo-result success">
            <CodeBlock
              :hide-header="true"
              lang="python"
              content="import mi_robot.motor
# ✅ Funciona!"
            />
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>¿Qué Poner en __init__.py?</SectionTitle>
        <div class="row q-col-gutter-md q-mt-sm">
          <div class="col-12 col-md-6">
            <div class="init-option">
              <div class="option-title">Opción 1: Vacío (Mínimo)</div>
              <CodeBlock
                lang="python"
                content="# __init__.py
# (archivo vacío, solo marca la carpeta como paquete)"
                :copyable="true"
              />
            </div>
          </div>
          <div class="col-12 col-md-6">
            <div class="init-option">
              <div class="option-title">Opción 2: Re-exportar (Avanzado)</div>
              <CodeBlock
                lang="python"
                content="# __init__.py
from .motor import MotorController
from .sensor import LidarSensor

__all__ = ['MotorController', 'LidarSensor']"
                :copyable="true"
              />
              <div class="option-note">
                Permite hacer <code>from mi_robot import MotorController</code> directamente
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- PYTHONPATH -->
    <div class="section-group">
      <SectionTitle>4. El Mapa del Tesoro: sys.path y PYTHONPATH</SectionTitle>
      <TextBlock>
        Cuando haces <code>import mi_modulo</code>, Python busca en una lista de directorios llamada
        <code>sys.path</code>. Si tu módulo no está en ninguno de esos directorios, obtendrás
        <code>ModuleNotFoundError</code>.
      </TextBlock>

      <div class="path-visualization q-mt-md">
        <div class="path-header">
          <q-icon name="map" color="blue-4" size="md" />
          <span>¿Dónde busca Python? (en orden)</span>
        </div>
        <div class="path-list">
          <div class="path-item">
            <div class="path-num">1</div>
            <div class="path-desc">
              <strong>Directorio del script actual</strong>
              <br />
              <span class="path-example">Donde está tu archivo .py</span>
            </div>
          </div>
          <div class="path-item">
            <div class="path-num">2</div>
            <div class="path-desc">
              <strong>PYTHONPATH</strong>
              <br />
              <span class="path-example">Variable de entorno personalizada</span>
            </div>
          </div>
          <div class="path-item">
            <div class="path-num">3</div>
            <div class="path-desc">
              <strong>Librerías estándar</strong>
              <br />
              <span class="path-example">/usr/lib/python3.10</span>
            </div>
          </div>
          <div class="path-item">
            <div class="path-num">4</div>
            <div class="path-desc">
              <strong>site-packages</strong>
              <br />
              <span class="path-example">Paquetes instalados con pip</span>
            </div>
          </div>
          <div class="path-item highlight">
            <div class="path-num">5</div>
            <div class="path-desc">
              <strong>ROS 2 packages</strong>
              <br />
              <span class="path-example">/opt/ros/humble/lib/python3.10/site-packages</span>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="Ver tu sys.path actual"
          lang="python"
          content="import sys
import pprint

pprint.pprint(sys.path)"
          :copyable="true"
        />
      </div>

      <div class="q-mt-md">
        <AlertBlock type="success" title="Cómo ROS 2 Agrega sus Paquetes">
          Cuando haces <code>source /opt/ros/humble/setup.bash</code>, ROS modifica la variable
          <code>PYTHONPATH</code> para agregar sus directorios. Por eso puedes hacer
          <code>import rclpy</code> después de hacer source.
        </AlertBlock>
      </div>
    </div>

    <!-- ESTRUCTURA DE PROYECTO -->
    <div class="section-group">
      <SectionTitle>5. Estructura de Proyecto ROS 2 (Python)</SectionTitle>
      <TextBlock>
        Esta es la estructura estándar de un paquete ROS 2 en Python. Cada elemento tiene un
        propósito específico:
      </TextBlock>

      <div class="project-structure q-mt-md">
        <div class="structure-tree">
          <div class="tree-item folder">
            <q-icon name="folder" color="blue-4" />
            <span>mi_robot_pkg/</span>
            <div class="item-desc">Raíz del paquete</div>
          </div>

          <div class="tree-children">
            <div class="tree-item file important">
              <q-icon name="description" color="orange-4" />
              <span>package.xml</span>
              <div class="item-desc">Metadatos del paquete (nombre, dependencias)</div>
            </div>

            <div class="tree-item file important">
              <q-icon name="description" color="orange-4" />
              <span>setup.py</span>
              <div class="item-desc">Instalador Python (entry points, scripts)</div>
            </div>

            <div class="tree-item folder">
              <q-icon name="folder" color="green-4" />
              <span>mi_robot_pkg/</span>
              <div class="item-desc">Código fuente (mismo nombre que raíz)</div>
            </div>

            <div class="tree-children">
              <div class="tree-item file critical">
                <q-icon name="description" color="yellow-6" />
                <span>__init__.py</span>
                <div class="item-desc">🔥 CRUCIAL: Marca como paquete Python</div>
              </div>

              <div class="tree-item file">
                <q-icon name="description" color="green-4" />
                <span>control_node.py</span>
                <div class="item-desc">Nodo de control</div>
              </div>

              <div class="tree-item file">
                <q-icon name="description" color="green-4" />
                <span>vision_node.py</span>
                <div class="item-desc">Nodo de visión</div>
              </div>
            </div>

            <div class="tree-item folder">
              <q-icon name="folder" color="purple-4" />
              <span>launch/</span>
              <div class="item-desc">Archivos de lanzamiento</div>
            </div>

            <div class="tree-item folder">
              <q-icon name="folder" color="cyan-4" />
              <span>config/</span>
              <div class="item-desc">Archivos YAML de configuración</div>
            </div>

            <div class="tree-item folder">
              <q-icon name="folder" color="grey-5" />
              <span>test/</span>
              <div class="item-desc">Tests unitarios</div>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="info" title="Nota Importante">
          La carpeta de código fuente (segunda <code>mi_robot_pkg/</code>) DEBE tener el mismo
          nombre que la raíz del paquete. Esto es un requisito de ROS 2.
        </AlertBlock>
      </div>
    </div>

    <!-- IMPORTS RELATIVOS -->
    <div class="section-group">
      <SectionTitle>6. Imports Relativos vs Absolutos</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-6">
          <div class="import-type-card">
            <div class="import-type-header absolute">
              <q-icon name="public" size="md" />
              <span>Import Absoluto</span>
            </div>
            <div class="import-type-content">
              <CodeBlock
                lang="python"
                content="# En mi_robot_pkg/vision_node.py
from mi_robot_pkg.utils import calcular_distancia

# Ruta completa desde la raíz"
                :copyable="true"
              />
              <div class="import-type-pros">
                <strong>Ventajas:</strong> Claro, explícito, funciona desde cualquier lugar
              </div>
            </div>
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="import-type-card">
            <div class="import-type-header relative">
              <q-icon name="share" size="md" />
              <span>Import Relativo</span>
            </div>
            <div class="import-type-content">
              <CodeBlock
                lang="python"
                content="# En mi_robot_pkg/vision_node.py
from .utils import calcular_distancia

# El punto (.) significa 'mismo paquete'"
                :copyable="true"
              />
              <div class="import-type-pros">
                <strong>Ventajas:</strong> Más corto, fácil de refactorizar
                <br />
                <strong>Desventajas:</strong> Solo funciona dentro del paquete
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-md">
        <AlertBlock type="warning" title="Regla de Oro">
          En ROS 2, usa <strong>imports absolutos</strong> para mayor claridad. Los imports
          relativos pueden causar problemas con los entry points de <code>setup.py</code>.
        </AlertBlock>
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <q-expansion-item
        icon="error"
        label="ModuleNotFoundError: No module named 'mi_paquete'"
        header-class="error-header"
      >
        <div class="error-content">
          <strong>Causas posibles:</strong>
          <ol>
            <li>No hiciste <code>source install/setup.bash</code> después de compilar</li>
            <li>Falta el archivo <code>__init__.py</code> en la carpeta del paquete</li>
            <li>El paquete no está instalado (no hiciste <code>colcon build</code>)</li>
          </ol>
          <strong>Solución:</strong>
          <CodeBlock
            lang="bash"
            content="cd ~/ros2_ws
colcon build --packages-select mi_paquete
source install/setup.bash"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="ImportError: attempted relative import with no known parent package"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> Intentaste ejecutar un archivo con imports relativos directamente.
          <br /><br />
          <strong>Solución:</strong> Los imports relativos solo funcionan cuando el módulo es
          importado, no ejecutado directamente. Usa imports absolutos o ejecuta como módulo:
          <CodeBlock
            lang="bash"
            content="# En lugar de:
python3 mi_paquete/nodo.py

# Usa:
python3 -m mi_paquete.nodo"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="AttributeError: module 'mi_modulo' has no attribute 'funcion'"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> El módulo existe pero no tiene la función/clase que buscas.
          <br /><br />
          <strong>Debugging:</strong>
          <CodeBlock
            lang="python"
            content="import mi_modulo
print(dir(mi_modulo))  # Ver todo lo que contiene"
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
            title="Módulos y Paquetes en Python"
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
      <SectionTitle>📝 Resumen de Conceptos Clave</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>import modulo</code>
          <span>Import con namespace completo</span>
        </div>
        <div class="summary-item">
          <code>from modulo import func</code>
          <span>Import específico</span>
        </div>
        <div class="summary-item">
          <code>__init__.py</code>
          <span>Marca carpeta como paquete Python</span>
        </div>
        <div class="summary-item">
          <code>sys.path</code>
          <span>Lista de directorios donde Python busca</span>
        </div>
        <div class="summary-item">
          <code>from . import</code>
          <span>Import relativo (mismo paquete)</span>
        </div>
        <div class="summary-item">
          <code>source setup.bash</code>
          <span>Agrega paquetes ROS 2 a PYTHONPATH</span>
        </div>
      </div>

      <AlertBlock type="success" title="Checklist de Paquete ROS 2" class="q-mt-lg">
        ✅ Carpeta raíz con nombre del paquete
        <br />
        ✅ <code>package.xml</code> con metadatos
        <br />
        ✅ <code>setup.py</code> con entry points
        <br />
        ✅ Carpeta de código fuente (mismo nombre)
        <br />
        ✅ <code>__init__.py</code> en carpeta de código
        <br />
        ✅ Imports absolutos (no relativos)
        <br />
        ✅ <code>colcon build</code> + <code>source install/setup.bash</code>
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

/* IMPORT FLOW */
.import-flow {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.flow-step {
  display: flex;
  gap: 1.5rem;
  align-items: center;
  background: rgba(0, 0, 0, 0.3);
  padding: 1.5rem;
  border-radius: 12px;
}

.step-icon {
  flex-shrink: 0;
}

.step-content {
  flex-grow: 1;
}

.step-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.5rem;
}

.step-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.flow-arrow {
  text-align: center;
  font-size: 2rem;
  color: #3b82f6;
  font-weight: 700;
}

/* IMPORT STYLES */
.import-styles {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  margin-top: 1.5rem;
}

.style-card {
  border-radius: 16px;
  overflow: hidden;
}

.style-card.correct {
  border: 2px solid rgba(34, 197, 94, 0.5);
}

.style-card.warning {
  border: 2px solid rgba(234, 179, 8, 0.5);
}

.style-card.danger {
  border: 2px solid rgba(239, 68, 68, 0.5);
}

.style-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  font-weight: 700;
  font-size: 1.05rem;
}

.style-card.correct .style-header {
  background: rgba(20, 83, 45, 0.3);
  color: #86efac;
}

.style-card.warning .style-header {
  background: rgba(113, 63, 18, 0.3);
  color: #fde047;
}

.style-card.danger .style-header {
  background: rgba(127, 29, 29, 0.3);
  color: #fca5a5;
}

.style-content {
  padding: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
}

.style-pros,
.style-cons {
  margin-top: 1rem;
  padding: 0.75rem;
  border-radius: 8px;
  font-size: 0.9rem;
}

.style-pros {
  background: rgba(34, 197, 94, 0.1);
  color: #86efac;
}

.style-cons {
  background: rgba(239, 68, 68, 0.1);
  color: #fca5a5;
}

.style-pros ul,
.style-cons ul {
  margin: 0.5rem 0 0 1.5rem;
  padding: 0;
}

/* INIT DEMO */
.init-demo {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 2rem;
  align-items: center;
  padding: 2rem;
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
}

.init-before,
.init-after {
  text-align: center;
}

.demo-label {
  font-size: 1.1rem;
  font-weight: 700;
  margin-bottom: 1.5rem;
}

.folder-structure {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
  padding: 1.5rem;
  text-align: left;
  margin-bottom: 1rem;
}

.folder-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem;
  color: #cbd5e1;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.folder-item.indent {
  padding-left: 2rem;
}

.folder-item.highlight {
  background: rgba(234, 179, 8, 0.1);
  border-radius: 4px;
  border-left: 3px solid #fbbf24;
}

.demo-result {
  padding: 1rem;
  border-radius: 8px;
}

.demo-result.error {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
}

.demo-result.success {
  background: rgba(34, 197, 94, 0.1);
  border: 1px solid rgba(34, 197, 94, 0.3);
}

.init-arrow {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  color: #fbbf24;
  font-size: 0.85rem;
  font-weight: 700;
}

/* INIT OPTIONS */
.init-option {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  height: 100%;
}

.option-title {
  font-size: 1.05rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.option-note {
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 6px;
  font-size: 0.85rem;
  color: #94a3b8;
}

/* PATH VISUALIZATION */
.path-visualization {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.path-header {
  display: flex;
  align-items: center;
  gap: 12px;
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
}

.path-list {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.path-item {
  display: flex;
  gap: 1rem;
  align-items: flex-start;
  background: rgba(0, 0, 0, 0.3);
  padding: 1rem;
  border-radius: 8px;
}

.path-item.highlight {
  background: rgba(59, 130, 246, 0.1);
  border: 1px solid rgba(59, 130, 246, 0.3);
}

.path-num {
  min-width: 2rem;
  height: 2rem;
  background: #3b82f6;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.path-desc {
  color: #e2e8f0;
  font-size: 0.95rem;
}

.path-example {
  display: block;
  margin-top: 0.25rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: #94a3b8;
}

/* PROJECT STRUCTURE */
.project-structure {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.structure-tree {
  font-family: 'Fira Code', monospace;
}

.tree-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.75rem;
  margin-bottom: 0.5rem;
  border-radius: 6px;
  transition: background 0.2s;
}

.tree-item:hover {
  background: rgba(255, 255, 255, 0.05);
}

.tree-item.file.important {
  background: rgba(251, 146, 60, 0.1);
  border-left: 3px solid #fb923c;
}

.tree-item.file.critical {
  background: rgba(234, 179, 8, 0.1);
  border-left: 3px solid #fbbf24;
}

.tree-item span {
  color: #f1f5f9;
  font-weight: 500;
}

.item-desc {
  margin-left: auto;
  font-size: 0.75rem;
  color: #94a3b8;
  font-family: sans-serif;
}

.tree-children {
  margin-left: 2rem;
  border-left: 1px solid rgba(148, 163, 184, 0.2);
  padding-left: 1rem;
}

/* IMPORT TYPE CARDS */
.import-type-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  height: 100%;
}

.import-type-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  font-weight: 700;
  color: #f1f5f9;
}

.import-type-header.absolute {
  background: rgba(59, 130, 246, 0.1);
  border-bottom: 2px solid #3b82f6;
}

.import-type-header.relative {
  background: rgba(168, 85, 247, 0.1);
  border-bottom: 2px solid #a855f7;
}

.import-type-content {
  padding: 1.5rem;
}

.import-type-pros {
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  font-size: 0.85rem;
  color: #cbd5e1;
}

/* ERROR EXPANSION */
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

.error-content ol {
  margin: 0.5rem 0 1rem 1.5rem;
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
  .init-demo {
    grid-template-columns: 1fr;
  }

  .init-arrow {
    transform: rotate(90deg);
  }

  .flow-arrow {
    transform: rotate(90deg);
  }
}
</style>
